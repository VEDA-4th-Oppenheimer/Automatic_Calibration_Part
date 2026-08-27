#include "auto_calib/hybrid_orientation_analyzer.hpp"

#include "auto_calib/image_vanishing_estimator.hpp"
#include "auto_calib/lidar_manhattan_estimator.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <nlohmann/json.hpp>
#include <numeric>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace auto_calib {
namespace {
constexpr double kPi = 3.14159265358979323846;

double radians(double degrees) { return degrees * kPi / 180.0; }
double degrees(double value) { return value * 180.0 / kPi; }

double normalizeYaw(double yaw) {
  while (yaw >= 180.0)
    yaw -= 360.0;
  while (yaw < -180.0)
    yaw += 360.0;
  return yaw;
}

double circularDistance(double a, double b) {
  return std::abs(normalizeYaw(a - b));
}

std::vector<double> smooth(const std::vector<double> &values, int radius,
                           bool circular) {
  std::vector<double> out(values.size(), 0.0);
  if (values.empty())
    return out;
  for (std::size_t i = 0; i < values.size(); ++i) {
    double sum = 0.0;
    int count = 0;
    for (int d = -radius; d <= radius; ++d) {
      int index = static_cast<int>(i) + d;
      if (circular) {
        index %= static_cast<int>(values.size());
        if (index < 0)
          index += static_cast<int>(values.size());
      } else if (index < 0 || index >= static_cast<int>(values.size())) {
        continue;
      }
      sum += values[static_cast<std::size_t>(index)];
      ++count;
    }
    out[i] = count ? sum / count : 0.0;
  }
  return out;
}

double circularSample(const std::vector<double> &values, double index) {
  if (values.empty())
    return 0.0;
  const int period = values.size() > 1 ? static_cast<int>(values.size()) - 1
                                       : static_cast<int>(values.size());
  if (period <= 0)
    return values.front();
  index = std::fmod(index, static_cast<double>(period));
  if (index < 0)
    index += period;
  const int lo = static_cast<int>(std::floor(index));
  const int hi = (lo + 1) % period;
  const double alpha = index - lo;
  return values[static_cast<std::size_t>(lo)] * (1.0 - alpha) +
         values[static_cast<std::size_t>(hi)] * alpha;
}

std::vector<double> cameraSignature(const cv::Mat &bgr, int bins,
                                    int *edge_pixels) {
  cv::Mat small = bgr;
  if (bgr.cols > 640) {
    const double scale = 640.0 / bgr.cols;
    cv::resize(bgr, small, cv::Size(), scale, scale, cv::INTER_AREA);
  }
  cv::Mat gray;
  if (small.channels() == 1)
    gray = small;
  else
    cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.8);
  cv::Mat gx, gy, abs_x, abs_y, direction_mask, strength_mask, edges;
  cv::Sobel(gray, gx, CV_32F, 1, 0, 3);
  cv::Sobel(gray, gy, CV_32F, 0, 1, 3);
  cv::absdiff(gx, cv::Scalar(0), abs_x);
  cv::absdiff(gy, cv::Scalar(0), abs_y);
  // Only vertical image boundaries contribute to the azimuth signature.
  // Horizontal texture used to dominate the old all-Canny column sum.
  cv::compare(abs_x, abs_y * 1.25, direction_mask, cv::CMP_GT);
  cv::compare(abs_x, 32.0, strength_mask, cv::CMP_GT);
  cv::bitwise_and(direction_mask, strength_mask, edges);
  if (edge_pixels)
    *edge_pixels = cv::countNonZero(edges);
  std::vector<double> signature(static_cast<std::size_t>(bins), 0.0);
  for (int x = 0; x < edges.cols; ++x) {
    const int bin = std::min(bins - 1, x * bins / edges.cols);
    signature[static_cast<std::size_t>(bin)] += cv::countNonZero(edges.col(x));
  }
  const double column_scale =
      static_cast<double>(bins) / (edges.rows * edges.cols);
  for (double &value : signature)
    value *= column_scale;
  return smooth(signature, 1, false);
}

std::vector<double> cameraElevationSignature(const cv::Mat &bgr, int bins) {
  cv::Mat small = bgr;
  if (bgr.cols > 640) {
    const double scale = 640.0 / bgr.cols;
    cv::resize(bgr, small, cv::Size(), scale, scale, cv::INTER_AREA);
  }
  cv::Mat gray;
  if (small.channels() == 1)
    gray = small;
  else
    cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.8);
  cv::Mat gx, gy, abs_x, abs_y, direction_mask, strength_mask, edges;
  cv::Sobel(gray, gx, CV_32F, 1, 0, 3);
  cv::Sobel(gray, gy, CV_32F, 0, 1, 3);
  cv::absdiff(gx, cv::Scalar(0), abs_x);
  cv::absdiff(gy, cv::Scalar(0), abs_y);
  cv::compare(abs_y, abs_x * 1.25, direction_mask, cv::CMP_GT);
  cv::compare(abs_y, 32.0, strength_mask, cv::CMP_GT);
  cv::bitwise_and(direction_mask, strength_mask, edges);
  std::vector<double> signature(static_cast<std::size_t>(bins), 0.0);
  for (int y = 0; y < edges.rows; ++y) {
    const int bin = std::min(bins - 1, y * bins / edges.rows);
    signature[static_cast<std::size_t>(bin)] += cv::countNonZero(edges.row(y));
  }
  const double scale = static_cast<double>(bins) / (edges.rows * edges.cols);
  for (double &value : signature)
    value *= scale;
  return smooth(signature, 1, false);
}

std::vector<double> lidarElevationSignature(const PanoramaRaster &raster,
                                            double yaw_deg,
                                            const Eigen::Matrix3d &camera_k,
                                            int image_width) {
  const cv::Mat edge = combinedPanoramaEdge(raster);
  std::vector<double> signature(static_cast<std::size_t>(raster.rows), 0.0);
  std::vector<int> valid(static_cast<std::size_t>(raster.rows), 0);
  const double center_pan = -radians(yaw_deg);
  const double left = std::atan((0.0 - camera_k(0, 2)) / camera_k(0, 0));
  const double right =
      std::atan((image_width - camera_k(0, 2)) / camera_k(0, 0));
  const double span = raster.pan_max_rad - raster.pan_min_rad;
  for (int c = 0; c < raster.columns; ++c) {
    const double pan = raster.pan_min_rad +
                       span * static_cast<double>(c) /
                           std::max(1, raster.columns - 1);
    const double delta = std::atan2(std::sin(pan - center_pan),
                                    std::cos(pan - center_pan));
    if (delta < left || delta > right)
      continue;
    for (int r = 0; r < raster.rows; ++r) {
      if (!raster.valid.at<unsigned char>(r, c))
        continue;
      ++valid[static_cast<std::size_t>(r)];
      signature[static_cast<std::size_t>(r)] +=
          edge.at<unsigned char>(r, c) != 0;
    }
  }
  for (int r = 0; r < raster.rows; ++r)
    if (valid[static_cast<std::size_t>(r)] > 0)
      signature[static_cast<std::size_t>(r)] /=
          valid[static_cast<std::size_t>(r)];
  return smooth(signature, 1, false);
}

std::vector<double> scoreElevationSignatures(
    const std::vector<double> &camera_signature,
    const std::vector<double> &lidar_signature,
    const Eigen::Matrix3d &camera_k, int image_height, double tilt_min_rad,
    double tilt_max_rad, const std::vector<double> &down_degrees) {
  std::vector<double> scores;
  scores.reserve(down_degrees.size());
  const double tilt_span = tilt_max_rad - tilt_min_rad;
  for (double down_deg : down_degrees) {
    std::vector<double> camera_values;
    std::vector<double> lidar_values;
    for (std::size_t i = 0; i < camera_signature.size(); ++i) {
      const double v =
          (static_cast<double>(i) + 0.5) / camera_signature.size() * image_height;
      const double ray = std::atan((v - camera_k(1, 2)) / camera_k(1, 1));
      const double tilt = -radians(down_deg) - ray;
      const double row = (tilt_max_rad - tilt) / tilt_span *
                         (lidar_signature.size() - 1);
      if (row < 0.0 || row > lidar_signature.size() - 1)
        continue;
      const int lo = static_cast<int>(std::floor(row));
      const int hi = std::min(lo + 1, static_cast<int>(lidar_signature.size()) - 1);
      const double alpha = row - lo;
      camera_values.push_back(camera_signature[i]);
      lidar_values.push_back(lidar_signature[static_cast<std::size_t>(lo)] *
                                 (1.0 - alpha) +
                             lidar_signature[static_cast<std::size_t>(hi)] * alpha);
    }
    if (camera_values.size() < camera_signature.size() / 2) {
      scores.push_back(0.0);
      continue;
    }
    const double cm = std::accumulate(camera_values.begin(), camera_values.end(), 0.0) /
                      camera_values.size();
    const double lm = std::accumulate(lidar_values.begin(), lidar_values.end(), 0.0) /
                      lidar_values.size();
    double covariance = 0.0, camera_square = 0.0, lidar_square = 0.0;
    for (std::size_t i = 0; i < camera_values.size(); ++i) {
      const double dc = camera_values[i] - cm;
      const double dl = lidar_values[i] - lm;
      covariance += dc * dl;
      camera_square += dc * dc;
      lidar_square += dl * dl;
    }
    const double denominator = std::sqrt(camera_square * lidar_square);
    const double correlation = denominator > 1e-12 ? covariance / denominator : -1.0;
    scores.push_back(std::clamp((correlation + 1.0) * 0.5, 0.0, 1.0));
  }
  return scores;
}

std::vector<double> lidarSignature(const PanoramaRaster &raster) {
  const cv::Mat edge = combinedPanoramaEdge(raster);
  std::vector<double> signature(static_cast<std::size_t>(raster.columns), 0.0);
  for (int c = 0; c < raster.columns; ++c) {
    int valid = 0;
    int hits = 0;
    for (int r = 0; r < raster.rows; ++r) {
      if (raster.valid.at<unsigned char>(r, c)) {
        ++valid;
        hits += edge.at<unsigned char>(r, c) != 0;
      }
    }
    signature[static_cast<std::size_t>(c)] =
        valid ? static_cast<double>(hits) / valid : 0.0;
  }
  return smooth(signature, 2, true);
}

std::vector<Point> organizedPoints(const nlohmann::json &root, int rows,
                                   int columns) {
  std::vector<Point> points(static_cast<std::size_t>(rows) * columns);
  const double range_offset =
      root.contains("sensor") && root.at("sensor").contains("range_offset_m")
          ? root.at("sensor").at("range_offset_m").get<double>()
          : 0.0;
  for (const auto &m : root.at("measurements")) {
    const int row = m.at("row").get<int>();
    const int column = m.at("column").get<int>();
    Point &point = points[static_cast<std::size_t>(row) * columns + column];
    point.row = static_cast<std::size_t>(row);
    point.column = static_cast<std::size_t>(column);
    if (!m.value("valid", false) || !m.contains("distance_m") ||
        !m.at("distance_m").is_number() || !m.contains("pan_rad") ||
        !m.contains("tilt_rad"))
      continue;
    const double range = m.at("distance_m").get<double>() + range_offset;
    const double pan = m.at("pan_rad").get<double>();
    const double tilt = m.at("tilt_rad").get<double>();
    point.range = static_cast<float>(range);
    point.xyz = Eigen::Vector3f(range * std::cos(tilt) * std::sin(pan),
                                -range * std::sin(tilt),
                                range * std::cos(tilt) * std::cos(pan));
    point.flags = kValidRange;
  }
  return points;
}

Eigen::Matrix3d rotationFromCorrespondences(const Eigen::Vector3d *source,
                                            const Eigen::Vector3d *target,
                                            int count) {
  Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
  for (int i = 0; i < count; ++i)
    h += source[i] * target[i].transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
  d(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() >= 0
                ? 1.0
                : -1.0;
  return svd.matrixV() * d * svd.matrixU().transpose();
}

void decompose(const Eigen::Matrix3d &rotation, double *roll_deg,
               double *down_deg, double *yaw_deg) {
  const double down = std::asin(std::clamp(rotation(2, 1), -1.0, 1.0));
  const double yaw = std::atan2(-rotation(2, 0), rotation(2, 2));
  const Eigen::Matrix3d no_roll =
      Eigen::AngleAxisd(down, Eigen::Vector3d::UnitX()).toRotationMatrix() *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d roll_only = rotation * no_roll.transpose();
  *roll_deg = degrees(std::atan2(roll_only(1, 0), roll_only(0, 0)));
  *down_deg = degrees(down);
  *yaw_deg = normalizeYaw(degrees(yaw));
}

struct StructuralHypothesis {
  double yaw = 0.0;
  double down = 0.0;
  double roll = 0.0;
};

std::vector<StructuralHypothesis>
structuralHypotheses(const std::vector<VanishingDirection> &vanishing,
                     const LidarManhattanFrame &manhattan) {
  std::vector<StructuralHypothesis> out;
  const std::size_t count = std::min<std::size_t>(vanishing.size(), 3);
  if (count < 2 || !manhattan.valid)
    return out;
  const Eigen::Vector3d lidar_axes[3] = {manhattan.v1, manhattan.v2,
                                         manhattan.v3};
  auto emit = [&](const std::array<int, 3> &assignment, int sign_bits) {
    Eigen::Vector3d source[3], target[3];
    for (int i = 0; i < static_cast<int>(count); ++i) {
      source[i] = lidar_axes[assignment[static_cast<std::size_t>(i)]];
      const double sign = ((sign_bits >> i) & 1) ? -1.0 : 1.0;
      target[i] = vanishing[static_cast<std::size_t>(i)].direction * sign;
    }
    double roll = 0.0, down = 0.0, yaw = 0.0;
    decompose(
        rotationFromCorrespondences(source, target, static_cast<int>(count)),
        &roll, &down, &yaw);
    if (down >= 0.0 && down <= 75.0 && std::abs(roll) <= 30.0)
      out.push_back({yaw, down, roll});
  };
  if (count == 3) {
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b) {
        if (a == b)
          continue;
        for (int c = 0; c < 3; ++c) {
          if (c == a || c == b)
            continue;
          for (int signs = 0; signs < 8; ++signs)
            emit({a, b, c}, signs);
        }
      }
  } else {
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b) {
        if (a == b)
          continue;
        for (int signs = 0; signs < 4; ++signs)
          emit({a, b, 0}, signs);
      }
  }
  return out;
}

double standardDeviation(const std::vector<double> &values, double mean) {
  if (values.empty())
    return 0.0;
  double sum = 0.0;
  for (double value : values) {
    const double delta = value - mean;
    sum += delta * delta;
  }
  return std::sqrt(sum / values.size());
}
} // namespace

std::vector<double>
scoreAngularSignatures(const std::vector<double> &camera_signature,
                       const std::vector<double> &lidar_signature,
                       const Eigen::Matrix3d &camera_k, int image_width,
                       double pan_min_rad, double pan_max_rad,
                       const std::vector<double> &yaw_degrees) {
  std::vector<double> scores;
  scores.reserve(yaw_degrees.size());
  if (camera_signature.size() < 4 || lidar_signature.size() < 4 ||
      image_width <= 0 || camera_k(0, 0) <= 0 || pan_max_rad <= pan_min_rad)
    return std::vector<double>(yaw_degrees.size(), 0.0);
  const double camera_mean =
      std::accumulate(camera_signature.begin(), camera_signature.end(), 0.0) /
      camera_signature.size();
  const double camera_std = standardDeviation(camera_signature, camera_mean);
  for (double yaw_deg : yaw_degrees) {
    std::vector<double> sampled(camera_signature.size(), 0.0);
    for (std::size_t i = 0; i < sampled.size(); ++i) {
      const double u =
          (static_cast<double>(i) + 0.5) / sampled.size() * image_width;
      const double ray_angle = std::atan((u - camera_k(0, 2)) / camera_k(0, 0));
      double pan = ray_angle - radians(yaw_deg);
      const double span = pan_max_rad - pan_min_rad;
      while (pan < pan_min_rad)
        pan += span;
      while (pan >= pan_max_rad)
        pan -= span;
      const double column =
          (pan - pan_min_rad) / span * (lidar_signature.size() - 1);
      sampled[i] = circularSample(lidar_signature, column);
    }
    const double lidar_mean =
        std::accumulate(sampled.begin(), sampled.end(), 0.0) / sampled.size();
    const double lidar_std = standardDeviation(sampled, lidar_mean);
    if (camera_std < 1e-9 || lidar_std < 1e-9) {
      scores.push_back(0.0);
      continue;
    }
    double correlation = 0.0;
    for (std::size_t i = 0; i < sampled.size(); ++i)
      correlation +=
          (camera_signature[i] - camera_mean) * (sampled[i] - lidar_mean);
    correlation /= camera_signature.size() * camera_std * lidar_std;
    scores.push_back(std::clamp((correlation + 1.0) * 0.5, 0.0, 1.0));
  }
  return scores;
}

HybridAnalyzerResult
analyzeHybridOrientation(const std::filesystem::path &json_path,
                         const cv::Mat &camera_bgr, const CameraModel &camera,
                         const HybridAnalyzerOptions &options) {
  const auto total_start = std::chrono::steady_clock::now();
  HybridAnalyzerResult out;
  if (camera_bgr.empty() || options.top_k <= 0 || options.signature_bins < 8 ||
      options.yaw_step_deg <= 0.0 || !camera.k.allFinite()) {
    out.fallback_reason = "INVALID_INPUT";
    return out;
  }
  try {
    std::ifstream stream(json_path);
    if (!stream)
      throw std::runtime_error("cannot open scan JSON");
    const nlohmann::json root = nlohmann::json::parse(stream);

    const auto lidar_start = std::chrono::steady_clock::now();
    out.raster = buildPanoramaRaster(root);
    out.coverage = out.raster.coverage;
    out.lidar_signature = lidarSignature(out.raster);
    const auto points =
        organizedPoints(root, out.raster.rows, out.raster.columns);
    const LidarManhattanFrame manhattan = estimateManhattanFrame(
        points, static_cast<std::size_t>(out.raster.rows),
        static_cast<std::size_t>(out.raster.columns));
    out.normal_count = manhattan.normal_count;
    out.lidar_feature_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - lidar_start)
                               .count();

    const auto image_start = std::chrono::steady_clock::now();
    int edge_pixels = 0;
    out.camera_signature =
        cameraSignature(camera_bgr, options.signature_bins, &edge_pixels);
    out.camera_elevation_signature =
        cameraElevationSignature(camera_bgr, options.signature_bins);
    cv::Mat gray;
    if (camera_bgr.channels() == 1)
      gray = camera_bgr;
    else
      cv::cvtColor(camera_bgr, gray, cv::COLOR_BGR2GRAY);
    const auto segments = detectLineSegments(gray, 8.0, 0.04);
    out.line_count = static_cast<int>(segments.size());
    const auto vanishing = estimateVanishingDirections(segments, camera.k, 3);
    for (const auto &direction : vanishing) {
      out.image_vanishing_directions.push_back(direction.direction);
      out.image_vanishing_line_counts.push_back(direction.line_count);
      out.image_vanishing_support_weights.push_back(direction.support_weight);
    }
    out.image_feature_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - image_start)
                               .count();

    if (out.coverage < options.minimum_coverage)
      out.fallback_reason = "LIDAR_COVERAGE_INSUFFICIENT";
    else if (edge_pixels < options.minimum_camera_edge_pixels)
      out.fallback_reason = "CAMERA_EDGE_INSUFFICIENT";
    else if (vanishing.size() < 2)
      out.fallback_reason = "INSUFFICIENT_IMAGE_DIRECTIONS";
    else if (!manhattan.valid)
      out.fallback_reason = "INSUFFICIENT_LIDAR_AXES";

    const auto structural_start = std::chrono::steady_clock::now();
    const auto structural = structuralHypotheses(vanishing, manhattan);
    // The LiDAR Manhattan frame supplies gravity.  In the camera frame the
    // corresponding vanishing direction predicts down and optical roll
    // directly; choosing an arbitrary signed-permutation candidate by yaw can
    // otherwise introduce a 20-degree down error even when yaw is correct.
    const VanishingDirection *gravity_vanishing = nullptr;
    double gravity_score = -1.0;
    for (const auto &direction : vanishing) {
      const Eigen::Vector3d v = direction.direction.normalized();
      // For a downward-looking camera below nadir, the visible world-down
      // direction has camera y and z with the same sign.  The product is sign
      // invariant, so it also works when the VP estimator flips the axis.
      if (v.y() * v.z() <= 0.0)
        continue;
      const double score = std::abs(v.y());
      if (score > gravity_score) {
        gravity_score = score;
        gravity_vanishing = &direction;
        out.gravity_vanishing_index =
            static_cast<int>(&direction - vanishing.data());
      }
    }
    double predicted_down_deg = 0.0;
    double predicted_roll_deg = 0.0;
    double predicted_roll_sigma_deg = 8.0;
    if (gravity_vanishing) {
      Eigen::Vector3d vertical = gravity_vanishing->direction.normalized();
      if (vertical.z() < 0.0)
        vertical = -vertical;
      predicted_down_deg = degrees(std::asin(std::clamp(vertical.z(), 0.0, 1.0)));
      predicted_roll_deg = degrees(std::atan2(-vertical.x(), vertical.y()));
      const double support =
          std::sqrt(static_cast<double>(std::max(1, gravity_vanishing->line_count)));
      predicted_roll_sigma_deg = std::clamp(15.0 / support, 2.0, 8.0);
    }
    out.structural_hypothesis_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - structural_start)
            .count();
    if (out.fallback_reason.empty() && structural.empty())
      out.fallback_reason = "NO_VALID_STRUCTURAL_HYPOTHESIS";
    if (out.fallback_reason.empty() && !gravity_vanishing)
      out.fallback_reason = "NO_VALID_GRAVITY_VANISHING_DIRECTION";

    const auto search_start = std::chrono::steady_clock::now();
    for (double down = 0.0; down <= 75.0; down += 1.0)
      out.down_degrees.push_back(down);
    for (double yaw = -180.0; yaw < 180.0 - 1e-9; yaw += options.yaw_step_deg)
      out.yaw_degrees.push_back(yaw);
    out.raw_scores = scoreAngularSignatures(
        out.camera_signature, out.lidar_signature, camera.k, camera_bgr.cols,
        out.raster.pan_min_rad, out.raster.pan_max_rad, out.yaw_degrees);
    out.evaluated_signature_yaws = static_cast<int>(out.raw_scores.size());
    const int basin_radius =
        std::max(1, static_cast<int>(std::lround(options.basin_radius_deg /
                                                 options.yaw_step_deg)));
    out.basin_scores = smooth(out.raw_scores, basin_radius, true);
    out.signature_search_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - search_start)
            .count();

    const double mean = out.basin_scores.empty()
                            ? 0.0
                            : std::accumulate(out.basin_scores.begin(),
                                              out.basin_scores.end(), 0.0) /
                                  out.basin_scores.size();
    const double sigma = standardDeviation(out.basin_scores, mean);
    std::vector<std::size_t> order(out.basin_scores.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return out.basin_scores[a] > out.basin_scores[b];
    });
    for (std::size_t index : order) {
      const double yaw = out.yaw_degrees[index];
      bool separated = true;
      for (const auto &proposal : out.proposals)
        if (circularDistance(yaw, proposal.yaw_deg) <
            options.nms_separation_deg)
          separated = false;
      if (!separated)
        continue;

      const StructuralHypothesis *best = nullptr;
      double best_distance = std::numeric_limits<double>::infinity();
      for (const auto &candidate : structural) {
        const double distance = circularDistance(yaw, candidate.yaw);
        if (distance < best_distance) {
          best_distance = distance;
          best = &candidate;
        }
      }
      if (!best)
        continue;

      double weighted_square = 0.0;
      double weight_sum = 0.0;
      const int uncertainty_radius =
          std::max(basin_radius,
                   static_cast<int>(std::lround(10.0 / options.yaw_step_deg)));
      for (int d = -uncertainty_radius; d <= uncertainty_radius; ++d) {
        int j = static_cast<int>(index) + d;
        j %= static_cast<int>(out.basin_scores.size());
        if (j < 0)
          j += static_cast<int>(out.basin_scores.size());
        const double weight =
            std::exp(12.0 * (out.basin_scores[static_cast<std::size_t>(j)] -
                             out.basin_scores[index]));
        const double angle = d * options.yaw_step_deg;
        weighted_square += weight * angle * angle;
        weight_sum += weight;
      }
      const double yaw_sigma =
          weight_sum > 0 ? std::sqrt(weighted_square / weight_sum) : 10.0;
      const double zscore =
          sigma > 1e-9 ? (out.basin_scores[index] - mean) / sigma : 0.0;
      const auto elevation_signature = lidarElevationSignature(
          out.raster, yaw, camera.k, camera_bgr.cols);
      auto down_scores = scoreElevationSignatures(
          out.camera_elevation_signature, elevation_signature, camera.k,
          camera_bgr.rows, out.raster.tilt_min_rad, out.raster.tilt_max_rad,
          out.down_degrees);
      down_scores = smooth(down_scores, 3, false);
      out.evaluated_elevation_candidates += static_cast<int>(down_scores.size());
      const auto best_down_it =
          std::max_element(down_scores.begin(), down_scores.end());
      const std::size_t best_down_index = static_cast<std::size_t>(
          std::distance(down_scores.begin(), best_down_it));
      const double signature_down_deg = out.down_degrees[best_down_index];
      double down_weight = 0.0;
      double down_square = 0.0;
      for (int d = -10; d <= 10; ++d) {
        const int j = static_cast<int>(best_down_index) + d;
        if (j < 0 || j >= static_cast<int>(down_scores.size()))
          continue;
        const double weight = std::exp(10.0 *
            (down_scores[static_cast<std::size_t>(j)] - *best_down_it));
        down_weight += weight;
        down_square += weight * d * d;
      }
      const double signature_down_sigma =
          down_weight > 0.0
              ? std::clamp(std::sqrt(down_square / down_weight), 3.0, 15.0)
              : 15.0;
      const bool vanishing_consistent =
          std::abs(predicted_down_deg - signature_down_deg) <= 15.0 &&
          std::abs(predicted_roll_deg) <= 10.0;
      const double selected_roll_deg =
          vanishing_consistent ? predicted_roll_deg : 0.0;
      const double selected_roll_sigma_deg =
          vanishing_consistent ? predicted_roll_sigma_deg : 10.0;
      out.lidar_elevation_signatures.push_back(elevation_signature);
      out.down_score_curves.push_back(down_scores);
      HybridOrientationProposal proposal;
      proposal.rank = static_cast<int>(out.proposals.size()) + 1;
      proposal.yaw_deg = yaw;
      proposal.down_deg = signature_down_deg;
      proposal.roll_deg = selected_roll_deg;
      proposal.raw_score = out.raw_scores[index];
      proposal.basin_score = out.basin_scores[index];
      proposal.confidence = std::clamp(zscore / 3.0, 0.0, 1.0) *
                            std::clamp(1.0 - best_distance / 45.0, 0.25, 1.0);
      proposal.yaw_sigma_deg = yaw_sigma;
      proposal.down_sigma_deg = signature_down_sigma;
      proposal.roll_sigma_deg = selected_roll_sigma_deg;
      proposal.search_radius_deg = std::clamp(2.0 * yaw_sigma, 5.0, 20.0);
      proposal.evidence =
          "AZIMUTH_SIGNATURE,ELEVATION_SIGNATURE,GRAVITY_VP_CHECK,MANHATTAN_GATE";
      out.proposals.push_back(proposal);
      if (static_cast<int>(out.proposals.size()) >= options.top_k)
        break;
    }

    const double best_zscore =
        out.proposals.empty() || sigma <= 1e-9
            ? 0.0
            : (out.proposals.front().basin_score - mean) / sigma;
    if (out.fallback_reason.empty() && out.proposals.empty())
      out.fallback_reason = "NO_DISTINCT_YAW_BASIN";
    if (out.fallback_reason.empty() &&
        best_zscore < options.minimum_peak_zscore)
      out.fallback_reason = "YAW_SIGNATURE_AMBIGUOUS";

    out.fallback_required = !out.fallback_reason.empty();
    out.status =
        out.fallback_required ? "INSUFFICIENT_FEATURES" : "PROPOSALS_READY";
  } catch (const std::exception &error) {
    out.status = "INVALID_INPUT";
    out.fallback_required = true;
    out.fallback_reason = error.what();
  }
  out.runtime_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - total_start)
                       .count();
  return out;
}

bool writeHybridAnalyzerArtifacts(const HybridAnalyzerResult &r,
                                  const std::filesystem::path &directory) {
  std::filesystem::create_directories(directory);
  nlohmann::json proposals = nlohmann::json::array();
  for (const auto &p : r.proposals)
    proposals.push_back({{"rank", p.rank},
                         {"yaw_deg", p.yaw_deg},
                         {"down_deg", p.down_deg},
                         {"roll_deg", p.roll_deg},
                         {"raw_score", p.raw_score},
                         {"basin_score", p.basin_score},
                         {"confidence", p.confidence},
                         {"yaw_sigma_deg", p.yaw_sigma_deg},
                         {"down_sigma_deg", p.down_sigma_deg},
                         {"roll_sigma_deg", p.roll_sigma_deg},
                         {"search_radius_deg", p.search_radius_deg},
                         {"evidence", p.evidence}});
  const nlohmann::json root = {
      {"gravity_vanishing_index", r.gravity_vanishing_index},
      {"image_vanishing_directions",
       [&] {
         nlohmann::json axes = nlohmann::json::array();
         for (std::size_t i = 0; i < r.image_vanishing_directions.size(); ++i)
           axes.push_back({{"direction", {r.image_vanishing_directions[i].x(),
                                            r.image_vanishing_directions[i].y(),
                                            r.image_vanishing_directions[i].z()}},
                           {"line_count", r.image_vanishing_line_counts[i]},
                           {"support_weight", r.image_vanishing_support_weights[i]}});
         return axes;
       }()},
      {"schema_version", r.schema_version},
      {"mode", r.mode},
      {"status", r.status},
      {"fallback_required", r.fallback_required},
      {"fallback_reason", r.fallback_reason},
      {"coverage", r.coverage},
      {"line_count", r.line_count},
      {"normal_count", r.normal_count},
      {"proposal_count", r.proposals.size()},
      {"proposals", proposals},
      {"evaluated_signature_yaws", r.evaluated_signature_yaws},
      {"evaluated_elevation_candidates", r.evaluated_elevation_candidates},
      {"perspective_remaps", r.perspective_remaps},
      {"expensive_projection_evaluations", r.expensive_projection_evaluations},
      {"runtime_ms", r.runtime_ms},
      {"timing_ms",
       {{"image_feature", r.image_feature_ms},
        {"lidar_feature", r.lidar_feature_ms},
        {"signature_search", r.signature_search_ms},
        {"structural_hypothesis", r.structural_hypothesis_ms}}}};
  std::ofstream json_file(directory / "analyzer_result.json");
  if (!json_file)
    return false;
  json_file << std::setw(2) << root << '\n';

  std::ofstream proposal_file(directory / "orientation_proposals.csv");
  proposal_file << "rank,yaw_deg,down_deg,roll_deg,raw_score,basin_score,"
                   "confidence,yaw_sigma_deg,down_sigma_deg,roll_sigma_deg,"
                   "search_radius_deg,evidence\n";
  for (const auto &p : r.proposals)
    proposal_file << p.rank << ',' << p.yaw_deg << ',' << p.down_deg << ','
                  << p.roll_deg << ',' << p.raw_score << ',' << p.basin_score
                  << ',' << p.confidence << ',' << p.yaw_sigma_deg << ','
                  << p.down_sigma_deg << ',' << p.roll_sigma_deg << ','
                  << p.search_radius_deg << ',' << p.evidence << '\n';

  const auto write_series = [&](const char *name,
                                const std::vector<double> &values) {
    std::ofstream file(directory / name);
    file << "index,value\n";
    for (std::size_t i = 0; i < values.size(); ++i)
      file << i << ',' << values[i] << '\n';
  };
  write_series("azimuth_signature_camera.csv", r.camera_signature);
  write_series("azimuth_signature_lidar.csv", r.lidar_signature);
  write_series("elevation_signature_camera.csv", r.camera_elevation_signature);
  for (std::size_t i = 0; i < r.down_score_curves.size(); ++i) {
    std::ofstream down_curve(
        directory / ("elevation_score_curve_rank_" + std::to_string(i + 1) + ".csv"));
    down_curve << "down_deg,score\n";
    for (std::size_t j = 0; j < r.down_degrees.size(); ++j)
      down_curve << r.down_degrees[j] << ',' << r.down_score_curves[i][j] << '\n';
    write_series(("elevation_signature_lidar_rank_" + std::to_string(i + 1) +
                  ".csv").c_str(), r.lidar_elevation_signatures[i]);
  }
  std::ofstream curve(directory / "azimuth_score_curve.csv");
  curve << "yaw_deg,raw_score,basin_score\n";
  for (std::size_t i = 0; i < r.yaw_degrees.size(); ++i)
    curve << r.yaw_degrees[i] << ',' << r.raw_scores[i] << ','
          << r.basin_scores[i] << '\n';
  std::ofstream timing(directory / "analyzer_timing.json");
  timing << std::setw(2)
         << nlohmann::json(
                {{"runtime_ms", r.runtime_ms},
                 {"image_feature_ms", r.image_feature_ms},
                 {"lidar_feature_ms", r.lidar_feature_ms},
                 {"signature_search_ms", r.signature_search_ms},
                 {"structural_hypothesis_ms", r.structural_hypothesis_ms},
                 {"evaluated_signature_yaws", r.evaluated_signature_yaws},
                 {"evaluated_elevation_candidates", r.evaluated_elevation_candidates},
                 {"perspective_remaps", r.perspective_remaps},
                 {"expensive_projection_evaluations",
                  r.expensive_projection_evaluations}})
         << '\n';
  if (!r.raster.range_m.empty()) {
    cv::Mat range_preview;
    cv::normalize(r.raster.range_m, range_preview, 0, 255, cv::NORM_MINMAX,
                  CV_8U);
    cv::imwrite((directory / "panorama_range.png").string(), range_preview);
    cv::imwrite((directory / "panorama_normal_edge.png").string(),
                r.raster.normal_edge);
  }
  return proposal_file && curve && timing;
}

} // namespace auto_calib

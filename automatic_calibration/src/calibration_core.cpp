#include "auto_calib/calibration_core.hpp"
#include <algorithm>
#include <array>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <chrono>
#include <cmath>
#include <deque>
#include <Eigen/Eigenvalues>
#include <limits>
#include <map>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace auto_calib {
namespace {
constexpr double kRadToDeg = 57.295779513082320876;
using Parameters = std::array<double, 6>;
using Intrinsics = std::array<double, 4>;
Parameters toParameters(const Transform &t) {
  Eigen::AngleAxisd aa(t.rotation);
  Eigen::Vector3d v = aa.angle() * aa.axis();
  return {v.x(),
          v.y(),
          v.z(),
          t.translation_m.x(),
          t.translation_m.y(),
          t.translation_m.z()};
}
Transform fromParameters(const Parameters &p) {
  double angle = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
  Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
  if (angle > 1e-12)
    r = Eigen::AngleAxisd(angle, Eigen::Vector3d(p[0], p[1], p[2]) / angle)
            .toRotationMatrix();
  return {r, {p[3], p[4], p[5]}};
}
float bilinear(const cv::Mat &image, double u, double v) {
  int x = static_cast<int>(std::floor(u)), y = static_cast<int>(std::floor(v));
  double dx = u - x, dy = v - y;
  const float *a = image.ptr<float>(y);
  const float *b = image.ptr<float>(y + 1);
  return static_cast<float>((1 - dy) * ((1 - dx) * a[x] + dx * a[x + 1]) +
                            dy * ((1 - dx) * b[x] + dx * b[x + 1]));
}
double residualForPoint(const double *p, const Eigen::Vector3d &point,
                        const CameraModel &camera, const cv::Mat &distance,
                        const double *intrinsics = nullptr) {
  double in[3] = {point.x(), point.y(), point.z()}, out[3];
  ceres::AngleAxisRotatePoint(p, in, out);
  out[0] += p[3];
  out[1] += p[4];
  out[2] += p[5];
  if (out[2] <= 0.05)
    return 50.0;
  const double fx = intrinsics ? intrinsics[0] : camera.k(0, 0);
  const double fy = intrinsics ? intrinsics[1] : camera.k(1, 1);
  const double cx = intrinsics ? intrinsics[2] : camera.k(0, 2);
  const double cy = intrinsics ? intrinsics[3] : camera.k(1, 2);
  double u = fx * out[0] / out[2] + cx, v = fy * out[1] / out[2] + cy;
  if (u < 0 || v < 0 || u >= distance.cols - 1 || v >= distance.rows - 1) {
    double du = u < 0 ? -u
                      : (u >= distance.cols - 1 ? u - (distance.cols - 2) : 0),
           dv = v < 0 ? -v
                      : (v >= distance.rows - 1 ? v - (distance.rows - 2) : 0);
    return 30.0 + std::hypot(du, dv);
  }
  return bilinear(distance, u, v);
}
struct GeometryFeature {
  Eigen::Vector3d point;
  Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
  double value = 0.0;
};
struct NidEvaluation {
  double score = 1.0;
  double squared_score = 1.0;
  std::size_t projected = 0;
};
struct Evaluation {
  double normalized_squared = std::numeric_limits<double>::infinity();
  double mean = std::numeric_limits<double>::infinity();
  std::size_t projected = 0, in_frame = 0, visible = 0, occluded = 0;
  double horizontal_normalized_squared =
      std::numeric_limits<double>::infinity();
  double vertical_normalized_squared = std::numeric_limits<double>::infinity();
  std::size_t horizontal_visible = 0, vertical_visible = 0;
};
cv::Mat buildCameraGradientFeature(const cv::Mat &bgr) {
  cv::Mat gray;
  if (bgr.channels() == 3)
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  else
    gray = bgr;
  cv::Mat gray_f, gx, gy, magnitude;
  gray.convertTo(gray_f, CV_32F, 1.0 / 255.0);
  cv::Sobel(gray_f, gx, CV_32F, 1, 0, 3);
  cv::Sobel(gray_f, gy, CV_32F, 0, 1, 3);
  cv::magnitude(gx, gy, magnitude);
  cv::GaussianBlur(magnitude, magnitude, {5, 5}, 1.5);
  cv::Scalar mean, deviation;
  cv::meanStdDev(magnitude, mean, deviation);
  magnitude /= std::max(1e-6, mean[0] + 3.0 * deviation[0]);
  cv::threshold(magnitude, magnitude, 1.0, 1.0, cv::THRESH_TRUNC);
  return magnitude;
}
bool projectPoint(const double *p, const Eigen::Vector3d &point,
                  const CameraModel &camera, const double *intrinsics,
                  double *u, double *v, double *depth = nullptr) {
  double in[3] = {point.x(), point.y(), point.z()}, out[3];
  ceres::AngleAxisRotatePoint(p, in, out);
  out[0] += p[3];
  out[1] += p[4];
  out[2] += p[5];
  if (out[2] <= 0.05)
    return false;
  if (depth)
    *depth = out[2];
  const double fx = intrinsics ? intrinsics[0] : camera.k(0, 0);
  const double fy = intrinsics ? intrinsics[1] : camera.k(1, 1);
  const double cx = intrinsics ? intrinsics[2] : camera.k(0, 2);
  const double cy = intrinsics ? intrinsics[3] : camera.k(1, 2);
  *u = fx * out[0] / out[2] + cx;
  *v = fy * out[1] / out[2] + cy;
  return *u >= 0 && *v >= 0 && *u < camera.width - 1 && *v < camera.height - 1;
}
struct VisibilityBuffer {
  cv::Mat depth;
  double scale = 1.0;
};
VisibilityBuffer buildVisibilityBuffer(
    const Parameters &parameters, const std::vector<Eigen::Vector3d> &cloud,
    const CameraModel &camera, const Intrinsics *intrinsics, double scale) {
  VisibilityBuffer result;
  result.scale = std::clamp(scale, 0.05, 1.0);
  const int width = std::max(1, static_cast<int>(camera.width * result.scale));
  const int height =
      std::max(1, static_cast<int>(camera.height * result.scale));
  result.depth = cv::Mat(height, width, CV_32F,
                         cv::Scalar(std::numeric_limits<float>::infinity()));
  for (const auto &point : cloud) {
    double u = 0.0, v = 0.0, z = 0.0;
    if (!projectPoint(parameters.data(), point, camera,
                      intrinsics ? intrinsics->data() : nullptr, &u, &v, &z))
      continue;
    const int x = std::clamp(static_cast<int>(u * result.scale), 0, width - 1);
    const int y = std::clamp(static_cast<int>(v * result.scale), 0, height - 1);
    result.depth.at<float>(y, x) =
        std::min(result.depth.at<float>(y, x), static_cast<float>(z));
  }
  return result;
}
bool visiblePoint(const Parameters &parameters, const Eigen::Vector3d &point,
                  const CameraModel &camera, const Intrinsics *intrinsics,
                  const VisibilityBuffer *visibility, double tolerance_m,
                  double *u = nullptr, double *v = nullptr) {
  double px = 0.0, py = 0.0, z = 0.0;
  if (!projectPoint(parameters.data(), point, camera,
                    intrinsics ? intrinsics->data() : nullptr, &px, &py, &z))
    return false;
  if (u)
    *u = px;
  if (v)
    *v = py;
  if (!visibility)
    return true;
  const int x = std::clamp(static_cast<int>(px * visibility->scale), 0,
                           visibility->depth.cols - 1);
  const int y = std::clamp(static_cast<int>(py * visibility->scale), 0,
                           visibility->depth.rows - 1);
  return z <= visibility->depth.at<float>(y, x) + tolerance_m;
}
struct ImageLineSegment {
  cv::Point2d a, b;
  double angle = 0.0;
  double length = 0.0;
};
using LidarLineSegment = StructuralLineSegment3d;
struct StructuralImageFeature {
  std::vector<cv::Mat> directional_distance;
  std::vector<ImageLineSegment> segments;
  std::size_t line_count = 0;
};
StructuralImageFeature buildStructuralLineDistance(
    const cv::Mat &bgr, const CalibrationConfig &config) {
  cv::Mat gray;
  if (bgr.channels() == 3)
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  else
    gray = bgr;
  std::vector<cv::Vec4f> lines;
  cv::createLineSegmentDetector(cv::LSD_REFINE_STD)->detect(gray, lines);
  constexpr int direction_bins = 8;
  std::vector<cv::Mat> masks(
      direction_bins, cv::Mat(gray.size(), CV_8U, cv::Scalar(0)));
  const double minimum_length = config.minimum_structural_line_length_ratio *
                                std::min(gray.cols, gray.rows);
  std::size_t retained = 0;
  std::vector<ImageLineSegment> retained_segments;
  for (const auto &line : lines) {
    const double length = std::hypot(line[2] - line[0], line[3] - line[1]);
    if (length < minimum_length)
      continue;
    double angle = std::atan2(line[3] - line[1], line[2] - line[0]);
    if (angle < 0.0)
      angle += M_PI;
    const int bin = static_cast<int>(
                        std::lround(angle * direction_bins / M_PI)) %
                    direction_bins;
    cv::line(masks[bin], {cvRound(line[0]), cvRound(line[1])},
             {cvRound(line[2]), cvRound(line[3])}, cv::Scalar(255), 2,
             cv::LINE_AA);
    retained_segments.push_back(
        {{line[0], line[1]}, {line[2], line[3]}, angle, length});
    ++retained;
  }
  std::vector<cv::Mat> distances;
  distances.reserve(direction_bins);
  for (int bin = 0; bin < direction_bins; ++bin) {
    cv::Mat combined = masks[bin].clone();
    cv::bitwise_or(combined,
                   masks[(bin + direction_bins - 1) % direction_bins],
                   combined);
    cv::bitwise_or(combined, masks[(bin + 1) % direction_bins], combined);
    cv::Mat inverse, distance;
    cv::bitwise_not(combined, inverse);
    cv::distanceTransform(inverse, distance, cv::DIST_L2, 3);
    distances.push_back(std::move(distance));
  }
  return {std::move(distances), std::move(retained_segments), retained};
}

bool isRangeDiscontinuity(const Point &a, const Point &b,
                          const CalibrationConfig &config) {
  if (!a.valid() || !b.valid())
    return false;
  const double threshold =
      std::max(config.lidar_edge_absolute_threshold_m,
               config.lidar_edge_relative_threshold *
                   std::min(a.range, b.range));
  return std::abs(a.range - b.range) > threshold;
}

std::vector<LidarLineSegment>
extractRangeDiscontinuityLineSegments(const Scan &scan,
                                      const CalibrationConfig &config) {
  std::vector<LidarLineSegment> segments;
  const auto index = [&](std::uint32_t row, std::uint32_t column) {
    return static_cast<std::size_t>(row) * scan.config.columns + column;
  };
  std::vector<Eigen::Vector3d> run;
  const auto flush = [&]() {
    if (run.size() >= config.minimum_lidar_structural_segment_points &&
        (run.back() - run.front()).norm() >=
            config.minimum_lidar_structural_segment_length_m)
      segments.push_back({run.front(), run.back()});
    run.clear();
  };
  const auto append = [&](const Point *point) {
    if (!point) {
      flush();
      return;
    }
    const Eigen::Vector3d xyz = point->xyz.cast<double>();
    if (!run.empty() &&
        (xyz - run.back()).norm() >
            std::max(0.25, 2.0 * config.minimum_lidar_structural_segment_length_m))
      flush();
    run.push_back(xyz);
  };

  // A discontinuity between adjacent pan columns forms a segment while tilt
  // changes. Choose the nearer surface, then require a contiguous run.
  for (std::uint32_t column = 0; column + 1 < scan.config.columns; ++column) {
    for (std::uint32_t row = 0; row < scan.config.rows; ++row) {
      const auto &a = scan.points[index(row, column)];
      const auto &b = scan.points[index(row, column + 1)];
      append(isRangeDiscontinuity(a, b, config)
                 ? (a.range <= b.range ? &a : &b)
                 : nullptr);
    }
    flush();
  }
  // A discontinuity between adjacent tilt rows forms a segment while pan
  // changes. This is the case expected for desk/floor height boundaries.
  for (std::uint32_t row = 0; row + 1 < scan.config.rows; ++row) {
    for (std::uint32_t column = 0; column < scan.config.columns; ++column) {
      const auto &a = scan.points[index(row, column)];
      const auto &b = scan.points[index(row + 1, column)];
      append(isRangeDiscontinuity(a, b, config)
                 ? (a.range <= b.range ? &a : &b)
                 : nullptr);
    }
    flush();
  }
  return segments;
}

void validateOrganizedScan(const Scan &scan) {
  if (scan.points.size() !=
      static_cast<std::size_t>(scan.config.rows) * scan.config.columns)
    throw std::invalid_argument("Organized scan shape mismatch");
}

LidarPlaneSegmentation computeRobustNormals(
    const Scan &scan, const CalibrationConfig &config) {
  validateOrganizedScan(scan);
  LidarPlaneSegmentation result;
  result.normals.assign(scan.points.size(), Eigen::Vector3d::Zero());
  result.has_normal.assign(scan.points.size(), 0);
  result.labels.assign(scan.points.size(), -1);
  const auto index = [&](std::uint32_t row, std::uint32_t column) {
    return static_cast<std::size_t>(row) * scan.config.columns + column;
  };
  const auto connected = [&](std::size_t a, std::size_t b) {
    return scan.points[a].valid() && scan.points[b].valid() &&
           !isRangeDiscontinuity(scan.points[a], scan.points[b], config);
  };
  for (std::uint32_t row = 0; row < scan.config.rows; ++row)
    for (std::uint32_t column = 0; column < scan.config.columns; ++column) {
      const std::size_t current = index(row, column);
      if (!scan.points[current].valid())
        continue;
      std::vector<Eigen::Vector3d> horizontal, vertical;
      const Eigen::Vector3d center =
          scan.points[current].xyz.cast<double>();
      if (column > 0 && connected(current, index(row, column - 1)))
        horizontal.push_back(
            center - scan.points[index(row, column - 1)].xyz.cast<double>());
      if (column + 1 < scan.config.columns &&
          connected(current, index(row, column + 1)))
        horizontal.push_back(
            scan.points[index(row, column + 1)].xyz.cast<double>() - center);
      if (row > 0 && connected(current, index(row - 1, column)))
        vertical.push_back(
            center - scan.points[index(row - 1, column)].xyz.cast<double>());
      if (row + 1 < scan.config.rows &&
          connected(current, index(row + 1, column)))
        vertical.push_back(
            scan.points[index(row + 1, column)].xyz.cast<double>() - center);
      Eigen::Vector3d best = Eigen::Vector3d::Zero();
      for (const auto &h : horizontal)
        for (const auto &v : vertical) {
          const Eigen::Vector3d candidate = h.cross(v);
          if (candidate.squaredNorm() > best.squaredNorm())
            best = candidate;
        }
      if (best.norm() <= 1e-9)
        continue;
      best.normalize();
      if (best.dot(center) > 0.0)
        best = -best;
      result.normals[current] = best;
      result.has_normal[current] = 1;
    }
  return result;
}

std::vector<GeometryFeature>
extractGeometryFeatures(const Scan &scan, const CalibrationConfig &config) {
  const auto normal_field = computeRobustNormals(scan, config);
  const auto &normals = normal_field.normals;
  const auto &has_normal = normal_field.has_normal;
  auto index = [&](std::uint32_t row, std::uint32_t column) {
    return static_cast<std::size_t>(row) * scan.config.columns + column;
  };
  std::vector<GeometryFeature> features;
  features.reserve(scan.valid_count);
  constexpr int offsets[4][2] = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};
  for (std::uint32_t row = 0; row < scan.config.rows; ++row)
    for (std::uint32_t column = 0; column < scan.config.columns; ++column) {
      const std::size_t current = index(row, column);
      const auto &point = scan.points[current];
      if (!point.valid())
        continue;
      double range_score = 0.0;
      double normal_score = 0.0;
      Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
      for (const auto &offset : offsets) {
        const int neighbor_row = static_cast<int>(row) + offset[0];
        const int neighbor_column = static_cast<int>(column) + offset[1];
        if (neighbor_row < 0 || neighbor_column < 0 ||
            neighbor_row >= static_cast<int>(scan.config.rows) ||
            neighbor_column >= static_cast<int>(scan.config.columns))
          continue;
        const std::size_t neighbor =
            index(static_cast<std::uint32_t>(neighbor_row),
                  static_cast<std::uint32_t>(neighbor_column));
        const auto &other = scan.points[neighbor];
        if (!other.valid())
          continue;
        const double threshold =
            std::max(config.lidar_edge_absolute_threshold_m,
                     config.lidar_edge_relative_threshold *
                         std::min(point.range, other.range));
        const double current_range_score = std::clamp(
            (std::abs(point.range - other.range) / threshold - 1.0) / 2.0,
            0.0, 1.0);
        if (current_range_score > range_score) {
          range_score = current_range_score;
          const bool pan_discontinuity = offset[0] == 0;
          const auto a = pan_discontinuity ? index(row - (row > 0), column)
                                           : index(row, column - (column > 0));
          const auto b = pan_discontinuity
                             ? index(std::min(row + 1, scan.config.rows - 1),
                                     column)
                             : index(row, std::min(column + 1,
                                                   scan.config.columns - 1));
          if (scan.points[a].valid() && scan.points[b].valid())
            tangent = (scan.points[b].xyz - scan.points[a].xyz).cast<double>();
        }
        if (has_normal[current] && has_normal[neighbor]) {
          const double cosine = std::clamp(
              std::abs(normals[current].dot(normals[neighbor])), 0.0, 1.0);
          const double angle = std::acos(cosine);
          normal_score = std::max(
              normal_score,
              std::clamp(angle / config.lidar_normal_change_threshold_rad, 0.0,
                         1.0));
        }
      }
      const double structural_score = std::max(range_score, normal_score);
      if (structural_score >= 0.05)
        features.push_back(
            {point.xyz.cast<double>(), tangent.normalized(), structural_score});
    }
  if (config.maximum_nid_points > 1 &&
      features.size() > config.maximum_nid_points) {
    const int bins = std::max(2, config.nid_histogram_bins);
    std::vector<std::vector<GeometryFeature>> buckets(bins);
    for (const auto &feature : features) {
      const int bin =
          std::min(static_cast<int>(std::clamp(feature.value, 0.0, 1.0) * bins),
                   bins - 1);
      buckets[bin].push_back(feature);
    }
    const std::size_t active_buckets = static_cast<std::size_t>(
        std::count_if(buckets.begin(), buckets.end(),
                      [](const auto &bucket) { return !bucket.empty(); }));
    const std::size_t quota =
        std::max<std::size_t>(1, config.maximum_nid_points / active_buckets);
    std::vector<GeometryFeature> sampled;
    sampled.reserve(config.maximum_nid_points);
    for (const auto &bucket : buckets) {
      const std::size_t take = std::min(quota, bucket.size());
      for (std::size_t i = 0; i < take; ++i)
        sampled.push_back(bucket[i * bucket.size() / take]);
    }
    return sampled;
  }
  return features;
}
double undirectedAngleDifference(double a, double b) {
  const double difference = std::abs(a - b);
  return std::min(difference, M_PI - difference);
}

Evaluation evaluateStructuralLines(
    const Parameters &p, const std::vector<LidarLineSegment> &segments,
    const CameraModel &camera, const StructuralImageFeature &image_lines,
    const Intrinsics *intrinsics, double residual_cap_px,
    const VisibilityBuffer *visibility, double visibility_tolerance_m,
    const CalibrationConfig &config) {
  Evaluation result;
  if (segments.empty() || image_lines.segments.empty())
    return result;
  double sum = 0.0, squared_sum = 0.0;
  double horizontal_sum = 0.0, vertical_sum = 0.0;
  const double cap = std::max(1.0, residual_cap_px);
  for (const auto &segment : segments) {
    // A fitted plane intersection often crosses the image while both fitted
    // endpoints are outside it. Keep the visible in-frame portion instead of
    // dropping the whole structural line.
    std::vector<cv::Point2d> visible_samples;
    constexpr int sample_count = 33;
    const double line_visibility_tolerance =
        std::max(visibility_tolerance_m,
                 config.maximum_lidar_plane_rms_error_m);
    for (int sample = 0; sample < sample_count; ++sample) {
      const double t = static_cast<double>(sample) / (sample_count - 1);
      const Eigen::Vector3d point = (1.0 - t) * segment.a + t * segment.b;
      double u = 0.0, v = 0.0;
      if (visiblePoint(p, point, camera, intrinsics, visibility,
                       line_visibility_tolerance, &u, &v))
        visible_samples.emplace_back(u, v);
    }
    if (visible_samples.size() < 2)
      continue;
    const cv::Point2d a = visible_samples.front();
    const cv::Point2d b = visible_samples.back();
    const cv::Point2d projected_vector = b - a;
    const double projected_length = cv::norm(projected_vector);
    if (projected_length < 4.0)
      continue;
    double angle = std::atan2(projected_vector.y, projected_vector.x);
    if (angle < 0.0)
      angle += M_PI;
    double best = std::numeric_limits<double>::infinity();
    for (const auto &image_line : image_lines.segments) {
      const double direction_difference =
          undirectedAngleDifference(angle, image_line.angle);
      if (direction_difference > config.structural_max_direction_difference_rad)
        continue;
      const cv::Point2d axis = (image_line.b - image_line.a) / image_line.length;
      const auto along = [&](const cv::Point2d &point) {
        return (point - image_line.a).dot(axis);
      };
      const auto across = [&](const cv::Point2d &point) {
        const cv::Point2d delta = point - image_line.a;
        return std::abs(delta.x * axis.y - delta.y * axis.x);
      };
      const double endpoint_distance = 0.5 * (across(a) + across(b));
      const double t0 = along(a), t1 = along(b);
      const double overlap =
          std::max(0.0, std::min(std::max(t0, t1), image_line.length) -
                            std::max(std::min(t0, t1), 0.0));
      const double overlap_ratio =
          std::clamp(overlap / std::max(1.0, std::min(projected_length,
                                                     image_line.length)),
                     0.0, 1.0);
      const double endpoint_objective =
          std::pow(std::min(endpoint_distance, cap) / cap, 2.0);
      const double direction_objective =
          std::pow(direction_difference /
                       config.structural_max_direction_difference_rad,
                   2.0);
      const double overlap_objective = std::pow(1.0 - overlap_ratio, 2.0);
      best = std::min(best,
                      config.structural_endpoint_weight * endpoint_objective +
                          config.structural_direction_weight *
                              direction_objective +
                          config.structural_overlap_weight * overlap_objective);
    }
    if (!std::isfinite(best))
      best = 1.0;
    sum += std::sqrt(best) * cap;
    squared_sum += best;
    ++result.visible;
    if (best < 0.5)
      ++result.projected;
    const double horizontal_distance = std::min(angle, M_PI - angle);
    const double vertical_distance = std::abs(angle - 0.5 * M_PI);
    if (horizontal_distance <= M_PI / 6.0) {
      horizontal_sum += best;
      ++result.horizontal_visible;
    } else if (vertical_distance <= M_PI / 6.0) {
      vertical_sum += best;
      ++result.vertical_visible;
    }
  }
  if (result.visible > 0) {
    result.mean = sum / result.visible;
    result.normalized_squared = squared_sum / result.visible;
  }
  if (result.horizontal_visible > 0)
    result.horizontal_normalized_squared =
        horizontal_sum / result.horizontal_visible;
  if (result.vertical_visible > 0)
    result.vertical_normalized_squared = vertical_sum / result.vertical_visible;
  return result;
}
NidEvaluation normalizedInformationDistance(
    const double *parameters, const std::vector<GeometryFeature> &features,
    const CameraModel &camera, const cv::Mat &camera_feature, int bins,
    const double *intrinsics = nullptr,
    const VisibilityBuffer *visibility = nullptr,
    double visibility_tolerance_m = 0.01) {
  NidEvaluation result;
  if (bins < 2 || features.empty())
    return result;
  std::vector<double> joint(static_cast<std::size_t>(bins) * bins, 1e-12);
  Parameters p;
  std::copy(parameters, parameters + 6, p.begin());
  Intrinsics k;
  const Intrinsics *kp = nullptr;
  if (intrinsics) {
    std::copy(intrinsics, intrinsics + 4, k.begin());
    kp = &k;
  }
  for (const auto &feature : features) {
    double u = 0.0, v = 0.0;
    if (!visiblePoint(p, feature.point, camera, kp, visibility,
                      visibility_tolerance_m, &u, &v))
      continue;
    const double x = std::clamp(feature.value, 0.0, 1.0) * (bins - 1);
    const double y =
        std::clamp(static_cast<double>(bilinear(camera_feature, u, v)), 0.0,
                   1.0) *
        (bins - 1);
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, bins - 1);
    const int y1 = std::min(y0 + 1, bins - 1);
    const double dx = x - x0, dy = y - y0;
    joint[static_cast<std::size_t>(x0) * bins + y0] += (1.0 - dx) * (1.0 - dy);
    joint[static_cast<std::size_t>(x0) * bins + y1] += (1.0 - dx) * dy;
    joint[static_cast<std::size_t>(x1) * bins + y0] += dx * (1.0 - dy);
    joint[static_cast<std::size_t>(x1) * bins + y1] += dx * dy;
    ++result.projected;
  }
  if (result.projected == 0)
    return result;
  std::vector<double> lidar_hist(bins, 0.0), camera_hist(bins, 0.0);
  double total = 0.0;
  for (int x = 0; x < bins; ++x)
    for (int y = 0; y < bins; ++y) {
      const double value = joint[static_cast<std::size_t>(x) * bins + y];
      lidar_hist[x] += value;
      camera_hist[y] += value;
      total += value;
    }
  auto entropy = [total](const std::vector<double> &histogram) {
    double value = 0.0;
    for (double count : histogram) {
      const double probability = count / total;
      if (probability > 0.0)
        value -= probability * std::log(probability);
    }
    return value;
  };
  const double lidar_entropy = entropy(lidar_hist);
  const double camera_entropy = entropy(camera_hist);
  const double joint_entropy = entropy(joint);
  if (joint_entropy <= 1e-9)
    return result;
  const double mutual_information =
      std::max(0.0, lidar_entropy + camera_entropy - joint_entropy);
  result.score = std::clamp(1.0 - mutual_information / joint_entropy, 0.0, 1.0);
  result.squared_score = result.score * result.score;
  return result;
}
struct NidCost {
  const std::vector<GeometryFeature> &features;
  const CameraModel &camera;
  const cv::Mat &camera_feature;
  int bins;
  double scale;
  bool optimize_intrinsics;
  bool operator()(double const *const *blocks, double *residuals) const {
    const double *intrinsics = optimize_intrinsics ? blocks[1] : nullptr;
    residuals[0] =
        scale * normalizedInformationDistance(blocks[0], features, camera,
                                              camera_feature, bins, intrinsics)
                    .score;
    return true;
  }
};
struct EdgeCost {
  const std::vector<Eigen::Vector3d> &points;
  const CameraModel &camera;
  const cv::Mat &distance;
  double residual_cap_px, scale;
  bool optimize_intrinsics;
  bool operator()(double const *const *blocks, double *residuals) const {
    const double *intrinsics = optimize_intrinsics ? blocks[1] : nullptr;
    for (std::size_t i = 0; i < points.size(); ++i)
      residuals[i] =
          scale * std::min(residual_cap_px,
                           residualForPoint(blocks[0], points[i], camera,
                                            distance, intrinsics));
    return true;
  }
};
struct StructuralLineCost {
  const std::vector<LidarLineSegment> &segments;
  const CameraModel &camera;
  const StructuralImageFeature &image_lines;
  const CalibrationConfig &config;
  double scale;
  bool optimize_intrinsics;
  bool operator()(double const *const *blocks, double *residuals) const {
    Parameters parameters;
    std::copy(blocks[0], blocks[0] + 6, parameters.begin());
    Intrinsics intrinsics;
    const Intrinsics *active = nullptr;
    if (optimize_intrinsics) {
      std::copy(blocks[1], blocks[1] + 4, intrinsics.begin());
      active = &intrinsics;
    }
    const Evaluation evaluation = evaluateStructuralLines(
        parameters, segments, camera, image_lines, active,
        config.residual_cap_px, nullptr, config.visibility_depth_tolerance_m,
        config);
    residuals[0] =
        scale * std::sqrt(std::isfinite(evaluation.normalized_squared)
                              ? evaluation.normalized_squared
                              : 1.0);
    return true;
  }
};
struct PriorCost {
  Parameters prior;
  double rs, ts, weight;
  bool constrain_translation;
  template <typename T> bool operator()(const T *p, T *residuals) const {
    for (int i = 0; i < 3; ++i)
      residuals[i] = T(weight) * (p[i] - T(prior[i])) / T(rs);
    for (int i = 3; i < 6; ++i)
      residuals[i] = constrain_translation
                         ? T(weight) * (p[i] - T(prior[i])) / T(ts)
                         : T(0);
    return true;
  }
};
struct CameraCenterPriorCost {
  Eigen::Vector3d expected;
  double scale;
  bool operator()(double const *const *blocks, double *residuals) const {
    const double inverse_rotation[3] = {-blocks[0][0], -blocks[0][1],
                                        -blocks[0][2]};
    const double translation[3] = {-blocks[0][3], -blocks[0][4],
                                    -blocks[0][5]};
    double center[3];
    ceres::AngleAxisRotatePoint(inverse_rotation, translation, center);
    for (int i = 0; i < 3; ++i)
      residuals[i] = scale * (center[i] - expected[i]);
    return true;
  }
};
struct IntrinsicPriorCost {
  Intrinsics prior;
  double focal_sigma_ratio, cx_sigma_px, cy_sigma_px, weight;
  template <typename T> bool operator()(const T *k, T *residuals) const {
    residuals[0] =
        T(weight) * (k[0] - T(prior[0])) / T(prior[0] * focal_sigma_ratio);
    residuals[1] =
        T(weight) * (k[1] - T(prior[1])) / T(prior[1] * focal_sigma_ratio);
    residuals[2] = T(weight) * (k[2] - T(prior[2])) / T(cx_sigma_px);
    residuals[3] = T(weight) * (k[3] - T(prior[3])) / T(cy_sigma_px);
    return true;
  }
};
CameraModel withIntrinsics(CameraModel camera, const Intrinsics &intrinsics) {
  camera.k(0, 0) = intrinsics[0];
  camera.k(1, 1) = intrinsics[1];
  camera.k(0, 2) = intrinsics[2];
  camera.k(1, 2) = intrinsics[3];
  return camera;
}
double directionPriorObjective(const Parameters &p,
                               const CalibrationConfig &config) {
  if (config.camera_direction_prior_weight <= 0.0)
    return 0.0;
  const Eigen::Matrix3d inverse = fromParameters(p).rotation.transpose();
  double objective = 0.0;
  if (config.expected_camera_forward_lidar.norm() > 1e-9)
    objective +=
        0.25 * (inverse * Eigen::Vector3d::UnitZ() -
                config.expected_camera_forward_lidar.normalized())
                   .squaredNorm();
  if (config.expected_camera_down_lidar.norm() > 1e-9)
    objective +=
        0.25 * (inverse * Eigen::Vector3d::UnitY() -
                config.expected_camera_down_lidar.normalized())
                   .squaredNorm();
  return objective;
}
Evaluation evaluate(const Parameters &p,
                    const std::vector<Eigen::Vector3d> &points,
                    const CameraModel &camera, const cv::Mat &distance,
                    const Intrinsics *intrinsics = nullptr,
                    double residual_cap_px = 20.0,
                    const VisibilityBuffer *visibility = nullptr,
                    double visibility_tolerance_m = 0.01) {
  Evaluation e;
  if (points.empty())
    return e;
  double sum = 0.0, normalized_squared_sum = 0.0;
  for (const auto &point : points) {
    double u = 0.0, v = 0.0;
    if (!projectPoint(p.data(), point, camera,
                      intrinsics ? intrinsics->data() : nullptr, &u, &v))
      continue;
    ++e.in_frame;
    if (!visiblePoint(p, point, camera, intrinsics, visibility,
                      visibility_tolerance_m)) {
      ++e.occluded;
      continue;
    }
    ++e.visible;
    const double r =
        residualForPoint(p.data(), point, camera, distance,
                         intrinsics ? intrinsics->data() : nullptr);
    sum += r;
    const double normalized =
        std::min(r, residual_cap_px) / std::max(1.0, residual_cap_px);
    normalized_squared_sum += normalized * normalized;
    if (r < 30.0)
      ++e.projected;
  }
  if (e.visible == 0)
    return e;
  e.mean = sum / e.visible;
  e.normalized_squared = normalized_squared_sum / e.visible;
  return e;
}
double compositeObjective(const Evaluation &edge, const NidEvaluation &nid,
                          const Evaluation &line,
                          const CalibrationConfig &config,
                          double direction_prior = 0.0) {
  return config.edge_alignment_weight * edge.normalized_squared +
         config.normalized_information_distance_weight * nid.squared_score +
         config.structural_line_weight * line.normalized_squared +
         config.camera_direction_prior_weight * direction_prior;
}
struct DirectionPriorCost {
  Eigen::Vector3d expected_forward;
  Eigen::Vector3d expected_down;
  double scale;
  bool operator()(double const *const *blocks, double *residuals) const {
    double inverse_rotation[3] = {-blocks[0][0], -blocks[0][1],
                                  -blocks[0][2]};
    const double camera_z[3] = {0.0, 0.0, 1.0};
    const double camera_y[3] = {0.0, 1.0, 0.0};
    double forward[3], down[3];
    ceres::AngleAxisRotatePoint(inverse_rotation, camera_z, forward);
    ceres::AngleAxisRotatePoint(inverse_rotation, camera_y, down);
    for (int i = 0; i < 3; ++i)
      residuals[i] = expected_forward.norm() > 1e-9
                         ? scale * (forward[i] - expected_forward[i])
                         : 0.0;
    for (int i = 0; i < 3; ++i)
      residuals[i + 3] = expected_down.norm() > 1e-9
                             ? scale * (down[i] - expected_down[i])
                             : 0.0;
    return true;
  }
};
Parameters coarseSearch(Parameters p, const Parameters &prior,
                        const std::vector<Eigen::Vector3d> &points,
                        const CameraModel &camera, const cv::Mat &distance,
                        const CalibrationConfig &config) {
  auto best =
      evaluate(p, points, camera, distance, nullptr, config.residual_cap_px)
          .mean;
  double rs = config.coarse_rotation_step_rad,
         ts = config.coarse_translation_step_m;
  for (int round = 0; round < config.coarse_rounds; ++round) {
    for (int axis = 0; axis < 6; ++axis) {
      Parameters winner = p;
      double winner_cost = best;
      double step = axis < 3 ? rs : ts;
      for (double sign : {-1.0, 1.0}) {
        Parameters candidate = p;
        candidate[axis] += sign * step;
        double bound = axis < 3 ? config.rotation_search_bound_rad
                                : config.translation_search_bound_m;
        if (std::abs(candidate[axis] - prior[axis]) > bound)
          continue;
        double cost = evaluate(candidate, points, camera, distance, nullptr,
                               config.residual_cap_px)
                          .mean;
        if (cost < winner_cost) {
          winner = candidate;
          winner_cost = cost;
        }
      }
      p = winner;
      best = winner_cost;
    }
    rs *= 0.5;
    ts *= 0.5;
  }
  return p;
}
} // namespace

cv::Mat buildCameraEdgeDistanceTransform(const cv::Mat &bgr,
                                         const CalibrationConfig &config,
                                         std::size_t *edge_count) {
  if (bgr.empty())
    throw std::invalid_argument("Camera image is empty");
  cv::Mat gray;
  if (bgr.channels() == 3)
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  else if (bgr.channels() == 1)
    gray = bgr;
  else
    throw std::invalid_argument("Camera image must have 1 or 3 channels");
  cv::Mat blurred, edges;
  cv::GaussianBlur(gray, blurred, {5, 5}, 1.0);
  cv::Canny(blurred, edges, config.canny_low_threshold,
            config.canny_high_threshold);
  if (edge_count)
    *edge_count = cv::countNonZero(edges);
  cv::Mat inverse, distance;
  cv::bitwise_not(edges, inverse);
  cv::distanceTransform(inverse, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  return distance;
}

std::vector<Eigen::Vector3d>
extractLidarEdgePoints(const Scan &scan, const CalibrationConfig &config) {
  if (scan.points.size() !=
      static_cast<std::size_t>(scan.config.rows) * scan.config.columns)
    throw std::invalid_argument("Organized scan shape mismatch");
  std::vector<bool> selected(scan.points.size(), false);
  auto mark = [&](std::size_t a, std::size_t b) {
    const auto &x = scan.points[a];
    const auto &y = scan.points[b];
    if (!x.valid() || !y.valid())
      return;
    double threshold = std::max(config.lidar_edge_absolute_threshold_m,
                                config.lidar_edge_relative_threshold *
                                    std::min(x.range, y.range));
    if (std::abs(x.range - y.range) > threshold) {
      selected[a] = true;
      selected[b] = true;
    }
  };
  for (std::uint32_t r = 0; r < scan.config.rows; ++r)
    for (std::uint32_t c = 0; c < scan.config.columns; ++c) {
      std::size_t i = static_cast<std::size_t>(r) * scan.config.columns + c;
      if (c + 1 < scan.config.columns)
        mark(i, i + 1);
      if (r + 1 < scan.config.rows)
        mark(i, i + scan.config.columns);
    }
  std::vector<Eigen::Vector3d> out;
  for (std::size_t i = 0; i < selected.size(); ++i)
    if (selected[i])
      out.push_back(scan.points[i].xyz.cast<double>());
  return out;
}

LidarPlaneSegmentation segmentLidarPlanes(const Scan &scan,
                                          const CalibrationConfig &config) {
  auto result = computeRobustNormals(scan, config);
  const auto index = [&](std::uint32_t row, std::uint32_t column) {
    return static_cast<std::size_t>(row) * scan.config.columns + column;
  };
  const double normal_cosine = std::cos(config.lidar_plane_normal_threshold_rad);
  const auto fit_plane = [&](const std::vector<std::size_t> &cells,
                             LidarPlane3d &plane) {
    if (cells.size() < 3)
      return false;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const std::size_t cell : cells)
      centroid += scan.points[cell].xyz.cast<double>();
    centroid /= static_cast<double>(cells.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    Eigen::Vector3d average_normal = Eigen::Vector3d::Zero();
    for (const std::size_t cell : cells) {
      const Eigen::Vector3d delta =
          scan.points[cell].xyz.cast<double>() - centroid;
      covariance += delta * delta.transpose();
      average_normal += result.normals[cell];
    }
    covariance /= static_cast<double>(cells.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success ||
        std::sqrt(12.0 * std::max(0.0, solver.eigenvalues()[1])) <
            config.minimum_lidar_plane_extent_m)
      return false;
    Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
    if (normal.dot(average_normal) < 0.0)
      normal = -normal;
    const double offset = -normal.dot(centroid);
    double squared_error = 0.0;
    for (const std::size_t cell : cells) {
      const double error =
          normal.dot(scan.points[cell].xyz.cast<double>()) + offset;
      squared_error += error * error;
    }
    const double rms =
        std::sqrt(squared_error / static_cast<double>(cells.size()));
    if (rms > config.maximum_lidar_plane_rms_error_m)
      return false;
    plane = {normal, offset, cells.size(), rms};
    return true;
  };
  std::vector<unsigned char> visited(scan.points.size(), 0);
  constexpr int offsets[4][2] = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};
  for (std::uint32_t seed_row = 0; seed_row < scan.config.rows; ++seed_row)
    for (std::uint32_t seed_column = 0; seed_column < scan.config.columns;
         ++seed_column) {
      const std::size_t seed = index(seed_row, seed_column);
      if (visited[seed] || !result.has_normal[seed])
        continue;
      visited[seed] = 1;
      std::deque<std::size_t> queue{seed};
      std::vector<std::size_t> component;
      while (!queue.empty()) {
        const std::size_t current = queue.front();
        queue.pop_front();
        component.push_back(current);
        const std::uint32_t row = current / scan.config.columns;
        const std::uint32_t column = current % scan.config.columns;
        const Eigen::Vector3d p = scan.points[current].xyz.cast<double>();
        for (const auto &offset : offsets) {
          const int neighbor_row = static_cast<int>(row) + offset[0];
          const int neighbor_column = static_cast<int>(column) + offset[1];
          if (neighbor_row < 0 || neighbor_column < 0 ||
              neighbor_row >= static_cast<int>(scan.config.rows) ||
              neighbor_column >= static_cast<int>(scan.config.columns))
            continue;
          const std::size_t neighbor =
              index(static_cast<std::uint32_t>(neighbor_row),
                    static_cast<std::uint32_t>(neighbor_column));
          if (visited[neighbor] || !result.has_normal[neighbor] ||
              isRangeDiscontinuity(scan.points[current], scan.points[neighbor],
                                   config))
            continue;
          if (std::abs(result.normals[current].dot(result.normals[neighbor])) <
              normal_cosine)
            continue;
          const Eigen::Vector3d delta =
              scan.points[neighbor].xyz.cast<double>() - p;
          if (std::abs(result.normals[current].dot(delta)) >
                  config.lidar_plane_neighbor_distance_threshold_m ||
              std::abs(result.normals[neighbor].dot(delta)) >
                  config.lidar_plane_neighbor_distance_threshold_m)
            continue;
          visited[neighbor] = 1;
          queue.push_back(neighbor);
        }
      }
      if (component.size() < config.minimum_lidar_plane_points)
        continue;
      LidarPlane3d plane;
      if (!fit_plane(component, plane))
        continue;
      const int label = static_cast<int>(result.planes.size());
      result.planes.push_back(plane);
      for (const std::size_t cell : component)
        result.labels[cell] = label;
    }

  // Recover rejected small fragments from an accepted neighboring plane. The
  // plane equation and normal gates prevent propagation across real corners.
  std::deque<std::size_t> frontier;
  for (std::size_t cell = 0; cell < result.labels.size(); ++cell)
    if (result.labels[cell] >= 0)
      frontier.push_back(cell);
  while (!frontier.empty()) {
    const std::size_t current = frontier.front();
    frontier.pop_front();
    const int label = result.labels[current];
    const auto &plane = result.planes[static_cast<std::size_t>(label)];
    const std::uint32_t row = current / scan.config.columns;
    const std::uint32_t column = current % scan.config.columns;
    for (const auto &offset : offsets) {
      const int neighbor_row = static_cast<int>(row) + offset[0];
      const int neighbor_column = static_cast<int>(column) + offset[1];
      if (neighbor_row < 0 || neighbor_column < 0 ||
          neighbor_row >= static_cast<int>(scan.config.rows) ||
          neighbor_column >= static_cast<int>(scan.config.columns))
        continue;
      const std::size_t neighbor =
          index(static_cast<std::uint32_t>(neighbor_row),
                static_cast<std::uint32_t>(neighbor_column));
      if (result.labels[neighbor] >= 0 || !result.has_normal[neighbor] ||
          std::abs(result.normals[neighbor].dot(plane.normal)) < normal_cosine)
        continue;
      const double distance = std::abs(
          plane.normal.dot(scan.points[neighbor].xyz.cast<double>()) +
          plane.offset);
      if (distance > config.lidar_plane_neighbor_distance_threshold_m)
        continue;
      result.labels[neighbor] = label;
      frontier.push_back(neighbor);
    }
  }

  // The IMU-gated LiDAR frame has +Y down. Recover horizontal surfaces by
  // height when region growing split a desk/floor into sub-threshold pieces.
  const double height_bin_m =
      std::max(0.005, config.maximum_lidar_plane_rms_error_m / 3.0);
  const int height_radius = std::max(
      1, static_cast<int>(std::ceil(
             config.maximum_lidar_plane_rms_error_m / height_bin_m)));
  const double horizontal_cosine = std::cos(std::min(
      1.0471975511965976, 2.0 * config.lidar_plane_normal_threshold_rad));
  std::map<int, std::vector<std::size_t>> height_bins;
  for (std::size_t cell = 0; cell < scan.points.size(); ++cell)
    if (result.has_normal[cell] &&
        std::abs(result.normals[cell].y()) >= horizontal_cosine) {
      const int bin = static_cast<int>(std::floor(
          scan.points[cell].xyz.y() / height_bin_m));
      height_bins[bin].push_back(cell);
    }
  const auto height_support = [&](int center) {
    std::size_t count = 0;
    for (int offset = -height_radius; offset <= height_radius; ++offset) {
      const auto found = height_bins.find(center + offset);
      if (found != height_bins.end())
        count += found->second.size();
    }
    return count;
  };
  std::vector<std::pair<std::size_t, int>> height_peaks;
  for (const auto &[bin, cells] : height_bins) {
    const std::size_t support = height_support(bin);
    if (support >= config.minimum_lidar_plane_points)
      height_peaks.push_back({support, bin});
  }
  std::sort(height_peaks.begin(), height_peaks.end(), std::greater<>());
  std::vector<unsigned char> height_claimed(scan.points.size(), 0);
  for (const auto &[support, center] : height_peaks) {
    (void)support;
    std::vector<std::size_t> cells;
    for (int offset = -height_radius; offset <= height_radius; ++offset) {
      const auto found = height_bins.find(center + offset);
      if (found == height_bins.end())
        continue;
      for (const std::size_t cell : found->second)
        if (!height_claimed[cell])
          cells.push_back(cell);
    }
    if (cells.size() < config.minimum_lidar_plane_points)
      continue;
    LidarPlane3d plane;
    if (!fit_plane(cells, plane) ||
        std::abs(plane.normal.y()) < normal_cosine)
      continue;
    const int label = static_cast<int>(result.planes.size());
    result.planes.push_back(plane);
    for (const std::size_t cell : cells) {
      result.labels[cell] = label;
      height_claimed[cell] = 1;
    }
  }

  // Merge only neighboring, geometrically coplanar labels. This repairs seams
  // created by dropout/range discontinuities without joining parallel walls.
  std::vector<int> parent(result.planes.size());
  for (std::size_t i = 0; i < parent.size(); ++i)
    parent[i] = static_cast<int>(i);
  const auto root = [&](int label) {
    while (parent[static_cast<std::size_t>(label)] != label)
      label = parent[static_cast<std::size_t>(label)];
    return label;
  };
  const auto merge = [&](int a, int b) {
    a = root(a);
    b = root(b);
    if (a != b)
      parent[static_cast<std::size_t>(b)] = a;
  };
  for (std::uint32_t row = 0; row < scan.config.rows; ++row)
    for (std::uint32_t column = 0; column < scan.config.columns; ++column) {
      const int first = result.labels[index(row, column)];
      if (first < 0)
        continue;
      for (const auto &offset : offsets) {
        const int neighbor_row = static_cast<int>(row) + offset[0];
        const int neighbor_column = static_cast<int>(column) + offset[1];
        if (neighbor_row < 0 || neighbor_column < 0 ||
            neighbor_row >= static_cast<int>(scan.config.rows) ||
            neighbor_column >= static_cast<int>(scan.config.columns))
          continue;
        const int second = result.labels[index(
            static_cast<std::uint32_t>(neighbor_row),
            static_cast<std::uint32_t>(neighbor_column))];
        if (second < 0 || first == second)
          continue;
        const auto &a = result.planes[static_cast<std::size_t>(first)];
        const auto &b = result.planes[static_cast<std::size_t>(second)];
        const double alignment = a.normal.dot(b.normal);
        if (std::abs(alignment) < normal_cosine)
          continue;
        const double aligned_offset = alignment < 0.0 ? -b.offset : b.offset;
        if (std::abs(a.offset - aligned_offset) <=
            config.lidar_plane_neighbor_distance_threshold_m)
          merge(first, second);
      }
    }

  std::vector<std::vector<std::size_t>> merged_cells(result.planes.size());
  for (std::size_t cell = 0; cell < result.labels.size(); ++cell)
    if (result.labels[cell] >= 0)
      merged_cells[static_cast<std::size_t>(root(result.labels[cell]))]
          .push_back(cell);
  std::vector<LidarPlane3d> merged_planes;
  std::vector<int> remap(result.planes.size(), -1);
  for (std::size_t old = 0; old < merged_cells.size(); ++old) {
    if (merged_cells[old].size() < config.minimum_lidar_plane_points)
      continue;
    LidarPlane3d plane;
    if (!fit_plane(merged_cells[old], plane))
      plane = result.planes[old];
    remap[old] = static_cast<int>(merged_planes.size());
    merged_planes.push_back(plane);
  }
  for (int &label : result.labels)
    if (label >= 0)
      label = remap[static_cast<std::size_t>(root(label))];
  result.planes = std::move(merged_planes);
  return result;
}

std::vector<StructuralLineSegment3d> extractLidarPlaneIntersectionSegments(
    const Scan &scan, const LidarPlaneSegmentation &segmentation,
    const CalibrationConfig &config,
    std::vector<PlaneIntersectionDiagnostic> *diagnostics) {
  validateOrganizedScan(scan);
  if (segmentation.labels.size() != scan.points.size())
    throw std::invalid_argument("Plane labels do not match organized scan");
  struct BoundarySupport {
    std::size_t contacts = 0;
    std::vector<std::size_t> cells;
  };
  std::map<std::pair<int, int>, BoundarySupport> boundaries;
  const auto index = [&](std::uint32_t row, std::uint32_t column) {
    return static_cast<std::size_t>(row) * scan.config.columns + column;
  };
  const auto add_boundary = [&](std::size_t a, std::size_t b) {
    int first = segmentation.labels[a], second = segmentation.labels[b];
    if (first < 0 || second < 0 || first == second)
      return;
    if (first > second)
      std::swap(first, second);
    auto &support = boundaries[{first, second}];
    ++support.contacts;
    support.cells.push_back(a);
    support.cells.push_back(b);
  };
  const int radius = static_cast<int>(config.plane_pair_neighbor_radius_cells);
  for (std::uint32_t row = 0; row < scan.config.rows; ++row)
    for (std::uint32_t column = 0; column < scan.config.columns; ++column) {
      const std::size_t current = index(row, column);
      if (segmentation.labels[current] < 0)
        continue;
      for (int row_offset = 0; row_offset <= radius; ++row_offset)
        for (int column_offset = -radius; column_offset <= radius;
             ++column_offset) {
          if (row_offset == 0 && column_offset <= 0)
            continue;
          const int neighbor_row = static_cast<int>(row) + row_offset;
          const int neighbor_column =
              static_cast<int>(column) + column_offset;
          if (neighbor_row >= static_cast<int>(scan.config.rows) ||
              neighbor_column < 0 ||
              neighbor_column >= static_cast<int>(scan.config.columns))
            continue;
          add_boundary(
              current,
              index(static_cast<std::uint32_t>(neighbor_row),
                    static_cast<std::uint32_t>(neighbor_column)));
        }
    }

  std::vector<StructuralLineSegment3d> segments;
  for (const auto &[pair, support] : boundaries) {
    PlaneIntersectionDiagnostic diagnostic;
    diagnostic.first_plane = pair.first;
    diagnostic.second_plane = pair.second;
    diagnostic.boundary_contacts = support.contacts;
    if (support.contacts < config.minimum_plane_pair_boundary_contacts) {
      diagnostic.reason = "BOUNDARY_CONTACTS_INSUFFICIENT";
      if (diagnostics)
        diagnostics->push_back(diagnostic);
      continue;
    }
    const auto &first = segmentation.planes.at(pair.first);
    const auto &second = segmentation.planes.at(pair.second);
    const double cosine =
        std::clamp(std::abs(first.normal.dot(second.normal)), 0.0, 1.0);
    const double plane_angle = std::acos(cosine);
    diagnostic.plane_angle_deg = plane_angle * kRadToDeg;
    if (plane_angle < config.minimum_plane_intersection_angle_rad) {
      diagnostic.reason = "PLANE_ANGLE_TOO_SMALL";
      if (diagnostics)
        diagnostics->push_back(diagnostic);
      continue;
    }
    const Eigen::Vector3d raw_direction = first.normal.cross(second.normal);
    const double denominator = raw_direction.squaredNorm();
    if (denominator <= 1e-12) {
      diagnostic.reason = "PLANE_INTERSECTION_DEGENERATE";
      if (diagnostics)
        diagnostics->push_back(diagnostic);
      continue;
    }
    const Eigen::Vector3d direction = raw_direction.normalized();
    const Eigen::Vector3d origin =
        ((second.offset * first.normal - first.offset * second.normal)
             .cross(raw_direction)) /
        denominator;
    auto cells = support.cells;
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
    std::vector<double> distances, inlier_positions;
    distances.reserve(cells.size());
    inlier_positions.reserve(cells.size());
    for (const std::size_t cell : cells) {
      const Eigen::Vector3d point = scan.points[cell].xyz.cast<double>();
      const Eigen::Vector3d delta = point - origin;
      const double position = delta.dot(direction);
      const double distance = (delta - position * direction).norm();
      distances.push_back(distance);
      if (distance <=
          config.maximum_plane_intersection_boundary_distance_m)
        inlier_positions.push_back(position);
    }
    std::sort(distances.begin(), distances.end());
    const std::size_t p75 =
        std::min(distances.size() - 1, 3 * distances.size() / 4);
    diagnostic.boundary_distance_p75_m = distances[p75];
    diagnostic.boundary_inlier_points = inlier_positions.size();
    if (inlier_positions.size() <
        2 * config.minimum_plane_pair_boundary_contacts) {
      diagnostic.reason = "BOUNDARY_INLIERS_INSUFFICIENT";
      if (diagnostics)
        diagnostics->push_back(diagnostic);
      continue;
    }
    std::sort(inlier_positions.begin(), inlier_positions.end());
    const std::size_t lower = inlier_positions.size() / 20;
    const std::size_t upper = inlier_positions.size() - 1 - lower;
    const Eigen::Vector3d a = origin + inlier_positions[lower] * direction;
    const Eigen::Vector3d b = origin + inlier_positions[upper] * direction;
    diagnostic.segment_length_m = (b - a).norm();
    if (diagnostic.segment_length_m >=
        config.minimum_lidar_structural_segment_length_m) {
      segments.push_back({a, b});
      diagnostic.accepted = true;
      diagnostic.reason = "ACCEPTED";
    } else {
      diagnostic.reason = "SEGMENT_TOO_SHORT";
    }
    if (diagnostics)
      diagnostics->push_back(diagnostic);
  }
  return segments;
}

std::vector<StructuralLineSegment3d>
extractLidarOcclusionSegments(const Scan &scan,
                              const CalibrationConfig &config) {
  validateOrganizedScan(scan);
  return extractRangeDiscontinuityLineSegments(scan, config);
}

std::vector<StructuralLineSegment3d>
extractLidarStructuralSegments(const Scan &scan,
                               const CalibrationConfig &config) {
  const auto segmentation = segmentLidarPlanes(scan, config);
  return extractLidarPlaneIntersectionSegments(scan, segmentation, config);
}

CalibrationResult calibrateExtrinsic(const cv::Mat &bgr,
                                     const CameraModel &camera,
                                     const Scan &scan,
                                     const Transform &mechanical_prior,
                                     const CalibrationConfig &config) {
  auto started = std::chrono::steady_clock::now();
  CalibrationResult result;
  result.estimated_t_camera_lidar = mechanical_prior;
  result.candidate_t_camera_lidar = mechanical_prior;
  result.estimated_camera = camera;
  result.candidate_camera = camera;
  auto finish = [&]() {
    result.metrics.runtime_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - started)
                                    .count();
    return result;
  };
  try {
    if (camera.k(0, 0) <= 0 || camera.k(1, 1) <= 0) {
      result.reason_code = "INVALID_CAMERA_INTRINSIC";
      return finish();
    }
    cv::Mat distance = buildCameraEdgeDistanceTransform(
        bgr, config, &result.metrics.camera_edge_pixels);
    if (result.metrics.camera_edge_pixels < config.minimum_camera_edge_pixels) {
      result.reason_code = "CAMERA_EDGE_INSUFFICIENT";
      return finish();
    }
    auto points = extractLidarEdgePoints(scan, config);
    result.metrics.lidar_edge_points = points.size();
    if (points.size() < config.minimum_lidar_edge_points) {
      result.reason_code = "LIDAR_EDGE_INSUFFICIENT";
      return finish();
    }
    Parameters prior = toParameters(mechanical_prior), params = prior;
    auto initial = evaluate(params, points, camera, distance, nullptr,
                            config.residual_cap_px);
    result.metrics.initial_mean_edge_distance_px = initial.mean;
    params = coarseSearch(params, prior, points, camera, distance, config);
    ceres::Problem problem;
    auto *edge_cost =
        new ceres::DynamicNumericDiffCostFunction<EdgeCost, ceres::CENTRAL>(
            new EdgeCost{points, camera, distance, config.residual_cap_px,
                         1.0 / std::sqrt(static_cast<double>(points.size())),
                         false});
    edge_cost->AddParameterBlock(6);
    edge_cost->SetNumResiduals(static_cast<int>(points.size()));
    problem.AddResidualBlock(edge_cost, nullptr, params.data());
    auto *prior_cost = new ceres::AutoDiffCostFunction<PriorCost, 6, 6>(
        new PriorCost{prior, config.rotation_prior_sigma_rad,
                      config.translation_prior_sigma_m, config.prior_weight,
                      true});
    problem.AddResidualBlock(prior_cost, nullptr, params.data());
    for (int i = 0; i < 6; ++i) {
      double bound = i < 3 ? config.rotation_search_bound_rad
                           : config.translation_search_bound_m;
      problem.SetParameterLowerBound(params.data(), i, prior[i] - bound);
      problem.SetParameterUpperBound(params.data(), i, prior[i] + bound);
    }
    ceres::Solver::Options options;
    options.max_num_iterations = config.maximum_solver_iterations;
    options.linear_solver_type = ceres::DENSE_QR;
    options.num_threads = 1;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    auto final = evaluate(params, points, camera, distance, nullptr,
                          config.residual_cap_px);
    const Transform candidate = fromParameters(params);
    result.candidate_t_camera_lidar = candidate;
    result.metrics.projected_edge_points = final.projected;
    result.metrics.projected_ratio = double(final.projected) / points.size();
    result.metrics.final_mean_edge_distance_px = final.mean;
    result.metrics.objective_improvement_ratio =
        std::isfinite(initial.mean) && initial.mean > 0.0
            ? (initial.mean - final.mean) / initial.mean
            : 0.0;
    result.metrics.solver_iterations = summary.iterations.size();
    result.solver_summary = summary.BriefReport();
    const PoseError prior_update =
        calculatePoseError(candidate, mechanical_prior);
    if (!summary.IsSolutionUsable())
      result.reason_code = "OPTIMIZER_FAILED";
    else if (summary.termination_type == ceres::NO_CONVERGENCE)
      result.reason_code = "OPTIMIZER_NOT_CONVERGED";
    else if (result.metrics.projected_ratio < config.minimum_projected_ratio)
      result.reason_code = "OVERLAP_INSUFFICIENT";
    else if (final.mean > config.maximum_mean_edge_distance_px)
      result.reason_code = "EDGE_ALIGNMENT_POOR";
    else if (result.metrics.objective_improvement_ratio <
             config.minimum_objective_improvement_ratio)
      result.reason_code = "OBJECTIVE_IMPROVEMENT_INSUFFICIENT";
    else if (prior_update.rotation_deg >
                 config.maximum_rotation_update_rad * kRadToDeg ||
             prior_update.translation_m > config.maximum_translation_update_m)
      result.reason_code = "PRIOR_DEVIATION_EXCESSIVE";
    else {
      result.estimated_t_camera_lidar = candidate;
      result.success = true;
      result.reason_code = "PASS";
    }
  } catch (const std::exception &e) {
    result.reason_code = std::string("INPUT_ERROR: ") + e.what();
  }
  result.metrics.runtime_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - started)
                                  .count();
  return finish();
}

CalibrationResult calibrateExtrinsicMultiScene(
    const std::vector<CalibrationObservation> &observations,
    const Transform &mechanical_prior, const CalibrationConfig &config) {
  auto started = std::chrono::steady_clock::now();
  CalibrationResult result;
  result.estimated_t_camera_lidar = mechanical_prior;
  result.candidate_t_camera_lidar = mechanical_prior;
  auto finish = [&]() {
    result.metrics.runtime_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - started)
                                    .count();
    return result;
  };
  struct Prepared {
    CameraModel camera;
    cv::Mat distance;
    cv::Mat camera_feature;
    StructuralImageFeature structural_image;
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> cloud;
    std::vector<GeometryFeature> geometry;
    std::vector<LidarLineSegment> structural_segments;
  };
  try {
    if (observations.empty()) {
      result.reason_code = "OBSERVATIONS_EMPTY";
      return finish();
    }
    result.estimated_camera = observations.front().camera;
    result.candidate_camera = observations.front().camera;
    if (config.optimize_camera_intrinsics &&
        observations.size() < config.minimum_intrinsic_observations) {
      result.reason_code = "INTRINSIC_OBSERVATIONS_INSUFFICIENT";
      return finish();
    }
    if (config.optimize_camera_intrinsics &&
        (!(config.focal_length_relative_bound > 0.0 &&
           config.focal_length_relative_bound < 1.0) ||
         !(config.principal_point_bound_ratio > 0.0 &&
           config.principal_point_bound_ratio < 0.5))) {
      result.reason_code = "INVALID_INTRINSIC_SEARCH_BOUNDS";
      return finish();
    }
    if (config.edge_alignment_weight < 0.0 ||
        config.normalized_information_distance_weight < 0.0 ||
        config.structural_line_weight < 0.0 ||
        config.camera_direction_prior_weight < 0.0 ||
        config.edge_alignment_weight +
                config.normalized_information_distance_weight +
                config.structural_line_weight +
                config.camera_direction_prior_weight <=
            0.0) {
      result.reason_code = "INVALID_OBJECTIVE_WEIGHTS";
      return finish();
    }
    if (config.normalized_information_distance_weight > 0.0 &&
        (config.nid_histogram_bins < 2 || config.maximum_nid_points < 2 ||
         config.lidar_normal_change_threshold_rad <= 0.0)) {
      result.reason_code = "INVALID_NID_CONFIG";
      return finish();
    }
    if ((config.coarse_yaw_span_rad > 0.0 || config.use_coarse_yaw_bounds) &&
        (config.coarse_yaw_step_rad <= 0.0 ||
         (config.use_coarse_yaw_bounds &&
          config.coarse_yaw_max_rad < config.coarse_yaw_min_rad))) {
      result.reason_code = "INVALID_MULTISTART_CONFIG";
      return finish();
    }
    const double structural_component_weight =
        config.structural_direction_weight +
        config.structural_endpoint_weight + config.structural_overlap_weight;
    if (config.structural_line_weight > 0.0 &&
        (config.minimum_lidar_structural_segment_points < 2 ||
         config.minimum_lidar_structural_segment_length_m <= 0.0 ||
         config.minimum_lidar_plane_points < 3 ||
         config.lidar_plane_normal_threshold_rad <= 0.0 ||
         config.lidar_plane_neighbor_distance_threshold_m <= 0.0 ||
         config.minimum_lidar_plane_extent_m <= 0.0 ||
         config.maximum_lidar_plane_rms_error_m <= 0.0 ||
         config.plane_pair_neighbor_radius_cells < 1 ||
         config.minimum_plane_pair_boundary_contacts < 2 ||
         config.minimum_plane_intersection_angle_rad <= 0.0 ||
         config.maximum_plane_intersection_boundary_distance_m <= 0.0 ||
         config.structural_max_direction_difference_rad <= 0.0 ||
         std::abs(structural_component_weight - 1.0) > 1e-6)) {
      result.reason_code = "INVALID_STRUCTURAL_LINE_CONFIG";
      return finish();
    }

    std::vector<Prepared> prepared;
    prepared.reserve(observations.size());
    std::size_t total_points = 0;
    std::size_t total_geometry_points = 0;
    for (const auto &observation : observations) {
      if (observation.camera.k(0, 0) <= 0 || observation.camera.k(1, 1) <= 0) {
        result.reason_code = "INVALID_CAMERA_INTRINSIC";
        return finish();
      }
      if (observation.camera.width != observations.front().camera.width ||
          observation.camera.height != observations.front().camera.height) {
        result.reason_code = "CAMERA_MODEL_MISMATCH";
        return finish();
      }
      std::size_t camera_edges = 0;
      cv::Mat distance = buildCameraEdgeDistanceTransform(
          observation.bgr, config, &camera_edges);
      if (camera_edges < config.minimum_camera_edge_pixels) {
        result.reason_code = "CAMERA_EDGE_INSUFFICIENT";
        return finish();
      }
      auto points = extractLidarEdgePoints(observation.scan, config);
      if (points.size() < config.minimum_lidar_edge_points) {
        result.reason_code = "LIDAR_EDGE_INSUFFICIENT";
        return finish();
      }
      cv::Mat camera_feature = buildCameraGradientFeature(observation.bgr);
      auto geometry = extractGeometryFeatures(observation.scan, config);
      const auto plane_segmentation =
          segmentLidarPlanes(observation.scan, config);
      auto structural_segments = extractLidarPlaneIntersectionSegments(
          observation.scan, plane_segmentation, config);
      const auto occlusion_segments =
          extractRangeDiscontinuityLineSegments(observation.scan, config);
      const auto structural = buildStructuralLineDistance(observation.bgr, config);
      if (config.normalized_information_distance_weight > 0.0 &&
          geometry.size() < config.minimum_nid_projected_points) {
        result.reason_code = "LIDAR_GEOMETRY_FEATURE_INSUFFICIENT";
        return finish();
      }
      result.metrics.camera_edge_pixels += camera_edges;
      result.metrics.lidar_edge_points += points.size();
      result.metrics.lidar_geometry_points += geometry.size();
      result.metrics.camera_structural_lines += structural.line_count;
      result.metrics.lidar_planes += plane_segmentation.planes.size();
      result.metrics.lidar_structural_segments += structural_segments.size();
      result.metrics.lidar_occlusion_segments += occlusion_segments.size();
      total_points += points.size();
      total_geometry_points += geometry.size();
      std::vector<Eigen::Vector3d> cloud;
      cloud.reserve(observation.scan.valid_count);
      for (const auto &point : observation.scan.points)
        if (point.valid())
          cloud.push_back(point.xyz.cast<double>());
      prepared.push_back({observation.camera, std::move(distance),
                          std::move(camera_feature), structural,
                          std::move(points), std::move(cloud),
                          std::move(geometry),
                          std::move(structural_segments)});
    }

    Parameters prior = toParameters(mechanical_prior), params = prior;
    Intrinsics intrinsic_prior = {observations.front().camera.k(0, 0),
                                  observations.front().camera.k(1, 1),
                                  observations.front().camera.k(0, 2),
                                  observations.front().camera.k(1, 2)};
    Intrinsics intrinsics = intrinsic_prior;
    auto evaluate_edge_all = [&](const Parameters &parameters,
                                 const Intrinsics *camera_intrinsics) {
      Evaluation aggregate;
      aggregate.mean = 0.0;
      aggregate.normalized_squared = 0.0;
      aggregate.horizontal_normalized_squared = 0.0;
      aggregate.vertical_normalized_squared = 0.0;
      aggregate.projected = 0;
      aggregate.in_frame = 0;
      aggregate.visible = 0;
      aggregate.occluded = 0;
      for (const auto &item : prepared) {
        const auto visibility = config.enable_visibility_filter
                                    ? buildVisibilityBuffer(
                                          parameters, item.cloud, item.camera,
                                          camera_intrinsics,
                                          config.coarse_visibility_scale)
                                    : VisibilityBuffer{};
        const Evaluation current =
            evaluate(parameters, item.points, item.camera, item.distance,
                     camera_intrinsics, config.residual_cap_px,
                     config.enable_visibility_filter ? &visibility : nullptr,
                     config.visibility_depth_tolerance_m);
        if (current.visible > 0) {
          aggregate.mean += current.mean * current.visible;
          aggregate.normalized_squared +=
              current.normalized_squared * current.visible;
        }
        aggregate.projected += current.projected;
        aggregate.in_frame += current.in_frame;
        aggregate.visible += current.visible;
        aggregate.occluded += current.occluded;
      }
      if (aggregate.visible > 0) {
        aggregate.mean /= static_cast<double>(aggregate.visible);
        aggregate.normalized_squared /= static_cast<double>(aggregate.visible);
      }
      return aggregate;
    };
    auto evaluate_lines_all = [&](const Parameters &parameters,
                                  const Intrinsics *camera_intrinsics) {
      Evaluation aggregate;
      aggregate.mean = 0.0;
      aggregate.normalized_squared = 0.0;
      aggregate.horizontal_normalized_squared = 0.0;
      aggregate.vertical_normalized_squared = 0.0;
      for (const auto &item : prepared) {
        const auto visibility = config.enable_visibility_filter
                                    ? buildVisibilityBuffer(
                                          parameters, item.cloud, item.camera,
                                          camera_intrinsics,
                                          config.coarse_visibility_scale)
                                    : VisibilityBuffer{};
        const Evaluation current = evaluateStructuralLines(
            parameters, item.structural_segments, item.camera,
            item.structural_image,
            camera_intrinsics, config.residual_cap_px,
            config.enable_visibility_filter ? &visibility : nullptr,
            config.visibility_depth_tolerance_m, config);
        if (current.visible > 0) {
          aggregate.mean += current.mean * current.visible;
          aggregate.normalized_squared +=
              current.normalized_squared * current.visible;
        }
        aggregate.projected += current.projected;
        aggregate.in_frame += current.in_frame;
        aggregate.visible += current.visible;
        aggregate.occluded += current.occluded;
        if (current.horizontal_visible > 0) {
          aggregate.horizontal_normalized_squared +=
              current.horizontal_normalized_squared *
              current.horizontal_visible;
          aggregate.horizontal_visible += current.horizontal_visible;
        }
        if (current.vertical_visible > 0) {
          aggregate.vertical_normalized_squared +=
              current.vertical_normalized_squared * current.vertical_visible;
          aggregate.vertical_visible += current.vertical_visible;
        }
      }
      if (aggregate.visible > 0) {
        aggregate.mean /= static_cast<double>(aggregate.visible);
        aggregate.normalized_squared /= static_cast<double>(aggregate.visible);
      } else {
        aggregate.mean = config.residual_cap_px;
        aggregate.normalized_squared = 1.0;
      }
      if (aggregate.horizontal_visible > 0)
        aggregate.horizontal_normalized_squared /=
            static_cast<double>(aggregate.horizontal_visible);
      else
        aggregate.horizontal_normalized_squared =
            std::numeric_limits<double>::infinity();
      if (aggregate.vertical_visible > 0)
        aggregate.vertical_normalized_squared /=
            static_cast<double>(aggregate.vertical_visible);
      else
        aggregate.vertical_normalized_squared =
            std::numeric_limits<double>::infinity();
      return aggregate;
    };
    auto evaluate_nid_all = [&](const Parameters &parameters,
                                const Intrinsics *camera_intrinsics) {
      NidEvaluation aggregate;
      aggregate.score = 0.0;
      aggregate.squared_score = 0.0;
      aggregate.projected = 0;
      for (const auto &item : prepared) {
        const auto visibility = config.enable_visibility_filter
                                    ? buildVisibilityBuffer(
                                          parameters, item.cloud, item.camera,
                                          camera_intrinsics,
                                          config.coarse_visibility_scale)
                                    : VisibilityBuffer{};
        const NidEvaluation current = normalizedInformationDistance(
            parameters.data(), item.geometry, item.camera, item.camera_feature,
            config.nid_histogram_bins,
            camera_intrinsics ? camera_intrinsics->data() : nullptr,
            config.enable_visibility_filter ? &visibility : nullptr,
            config.visibility_depth_tolerance_m);
        aggregate.score += current.score;
        aggregate.squared_score += current.squared_score;
        aggregate.projected += current.projected;
      }
      aggregate.score /= static_cast<double>(prepared.size());
      aggregate.squared_score /= static_cast<double>(prepared.size());
      return aggregate;
    };
    const Intrinsics *active_initial_intrinsics =
        config.optimize_camera_intrinsics ? &intrinsics : nullptr;
    const Evaluation initial_edge =
        evaluate_edge_all(prior, active_initial_intrinsics);
    const NidEvaluation initial_nid =
        evaluate_nid_all(prior, active_initial_intrinsics);
    const Evaluation initial_line =
        evaluate_lines_all(prior, active_initial_intrinsics);
    result.metrics.initial_mean_edge_distance_px = initial_edge.mean;
    result.metrics.initial_nid = initial_nid.score;
    result.metrics.initial_composite_objective =
        compositeObjective(initial_edge, initial_nid, initial_line, config,
                           directionPriorObjective(prior, config));

    struct Start {
      Parameters parameters;
      double yaw_offset = 0.0;
      double objective = std::numeric_limits<double>::infinity();
      std::size_t edge_in_frame = 0;
      std::size_t edge_visible = 0;
      std::size_t edge_occluded = 0;
      std::size_t nid_projected = 0;
      double edge_objective = 0.0;
      double nid_objective = 0.0;
      double line_objective = 0.0;
      double horizontal_line_objective = 0.0;
      double vertical_line_objective = 0.0;
      double direction_objective = 0.0;
      std::size_t horizontal_line_segments = 0;
      std::size_t vertical_line_segments = 0;
    };
    std::vector<Start> starts;
    if (config.coarse_yaw_span_rad > 0.0 || config.use_coarse_yaw_bounds) {
      const double yaw_min = config.use_coarse_yaw_bounds
                                 ? config.coarse_yaw_min_rad
                                 : -config.coarse_yaw_span_rad;
      const double yaw_max = config.use_coarse_yaw_bounds
                                 ? config.coarse_yaw_max_rad
                                 : config.coarse_yaw_span_rad -
                                       0.5 * config.coarse_yaw_step_rad;
      for (double offset = yaw_min;
           offset <= yaw_max + 0.25 * config.coarse_yaw_step_rad;
           offset += config.coarse_yaw_step_rad) {
        Transform candidate = mechanical_prior;
        candidate.rotation = mechanical_prior.rotation *
                             Eigen::AngleAxisd(offset, Eigen::Vector3d::UnitY())
                                 .toRotationMatrix();
        if (config.use_camera_center_prior)
          candidate.translation_m =
              -candidate.rotation * config.expected_camera_center_lidar;
        const Parameters candidate_parameters = toParameters(candidate);
        const Evaluation edge =
            evaluate_edge_all(candidate_parameters, active_initial_intrinsics);
        const NidEvaluation nid =
            evaluate_nid_all(candidate_parameters, active_initial_intrinsics);
        const Evaluation line =
            evaluate_lines_all(candidate_parameters, active_initial_intrinsics);
        starts.push_back({candidate_parameters, offset,
                          compositeObjective(
                              edge, nid, line, config,
                              directionPriorObjective(candidate_parameters,
                                                      config)),
                          edge.in_frame, edge.visible, edge.occluded,
                          nid.projected, edge.normalized_squared,
                          nid.squared_score, line.normalized_squared,
                          line.horizontal_normalized_squared,
                          line.vertical_normalized_squared,
                          directionPriorObjective(candidate_parameters,
                                                  config),
                          line.horizontal_visible, line.vertical_visible});
      }
    } else {
      starts.push_back({prior, 0.0, result.metrics.initial_composite_objective,
                        initial_edge.in_frame, initial_edge.visible,
                        initial_edge.occluded, initial_nid.projected,
                        initial_edge.normalized_squared, initial_nid.squared_score,
                        initial_line.normalized_squared,
                        initial_line.horizontal_normalized_squared,
                        initial_line.vertical_normalized_squared,
                        directionPriorObjective(prior, config),
                        initial_line.horizontal_visible,
                        initial_line.vertical_visible});
    }
    result.coarse_orientation_scores.reserve(starts.size());
    for (auto &start : starts) {
      // The scan covers 360 degrees while a camera sees only one sector, so a
      // ratio against the full cloud rejects physically valid orientations.
      // Require enough absolute overlap for both independent objectives.
      const std::size_t minimum_edge_overlap =
          std::max<std::size_t>(100, config.minimum_lidar_edge_points);
      const std::size_t minimum_nid_overlap =
          config.minimum_nid_projected_points * prepared.size();
      const bool overlap_valid =
          start.edge_visible >= minimum_edge_overlap &&
          start.nid_projected >= minimum_nid_overlap;
      if (!overlap_valid)
        start.objective = std::numeric_limits<double>::infinity();
      result.coarse_orientation_scores.push_back(
          {start.yaw_offset * kRadToDeg, start.objective,
           start.edge_objective, start.nid_objective, start.line_objective,
           start.horizontal_line_objective,
           start.vertical_line_objective,
           start.direction_objective,
           start.edge_in_frame, start.edge_visible, start.edge_occluded,
           start.nid_projected, start.horizontal_line_segments,
           start.vertical_line_segments, overlap_valid});
    }
    std::sort(starts.begin(), starts.end(), [](const Start &a, const Start &b) {
      return a.objective < b.objective;
    });
    if (starts.empty() || !std::isfinite(starts.front().objective)) {
      result.reason_code = "COARSE_OVERLAP_INSUFFICIENT";
      result.metrics.multistart_candidates = starts.size();
      return finish();
    }
    params = starts.front().parameters;
    result.metrics.visible_edge_points = starts.front().edge_visible;
    result.metrics.occluded_edge_points = starts.front().edge_occluded;
    result.metrics.selected_multistart_yaw_deg =
        starts.front().yaw_offset * kRadToDeg;
    result.metrics.multistart_candidates = starts.size();
    double separated_second = std::numeric_limits<double>::infinity();
    const double separation =
        std::max(0.7854, 1.5 * config.coarse_yaw_step_rad);
    for (std::size_t i = 1; i < starts.size(); ++i) {
      const double angular_distance = std::acos(
          std::clamp(std::cos(starts[i].yaw_offset - starts.front().yaw_offset),
                     -1.0, 1.0));
      if (angular_distance >= separation) {
        separated_second = starts[i].objective;
        break;
      }
    }
    if (std::isfinite(separated_second))
      result.metrics.multistart_objective_margin =
          (separated_second - starts.front().objective) /
          std::max(separated_second, 1e-12);

    // Freeze candidate-specific visibility during Ceres. Rebuilding a
    // z-buffer inside numeric differentiation makes the residual discontinuous;
    // final scoring rebuilds visibility at the optimized pose.
    struct VisiblePrepared {
      std::vector<Eigen::Vector3d> points;
      std::vector<GeometryFeature> geometry;
      std::vector<LidarLineSegment> structural_segments;
    };
    std::vector<VisiblePrepared> visible_prepared;
    visible_prepared.reserve(prepared.size());
    for (const auto &item : prepared) {
      VisiblePrepared visible_item;
      const auto visibility = config.enable_visibility_filter
                                  ? buildVisibilityBuffer(
                                        params, item.cloud, item.camera,
                                        active_initial_intrinsics,
                                        config.coarse_visibility_scale)
                                  : VisibilityBuffer{};
      for (const auto &point : item.points)
        if (visiblePoint(params, point, item.camera, active_initial_intrinsics,
                         config.enable_visibility_filter ? &visibility : nullptr,
                         config.visibility_depth_tolerance_m))
          visible_item.points.push_back(point);
      for (const auto &feature : item.geometry)
        if (visiblePoint(params, feature.point, item.camera,
                         active_initial_intrinsics,
                         config.enable_visibility_filter ? &visibility : nullptr,
                         config.visibility_depth_tolerance_m))
          visible_item.geometry.push_back(feature);
      for (const auto &segment : item.structural_segments)
        if (visiblePoint(params, 0.5 * (segment.a + segment.b), item.camera,
                         active_initial_intrinsics,
                         config.enable_visibility_filter ? &visibility : nullptr,
                         config.visibility_depth_tolerance_m))
          visible_item.structural_segments.push_back(segment);
      visible_prepared.push_back(std::move(visible_item));
    }

    ceres::Problem problem;
    const double edge_scale = std::sqrt(config.edge_alignment_weight) /
                              (std::max(1.0, config.residual_cap_px) *
                               std::sqrt(static_cast<double>(total_points)));
    const double nid_scale = std::sqrt(
        config.normalized_information_distance_weight / prepared.size());
    for (std::size_t prepared_index = 0; prepared_index < prepared.size();
         ++prepared_index) {
      const auto &item = prepared[prepared_index];
      const auto &visible_item = visible_prepared[prepared_index];
      if (config.edge_alignment_weight > 0.0) {
        auto *edge_cost =
            new ceres::DynamicNumericDiffCostFunction<EdgeCost, ceres::CENTRAL>(
                new EdgeCost{visible_item.points, item.camera, item.distance,
                             config.residual_cap_px, edge_scale,
                             config.optimize_camera_intrinsics});
        edge_cost->AddParameterBlock(6);
        if (config.optimize_camera_intrinsics)
          edge_cost->AddParameterBlock(4);
        edge_cost->SetNumResiduals(
            static_cast<int>(visible_item.points.size()));
        if (config.optimize_camera_intrinsics)
          problem.AddResidualBlock(edge_cost, nullptr, params.data(),
                                   intrinsics.data());
        else
          problem.AddResidualBlock(edge_cost, nullptr, params.data());
      }
      if (config.normalized_information_distance_weight > 0.0) {
        auto *nid_cost =
            new ceres::DynamicNumericDiffCostFunction<NidCost, ceres::CENTRAL>(
                new NidCost{visible_item.geometry, item.camera,
                            item.camera_feature, config.nid_histogram_bins, nid_scale,
                            config.optimize_camera_intrinsics});
        nid_cost->AddParameterBlock(6);
        if (config.optimize_camera_intrinsics)
          nid_cost->AddParameterBlock(4);
        nid_cost->SetNumResiduals(1);
        if (config.optimize_camera_intrinsics)
          problem.AddResidualBlock(nid_cost, nullptr, params.data(),
                                   intrinsics.data());
        else
          problem.AddResidualBlock(nid_cost, nullptr, params.data());
      }
      if (config.structural_line_weight > 0.0 &&
          !visible_item.structural_segments.empty()) {
        auto *line_cost =
            new ceres::DynamicNumericDiffCostFunction<StructuralLineCost,
                                                      ceres::CENTRAL>(
                new StructuralLineCost{
                    visible_item.structural_segments, item.camera,
                    item.structural_image, config,
                    std::sqrt(config.structural_line_weight / prepared.size()),
                    config.optimize_camera_intrinsics});
        line_cost->AddParameterBlock(6);
        if (config.optimize_camera_intrinsics)
          line_cost->AddParameterBlock(4);
        line_cost->SetNumResiduals(1);
        if (config.optimize_camera_intrinsics)
          problem.AddResidualBlock(line_cost, nullptr, params.data(),
                                   intrinsics.data());
        else
          problem.AddResidualBlock(line_cost, nullptr, params.data());
      }
    }
    if (config.camera_direction_prior_weight > 0.0 &&
        (config.expected_camera_forward_lidar.norm() > 1e-9 ||
         config.expected_camera_down_lidar.norm() > 1e-9)) {
      auto *direction_cost =
          new ceres::DynamicNumericDiffCostFunction<DirectionPriorCost,
                                                    ceres::CENTRAL>(
              new DirectionPriorCost{
                  config.expected_camera_forward_lidar.norm() > 1e-9
                      ? config.expected_camera_forward_lidar.normalized()
                      : Eigen::Vector3d::Zero(),
                  config.expected_camera_down_lidar.norm() > 1e-9
                      ? config.expected_camera_down_lidar.normalized()
                      : Eigen::Vector3d::Zero(),
                  std::sqrt(0.5 * config.camera_direction_prior_weight)});
      direction_cost->AddParameterBlock(6);
      direction_cost->SetNumResiduals(6);
      problem.AddResidualBlock(direction_cost, nullptr, params.data());
    }
    const Parameters refinement_prior = params;
    auto *prior_cost = new ceres::AutoDiffCostFunction<PriorCost, 6, 6>(
        new PriorCost{refinement_prior, config.rotation_prior_sigma_rad,
                      config.translation_prior_sigma_m, config.prior_weight,
                      !config.use_camera_center_prior});
    problem.AddResidualBlock(prior_cost, nullptr, params.data());
    if (config.use_camera_center_prior) {
      auto *center_cost =
          new ceres::DynamicNumericDiffCostFunction<CameraCenterPriorCost,
                                                    ceres::CENTRAL>(
              new CameraCenterPriorCost{
                  config.expected_camera_center_lidar,
                  std::sqrt(config.camera_center_prior_weight) /
                      config.camera_center_prior_sigma_m});
      center_cost->AddParameterBlock(6);
      center_cost->SetNumResiduals(3);
      problem.AddResidualBlock(center_cost, nullptr, params.data());
    }
    for (int i = 0; i < 6; ++i) {
      const double bound = i < 3 ? config.rotation_search_bound_rad
                                 : config.translation_search_bound_m;
      const double bound_center =
          i >= 3 && config.use_camera_center_prior ? params[i]
                                                   : refinement_prior[i];
      problem.SetParameterLowerBound(params.data(), i, bound_center - bound);
      problem.SetParameterUpperBound(params.data(), i, bound_center + bound);
    }

    Intrinsics lower = intrinsic_prior, upper = intrinsic_prior;
    if (config.optimize_camera_intrinsics) {
      const double focal_sigma =
          std::max(0.05, config.focal_length_relative_bound * 0.5);
      const double cx_sigma =
          std::max(1.0, observations.front().camera.width *
                            config.principal_point_bound_ratio * 0.5);
      const double cy_sigma =
          std::max(1.0, observations.front().camera.height *
                            config.principal_point_bound_ratio * 0.5);
      auto *intrinsic_prior_cost =
          new ceres::AutoDiffCostFunction<IntrinsicPriorCost, 4, 4>(
              new IntrinsicPriorCost{intrinsic_prior, focal_sigma, cx_sigma,
                                     cy_sigma, config.intrinsic_prior_weight});
      problem.AddResidualBlock(intrinsic_prior_cost, nullptr,
                               intrinsics.data());
      lower = {intrinsic_prior[0] * (1.0 - config.focal_length_relative_bound),
               intrinsic_prior[1] * (1.0 - config.focal_length_relative_bound),
               intrinsic_prior[2] - observations.front().camera.width *
                                        config.principal_point_bound_ratio,
               intrinsic_prior[3] - observations.front().camera.height *
                                        config.principal_point_bound_ratio};
      upper = {intrinsic_prior[0] * (1.0 + config.focal_length_relative_bound),
               intrinsic_prior[1] * (1.0 + config.focal_length_relative_bound),
               intrinsic_prior[2] + observations.front().camera.width *
                                        config.principal_point_bound_ratio,
               intrinsic_prior[3] + observations.front().camera.height *
                                        config.principal_point_bound_ratio};
      for (int i = 0; i < 4; ++i) {
        problem.SetParameterLowerBound(intrinsics.data(), i, lower[i]);
        problem.SetParameterUpperBound(intrinsics.data(), i, upper[i]);
      }
    }

    ceres::Solver::Options options;
    options.max_num_iterations = config.maximum_solver_iterations;
    options.linear_solver_type = ceres::DENSE_QR;
    options.num_threads = 1;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    const Intrinsics *active_final_intrinsics =
        config.optimize_camera_intrinsics ? &intrinsics : nullptr;
    const Evaluation final_edge =
        evaluate_edge_all(params, active_final_intrinsics);
    const NidEvaluation final_nid =
        evaluate_nid_all(params, active_final_intrinsics);
    const Evaluation final_line =
        evaluate_lines_all(params, active_final_intrinsics);
    const Transform candidate = fromParameters(params);
    result.candidate_t_camera_lidar = candidate;
    const CameraModel candidate_camera =
        withIntrinsics(observations.front().camera, intrinsics);
    result.candidate_camera = candidate_camera;
    PoseError prior_update =
        calculatePoseError(candidate, fromParameters(refinement_prior));
    if (config.use_camera_center_prior) {
      const Eigen::Vector3d candidate_center =
          -candidate.rotation.transpose() * candidate.translation_m;
      prior_update.translation_m =
          (candidate_center - config.expected_camera_center_lidar).norm();
    }
    result.metrics.projected_edge_points = final_edge.projected;
    result.metrics.projected_ratio = final_edge.visible > 0
                                         ? static_cast<double>(
                                               final_edge.projected) /
                                               final_edge.visible
                                         : 0.0;
    result.metrics.nid_projected_points = final_nid.projected;
    result.metrics.visible_edge_points = final_edge.visible;
    result.metrics.occluded_edge_points = final_edge.occluded;
    result.metrics.structural_projected_points = final_line.visible;
    result.metrics.final_horizontal_structural_objective =
        std::isfinite(final_line.horizontal_normalized_squared)
            ? final_line.horizontal_normalized_squared
            : -1.0;
    result.metrics.final_vertical_structural_objective =
        std::isfinite(final_line.vertical_normalized_squared)
            ? final_line.vertical_normalized_squared
            : -1.0;
    result.metrics.final_mean_edge_distance_px = final_edge.mean;
    result.metrics.final_nid = final_nid.score;
    result.metrics.final_composite_objective =
        compositeObjective(final_edge, final_nid, final_line, config,
                           directionPriorObjective(params, config));
    result.metrics.objective_improvement_ratio =
        result.metrics.initial_composite_objective > 0.0
            ? (result.metrics.initial_composite_objective -
               result.metrics.final_composite_objective) /
                  result.metrics.initial_composite_objective
            : 0.0;
    result.metrics.nid_improvement_ratio =
        initial_nid.score > 0.0
            ? (initial_nid.score - final_nid.score) / initial_nid.score
            : 0.0;
    result.metrics.solver_iterations = summary.iterations.size();
    result.solver_summary = summary.BriefReport();

    bool intrinsic_at_bound = false;
    if (config.optimize_camera_intrinsics)
      for (int i = 0; i < 4; ++i) {
        const double tolerance = (upper[i] - lower[i]) * 1e-4;
        intrinsic_at_bound = intrinsic_at_bound ||
                             intrinsics[i] <= lower[i] + tolerance ||
                             intrinsics[i] >= upper[i] - tolerance;
      }

    const std::size_t minimum_nid_projection =
        config.minimum_nid_projected_points * observations.size();
    if (!summary.IsSolutionUsable())
      result.reason_code = "OPTIMIZER_FAILED";
    else if (summary.termination_type == ceres::NO_CONVERGENCE)
      result.reason_code = "OPTIMIZER_NOT_CONVERGED";
    else if (config.normalized_information_distance_weight > 0.0 &&
             final_nid.projected < minimum_nid_projection)
      result.reason_code = "NID_OVERLAP_INSUFFICIENT";
    else if (config.structural_line_weight > 0.0 &&
             final_line.visible <
                 config.minimum_projected_structural_segments)
      result.reason_code = "STRUCTURAL_OVERLAP_INSUFFICIENT";
    else if (result.metrics.multistart_objective_margin <
             config.minimum_multistart_objective_margin)
      result.reason_code = "MULTISTART_AMBIGUOUS";
    else if (result.metrics.projected_ratio < config.minimum_projected_ratio)
      result.reason_code = "OVERLAP_INSUFFICIENT";
    else if (final_edge.mean > config.maximum_mean_edge_distance_px)
      result.reason_code = "EDGE_ALIGNMENT_POOR";
    else if (result.metrics.objective_improvement_ratio <
             config.minimum_objective_improvement_ratio)
      result.reason_code = "OBJECTIVE_IMPROVEMENT_INSUFFICIENT";
    else if (result.metrics.nid_improvement_ratio <
             config.minimum_nid_improvement_ratio)
      result.reason_code = "NID_IMPROVEMENT_INSUFFICIENT";
    else if (intrinsic_at_bound)
      result.reason_code = "INTRINSIC_BOUND_REACHED";
    else if (prior_update.rotation_deg >
                 config.maximum_rotation_update_rad * kRadToDeg ||
             prior_update.translation_m > config.maximum_translation_update_m)
      result.reason_code = "PRIOR_DEVIATION_EXCESSIVE";
    else {
      result.estimated_t_camera_lidar = candidate;
      result.estimated_camera = candidate_camera;
      result.success = true;
      result.reason_code = "PASS";
    }
  } catch (const std::exception &e) {
    result.reason_code = std::string("INPUT_ERROR: ") + e.what();
  }
  result.metrics.runtime_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - started)
                                  .count();
  return finish();
}

PoseError calculatePoseError(const Transform &estimated,
                             const Transform &ground_truth) {
  Eigen::Matrix3d delta =
      estimated.rotation * ground_truth.rotation.transpose();
  Eigen::AngleAxisd aa(delta);
  return {(estimated.translation_m - ground_truth.translation_m).norm(),
          std::abs(aa.angle()) * kRadToDeg};
}
} // namespace auto_calib

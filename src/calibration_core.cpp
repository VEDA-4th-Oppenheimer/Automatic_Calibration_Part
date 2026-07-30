#include "auto_calib/calibration_core.hpp"
#include <algorithm>
#include <array>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace auto_calib {
namespace {
constexpr double kRadToDeg = 57.295779513082320876;
using Parameters = std::array<double, 6>;
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
                        const CameraModel &camera, const cv::Mat &distance) {
  double in[3] = {point.x(), point.y(), point.z()}, out[3];
  ceres::AngleAxisRotatePoint(p, in, out);
  out[0] += p[3];
  out[1] += p[4];
  out[2] += p[5];
  if (out[2] <= 0.05)
    return 50.0;
  double u = camera.k(0, 0) * out[0] / out[2] + camera.k(0, 2),
         v = camera.k(1, 1) * out[1] / out[2] + camera.k(1, 2);
  if (u < 0 || v < 0 || u >= distance.cols - 1 || v >= distance.rows - 1) {
    double du = u < 0 ? -u
                      : (u >= distance.cols - 1 ? u - (distance.cols - 2) : 0),
           dv = v < 0 ? -v
                      : (v >= distance.rows - 1 ? v - (distance.rows - 2) : 0);
    return 30.0 + std::hypot(du, dv);
  }
  return bilinear(distance, u, v);
}
struct EdgeCost {
  const std::vector<Eigen::Vector3d> &points;
  const CameraModel &camera;
  const cv::Mat &distance;
  double residual_cap_px, scale;
  bool operator()(double const *const *blocks, double *residuals) const {
    for (std::size_t i = 0; i < points.size(); ++i)
      residuals[i] = scale * std::min(residual_cap_px,
                                      residualForPoint(blocks[0], points[i],
                                                       camera, distance));
    return true;
  }
};
struct PriorCost {
  Parameters prior;
  double rs, ts, weight;
  template <typename T> bool operator()(const T *p, T *residuals) const {
    for (int i = 0; i < 3; ++i)
      residuals[i] = T(weight) * (p[i] - T(prior[i])) / T(rs);
    for (int i = 3; i < 6; ++i)
      residuals[i] = T(weight) * (p[i] - T(prior[i])) / T(ts);
    return true;
  }
};
struct Evaluation {
  double mean = std::numeric_limits<double>::infinity();
  std::size_t projected = 0;
};
Evaluation evaluate(const Parameters &p,
                    const std::vector<Eigen::Vector3d> &points,
                    const CameraModel &camera, const cv::Mat &distance) {
  Evaluation e;
  if (points.empty())
    return e;
  double sum = 0;
  for (const auto &point : points) {
    double r = residualForPoint(p.data(), point, camera, distance);
    sum += r;
    if (r < 30)
      e.projected++;
  }
  e.mean = sum / points.size();
  return e;
}
Parameters coarseSearch(Parameters p, const Parameters &prior,
                        const std::vector<Eigen::Vector3d> &points,
                        const CameraModel &camera, const cv::Mat &distance,
                        const CalibrationConfig &config) {
  auto best = evaluate(p, points, camera, distance).mean;
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
        double cost = evaluate(candidate, points, camera, distance).mean;
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

CalibrationResult calibrateExtrinsic(const cv::Mat &bgr,
                                     const CameraModel &camera,
                                     const Scan &scan,
                                     const Transform &mechanical_prior,
                                     const CalibrationConfig &config) {
  auto started = std::chrono::steady_clock::now();
  CalibrationResult result;
  result.estimated_t_camera_lidar = mechanical_prior;
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
    auto initial = evaluate(params, points, camera, distance);
    result.metrics.initial_mean_edge_distance_px = initial.mean;
    params = coarseSearch(params, prior, points, camera, distance, config);
    ceres::Problem problem;
    auto *edge_cost =
        new ceres::DynamicNumericDiffCostFunction<EdgeCost, ceres::CENTRAL>(
            new EdgeCost{points, camera, distance, config.residual_cap_px,
                         1.0 / std::sqrt(static_cast<double>(points.size()))});
    edge_cost->AddParameterBlock(6);
    edge_cost->SetNumResiduals(static_cast<int>(points.size()));
    problem.AddResidualBlock(edge_cost, nullptr, params.data());
    auto *prior_cost = new ceres::AutoDiffCostFunction<PriorCost, 6, 6>(
        new PriorCost{prior, config.rotation_prior_sigma_rad,
                      config.translation_prior_sigma_m, config.prior_weight});
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
    auto final = evaluate(params, points, camera, distance);
    const Transform candidate = fromParameters(params);
    result.metrics.projected_edge_points = final.projected;
    result.metrics.projected_ratio = double(final.projected) / points.size();
    result.metrics.final_mean_edge_distance_px = final.mean;
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
    else if (final.mean > initial.mean * 1.05)
      result.reason_code = "OBJECTIVE_NOT_IMPROVED";
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
  auto finish = [&]() {
    result.metrics.runtime_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - started)
                                    .count();
    return result;
  };
  struct Prepared {
    CameraModel camera;
    cv::Mat distance;
    std::vector<Eigen::Vector3d> points;
  };
  try {
    if (observations.empty()) {
      result.reason_code = "OBSERVATIONS_EMPTY";
      return finish();
    }
    std::vector<Prepared> prepared;
    prepared.reserve(observations.size());
    std::size_t total_points = 0;
    for (const auto &observation : observations) {
      if (observation.camera.k(0, 0) <= 0 || observation.camera.k(1, 1) <= 0) {
        result.reason_code = "INVALID_CAMERA_INTRINSIC";
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
      result.metrics.camera_edge_pixels += camera_edges;
      result.metrics.lidar_edge_points += points.size();
      total_points += points.size();
      prepared.push_back(
          {observation.camera, std::move(distance), std::move(points)});
    }
    auto evaluate_all = [&](const Parameters &parameters) {
      Evaluation aggregate;
      aggregate.mean = 0.0;
      aggregate.projected = 0;
      for (const auto &item : prepared) {
        Evaluation current =
            evaluate(parameters, item.points, item.camera, item.distance);
        aggregate.mean += current.mean * item.points.size();
        aggregate.projected += current.projected;
      }
      aggregate.mean /= static_cast<double>(total_points);
      return aggregate;
    };
    Parameters prior = toParameters(mechanical_prior), params = prior;
    const Evaluation initial = evaluate_all(params);
    result.metrics.initial_mean_edge_distance_px = initial.mean;

    ceres::Problem problem;
    const double scale = 1.0 / std::sqrt(static_cast<double>(total_points));
    for (const auto &item : prepared) {
      auto *edge_cost =
          new ceres::DynamicNumericDiffCostFunction<EdgeCost, ceres::CENTRAL>(
              new EdgeCost{item.points, item.camera, item.distance,
                           config.residual_cap_px, scale});
      edge_cost->AddParameterBlock(6);
      edge_cost->SetNumResiduals(static_cast<int>(item.points.size()));
      problem.AddResidualBlock(edge_cost, nullptr, params.data());
    }
    auto *prior_cost = new ceres::AutoDiffCostFunction<PriorCost, 6, 6>(
        new PriorCost{prior, config.rotation_prior_sigma_rad,
                      config.translation_prior_sigma_m, config.prior_weight});
    problem.AddResidualBlock(prior_cost, nullptr, params.data());
    for (int i = 0; i < 6; ++i) {
      const double bound = i < 3 ? config.rotation_search_bound_rad
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

    const Evaluation final = evaluate_all(params);
    const Transform candidate = fromParameters(params);
    const PoseError prior_update =
        calculatePoseError(candidate, mechanical_prior);
    result.metrics.projected_edge_points = final.projected;
    result.metrics.projected_ratio =
        static_cast<double>(final.projected) / total_points;
    result.metrics.final_mean_edge_distance_px = final.mean;
    result.metrics.solver_iterations = summary.iterations.size();
    result.solver_summary = summary.BriefReport();
    if (!summary.IsSolutionUsable())
      result.reason_code = "OPTIMIZER_FAILED";
    else if (summary.termination_type == ceres::NO_CONVERGENCE)
      result.reason_code = "OPTIMIZER_NOT_CONVERGED";
    else if (result.metrics.projected_ratio < config.minimum_projected_ratio)
      result.reason_code = "OVERLAP_INSUFFICIENT";
    else if (final.mean > config.maximum_mean_edge_distance_px)
      result.reason_code = "EDGE_ALIGNMENT_POOR";
    else if (final.mean > initial.mean * 1.05)
      result.reason_code = "OBJECTIVE_NOT_IMPROVED";
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

PoseError calculatePoseError(const Transform &estimated,
                             const Transform &ground_truth) {
  Eigen::Matrix3d delta =
      estimated.rotation * ground_truth.rotation.transpose();
  Eigen::AngleAxisd aa(delta);
  return {(estimated.translation_m - ground_truth.translation_m).norm(),
          std::abs(aa.angle()) * kRadToDeg};
}
} // namespace auto_calib

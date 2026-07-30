#pragma once
#include "auto_calib/synthetic_lidar.hpp"
#include <cstddef>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace auto_calib {
struct CalibrationConfig {
  double lidar_edge_absolute_threshold_m = 0.08;
  double lidar_edge_relative_threshold = 0.03;
  std::size_t minimum_lidar_edge_points = 50;
  std::size_t minimum_camera_edge_pixels = 100;
  double minimum_projected_ratio = 0.20;
  double maximum_mean_edge_distance_px = 40.0;
  double residual_cap_px = 20.0;
  int canny_low_threshold = 50, canny_high_threshold = 150;
  int coarse_rounds = 0;
  double coarse_rotation_step_rad = 0.0175;
  double coarse_translation_step_m = 0.015;
  int maximum_solver_iterations = 100;
  double rotation_search_bound_rad = 0.0873;
  double translation_search_bound_m = 0.10;
  double rotation_prior_sigma_rad = 0.05;
  double translation_prior_sigma_m = 0.02;
  double prior_weight = 1.0;
  double maximum_rotation_update_rad = 0.0873;
  double maximum_translation_update_m = 0.08;
};
struct CalibrationMetrics {
  std::size_t camera_edge_pixels = 0, lidar_edge_points = 0,
              projected_edge_points = 0;
  double projected_ratio = 0.0, initial_mean_edge_distance_px = 0.0,
         final_mean_edge_distance_px = 0.0;
  int solver_iterations = 0;
  double runtime_ms = 0.0;
};
struct CalibrationObservation {
  cv::Mat bgr;
  CameraModel camera;
  Scan scan;
};
struct CalibrationResult {
  bool success = false;
  std::string reason_code;
  Transform estimated_t_camera_lidar;
  CalibrationMetrics metrics;
  std::string solver_summary;
};
struct PoseError {
  double translation_m = 0.0, rotation_deg = 0.0;
};

cv::Mat buildCameraEdgeDistanceTransform(const cv::Mat &bgr,
                                         const CalibrationConfig &config,
                                         std::size_t *edge_count = nullptr);
std::vector<Eigen::Vector3d>
extractLidarEdgePoints(const Scan &scan, const CalibrationConfig &config);
CalibrationResult calibrateExtrinsic(const cv::Mat &bgr,
                                     const CameraModel &camera,
                                     const Scan &scan,
                                     const Transform &mechanical_prior,
                                     const CalibrationConfig &config = {});
CalibrationResult calibrateExtrinsicMultiScene(
    const std::vector<CalibrationObservation> &observations,
    const Transform &mechanical_prior, const CalibrationConfig &config = {});
PoseError calculatePoseError(const Transform &estimated,
                             const Transform &ground_truth);
} // namespace auto_calib

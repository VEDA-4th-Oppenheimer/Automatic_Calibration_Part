#pragma once

#include "auto_calib/panorama_raster_builder.hpp"
#include "auto_calib/synthetic_lidar.hpp"

#include <Eigen/Core>
#include <filesystem>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace auto_calib {

struct HybridOrientationProposal {
  int rank = 0;
  double yaw_deg = 0.0;
  double down_deg = 0.0;
  double roll_deg = 0.0;
  double raw_score = 0.0;
  double basin_score = 0.0;
  double confidence = 0.0;
  double yaw_sigma_deg = 0.0;
  double down_sigma_deg = 15.0;
  double roll_sigma_deg = 10.0;
  double search_radius_deg = 10.0;
  std::string evidence = "ANGULAR_SIGNATURE,MANHATTAN_SVD";
};

struct HybridAnalyzerOptions {
  int top_k = 3;
  int signature_bins = 64;
  double yaw_step_deg = 1.0;
  double nms_separation_deg = 30.0;
  double basin_radius_deg = 5.0;
  double minimum_coverage = 0.20;
  int minimum_camera_edge_pixels = 100;
  double minimum_peak_zscore = 0.25;
};

struct HybridAnalyzerResult {
  std::string schema_version = "3.0";
  std::string mode = "hybrid";
  std::string status = "INVALID_INPUT";
  bool fallback_required = true;
  std::string fallback_reason;
  double coverage = 0.0;
  int line_count = 0;
  int normal_count = 0;
  int gravity_vanishing_index = -1;
  std::vector<Eigen::Vector3d> image_vanishing_directions;
  std::vector<int> image_vanishing_line_counts;
  std::vector<double> image_vanishing_support_weights;
  int evaluated_signature_yaws = 0;
  int evaluated_elevation_candidates = 0;
  int perspective_remaps = 0;
  int expensive_projection_evaluations = 0;
  double image_feature_ms = 0.0;
  double lidar_feature_ms = 0.0;
  double signature_search_ms = 0.0;
  double structural_hypothesis_ms = 0.0;
  double runtime_ms = 0.0;
  std::vector<double> camera_signature;
  std::vector<double> lidar_signature;
  std::vector<double> camera_elevation_signature;
  std::vector<double> down_degrees;
  std::vector<std::vector<double>> lidar_elevation_signatures;
  std::vector<std::vector<double>> down_score_curves;
  std::vector<double> yaw_degrees;
  std::vector<double> raw_scores;
  std::vector<double> basin_scores;
  std::vector<HybridOrientationProposal> proposals;
  PanoramaRaster raster;
};

// Angularly samples a circular LiDAR signature through the camera rays and
// returns one normalized correlation score per yaw. This is the V3 global
// search: no perspective image or 3D projection is created here.
std::vector<double>
scoreAngularSignatures(const std::vector<double> &camera_signature,
                       const std::vector<double> &lidar_signature,
                       const Eigen::Matrix3d &camera_k, int image_width,
                       double pan_min_rad, double pan_max_rad,
                       const std::vector<double> &yaw_degrees);

HybridAnalyzerResult
analyzeHybridOrientation(const std::filesystem::path &json_path,
                         const cv::Mat &camera_bgr, const CameraModel &camera,
                         const HybridAnalyzerOptions &options = {});

bool writeHybridAnalyzerArtifacts(const HybridAnalyzerResult &result,
                                  const std::filesystem::path &directory);

} // namespace auto_calib

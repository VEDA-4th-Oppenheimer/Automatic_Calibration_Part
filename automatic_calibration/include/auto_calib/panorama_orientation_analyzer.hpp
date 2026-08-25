#pragma once

// T2 Panorama Orientation Analyzer, remediation schema 2.0.
//
// Replaces the deprecated 400x1 1D resample (which distorted the ~60 deg
// camera FOV across 360 deg) with:
//   * seam-safe masked multi-channel panorama rasters,
//   * 320x180 perspective remapping (cv::remap) of the virtual LiDAR edge
//     view over a 1D yaw sweep (10 deg) x down-angle candidates,
//   * bidirectional Chamfer overlap against the camera edge distance
//     transform,
//   * circular/geodesic NMS (>= 30 deg) for distinct Top-3 basins,
//   * Peak-to-Sidelobe Ratio (>= 1.15) statistical confidence gate with
//     fail-safe INSUFFICIENT_FEATURES fallback.

#include "auto_calib/panorama_raster_builder.hpp"
#include "auto_calib/perspective_remapper.hpp"
#include "auto_calib/synthetic_lidar.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace auto_calib {

struct PanoramaProposal {
  int rank = 0;
  double yaw_deg = 0.0;  // [-180, 180)
  double down_deg = 0.0; // [0, 90]
  double roll_deg = 0.0;
  double raw_score = 0.0;
  double normalized_score = 0.0;
  double confidence = 0.0;
  double search_radius_deg = 10.0;
  std::string evidence = "PERSPECTIVE_CHAMFER,PANORAMA_REMAP";
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(); // R_camera_lidar
};

struct PanoramaAnalyzerOptions {
  int top_k = 3;
  double minimum_coverage = 0.20;
  double minimum_pslr = 1.15;
  double yaw_step_deg = 10.0;
  std::vector<double> down_candidates_deg = {0.0, 15.0, 30.0, 45.0, 60.0,
                                             75.0};
  double nms_separation_deg = 30.0;
  double chamfer_sigma_px = 5.0;
  cv::Size perspective_size = {320, 180};
  std::filesystem::path output_dir;
};

struct PanoramaAnalyzerResult {
  std::string schema_version = "2.0";
  std::string mode = "panorama";
  std::string status = "INVALID_INPUT";
  int rows = 0, columns = 0;
  double coverage = 0.0;
  bool fallback_required = true;
  std::string fallback_reason;
  std::vector<PanoramaProposal> proposals;
  std::vector<double> score_curve; // best-down score per yaw bin
  cv::Mat score_grid;              // CV_32F [yaw_bin][down_bin]
  cv::Mat range_mm, valid, range_edge, normal_edge, plane_intersection;
  double peak_to_sidelobe_ratio = 0.0;
  int yaw_bins = 0, down_bins = 0;
  int evaluated_candidates = 0;
  double runtime_ms = 0.0;
};

PanoramaAnalyzerResult analyzePanorama(const std::filesystem::path &json_path,
                                       const cv::Mat &camera_bgr,
                                       const CameraModel &camera,
                                       const PanoramaAnalyzerOptions &options =
                                           {});

bool writePanoramaAnalyzerArtifacts(const PanoramaAnalyzerResult &result,
                                    const std::filesystem::path &directory);

double normalizeYawDeg(double yaw);
double circularDistanceDeg(double a, double b);

// Geodesic NMS over (yaw, down) peaks: greedy by score, accepting a peak
// only when its composed rotation is >= min_separation_deg from every
// accepted peak. Exposed for unit testing.
std::vector<int> selectDistinctPeaks(
    const std::vector<double> &yaws_deg, const std::vector<double> &downs_deg,
    const std::vector<double> &scores, double min_separation_deg, int top_k);

} // namespace auto_calib

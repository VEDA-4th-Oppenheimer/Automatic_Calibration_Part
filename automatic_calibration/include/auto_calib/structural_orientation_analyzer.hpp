#pragma once

// T1 Structural Orientation Analyzer, remediation schema 2.0.
//
// Replaces the removed 2D-3D subtraction and 0-degree prior bias with:
//   * projective vanishing direction estimation (K^-1 back-projection + SVD),
//   * LiDAR oriented-normal Manhattan frame estimation,
//   * Wahba SVD 3-DoF rotation solving over right-handed permutations,
//   * projected line-to-edge Chamfer residual scoring,
//   * SO(3) geodesic non-maximum suppression (>= 30 deg) for Top-3 proposals.

#include "auto_calib/image_vanishing_estimator.hpp"
#include "auto_calib/synthetic_lidar.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace auto_calib {

struct StructuralOrientationProposal {
  int rank = 0;
  double yaw_deg = 0;   // [-180, 180) rotation about lidar +y (down)
  double down_deg = 0;  // [0, 90] camera optical axis downward pitch
  double roll_deg = 0;  // [-30, 30] optical roll about the forward axis
  double raw_score = 0; // geometric chamfer agreement in (0, 1]
  double normalized_score = 0;
  double confidence = 0;
  double search_radius_deg = 10.0;
  std::vector<std::string> evidence;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(); // R_camera_lidar
};

struct StructuralAnalyzerInput {
  cv::Mat image;                      // grayscale or BGR (undistorted)
  std::vector<Point> organized_lidar; // row-major organized grid
  std::size_t rows = 0, columns = 0;
  CameraModel camera;
  std::size_t top_k = 3;
  double chamfer_sigma_px = 6.0;
  // Expected lateral sensor offset used to normalize projection parallax
  // when the extrinsic translation is unknown at proposal time.
  double translation_hint_m = 0.30;
  // Optional pre-detected segments (bypasses LSD; used by regression tests
  // and by callers with an external line detector).
  std::vector<LineSegment2D> segment_override;
};

struct StructuralAnalyzerResult {
  std::string schema_version = "2.0";
  std::string mode = "structural";
  std::string status = "INVALID_INPUT";
  std::size_t input_rows = 0, input_columns = 0, line_count = 0;
  std::size_t normal_count = 0;
  std::size_t camera_direction_count = 0;
  std::size_t candidate_rotation_count = 0;
  std::size_t projected_edge_points = 0;
  bool lidar_wall2_synthetic = false;
  std::vector<StructuralOrientationProposal> proposals;
  bool fallback_required = true;
  std::string fallback_reason;
  double runtime_ms = 0;
  double peak_memory_bytes = 0;
};

StructuralAnalyzerResult analyzeStructuralOrientation(
    const StructuralAnalyzerInput &input);

bool writeStructuralAnalyzerArtifacts(const StructuralAnalyzerResult &result,
                                      const std::string &directory);

// Geodesic distance on SO(3): acos((tr(R1 R2^T) - 1) / 2), degrees.
double geodesicDistanceDeg(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b);

// Decompose R_camera_lidar = Rz(roll) Rx(down) Ry(yaw).
void decomposeRollDownYaw(const Eigen::Matrix3d &rotation, double *roll_deg,
                          double *down_deg, double *yaw_deg);

} // namespace auto_calib

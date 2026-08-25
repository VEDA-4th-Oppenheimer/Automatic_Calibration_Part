#pragma once

// LiDAR oriented surface normal & Manhattan frame estimator
// (remediation Task 1.2).
//
// Local PCA normals are computed on the organized grid, signed to face the
// sensor (n . p < 0), and clustered into a gravity axis plus orthogonal wall
// normals forming a right-handed Manhattan triad (v1, v2, v3).

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "auto_calib/synthetic_lidar.hpp"

namespace auto_calib {

struct LidarManhattanFrame {
  bool valid = false;
  Eigen::Vector3d v1 = Eigen::Vector3d::Zero(); // gravity / vertical axis
  Eigen::Vector3d v2 = Eigen::Vector3d::Zero(); // dominant wall normal
  Eigen::Vector3d v3 = Eigen::Vector3d::Zero(); // remaining horizontal axis
  int normal_count = 0;
  int vertical_inliers = 0;
  int wall_inliers = 0;
  bool wall2_synthetic = false; // v3 completed by orthogonality assumption
};

struct ManhattanEstimatorOptions {
  double vertical_threshold_deg = 30.0; // |angle(normal, vertical)|
  double azimuth_tolerance_deg = 15.0;  // wall normal cluster tolerance
  double max_curvature_ratio = 0.05;   // lambda_min / lambda_max ceiling
};

LidarManhattanFrame estimateManhattanFrame(
    const std::vector<Point> &organized_lidar, std::size_t rows,
    std::size_t columns, const ManhattanEstimatorOptions &options = {});

// Oriented PCA surface normal at a cell using its 3x3 neighborhood
// (sensor-facing sign: n . p < 0). Returns false for incomplete or
// non-planar neighborhoods.
bool orientedSurfaceNormal(const std::vector<Point> &organized_lidar,
                           std::size_t rows, std::size_t columns,
                           std::size_t row, std::size_t column,
                           double max_curvature_ratio,
                           Eigen::Vector3d *normal);

} // namespace auto_calib

#pragma once

// Image 3D vanishing direction estimator (remediation Task 1.1).
//
// Implements strict projective geometry: 2D LSD line segments are back
// projected through K^-1 into projection plane normals
//     n_i = K^T (p1 x p2) / ||K^T (p1 x p2)||,
// and 3D vanishing directions are extracted as the null space (smallest
// singular vector) of the weighted scatter M = sum_i w_i n_i n_i^T.

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <vector>

namespace auto_calib {

struct LineSegment2D {
  double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
  double length = 0;
};

struct VanishingDirection {
  Eigen::Vector3d direction = Eigen::Vector3d::Zero(); // unit, camera frame
  double support_weight = 0;                           // summed segment length
  int line_count = 0;
};

// LSD detection with length filtering (drops segments shorter than
// max(min_length_px, relative_fraction * min(image dims))).
std::vector<LineSegment2D> detectLineSegments(const cv::Mat &gray,
                                              double min_length_px,
                                              double relative_fraction = 0.04);

// Projection plane normal of a segment under intrinsics k:
// n = K^T (p1 x p2), normalized. Returns false for degenerate segments.
bool projectionPlaneNormal(const LineSegment2D &segment,
                           const Eigen::Matrix3d &k, Eigen::Vector3d *normal);

// Deterministic consensus clustering over pairwise intersection directions
// d_ij ∝ n_i x n_j followed by per-cluster SVD null-space extraction.
std::vector<VanishingDirection> estimateVanishingDirections(
    const std::vector<LineSegment2D> &segments, const Eigen::Matrix3d &k,
    std::size_t max_directions = 3, double angular_tolerance_deg = 10.0,
    std::size_t max_consensus_lines = 60);

} // namespace auto_calib

#include "auto_calib/lidar_manhattan_estimator.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace auto_calib {
namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

// Oriented PCA surface normal of the 3x3 neighborhood around (row, column).
// Returns false when the neighborhood is incomplete or degenerate.
bool orientedSurfaceNormal(const std::vector<Point> &grid,
                           std::size_t rows, std::size_t columns,
                           std::size_t row, std::size_t column,
                           double max_curvature_ratio,
                           Eigen::Vector3d *normal) {
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  int count = 0;
  for (std::size_t dr = 0; dr < 3; ++dr) {
    const std::size_t r = row + dr;
    if (r >= rows)
      return false;
    for (std::size_t dc = 0; dc < 3; ++dc) {
      const std::size_t c = column + dc;
      if (c >= columns)
        return false;
      const Point &p = grid[r * columns + c];
      if (!p.valid() || !p.xyz.allFinite())
        return false;
      centroid += p.xyz.cast<double>();
      ++count;
    }
  }
  if (count != 9)
    return false;
  centroid /= static_cast<double>(count);
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (std::size_t dr = 0; dr < 3; ++dr)
    for (std::size_t dc = 0; dc < 3; ++dc) {
      const Eigen::Vector3d d =
          grid[(row + dr) * columns + (column + dc)].xyz.cast<double>() -
          centroid;
      covariance += d * d.transpose();
    }
  covariance /= static_cast<double>(count);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success)
    return false;
  const double lambda_min = solver.eigenvalues()(0);
  const double lambda_max = solver.eigenvalues()(2);
  // Planarity gate: reject curved/corner cells whose smallest covariance
  // eigenvalue is not negligible against the dominant one.
  if (!std::isfinite(lambda_min) || lambda_max <= 1e-12 ||
      lambda_min > max_curvature_ratio * lambda_max)
    return false;
  Eigen::Vector3d n = solver.eigenvectors().col(0);
  if (!n.allFinite() || n.norm() < 1e-9)
    return false;
  n.normalize();
  // Sensor-facing sign convention: n . p < 0 with p measured from the sensor.
  const Eigen::Vector3d p =
      grid[(row + 1) * columns + (column + 1)].xyz.cast<double>();
  if (n.dot(p) > 0)
    n = -n;
  *normal = n;
  return true;
}

LidarManhattanFrame estimateManhattanFrame(
    const std::vector<Point> &organized_lidar, std::size_t rows,
    std::size_t columns, const ManhattanEstimatorOptions &options) {
  LidarManhattanFrame out;
  if (rows < 3 || columns < 3 ||
      organized_lidar.size() < rows * columns)
    return out;

  struct ClusterAccumulator {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    double weight = 0;
    int count = 0;
  };
  std::vector<ClusterAccumulator> wall_clusters;
  Eigen::Vector3d vertical_sum = Eigen::Vector3d::Zero();
  int vertical_count = 0;
  int normal_count = 0;

  const double cos_vertical =
      std::cos(options.vertical_threshold_deg * kPi / 180.0);
  const double azimuth_tol_rad = options.azimuth_tolerance_deg * kPi / 180.0;

  for (std::size_t r = 0; r + 2 < rows; ++r)
    for (std::size_t c = 0; c + 2 < columns; ++c) {
      Eigen::Vector3d n;
      if (!orientedSurfaceNormal(organized_lidar, rows, columns, r, c,
                                 options.max_curvature_ratio, &n))
        continue;
      ++normal_count;
      if (std::abs(n.y()) >= cos_vertical) {
        // Floor/ceiling normal: accumulate along a consistent orientation.
        vertical_sum += (n.y() < 0 ? n : -n);
        ++vertical_count;
        continue;
      }
      // Horizontal wall normal: greedy circular clustering on azimuth.
      const double azimuth = std::atan2(n.x(), n.z());
      bool merged = false;
      for (auto &cluster : wall_clusters) {
        const Eigen::Vector3d mean = cluster.sum / cluster.weight;
        const double mean_azimuth = std::atan2(mean.x(), mean.z());
        double delta = std::abs(azimuth - mean_azimuth);
        delta = std::min(delta, 2.0 * kPi - delta);
        if (delta < azimuth_tol_rad) {
          cluster.sum += n;
          cluster.weight += 1.0;
          ++cluster.count;
          merged = true;
          break;
        }
      }
      if (!merged)
        wall_clusters.push_back({n, 1.0, 1});
    }

  out.normal_count = normal_count;
  if (normal_count <= 0 || vertical_count == 0 || wall_clusters.empty())
    return out; // fewer than two independent Manhattan axes

  auto finalize_axis = [](const Eigen::Vector3d &sum, double weight,
                          Eigen::Vector3d *axis) {
    if (weight <= 0)
      return false;
    Eigen::Vector3d v = sum / weight;
    if (!v.allFinite() || v.norm() < 1e-6)
      return false;
    *axis = v.normalized();
    return true;
  };

  Eigen::Vector3d v1;
  if (!finalize_axis(vertical_sum, static_cast<double>(vertical_count), &v1))
    return out;

  auto by_weight = [](const ClusterAccumulator &a,
                      const ClusterAccumulator &b) { return a.count > b.count; };
  std::sort(wall_clusters.begin(), wall_clusters.end(), by_weight);

  // Dominant measured wall normal.
  bool have_v2 = false;
  Eigen::Vector3d v2 = Eigen::Vector3d::Zero();
  for (const auto &cluster : wall_clusters) {
    Eigen::Vector3d candidate;
    if (!finalize_axis(cluster.sum, cluster.weight, &candidate))
      continue;
    if (std::abs(candidate.dot(v1)) >
        std::sin(options.vertical_threshold_deg * kPi / 180.0))
      continue; // not a horizontal axis after averaging
    v2 = candidate;
    have_v2 = true;
    break;
  }
  if (!have_v2)
    return out;

  // Prefer a second measured wall cluster near-orthogonal to v2; otherwise
  // complete the triad through the Manhattan orthogonality assumption.
  bool synthetic = true;
  for (const auto &cluster : wall_clusters) {
    Eigen::Vector3d candidate;
    if (!finalize_axis(cluster.sum, cluster.weight, &candidate))
      continue;
    if (std::abs(candidate.dot(v1)) >
        std::sin(options.vertical_threshold_deg * kPi / 180.0))
      continue;
    if (std::abs(candidate.dot(v2)) >
        std::cos((90.0 - options.azimuth_tolerance_deg) * kPi / 180.0))
      continue; // same wall family as v2
    if (std::abs(candidate.dot(v2)) > 0.35)
      continue; // not orthogonal enough to form a Manhattan frame
    synthetic = false;
    break;
  }

  // Orthonormal right-handed triad (v1, v2, v3). Residual ± axis direction
  // ambiguity is resolved downstream by the signed-permutation enumeration.
  v2 = (v2 - v2.dot(v1) * v1).normalized();
  Eigen::Vector3d v3 = v1.cross(v2).normalized();

  out.v1 = v1;
  out.v2 = v2;
  out.v3 = v3;
  out.wall2_synthetic = synthetic;
  out.vertical_inliers = vertical_count;
  out.wall_inliers = wall_clusters.front().count;
  out.valid = true;
  return out;
}

} // namespace auto_calib

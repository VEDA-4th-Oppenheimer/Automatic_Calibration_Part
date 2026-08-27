#include "auto_calib/image_vanishing_estimator.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <utility>

namespace auto_calib {
namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

std::vector<LineSegment2D> detectLineSegments(const cv::Mat &gray,
                                              double min_length_px,
                                              double relative_fraction) {
  std::vector<LineSegment2D> out;
  if (gray.empty())
    return out;
  cv::Mat binary = gray;
  if (gray.channels() != 1)
    cv::cvtColor(gray, binary, cv::COLOR_BGR2GRAY);
  const auto detector = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
  std::vector<cv::Vec4f> lines;
  detector->detect(binary, lines);
  const double adaptive =
      std::max(min_length_px,
               relative_fraction * static_cast<double>(std::min(binary.cols,
                                                                binary.rows)));
  for (const auto &l : lines) {
    LineSegment2D s{l[0], l[1], l[2], l[3], 0};
    s.length = std::hypot(s.u2 - s.u1, s.v2 - s.v1);
    if (s.length < adaptive || !std::isfinite(s.length))
      continue;
    out.push_back(s);
  }
  std::sort(out.begin(), out.end(),
            [](const LineSegment2D &a, const LineSegment2D &b) {
              return a.length > b.length;
            });
  return out;
}

bool projectionPlaneNormal(const LineSegment2D &segment,
                           const Eigen::Matrix3d &k, Eigen::Vector3d *normal) {
  const Eigen::Vector3d p1(segment.u1, segment.v1, 1.0);
  const Eigen::Vector3d p2(segment.u2, segment.v2, 1.0);
  const Eigen::Vector3d line = p1.cross(p2);
  if (line.norm() < 1e-12)
    return false;
  const Eigen::Vector3d n = k.transpose() * line;
  const double norm = n.norm();
  if (!std::isfinite(norm) || norm < 1e-12)
    return false;
  *normal = n / norm;
  return true;
}

std::vector<VanishingDirection> estimateVanishingDirections(
    const std::vector<LineSegment2D> &segments, const Eigen::Matrix3d &k,
    std::size_t max_directions, double angular_tolerance_deg,
    std::size_t max_consensus_lines) {
  struct WeightedNormal {
    Eigen::Vector3d n;
    double weight;
  };
  std::vector<WeightedNormal> pool;
  pool.reserve(segments.size());
  for (const auto &segment : segments) {
    WeightedNormal wn;
    if (!projectionPlaneNormal(segment, k, &wn.n))
      continue;
    wn.weight = segment.length;
    pool.push_back(wn);
  }
  // Deterministic consensus: cap the pool to the longest segments so the
  // pairwise enumeration stays bounded on embedded targets.
  if (pool.size() > max_consensus_lines)
    pool.resize(max_consensus_lines);

  const double sin_tol = std::sin(angular_tolerance_deg * kPi / 180.0);
  auto cluster_support = [&](const Eigen::Vector3d &direction,
                             std::vector<int> *members) {
    double support = 0;
    int count = 0;
    for (std::size_t w = 0; w < pool.size(); ++w)
      if (std::abs(pool[w].n.dot(direction)) < sin_tol) {
        support += pool[w].weight;
        ++count;
        if (members)
          members->push_back(static_cast<int>(w));
      }
    return std::make_pair(support, count);
  };
  (void)cluster_support;

  // --- Exhaustive disjoint Manhattan-triad enumeration -----------------------
  // A bare pairwise consensus admits mixed directions when line families are
  // unbalanced. In a Manhattan world the dominant structure is an orthogonal
  // triad, so enumerate triads (d1 from pairwise intersections, d2 = d1 x n_k,
  // d3 = d1 x d2) and maximize the TOTAL support under disjoint membership.
  // The true triad absorbs every line; mixed triads lose members to
  // disjointness and score lower.
  auto members_of = [&](const Eigen::Vector3d &direction,
                        const std::vector<char> &taken,
                        std::vector<int> *members) {
    double support = 0;
    int count = 0;
    for (std::size_t w = 0; w < pool.size(); ++w) {
      if (taken[w])
        continue;
      if (std::abs(pool[w].n.dot(direction)) < sin_tol) {
        support += pool[w].weight;
        ++count;
        if (members)
          members->push_back(static_cast<int>(w));
      }
    }
    return std::make_pair(support, count);
  };

  double best_total = 0;
  Eigen::Vector3d best_d1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d best_d2 = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i + 1 < pool.size(); ++i)
    for (std::size_t j = i + 1; j < pool.size(); ++j) {
      const Eigen::Vector3d cross = pool[i].n.cross(pool[j].n);
      const double cross_norm = cross.norm();
      if (cross_norm < std::sin(15.0 * kPi / 180.0))
        continue;
      const Eigen::Vector3d d1 = cross / cross_norm;
      std::vector<char> taken(pool.size(), 0);
      const auto first = members_of(d1, taken, nullptr);
      if (first.second < 2)
        continue;
      for (std::size_t w = 0; w < pool.size(); ++w)
        if (std::abs(pool[w].n.dot(d1)) < sin_tol)
          taken[w] = 1;
      for (std::size_t k = 0; k < pool.size(); ++k) {
        Eigen::Vector3d d2 = d1.cross(pool[k].n);
        const double d2_norm = d2.norm();
        if (d2_norm < 1e-9)
          continue;
        d2 /= d2_norm;
        const auto second = members_of(d2, taken, nullptr);
        if (second.second < 2)
          continue;
        const Eigen::Vector3d d3 = d1.cross(d2);
        std::vector<char> taken2 = taken;
        for (std::size_t w = 0; w < pool.size(); ++w)
          if (std::abs(pool[w].n.dot(d2)) < sin_tol)
            taken2[w] = 1;
        const auto third = members_of(d3, taken2, nullptr);
        const double total = first.first + second.first + third.first;
        if (total > best_total) {
          best_total = total;
          best_d1 = d1;
          best_d2 = d2;
        }
      }
    }
  if (best_d1.norm() < 0.5 || best_total <= 0)
    return {};

  // SVD null-space refinement of each triad axis over its (disjoint) support,
  // re-enforcing exact orthogonality afterwards.
  std::vector<VanishingDirection> results;
  std::vector<char> taken(pool.size(), 0);
  Eigen::Vector3d axes[3] = {best_d1, best_d2,
                             best_d1.cross(best_d2).normalized()};
  for (int axis_index = 0; axis_index < 3; ++axis_index) {
    if (axis_index >= static_cast<int>(max_directions))
      break;
    std::vector<int> members;
    const auto support = members_of(axes[axis_index], taken, &members);
    VanishingDirection out;
    out.support_weight = support.first;
    out.line_count = support.second;
    if (members.size() >= 2) {
      Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
      for (const int idx : members)
        scatter += pool[idx].weight * pool[idx].n * pool[idx].n.transpose();
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(scatter, Eigen::ComputeFullU |
                                                         Eigen::ComputeFullV);
      Eigen::Vector3d refined = svd.matrixU().col(2);
      // Sign convention: point towards the observed scene half-space.
      Eigen::Vector3d mean_ray = Eigen::Vector3d::Zero();
      for (const auto &segment : segments) {
        const Eigen::Vector3d mid((segment.u1 + segment.u2) * 0.5,
                                  (segment.v1 + segment.v2) * 0.5, 1.0);
        mean_ray += k.inverse() * mid;
      }
      if (mean_ray.norm() > 1e-9 && refined.dot(mean_ray) < 0)
        refined = -refined;
      if (refined.norm() >= 0.5)
        axes[axis_index] = refined;
    }
    for (const int idx : members)
      taken[idx] = 1;
    out.direction = axes[axis_index];
    results.push_back(out);
  }
  // Re-orthogonalize the triad after per-axis refinement.
  if (results.size() >= 2) {
    results[1].direction =
        (results[1].direction -
         results[1].direction.dot(results[0].direction) *
             results[0].direction)
            .normalized();
    if (results.size() == 3)
      results[2].direction =
          results[0].direction.cross(results[1].direction).normalized();
  }
  return results;
}

} // namespace auto_calib

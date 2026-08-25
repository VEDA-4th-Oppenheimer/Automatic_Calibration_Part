#include "auto_calib/structural_orientation_analyzer.hpp"

#include "auto_calib/image_vanishing_estimator.hpp"
#include "auto_calib/lidar_manhattan_estimator.hpp"

#include <Eigen/SVD>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <iomanip>

namespace auto_calib {
namespace {
constexpr double kPi = 3.14159265358979323846;
double deg(double r) { return r * 180.0 / kPi; }
double rad(double d) { return d * kPi / 180.0; }
double circular(double a) { return std::remainder(a, 360.0); }

// Coarse ranking tolerance (px). Wide enough to absorb vanishing-direction
// noise before the local refinement stage tightens the estimate.
constexpr double kCoarseSigmaPx = 20.0;

// LiDAR structural edge points: depth discontinuities (curvature-gated)
// plus surface-normal creases on the organized grid. Convex-room junctions
// (wall-floor, wall-wall) are range-continuous creases invisible to depth
// differences, so the normal channel is essential. Circular column indexing
// keeps the 0/360 seam continuous.
std::vector<Eigen::Vector3d> lidarStructuralEdgePoints(
    const std::vector<Point> &grid, std::size_t rows, std::size_t columns) {
  std::vector<Eigen::Vector3d> out;
  if (rows < 3 || columns < 3 || grid.size() < rows * columns)
    return out;
  const auto range_at = [&](std::size_t r, std::size_t c) -> double {
    const Point &p = grid[r * columns + c];
    if (!p.valid())
      return 0.0;
    return p.range > 0 && std::isfinite(p.range)
               ? static_cast<double>(p.range)
               : p.xyz.norm();
  };
  // Pre-compute oriented surface normals where neighborhoods are planar.
  std::vector<Eigen::Vector3d> normals(rows * columns,
                                       Eigen::Vector3d::Zero());
  std::vector<char> has_normal(rows * columns, 0);
  for (std::size_t r = 0; r + 2 < rows; ++r)
    for (std::size_t c = 0; c + 2 < columns; ++c) {
      Eigen::Vector3d n;
      if (orientedSurfaceNormal(grid, rows, columns, r, c,
                                /*max_curvature_ratio=*/0.05, &n)) {
        normals[(r + 1) * columns + (c + 1)] = n;
        has_normal[(r + 1) * columns + (c + 1)] = 1;
      }
    }
  constexpr double kCurvatureThreshold = 0.06;
  constexpr double kCreaseCosine = std::cos(20.0 * kPi / 180.0);
  for (std::size_t r = 0; r < rows; ++r)
    for (std::size_t c = 0; c < columns; ++c) {
      const Point &p = grid[r * columns + c];
      if (!p.valid())
        continue;
      const double center = range_at(r, c);
      if (!(center > 0))
        continue;
      bool edge = false;
      // Depth channel: relative second difference cancels grazing gradients.
      const double left = range_at(r, (c + columns - 1) % columns);
      const double right = range_at(r, (c + 1) % columns);
      if (left > 0 && right > 0 &&
          std::abs(right - 2.0 * center + left) / center >=
              kCurvatureThreshold)
        edge = true;
      if (!edge && r > 0 && r + 1 < rows) {
        const double up = range_at(r - 1, c);
        const double down = range_at(r + 1, c);
        if (up > 0 && down > 0 &&
            std::abs(down - 2.0 * center + up) / center >=
                kCurvatureThreshold)
          edge = true;
      }
      // Normal channel: creases where the oriented normal turns sharply.
      // Junction cells themselves have mixed (rejected) neighborhoods, so
      // compare against the nearest valid normals within a small window.
      if (!edge) {
        const auto nearest_normal = [&](int r0, int c0, int dr,
                                        int dc) -> const Eigen::Vector3d * {
          for (int step = 1; step <= 4; ++step) {
            const int rr = r0 + step * dr;
            const int cc = c0 + step * dc;
            if (rr < 0 || rr >= static_cast<int>(rows) || cc < 0 ||
                cc >= static_cast<int>(columns))
              return nullptr;
            const std::size_t idx = static_cast<std::size_t>(rr) * columns +
                                    static_cast<std::size_t>(cc);
            if (has_normal[idx])
              return &normals[idx];
          }
          return nullptr;
        };
        const Eigen::Vector3d *pairs[2][2] = {
            {nearest_normal(static_cast<int>(r), static_cast<int>(c), 0, -1),
             nearest_normal(static_cast<int>(r), static_cast<int>(c), 0, 1)},
            {nearest_normal(static_cast<int>(r), static_cast<int>(c), -1, 0),
             nearest_normal(static_cast<int>(r), static_cast<int>(c), 1, 0)}};
        for (auto &pair : pairs) {
          const auto *a = pair[0];
          const auto *b = pair[1];
          if (!a || !b || a == b)
            continue;
          if (a->dot(*b) < kCreaseCosine) {
            edge = true;
            break;
          }
        }
      }
      if (edge)
        out.push_back(p.xyz.cast<double>());
    }
  return out;
}

// Wahba's problem: maximize sum b^T R a with source a (lidar axes) and target
// b (camera axes). H = sum a b^T; SVD H = U S V^T;
// R = V diag(1, 1, det(V U^T)) U^T.
Eigen::Matrix3d
rotationFromCorrespondences(const Eigen::Vector3d *sources,
                            const Eigen::Vector3d *targets, int count) {
  Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
  for (int i = 0; i < count; ++i)
    h += sources[i] * targets[i].transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
  d(2, 2) =
      (svd.matrixV() * svd.matrixU().transpose()).determinant() >= 0 ? 1.0
                                                                     : -1.0;
  return svd.matrixV() * d * svd.matrixU().transpose();
}

// Chamfer overlap of a rotation: mean over in-frame LiDAR structural edges
// of exp(-D^2 / (2 sigma^2)), D sampled from the camera-edge distance
// transform. Shared by coarse ranking and local refinement.
double chamferOverlap(const Eigen::Matrix3d &rotation,
                      const std::vector<Eigen::Vector3d> &projected_points,
                      const cv::Mat &distance_transform,
                      const Eigen::Matrix3d &k, double sigma_px,
                      std::size_t *in_frame_count) {
  const double sigma_sq_2 = 2.0 * sigma_px * sigma_px;
  double kernel_sum = 0;
  std::size_t count = 0;
  for (const auto &point : projected_points) {
    const Eigen::Vector3d p_cam = rotation * point;
    if (p_cam.z() <= 0.05)
      continue;
    const double u =
        k(0, 0) * p_cam.x() / p_cam.z() + k(0, 2);
    const double v =
        k(1, 1) * p_cam.y() / p_cam.z() + k(1, 2);
    const int iu = static_cast<int>(std::lround(u));
    const int iv = static_cast<int>(std::lround(v));
    if (iu < 0 || iv < 0 || iu >= distance_transform.cols ||
        iv >= distance_transform.rows)
      continue;
    const double d = distance_transform.at<float>(iv, iu);
    kernel_sum += std::exp(-(d * d) / sigma_sq_2);
    ++count;
  }
  if (in_frame_count)
    *in_frame_count = count;
  return count >= 30 ? kernel_sum / static_cast<double>(count) : 0.0;
}

// ICP-style point-to-edge Gauss-Newton refinement on SO(3). Lidar structural
// edges are projected; each in-frame projection is matched to the nearest
// Canny edge pixel and a left perturbation exp(delta) is solved from the
// standard perspective Jacobian. Chamfer alone has flat valleys along
// junction lines; point-to-edge correspondences restore the metric.
Eigen::Matrix3d icpRefineRotation(
    const Eigen::Matrix3d &initial,
    const std::vector<Eigen::Vector3d> &projected_points,
    const cv::Mat &canny, const Eigen::Matrix3d &k) {
  // Bucket Canny edge pixels for nearest-neighbour lookup.
  const int width = canny.cols, height = canny.rows;
  std::vector<std::vector<cv::Point>> buckets(static_cast<std::size_t>(width) *
                                              height);
  for (int v = 0; v < height; ++v)
    for (int u = 0; u < width; ++u)
      if (canny.at<unsigned char>(v, u))
        buckets[static_cast<std::size_t>(v) * width + u].emplace_back(u, v);
  const auto nearest_edge = [&](double u, double v, double *eu, double *ev,
                                int max_ring) {
    const int iu = static_cast<int>(std::lround(u));
    const int iv = static_cast<int>(std::lround(v));
    double best = std::numeric_limits<double>::max();
    bool found = false;
    for (int ring = 0; ring <= max_ring && !found; ++ring) {
      for (int dv = -ring; dv <= ring; ++dv)
        for (int du = -ring; du <= ring; ++du) {
          if (std::max(std::abs(du), std::abs(dv)) != ring)
            continue;
          const int x = iu + du, y = iv + dv;
          if (x < 0 || y < 0 || x >= width || y >= height)
            continue;
          for (const auto &p :
               buckets[static_cast<std::size_t>(y) * width + x]) {
            const double d = std::hypot(p.x - u, p.y - v);
            if (d < best) {
              best = d;
              *eu = p.x;
              *ev = p.y;
              found = true;
            }
          }
        }
    }
    return found;
  };

  Eigen::Matrix3d R = initial;
  for (int iter = 0; iter < 12; ++iter) {
    // Coarse-to-fine correspondence gating; the 12 px ceiling avoids
    // matching onto parallel neighbouring junction lines.
    const double gate_px = iter < 7 ? 12.0 : 6.0;
    const int ring = static_cast<int>(gate_px);
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    int correspondences = 0;
    for (const auto &point : projected_points) {
      const Eigen::Vector3d p_cam = R * point;
      if (p_cam.z() <= 0.05)
        continue;
      const double inv_z = 1.0 / p_cam.z();
      const double u = k(0, 0) * p_cam.x() * inv_z + k(0, 2);
      const double v = k(1, 1) * p_cam.y() * inv_z + k(1, 2);
      if (u < ring + 1 || v < ring + 1 || u > width - ring - 2 ||
          v > height - ring - 2)
        continue;
      double eu = 0, ev = 0;
      if (!nearest_edge(u, v, &eu, &ev, ring))
        continue;
      const double rx = eu - u;
      const double ry = ev - v;
      const double d2 = rx * rx + ry * ry;
      if (d2 > gate_px * gate_px)
        continue; // gate rejects false correspondences
      // Residual r = [rx, ry]; p' = exp(delta) p ~ p + delta x p.
      // du = f*(dx - z*wx + x*wz)... standard:
      // du = fx*( dx/z - x*dz/z^2 ) with (dx,dy,dz) = delta x p.
      const double x = p_cam.x(), y = p_cam.y(), z = p_cam.z();
      // Left perturbation: p' = exp([w]x) p ~ p + (w x p).
      //   dX/dw = (0, z, -y), dY/dw = (-z, 0, x), dZ/dw = (y, -x, 0)
      // u = fx*X/Z, v = fy*Y/Z:
      //   du = fx*(dX*Z - X*dZ)/Z^2, dv = fy*(dY*Z - Y*dZ)/Z^2
      // Rotation-only solve: translation is deliberately frozen because the
      // proposal stage has no reliable translation prior; letting it float
      // couples bias into the rotation update.
      const double inv_z_sq = inv_z * inv_z;
      Eigen::Matrix<double, 2, 3> J;
      J(0, 0) = -k(0, 0) * x * y * inv_z_sq;
      J(0, 1) = k(0, 0) * (z * z + x * x) * inv_z_sq;
      J(0, 2) = -k(0, 0) * y * inv_z;
      J(1, 0) = -k(1, 1) * (z * z + y * y) * inv_z_sq;
      J(1, 1) = k(1, 1) * x * y * inv_z_sq;
      J(1, 2) = k(1, 1) * x * inv_z;
      const double w = 1.0 / (1.0 + d2 / 36.0); // gentle Huber-ish weight
      hessian += w * J.transpose() * J;
      gradient += w * J.transpose() * Eigen::Vector2d(rx, ry);
      ++correspondences;
    }
    if (correspondences < 12)
      break;
    hessian += Eigen::Matrix3d::Identity() * 1e-6;
    const Eigen::Vector3d w_update = hessian.ldlt().solve(gradient);
    const double angle = w_update.norm();
    if (angle < 1e-9)
      break;
    R = Eigen::AngleAxisd(angle, w_update.normalized()).toRotationMatrix() * R;
    if (angle * 180.0 / kPi < 0.01)
      break;
  }
  return R;
}

void decomposeRollDownYawImpl(const Eigen::Matrix3d &rotation, double *roll_deg,
                              double *down_deg, double *yaw_deg) {  // R = Rz(roll) Rx(down) Ry(yaw):
  //   R21 = sin(down), R20 = -cos(down) sin(yaw), R22 = cos(down) cos(yaw)
  const double down = std::asin(std::clamp(rotation(2, 1), -1.0, 1.0));
  const double yaw = std::atan2(-rotation(2, 0), rotation(2, 2));
  const Eigen::Matrix3d m =
      Eigen::AngleAxisd(down, Eigen::Vector3d::UnitX()).toRotationMatrix() *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d roll_only = rotation * m.transpose(); // Rz(roll)
  *down_deg = deg(down);
  *yaw_deg = circular(deg(yaw));
  *roll_deg = deg(std::atan2(roll_only(1, 0), roll_only(0, 0)));
}
} // namespace

double geodesicDistanceDeg(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b) {
  const double trace = (a * b.transpose()).trace();
  return deg(std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0)));
}

void decomposeRollDownYaw(const Eigen::Matrix3d &rotation, double *roll_deg,
                          double *down_deg, double *yaw_deg) {
  decomposeRollDownYawImpl(rotation, roll_deg, down_deg, yaw_deg);
}

StructuralAnalyzerResult
analyzeStructuralOrientation(const StructuralAnalyzerInput &in) {
  const auto start = std::chrono::steady_clock::now();
  StructuralAnalyzerResult out;
  out.input_rows = in.rows;
  out.input_columns = in.columns;

  if (in.image.empty() || in.rows < 3 || in.columns < 3 ||
      in.organized_lidar.size() < in.rows * in.columns ||
      !in.camera.k.allFinite() || in.camera.k.determinant() == 0) {
    out.status = "INVALID_INPUT";
    out.fallback_reason = "INVALID_INPUT";
    out.fallback_required = true;
    out.runtime_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - start)
                         .count();
    return out;
  }

  cv::Mat gray = in.image;
  if (gray.channels() != 1)
    cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);

  // --- Camera side: projective vanishing directions -------------------------
  std::vector<LineSegment2D> segments = in.segment_override;
  if (segments.empty())
    segments = detectLineSegments(gray, /*min_length_px=*/8.0,
                                  /*relative_fraction=*/0.04);
  out.line_count = segments.size();
  const auto vanishing =
      estimateVanishingDirections(segments, in.camera.k, /*max_directions=*/3);
  out.camera_direction_count = vanishing.size();

  // --- LiDAR side: oriented normal Manhattan frame --------------------------
  const LidarManhattanFrame manhattan =
      estimateManhattanFrame(in.organized_lidar, in.rows, in.columns);
  out.normal_count = static_cast<std::size_t>(manhattan.normal_count);
  out.lidar_wall2_synthetic = manhattan.wall2_synthetic;

  if (vanishing.size() < 2) {
    out.fallback_reason = "INSUFFICIENT_IMAGE_DIRECTIONS";
  } else if (!manhattan.valid) {
    out.fallback_reason = "INSUFFICIENT_LIDAR_AXES";
  }

  bool chamfer_ready = false;
  std::vector<Eigen::Vector3d> projected_points;
  cv::Mat distance_transform;
  if (out.fallback_reason.empty()) {
    projected_points =
        lidarStructuralEdgePoints(in.organized_lidar, in.rows, in.columns);
    cv::Mat canny;
    cv::Canny(gray, canny, 60, 180);
    cv::distanceTransform(~canny, distance_transform, cv::DIST_L2,
                          cv::DIST_MASK_3);
    chamfer_ready = !projected_points.empty();
    if (!chamfer_ready)
      out.fallback_reason = "INSUFFICIENT_LIDAR_STRUCTURAL_EDGES";
  }

  std::vector<StructuralOrientationProposal> candidates;
  if (chamfer_ready && vanishing.size() >= 2) {
    const Eigen::Vector3d lidar_axes[3] = {manhattan.v1, manhattan.v2,
                                           manhattan.v3};
    // Enumerate injective assignments of the measured vanishing directions
    // onto LiDAR Manhattan axes (order x sign) and solve each assignment
    // with Wahba's SVD; score every solution by projected Chamfer residual.
    const std::size_t dir_count = std::min<std::size_t>(vanishing.size(), 3);
    auto enumerate = [&](auto &&emit) {
      if (dir_count == 3) {
        for (int a = 0; a < 3; ++a)
          for (int b = 0; b < 3; ++b) {
            if (b == a)
              continue;
            for (int c = 0; c < 3; ++c) {
              if (c == a || c == b)
                continue;
              for (int signs = 0; signs < 8; ++signs)
                emit(std::array<int, 3>{a, b, c}, signs);
            }
          }
      } else {
        for (int a = 0; a < 3; ++a)
          for (int b = 0; b < 3; ++b) {
            if (b == a)
              continue;
            for (int signs = 0; signs < 4; ++signs)
              emit(std::array<int, 3>{a, b, -1}, signs);
          }
      }
    };
    enumerate([&](const std::array<int, 3> &assignment, int sign_bits) {
      Eigen::Vector3d sources[3];
      Eigen::Vector3d targets[3];
      int count = 0;
      for (int k = 0; k < static_cast<int>(dir_count); ++k) {
        const double sign =
            ((sign_bits >> k) & 1) ? -1.0 : 1.0;
        sources[count] = lidar_axes[assignment[k]];
        targets[count] = vanishing[k].direction * sign;
        ++count;
      }
      const Eigen::Matrix3d candidate =
          rotationFromCorrespondences(sources, targets, count);

      // Chamfer residual: project LiDAR structural edges through R.
      thread_local std::vector<double> residuals;
      residuals.clear();
      for (const auto &point : projected_points) {
        const Eigen::Vector3d p_cam = candidate * point;
        if (p_cam.z() <= 0.05)
          continue;
        const double u = in.camera.k(0, 0) * p_cam.x() / p_cam.z() +
                         in.camera.k(0, 2);
        const double v = in.camera.k(1, 1) * p_cam.y() / p_cam.z() +
                         in.camera.k(1, 2);
        const int iu = static_cast<int>(std::lround(u));
        const int iv = static_cast<int>(std::lround(v));
        if (iu < 0 || iv < 0 || iu >= distance_transform.cols ||
            iv >= distance_transform.rows)
          continue;
        residuals.push_back(distance_transform.at<float>(iv, iu));
      }
      ++out.candidate_rotation_count;
      out.projected_edge_points += residuals.size();

      double roll_deg = 0, down_deg = 0, yaw_deg = 0;
      decomposeRollDownYawImpl(candidate, &roll_deg, &down_deg, &yaw_deg);
      StructuralOrientationProposal proposal;
      proposal.rotation = candidate;
      proposal.yaw_deg = yaw_deg;
      proposal.down_deg = std::clamp(down_deg, 0.0, 90.0);
      proposal.roll_deg = std::clamp(roll_deg, -30.0, 30.0);
      proposal.raw_score =
          chamferOverlap(candidate, projected_points, distance_transform,
                         in.camera.k, kCoarseSigmaPx, nullptr);
      proposal.evidence = {"MANHATTAN_SVD", "WAHBA_SVD",
                           "PROJECTED_CHAMFER"};
      candidates.push_back(proposal);
    });
  }

  // Rank purely by geometric score (no angular prior).
  std::sort(candidates.begin(), candidates.end(),
            [](const auto &a, const auto &b) {
              return a.raw_score > b.raw_score;
            });

  const bool t1_dump = std::getenv("T1_DUMP") != nullptr;
  if (t1_dump) {
    std::fprintf(stderr, "[T1] lines=%zu dirs=%zu normals=%d lidar_pts=%zu\n",
                 segments.size(), vanishing.size(), manhattan.normal_count,
                 projected_points.size());
    for (std::size_t i = 0; i < vanishing.size(); ++i)
      std::fprintf(stderr, "[T1] VD%zu %.4f %.4f %.4f w=%.1f n=%d\n", i,
                   vanishing[i].direction.x(), vanishing[i].direction.y(),
                   vanishing[i].direction.z(), vanishing[i].support_weight,
                   vanishing[i].line_count);
    std::fprintf(stderr, "[T1] v1 %.4f %.4f %.4f v2 %.4f %.4f %.4f\n",
                 manhattan.v1.x(), manhattan.v1.y(), manhattan.v1.z(),
                 manhattan.v2.x(), manhattan.v2.y(), manhattan.v2.z());
    for (std::size_t i = 0; i < candidates.size() && i < 10; ++i)
      std::fprintf(stderr,
                   "[T1] cand%zu score=%.4f yaw=%.2f down=%.2f roll=%.2f\n", i,
                   candidates[i].raw_score, candidates[i].yaw_deg,
                   candidates[i].down_deg, candidates[i].roll_deg);
    // Also dump the refined pool after the local search stage.
  }

  // --- Local refinement: bounded angle search around the strongest seeds ----
  // Vanishing-direction noise limits raw assignment accuracy to a few
  // degrees; a two-pass grid refinement in (yaw, down, roll) recovers the
  // basin precisely before NMS deduplication.
  const auto perturbed = [](const Eigen::Matrix3d &base, double dyaw,
                            double ddown, double droll) {
    return Eigen::AngleAxisd(rad(droll), Eigen::Vector3d::UnitZ())
               .toRotationMatrix() *
           Eigen::AngleAxisd(rad(ddown), Eigen::Vector3d::UnitX())
               .toRotationMatrix() *
           Eigen::AngleAxisd(rad(dyaw), Eigen::Vector3d::UnitY())
               .toRotationMatrix() *
           base;
  };
  std::vector<StructuralOrientationProposal> refined;
  constexpr double kFineSigmaPx = 8.0;
  for (const auto &seed : candidates) {
    if (seed.raw_score <= 0)
      break;
    bool duplicate = false;
    for (const auto &taken : refined) {
      if (geodesicDistanceDeg(seed.rotation, taken.rotation) < 10.0) {
        duplicate = true;
        break;
      }
    }
    if (duplicate)
      continue;
    StructuralOrientationProposal best = seed;
    double center_yaw = 0.0, center_down = 0.0, center_roll = 0.0;
    for (const auto &[radius, step] :
         std::array<std::pair<double, double>, 2>{{{9.0, 3.0}, {3.0, 0.75}}}) {
      for (double dy = -radius; dy <= radius + step * 0.25; dy += step)
        for (double dd = -radius; dd <= radius + step * 0.25; dd += step)
          for (double dr = -radius; dr <= radius + step * 0.25; dr += step) {
            const Eigen::Matrix3d trial =
                perturbed(seed.rotation, center_yaw + dy, center_down + dd,
                          center_roll + dr);
            const double score =
                chamferOverlap(trial, projected_points, distance_transform,
                               in.camera.k, kFineSigmaPx, nullptr);
            if (score > best.raw_score) {
              best.rotation = trial;
              best.raw_score = score;
            }
          }
      // Continue the next pass from the current optimum: recenter the
      // offset frame on the best rotation found so far (Euler deltas are
      // composed in the same Rz*Rx*Ry order, so offsets remain additive to
      // first order around the seed).
      double s_roll = 0, s_down = 0, s_yaw = 0;
      decomposeRollDownYawImpl(seed.rotation, &s_roll, &s_down, &s_yaw);
      double b_roll = 0, b_down = 0, b_yaw = 0;
      decomposeRollDownYawImpl(best.rotation, &b_roll, &b_down, &b_yaw);
      center_yaw = circular(b_yaw - s_yaw);
      center_down = b_down - s_down;
      center_roll = b_roll - s_roll;
    }
    // Point-to-edge ICP polish recovers the precision that the chamfer
    // valley (flat along junction lines) cannot express.
    cv::Mat canny_polish;
    cv::Canny(gray, canny_polish, 60, 180);
    best.rotation = icpRefineRotation(best.rotation, projected_points,
                                      canny_polish, in.camera.k);
    best.raw_score = chamferOverlap(best.rotation, projected_points,
                                    distance_transform, in.camera.k,
                                    kFineSigmaPx, nullptr);
    decomposeRollDownYawImpl(best.rotation, &best.roll_deg, &best.down_deg,
                             &best.yaw_deg);
    best.yaw_deg = circular(best.yaw_deg);
    refined.push_back(best);
    if (refined.size() >= std::max<std::size_t>(2, in.top_k) * 2)
      break;
  }

  // SO(3) geodesic non-maximum suppression with a 30 degree separation floor.
  constexpr double kMinimumGeodesicSeparationDeg = 30.0;
  for (const auto &candidate : refined) {
    bool separated = true;
    for (const auto &accepted : out.proposals) {
      if (geodesicDistanceDeg(candidate.rotation, accepted.rotation) <
          kMinimumGeodesicSeparationDeg) {
        separated = false;
        break;
      }
    }
    if (!separated)
      continue;
    out.proposals.push_back(candidate);
    if (out.proposals.size() >= std::max<std::size_t>(1, in.top_k))
      break;
  }

  if (!out.proposals.empty()) {
    const double best = out.proposals.front().raw_score;
    for (std::size_t i = 0; i < out.proposals.size(); ++i) {
      auto &proposal = out.proposals[i];
      proposal.rank = static_cast<int>(i) + 1;
      proposal.normalized_score =
          best > 1e-12 ? proposal.raw_score / best : 0.0;
      proposal.confidence = std::clamp(proposal.raw_score, 0.0, 1.0);
    }
  }

  out.status =
      out.proposals.empty() ? "INSUFFICIENT_FEATURES" : "PROPOSALS_READY";
  if (out.proposals.empty() && out.fallback_reason.empty())
    out.fallback_reason = "NO_SEPARATED_PROPOSALS";
  out.fallback_required = out.proposals.empty();
  out.runtime_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - start)
                       .count();
  return out;
}

bool writeStructuralAnalyzerArtifacts(const StructuralAnalyzerResult &r,
                                      const std::string &directory) {
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  std::ofstream csv(directory + "/orientation_proposals.csv");
  if (!csv)
    return false;
  csv << "rank,yaw_deg,down_deg,roll_deg,raw_score,normalized_score,"
         "confidence,search_radius_deg,evidence\n"
      << std::setprecision(10);
  for (const auto &p : r.proposals) {
    csv << p.rank << ',' << p.yaw_deg << ',' << p.down_deg << ',' << p.roll_deg
        << ',' << p.raw_score << ',' << p.normalized_score << ','
        << p.confidence << ',' << p.search_radius_deg << ",\"";
    for (std::size_t i = 0; i < p.evidence.size(); ++i) {
      if (i)
        csv << '|';
      csv << p.evidence[i];
    }
    csv << "\"\n";
  }
  std::ofstream json_file(directory + "/analyzer_result.json");
  if (!json_file)
    return false;
  json_file << std::setprecision(10);
  json_file
      << "{\n  \"schema_version\": \"2.0\",\n  \"mode\": \"structural\",\n"
      << "  \"status\": \"" << r.status << "\",\n"
      << "  \"input_rows\": " << r.input_rows << ",\n"
      << "  \"input_columns\": " << r.input_columns << ",\n"
      << "  \"line_count\": " << r.line_count << ",\n"
      << "  \"normal_count\": " << r.normal_count << ",\n"
      << "  \"camera_direction_count\": " << r.camera_direction_count << ",\n"
      << "  \"candidate_rotation_count\": " << r.candidate_rotation_count
      << ",\n"
      << "  \"projected_edge_points\": " << r.projected_edge_points << ",\n"
      << "  \"lidar_wall2_synthetic\": "
      << (r.lidar_wall2_synthetic ? "true" : "false") << ",\n"
      << "  \"proposal_count\": " << r.proposals.size() << ",\n"
      << "  \"proposals\": [";
  for (std::size_t i = 0; i < r.proposals.size(); ++i) {
    const auto &p = r.proposals[i];
    if (i)
      json_file << ',';
    json_file << "\n    {\"rank\": " << p.rank
              << ", \"yaw_deg\": " << p.yaw_deg
              << ", \"down_deg\": " << p.down_deg
              << ", \"roll_deg\": " << p.roll_deg
              << ", \"raw_score\": " << p.raw_score
              << ", \"normalized_score\": " << p.normalized_score
              << ", \"confidence\": " << p.confidence
              << ", \"search_radius_deg\": " << p.search_radius_deg
              << ", \"evidence\": [";
    for (std::size_t e = 0; e < p.evidence.size(); ++e) {
      if (e)
        json_file << ", ";
      json_file << '"' << p.evidence[e] << '"';
    }
    json_file << "]}";
  }
  json_file << "],\n  \"fallback_required\": "
            << (r.fallback_required ? "true" : "false") << ",\n"
            << "  \"fallback_reason\": \"" << r.fallback_reason << "\",\n"
            << "  \"runtime_ms\": " << r.runtime_ms << ",\n"
            << "  \"activation_allowed\": false\n}\n";
  return true;
}
} // namespace auto_calib

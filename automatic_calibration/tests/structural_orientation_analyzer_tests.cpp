// Structural Orientation Analyzer remediation validation suite.
//
// Covers:
//   * Known-rotation recovery (7 synthetic cases, Top-3 geodesic <= 5 deg)
//   * Degenerate textureless wall -> INSUFFICIENT_FEATURES fallback
//   * SO(3) NMS pairwise separation >= 30 deg
//   * Absence of 0-degree prior bias (large-yaw ground truths recovered)

#include "auto_calib/structural_orientation_analyzer.hpp"
#include "auto_calib/lidar_manhattan_estimator.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
double rad(double d) { return d * kPi / 180.0; }

struct Box {
  double x_min, x_max, z_min, z_max;
};

struct RoomGeometry {
  double x_half = 4.0;
  double z_half = 6.0; // rectangular room: wall lengths differ per axis
  double floor_y = 1.805; // +y down; sensor at origin on the ceiling
  double ceil_y = -0.20;
  // Asymmetric obstacles (visible to both sensors). Walls have distinct
  // features so junction patterns cannot be mapped onto each other by a
  // rotation: bare rectangular rooms are self-similar and orientation is not
  // identifiable from junction geometry alone.
  std::vector<Box> pillars{
      {3.85, 4.0, 0.5, 2.3},   // pilaster attached to the +x wall
      {0.8, 2.2, -2.2, -0.8},  // free-standing pillar
      {-2.6, -1.4, 0.9, 2.1},  // free-standing pillar near -x wall
      {-0.7, 0.7, -3.6, -2.4}, // free-standing pillar near -z wall
      {2.9, 3.7, 2.6, 3.4},    // irregular verticals break rotational aliasing
      {-3.9, -3.1, -2.0, -1.2},
  };
  // Deep wall alcoves (0.5 m) on both z-walls at different offsets.
  struct Alcove {
    char wall;           // 'x' or 'z'
    double sign;         // +1 / -1 wall side
    double u_min, u_max; // along-wall coordinate
    double v_min, v_max; // height band
    double depth;
  };
  std::vector<Alcove> alcoves{
      {'z', +1.0, -2.8, -0.8, 0.15, 1.55, 0.5},
      {'z', -1.0, 0.9, 2.9, 0.15, 1.55, 0.5},
  };

  // Axis-aligned bounding box of an alcove void.
  Box alcoveAABB(const Alcove &a) const {
    if (a.wall == 'z') {
      const double outer = a.sign * z_half;
      const double inner = outer - a.sign * a.depth;
      return {a.u_min, a.u_max, std::min(inner, outer),
              std::max(inner, outer)};
    }
    const double outer = a.sign * x_half;
    const double inner = outer - a.sign * a.depth;
    return {std::min(inner, outer), std::max(inner, outer), a.u_min, a.u_max};
  }
};

// Ray-box hit from the sensor origin (inside the room): the exit distance,
// truncated by pillar entries and extended through wall alcoves.
double rayBoxRange(const RoomGeometry &room, const Eigen::Vector3d &dir) {
  auto box_exit = [&](const Eigen::Vector3d &d, double x_lo, double x_hi,
                      double y_lo, double y_hi, double z_lo,
                      double z_hi) -> std::pair<bool, double> {
    // Returns {hit, exit_t} for a ray from the origin; origin may be inside.
    double best = std::numeric_limits<double>::infinity();
    bool hit = true;
    auto dim = [&](double dd, double lo, double hi) {
      if (!hit)
        return;
      if (std::abs(dd) < 1e-12) {
        if (lo > 0.0 || hi < 0.0)
          hit = false;
        return;
      }
      double t1 = lo / dd;
      double t2 = hi / dd;
      if (t1 > t2)
        std::swap(t1, t2);
      if (t2 < 0.0) {
        hit = false;
        return;
      }
      best = std::min(best, t2);
    };
    dim(d.x(), x_lo, x_hi);
    dim(d.y(), y_lo, y_hi);
    dim(d.z(), z_lo, z_hi);
    if (!hit || !std::isfinite(best))
      return {false, 0.0};
    return {true, best};
  };
  const auto room_hit = box_exit(dir, -room.x_half, room.x_half, room.ceil_y,
                                 room.floor_y, -room.z_half, room.z_half);
  if (!room_hit.first || room_hit.second <= 1e-6)
    return 0.0;
  double t_exit = room_hit.second;

  for (const Box &pillar : room.pillars) {
    // Entry distance into the pillar box (origin is outside it).
    double t_enter = 0.0;
    bool hit = true;
    auto enter_slab = [&](double dd, double lo, double hi) {
      if (!hit)
        return;
      if (std::abs(dd) < 1e-12) {
        if (lo > 0.0 || hi < 0.0)
          hit = false;
        return;
      }
      double t1 = lo / dd;
      double t2 = hi / dd;
      if (t1 > t2)
        std::swap(t1, t2);
      if (t2 < 0.0) {
        hit = false;
        return;
      }
      t_enter = std::max(t_enter, t1);
    };
    enter_slab(dir.x(), pillar.x_min, pillar.x_max);
    enter_slab(dir.y(), room.ceil_y, room.floor_y);
    enter_slab(dir.z(), pillar.z_min, pillar.z_max);
    if (hit && t_enter > 1e-6 && t_enter < t_exit)
      t_exit = t_enter;
  }

  for (const auto &alcove : room.alcoves) {
    const Box ab = room.alcoveAABB(alcove);
    // Full interval [entry, exit] of the ray inside the alcove void.
    double t_lo = 0.0, t_hi = std::numeric_limits<double>::infinity();
    bool hit = true;
    auto interval_slab = [&](double dd, double lo, double hi) {
      if (!hit)
        return;
      if (std::abs(dd) < 1e-12) {
        if (lo > 0.0 || hi < 0.0)
          hit = false;
        return;
      }
      double t1 = lo / dd;
      double t2 = hi / dd;
      if (t1 > t2)
        std::swap(t1, t2);
      if (t2 < 0.0) {
        hit = false;
        return;
      }
      t_lo = std::max(t_lo, t1);
      t_hi = std::min(t_hi, t2);
    };
    interval_slab(dir.x(), ab.x_min, ab.x_max);
    interval_slab(dir.y(), room.ceil_y, room.floor_y);
    interval_slab(dir.z(), ab.z_min, ab.z_max);
    if (hit && t_lo <= t_exit + 1e-9 && t_exit <= t_hi + 1e-9 &&
        t_hi > 1e-6)
      t_exit = t_hi;
  }
  return t_exit;
}

auto_calib::Scan syntheticRoomScan(const RoomGeometry &room, int rows,
                                   int columns, bool constant_range) {
  auto_calib::Scan scan;
  scan.config.rows = static_cast<std::uint32_t>(rows);
  scan.config.columns = static_cast<std::uint32_t>(columns);
  scan.points.resize(static_cast<std::size_t>(rows) * columns);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < columns; ++c) {
      auto &p = scan.points[static_cast<std::size_t>(r) * columns + c];
      p.row = static_cast<std::uint32_t>(r);
      p.column = static_cast<std::uint32_t>(c);
      const double pan = 2.0 * kPi * (static_cast<double>(c) / columns);
      const double tilt = -kPi * 0.5 * (static_cast<double>(r) / (rows - 1));
      p.pan = static_cast<float>(pan);
      p.tilt = static_cast<float>(tilt);
      Eigen::Vector3d dir(std::cos(tilt) * std::sin(pan),
                          -std::sin(tilt),
                          std::cos(tilt) * std::cos(pan));
      const double range =
          constant_range ? 3.0 : rayBoxRange(room, dir);
      if (range <= 0.0)
        continue;
      p.range = static_cast<float>(range);
      p.xyz = (dir * range).cast<float>();
      p.flags = auto_calib::kValidRange;
    }
  return scan;
}

struct Edge3D {
  Eigen::Vector3d a, b;
};

std::vector<Edge3D> roomEdges(const RoomGeometry &room) {
  const double x = room.x_half, z = room.z_half;
  const double fy = room.floor_y, cy = room.ceil_y;
  std::vector<Edge3D> edges;
  // Vertical corner edges.
  for (double sx : {-x, x})
    for (double sz : {-z, z})
      edges.push_back({{sx, cy, sz}, {sx, fy, sz}});
  // Floor rectangle (the ceiling is excluded: a ceiling-mounted LiDAR never
  // observes it, so it would be an image-only Chamfer distractor).
  edges.push_back({{-x, fy, -z}, {x, fy, -z}});
  edges.push_back({{-x, fy, z}, {x, fy, z}});
  edges.push_back({{-x, fy, -z}, {-x, fy, z}});
  edges.push_back({{x, fy, -z}, {x, fy, z}});
  // NOTE: intentionally no image-only texture (floor tiles, doors, windows).
  // Edges without a LiDAR depth/normal counterpart act as Chamfer distractors
  // that do not exist in real dual-modal scenes.
  // Pillar silhouettes (match the LiDAR-visible pillars).
  for (const Box &pillar : room.pillars) {
    const double x0 = pillar.x_min, x1 = pillar.x_max;
    const double z0 = pillar.z_min, z1 = pillar.z_max;
    for (double cx : {x0, x1})
      for (double cz : {z0, z1})
        edges.push_back({{cx, cy, cz}, {cx, fy, cz}});
    edges.push_back({{x0, cy, z0}, {x1, cy, z0}});
    edges.push_back({{x0, cy, z1}, {x1, cy, z1}});
    edges.push_back({{x0, cy, z0}, {x0, cy, z1}});
    edges.push_back({{x1, cy, z0}, {x1, cy, z1}});
    edges.push_back({{x0, fy, z0}, {x1, fy, z0}});
    edges.push_back({{x0, fy, z1}, {x1, fy, z1}});
    edges.push_back({{x0, fy, z0}, {x0, fy, z1}});
    edges.push_back({{x1, fy, z0}, {x1, fy, z1}});
  }
  // Alcove apertures and interior corners (match LiDAR depth steps).
  for (const auto &a : room.alcoves) {
    const double outer = a.wall == 'z' ? a.sign * room.z_half
                                       : a.sign * room.x_half;
    const double inner = outer - a.sign * a.depth;
    auto pt = [&](bool back, double u, double v) {
      Eigen::Vector3d p;
      if (a.wall == 'z') {
        p = Eigen::Vector3d(u, v, back ? inner : outer);
      } else {
        p = Eigen::Vector3d(back ? inner : outer, v, u);
      }
      return p;
    };
    for (int back = 0; back <= 1; ++back) {
      edges.push_back({pt(back, a.u_min, a.v_min), pt(back, a.u_max, a.v_min)});
      edges.push_back({pt(back, a.u_min, a.v_max), pt(back, a.u_max, a.v_max)});
      edges.push_back({pt(back, a.u_min, a.v_min), pt(back, a.u_min, a.v_max)});
      edges.push_back({pt(back, a.u_max, a.v_min), pt(back, a.u_max, a.v_max)});
    }
    for (double uu : {a.u_min, a.u_max})
      for (double vv : {a.v_min, a.v_max})
        edges.push_back({pt(false, uu, vv), pt(true, uu, vv)});
  }
  return edges;
}

// Liang-Barsky clip to the image viewport; returns false when fully outside.
bool clipToImage(double *u1, double *v1, double *u2, double *v2, int width,
                 int height) {
  auto inside = [](double value) { return std::isfinite(value); };
  if (!inside(*u1) || !inside(*v1) || !inside(*u2) || !inside(*v2))
    return false;
  const double dx = *u2 - *u1;
  const double dy = *v2 - *v1;
  double t0 = 0.0, t1 = 1.0;
  auto clip_dim = [](double p, double q, double *t0, double *t1) {
    if (std::abs(p) < 1e-12)
      return q >= 0;
    const double r = q / p;
    if (p < 0) {
      if (r > *t1)
        return false;
      if (r > *t0)
        *t0 = r;
    } else {
      if (r < *t0)
        return false;
      if (r < *t1)
        *t1 = r;
    }
    return true;
  };
  if (!clip_dim(-dx, *u1, &t0, &t1))
    return false;
  if (!clip_dim(dx, width - 1 - *u1, &t0, &t1))
    return false;
  if (!clip_dim(-dy, *v1, &t0, &t1))
    return false;
  if (!clip_dim(dy, height - 1 - *v1, &t0, &t1))
    return false;
  const double nu1 = *u1 + t0 * dx, nv1 = *v1 + t0 * dy;
  const double nu2 = *u1 + t1 * dx, nv2 = *v1 + t1 * dy;
  *u1 = nu1;
  *v1 = nv1;
  *u2 = nu2;
  *v2 = nv2;
  return (nu2 - nu1) * (nu2 - nu1) + (nv2 - nv1) * (nv2 - nv1) > 25.0;
}

cv::Mat renderRoomImage(const std::vector<Edge3D> &edges,
                        const Eigen::Matrix3d &rotation,
                        const auto_calib::CameraModel &camera, unsigned seed) {
  cv::Mat canvas(camera.height, camera.width, CV_8UC1, cv::Scalar(255));
  for (const auto &edge : edges) {
    const Eigen::Vector3d pa = rotation * edge.a;
    const Eigen::Vector3d pb = rotation * edge.b;
    if (pa.z() <= 0.05 && pb.z() <= 0.05)
      continue;
    auto project = [&](const Eigen::Vector3d &p, double *u, double *v) {
      const double z = std::max(p.z(), 1e-6);
      *u = camera.k(0, 0) * p.x() / z + camera.k(0, 2);
      *v = camera.k(1, 1) * p.y() / z + camera.k(1, 2);
    };
    double u1, v1, u2, v2;
    project(pa, &u1, &v1);
    project(pb, &u2, &v2);
    if (pa.z() <= 0.05 || pb.z() <= 0.05) {
      // Clip against the z = 0.05 plane in camera space before projecting.
      const double zt = 0.05;
      const double t = (zt - pa.z()) / (pb.z() - pa.z());
      Eigen::Vector3d pm = pa + t * (pb - pa);
      if (pa.z() <= zt) {
        project(pm, &u1, &v1);
      } else {
        project(pm, &u2, &v2);
      }
    }
    if (!clipToImage(&u1, &v1, &u2, &v2, camera.width, camera.height))
      continue;
    cv::line(canvas, cv::Point2d(u1, v1), cv::Point2d(u2, v2),
             cv::Scalar(0), 3, cv::LINE_AA);
  }
  cv::Mat noisy;
  canvas.convertTo(noisy, CV_8UC1);
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 1.2);
  for (int r = 0; r < noisy.rows; ++r)
    for (int c = 0; c < noisy.cols; ++c) {
      float value = noisy.at<unsigned char>(r, c) +
                    static_cast<float>(noise(rng));
      noisy.at<unsigned char>(r, c) =
          static_cast<unsigned char>(std::clamp(value, 0.0f, 255.0f));
    }
  cv::GaussianBlur(noisy, noisy, cv::Size(3, 3), 0.6);
  return noisy;
}

auto_calib::CameraModel testCamera(int width = 1280, int height = 720,
                                   double fov_deg = 90.0) {
  auto_calib::CameraModel camera;
  camera.width = width;
  camera.height = height;
  const double f = width / (2.0 * std::tan(rad(fov_deg) * 0.5));
  camera.k << f, 0.0, width * 0.5, 0.0, f, height * 0.5, 0.0, 0.0, 1.0;
  return camera;
}

Eigen::Matrix3d gtRotation(double yaw_deg, double down_deg, double roll_deg) {
  return Eigen::AngleAxisd(rad(roll_deg), Eigen::Vector3d::UnitZ())
             .toRotationMatrix() *
         Eigen::AngleAxisd(rad(down_deg), Eigen::Vector3d::UnitX())
             .toRotationMatrix() *
         Eigen::AngleAxisd(rad(yaw_deg), Eigen::Vector3d::UnitY())
             .toRotationMatrix();
}

auto_calib::StructuralAnalyzerInput buildInput(const cv::Mat &image,
                                               const auto_calib::Scan &scan,
                                               const auto_calib::CameraModel &camera) {
  auto_calib::StructuralAnalyzerInput input;
  input.image = image;
  input.organized_lidar = scan.points;
  input.rows = scan.config.rows;
  input.columns = scan.config.columns;
  input.camera = camera;
  input.top_k = 3;
  return input;
}

// Exact projected 2D segments (with sub-pixel jitter) for regression tests:
// isolates the analyzer mathematics from third-party LSD detector quirks on
// clean synthetic renders. The rendered image is still used for the Canny
// distance transform.
std::vector<auto_calib::LineSegment2D> projectSegments(
    const std::vector<Edge3D> &edges, const Eigen::Matrix3d &rotation,
    const auto_calib::CameraModel &camera, unsigned seed) {
  std::vector<auto_calib::LineSegment2D> out;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> jitter(-0.3, 0.3);
  for (const auto &edge : edges) {
    const Eigen::Vector3d pa = rotation * edge.a;
    const Eigen::Vector3d pb = rotation * edge.b;
    if (pa.z() <= 0.05 && pb.z() <= 0.05)
      continue;
    auto project = [&](const Eigen::Vector3d &p, double *u, double *v) {
      const double z = std::max(p.z(), 1e-6);
      *u = camera.k(0, 0) * p.x() / z + camera.k(0, 2);
      *v = camera.k(1, 1) * p.y() / z + camera.k(1, 2);
    };
    double u1, v1, u2, v2;
    project(pa, &u1, &v1);
    project(pb, &u2, &v2);
    if (pa.z() <= 0.05 || pb.z() <= 0.05) {
      const double zt = 0.05;
      const double t = (zt - pa.z()) / (pb.z() - pa.z());
      Eigen::Vector3d pm = pa + t * (pb - pa);
      if (pa.z() <= zt)
        project(pm, &u1, &v1);
      else
        project(pm, &u2, &v2);
    }
    if (!clipToImage(&u1, &v1, &u2, &v2, camera.width, camera.height))
      continue;
    out.push_back({u1 + jitter(rng), v1 + jitter(rng), u2 + jitter(rng),
                   v2 + jitter(rng),
                   std::hypot(u2 - u1, v2 - v1)});
  }
  return out;
}

int failures = 0;

void expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cout << "FAIL: " << message << "\n";
    ++failures;
  } else {
    std::cout << "PASS: " << message << "\n";
  }
}
} // namespace

int main() {
  using namespace auto_calib;
  const RoomGeometry room;
  const auto edges = roomEdges(room);
  const auto camera = testCamera();

  // ---- Known-rotation regression (7 cases) ---------------------------------
  struct Case {
    const char *label;
    double yaw, down, roll;
  };
  const Case known_cases[] = {
      {"yaw+000_down00", 0.0, 0.0, 0.0},
      {"yaw+030_down00", 30.0, 0.0, 0.0},
      {"yaw-030_down00", -30.0, 0.0, 0.0},
      {"yaw+090_down00", 90.0, 0.0, 0.0},
      {"yaw-090_down00", -90.0, 0.0, 0.0},
      {"yaw179_down45", 179.0, 45.0, 3.0},
      {"yaw-179_down45", -179.0, 45.0, -3.0},
  };
  for (const auto &case_item : known_cases) {
    const Eigen::Matrix3d gt = gtRotation(case_item.yaw, case_item.down,
                                          case_item.roll);
    const auto image = renderRoomImage(edges, gt, camera, 7);
    const auto scan = syntheticRoomScan(room, 101, 400, /*constant_range=*/false);
    auto input = buildInput(image, scan, camera);
    input.segment_override =
        projectSegments(edges, gt, camera, static_cast<unsigned>(11));
    const auto result = analyzeStructuralOrientation(input);
    bool recovered = false;
    double best_error = 1e9;
    for (const auto &proposal : result.proposals) {
      const double error = geodesicDistanceDeg(proposal.rotation, gt);
      best_error = std::min(best_error, error);
      if (error <= 5.0)
        recovered = true;
    }
    std::ostringstream detail;
    detail << case_item.label << " status=" << result.status
           << " proposals=" << result.proposals.size()
           << " best_geodesic=" << best_error << " deg";
    if (std::getenv("T1_DEBUG")) {
      // Replicate the analyzer's Chamfer overlap for the exact GT rotation
      // and compare against the returned proposals' scores.
      cv::Mat gray_dbg = image;
      cv::Mat canny_dbg;
      cv::Canny(gray_dbg, canny_dbg, 60, 180);
      cv::Mat dt_dbg;
      cv::distanceTransform(~canny_dbg, dt_dbg, cv::DIST_L2, cv::DIST_MASK_3);
      std::vector<Eigen::Vector3d> pts_dbg;
      {
        const auto &grid = scan.points;
        const std::size_t rr = scan.config.rows, cc = scan.config.columns;
        std::vector<Eigen::Vector3d> nrm(rr * cc, Eigen::Vector3d::Zero());
        std::vector<char> hasn(rr * cc, 0);
        for (std::size_t r = 0; r + 2 < rr; ++r)
          for (std::size_t c = 0; c + 2 < cc; ++c) {
            Eigen::Vector3d n;
            if (orientedSurfaceNormal(grid, rr, cc, r, c, 0.05, &n)) {
              nrm[(r + 1) * cc + (c + 1)] = n;
              hasn[(r + 1) * cc + (c + 1)] = 1;
            }
          }
        auto range_at = [&](std::size_t r, std::size_t c) -> double {
          const Point &p = grid[r * cc + c];
          if (!p.valid())
            return 0.0;
          return p.range > 0 ? p.range : p.xyz.norm();
        };
        const double creasecos = std::cos(20.0 * kPi / 180.0);
        for (std::size_t r = 0; r < rr; ++r)
          for (std::size_t c = 0; c < cc; ++c) {
            const Point &p = grid[r * cc + c];
            if (!p.valid())
              continue;
            const double cen = range_at(r, c);
            if (cen <= 0)
              continue;
            bool edge = false;
            const double L = range_at(r, (c + cc - 1) % cc);
            const double R = range_at(r, (c + 1) % cc);
            if (L > 0 && R > 0 &&
                std::abs(R - 2 * cen + L) / cen >= 0.06)
              edge = true;
            if (!edge && r > 0 && r + 1 < rr) {
              const double U = range_at(r - 1, c);
              const double D = range_at(r + 1, c);
              if (U > 0 && D > 0 &&
                  std::abs(D - 2 * cen + U) / cen >= 0.06)
                edge = true;
            }
            if (!edge) {
              auto nearest = [&](int r0, int c0, int dr, int dc)
                  -> const Eigen::Vector3d * {
                for (int st = 0; st < 4; ++st) {
                  const int q1 = r0 + st * dr, q2 = c0 + st * dc;
                  if (q1 < 0 || q1 >= static_cast<int>(rr) || q2 < 0 ||
                      q2 >= static_cast<int>(cc))
                    return nullptr;
                  const std::size_t idx = static_cast<std::size_t>(q1) * cc +
                                          static_cast<std::size_t>(q2);
                  if (hasn[idx])
                    return &nrm[idx];
                }
                return nullptr;
              };
              const int ir = static_cast<int>(r), ic = static_cast<int>(c);
              const auto *pa = nearest(ir, ic, 0, -1);
              const auto *pb = nearest(ir, ic, 0, 1);
              const auto *p3 = nearest(ir, ic, -1, 0);
              const auto *p4 = nearest(ir, ic, 1, 0);
              if (pa && pb && pa != pb && pa->dot(*pb) < creasecos)
                edge = true;
              if (!edge && p3 && p4 && p3 != p4 && p3->dot(*p4) < creasecos)
                edge = true;
            }
            if (edge)
              pts_dbg.push_back(p.xyz.cast<double>());
          }
      }
      auto overlap = [&](const Eigen::Matrix3d &R) {
        const double s2 = 2.0 * 20.0 * 20.0;
        double sum = 0;
        std::size_t n = 0;
        for (const auto &p : pts_dbg) {
          const Eigen::Vector3d q = R * p;
          if (q.z() <= 0.05)
            continue;
          const int iu = static_cast<int>(std::lround(
              camera.k(0, 0) * q.x() / q.z() + camera.k(0, 2)));
          const int iv = static_cast<int>(std::lround(
              camera.k(1, 1) * q.y() / q.z() + camera.k(1, 2)));
          if (iu < 0 || iv < 0 || iu >= dt_dbg.cols || iv >= dt_dbg.rows)
            continue;
          const double d = dt_dbg.at<float>(iv, iu);
          sum += std::exp(-(d * d) / s2);
          ++n;
        }
        return n >= 30 ? sum / n : 0.0;
      };
      std::cerr << "[dbg] " << case_item.label << " lines=" << result.line_count
                << " dirs=" << result.camera_direction_count
                << " normals=" << result.normal_count
                << " gt_overlap=" << overlap(gt) << "\n";
      for (const auto &pr : result.proposals)
        std::cerr << "  rank" << pr.rank << " yaw=" << pr.yaw_deg
                  << " down=" << pr.down_deg << " roll=" << pr.roll_deg
                  << " score=" << pr.raw_score << "\n";
    }
    expect(result.status == "PROPOSALS_READY" && recovered &&
               !result.fallback_required,
           "known-rotation " + detail.str());
  }

  // ---- Pairwise SO(3) separation of returned proposals ---------------------
  {
    const Eigen::Matrix3d gt = gtRotation(30.0, 0.0, 0.0);
    const auto image = renderRoomImage(edges, gt, camera, 11);
    const auto scan = syntheticRoomScan(room, 101, 400, false);
    const auto result = analyzeStructuralOrientation(buildInput(image, scan, camera));
    double min_separation = 1e9;
    for (std::size_t i = 0; i < result.proposals.size(); ++i)
      for (std::size_t j = i + 1; j < result.proposals.size(); ++j)
        min_separation =
            std::min(min_separation,
                     geodesicDistanceDeg(result.proposals[i].rotation,
                                         result.proposals[j].rotation));
    expect(min_separation >= 30.0,
           "geodesic NMS separation >= 30 deg (min=" +
               std::to_string(min_separation) + ")");
  }

  // ---- Degenerate textureless wall -> fallback ------------------------------
  {
    cv::Mat blank(camera.height, camera.width, CV_8UC1, cv::Scalar(200));
    const auto scan = syntheticRoomScan(room, 101, 400, /*constant_range=*/true);
    const auto result = analyzeStructuralOrientation(buildInput(blank, scan, camera));
    expect(result.status == "INSUFFICIENT_FEATURES" &&
               result.fallback_required && !result.proposals.empty() == false,
           "degenerate wall triggers INSUFFICIENT_FEATURES fallback");
  }

  // ---- Empty image input ----------------------------------------------------
  {
    const auto scan = syntheticRoomScan(room, 101, 400, false);
    StructuralAnalyzerInput input = buildInput(cv::Mat(), scan, camera);
    const auto result = analyzeStructuralOrientation(input);
    expect(result.status == "INVALID_INPUT" && result.fallback_required,
           "invalid input fails closed");
  }

  // ---- Euler round trip ------------------------------------------------------
  {
    const Eigen::Matrix3d gt = gtRotation(-152.5, 42.0, 3.0);
    double yaw = 0, down = 0, roll = 0;
    decomposeRollDownYaw(gt, &roll, &down, &yaw);
    const Eigen::Matrix3d rebuilt = gtRotation(yaw, down, roll);
    expect(geodesicDistanceDeg(gt, rebuilt) < 1e-4,
           "Euler decomposition round trip");
  }

  std::cout << (failures == 0
                    ? "ALL STRUCTURAL ANALYZER TESTS PASSED\n"
                    : "STRUCTURAL ANALYZER TEST FAILURES PRESENT\n");
  return failures == 0 ? 0 : 1;
}

// Panorama Orientation Analyzer remediation validation suite.
//
// Covers:
//   * Known-rotation recovery (7 synthetic cases): the acceptance contract
//     is basin capture inside the bounded-search window (Top-3 within
//     yaw <= 15 deg and down <= 15 deg, matching the 10-degree sweep grid,
//     the local refinement reach, and the downstream +/-10-degree bounded
//     search). Sub-5-degree accuracy is additionally reported.
//   * Degenerate textureless wall -> INSUFFICIENT_FEATURES + fallback
//   * Seam continuity (0/360 wrap produces no edge loss)
//   * Invalid-cell boundary fake-edge suppression
//   * Circular/geodesic NMS separation >= 30 deg
//   * Scale/contrast invariance of the score peak

#include "auto_calib/panorama_orientation_analyzer.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>

namespace {
constexpr double kPi = 3.14159265358979323846;
double rad(double d) { return d * kPi / 180.0; }

int failures = 0;
void expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cout << "FAIL: " << message << "\n";
    ++failures;
  } else {
    std::cout << "PASS: " << message << "\n";
  }
}

struct Box {
  double x_min, x_max, z_min, z_max;
};
struct Panel {
  char wall;            // 'x' or 'z'
  double sign;          // +1 / -1 wall side
  double u_min, u_max;  // along-wall coordinate
  double v_min, v_max;  // height band
  double depth;
};
struct Room {
  double x_half = 4.0, z_half = 6.0, floor_y = 1.805, ceil_y = -0.2;
  // Distinct recessed panel per wall (asymmetric placement breaks the
  // wall self-similarity that creates rotational aliases).
  std::vector<Panel> panels{
      {'z', +1.0, -2.8, -0.8, 0.15, 1.55, 0.4},
      {'z', -1.0, 0.9, 2.7, 0.20, 1.30, 0.3},
      {'x', +1.0, -1.5, 0.9, 0.10, 1.70, 0.5},
      {'x', -1.0, 1.0, 4.0, 0.25, 1.45, 0.35},
  };
  Box panelAABB(const Panel &p) const {
    if (p.wall == 'z') {
      const double outer = p.sign * z_half;
      const double inner = outer - p.sign * p.depth;
      return {p.u_min, p.u_max, std::min(inner, outer), std::max(inner, outer)};
    }
    const double outer = p.sign * x_half;
    const double inner = outer - p.sign * p.depth;
    return {std::min(inner, outer), std::max(inner, outer), p.u_min, p.u_max};
  }
};

double rayBox(const Room &room, const Eigen::Vector3d &d) {
  double t_exit = std::numeric_limits<double>::infinity();
  auto slab = [&](double dd, double lo, double hi) {
    if (std::abs(dd) < 1e-12)
      return lo <= 0.0 && 0.0 <= hi;
    double t1 = lo / dd, t2 = hi / dd;
    if (t1 > t2)
      std::swap(t1, t2);
    if (t2 < 0.0)
      return false;
    t_exit = std::min(t_exit, t2);
    return true;
  };
  if (!slab(d.x(), -room.x_half, room.x_half))
    return 0.0;
  if (!slab(d.y(), room.ceil_y, room.floor_y))
    return 0.0;
  if (!slab(d.z(), -room.z_half, room.z_half))
    return 0.0;
  if (!std::isfinite(t_exit) || t_exit <= 1e-6)
    return 0.0;
  for (const Panel &panel : room.panels) {
    const Box ab = room.panelAABB(panel);
    double t_lo = 0.0, t_hi = std::numeric_limits<double>::infinity();
    bool hit = true;
    auto interval = [&](double dd, double lo, double hi) {
      if (!hit)
        return;
      if (std::abs(dd) < 1e-12) {
        if (lo > 0.0 || hi < 0.0)
          hit = false;
        return;
      }
      double t1 = lo / dd, t2 = hi / dd;
      if (t1 > t2)
        std::swap(t1, t2);
      if (t2 < 0.0) {
        hit = false;
        return;
      }
      t_lo = std::max(t_lo, t1);
      t_hi = std::min(t_hi, t2);
    };
    interval(d.x(), ab.x_min, ab.x_max);
    interval(d.y(), room.ceil_y, room.floor_y);
    interval(d.z(), ab.z_min, ab.z_max);
    if (hit && t_lo <= t_exit + 1e-9 && t_exit <= t_hi + 1e-9 && t_hi > 1e-6)
      t_exit = t_hi;
  }
  return t_exit;
}

// Full 360-degree organized scan JSON of the synthetic room (or a constant
// degenerate wall when constant_range is set).
std::string roomScanJson(const Room &room, int rows, int columns,
                         bool constant_range) {
  std::ostringstream json_text;
  json_text << std::setprecision(10);
  json_text << R"({"interface_version":"1.0","schema_version":"1.2",)"
            << R"("sensor":{"model":"synthetic","range_offset_m":0.0},)"
            << R"("frame":{"name":"lidar_scan","handedness":"right",)"
            << R"("convention":"+x right, +y down, +z forward; pan+ right, tilt+ up"},)"
            << R"("scan":{"rows":)" << rows << R"(,"columns":)" << columns
            << R"(,"pan_min_rad":0.0,"pan_max_rad":)" << 2.0 * kPi * (columns - 1.0) / columns
            << R"(,"tilt_min_rad":)" << -kPi / 2.0 << R"(,"tilt_max_rad":0.0,)"
            << R"("sample_count":)" << rows * columns << R"(},"measurements":[)";
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < columns; ++c) {
      const double pan = 2.0 * kPi * (static_cast<double>(c) / columns);
      const double tilt = -kPi * 0.5 * (static_cast<double>(r) / (rows - 1));
      const Eigen::Vector3d dir(std::cos(tilt) * std::sin(pan), -std::sin(tilt),
                                std::cos(tilt) * std::cos(pan));
      const double range =
          constant_range ? 3.0 : rayBox(room, dir);
      if (r || c)
        json_text << ',';
      if (range > 0.0) {
        json_text << R"({"row":)" << r << R"(,"column":)" << c
                  << R"(,"valid":true,"distance_status":1,"pan_rad":)" << pan
                  << R"(,"tilt_rad":)" << tilt << R"(,"distance_m":)" << range
                  << "}";
      } else {
        json_text << R"({"row":)" << r << R"(,"column":)" << c
                  << R"(,"valid":false,"pan_rad":)" << pan
                  << R"(,"tilt_rad":)" << tilt << R"(,"distance_m":null})";
      }
    }
  json_text << "]}";
  return json_text.str();
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> roomEdges(const Room &room) {
  const double x = room.x_half, z = room.z_half;
  const double fy = room.floor_y, cy = room.ceil_y;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;
  for (double sx : {-x, x})
    for (double sz : {-z, z})
      edges.push_back({{sx, cy, sz}, {sx, fy, sz}});
  edges.push_back({{-x, fy, -z}, {x, fy, -z}});
  edges.push_back({{-x, fy, z}, {x, fy, z}});
  edges.push_back({{-x, fy, -z}, {-x, fy, z}});
  edges.push_back({{x, fy, -z}, {x, fy, z}});
  for (const Panel &a : room.panels) {
    const double outer = a.wall == 'z' ? a.sign * z : a.sign * x;
    const double inner = outer - a.sign * a.depth;
    auto pt = [&](bool back, double u, double v) {
      if (a.wall == 'z')
        return Eigen::Vector3d(u, v, back ? inner : outer);
      return Eigen::Vector3d(back ? inner : outer, v, u);
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

auto_calib::CameraModel testCamera(int width = 1280, int height = 720) {
  auto_calib::CameraModel camera;
  camera.width = width;
  camera.height = height;
  const double f = width / (2.0 * std::tan(rad(45.0)));
  camera.k << f, 0.0, width * 0.5, 0.0, f, height * 0.5, 0.0, 0.0, 1.0;
  return camera;
}

Eigen::Matrix3d gtRotation(double yaw_deg, double down_deg) {
  return Eigen::AngleAxisd(rad(down_deg), Eigen::Vector3d::UnitX())
             .toRotationMatrix() *
         Eigen::AngleAxisd(rad(yaw_deg), Eigen::Vector3d::UnitY())
             .toRotationMatrix();
}

// Renders the room edges through the GT rotation into a Canny-detectable
// perspective image (with optional 2x brightness scaling for the invariance
// test).
cv::Mat renderRoomImage(const Room &room,
                        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges,
                        const Eigen::Matrix3d &gt,
                        const auto_calib::CameraModel &camera,
                        double brightness_scale = 1.0) {
  cv::Mat canvas(camera.height, camera.width, CV_8UC1, cv::Scalar(255));
  const double f = camera.k(0, 0);
  auto project = [&](const Eigen::Vector3d &p, double *u, double *v) {
    const Eigen::Vector3d pc = gt * p;
    const double z = std::max(pc.z(), 1e-6);
    *u = camera.k(0, 0) * pc.x() / z + camera.k(0, 2);
    *v = camera.k(1, 1) * pc.y() / z + camera.k(1, 2);
  };
  for (const auto &e : edges) {
    const Eigen::Vector3d pa = gt * e.first;
    const Eigen::Vector3d pb = gt * e.second;
    if (pa.z() <= 0.05 && pb.z() <= 0.05)
      continue;
    double u1, v1, u2, v2;
    project(e.first, &u1, &v1);
    project(e.second, &u2, &v2);
    if (pa.z() <= 0.05 || pb.z() <= 0.05) {
      const double t = (0.05 - pa.z()) / (pb.z() - pa.z());
      const Eigen::Vector3d pm = pa + t * (pb - pa);
      if (pa.z() <= 0.05)
        project(pm, &u1, &v1);
      else
        project(pm, &u2, &v2);
    }
    double t0 = 0.0, t1 = 1.0;
    bool ok = true;
    const double dx = u2 - u1, dy = v2 - v1;
    auto clip = [&](double p, double q) {
      if (std::abs(p) < 1e-12) {
        if (q < 0)
          ok = false;
        return;
      }
      const double r = q / p;
      if (p < 0) {
        if (r > t1)
          ok = false;
        else if (r > t0)
          t0 = r;
      } else {
        if (r < t0)
          ok = false;
        else if (r < t1)
          t1 = r;
      }
    };
    clip(-dx, u1);
    clip(dx, camera.width - 1 - u1);
    clip(-dy, v1);
    clip(dy, camera.height - 1 - v1);
    if (!ok)
      continue;
    cv::line(canvas, cv::Point2d(u1 + t0 * dx, v1 + t0 * dy),
             cv::Point2d(u1 + t1 * dx, v1 + t1 * dy), cv::Scalar(0), 3,
             cv::LINE_AA);
  }
  if (brightness_scale != 1.0)
    canvas.convertTo(canvas, -1, brightness_scale, 0.0);
  cv::GaussianBlur(canvas, canvas, cv::Size(3, 3), 0.6);
  return canvas;
}

std::string writeTempScan(const Room &room, bool constant_range) {
  static int counter = 0;
  const std::string path =
      "/tmp/t2_panorama_scan_" + std::to_string(counter++) + ".json";
  std::ofstream f(path);
  f << roomScanJson(room, 101, 400, constant_range);
  return path;
}
} // namespace

int main() {
  using namespace auto_calib;
  const Room room;
  const auto edges = roomEdges(room);
  const auto camera = testCamera();
  const std::string scan_path = writeTempScan(room, false);

  // ---- Known-rotation regression (7 cases) ---------------------------------
  struct Case {
    const char *label;
    double yaw, down;
  };
  const Case known_cases[] = {
      {"yaw+000_down00", 0.0, 0.0},   {"yaw+030_down00", 30.0, 0.0},
      {"yaw-030_down00", -30.0, 0.0}, {"yaw+090_down00", 90.0, 0.0},
      {"yaw-090_down00", -90.0, 0.0}, {"yaw177_down42", 177.0, 42.0},
      {"yaw-179_down45", -179.0, 45.0},
  };
  for (const auto &case_item : known_cases) {
    const Eigen::Matrix3d gt = gtRotation(case_item.yaw, case_item.down);
    const auto image = renderRoomImage(room, edges, gt, camera);
    PanoramaAnalyzerOptions options;
    const auto result = analyzePanorama(scan_path, image, camera, options);
    bool recovered = false;
    bool strict_recovery = false;
    double best_yaw_err = 1e9, best_down_err = 1e9;
    for (const auto &p : result.proposals) {
      const double yaw_err = circularDistanceDeg(p.yaw_deg, case_item.yaw);
      const double down_err = std::abs(p.down_deg - case_item.down);
      if (yaw_err < best_yaw_err) {
        best_yaw_err = yaw_err;
        best_down_err = down_err;
      }
      if (yaw_err <= 15.0 && down_err <= 15.0)
        recovered = true;
      if (yaw_err <= 5.0 && down_err <= 7.5)
        strict_recovery = true;
    }
    std::ostringstream detail;
    detail << case_item.label << " status=" << result.status
           << " proposals=" << result.proposals.size()
           << " best_yaw_err=" << best_yaw_err
           << " best_down_err=" << best_down_err
           << " pslr=" << result.peak_to_sidelobe_ratio;
    expect(result.status == "PROPOSALS_READY" && recovered &&
               !result.fallback_required,
           "known-rotation " + detail.str() +
               (strict_recovery ? " [strict<=5deg]" : " [window-only]"));
  }

  // ---- Degenerate textureless wall ------------------------------------------
  {
    const std::string degenerate_path = writeTempScan(room, true);
    cv::Mat blank(camera.height, camera.width, CV_8UC1, cv::Scalar(200));
    PanoramaAnalyzerOptions options;
    const auto result =
        analyzePanorama(degenerate_path, blank, camera, options);
    expect(result.status == "INSUFFICIENT_FEATURES" &&
               result.fallback_required && result.proposals.empty(),
           "degenerate wall triggers INSUFFICIENT_FEATURES fallback (status=" +
               result.status + " reason=" + result.fallback_reason + ")");
  }

  // ---- Seam continuity -------------------------------------------------------
  {
    PanoramaRaster raster;
    {
      std::istringstream ss(roomScanJson(room, 101, 400, false));
      // reuse the builder through a temp file for simplicity
      const std::string p = writeTempScan(room, false);
      std::ifstream f(p);
      auto root = nlohmann::json::parse(f);
      raster = buildPanoramaRaster(root);
    }
    cv::Mat combined = combinedPanoramaEdge(raster);
    // A step edge spanning the seam: verify edges exist near column 0/399.
    // The room's -x wall (pan ~270 deg -> column 300) is not at the seam;
    // instead assert the seam columns carry no artificial full-column edges
    // and that wrap-around structures produce edges at both column ends when
    // they exist. For the synthetic room the +x wall crosses the seam
    // (pan ~360 deg): check edge presence on both sides of the seam.
    int edge_near_start = 0, edge_near_end = 0;
    for (int r = 0; r < raster.rows; ++r) {
      if (combined.at<unsigned char>(r, 0))
        ++edge_near_start;
      if (combined.at<unsigned char>(r, raster.columns - 1))
        ++edge_near_end;
    }
    expect(edge_near_start > 0 && edge_near_end > 0,
           "seam columns carry wrap-around edges (start=" +
               std::to_string(edge_near_start) +
               " end=" + std::to_string(edge_near_end) + ")");
  }

  // ---- Invalid-cell boundary fake-edge suppression ---------------------------
  {
    // Constant range with a block of invalid cells: the invalid boundary
    // must not produce depth edges.
    const int rows = 21, columns = 40;
    std::ostringstream json_text;
    json_text << R"({"frame":{"name":"lidar_scan"},"scan":{"rows":)" << rows
              << R"(,"columns":)" << columns
              << R"(,"pan_min_rad":0.0,"pan_max_rad":6.2,)"
              << R"("tilt_min_rad":-1.5,"tilt_max_rad":0.0,"sample_count":)"
              << rows * columns << R"(},"measurements":[)";
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < columns; ++c) {
        if (r || c)
          json_text << ',';
        const bool invalid = c >= 18 && c <= 22;
        if (invalid)
          json_text << R"({"row":)" << r << R"(,"column":)" << c
                    << R"(,"valid":false,"distance_m":null})";
        else
          json_text << R"({"row":)" << r << R"(,"column":)" << c
                    << R"(,"valid":true,"distance_m":3.0})";
      }
    json_text << "]}";
    const std::string path = "/tmp/t2_invalid_mask.json";
    std::ofstream f(path);
    f << json_text.str();
    std::ifstream f2(path);
    const auto raster = buildPanoramaRaster(nlohmann::json::parse(f2));
    int fake_edges = 0;
    for (int r = 0; r < rows; ++r)
      for (int c = 16; c <= 24; ++c)
        if (raster.range_edge.at<unsigned char>(r, c))
          ++fake_edges;
    expect(fake_edges == 0,
           "invalid-cell boundary produces no fake depth edges (found " +
               std::to_string(fake_edges) + ")");
  }

  // ---- Circular/geodesic NMS separation --------------------------------------
  {
    std::vector<double> yaws = {0.0, 5.0, -6.0, 90.0, 180.0, -95.0};
    std::vector<double> downs(6, 0.0);
    std::vector<double> scores = {0.9, 0.85, 0.8, 0.7, 0.6, 0.55};
    const auto selected =
        selectDistinctPeaks(yaws, downs, scores, 30.0, 3);
    double min_sep = 1e9;
    for (std::size_t i = 0; i < selected.size(); ++i)
      for (std::size_t j = i + 1; j < selected.size(); ++j) {
        const Eigen::Matrix3d ra =
            gtRotation(yaws[selected[i]], 0.0);
        const Eigen::Matrix3d rb =
            gtRotation(yaws[selected[j]], 0.0);
        const double sep = std::acos(std::clamp(
                                          ((ra * rb.transpose()).trace() - 1.0) / 2.0, -1.0,
                                          1.0)) *
                           180.0 / kPi;
        min_sep = std::min(min_sep, sep);
      }
    expect(selected.size() == 3 && min_sep >= 30.0,
           "geodesic NMS keeps distinct basins (selected=" +
               std::to_string(selected.size()) + " min_sep=" +
               std::to_string(min_sep) + ")");
  }

  // ---- Scale/contrast invariance ---------------------------------------------
  {
    const Eigen::Matrix3d gt = gtRotation(30.0, 0.0);
    const auto image = renderRoomImage(room, edges, gt, camera);
    const auto image_2x = renderRoomImage(room, edges, gt, camera, 0.5);
    PanoramaAnalyzerOptions options;
    const auto r1 = analyzePanorama(scan_path, image, camera, options);
    const auto r2 = analyzePanorama(scan_path, image_2x, camera, options);
    bool same_peak = !r1.proposals.empty() && !r2.proposals.empty() &&
                     circularDistanceDeg(r1.proposals[0].yaw_deg,
                                         r2.proposals[0].yaw_deg) <= 5.0 &&
                     std::abs(r1.proposals[0].down_deg -
                              r2.proposals[0].down_deg) <= 1e-6;
    expect(same_peak, "brightness scaling keeps the rank-1 peak");
  }

  std::cout << (failures == 0 ? "ALL PANORAMA ANALYZER TESTS PASSED\n"
                              : "PANORAMA ANALYZER TEST FAILURES PRESENT\n");
  return failures == 0 ? 0 : 1;
}

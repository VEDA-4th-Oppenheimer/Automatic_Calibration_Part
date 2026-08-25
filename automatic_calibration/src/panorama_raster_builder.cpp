#include "auto_calib/panorama_raster_builder.hpp"

#include <Eigen/Eigenvalues>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>

namespace auto_calib {
namespace {
constexpr double kPi = 3.14159265358979323846;

bool measurementBool(const nlohmann::json &m, const char *key) {
  const auto it = m.find(key);
  return it != m.end() && it->is_boolean() && it->get<bool>();
}
} // namespace

PanoramaRaster buildPanoramaRaster(const nlohmann::json &root) {
  PanoramaRaster out;
  if (!root.contains("frame") ||
      root.at("frame").value("name", "") != "lidar_scan")
    throw std::runtime_error("Expected frame.name=lidar_scan");
  if (!root.contains("scan") || !root.contains("measurements"))
    throw std::runtime_error("LiDAR JSON requires scan and measurements");
  const auto &scan = root.at("scan");
  out.rows = scan.at("rows").get<int>();
  out.columns = scan.at("columns").get<int>();
  const int count = scan.at("sample_count").get<int>();
  if (out.rows <= 0 || out.columns <= 0 || count != out.rows * out.columns)
    throw std::runtime_error("organized shape/sample_count mismatch");
  out.pan_min_rad = scan.value("pan_min_rad", 0.0);
  out.pan_max_rad = scan.value("pan_max_rad", 2.0 * kPi);
  out.tilt_min_rad = scan.value("tilt_min_rad", -kPi / 2.0);
  out.tilt_max_rad = scan.value("tilt_max_rad", 0.0);

  out.range_m = cv::Mat(out.rows, out.columns, CV_32F, cv::Scalar(0.0f));
  out.valid = cv::Mat(out.rows, out.columns, CV_8UC1, cv::Scalar(0));
  const double range_offset =
      root.contains("sensor") && root.at("sensor").contains("range_offset_m")
          ? root.at("sensor").at("range_offset_m").get<double>()
          : 0.0;
  std::vector<unsigned char> seen(static_cast<std::size_t>(count), 0);
  for (const auto &m : root.at("measurements")) {
    const int r = m.at("row").get<int>();
    const int c = m.at("column").get<int>();
    if (r < 0 || r >= out.rows || c < 0 || c >= out.columns)
      throw std::runtime_error("measurement cell outside raster");
    auto &seen_cell = seen[static_cast<std::size_t>(r) * out.columns + c];
    if (seen_cell)
      throw std::runtime_error("duplicate raster cell");
    seen_cell = 1;
    const auto dist_it = m.find("distance_m");
    const bool has_range = dist_it != m.end() && dist_it->is_number();
    if (!measurementBool(m, "valid") || !has_range)
      continue;
    const double range = dist_it->get<double>() + range_offset;
    if (!std::isfinite(range) || range <= 0)
      continue;
    out.range_m.at<float>(r, c) = static_cast<float>(range);
    out.valid.at<unsigned char>(r, c) = 255;
  }
  if (std::count(seen.begin(), seen.end(), 0) != 0)
    throw std::runtime_error("missing measurement cell");
  out.coverage = static_cast<double>(cv::countNonZero(out.valid)) / count;

  const auto valid_at = [&](int r, int c) {
    return out.valid.at<unsigned char>(r, c) != 0;
  };
  const auto range_at = [&](int r, int c) {
    return static_cast<double>(out.range_m.at<float>(r, c));
  };
  // 3D point of a cell under the PAN_TILT contract:
  // x = r cos(tilt) sin(pan), y = -r sin(tilt), z = r cos(tilt) cos(pan).
  const auto point_at = [&](int r, int c, Eigen::Vector3d *p) {
    const double range = range_at(r, c);
    // Endpoint-inclusive grid contract: column 0 <-> pan_min, column
    // columns-1 <-> pan_max (matching the PAN_TILT device sweep).
    const double pan = out.pan_min_rad +
                       (out.pan_max_rad - out.pan_min_rad) *
                           (static_cast<double>(c) / (out.columns - 1));
    const double tilt = out.tilt_max_rad -
                        (out.tilt_max_rad - out.tilt_min_rad) *
                            (static_cast<double>(r) / (out.rows - 1));
    *p = Eigen::Vector3d(range * std::cos(tilt) * std::sin(pan),
                         -range * std::sin(tilt),
                         range * std::cos(tilt) * std::cos(pan));
  };

  // Oriented PCA normals on 3x3 neighborhoods (planar cells only).
  // Column indexing is circular so seam columns (0 and columns-1) receive
  // normals exactly like interior columns.
  std::vector<Eigen::Vector3d> normals(static_cast<std::size_t>(out.rows) *
                                       out.columns);
  std::vector<char> has_normal(static_cast<std::size_t>(out.rows) *
                               out.columns, 0);
  for (int r = 1; r + 1 < out.rows; ++r)
    for (int c = 0; c < out.columns; ++c) {
      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      int n = 0;
      for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc) {
          const int cc = (c + dc + out.columns) % out.columns;
          if (!valid_at(r + dr, cc)) {
            n = -1;
            dr = 2;
            break;
          }
          Eigen::Vector3d p;
          point_at(r + dr, cc, &p);
          centroid += p;
          ++n;
        }
      if (n != 9)
        continue;
      centroid /= 9.0;
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc) {
          const int cc = (c + dc + out.columns) % out.columns;
          Eigen::Vector3d p;
          point_at(r + dr, cc, &p);
          const Eigen::Vector3d d = p - centroid;
          cov += d * d.transpose();
        }
      cov /= 9.0;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
      if (solver.info() != Eigen::Success)
        continue;
      const double lmin = solver.eigenvalues()(0);
      const double lmax = solver.eigenvalues()(2);
      if (lmax <= 1e-12 || lmin > 0.05 * lmax)
        continue;
      Eigen::Vector3d normal = solver.eigenvectors().col(0);
      Eigen::Vector3d p;
      point_at(r, c, &p);
      if (normal.dot(p) > 0)
        normal = -normal; // sensor-facing: n . p < 0
      normals[static_cast<std::size_t>(r) * out.columns + c] = normal;
      has_normal[static_cast<std::size_t>(r) * out.columns + c] = 1;
    }

  out.range_edge = cv::Mat(out.rows, out.columns, CV_8UC1, cv::Scalar(0));
  out.normal_edge = cv::Mat(out.rows, out.columns, CV_8UC1, cv::Scalar(0));
  out.plane_intersection =
      cv::Mat(out.rows, out.columns, CV_8UC1, cv::Scalar(0));
  constexpr double kCurvatureThreshold = 0.06;
  constexpr double kCreaseCosine = std::cos(20.0 * kPi / 180.0);
  constexpr double kPlaneBoundaryCosine = std::cos(12.0 * kPi / 180.0);
  const auto nearest_normal = [&](int r, int c, int dr,
                                  int dc) -> const Eigen::Vector3d * {
    // Search strictly away from the cell: starting at step 0 would return
    // the cell's own normal in both directions and block crease pairing.
    for (int step = 1; step <= 4; ++step) {
      const int rr = r + step * dr;
      const int cc = (c + step * dc + out.columns) % out.columns;
      if (rr < 0 || rr >= out.rows)
        return nullptr;
      const std::size_t idx = static_cast<std::size_t>(rr) * out.columns + cc;
      if (has_normal[idx])
        return &normals[idx];
    }
    return nullptr;
  };
  for (int r = 0; r < out.rows; ++r)
    for (int c = 0; c < out.columns; ++c) {
      if (!valid_at(r, c))
        continue;
      const double center = range_at(r, c);
      if (center <= 0)
        continue;
      bool depth_edge = false;
      bool crease = false;
      bool plane_boundary = false;
      // Horizontal channel with circular (seam-safe) neighbours.
      {
        const int cl = (c + out.columns - 1) % out.columns;
        const int cr = (c + 1) % out.columns;
        if (valid_at(r, cl) && valid_at(r, cr)) {
          const double left = range_at(r, cl);
          const double right = range_at(r, cr);
          if (std::abs(right - 2.0 * center + left) / center >=
              kCurvatureThreshold)
            depth_edge = true;
        }
      }
      // Vertical channel without row wrap.
      if (!depth_edge && r > 0 && r + 1 < out.rows &&
          valid_at(r - 1, c) && valid_at(r + 1, c)) {
        const double up = range_at(r - 1, c);
        const double down = range_at(r + 1, c);
        if (std::abs(down - 2.0 * center + up) / center >=
            kCurvatureThreshold)
          depth_edge = true;
      }
      // Normal channels with windowed seam-safe pairing.
      const Eigen::Vector3d *nl = nearest_normal(r, c, 0, -1);
      const Eigen::Vector3d *nr = nearest_normal(r, c, 0, 1);
      const Eigen::Vector3d *nu = nearest_normal(r, c, -1, 0);
      const Eigen::Vector3d *nd = nearest_normal(r, c, 1, 0);
      if (nl && nr && nl != nr) {
        const double d = nl->dot(*nr);
        if (d < kCreaseCosine)
          crease = true;
        else if (d < kPlaneBoundaryCosine)
          plane_boundary = true;
      }
      if (nu && nd && nu != nd) {
        const double d = nu->dot(*nd);
        if (d < kCreaseCosine)
          crease = true;
        else if (d < kPlaneBoundaryCosine)
          plane_boundary = true;
      }
      if (std::getenv("T2_RASTER_DEBUG") && c <= 1 && r >= 15 && r <= 23) {
        std::fprintf(stderr, "[R] r=%d c=%d valid=%d nl=%d nr=%d nu=%d nd=%d depth=%d crease=%d\n",
                     r, c, (int)valid_at(r, c), nl != nullptr, nr != nullptr,
                     nu != nullptr, nd != nullptr, (int)depth_edge, (int)crease);
        if (nu && nd)
          std::fprintf(stderr, "[R]   nu.ndot=%.3f\n", nu->dot(*nd));
      }
      if (depth_edge)
        out.range_edge.at<unsigned char>(r, c) = 255;
      if (crease)
        out.normal_edge.at<unsigned char>(r, c) = 255;
      if (plane_boundary)
        out.plane_intersection.at<unsigned char>(r, c) = 255;
    }
  return out;
}

cv::Mat combinedPanoramaEdge(const PanoramaRaster &raster) {
  cv::Mat combined = cv::Mat::zeros(raster.rows, raster.columns, CV_8UC1);
  cv::bitwise_or(combined, raster.range_edge, combined);
  cv::bitwise_or(combined, raster.normal_edge, combined);
  cv::bitwise_or(combined, raster.plane_intersection, combined);
  return combined;
}

} // namespace auto_calib

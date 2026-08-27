#include "auto_calib/hybrid_orientation_analyzer.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

namespace {

std::filesystem::path writeConstantScan() {
  constexpr int rows = 11;
  constexpr int columns = 40;
  constexpr double pi = 3.14159265358979323846;
  const auto path = std::filesystem::temp_directory_path() /
                    "v3_hybrid_constant_scan.json";
  std::ofstream out(path);
  out << R"({"interface_version":"1.0","schema_version":"1.2",)"
      << R"("sensor":{"model":"synthetic","range_offset_m":0.0},)"
      << R"("frame":{"name":"lidar_scan","handedness":"right",)"
      << R"("convention":"+x right, +y down, +z forward; pan+ right, tilt+ up"},)"
      << R"("scan":{"rows":)" << rows << R"(,"columns":)" << columns
      << R"(,"pan_min_rad":0.0,"pan_max_rad":)"
      << 2.0 * pi * (columns - 1.0) / columns
      << R"(,"tilt_min_rad":)" << -pi / 2.0
      << R"(,"tilt_max_rad":0.0,"sample_count":)" << rows * columns
      << R"(},"measurements":[)";
  for (int row = 0; row < rows; ++row) {
    for (int column = 0; column < columns; ++column) {
      if (row || column)
        out << ',';
      const double pan = 2.0 * pi * column / columns;
      const double tilt = -pi * 0.5 * row / (rows - 1.0);
      out << R"({"row":)" << row << R"(,"column":)" << column
          << R"(,"valid":true,"distance_status":1,"pan_rad":)" << pan
          << R"(,"tilt_rad":)" << tilt << R"(,"distance_m":3.0})";
    }
  }
  out << "]}";
  return path;
}

} // namespace

int main() {
  constexpr double pi = 3.14159265358979323846;
  constexpr int camera_bins = 64;
  constexpr int lidar_bins = 401;
  std::vector<double> camera(camera_bins, 0.0);
  std::vector<double> lidar(lidar_bins, 0.0);
  camera[8] = 1.0;
  camera[31] = 0.8;
  camera[52] = 0.6;

  Eigen::Matrix3d k = Eigen::Matrix3d::Identity();
  k(0, 0) = 320.0;
  k(0, 2) = 320.0;
  const double expected_yaw = 70.0;
  for (int i = 0; i < camera_bins; ++i) {
    const double u = (i + 0.5) / camera_bins * 640.0;
    const double ray = std::atan((u - k(0, 2)) / k(0, 0));
    double pan = ray - expected_yaw * pi / 180.0;
    while (pan < -pi)
      pan += 2.0 * pi;
    while (pan >= pi)
      pan -= 2.0 * pi;
    const int column = static_cast<int>(
        std::lround((pan + pi) / (2.0 * pi) * (lidar_bins - 1)));
    lidar[static_cast<std::size_t>((column + lidar_bins - 1) %
                                   (lidar_bins - 1))] += camera[i] * 0.25;
    lidar[static_cast<std::size_t>(column % (lidar_bins - 1))] +=
        camera[i] * 0.5;
    lidar[static_cast<std::size_t>((column + 1) % (lidar_bins - 1))] +=
        camera[i] * 0.25;
  }
  lidar.back() = lidar.front();
  std::vector<double> yaws;
  for (int yaw = -180; yaw < 180; ++yaw)
    yaws.push_back(yaw);
  const auto scores =
      auto_calib::scoreAngularSignatures(camera, lidar, k, 640, -pi, pi, yaws);
  const auto best = static_cast<int>(
      std::max_element(scores.begin(), scores.end()) - scores.begin());
  const double recovered = yaws[static_cast<std::size_t>(best)];
  if (std::abs(recovered - expected_yaw) > 1.0 || scores.size() != 360) {
    std::cerr << "signature recovery failed: " << recovered << "\n";
    return 1;
  }

  auto_calib::CameraModel camera_model;
  camera_model.width = 640;
  camera_model.height = 480;
  camera_model.k << 320.0, 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0;
  const cv::Mat textureless(camera_model.height, camera_model.width, CV_8UC1,
                            cv::Scalar(127));
  const auto scan_path = writeConstantScan();
  const auto degenerate = auto_calib::analyzeHybridOrientation(
      scan_path, textureless, camera_model);
  std::filesystem::remove(scan_path);
  if (!degenerate.fallback_required ||
      degenerate.status != "INSUFFICIENT_FEATURES" ||
      degenerate.fallback_reason != "CAMERA_EDGE_INSUFFICIENT") {
    std::cerr << "degenerate fallback failed: " << degenerate.status << ' '
              << degenerate.fallback_reason << "\n";
    return 1;
  }

  std::cout << "hybrid analyzer signature test passed\n";
}

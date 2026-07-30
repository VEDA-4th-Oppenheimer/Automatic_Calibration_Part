#include "auto_calib/calibration_core.hpp"
#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>
namespace {
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
Eigen::Vector3d ray(double pan, double tilt) {
  double c = std::cos(tilt);
  return {c * std::sin(pan), -std::sin(tilt), c * std::cos(pan)};
}
} // namespace
int main() {
  try {
    auto_calib::ScanConfig sc;
    sc.rows = 31;
    sc.columns = 41;
    sc.pan_min = -.35;
    sc.pan_max = .35;
    sc.tilt_min = -.25;
    sc.tilt_max = .25;
    sc.noise_stddev = 0;
    sc.dropout = 0;
    std::vector<Eigen::Vector3d> source;
    for (std::uint32_t r = 0; r < sc.rows; ++r)
      for (std::uint32_t c = 0; c < sc.columns; ++c) {
        double pan = sc.pan_min +
                     (sc.pan_max - sc.pan_min) * c / (sc.columns - 1),
               tilt = sc.tilt_max -
                      (sc.tilt_max - sc.tilt_min) * r / (sc.rows - 1);
        double range = (c > 10 && c < 30 && r > 7 && r < 23) ? 3.0 : 5.0;
        if (c > 16 && c < 24 && r > 11 && r < 19)
          range = 2.2;
        source.push_back(ray(pan, tilt) * range);
      }
    auto scan = auto_calib::generateScan(source, {}, sc);
    auto_calib::CalibrationConfig cfg;
    cfg.minimum_lidar_edge_points = 10;
    cfg.minimum_camera_edge_pixels = 10;
    auto edges = auto_calib::extractLidarEdgePoints(scan, cfg);
    require(edges.size() > 20, "LiDAR edges missing");
    auto_calib::CameraModel camera;
    camera.width = 400;
    camera.height = 400;
    camera.k << 360, 0, 200, 0, 360, 200, 0, 0, 1;
    cv::Mat image(400, 400, CV_8UC3, cv::Scalar(0, 0, 0));
    for (const auto &p : edges) {
      int u = std::lround(camera.k(0, 0) * p.x() / p.z() + camera.k(0, 2)),
          v = std::lround(camera.k(1, 1) * p.y() / p.z() + camera.k(1, 2));
      if (u >= 0 && v >= 0 && u < 400 && v < 400)
        cv::circle(image, {u, v}, 2, {255, 255, 255}, -1);
    }
    auto initial =
        auto_calib::makeTransform({.01, -.01, .015}, {.005, -.006, .008});
    cfg.maximum_mean_edge_distance_px = 25;
    cfg.minimum_projected_ratio = .5;
    cfg.maximum_solver_iterations = 30;
    auto result =
        auto_calib::calibrateExtrinsic(image, camera, scan, initial, cfg);
    require(result.metrics.camera_edge_pixels > 10, "Camera edges missing");
    require(result.metrics.final_mean_edge_distance_px <=
                result.metrics.initial_mean_edge_distance_px * 1.05,
            "Objective did not improve");
    require(result.success, "Synthetic calibration failed");
    std::vector<auto_calib::CalibrationObservation> observations = {
        {image, camera, scan}, {image, camera, scan}};
    auto multi =
        auto_calib::calibrateExtrinsicMultiScene(observations, initial, cfg);
    require(multi.success, "Multi-scene calibration failed");
    auto strict = cfg;
    strict.maximum_rotation_update_rad = 0.0;
    strict.maximum_translation_update_m = 0.0;
    auto rejected_update =
        auto_calib::calibrateExtrinsicMultiScene(observations, initial, strict);
    require(!rejected_update.success, "Unsafe update gate failed");
    auto fallback_error = auto_calib::calculatePoseError(
        rejected_update.estimated_t_camera_lidar, initial);
    require(fallback_error.translation_m < 1e-12 &&
                fallback_error.rotation_deg < 1e-12,
            "Rejected candidate did not fall back to prior");
    cv::Mat blank(400, 400, CV_8UC3, cv::Scalar(0));
    auto rejected =
        auto_calib::calibrateExtrinsic(blank, camera, scan, {}, cfg);
    require(!rejected.success &&
                rejected.reason_code == "CAMERA_EDGE_INSUFFICIENT",
            "Blank image gate failed");
    require(rejected.metrics.runtime_ms > 0.0,
            "Rejected input runtime was not recorded");
    auto error = auto_calib::calculatePoseError({}, {});
    require(error.translation_m < 1e-12 && error.rotation_deg < 1e-12,
            "Pose error failed");
    std::cout << "All Calibration Core tests passed.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Test failure: " << e.what() << '\n';
    return 1;
  }
}

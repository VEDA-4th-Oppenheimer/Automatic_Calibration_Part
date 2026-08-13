#include "top_view/top_view.hpp"

#include <cmath>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>

namespace {
void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}

bool close(double a, double b, double tolerance = 1e-9) {
  return std::abs(a - b) <= tolerance;
}
} // namespace

int main() {
  try {
    const nlohmann::json automatic = {
        {"estimated",
         {{"parent_frame", "camera_optical"},
          {"child_frame", "lidar_scan"},
          {"rotation_matrix",
           {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
          {"translation_m", {0.0, 0.0, 5.0}}}}};
    const auto t_camera_lidar = top_view::parseTransform(automatic);
    require(t_camera_lidar.parent_frame == "camera_optical" &&
                t_camera_lidar.child_frame == "lidar_scan",
            "Automatic transform key was not detected");

    const nlohmann::json manual = {{"method", "charuco_2d_image_only_pose"},
                                   {"extrinsic",
                                    {{"parent_frame", "camera_optical"},
                                     {"child_frame", "marker_board"},
                                     {"quaternion_xyzw", {0.0, 0.0, 0.0, 1.0}},
                                     {"translation_m", {0.0, 0.0, 5.0}}}}};
    const auto t_camera_board = top_view::parseTransform(manual);
    require(t_camera_board.child_frame == "marker_board",
            "Manual extrinsic key was not detected");

    auto t_lidar_ground =
        top_view::identityPlaneTransform("lidar_scan", "ground_plane");
    t_lidar_ground.translation_m = {1.0, 2.0, 0.0};
    const auto t_camera_ground =
        top_view::compose(t_camera_lidar, t_lidar_ground);
    require(close(t_camera_ground.translation_m[0], 1.0) &&
                close(t_camera_ground.translation_m[1], 2.0) &&
                close(t_camera_ground.translation_m[2], 5.0),
            "Transform composition is incorrect");

    const nlohmann::json camera_json = {
        {"camera",
         {{"model", "test_camera"},
          {"resolution", {100, 100}},
          {"intrinsic_profile_id", "test"},
          {"intrinsic",
           {{"fx", 100.0}, {"fy", 100.0}, {"cx", 50.0}, {"cy", 50.0}}}}}};
    const auto camera = top_view::parseCameraModel(camera_json);
    top_view::TopViewConfig config;
    config.x_min_m = -1.0;
    config.x_max_m = 1.0;
    config.y_min_m = -1.0;
    config.y_max_m = 1.0;
    config.pixels_per_meter = 50.0;
    config.grid_spacing_m = 0.5;
    config.draw_grid = false;
    const auto homography =
        top_view::imageFromTopViewHomography(camera, t_camera_lidar, config);
    const cv::Vec3d center = homography * cv::Vec3d(50.0, 50.0, 1.0);
    require(close(center[0] / center[2], 50.0, 1e-6) &&
                close(center[1] / center[2], 50.0, 1e-6),
            "Top-view center does not map to camera principal point");

    cv::Mat image(100, 100, CV_8UC3, cv::Scalar(20, 80, 160));
    const auto rendered =
        top_view::renderTopView(image, camera, t_camera_lidar, config);
    require(rendered.cols == 100 && rendered.rows == 100,
            "Rendered Top-View has unexpected size");
    const auto metadata = top_view::renderMetadata(
        camera, t_camera_lidar, top_view::identityPlaneTransform("lidar_scan"),
        t_camera_lidar, config);
    require(metadata.at("schema_version") == "1.0" &&
                metadata.at("top_view").at("output_size").at(0) == 100,
            "Render metadata is incomplete");

    std::cout << "top_view_tests: PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "top_view_tests: " << error.what() << '\n';
    return 1;
  }
}

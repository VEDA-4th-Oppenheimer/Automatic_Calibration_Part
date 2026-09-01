#include "manual_marker/marker_calibration.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}
} // namespace

int main() {
  try {
    manual_marker::BoardConfig board;
    board.squares_x = 7;
    board.squares_y = 5;
    board.square_length_m = 0.04;
    board.marker_length_m = 0.03;
    board.minimum_charuco_corners = 6;
    const cv::Mat image =
        manual_marker::generateBoardImage(board, 1400, 1000, 30);
    require(!image.empty(), "ChArUco image generation failed");
    const auto detection = manual_marker::detect(image, "generated.png", board);
    require(detection.accepted, "Generated ChArUco board was not detected");
    require(detection.charuco_corner_count >= 12,
            "Too few generated ChArUco corners were detected");

    manual_marker::BoardConfig tablet_board = board;
    constexpr double tablet_width_m = 0.235828328343815;
    constexpr double tablet_pixels_per_meter = 2560.0 / tablet_width_m;
    tablet_board.square_length_m = 260.0 / tablet_pixels_per_meter;
    tablet_board.marker_length_m = 195.0 / tablet_pixels_per_meter;
    const cv::Mat tablet_image = manual_marker::generateDisplayBoardImage(
        tablet_board, 2560, 1600, 260, tablet_pixels_per_meter, 0.1);
    require(tablet_image.cols == 2560 && tablet_image.rows == 1600,
            "Galaxy Tab S7 canvas resolution is wrong");
    const auto tablet_detection =
        manual_marker::detect(tablet_image, "galaxy_tab_s7.png", tablet_board);
    require(tablet_detection.accepted,
            "Galaxy Tab S7 ChArUco board was not detected");
    require(tablet_detection.charuco_corner_count >= 12,
            "Too few tablet ChArUco corners were detected");

    manual_marker::CameraModel camera;
    camera.width = image.cols;
    camera.height = image.rows;
    camera.k = {1200.0, 0.0,    image.cols / 2.0,
                0.0,    1200.0, image.rows / 2.0,
                0.0,    0.0,    1.0};
    camera.distortion = cv::Mat::zeros(1, 5, CV_64F);
    const auto pose = manual_marker::estimateBoardPose(image, "generated.png",
                                                       board, camera, 3.0);
    require(pose.solved, "Generated ChArUco board pose failed");
    require(pose.reprojection_rmse_px < 1.0,
            "Generated board pose reprojection error is too high");
    require(pose.t_camera_board.parent_frame == "camera_optical" &&
                pose.t_camera_board.child_frame == "marker_board",
            "Manual marker pose frame contract failed");

    manual_marker::Transform t_camera_board;
    t_camera_board.parent_frame = "camera_optical";
    t_camera_board.child_frame = "marker_board";
    t_camera_board.translation_m = {0.1, 0.2, 1.0};
    manual_marker::Transform t_lidar_board;
    t_lidar_board.parent_frame = "lidar_scan";
    t_lidar_board.child_frame = "marker_board";
    t_lidar_board.translation_m = {0.1, 0.2, 1.0};
    const auto t_camera_lidar = manual_marker::compose(
        t_camera_board, manual_marker::inverse(t_lidar_board));
    require(t_camera_lidar.parent_frame == "camera_optical" &&
                t_camera_lidar.child_frame == "lidar_scan",
            "Reference frame composition failed");
    require(cv::norm(t_camera_lidar.translation_m) < 1e-12,
            "Reference transform composition numeric failure");
    require(manual_marker::rotationDistanceDeg(t_camera_lidar.rotation,
                                               cv::Matx33d::eye()) < 1e-12,
            "Rotation comparison failed");
    std::cout << "manual_marker_tests: PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "manual_marker_tests: " << error.what() << '\n';
    return 1;
  }
}

#include "manual_marker/marker_calibration.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace fs = std::filesystem;
using Args = std::unordered_map<std::string, std::string>;

namespace {
cv::Rect parseRoi(const std::string &value, const cv::Size &image_size) {
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
  char trailing = '\0';
  if (std::sscanf(value.c_str(), "%d,%d,%d,%d%c", &x, &y, &width,
                  &height, &trailing) != 4)
    throw std::invalid_argument("--roi must be x,y,width,height");
  if (x < 0 || y < 0 || width <= 0 || height <= 0 ||
      width > image_size.width || height > image_size.height ||
      x > image_size.width - width || y > image_size.height - height)
    throw std::invalid_argument("--roi is outside the input image");
  return {x, y, width, height};
}
} // namespace

int main(int argc, char **argv) {
  try {
    Args args;
    for (int index = 1; index < argc; ++index) {
      const std::string key = argv[index];
      if (key == "--help") {
        std::cout << "estimate_marker_pose --board board.json --camera "
                     "camera_intrinsic.json --image image.png --output-dir "
                     "DIR [--maximum-rms-px 3] "
                     "[--roi x,y,width,height]\n";
        return 0;
      }
      if (index + 1 >= argc)
        throw std::invalid_argument("Missing value for " + key);
      args[key] = argv[++index];
    }
    for (const char *key : {"--board", "--camera", "--image", "--output-dir"})
      if (!args.count(key))
        throw std::invalid_argument(std::string("Missing option: ") + key);
    const auto board = manual_marker::loadBoardConfig(args.at("--board"));
    const auto camera = manual_marker::loadCamera(args.at("--camera"));
    const cv::Mat image = cv::imread(args.at("--image"), cv::IMREAD_COLOR);
    if (image.empty())
      throw std::runtime_error("Cannot read input image");
    if (image.cols != camera.width || image.rows != camera.height)
      throw std::runtime_error(
          "Image resolution does not match intrinsic calibration");
    cv::Mat pose_image = image;
    auto pose_camera = camera;
    cv::Rect roi{0, 0, image.cols, image.rows};
    const bool uses_roi = args.count("--roi");
    if (uses_roi) {
      roi = parseRoi(args.at("--roi"), image.size());
      pose_image = image(roi).clone();
      pose_camera.width = roi.width;
      pose_camera.height = roi.height;
      pose_camera.k(0, 2) -= roi.x;
      pose_camera.k(1, 2) -= roi.y;
    }
    const double maximum_rms = args.count("--maximum-rms-px")
                                   ? std::stod(args.at("--maximum-rms-px"))
                                   : 3.0;
    const auto result = manual_marker::estimateBoardPose(
        pose_image, args.at("--image"), board, pose_camera, maximum_rms);
    const fs::path output = args.at("--output-dir");
    fs::create_directories(output);
    auto report = manual_marker::poseResultToJson(result, board);
    report["provenance"] = {
        {"board_file", fs::absolute(args.at("--board")).string()},
        {"camera_file", fs::absolute(args.at("--camera")).string()},
        {"image_file", fs::absolute(args.at("--image")).string()}};
    if (uses_roi) {
      report["provenance"]["roi_xywh"] =
          {roi.x, roi.y, roi.width, roi.height};
      report["provenance"]["roi_coordinate_contract"] =
          "crop-local pixels with cx/cy shifted; pose remains camera_optical";
    }
    std::ofstream(output / "marker_pose_result.json")
        << std::setw(2) << report << '\n';
    cv::Mat overlay =
        manual_marker::drawDetection(pose_image, result.detection, &result);
    if (uses_roi) {
      cv::Mat full_overlay = image.clone();
      overlay.copyTo(full_overlay(roi));
      cv::rectangle(full_overlay, roi, cv::Scalar(0, 255, 255), 2);
      overlay = full_overlay;
    }
    cv::imwrite((output / "marker_pose_overlay.png").string(), overlay);
    std::ofstream summary(output / "marker_pose_report.md");
    summary << "# 2D Marker Camera Pose\n\n"
            << "- Status: `" << result.status << "`\n"
            << "- Reason: `" << result.reason_code << "`\n"
            << "- Output: `T_camera_marker_board`\n"
            << "- Reprojection RMS: " << result.reprojection_rmse_px << " px\n"
            << "- ChArUco corners: " << result.detection.charuco_corner_count
            << "\n";
    if (uses_roi)
      summary << "- ROI: `" << roi.x << ',' << roi.y << ',' << roi.width
              << ',' << roi.height << "`\n";
    std::cout << std::setw(2) << report << '\n';
    return result.solved ? 0 : 2;
  } catch (const std::exception &error) {
    std::cerr << "estimate_marker_pose: " << error.what() << '\n';
    return 1;
  }
}

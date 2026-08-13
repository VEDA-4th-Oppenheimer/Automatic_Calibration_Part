#include "manual_marker/marker_calibration.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace fs = std::filesystem;
using Args = std::unordered_map<std::string, std::string>;

int main(int argc, char **argv) {
  try {
    Args args;
    for (int index = 1; index < argc; ++index) {
      const std::string key = argv[index];
      if (key == "--help") {
        std::cout << "estimate_marker_pose --board board.json --camera "
                     "camera_intrinsic.json --image image.png --output-dir "
                     "DIR [--maximum-rms-px 3]\n";
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
    const double maximum_rms = args.count("--maximum-rms-px")
                                   ? std::stod(args.at("--maximum-rms-px"))
                                   : 3.0;
    const auto result = manual_marker::estimateBoardPose(
        image, args.at("--image"), board, camera, maximum_rms);
    const fs::path output = args.at("--output-dir");
    fs::create_directories(output);
    auto report = manual_marker::poseResultToJson(result, board);
    report["provenance"] = {
        {"board_file", fs::absolute(args.at("--board")).string()},
        {"camera_file", fs::absolute(args.at("--camera")).string()},
        {"image_file", fs::absolute(args.at("--image")).string()}};
    std::ofstream(output / "marker_pose_result.json")
        << std::setw(2) << report << '\n';
    cv::imwrite((output / "marker_pose_overlay.png").string(),
                manual_marker::drawDetection(image, result.detection, &result));
    std::ofstream summary(output / "marker_pose_report.md");
    summary << "# 2D Marker Camera Pose\n\n"
            << "- Status: `" << result.status << "`\n"
            << "- Reason: `" << result.reason_code << "`\n"
            << "- Output: `T_camera_marker_board`\n"
            << "- Reprojection RMS: " << result.reprojection_rmse_px << " px\n"
            << "- ChArUco corners: " << result.detection.charuco_corner_count
            << "\n";
    std::cout << std::setw(2) << report << '\n';
    return result.solved ? 0 : 2;
  } catch (const std::exception &error) {
    std::cerr << "estimate_marker_pose: " << error.what() << '\n';
    return 1;
  }
}

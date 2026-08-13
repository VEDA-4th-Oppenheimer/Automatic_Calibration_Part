#include "manual_marker/marker_calibration.hpp"

#include <cstdlib>
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
        std::cout << "calibrate_camera_markers --board board.json "
                     "--images-dir DIR --output-dir DIR [--minimum-frames 10] "
                     "[--maximum-rms-px 2] [--camera-model MODEL] "
                     "[--profile-id ID]\n";
        return 0;
      }
      if (index + 1 >= argc)
        throw std::invalid_argument("Missing value for " + key);
      args[key] = argv[++index];
    }
    for (const char *key : {"--board", "--images-dir", "--output-dir"})
      if (!args.count(key))
        throw std::invalid_argument(std::string("Missing option: ") + key);
    const auto board = manual_marker::loadBoardConfig(args.at("--board"));
    const auto images = manual_marker::listImages(args.at("--images-dir"));
    manual_marker::IntrinsicConfig config;
    if (args.count("--minimum-frames"))
      config.minimum_frames = std::stoi(args.at("--minimum-frames"));
    if (args.count("--maximum-rms-px"))
      config.maximum_rms_px = std::stod(args.at("--maximum-rms-px"));
    auto result = manual_marker::calibrateIntrinsics(images, board, config);
    result.camera.model = args.count("--camera-model")
                              ? args.at("--camera-model")
                              : "PNM-C16083RVQ";
    result.camera.profile_id = args.count("--profile-id")
                                   ? args.at("--profile-id")
                                   : "charuco-fixed-zoom-focus";
    const fs::path output = args.at("--output-dir");
    fs::create_directories(output / "detections");
    for (const auto &detection : result.detections) {
      const auto image =
          cv::imread(detection.image_path.string(), cv::IMREAD_COLOR);
      if (image.empty())
        continue;
      cv::imwrite(
          (output / "detections" / detection.image_path.filename()).string(),
          manual_marker::drawDetection(image, detection));
    }
    auto report = manual_marker::intrinsicResultToJson(result, board);
    report["provenance"] = {
        {"board_file", fs::absolute(args.at("--board")).string()},
        {"images_dir", fs::absolute(args.at("--images-dir")).string()}};
    std::ofstream(output / "camera_intrinsic.json")
        << std::setw(2) << report << '\n';
    std::ofstream summary(output / "camera_intrinsic_report.md");
    summary << "# ChArUco Camera Intrinsic Calibration\n\n"
            << "- Status: `" << result.status << "`\n"
            << "- Reason: `" << result.reason_code << "`\n"
            << "- Input frames: " << result.input_frame_count << "\n"
            << "- Accepted frames: " << result.accepted_frame_count << "\n"
            << "- Calibration RMS: " << result.calibration_rms_px << " px\n";
    std::cout << std::setw(2) << report << '\n';
    return result.solved ? 0 : 2;
  } catch (const std::exception &error) {
    std::cerr << "calibrate_camera_markers: " << error.what() << '\n';
    return 1;
  }
}

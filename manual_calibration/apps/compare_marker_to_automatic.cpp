#include "manual_marker/marker_calibration.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace fs = std::filesystem;
using Args = std::unordered_map<std::string, std::string>;
using nlohmann::json;

namespace {
json readJson(const fs::path &path) {
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("Cannot open JSON: " + path.string());
  json value;
  input >> value;
  return value;
}
} // namespace

int main(int argc, char **argv) {
  try {
    Args args;
    for (int index = 1; index < argc; ++index) {
      const std::string key = argv[index];
      if (key == "--help") {
        std::cout << "compare_marker_to_automatic --manual-pose marker.json "
                     "--board-in-lidar T_lidar_board.json --automatic "
                     "automatic.json --output-dir DIR [--max-rotation-deg D] "
                     "[--max-translation-m M]\n";
        return 0;
      }
      if (index + 1 >= argc)
        throw std::invalid_argument("Missing value for " + key);
      args[key] = argv[++index];
    }
    for (const char *key :
         {"--manual-pose", "--board-in-lidar", "--automatic", "--output-dir"})
      if (!args.count(key))
        throw std::invalid_argument(
            std::string("Direct pose comparison requires option: ") + key);

    const auto t_camera_board = manual_marker::transformFromFlexibleJson(
        readJson(args.at("--manual-pose")), "camera_optical", "marker_board");
    const auto t_lidar_board = manual_marker::transformFromFlexibleJson(
        readJson(args.at("--board-in-lidar")), "lidar_scan", "marker_board");
    const auto t_camera_lidar = manual_marker::compose(
        t_camera_board, manual_marker::inverse(t_lidar_board));
    const auto automatic = manual_marker::transformFromFlexibleJson(
        readJson(args.at("--automatic")), "camera_optical", "lidar_scan");
    if (t_camera_lidar.parent_frame != automatic.parent_frame ||
        t_camera_lidar.child_frame != automatic.child_frame)
      throw std::runtime_error(
          "Derived manual and automatic transforms do not share frames");

    const double rotation_deg = manual_marker::rotationDistanceDeg(
        t_camera_lidar.rotation, automatic.rotation);
    const cv::Vec3d translation_delta =
        t_camera_lidar.translation_m - automatic.translation_m;
    const double translation_norm = cv::norm(translation_delta);
    const bool has_thresholds =
        args.count("--max-rotation-deg") && args.count("--max-translation-m");
    std::string status = "INFO";
    if (has_thresholds)
      status =
          rotation_deg <= std::stod(args.at("--max-rotation-deg")) &&
                  translation_norm <= std::stod(args.at("--max-translation-m"))
              ? "PASS"
              : "FAIL";
    json report = {
        {"schema_version", "1.0"},
        {"method", "charuco_manual_vs_targetless_automatic"},
        {"status", status},
        {"manual_camera_lidar", manual_marker::transformToJson(t_camera_lidar)},
        {"automatic_camera_lidar", manual_marker::transformToJson(automatic)},
        {"difference",
         {{"rotation_geodesic_deg", rotation_deg},
          {"translation_norm_m", translation_norm},
          {"translation_manual_minus_automatic_m",
           {translation_delta[0], translation_delta[1],
            translation_delta[2]}}}},
        {"reference_warning",
         "T_lidar_marker_board must come from an independent measured jig or "
         "survey; a marker image alone cannot produce T_camera_lidar."}};
    if (has_thresholds)
      report["thresholds"] = {
          {"maximum_rotation_deg", std::stod(args.at("--max-rotation-deg"))},
          {"maximum_translation_m", std::stod(args.at("--max-translation-m"))}};
    const fs::path output = args.at("--output-dir");
    fs::create_directories(output);
    std::ofstream(output / "manual_vs_automatic.json")
        << std::setw(2) << report << '\n';
    std::ofstream markdown(output / "manual_vs_automatic.md");
    markdown << "# Marker Manual vs Targetless Automatic\n\n"
             << "- Status: `" << status << "`\n"
             << "- Rotation difference: " << rotation_deg << " deg\n"
             << "- Translation difference: " << translation_norm << " m\n\n"
             << "The board-to-LiDAR reference is independently measured and "
                "is not estimated from the marker image.\n";
    std::cout << std::setw(2) << report << '\n';
    return status == "FAIL" ? 3 : 0;
  } catch (const std::exception &error) {
    std::cerr << "compare_marker_to_automatic: " << error.what() << '\n';
    return 1;
  }
}

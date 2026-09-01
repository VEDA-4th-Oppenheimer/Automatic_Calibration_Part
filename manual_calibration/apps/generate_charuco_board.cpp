#include "manual_marker/marker_calibration.hpp"

#include <cmath>
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
using nlohmann::json;

namespace {
constexpr int kTabS7WidthPx = 2560;
constexpr int kTabS7HeightPx = 1600;
constexpr double kTabS7DiagonalM = 0.2781;
constexpr int kTabS7SquarePx = 260;
constexpr int kTabS7MarkerPx = 195;

double nominalTabS7WidthM() {
  return kTabS7DiagonalM * 16.0 / std::sqrt(16.0 * 16.0 + 10.0 * 10.0);
}

json boardDocument(const manual_marker::BoardConfig &board) {
  return {{"schema_version", "1.0"},
          {"board",
           {{"dictionary", board.dictionary},
            {"squares_x", board.squares_x},
            {"squares_y", board.squares_y},
            {"square_length_m", board.square_length_m},
            {"marker_length_m", board.marker_length_m},
            {"legacy_pattern", board.legacy_pattern},
            {"minimum_charuco_corners", board.minimum_charuco_corners}}}};
}
} // namespace

int main(int argc, char **argv) {
  try {
    Args args;
    for (int index = 1; index < argc; ++index) {
      const std::string key = argv[index];
      if (key == "--help") {
        std::cout << "Galaxy Tab S7 mode:\n"
                     "  generate_charuco_board --display-profile "
                     "galaxy-tab-s7 --output-dir DIR "
                     "[--display-width-mm 235.828]\n"
                     "Generic/print mode:\n"
                     "  generate_charuco_board --board board.json "
                     "--output-dir DIR [--width-px 2000] [--height-px 0] "
                     "[--margin-px 40]\n";
        return 0;
      }
      if (index + 1 >= argc)
        throw std::invalid_argument("Missing value for " + key);
      args[key] = argv[++index];
    }
    if (!args.count("--output-dir"))
      throw std::invalid_argument("--output-dir is required");
    const bool display_mode = args.count("--display-profile") != 0;
    if (display_mode && args.at("--display-profile") != "galaxy-tab-s7")
      throw std::invalid_argument("Supported display profile: galaxy-tab-s7");
    if (!display_mode && !args.count("--board"))
      throw std::invalid_argument(
          "--board is required unless --display-profile is used");
    if (display_mode && args.count("--board"))
      throw std::invalid_argument(
          "--board and --display-profile cannot be used together");

    const fs::path output = args.at("--output-dir");
    fs::create_directories(output);

    if (display_mode) {
      const double display_width_m =
          args.count("--display-width-mm")
              ? std::stod(args.at("--display-width-mm")) / 1000.0
              : nominalTabS7WidthM();
      if (display_width_m <= 0.0)
        throw std::invalid_argument("--display-width-mm must be positive");
      const double pixels_per_meter = kTabS7WidthPx / display_width_m;
      manual_marker::BoardConfig board;
      board.dictionary = "DICT_5X5_100";
      board.squares_x = 7;
      board.squares_y = 5;
      board.square_length_m = kTabS7SquarePx / pixels_per_meter;
      board.marker_length_m = kTabS7MarkerPx / pixels_per_meter;
      board.legacy_pattern = false;
      board.minimum_charuco_corners = 6;
      const cv::Mat image = manual_marker::generateDisplayBoardImage(
          board, kTabS7WidthPx, kTabS7HeightPx, kTabS7SquarePx,
          pixels_per_meter, 0.1);
      if (!cv::imwrite((output / "charuco_board.png").string(), image))
        throw std::runtime_error("Cannot write display board image");
      std::ofstream(output / "board_config.json")
          << std::setw(2) << boardDocument(board) << std::endl;
      const int ruler_px =
          static_cast<int>(std::lround(0.1 * pixels_per_meter));
      const json specification = {
          {"schema_version", "1.0"},
          {"mode", "native_pixel_fullscreen_display"},
          {"display_profile", "galaxy-tab-s7"},
          {"display_model", "Samsung Galaxy Tab S7 11-inch"},
          {"display_resolution_px", {kTabS7WidthPx, kTabS7HeightPx}},
          {"display_diagonal_m", kTabS7DiagonalM},
          {"display_width_m", display_width_m},
          {"display_height_m", display_width_m * 10.0 / 16.0},
          {"pixels_per_meter", pixels_per_meter},
          {"board_square_px", kTabS7SquarePx},
          {"board_marker_px", kTabS7MarkerPx},
          {"board_size_px",
           {board.squares_x * kTabS7SquarePx,
            board.squares_y * kTabS7SquarePx}},
          {"board_size_m",
           {board.squares_x * board.square_length_m,
            board.squares_y * board.square_length_m}},
          {"verification_ruler_px", ruler_px},
          {"verification_ruler_expected_m", ruler_px / pixels_per_meter},
          {"native_display_requirement",
           "Show charuco_board.png at 2560x1600 in immersive fullscreen "
           "with no scaling, crop, status bar, or navigation bar."},
          {"measurement_requirement",
           "Measure the on-screen verification ruler. If it is not the "
           "reported physical length, regenerate with the measured active "
           "display width via --display-width-mm."},
          {"official_spec_source",
           "https://www.samsung.com/sec/support/model/SM-T870NZSEKOO/"}};
      std::ofstream(output / "display_spec.json")
          << std::setw(2) << specification << std::endl;
      std::cout << std::setw(2) << specification << std::endl;
      return 0;
    }

    const auto board = manual_marker::loadBoardConfig(args.at("--board"));
    const int width =
        args.count("--width-px") ? std::stoi(args.at("--width-px")) : 2000;
    int height =
        args.count("--height-px") ? std::stoi(args.at("--height-px")) : 0;
    if (height <= 0)
      height = static_cast<int>(std::lround(static_cast<double>(width) *
                                            board.squares_y / board.squares_x));
    const int margin =
        args.count("--margin-px") ? std::stoi(args.at("--margin-px")) : 40;
    const auto image =
        manual_marker::generateBoardImage(board, width, height, margin);
    if (!cv::imwrite((output / "charuco_board.png").string(), image))
      throw std::runtime_error("Cannot write board image");
    const json specification = {
        {"schema_version", "1.0"},
        {"dictionary", board.dictionary},
        {"squares", {board.squares_x, board.squares_y}},
        {"square_length_m", board.square_length_m},
        {"marker_length_m", board.marker_length_m},
        {"active_board_size_m",
         {board.squares_x * board.square_length_m,
          board.squares_y * board.square_length_m}},
        {"image_size_px", {width, height}},
        {"margin_px", margin},
        {"print_warning",
         "Print at verified physical size; printer scaling must be disabled."}};
    std::ofstream(output / "board_config.json")
        << std::setw(2) << boardDocument(board) << std::endl;
    std::ofstream(output / "charuco_board_print_spec.json")
        << std::setw(2) << specification << std::endl;
    std::cout << std::setw(2) << specification << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "generate_charuco_board: " << error.what() << '\n';
    return 1;
  }
}

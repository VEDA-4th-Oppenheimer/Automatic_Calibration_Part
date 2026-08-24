#include "auto_calib/structural_orientation_analyzer.hpp"
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace fs = std::filesystem;
using nlohmann::json;
using namespace auto_calib;

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "usage: run_structural_orientation_analyzer IMAGE SCAN_JSON OUTPUT_DIR\n";
    return 2;
  }
  try {
    const cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
    if (image.empty()) throw std::runtime_error("cannot read image: " + std::string(argv[1]));
    std::ifstream stream(argv[2]); if (!stream) throw std::runtime_error("cannot read scan JSON");
    const json root = json::parse(stream);
    const auto &scan = root.at("scan");
    StructuralAnalyzerInput input;
    input.image = image; input.rows = scan.at("rows").get<std::size_t>();
    input.columns = scan.at("columns").get<std::size_t>();
    input.organized_lidar.resize(input.rows * input.columns);
    std::vector<bool> seen(input.rows * input.columns, false);
    for (const auto &m : root.at("measurements")) {
      const std::size_t row = m.at("row").get<std::size_t>();
      const std::size_t col = m.at("column").get<std::size_t>();
      if (row >= input.rows || col >= input.columns) throw std::runtime_error("measurement index out of range");
      auto &p = input.organized_lidar[row * input.columns + col];
      if (seen[row * input.columns + col]) throw std::runtime_error("duplicate measurement cell");
      seen[row * input.columns + col] = true;
      p.row = row; p.column = col;
      const bool has_range = m.contains("distance_m") && m["distance_m"].is_number();
      const bool has_angles = m.contains("pan_rad") && m["pan_rad"].is_number() &&
                              m.contains("tilt_rad") && m["tilt_rad"].is_number();
      if (!has_range || !has_angles) continue;
      p.range = m["distance_m"].get<float>() + root.at("sensor").value("range_offset_m", 0.0f);
      const float pan = m["pan_rad"].get<float>(), tilt = m["tilt_rad"].get<float>();
      p.xyz = Eigen::Vector3f(p.range * std::cos(tilt) * std::sin(pan),
                              -p.range * std::sin(tilt), p.range * std::cos(tilt) * std::cos(pan));
      if (m.value("valid", false) && m.value("distance_status", 0) == 1) p.flags = kValidRange;
    }
    const auto result = analyzeStructuralOrientation(input);
    if (!writeStructuralAnalyzerArtifacts(result, argv[3])) throw std::runtime_error("cannot write output");
    std::cout << result.status << " proposals=" << result.proposals.size()
              << " fallback_required=" << (result.fallback_required ? "true" : "false") << "\n";
    return result.proposals.empty() ? 3 : 0;
  } catch (const std::exception &e) { std::cerr << "INVALID_INPUT: " << e.what() << "\n"; return 2; }
}

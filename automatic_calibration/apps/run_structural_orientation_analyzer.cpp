#include "auto_calib/structural_orientation_analyzer.hpp"
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using nlohmann::json;
using namespace auto_calib;

namespace {
CameraModel cameraFromJsonOrSize(const fs::path &intrinsic_path,
                                 cv::Size size) {
  CameraModel camera;
  camera.width = size.width;
  camera.height = size.height;
  if (intrinsic_path.empty()) {
    // Manufacturer FOV midpoint fallback for the PNM-C16083RVQ CH1 channel.
    const double fov_h_deg = (53.0 + 100.0) * 0.5;
    const double fov_v_deg = (30.0 + 54.0) * 0.5;
    camera.k << size.width / (2.0 * std::tan(fov_h_deg * M_PI / 360.0)), 0.0,
        size.width * 0.5, 0.0,
        size.height / (2.0 * std::tan(fov_v_deg * M_PI / 360.0)),
        size.height * 0.5, 0.0, 0.0, 1.0;
    return camera;
  }
  std::ifstream stream(intrinsic_path);
  if (!stream)
    throw std::runtime_error("cannot open intrinsic JSON: " +
                             intrinsic_path.string());
  json root = json::parse(stream);
  const json *camera_json = &root;
  if (root.contains("camera"))
    camera_json = &root.at("camera");
  const json *intrinsic = camera_json;
  if (camera_json->contains("intrinsic"))
    intrinsic = &camera_json->at("intrinsic");
  camera.k << intrinsic->at("fx").get<double>(), 0.0,
      intrinsic->at("cx").get<double>(), 0.0,
      intrinsic->at("fy").get<double>(),
      intrinsic->at("cy").get<double>(), 0.0, 0.0, 1.0;
  return camera;
}
} // namespace

int main(int argc, char **argv) {
  std::string image_path, scan_path, output_dir, intrinsic_path;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  bool has_k = false;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      const auto next = [&]() -> std::string {
        if (i + 1 >= argc)
          throw std::runtime_error("missing value for " + arg);
        return argv[++i];
      };
      if (arg == "--image")
        image_path = next();
      else if (arg == "--scan")
        scan_path = next();
      else if (arg == "--output")
        output_dir = next();
      else if (arg == "--intrinsic-json")
        intrinsic_path = next();
      else if (arg == "--fx") { fx = std::stod(next()); has_k = true; }
      else if (arg == "--fy") { fy = std::stod(next()); has_k = true; }
      else if (arg == "--cx") { cx = std::stod(next()); has_k = true; }
      else if (arg == "--cy") { cy = std::stod(next()); has_k = true; }
      else throw std::runtime_error("unknown argument: " + arg);
    }
    if (image_path.empty() || scan_path.empty() || output_dir.empty())
      throw std::runtime_error(
          "usage: run_structural_orientation_analyzer --image IMAGE "
          "--scan SCAN_JSON --output DIR [--intrinsic-json PATH | "
          "--fx F --fy F --cx C --cy C]");
    const cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty())
      throw std::runtime_error("cannot read image: " + image_path);
    CameraModel camera;
    if (has_k) {
      camera.width = image.cols;
      camera.height = image.rows;
      camera.k << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    } else {
      camera = cameraFromJsonOrSize(intrinsic_path, image.size());
    }
    std::ifstream stream(scan_path);
    if (!stream)
      throw std::runtime_error("cannot read scan JSON");
    const json root = json::parse(stream);
    const auto &scan = root.at("scan");
    StructuralAnalyzerInput input;
    input.image = image;
    input.camera = camera;
    input.rows = scan.at("rows").get<std::size_t>();
    input.columns = scan.at("columns").get<std::size_t>();
    input.organized_lidar.resize(input.rows * input.columns);
    std::vector<bool> seen(input.rows * input.columns, false);
    const double range_offset =
        root.contains("sensor") && root.at("sensor").contains("range_offset_m")
            ? root.at("sensor").at("range_offset_m").get<double>()
            : 0.0;
    for (const auto &m : root.at("measurements")) {
      const std::size_t row = m.at("row").get<std::size_t>();
      const std::size_t col = m.at("column").get<std::size_t>();
      if (row >= input.rows || col >= input.columns)
        throw std::runtime_error("measurement index out of range");
      auto &p = input.organized_lidar[row * input.columns + col];
      if (seen[row * input.columns + col])
        throw std::runtime_error("duplicate measurement cell");
      seen[row * input.columns + col] = true;
      p.row = row;
      p.column = col;
      const bool has_range = m.contains("distance_m") && !m["distance_m"].is_null() &&
                             m["distance_m"].is_number();
      const bool has_angles = m.contains("pan_rad") && m["pan_rad"].is_number() &&
                              m.contains("tilt_rad") && m["tilt_rad"].is_number();
      if (!has_range || !has_angles || !m.value("valid", false))
        continue;
      p.range = static_cast<float>(m["distance_m"].get<double>() + range_offset);
      const float pan = m["pan_rad"].get<float>();
      const float tilt = m["tilt_rad"].get<float>();
      p.xyz = Eigen::Vector3f(p.range * std::cos(tilt) * std::sin(pan),
                              -p.range * std::sin(tilt),
                              p.range * std::cos(tilt) * std::cos(pan));
      if (m.value("valid", false) &&
          m.value("distance_status", 0) == 1)
        p.flags = kValidRange;
    }
    const auto result = analyzeStructuralOrientation(input);
    if (!writeStructuralAnalyzerArtifacts(result, output_dir))
      throw std::runtime_error("cannot write output");
    std::cout << result.status
              << " proposals=" << result.proposals.size()
              << " lines=" << result.line_count
              << " normals=" << result.normal_count
              << " fallback_required=" << (result.fallback_required ? "true" : "false")
              << "\n";
    return result.proposals.empty() ? 3 : 0;
  } catch (const std::exception &e) {
    std::cerr << "INVALID_INPUT: " << e.what() << "\n";
    return 2;
  }
}

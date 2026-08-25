#include "auto_calib/panorama_orientation_analyzer.hpp"
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
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
  std::string scan_path, image_path, output_dir, intrinsic_path;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      const auto next = [&]() -> std::string {
        if (i + 1 >= argc)
          throw std::runtime_error("missing value for " + arg);
        return argv[++i];
      };
      if (arg == "--scan")
        scan_path = next();
      else if (arg == "--image")
        image_path = next();
      else if (arg == "--output")
        output_dir = next();
      else if (arg == "--intrinsic-json")
        intrinsic_path = next();
      else
        throw std::runtime_error("unknown argument: " + arg);
    }
    if (scan_path.empty() || image_path.empty() || output_dir.empty())
      throw std::runtime_error(
          "usage: run_panorama_analyzer --scan SCAN_JSON --image IMAGE "
          "--output DIR [--intrinsic-json PATH]");
    const cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty())
      throw std::runtime_error("cannot read image: " + image_path);
    const CameraModel camera = cameraFromJsonOrSize(intrinsic_path, image.size());
    PanoramaAnalyzerOptions options;
    options.output_dir = output_dir;
    const auto result = analyzePanorama(scan_path, image, camera, options);
    if (!writePanoramaAnalyzerArtifacts(result, output_dir))
      throw std::runtime_error("cannot write output");
    std::cout << "status=" << result.status
              << " proposals=" << result.proposals.size()
              << " coverage=" << result.coverage
              << " pslr=" << result.peak_to_sidelobe_ratio
              << " candidates=" << result.evaluated_candidates
              << " runtime_ms=" << result.runtime_ms
              << " fallback_required=" << (result.fallback_required ? "true" : "false")
              << "\n";
    return result.fallback_required ? 3 : 0;
  } catch (const std::exception &e) {
    std::cerr << "INVALID_INPUT: " << e.what() << "\n";
    return 2;
  }
}

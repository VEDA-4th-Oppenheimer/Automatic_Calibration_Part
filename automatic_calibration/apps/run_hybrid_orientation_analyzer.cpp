#include "auto_calib/hybrid_orientation_analyzer.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>

namespace fs = std::filesystem;
using nlohmann::json;

namespace {
auto_calib::CameraModel cameraFromJson(const fs::path &path, cv::Size size) {
  auto_calib::CameraModel camera;
  camera.width = size.width;
  camera.height = size.height;
  if (path.empty()) {
    const double horizontal_fov_deg = 76.5;
    const double vertical_fov_deg = 42.0;
    camera.k << size.width / (2.0 * std::tan(horizontal_fov_deg *
                                             3.141592653589793 / 360.0)),
        0.0, size.width * 0.5, 0.0,
        size.height /
            (2.0 * std::tan(vertical_fov_deg * 3.141592653589793 / 360.0)),
        size.height * 0.5, 0.0, 0.0, 1.0;
    return camera;
  }
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("cannot open intrinsic JSON: " + path.string());
  const json root = json::parse(stream);
  const json *value = &root;
  if (value->contains("camera"))
    value = &value->at("camera");
  if (value->contains("intrinsic"))
    value = &value->at("intrinsic");
  camera.k << value->at("fx").get<double>(), 0.0, value->at("cx").get<double>(),
      0.0, value->at("fy").get<double>(), value->at("cy").get<double>(), 0.0,
      0.0, 1.0;
  return camera;
}
} // namespace

int main(int argc, char **argv) {
  fs::path scan, image, output, intrinsic;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      const auto next = [&]() -> fs::path {
        if (i + 1 >= argc)
          throw std::runtime_error("missing value for " + arg);
        return argv[++i];
      };
      if (arg == "--scan")
        scan = next();
      else if (arg == "--image")
        image = next();
      else if (arg == "--output")
        output = next();
      else if (arg == "--intrinsic-json")
        intrinsic = next();
      else
        throw std::runtime_error("unknown argument: " + arg);
    }
    if (scan.empty() || image.empty() || output.empty())
      throw std::runtime_error(
          "usage: run_hybrid_orientation_analyzer --scan JSON --image IMAGE "
          "--output DIR [--intrinsic-json JSON]");
    const cv::Mat bgr = cv::imread(image.string(), cv::IMREAD_COLOR);
    if (bgr.empty())
      throw std::runtime_error("cannot read image: " + image.string());
    const auto camera = cameraFromJson(intrinsic, bgr.size());
    const auto result = auto_calib::analyzeHybridOrientation(scan, bgr, camera);
    if (!auto_calib::writeHybridAnalyzerArtifacts(result, output))
      throw std::runtime_error("cannot write analyzer artifacts");
    std::cout << "status=" << result.status
              << " proposals=" << result.proposals.size()
              << " signature_yaws=" << result.evaluated_signature_yaws
              << " perspective_remaps=" << result.perspective_remaps
              << " runtime_ms=" << result.runtime_ms << " fallback_required="
              << (result.fallback_required ? "true" : "false") << '\n';
    return result.fallback_required ? 3 : 0;
  } catch (const std::exception &error) {
    std::cerr << "INVALID_INPUT: " << error.what() << '\n';
    return 2;
  }
}

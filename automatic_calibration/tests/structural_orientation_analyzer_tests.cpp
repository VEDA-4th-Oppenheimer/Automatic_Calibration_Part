#include "auto_calib/structural_orientation_analyzer.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace auto_calib;
int main() {
  StructuralAnalyzerInput bad; bad.image = cv::Mat::zeros(64, 64, CV_8UC1);
  auto failed = analyzeStructuralOrientation(bad);
  assert(failed.fallback_required && failed.status == "INSUFFICIENT_FEATURES");
  cv::Mat image = cv::Mat::zeros(160, 240, CV_8UC1);
  cv::line(image, {20, 20}, {220, 20}, 255, 3); cv::line(image, {20, 140}, {220, 40}, 255, 3);
  StructuralAnalyzerInput in; in.image = image; in.rows = 3; in.columns = 3;
  in.organized_lidar.resize(9);
  for (std::size_t r = 0; r < 3; ++r) for (std::size_t c = 0; c < 3; ++c) {
    auto &p = in.organized_lidar[r * 3 + c]; p.flags = kValidRange; p.row = r; p.column = c; p.range = 2; p.xyz = Eigen::Vector3f(float(c), float(r), 2);
  }
  auto result = analyzeStructuralOrientation(in); assert(!result.activation_allowed);
  assert(result.proposals.size() <= 3); assert(result.status == "INSUFFICIENT_FEATURES" || result.status == "PROPOSALS_READY");
  std::cout << "structural analyzer tests passed\n";
}

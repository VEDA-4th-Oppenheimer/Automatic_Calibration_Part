#pragma once

#include "auto_calib/synthetic_lidar.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace auto_calib {

struct StructuralOrientationProposal {
  int rank = 0;
  double yaw_deg = 0, down_deg = 0, roll_deg = 0;
  double raw_score = 0, normalized_score = 0, confidence = 0;
  double search_radius_deg = 10;
  std::vector<std::string> evidence;
};

struct StructuralAnalyzerInput {
  cv::Mat image;                         // grayscale or BGR
  std::vector<Point> organized_lidar;    // row-major points with row/column
  std::size_t rows = 0, columns = 0;
  CameraModel camera;
  std::size_t top_k = 3;
};

struct StructuralAnalyzerResult {
  std::string schema_version = "1.0";
  std::string mode = "structural";
  std::string status = "INVALID_INPUT";
  std::size_t input_rows = 0, input_columns = 0, line_count = 0;
  std::size_t normal_count = 0;
  std::vector<StructuralOrientationProposal> proposals;
  bool fallback_required = true;
  std::string fallback_reason;
  double runtime_ms = 0;
  bool activation_allowed = false;
  std::vector<double> camera_azimuth_signature;
  std::vector<double> lidar_azimuth_signature;
};

StructuralAnalyzerResult analyzeStructuralOrientation(
    const StructuralAnalyzerInput &input);

bool writeStructuralAnalyzerArtifacts(const StructuralAnalyzerResult &result,
                                      const std::string &directory);

} // namespace auto_calib

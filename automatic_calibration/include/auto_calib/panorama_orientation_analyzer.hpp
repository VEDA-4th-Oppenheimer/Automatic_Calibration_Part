#pragma once

#include <opencv2/core.hpp>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace auto_calib {

struct PanoramaProposal {
  int rank = 0;
  double yaw_deg = 0.0, down_deg = 0.0, roll_deg = 0.0;
  double raw_score = 0.0, normalized_score = 0.0, confidence = 0.0;
  double search_radius_deg = 10.0;
  std::string evidence = "AZIMUTH_SIGNATURE,MANHATTAN";
};

struct PanoramaAnalyzerOptions {
  int top_k = 3;
  double minimum_coverage = 0.20;
  // Optional camera structural signature; values are compared circularly.
  std::vector<double> camera_signature;
  std::filesystem::path output_dir;
};

struct PanoramaAnalyzerResult {
  std::string status = "INVALID_INPUT";
  int rows = 0, columns = 0;
  double coverage = 0.0;
  bool fallback_required = true;
  std::string fallback_reason;
  std::vector<PanoramaProposal> proposals;
  std::vector<double> lidar_signature, score_curve;
  cv::Mat range_mm, valid, range_edge, normal_edge, plane_intersection;
};

PanoramaAnalyzerResult analyzePanorama(const std::filesystem::path &json_path,
                                       const PanoramaAnalyzerOptions &options = {});
double circularDistanceDeg(double a, double b);
double normalizeYawDeg(double yaw);

}  // namespace auto_calib

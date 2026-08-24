#include "auto_calib/panorama_orientation_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace auto_calib {
namespace {
constexpr double kPi = 3.14159265358979323846;
double value(const nlohmann::json &j, const char *key, double fallback) {
  return j.contains(key) && j[key].is_number() ? j[key].get<double>() : fallback;
}
std::vector<double> resample(const std::vector<double> &v, int n) {
  std::vector<double> out(static_cast<std::size_t>(n));
  if (v.empty()) return out;
  for (int i = 0; i < n; ++i) out[static_cast<std::size_t>(i)] = v[static_cast<std::size_t>(i) * v.size() / n];
  return out;
}
double circularScore(const std::vector<double> &a, const std::vector<double> &b, int shift) {
  if (a.empty() || b.empty()) return 0.0;
  const int n = static_cast<int>(a.size()); double sum = 0.0, norm = 0.0;
  for (int i = 0; i < n; ++i) { const double d = a[i] - b[(i + shift + n * 2) % n]; sum += d * d; norm += a[i] * a[i] + b[(i + shift + n * 2) % n] * b[(i + shift + n * 2) % n]; }
  return norm > 1e-12 ? 1.0 - std::sqrt(sum / norm) : 0.0;
}
} // namespace

double normalizeYawDeg(double yaw) { while (yaw >= 180.0) yaw -= 360.0; while (yaw < -180.0) yaw += 360.0; return yaw; }
double circularDistanceDeg(double a, double b) { return std::abs(normalizeYawDeg(a - b)); }

PanoramaAnalyzerResult analyzePanorama(const std::filesystem::path &path, const PanoramaAnalyzerOptions &opt) {
  PanoramaAnalyzerResult out;
  try {
    std::ifstream input(path); if (!input) throw std::runtime_error("cannot open JSON");
    const auto root = nlohmann::json::parse(input);
    const auto &scan = root.at("scan");
    out.rows = scan.at("rows").get<int>(); out.columns = scan.at("columns").get<int>();
    const int count = scan.at("sample_count").get<int>();
    if (out.rows <= 0 || out.columns <= 0 || count != out.rows * out.columns) throw std::runtime_error("organized shape/sample_count mismatch");
    out.range_mm = cv::Mat(out.rows, out.columns, CV_16UC1, cv::Scalar(0));
    out.valid = cv::Mat(out.rows, out.columns, CV_8UC1, cv::Scalar(0));
    std::vector<unsigned char> seen(static_cast<std::size_t>(count), 0);
    const auto &measurements = root.at("measurements");
    for (const auto &m : measurements) {
      const int r = m.at("row").get<int>(), c = m.at("column").get<int>();
      if (r < 0 || r >= out.rows || c < 0 || c >= out.columns || seen[static_cast<std::size_t>(r * out.columns + c)]) throw std::runtime_error("duplicate/out-of-range raster cell");
      seen[static_cast<std::size_t>(r * out.columns + c)] = 1;
      if (m.value("valid", false) && m.contains("distance_m") && m["distance_m"].is_number()) { out.range_mm.at<std::uint16_t>(r, c) = static_cast<std::uint16_t>(std::clamp(value(m, "distance_m", 0.0) * 1000.0, 1.0, 65535.0)); out.valid.at<unsigned char>(r, c) = 255; }
    }
    if (static_cast<int>(measurements.size()) != count || std::count(seen.begin(), seen.end(), 0) != 0) throw std::runtime_error("missing measurement cell");
    out.coverage = static_cast<double>(cv::countNonZero(out.valid)) / count;
    if (out.coverage < opt.minimum_coverage) { out.status = "INSUFFICIENT_FEATURES"; out.fallback_reason = "INSUFFICIENT_PANORAMA_COVERAGE"; return out; }
    cv::Mat smooth; cv::Sobel(out.range_mm, smooth, CV_32F, 1, 0, 3); cv::convertScaleAbs(smooth, out.range_edge); out.range_edge.setTo(0, out.valid == 0);
    out.normal_edge = out.range_edge.clone(); out.plane_intersection = out.range_edge.clone();
    out.lidar_signature.assign(static_cast<std::size_t>(out.columns), 0.0);
    for (int c = 0; c < out.columns; ++c) { double sum = 0.0; int n = 0; for (int r = 0; r < out.rows; ++r) if (out.valid.at<unsigned char>(r, c)) { sum += out.range_edge.at<unsigned char>(r, c); ++n; } out.lidar_signature[static_cast<std::size_t>(c)] = n ? sum / n : 0.0; }
    const auto camera = resample(opt.camera_signature, out.columns);
    out.score_curve.resize(static_cast<std::size_t>(out.columns));
    for (int s = 0; s < out.columns; ++s) out.score_curve[static_cast<std::size_t>(s)] = opt.camera_signature.empty() ? out.lidar_signature[static_cast<std::size_t>(s)] : circularScore(out.lidar_signature, camera, s);
    std::vector<int> peaks; const int k = std::max(1, std::min(opt.top_k, 3));
    for (int n = 0; n < k; ++n) { int best = -1; for (int i = 0; i < out.columns; ++i) if (std::find(peaks.begin(), peaks.end(), i) == peaks.end() && (best < 0 || out.score_curve[static_cast<std::size_t>(i)] > out.score_curve[static_cast<std::size_t>(best)])) best = i; if (best < 0) break; peaks.push_back(best); }
    const double min_score = *std::min_element(out.score_curve.begin(), out.score_curve.end()), max_score = *std::max_element(out.score_curve.begin(), out.score_curve.end());
    for (std::size_t i = 0; i < peaks.size(); ++i) { const int p = peaks[i]; PanoramaProposal q; q.rank = static_cast<int>(i + 1); q.yaw_deg = normalizeYawDeg(360.0 * p / out.columns); q.raw_score = out.score_curve[static_cast<std::size_t>(p)]; q.normalized_score = (max_score > min_score) ? (q.raw_score - min_score) / (max_score - min_score) : 1.0; q.confidence = q.normalized_score; out.proposals.push_back(q); }
    out.status = out.proposals.empty() ? "INSUFFICIENT_FEATURES" : "PROPOSALS_READY"; out.fallback_required = out.proposals.empty(); if (out.fallback_required) out.fallback_reason = "NO_STRUCTURAL_PROPOSAL";
  } catch (const std::exception &e) { out.fallback_reason = e.what(); }
  return out;
}
} // namespace auto_calib

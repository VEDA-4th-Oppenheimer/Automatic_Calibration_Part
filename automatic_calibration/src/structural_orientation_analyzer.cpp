#include "auto_calib/structural_orientation_analyzer.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>

namespace auto_calib {
namespace {
constexpr double kPi = 3.14159265358979323846;
double deg(double r) { return r * 180.0 / kPi; }
double circular(double a) {
  while (a >= 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}
double distance(double a, double b) { return std::abs(circular(a - b)); }

std::vector<double> lineDirections(const cv::Mat &image, std::size_t &count) {
  cv::Mat gray;
  if (image.channels() == 1) gray = image;
  else cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  if (gray.empty()) return {};
  auto detector = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
  std::vector<cv::Vec4f> lines;
  detector->detect(gray, lines);
  std::vector<std::pair<double, double>> weighted;
  for (const auto &l : lines) {
    const double dx = l[2] - l[0], dy = l[3] - l[1];
    const double length = std::hypot(dx, dy);
    if (length < std::max(8.0, 0.04 * std::min(gray.cols, gray.rows))) continue;
    weighted.emplace_back(std::fmod(std::atan2(dy, dx) + kPi, kPi), length);
  }
  std::sort(weighted.begin(), weighted.end(),
            [](auto a, auto b) { return a.second > b.second; });
  if (weighted.size() > 80) weighted.resize(80);
  count = weighted.size();
  std::vector<double> directions;
  for (const auto &[a, w] : weighted) {
    (void)w;
    bool distinct = true;
    for (double old : directions) {
      const double d = std::abs(a - old);
      if (std::min(d, kPi - d) < 10.0 * kPi / 180.0) distinct = false;
    }
    if (distinct) directions.push_back(a);
    if (directions.size() == 3) break;
  }
  return directions;
}

std::vector<double> lidarAxes(const StructuralAnalyzerInput &in,
                              std::size_t &normal_count) {
  std::vector<double> angles;
  if (!in.rows || !in.columns || in.organized_lidar.size() < in.rows * in.columns)
    return angles;
  for (std::size_t r = 0; r + 1 < in.rows; ++r)
    for (std::size_t c = 0; c + 1 < in.columns; ++c) {
      const Point &a = in.organized_lidar[r * in.columns + c];
      const Point &b = in.organized_lidar[r * in.columns + c + 1];
      const Point &d = in.organized_lidar[(r + 1) * in.columns + c];
      if (!a.valid() || !b.valid() || !d.valid()) continue;
      const Eigen::Vector3d u = (b.xyz - a.xyz).cast<double>();
      const Eigen::Vector3d v = (d.xyz - a.xyz).cast<double>();
      const Eigen::Vector3d n = u.cross(v);
      if (n.norm() < 1e-6) continue;
      ++normal_count;
      const double az = std::atan2(n.x(), n.z());
      angles.push_back(std::fmod(az + kPi, kPi));
    }
  std::sort(angles.begin(), angles.end());
  std::vector<double> axes;
  for (double a : angles) {
    bool distinct = true;
    for (double old : axes) if (std::min(std::abs(a - old), kPi - std::abs(a - old)) < 15 * kPi / 180) distinct = false;
    if (distinct) axes.push_back(a);
    if (axes.size() == 3) break;
  }
  return axes;
}

std::vector<double> signature(const std::vector<Point> &points, std::size_t columns) {
  std::vector<double> out(columns, 0.0), counts(columns, 0.0);
  for (const auto &p : points) if (p.valid() && columns && p.column < columns) {
    out[p.column] += std::isfinite(p.range) ? p.range : p.xyz.norm(); counts[p.column]++;
  }
  for (std::size_t i = 0; i < columns; ++i) if (counts[i]) out[i] /= counts[i];
  return out;
}
} // namespace

StructuralAnalyzerResult analyzeStructuralOrientation(const StructuralAnalyzerInput &in) {
  const auto start = std::chrono::steady_clock::now();
  StructuralAnalyzerResult out;
  out.input_rows = in.rows; out.input_columns = in.columns;
  out.lidar_azimuth_signature = signature(in.organized_lidar, in.columns);
  std::size_t lines = 0; const auto camera_axes = lineDirections(in.image, lines);
  out.line_count = lines;
  out.camera_azimuth_signature.assign(36, 0.0);
  std::size_t normals = 0; const auto lidar_axes = lidarAxes(in, normals); out.normal_count = normals;
  if (camera_axes.size() < 2) out.fallback_reason = "INSUFFICIENT_IMAGE_DIRECTIONS";
  else if (lidar_axes.size() < 2) out.fallback_reason = "INSUFFICIENT_LIDAR_NORMALS";
  else {
    std::vector<double> seeds;
    for (double c : camera_axes) for (double l : lidar_axes) seeds.push_back(circular(deg(l - c)));
    std::sort(seeds.begin(), seeds.end(), [](double a, double b) { return std::abs(a) < std::abs(b); });
    for (double yaw : seeds) {
      bool separated = true; for (const auto &p : out.proposals) if (distance(yaw, p.yaw_deg) < 30) separated = false;
      if (!separated) continue;
      StructuralOrientationProposal p; p.rank = static_cast<int>(out.proposals.size()) + 1;
      p.yaw_deg = circular(yaw); p.raw_score = 1.0 / (1.0 + std::abs(yaw));
      p.normalized_score = p.raw_score; p.confidence = std::min(1.0, p.raw_score);
      p.evidence = {"MANHATTAN", "AZIMUTH_SIGNATURE"}; out.proposals.push_back(std::move(p));
      if (out.proposals.size() >= std::min<std::size_t>(3, std::max<std::size_t>(1, in.top_k))) break;
    }
    if (out.proposals.empty()) out.fallback_reason = "NO_SEPARATED_PROPOSALS";
  }
  out.status = out.proposals.empty() ? "INSUFFICIENT_FEATURES" : "PROPOSALS_READY";
  out.fallback_required = out.proposals.empty(); out.activation_allowed = false;
  out.runtime_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  return out;
}

bool writeStructuralAnalyzerArtifacts(const StructuralAnalyzerResult &r, const std::string &directory) {
  std::filesystem::create_directories(directory);
  std::ofstream csv(directory + "/orientation_proposals.csv");
  if (!csv) return false;
  csv << "rank,yaw_deg,down_deg,roll_deg,raw_score,normalized_score,confidence,search_radius_deg,evidence\n";
  for (const auto &p : r.proposals) csv << p.rank << ',' << p.yaw_deg << ',' << p.down_deg << ',' << p.roll_deg << ',' << p.raw_score << ',' << p.normalized_score << ',' << p.confidence << ',' << p.search_radius_deg << ",MANHATTAN|AZIMUTH_SIGNATURE\n";
  std::ofstream json(directory + "/analyzer_result.json");
  if (!json) return false;
  json << "{\n  \"schema_version\": \"1.0\",\n  \"mode\": \"structural\",\n  \"status\": \"" << r.status << "\",\n  \"input_rows\": " << r.input_rows << ",\n  \"input_columns\": " << r.input_columns << ",\n  \"proposal_count\": " << r.proposals.size() << ",\n  \"fallback_required\": " << (r.fallback_required ? "true" : "false") << ",\n  \"fallback_reason\": \"" << r.fallback_reason << "\",\n  \"runtime_ms\": " << r.runtime_ms << ",\n  \"activation_allowed\": false\n}\n";
  return true;
}
} // namespace auto_calib

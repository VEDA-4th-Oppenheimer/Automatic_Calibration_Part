#include "auto_calib/panorama_orientation_analyzer.hpp"
#include <fstream>
#include <filesystem>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv) {
  if (argc != 7 || std::string(argv[1]) != "--scan" || std::string(argv[3]) != "--image" || std::string(argv[5]) != "--output") {
    std::cerr << "usage: run_panorama_analyzer --scan scan.json --image image.jpg --output dir\n"; return 2;
  }
  const std::filesystem::path scan = argv[2], image_path = argv[4], output = argv[6];
  std::filesystem::create_directories(output);
  cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE), edges;
  if (image.empty()) { std::cerr << "cannot read image\n"; return 2; }
  cv::Canny(image, edges, 60, 180);
  auto result = auto_calib::analyzePanorama(scan);
  if (!result.lidar_signature.empty()) {
    cv::Mat reduced; cv::resize(edges, reduced, cv::Size(static_cast<int>(result.lidar_signature.size()), 1), 0, 0, cv::INTER_AREA);
    auto opt = auto_calib::PanoramaAnalyzerOptions{}; opt.camera_signature.resize(result.lidar_signature.size());
    for (std::size_t i = 0; i < opt.camera_signature.size(); ++i) opt.camera_signature[i] = reduced.at<unsigned char>(0, static_cast<int>(i));
    result = auto_calib::analyzePanorama(scan, opt);
  }
  cv::imwrite((output / "panorama_range.png").string(), result.range_mm);
  cv::imwrite((output / "panorama_valid.png").string(), result.valid);
  cv::imwrite((output / "panorama_range_edge.png").string(), result.range_edge);
  cv::imwrite((output / "panorama_normal_edge.png").string(), result.normal_edge);
  cv::imwrite((output / "panorama_plane_intersection.png").string(), result.plane_intersection);
  std::ofstream csv(output / "orientation_proposals.csv"); csv << "rank,yaw_deg,down_deg,roll_deg,raw_score,normalized_score,confidence,search_radius_deg,evidence\n";
  for (const auto &p : result.proposals) csv << p.rank << ',' << p.yaw_deg << ',' << p.down_deg << ',' << p.roll_deg << ',' << p.raw_score << ',' << p.normalized_score << ',' << p.confidence << ',' << p.search_radius_deg << ',' << p.evidence << '\n';
  std::ofstream json(output / "analyzer_result.json"); json << nlohmann::json{{"schema_version", "1.0"}, {"mode", "panorama"}, {"status", result.status}, {"input_rows", result.rows}, {"input_columns", result.columns}, {"proposal_count", result.proposals.size()}, {"fallback_required", result.fallback_required}, {"fallback_reason", result.fallback_reason}, {"coverage", result.coverage}}.dump(2) << '\n';
  std::cout << "status=" << result.status << " proposals=" << result.proposals.size() << " coverage=" << result.coverage << "\n";
  return result.fallback_required ? 3 : 0;
}

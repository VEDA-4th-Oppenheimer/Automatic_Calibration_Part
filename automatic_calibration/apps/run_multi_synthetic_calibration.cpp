#include "auto_calib/calibration_core.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
namespace fs = std::filesystem;
namespace {
using Args = std::unordered_map<std::string, std::string>;
constexpr double pi = 3.14159265358979323846;
double rad(double degrees) { return degrees * pi / 180.0; }
Args parse(int argc, char **argv) {
  Args out;
  for (int i = 1; i < argc; ++i) {
    std::string key = argv[i];
    if (key == "--help") {
      out[key] = "";
      continue;
    }
    if (key.rfind("--", 0) != 0 || i + 1 >= argc)
      throw std::invalid_argument("Use --key value arguments");
    out[key] = argv[++i];
  }
  return out;
}
double value(const Args &args, const std::string &key, double fallback) {
  auto it = args.find(key);
  return it == args.end() ? fallback : std::stod(it->second);
}
std::uint32_t uvalue(const Args &args, const std::string &key,
                     std::uint32_t fallback) {
  auto it = args.find(key);
  return it == args.end() ? fallback
                          : static_cast<std::uint32_t>(std::stoul(it->second));
}
std::vector<std::string> split(const std::string &text) {
  std::vector<std::string> out;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ','))
    if (!item.empty())
      out.push_back(item);
  return out;
}
nlohmann::json transformJson(const auto_calib::Transform &transform) {
  nlohmann::json rotation = nlohmann::json::array();
  for (int row = 0; row < 3; ++row)
    rotation.push_back({transform.rotation(row, 0), transform.rotation(row, 1),
                        transform.rotation(row, 2)});
  return {{"rotation_matrix", rotation},
          {"translation_m",
           {transform.translation_m.x(), transform.translation_m.y(),
            transform.translation_m.z()}}};
}
void usage() {
  std::cout << "run_multi_synthetic_calibration --dataset-root PATH --output "
               "PATH --frame-ids ID1,ID2,... [initial delta options]\n";
}
} // namespace
int main(int argc, char **argv) {
  try {
    const Args args = parse(argc, argv);
    if (args.count("--help")) {
      usage();
      return 0;
    }
    if (!args.count("--dataset-root") || !args.count("--output") ||
        !args.count("--frame-ids")) {
      usage();
      return 2;
    }
    const auto ids = split(args.at("--frame-ids"));
    if (ids.size() < 2)
      throw std::invalid_argument(
          "Multi-scene calibration needs at least 2 IDs");

    const Eigen::Vector3d gt_translation{0.15, -0.02, 0.08};
    const Eigen::Vector3d gt_rpy{rad(2), rad(-4), rad(6)};
    const auto ground_truth = auto_calib::makeTransform(gt_translation, gt_rpy);
    const Eigen::Vector3d initial_translation =
        gt_translation +
        Eigen::Vector3d(value(args, "--initial-tx-delta-m", 0.02),
                        value(args, "--initial-ty-delta-m", -0.015),
                        value(args, "--initial-tz-delta-m", 0.025));
    const Eigen::Vector3d initial_rpy =
        gt_rpy +
        Eigen::Vector3d(rad(value(args, "--initial-roll-delta-deg", 1.0)),
                        rad(value(args, "--initial-pitch-delta-deg", -1.5)),
                        rad(value(args, "--initial-yaw-delta-deg", 2.0)));
    const auto initial =
        auto_calib::makeTransform(initial_translation, initial_rpy);

    auto_calib::ScanConfig scan_config;
    scan_config.columns = uvalue(args, "--columns", 321);
    scan_config.rows = uvalue(args, "--rows", 121);
    scan_config.pixel_stride = uvalue(args, "--pixel-stride", 2);
    scan_config.noise_stddev = value(args, "--noise-stddev-m", 0.005);
    scan_config.dropout = value(args, "--dropout", 0.01);
    scan_config.seed = uvalue(args, "--seed", 7);

    const fs::path output = args.at("--output");
    std::vector<auto_calib::CalibrationObservation> observations;
    observations.reserve(ids.size());
    for (std::size_t index = 0; index < ids.size(); ++index) {
      auto frame =
          auto_calib::loadStanfordFrame(args.at("--dataset-root"), ids[index]);
      auto camera_points = auto_calib::projectDepth(frame.depth, frame.camera,
                                                    scan_config.pixel_stride);
      auto scan =
          auto_calib::generateScan(camera_points, ground_truth, scan_config);
      auto_calib::writePackage(output / "input" /
                                   ("scene_" + std::to_string(index)),
                               frame, scan, ground_truth);
      observations.push_back({frame.rgb, frame.camera, std::move(scan)});
    }

    auto_calib::CalibrationConfig config;
    config.minimum_lidar_edge_points =
        uvalue(args, "--minimum-lidar-edges", 50);
    config.maximum_solver_iterations =
        static_cast<int>(uvalue(args, "--max-iterations", 100));
    config.prior_weight = value(args, "--prior-weight", 1.0);
    config.rotation_prior_sigma_rad =
        rad(value(args, "--rotation-prior-sigma-deg", 2.8648));
    config.translation_prior_sigma_m =
        value(args, "--translation-prior-sigma-m", 0.02);
    config.maximum_mean_edge_distance_px =
        value(args, "--maximum-mean-edge-distance-px", 40.0);
    config.residual_cap_px = value(args, "--residual-cap-px", 20.0);
    const auto result =
        auto_calib::calibrateExtrinsicMultiScene(observations, initial, config);
    const auto initial_error =
        auto_calib::calculatePoseError(initial, ground_truth);
    const auto final_error = auto_calib::calculatePoseError(
        result.estimated_t_camera_lidar, ground_truth);
    const double maximum_rotation_error =
        value(args, "--maximum-rotation-error-deg", 3.0);
    const double maximum_translation_error =
        value(args, "--maximum-translation-error-m", 0.05);
    const bool conformance =
        result.success && final_error.rotation_deg <= maximum_rotation_error &&
        final_error.translation_m <= maximum_translation_error;

    const nlohmann::json report = {
        {"status", conformance ? "PASS" : "FAIL"},
        {"core_status", result.success ? "PASS" : "FAIL"},
        {"reason_code",
         conformance ? result.reason_code
                     : (result.success ? "GROUND_TRUTH_TOLERANCE_EXCEEDED"
                                       : result.reason_code)},
        {"scene_count", observations.size()},
        {"frame_ids", ids},
        {"ground_truth", transformJson(ground_truth)},
        {"initial", transformJson(initial)},
        {"estimated", transformJson(result.estimated_t_camera_lidar)},
        {"initial_error",
         {{"translation_m", initial_error.translation_m},
          {"rotation_deg", initial_error.rotation_deg}}},
        {"final_error",
         {{"translation_m", final_error.translation_m},
          {"rotation_deg", final_error.rotation_deg}}},
        {"tolerances",
         {{"maximum_translation_error_m", maximum_translation_error},
          {"maximum_rotation_error_deg", maximum_rotation_error}}},
        {"metrics",
         {{"camera_edge_pixels", result.metrics.camera_edge_pixels},
          {"lidar_edge_points", result.metrics.lidar_edge_points},
          {"lidar_geometry_points", result.metrics.lidar_geometry_points},
          {"nid_projected_points", result.metrics.nid_projected_points},
          {"projected_edge_points", result.metrics.projected_edge_points},
          {"projected_ratio", result.metrics.projected_ratio},
          {"initial_mean_edge_distance_px",
           result.metrics.initial_mean_edge_distance_px},
          {"final_mean_edge_distance_px",
           result.metrics.final_mean_edge_distance_px},
          {"initial_nid", result.metrics.initial_nid},
          {"final_nid", result.metrics.final_nid},
          {"nid_improvement_ratio", result.metrics.nid_improvement_ratio},
          {"initial_composite_objective",
           result.metrics.initial_composite_objective},
          {"final_composite_objective",
           result.metrics.final_composite_objective},
          {"objective_improvement_ratio",
           result.metrics.objective_improvement_ratio},
          {"solver_iterations", result.metrics.solver_iterations},
          {"runtime_ms", result.metrics.runtime_ms}}},
        {"solver_summary", result.solver_summary}};
    fs::create_directories(output);
    std::ofstream(output / "calibration_result.json")
        << std::setw(2) << report << '\n';
    std::cout << std::setw(2) << report << '\n';
    return conformance ? 0 : 3;
  } catch (const std::exception &error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}

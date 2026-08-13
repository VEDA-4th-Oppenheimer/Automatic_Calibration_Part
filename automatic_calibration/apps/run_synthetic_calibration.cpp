#include "auto_calib/calibration_core.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
namespace fs = std::filesystem;
namespace {
using Args = std::unordered_map<std::string, std::string>;
constexpr double pi = 3.14159265358979323846;
double rad(double d) { return d * pi / 180; }
Args parse(int argc, char **argv) {
  Args out;
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    if (k == "--help") {
      out[k] = "";
      continue;
    }
    if (k.rfind("--", 0) != 0 || i + 1 >= argc)
      throw std::invalid_argument("Use --key value arguments");
    out[k] = argv[++i];
  }
  return out;
}
double value(const Args &a, const std::string &k, double d) {
  auto i = a.find(k);
  return i == a.end() ? d : std::stod(i->second);
}
std::uint32_t uvalue(const Args &a, const std::string &k, std::uint32_t d) {
  auto i = a.find(k);
  return i == a.end() ? d : static_cast<std::uint32_t>(std::stoul(i->second));
}
nlohmann::json transformJson(const auto_calib::Transform &t) {
  nlohmann::json r = nlohmann::json::array();
  for (int i = 0; i < 3; ++i)
    r.push_back({t.rotation(i, 0), t.rotation(i, 1), t.rotation(i, 2)});
  return {{"rotation_matrix", r},
          {"translation_m",
           {t.translation_m.x(), t.translation_m.y(), t.translation_m.z()}}};
}
void usage() {
  std::cout << "run_synthetic_calibration --dataset-root PATH --output PATH "
               "[--frame-id ID] [--initial-roll-delta-deg D "
               "--initial-pitch-delta-deg D --initial-yaw-delta-deg D] "
               "[--initial-tx-delta-m M --initial-ty-delta-m M "
               "--initial-tz-delta-m M]\n";
}
} // namespace
int main(int argc, char **argv) {
  try {
    Args a = parse(argc, argv);
    if (a.count("--help")) {
      usage();
      return 0;
    }
    if (!a.count("--dataset-root") || !a.count("--output")) {
      usage();
      return 2;
    }
    std::optional<std::string> id;
    if (a.count("--frame-id"))
      id = a.at("--frame-id");
    auto frame = auto_calib::loadStanfordFrame(a.at("--dataset-root"), id);
    auto_calib::ScanConfig scan_config;
    scan_config.columns = uvalue(a, "--columns", 321);
    scan_config.rows = uvalue(a, "--rows", 121);
    scan_config.pixel_stride = uvalue(a, "--pixel-stride", 2);
    scan_config.noise_stddev = value(a, "--noise-stddev-m", 0.005);
    scan_config.dropout = value(a, "--dropout", 0.01);
    scan_config.seed = uvalue(a, "--seed", 7);
    Eigen::Vector3d gt_t{0.15, -0.02, 0.08}, gt_rpy{rad(2), rad(-4), rad(6)};
    auto ground_truth = auto_calib::makeTransform(gt_t, gt_rpy);
    auto camera_points = auto_calib::projectDepth(frame.depth, frame.camera,
                                                  scan_config.pixel_stride);
    auto scan =
        auto_calib::generateScan(camera_points, ground_truth, scan_config);
    fs::path output = a.at("--output");
    auto_calib::writePackage(output / "input", frame, scan, ground_truth);
    Eigen::Vector3d initial_t =
        gt_t + Eigen::Vector3d(value(a, "--initial-tx-delta-m", 0.02),
                               value(a, "--initial-ty-delta-m", -0.015),
                               value(a, "--initial-tz-delta-m", 0.025));
    Eigen::Vector3d initial_rpy =
        gt_rpy +
        Eigen::Vector3d(rad(value(a, "--initial-roll-delta-deg", 1.0)),
                        rad(value(a, "--initial-pitch-delta-deg", -1.5)),
                        rad(value(a, "--initial-yaw-delta-deg", 2.0)));
    auto initial = auto_calib::makeTransform(initial_t, initial_rpy);
    auto_calib::CalibrationConfig config;
    config.minimum_lidar_edge_points = uvalue(a, "--minimum-lidar-edges", 50);
    config.maximum_solver_iterations =
        static_cast<int>(uvalue(a, "--max-iterations", 100));
    config.prior_weight = value(a, "--prior-weight", 1.0);
    config.rotation_prior_sigma_rad =
        rad(value(a, "--rotation-prior-sigma-deg", 2.8648));
    config.translation_prior_sigma_m =
        value(a, "--translation-prior-sigma-m", 0.02);
    config.maximum_mean_edge_distance_px =
        value(a, "--maximum-mean-edge-distance-px", 40.0);
    config.residual_cap_px = value(a, "--residual-cap-px", 20.0);
    auto result = auto_calib::calibrateExtrinsic(frame.rgb, frame.camera, scan,
                                                 initial, config);
    auto initial_error = auto_calib::calculatePoseError(initial, ground_truth),
         final_error = auto_calib::calculatePoseError(
             result.estimated_t_camera_lidar, ground_truth);
    const double maximum_rotation_error =
        value(a, "--maximum-rotation-error-deg", 3.0);
    const double maximum_translation_error =
        value(a, "--maximum-translation-error-m", 0.05);
    const bool conformance =
        result.success && final_error.rotation_deg <= maximum_rotation_error &&
        final_error.translation_m <= maximum_translation_error;
    nlohmann::json report = {
        {"status", conformance ? "PASS" : "FAIL"},
        {"core_status", result.success ? "PASS" : "FAIL"},
        {"reason_code",
         conformance ? result.reason_code
                     : (result.success ? "GROUND_TRUTH_TOLERANCE_EXCEEDED"
                                       : result.reason_code)},
        {"frame_id", frame.id},
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
          {"projected_edge_points", result.metrics.projected_edge_points},
          {"projected_ratio", result.metrics.projected_ratio},
          {"initial_mean_edge_distance_px",
           result.metrics.initial_mean_edge_distance_px},
          {"final_mean_edge_distance_px",
           result.metrics.final_mean_edge_distance_px},
          {"solver_iterations", result.metrics.solver_iterations},
          {"runtime_ms", result.metrics.runtime_ms}}},
        {"solver_summary", result.solver_summary}};
    fs::create_directories(output);
    std::ofstream(output / "calibration_result.json")
        << std::setw(2) << report << '\n';
    std::cout << std::setw(2) << report << '\n';
    return conformance ? 0 : 3;
  } catch (const std::exception &e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
}

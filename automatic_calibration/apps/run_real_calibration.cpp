#include "auto_calib/calibration_core.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kFinalistSeparationAngleDeg = 15.0;
double radians(double degrees) { return degrees * kPi / 180.0; }
using Args = std::unordered_map<std::string, std::string>;
struct CameraCalibration {
  auto_calib::CameraModel camera;
  cv::Size resolution;
  std::string profile_id;
  std::string intrinsics_source = "manufacturer_fov_initialization";
  std::string manual_intrinsic_path;
  std::string distortion_model = "none";
  std::vector<double> distortion;
  std::string image_distortion_state = "unknown";
  double horizontal_fov_min_deg = 53.0;
  double horizontal_fov_max_deg = 100.0;
  double vertical_fov_min_deg = 30.0;
  double vertical_fov_max_deg = 54.0;
};
struct LoadedScan {
  auto_calib::Scan scan;
  nlohmann::json statistics;
};
Args parseArgs(int argc, char **argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    std::string key = argv[i];
    if (key == "--help") {
      args[key] = "";
      continue;
    }
    if (key.rfind("--", 0) != 0 || i + 1 >= argc)
      throw std::invalid_argument("Use --key value arguments");
    args[key] = argv[++i];
  }
  return args;
}
double value(const Args &args, const std::string &key, double fallback) {
  auto it = args.find(key);
  return it == args.end() ? fallback : std::stod(it->second);
}
std::string textValue(const Args &args, const std::string &key,
                      const std::string &fallback) {
  auto it = args.find(key);
  return it == args.end() ? fallback : it->second;
}
std::vector<double> numericRange(const Args &args, const std::string &minimum_key,
                                 const std::string &maximum_key,
                                 const std::string &step_key,
                                 double fallback) {
  const bool requested = args.count(minimum_key) || args.count(maximum_key) ||
                         args.count(step_key);
  if (!requested)
    return {fallback};
  if (!args.count(minimum_key) || !args.count(maximum_key) ||
      !args.count(step_key))
    throw std::invalid_argument(minimum_key + ", " + maximum_key + " and " +
                                step_key + " must be supplied together");
  const double minimum = value(args, minimum_key, fallback);
  const double maximum = value(args, maximum_key, fallback);
  const double step = value(args, step_key, 0.0);
  if (step <= 0.0 || maximum < minimum)
    throw std::invalid_argument("Invalid numeric search range");
  std::vector<double> values;
  for (double candidate = minimum; candidate <= maximum + 0.25 * step;
       candidate += step)
    values.push_back(candidate);
  return values;
}
std::vector<fs::path> files(const fs::path &dir,
                            const std::vector<std::string> &extensions) {
  std::vector<fs::path> out;
  for (const auto &entry : fs::directory_iterator(dir))
    if (entry.is_regular_file() &&
        std::find(extensions.begin(), extensions.end(),
                  entry.path().extension().string()) != extensions.end())
      out.push_back(entry.path());
  if (out.empty())
    for (const auto &entry : fs::recursive_directory_iterator(dir))
      if (entry.is_regular_file() &&
          std::find(extensions.begin(), extensions.end(),
                    entry.path().extension().string()) != extensions.end())
        out.push_back(entry.path());
  std::sort(out.begin(), out.end());
  return out;
}
bool hasSuffix(const std::string &text, const std::string &suffix) {
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}
struct InputFilePair {
  fs::path image;
  fs::path scan;
};
std::vector<InputFilePair> inputFilePairs(const fs::path &dir,
                                          int camera_channel) {
  auto images = files(dir, {".png", ".jpg", ".jpeg"});
  auto scans = files(dir, {".json"});
  if (camera_channel > 0) {
    const std::string channel_suffix_u = "_CH" + std::to_string(camera_channel);
    const std::string channel_suffix_d = "-CH" + std::to_string(camera_channel);
    const bool has_channel_tag = std::any_of(
        images.begin(), images.end(), [&](const fs::path &p) {
          const std::string stem = p.stem().string();
          return stem.find("_CH") != std::string::npos ||
                 stem.find("-CH") != std::string::npos;
        });
    if (has_channel_tag) {
      images.erase(
          std::remove_if(images.begin(), images.end(),
                         [&](const fs::path &path) {
                           const std::string stem = path.stem().string();
                           return !hasSuffix(stem, channel_suffix_u) &&
                                  !hasSuffix(stem, channel_suffix_d);
                         }),
          images.end());
    }
  }
  scans.erase(
      std::remove_if(scans.begin(), scans.end(),
                     [](const fs::path &path) {
                       const std::string filename = path.filename().string();
                       return filename == "camera_intrinsic.json" ||
                              filename == "manifest.json" ||
                              filename == "calibration_result.json" ||
                              filename == "full_search_baseline_result.json" ||
                              filename == "reference_rt_perturbation_result.json" ||
                              filename == "installation_constrained_rt.json";
                     }),
      scans.end());
  if (images.size() != scans.size() || images.empty())
    throw std::runtime_error(
        "Expected at least one image/LiDAR JSON pair; select a camera "
        "channel when reading a packaged multi-channel dataset");

  const bool nested = std::any_of(images.begin(), images.end(), [&](const auto &p) {
    return p.parent_path() != dir;
  });
  std::vector<InputFilePair> pairs;
  if (!nested) {
    for (std::size_t i = 0; i < images.size(); ++i)
      pairs.push_back({images[i], scans[i]});
    return pairs;
  }

  std::map<fs::path, fs::path> image_by_directory;
  std::map<fs::path, fs::path> scan_by_directory;
  for (const auto &image : images)
    if (!image_by_directory.emplace(image.parent_path(), image).second)
      throw std::runtime_error("Multiple selected camera images in package: " +
                               image.parent_path().string());
  for (const auto &scan : scans)
    if (!scan_by_directory.emplace(scan.parent_path(), scan).second)
      throw std::runtime_error("Multiple LiDAR JSON files in package: " +
                               scan.parent_path().string());
  for (const auto &[parent, image] : image_by_directory) {
    const auto scan = scan_by_directory.find(parent);
    if (scan == scan_by_directory.end())
      throw std::runtime_error("No LiDAR JSON for image package: " +
                               parent.string());
    pairs.push_back({image, scan->second});
  }
  if (pairs.size() != scan_by_directory.size())
    throw std::runtime_error("A LiDAR package has no selected camera image");
  std::sort(pairs.begin(), pairs.end(), [](const auto &a, const auto &b) {
    return std::make_tuple(a.scan.filename(), a.image.filename()) <
           std::make_tuple(b.scan.filename(), b.image.filename());
  });
  return pairs;
}
double focalFromFov(int image_size, double fov_deg) {
  return image_size / (2.0 * std::tan(radians(fov_deg) * 0.5));
}
CameraCalibration cameraFromManufacturerFov(cv::Size resolution) {
  CameraCalibration out;
  out.resolution = resolution;
  out.camera.width = resolution.width;
  out.camera.height = resolution.height;
  const double fx_min =
      focalFromFov(resolution.width, out.horizontal_fov_max_deg);
  const double fx_max =
      focalFromFov(resolution.width, out.horizontal_fov_min_deg);
  const double fy_min =
      focalFromFov(resolution.height, out.vertical_fov_max_deg);
  const double fy_max =
      focalFromFov(resolution.height, out.vertical_fov_min_deg);
  out.camera.k << (fx_min + fx_max) * 0.5, 0.0, resolution.width * 0.5, 0.0,
      (fy_min + fy_max) * 0.5, resolution.height * 0.5, 0.0, 0.0, 1.0;
  out.profile_id = "PNM-C16083RVQ-manufacturer-fov-initialization";
  return out;
}
CameraCalibration cameraFromManualIntrinsic(const fs::path &path,
                                            cv::Size resolution) {
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("Cannot open manual intrinsic JSON: " +
                             path.string());
  nlohmann::json root;
  stream >> root;
  const auto &camera_json =
      root.contains("camera") ? root.at("camera") : root;
  const auto &intrinsic_json =
      camera_json.contains("intrinsic") ? camera_json.at("intrinsic")
                                        : camera_json;
  const auto read_number = [&](const char *key) {
    if (!intrinsic_json.contains(key) || !intrinsic_json.at(key).is_number())
      throw std::runtime_error(std::string("Manual intrinsic is missing ") +
                               key);
    const double value = intrinsic_json.at(key).get<double>();
    if (!std::isfinite(value))
      throw std::runtime_error(std::string("Manual intrinsic is not finite: ") +
                               key);
    return value;
  };
  CameraCalibration out;
  out.resolution = resolution;
  if (!camera_json.contains("resolution") ||
      !camera_json.at("resolution").is_array())
    throw std::runtime_error(
        "Manual camera profile must declare resolution");
  const auto &size = camera_json.at("resolution");
  if (size.size() != 2 || size.at(0).get<int>() != resolution.width ||
      size.at(1).get<int>() != resolution.height)
    throw std::runtime_error(
        "Manual intrinsic resolution does not match camera image");
  out.camera.width = resolution.width;
  out.camera.height = resolution.height;
  out.camera.k << read_number("fx"), 0.0, read_number("cx"), 0.0,
      read_number("fy"), read_number("cy"), 0.0, 0.0, 1.0;
  if (out.camera.k(0, 0) <= 0.0 || out.camera.k(1, 1) <= 0.0)
    throw std::runtime_error("Manual intrinsic focal lengths must be positive");
  if (!camera_json.contains("distortion") ||
      !camera_json.at("distortion").is_array())
    throw std::runtime_error(
        "Manual camera profile must declare distortion coefficients");
  for (const auto &coefficient : camera_json.at("distortion")) {
    const double value = coefficient.get<double>();
    if (!std::isfinite(value))
      throw std::runtime_error(
          "Manual distortion contains a non-finite coefficient");
    out.distortion.push_back(value);
  }
  out.distortion_model =
      camera_json.value("distortion_model",
                        out.distortion.empty() ? "none" : "opencv_radtan");
  out.profile_id =
      camera_json.value("intrinsic_profile_id", path.stem().string());
  out.intrinsics_source = "manual_intrinsic_json";
  out.manual_intrinsic_path = path.string();
  return out;
}
auto_calib::Transform loadTransformJson(const fs::path &path) {
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("Cannot open transform JSON: " + path.string());
  nlohmann::json document;
  stream >> document;
  const nlohmann::json *value = &document;
  for (const char *key :
       {"extrinsic", "estimated", "transform", "manual_transform",
        "automatic_transform", "estimated_t_camera_lidar",
        "visualization_t_camera_lidar", "t_camera_lidar"}) {
    if (value->contains(key)) {
      value = &value->at(key);
      break;
    }
  }
  if (!value->contains("rotation_matrix") ||
      !value->contains("translation_m"))
    throw std::runtime_error(
        "Transform JSON must contain rotation_matrix and translation_m: " +
        path.string());
  auto_calib::Transform transform;
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      transform.rotation(row, column) =
          value->at("rotation_matrix").at(row).at(column).get<double>();
  for (int axis = 0; axis < 3; ++axis)
    transform.translation_m(axis) =
        value->at("translation_m").at(axis).get<double>();
  if (!transform.rotation.allFinite() ||
      !transform.translation_m.allFinite())
    throw std::runtime_error("Transform JSON contains non-finite values: " +
                             path.string());
  if ((transform.rotation.transpose() * transform.rotation -
       Eigen::Matrix3d::Identity())
          .norm() > 1e-3 ||
      std::abs(transform.rotation.determinant() - 1.0) > 1e-3)
    throw std::runtime_error(
        "Transform JSON rotation is not a proper rotation matrix: " +
        path.string());
  return transform;
}
LoadedScan loadScan(const fs::path &path, double minimum_range,
                    double maximum_range, double minimum_signal,
                    double legacy_range_offset) {
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("Cannot open LiDAR JSON: " + path.string());
  nlohmann::json root;
  stream >> root;
  LoadedScan out;
  const std::string schema_version = root.value("schema_version", "unknown");
  const bool header_has_range_offset =
      root.contains("sensor") && root.at("sensor").contains("range_offset_m");
  if (!header_has_range_offset && schema_version != "1.1")
    throw std::runtime_error(
        "Self-describing LiDAR JSON requires sensor.range_offset_m");
  if (!header_has_range_offset && !std::isfinite(legacy_range_offset))
    throw std::runtime_error(
        "Legacy LiDAR JSON requires --legacy-range-offset-m");
  const double range_offset =
      header_has_range_offset
          ? root.at("sensor").at("range_offset_m").get<double>()
          : legacy_range_offset;
  if (!std::isfinite(range_offset) || range_offset < 0.0)
    throw std::runtime_error("Invalid LiDAR range offset");
  if (!root.contains("frame") ||
      root.at("frame").value("name", "") != "lidar_scan")
    throw std::runtime_error("Expected frame.name=lidar_scan");
  const auto &frame = root.at("frame");
  if (frame.value("handedness", "") != "right" ||
      frame.value("convention", "") !=
          "+x right, +y down, +z forward; pan+ right, tilt+ up")
    throw std::runtime_error("Unsupported LiDAR frame convention");
  const auto &metadata = root.at("scan");
  out.scan.config.rows = metadata.at("rows").get<std::uint32_t>();
  out.scan.config.columns = metadata.at("columns").get<std::uint32_t>();
  out.scan.source_count = metadata.at("sample_count").get<std::size_t>();
  const std::size_t count =
      static_cast<std::size_t>(out.scan.config.rows) * out.scan.config.columns;
  out.scan.points.resize(count);
  const std::string tilt_zero =
      root.contains("mechanism")
          ? root.at("mechanism").value("tilt_zero", "unspecified")
          : "unspecified";
  std::size_t accepted = 0, invalid = 0, range_rejected = 0,
              signal_rejected = 0;
  for (const auto &m : root.at("measurements")) {
    const auto row = m.at("row").get<std::uint32_t>();
    const auto column = m.at("column").get<std::uint32_t>();
    if (row >= out.scan.config.rows || column >= out.scan.config.columns)
      throw std::runtime_error("Measurement cell outside scan dimensions");
    auto &point =
        out.scan
            .points[static_cast<std::size_t>(row) * out.scan.config.columns +
                    column];
    point.row = row;
    point.column = column;
    const bool source_valid = m.value("valid", false) &&
                              m.value("checksum_valid", false) &&
                              !m.at("distance_m").is_null();
    if (!source_valid) {
      ++invalid;
      continue;
    }
    point.timestamp = m.at("timestamp_ns").get<std::int64_t>();
    point.pan = m.at("pan_rad").get<float>();
    point.tilt = m.at("tilt_rad").get<float>();
    const double range = m.at("distance_m").get<double>() + range_offset;
    if (!std::isfinite(range) || range < minimum_range ||
        range > maximum_range) {
      ++range_rejected;
      continue;
    }
    const double signal = m.value("signal_strength", 0.0);
    if (signal < minimum_signal) {
      ++signal_rejected;
      continue;
    }
    const double tilt = point.tilt;
    const double horizontal = range * std::cos(tilt);
    point.range = static_cast<float>(range);
    point.signal_strength = static_cast<float>(signal);
    if (!m.at("range_precision_m").is_null())
      point.precision = m.at("range_precision_m").get<float>();
    point.xyz =
        Eigen::Vector3f(static_cast<float>(horizontal * std::sin(point.pan)),
                        static_cast<float>(-range * std::sin(tilt)),
                        static_cast<float>(horizontal * std::cos(point.pan)));
    point.flags = auto_calib::kValidRange;
    ++accepted;
  }
  out.scan.valid_count = accepted;
  out.statistics = {
      {"file", path.filename().string()},
      {"schema_version", schema_version},
      {"range_offset_m", range_offset},
      {"range_offset_source",
       header_has_range_offset ? "json_header" : "legacy_cli"},
      {"cells", count},
      {"accepted", accepted},
      {"rejected_invalid", invalid},
      {"rejected_range", range_rejected},
      {"rejected_signal", signal_rejected},
      {"minimum_range_m", minimum_range},
      {"maximum_range_m", maximum_range},
      {"minimum_signal", minimum_signal},
      {"source_valid_count", metadata.value("valid_count", 0)},
      {"source_duplicate_cell_diagnostic",
       root.at("diagnostics").value("duplicate_cell_count", 0)},
      {"tilt_zero_metadata", tilt_zero},
      {"tilt_zero_role", "mechanism_home_only"},
      {"measurement_tilt_reference",
       "scan.tilt_rad: zero=horizontal, negative=down"},
      {"frame_convention", frame.at("convention")},
      {"frame_handedness", frame.at("handedness")},
      {"range_formula", "x=r*cos(tilt)*sin(pan); y=-r*sin(tilt); "
                        "z=r*cos(tilt)*cos(pan)"},
      {"coordinate_contract_status", "validated_from_frame_range_formula"},
      {"coordinate_contract_warning",
       schema_version == "1.1"
           ? nlohmann::json(
                 "Legacy schema 1.1: range offset supplied by operator")
           : nlohmann::json(nullptr)}};
  return out;
}
cv::Mat loadImage(const fs::path &path, CameraCalibration *calibration,
                  auto_calib::CameraModel *camera) {
  cv::Mat image = cv::imread(path.string(), cv::IMREAD_COLOR);
  if (image.empty())
    throw std::runtime_error("Cannot read image: " + path.string());
  if (image.size() != calibration->resolution)
    throw std::runtime_error("All camera images must have one resolution");
  if (calibration->image_distortion_state == "raw" &&
      !calibration->distortion.empty()) {
    const cv::Mat coefficients(
        1, static_cast<int>(calibration->distortion.size()), CV_64F,
        calibration->distortion.data());
    cv::Mat camera_matrix(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row)
      for (int column = 0; column < 3; ++column)
        camera_matrix.at<double>(row, column) =
            calibration->camera.k(row, column);
    cv::Mat rectified;
    cv::undistort(image, rectified, camera_matrix, coefficients,
                  camera_matrix);
    image = std::move(rectified);
  }
  *camera = calibration->camera;
  return image;
}
nlohmann::json transformJson(const auto_calib::Transform &t) {
  nlohmann::json rotation = nlohmann::json::array();
  for (int r = 0; r < 3; ++r)
    rotation.push_back({t.rotation(r, 0), t.rotation(r, 1), t.rotation(r, 2)});
  return {{"rotation_matrix", rotation},
          {"translation_m",
          {t.translation_m.x(), t.translation_m.y(), t.translation_m.z()}}};
}
auto_calib::Transform installationTransform(
    const Eigen::Vector3d &camera_center_lidar,
    const Eigen::Vector3d &camera_forward_lidar,
    const Eigen::Vector3d &camera_down_lidar) {
  const Eigen::Vector3d forward = camera_forward_lidar.normalized();
  Eigen::Vector3d down =
      camera_down_lidar - forward * forward.dot(camera_down_lidar);
  if (down.norm() <= 1e-9)
    throw std::invalid_argument(
        "Installed camera forward and down vectors must not be parallel");
  down.normalize();
  const Eigen::Vector3d right = down.cross(forward).normalized();
  auto_calib::Transform transform;
  transform.rotation.row(0) = right.transpose();
  transform.rotation.row(1) = down.transpose();
  transform.rotation.row(2) = forward.transpose();
  transform.translation_m = -transform.rotation * camera_center_lidar;
  return transform;
}
nlohmann::json cameraJson(const auto_calib::CameraModel &camera) {
  return {{"fx", camera.k(0, 0)},
          {"fy", camera.k(1, 1)},
          {"cx", camera.k(0, 2)},
          {"cy", camera.k(1, 2)},
          {"resolution", {camera.width, camera.height}}};
}
cv::Scalar rangeColor(float range) {
  const double t =
      std::clamp((static_cast<double>(range) - 0.3) / 4.0, 0.0, 1.0);
  return {255.0 * (1.0 - t), 220.0 * (1.0 - std::abs(2.0 * t - 1.0)),
          255.0 * t};
}
bool project(const Eigen::Vector3d &point,
             const auto_calib::CameraModel &camera,
             const auto_calib::Transform &transform, cv::Point *pixel,
             double *depth = nullptr) {
  const Eigen::Vector3d camera_point = transform.lidarToCamera(point);
  if (camera_point.z() <= 0.05)
    return false;
  const int x = static_cast<int>(std::lround(
      camera.k(0, 0) * camera_point.x() / camera_point.z() + camera.k(0, 2)));
  const int y = static_cast<int>(std::lround(
      camera.k(1, 1) * camera_point.y() / camera_point.z() + camera.k(1, 2)));
  if (x < 0 || y < 0 || x >= camera.width || y >= camera.height)
    return false;
  *pixel = {x, y};
  if (depth)
    *depth = camera_point.z();
  return true;
}
std::vector<double> buildZBuffer(const auto_calib::Scan &scan,
                                 const auto_calib::CameraModel &camera,
                                 const auto_calib::Transform &transform) {
  std::vector<double> z(static_cast<std::size_t>(camera.width) * camera.height,
                        std::numeric_limits<double>::infinity());
  for (const auto &point : scan.points) {
    if (!point.valid())
      continue;
    cv::Point pixel;
    double depth = 0.0;
    if (project(point.xyz.cast<double>(), camera, transform, &pixel, &depth)) {
      const auto index = static_cast<std::size_t>(pixel.y) * camera.width +
                         static_cast<std::size_t>(pixel.x);
      z[index] = std::min(z[index], depth);
    }
  }
  return z;
}
bool visibleAt(const cv::Point &pixel, double depth,
               const auto_calib::CameraModel &camera,
               const std::vector<double> &z) {
  const auto index = static_cast<std::size_t>(pixel.y) * camera.width +
                     static_cast<std::size_t>(pixel.x);
  return depth <= z[index] + 0.01;
}
std::size_t writeMatchingVisualization(
    const auto_calib::CalibrationObservation &observation,
    const auto_calib::Transform &transform,
    const auto_calib::CalibrationConfig &config, const fs::path &path,
    const std::string &status) {
  cv::Mat points = observation.bgr.clone();
  const auto z_buffer =
      buildZBuffer(observation.scan, observation.camera, transform);
  std::size_t projected = 0;
  for (const auto &point : observation.scan.points) {
    if (!point.valid())
      continue;
    cv::Point pixel;
    double depth = 0.0;
    if (project(point.xyz.cast<double>(), observation.camera, transform, &pixel,
                &depth) &&
        visibleAt(pixel, depth, observation.camera, z_buffer)) {
      cv::circle(points, pixel, 2, rangeColor(point.range), cv::FILLED,
                 cv::LINE_AA);
      ++projected;
    }
  }
  cv::Mat edges = points.clone();
  for (const auto &edge :
       auto_calib::extractLidarEdgePoints(observation.scan, config)) {
    cv::Point pixel;
    double depth = 0.0;
    if (project(edge, observation.camera, transform, &pixel, &depth) &&
        visibleAt(pixel, depth, observation.camera, z_buffer))
      cv::circle(edges, pixel, 5, {0, 255, 0}, 2, cv::LINE_AA);
  }
  const int panel_width = 720;
  const int panel_height =
      observation.bgr.rows * panel_width / observation.bgr.cols;
  cv::Mat comparison(panel_height, panel_width * 3, CV_8UC3);
  const cv::Mat panels[] = {observation.bgr, points, edges};
  const std::string labels[] = {
      "Original", "All LiDAR points | projected=" + std::to_string(projected),
      "LiDAR depth edges (green) | " + status};
  for (int i = 0; i < 3; ++i) {
    cv::Mat resized;
    cv::resize(panels[i], resized, {panel_width, panel_height}, 0, 0,
               cv::INTER_AREA);
    cv::rectangle(resized, {0, 0}, {panel_width, 34}, {0, 0, 0}, cv::FILLED);
    cv::putText(resized, labels[i], {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.58,
                {255, 255, 255}, 2, cv::LINE_AA);
    resized.copyTo(
        comparison(cv::Rect(i * panel_width, 0, panel_width, panel_height)));
  }
  if (!cv::imwrite(path.string(), comparison))
    throw std::runtime_error("Cannot write matching visualization");
  return projected;
}
struct ColoredPoint {
  Eigen::Vector3d xyz;
  cv::Vec3b bgr;
  bool image_colored = false;
};
Eigen::Vector3d viewerZUp(const Eigen::Vector3d &point) {
  return {point.x(), point.z(), -point.y()};
}

void writePointViewerMesh(const std::vector<ColoredPoint> &points,
                          const fs::path &stem, bool z_up,
                          const auto_calib::Transform &camera_pose) {
  constexpr std::size_t maximum_points = 12000;
  constexpr double radius_m = 0.025;
  constexpr int faces[4][3] = {{0, 1, 2}, {0, 3, 1},
                               {0, 2, 3}, {1, 3, 2}};
  const std::size_t stride =
      std::max<std::size_t>(1, (points.size() + maximum_points - 1) /
                                   maximum_points);
  std::vector<const ColoredPoint *> selected;
  for (std::size_t i = 0; i < points.size(); i += stride)
    selected.push_back(&points[i]);
  const Eigen::Vector3d camera_center =
      -camera_pose.rotation.transpose() * camera_pose.translation_m;
  const Eigen::Vector3d camera_forward =
      camera_pose.rotation.transpose() * Eigen::Vector3d::UnitZ();
  std::vector<ColoredPoint> camera_marker;
  camera_marker.reserve(24);
  for (int i = 0; i <= 20; ++i)
    camera_marker.push_back(
        {camera_center + 0.05 * i * camera_forward, {255, 255, 0}, true});
  for (const double side : {-0.10, 0.10}) {
    camera_marker.push_back(
        {camera_center + camera_forward + side * Eigen::Vector3d::UnitX(),
         {255, 255, 0}, true});
    camera_marker.push_back(
        {camera_center + camera_forward + side * Eigen::Vector3d::UnitY(),
         {255, 255, 0}, true});
  }
  for (const auto &point : camera_marker)
    selected.push_back(&point);
  const fs::path ply = stem.string() + ".ply";
  const fs::path obj = stem.string() + ".obj";
  std::ofstream ply_out(ply), obj_out(obj);
  if (!ply_out || !obj_out)
    throw std::runtime_error("Cannot write VS Code viewer mesh");
  ply_out << "ply\nformat ascii 1.0\ncomment VS Code 3D Viewer triangle "
             "mesh preview, units m\n"
          << "element vertex " << selected.size() * 4
          << "\nproperty float x\nproperty float y\nproperty float z\n"
             "property float nx\nproperty float ny\nproperty float nz\n"
             "property uchar red\nproperty uchar green\nproperty uchar blue\n"
          << "element face " << selected.size() * 4
          << "\nproperty list uchar int vertex_indices\nend_header\n"
          << std::setprecision(9);
  obj_out << "# VS Code 3D Viewer triangle mesh preview\n# units m\n"
          << "# cyan markers: installed camera center and optical axis\n"
          << std::setprecision(9);
  const Eigen::Vector3d offsets[4] = {
      {radius_m, radius_m, radius_m},
      {radius_m, -radius_m, -radius_m},
      {-radius_m, radius_m, -radius_m},
      {-radius_m, -radius_m, radius_m}};
  for (const auto *point : selected) {
    Eigen::Vector3d center = z_up ? viewerZUp(point->xyz) : point->xyz;
    for (const auto &offset : offsets) {
      const Eigen::Vector3d vertex = center + offset;
      const Eigen::Vector3d normal = offset.normalized();
      ply_out << vertex.x() << ' ' << vertex.y() << ' ' << vertex.z() << ' '
              << normal.x() << ' ' << normal.y() << ' ' << normal.z() << ' '
              << static_cast<int>(point->bgr[2]) << ' '
              << static_cast<int>(point->bgr[1]) << ' '
              << static_cast<int>(point->bgr[0]) << '\n';
      obj_out << "v " << vertex.x() << ' ' << vertex.y() << ' '
              << vertex.z() << '\n';
      obj_out << "vn " << normal.x() << ' ' << normal.y() << ' '
              << normal.z() << '\n';
    }
  }
  for (std::size_t i = 0; i < selected.size(); ++i) {
    for (const auto &face : faces) {
      const std::size_t base = i * 4;
      ply_out << "3 " << base + face[0] << ' ' << base + face[1] << ' '
              << base + face[2] << '\n';
      obj_out << "f " << base + face[0] + 1 << "//" << base + face[0] + 1
              << ' ' << base + face[1] + 1 << "//" << base + face[1] + 1
              << ' ' << base + face[2] + 1 << "//" << base + face[2] + 1
              << '\n';
    }
  }
}

std::size_t
writeColorizedPointCloud(const auto_calib::CalibrationObservation &observation,
                         const auto_calib::Transform &transform,
                         const fs::path &stem, const std::string &status) {
  std::vector<ColoredPoint> points;
  points.reserve(observation.scan.valid_count);
  const auto z_buffer =
      buildZBuffer(observation.scan, observation.camera, transform);
  std::size_t colored = 0;
  for (const auto &point : observation.scan.points) {
    if (!point.valid())
      continue;
    ColoredPoint out{point.xyz.cast<double>(), {96, 96, 96}, false};
    cv::Point pixel;
    double depth = 0.0;
    if (project(out.xyz, observation.camera, transform, &pixel, &depth) &&
        visibleAt(pixel, depth, observation.camera, z_buffer)) {
      out.bgr = observation.bgr.at<cv::Vec3b>(pixel);
      out.image_colored = true;
      ++colored;
    }
    points.push_back(out);
  }
  const fs::path ply = stem.string() + ".ply";
  std::ofstream ply_out(ply);
  if (!ply_out)
    throw std::runtime_error("Cannot write PLY");
  ply_out << "ply\nformat ascii 1.0\ncomment frame lidar_scan\ncomment "
             "units mm\ncomment "
          << status << "\nelement vertex " << points.size()
          << "\nproperty float x\nproperty float y\nproperty float z\n"
             "property uchar red\nproperty uchar green\nproperty uchar blue\n"
             "end_header\n"
          << std::setprecision(9);
  for (const auto &point : points) {
    const Eigen::Vector3d p = 1000.0 * point.xyz;
    ply_out << p.x() << " " << p.y() << " " << p.z() << " "
            << static_cast<int>(point.bgr[2]) << " "
            << static_cast<int>(point.bgr[1]) << " "
            << static_cast<int>(point.bgr[0]) << "\n";
  }

  const fs::path obj = stem.string() + ".obj";
  std::ofstream obj_out(obj);
  if (!obj_out)
    throw std::runtime_error("Cannot write OBJ");
  obj_out << "# frame lidar_scan\n# units mm\n# " << status << "\n"
          << std::setprecision(9);
  for (const auto &point : points) {
    const Eigen::Vector3d p = 1000.0 * point.xyz;
    obj_out << "v " << p.x() << " " << p.y() << " " << p.z() << " "
            << point.bgr[2] / 255.0 << " " << point.bgr[1] / 255.0 << " "
            << point.bgr[0] / 255.0 << "\n";
  }
  for (std::size_t i = 0; i < points.size(); ++i)
    obj_out << "p " << (i + 1) << "\n";

  const fs::path z_up_ply = stem.string() + "_z_up.ply";
  std::ofstream z_up_ply_out(z_up_ply);
  if (!z_up_ply_out)
    throw std::runtime_error("Cannot write Z-up PLY");
  z_up_ply_out
      << "ply\nformat ascii 1.0\ncomment frame viewer_z_up "
         "(+x right +y forward +z up), units mm\ncomment "
      << status << "\nelement vertex " << points.size()
      << "\nproperty float x\nproperty float y\nproperty float z\n"
         "property uchar red\nproperty uchar green\nproperty uchar blue\n"
         "end_header\n"
      << std::setprecision(9);
  for (const auto &point : points) {
    const Eigen::Vector3d xyz = 1000.0 * viewerZUp(point.xyz);
    z_up_ply_out << xyz.x() << " " << xyz.y() << " " << xyz.z() << " "
                 << static_cast<int>(point.bgr[2]) << " "
                 << static_cast<int>(point.bgr[1]) << " "
                 << static_cast<int>(point.bgr[0]) << "\n";
  }

  const fs::path reprojection_ply =
      stem.string() + "_z_up_reprojection_m.ply";
  std::ofstream reprojection_out(reprojection_ply);
  if (!reprojection_out)
    throw std::runtime_error("Cannot write meter reprojection PLY");
  reprojection_out
      << "ply\nformat ascii 1.0\ncomment image-colored LiDAR reprojection\n"
         "comment frame viewer_z_up (+x right +y forward +z up)\n"
         "comment units m\n"
      << "element vertex " << points.size()
      << "\nproperty float x\nproperty float y\nproperty float z\n"
         "property uchar red\nproperty uchar green\nproperty uchar blue\n"
         "end_header\n"
      << std::setprecision(9);
  for (const auto &point : points) {
    const Eigen::Vector3d xyz = viewerZUp(point.xyz);
    reprojection_out << xyz.x() << ' ' << xyz.y() << ' ' << xyz.z() << ' '
                     << static_cast<int>(point.bgr[2]) << ' '
                     << static_cast<int>(point.bgr[1]) << ' '
                     << static_cast<int>(point.bgr[0]) << '\n';
  }

  const fs::path z_up_obj = stem.string() + "_z_up.obj";
  std::ofstream z_up_obj_out(z_up_obj);
  if (!z_up_obj_out)
    throw std::runtime_error("Cannot write Z-up OBJ");
  z_up_obj_out
      << "# frame viewer_z_up (+x right +y forward +z up)\n# units mm\n# "
      << status << "\n"
      << std::setprecision(9);
  for (const auto &point : points) {
    const Eigen::Vector3d xyz = 1000.0 * viewerZUp(point.xyz);
    z_up_obj_out << "v " << xyz.x() << " " << xyz.y() << " " << xyz.z() << " "
                 << point.bgr[2] / 255.0 << " " << point.bgr[1] / 255.0 << " "
                 << point.bgr[0] / 255.0 << "\n";
  }
  for (std::size_t i = 0; i < points.size(); ++i)
    z_up_obj_out << "p " << (i + 1) << "\n";

  writePointViewerMesh(points, stem.string() + "_viewer_mesh", false,
                       transform);
  writePointViewerMesh(points, stem.string() + "_z_up_viewer_mesh", true,
                       transform);

  std::vector<double> axes[3];
  for (auto &axis : axes)
    axis.reserve(points.size());
  for (const auto &point : points) {
    const Eigen::Vector3d xyz = viewerZUp(point.xyz);
    for (int axis = 0; axis < 3; ++axis)
      axes[axis].push_back(xyz(axis));
  }
  double low[3], high[3];
  for (int axis = 0; axis < 3; ++axis) {
    std::sort(axes[axis].begin(), axes[axis].end());
    low[axis] = axes[axis][axes[axis].size() / 100];
    high[axis] = axes[axis][axes[axis].size() * 99 / 100];
    if (high[axis] - low[axis] < 1e-6)
      high[axis] = low[axis] + 1.0;
  }
  const int size = 560, margin = 35;
  cv::Mat preview(size, size * 3, CV_8UC3, {24, 24, 24});
  const int horizontal[] = {0, 0, 1};
  const int vertical[] = {1, 2, 2};
  const std::string labels[] = {"TOP X-Y", "FRONT X-Z", "SIDE Y-Z"};
  const Eigen::Vector3d camera_center =
      -transform.rotation.transpose() * transform.translation_m;
  const Eigen::Vector3d camera_forward =
      transform.rotation.transpose() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d viewer_center = viewerZUp(camera_center);
  const Eigen::Vector3d viewer_forward = viewerZUp(camera_forward).normalized();
  for (int view = 0; view < 3; ++view) {
    cv::Mat panel = preview(cv::Rect(view * size, 0, size, size));
    cv::rectangle(panel, {0, 0}, {size - 1, size - 1}, {70, 70, 70}, 1);
    for (const auto &point : points) {
      const Eigen::Vector3d xyz = viewerZUp(point.xyz);
      const int h = horizontal[view], v = vertical[view];
      const double u0 = (xyz(h) - low[h]) / (high[h] - low[h]);
      const double v0 = (xyz(v) - low[v]) / (high[v] - low[v]);
      if (u0 < 0.0 || u0 > 1.0 || v0 < 0.0 || v0 > 1.0)
        continue;
      const int x = margin + static_cast<int>(u0 * (size - 2 * margin));
      const int y = size - margin - static_cast<int>(v0 * (size - 2 * margin));
      cv::circle(panel, {x, y}, point.image_colored ? 2 : 1,
                 cv::Scalar(point.bgr), cv::FILLED, cv::LINE_AA);
    }
    const int h = horizontal[view], v = vertical[view];
    const double arrow_length =
        0.18 * std::max(high[h] - low[h], high[v] - low[v]);
    const Eigen::Vector3d viewer_tip =
        viewer_center + arrow_length * viewer_forward;
    auto toPixel = [&](const Eigen::Vector3d &p) {
      const double u = (p(h) - low[h]) / (high[h] - low[h]);
      const double w = (p(v) - low[v]) / (high[v] - low[v]);
      return cv::Point{margin + static_cast<int>(u * (size - 2 * margin)),
                       size - margin -
                           static_cast<int>(w * (size - 2 * margin))};
    };
    const cv::Point center_pixel = toPixel(viewer_center);
    const cv::Point tip_pixel = toPixel(viewer_tip);
    const bool accepted = status.rfind("PASS", 0) == 0;
    const bool installed = status.find("INSTALLATION") != std::string::npos ||
                           status.find("MECHANICAL") != std::string::npos;
    const bool reference = status.rfind("MANUAL RT REFERENCE", 0) == 0;
    const cv::Scalar camera_color =
        accepted ? cv::Scalar(0, 180, 0)
        : installed ? cv::Scalar(255, 255, 0)
        : reference ? cv::Scalar(0, 165, 255)
                    : cv::Scalar(255, 0, 255);
    cv::circle(panel, center_pixel, 7, camera_color, cv::FILLED, cv::LINE_AA);
    cv::arrowedLine(panel, center_pixel, tip_pixel, camera_color, 3, cv::LINE_AA,
                    0, 0.18);
    const std::string camera_label =
        accepted ? "CALIBRATED CAM"
        : installed ? "INSTALLED CAMERA"
        : reference ? "MANUAL RT REF"
                    : "REJECTED RT";
    const cv::Point label_position{
        std::clamp(center_pixel.x + 9, 5, size - 185),
        std::clamp(center_pixel.y - 8, 48, size - 8)};
    cv::putText(panel, camera_label, label_position,
                cv::FONT_HERSHEY_SIMPLEX, 0.5, camera_color, 2, cv::LINE_AA);
    cv::rectangle(panel, {0, 0}, {size, 34}, {0, 0, 0}, cv::FILLED);
    cv::putText(panel, labels[view], {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.62,
                {255, 255, 255}, 2, cv::LINE_AA);
  }
  cv::putText(preview,
              status + " | camera-colored=" + std::to_string(colored) + "/" +
                  std::to_string(points.size()),
              {size + 120, size - 12}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
              {255, 255, 255}, 1, cv::LINE_AA);
  const fs::path preview_path = stem.string() + "_3d_preview.png";
  if (!cv::imwrite(preview_path.string(), preview))
    throw std::runtime_error("Cannot write 3D preview");
  return colored;
}
void writeSegmentMesh(
    const fs::path &stem,
    const std::vector<auto_calib::StructuralLineSegment3d> &segments,
    const std::string &description) {
  std::ofstream ply(stem.string() + ".ply");
  std::ofstream obj(stem.string() + ".obj");
  if (!ply || !obj)
    throw std::runtime_error("Cannot write segment PLY/OBJ: " +
                             stem.string());
  constexpr int faces[12][3] = {
      {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6},
      {0, 5, 1}, {0, 4, 5}, {1, 6, 2}, {1, 5, 6},
      {2, 7, 3}, {2, 6, 7}, {3, 4, 0}, {3, 7, 4}};
  ply << "ply\nformat ascii 1.0\ncomment " << description
      << ", units meter, triangle mesh\n"
      << "element vertex " << segments.size() * 8
      << "\nproperty float x\nproperty float y\nproperty float z\n"
         "property float nx\nproperty float ny\nproperty float nz\n"
         "property uchar red\nproperty uchar green\nproperty uchar blue\n"
      << "element face " << segments.size() * 12
      << "\nproperty list uchar int vertex_indices\nend_header\n"
      << std::setprecision(9);
  obj << "# " << description << " as 10 mm triangle-mesh bars; units meter\n"
      << std::setprecision(9);
  for (std::size_t segment_index = 0; segment_index < segments.size();
       ++segment_index) {
    const auto &segment = segments[segment_index];
    const auto color = cv::Vec3b{
        static_cast<unsigned char>((segment_index * 97U) % 206U + 50U),
        static_cast<unsigned char>((segment_index * 57U) % 206U + 50U),
        static_cast<unsigned char>((segment_index * 31U) % 206U + 50U)};
    const Eigen::Vector3d direction = (segment.b - segment.a).normalized();
    const Eigen::Vector3d helper = std::abs(direction.z()) < 0.9
                                       ? Eigen::Vector3d::UnitZ()
                                       : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d u = 0.005 * direction.cross(helper).normalized();
    const Eigen::Vector3d v = 0.005 * direction.cross(u).normalized();
    const Eigen::Vector3d vertices[8] = {
        segment.a + u + v, segment.a + u - v, segment.a - u - v,
        segment.a - u + v, segment.b + u + v, segment.b + u - v,
        segment.b - u - v, segment.b - u + v};
    const Eigen::Vector3d normals[8] = {
        (u + v).normalized(), (u - v).normalized(), (-u - v).normalized(),
        (-u + v).normalized(), (u + v).normalized(), (u - v).normalized(),
        (-u - v).normalized(), (-u + v).normalized()};
    for (int vertex_index = 0; vertex_index < 8; ++vertex_index) {
      const auto &point = vertices[vertex_index];
      const auto &normal = normals[vertex_index];
      ply << point.x() << ' ' << point.y() << ' ' << point.z() << ' '
          << normal.x() << ' ' << normal.y() << ' ' << normal.z() << ' '
          << static_cast<int>(color[2]) << ' ' << static_cast<int>(color[1])
          << ' ' << static_cast<int>(color[0]) << '\n';
      obj << "v " << point.x() << ' ' << point.y() << ' ' << point.z() << ' '
          << color[2] / 255.0 << ' ' << color[1] / 255.0 << ' '
          << color[0] / 255.0 << '\n';
      obj << "vn " << normal.x() << ' ' << normal.y() << ' ' << normal.z()
          << '\n';
    }
  }
  for (std::size_t segment_index = 0; segment_index < segments.size();
       ++segment_index) {
    const std::size_t base = segment_index * 8;
    for (const auto &face : faces) {
      ply << "3 " << base + face[0] << ' ' << base + face[1] << ' '
          << base + face[2] << '\n';
      obj << "f " << base + face[0] + 1 << "//" << base + face[0] + 1 << ' '
          << base + face[1] + 1 << "//" << base + face[1] + 1 << ' '
          << base + face[2] + 1 << "//" << base + face[2] + 1 << '\n';
    }
  }
}
void writeDebugArtifacts(
    const std::vector<auto_calib::CalibrationObservation> &observations,
    const auto_calib::Transform &initial,
    const auto_calib::Transform &visualization_transform,
    const auto_calib::CameraModel &visualization_camera,
    const std::string &visualization_label,
    const auto_calib::CalibrationConfig &config, const fs::path &root) {
  fs::create_directories(root);
  std::ofstream manifest(root / "README.txt");
  manifest
      << "Calibration debug artifacts\n"
      << "01_input.png: image after input decoding\n"
      << "02_camera_gradient.png: Sobel gradient magnitude used by NID\n"
      << "03_camera_edge_distance.png: Canny edge distance transform\n"
      << "03a_manhattan_vanishing_directions.csv: retained image vanishing "
         "directions and angular distance to the initial/final gravity axis\n"
      << "04_lidar_surface_normals.ply: robust organized-scan normals\n"
      << "04a_lidar_plane_labels.ply: recovered/merged plane support labels\n"
      << "04a_lidar_planes.csv: fitted plane coefficients and quality\n"
      << "04b_lidar_plane_pair_candidates.csv: intersection rejection stage\n"
      << "04b_lidar_plane_intersection_edges.{ply,obj}: intersections of "
         "adjacent fitted planes\n"
      << "04b1_lidar_plane_boundary_edges.{ply,obj}: fitted-plane boundaries "
         "against unlabelled geometry\n"
      << "04c_lidar_occlusion_edges.{ply,obj}: range-discontinuity "
         "silhouettes (diagnostic only)\n"
      << "04c1_lidar_persistent_occlusion_edges.{ply,obj}: occlusion lines "
         "repeated across observations\n"
      << "04d_lidar_edges_used_for_calibration.{ply,obj}: structural edges "
         "used by the objective\n"
      << "05_projection_initial.png: LiDAR points using initial RT prior\n"
      << "06_projection_final.png: LiDAR points using the reported "
         "visualization RT\n"
      << "07_projection_final_edges.png: LiDAR range edges using the "
         "visualization RT\n"
      << "07a_projection_final_edge_residual.png: visible LiDAR edges colored "
         "by Canny distance (green<=10px, yellow<=30px, red>30px)\n"
      << "07a is a full-resolution nearest-pixel diagnostic; acceptance "
         "metrics use the Core evaluator and quarter-resolution z-buffer\n"
      << "Viewer mesh OBJ/PLY: units m; colorized cloud OBJ/PLY: units mm\n"
      << "debug_summary.csv: per-scene counts and projection coverage\n";
  std::ofstream summary(root / "debug_summary.csv");
  summary << "scene,valid_points,normals,planes,plane_intersection_edges,"
             "plane_boundary_edges,occlusion_edges,persistent_occlusion_edges,"
             "calibration_structural_edges,camera_edges,lidar_edges,"
             "initial_projected,final_projected,final_edges,"
             "final_edge_mean_px,final_edge_p50_px,final_edge_p90_px,"
             "final_edge_over_30_ratio\n";
  struct EdgeResidualSummary {
    std::size_t visible = 0;
    double mean = std::numeric_limits<double>::infinity();
    double p50 = std::numeric_limits<double>::infinity();
    double p90 = std::numeric_limits<double>::infinity();
    double over_30_ratio = 1.0;
  };
  auto projection = [](const auto_calib::CalibrationObservation &observation,
                       const auto_calib::Transform &transform,
                       const auto_calib::CameraModel &camera,
                       const fs::path &path, const std::string &label,
                       bool edges_only,
                       const auto_calib::CalibrationConfig &cfg) {
    cv::Mat out = observation.bgr.clone();
    const auto z_buffer = buildZBuffer(observation.scan, camera, transform);
    std::size_t projected = 0;
    if (!edges_only) {
      for (const auto &point : observation.scan.points) {
        if (!point.valid())
          continue;
        cv::Point pixel;
        double depth = 0.0;
        if (project(point.xyz.cast<double>(), camera, transform, &pixel,
                    &depth) &&
            visibleAt(pixel, depth, camera, z_buffer)) {
          cv::circle(out, pixel, 2, rangeColor(point.range), cv::FILLED,
                     cv::LINE_AA);
          ++projected;
        }
      }
    } else {
      for (const auto &point :
           auto_calib::extractLidarEdgePoints(observation.scan, cfg)) {
        cv::Point pixel;
        double depth = 0.0;
        if (project(point, camera, transform, &pixel, &depth) &&
            visibleAt(pixel, depth, camera, z_buffer)) {
          cv::circle(out, pixel, 4, {0, 255, 0}, 2, cv::LINE_AA);
          ++projected;
        }
      }
    }
    cv::rectangle(out, {0, 0}, {out.cols, 38}, {0, 0, 0}, cv::FILLED);
    cv::putText(out, label + " | projected=" + std::to_string(projected),
                {10, 27}, cv::FONT_HERSHEY_SIMPLEX, 0.68, {255, 255, 255}, 2,
                cv::LINE_AA);
    if (!cv::imwrite(path.string(), out))
      throw std::runtime_error("Cannot write debug projection: " +
                               path.string());
    return projected;
  };
  auto edgeResidualProjection = [](
                                    const auto_calib::CalibrationObservation
                                        &observation,
                                    const auto_calib::Transform &transform,
                                    const auto_calib::CameraModel &camera,
                                    const cv::Mat &distance,
                                    const fs::path &path,
                                    const auto_calib::CalibrationConfig &cfg) {
    cv::Mat out = observation.bgr.clone();
    const auto z_buffer = buildZBuffer(observation.scan, camera, transform);
    std::vector<double> residuals;
    for (const auto &point :
         auto_calib::extractLidarEdgePoints(observation.scan, cfg)) {
      cv::Point pixel;
      double depth = 0.0;
      if (!project(point, camera, transform, &pixel, &depth) ||
          !visibleAt(pixel, depth, camera, z_buffer))
        continue;
      const double residual = distance.at<float>(pixel.y, pixel.x);
      residuals.push_back(residual);
      const cv::Scalar color = residual <= 10.0
                                   ? cv::Scalar(0, 255, 0)
                               : residual <= 30.0
                                   ? cv::Scalar(0, 255, 255)
                                   : cv::Scalar(0, 0, 255);
      cv::circle(out, pixel, 3, color, cv::FILLED, cv::LINE_AA);
    }
    EdgeResidualSummary result;
    result.visible = residuals.size();
    if (!residuals.empty()) {
      result.mean =
          std::accumulate(residuals.begin(), residuals.end(), 0.0) /
          residuals.size();
      result.over_30_ratio =
          static_cast<double>(std::count_if(
              residuals.begin(), residuals.end(),
              [](double residual) { return residual > 30.0; })) /
          residuals.size();
      std::sort(residuals.begin(), residuals.end());
      const auto quantile = [&](double value) {
        const auto index = static_cast<std::size_t>(
            std::floor(value * static_cast<double>(residuals.size() - 1)));
        return residuals[index];
      };
      result.p50 = quantile(0.50);
      result.p90 = quantile(0.90);
    }
    cv::rectangle(out, {0, 0}, {out.cols, 42}, {0, 0, 0}, cv::FILLED);
    std::ostringstream label;
    label << std::fixed << std::setprecision(1) << "edge residual | mean="
          << result.mean << " px | p50=" << result.p50 << " | p90="
          << result.p90 << " | >30=" << 100.0 * result.over_30_ratio << '%';
    cv::putText(out, label.str(), {10, 29}, cv::FONT_HERSHEY_SIMPLEX, 0.68,
                {255, 255, 255}, 2, cv::LINE_AA);
    if (!cv::imwrite(path.string(), out))
      throw std::runtime_error("Cannot write edge residual projection: " +
                               path.string());
    return result;
  };
  std::vector<std::vector<auto_calib::StructuralLineSegment3d>>
      raw_occlusion_segments;
  raw_occlusion_segments.reserve(observations.size());
  for (const auto &observation : observations)
    raw_occlusion_segments.push_back(
        auto_calib::extractLidarOcclusionSegments(observation.scan, config));
  const auto persistent_occlusion_segments =
      auto_calib::retainPersistentLidarOcclusionSegments(
          raw_occlusion_segments, config);
  for (std::size_t i = 0; i < observations.size(); ++i) {
    const auto &observation = observations[i];
    const fs::path scene = root / ("scene_" + std::to_string(i));
    fs::create_directories(scene);
    cv::imwrite((scene / "01_input.png").string(), observation.bgr);

    cv::Mat gray, gray_f, gx, gy, gradient;
    cv::cvtColor(observation.bgr, gray, cv::COLOR_BGR2GRAY);
    gray.convertTo(gray_f, CV_32F, 1.0 / 255.0);
    cv::Sobel(gray_f, gx, CV_32F, 1, 0, 3);
    cv::Sobel(gray_f, gy, CV_32F, 0, 1, 3);
    cv::magnitude(gx, gy, gradient);
    cv::GaussianBlur(gradient, gradient, {5, 5}, 1.5);
    double min_value = 0.0, max_value = 0.0;
    cv::minMaxLoc(gradient, &min_value, &max_value);
    cv::Mat gradient_u8;
    gradient.convertTo(gradient_u8, CV_8U, 255.0 / std::max(1e-9, max_value));
    cv::applyColorMap(gradient_u8, gradient_u8, cv::COLORMAP_JET);
    cv::imwrite((scene / "02_camera_gradient.png").string(), gradient_u8);

    std::size_t camera_edges = 0;
    cv::Mat distance = auto_calib::buildCameraEdgeDistanceTransform(
        observation.bgr, config, &camera_edges);
    cv::Mat distance_u8;
    distance.convertTo(distance_u8, CV_8U, 255.0 / 30.0);
    cv::threshold(distance_u8, distance_u8, 255, 255, cv::THRESH_TRUNC);
    cv::bitwise_not(distance_u8, distance_u8);
    cv::imwrite((scene / "03_camera_edge_distance.png").string(), distance_u8);

    const auto vanishing_directions =
        auto_calib::detectManhattanVanishingDirections(
            observation.bgr, observation.camera, config);
    const Eigen::Vector3d initial_gravity =
        (initial.rotation * config.lidar_gravity_axis).normalized();
    const Eigen::Vector3d final_gravity =
        (visualization_transform.rotation * config.lidar_gravity_axis)
            .normalized();
    auto axialAngleDeg = [](const Eigen::Vector3d &a,
                            const Eigen::Vector3d &b) {
      return std::acos(std::clamp(std::abs(a.normalized().dot(b.normalized())),
                                  0.0, 1.0)) *
             180.0 / kPi;
    };
    std::size_t selected_initial = vanishing_directions.size();
    std::size_t selected_final = vanishing_directions.size();
    double best_initial = std::numeric_limits<double>::infinity();
    double best_final = std::numeric_limits<double>::infinity();
    for (std::size_t direction = 0; direction < vanishing_directions.size();
         ++direction) {
      const double initial_error = axialAngleDeg(
          vanishing_directions[direction].camera_direction, initial_gravity);
      const double final_error = axialAngleDeg(
          vanishing_directions[direction].camera_direction, final_gravity);
      if (initial_error < best_initial) {
        best_initial = initial_error;
        selected_initial = direction;
      }
      if (final_error < best_final) {
        best_final = final_error;
        selected_final = direction;
      }
    }
    std::ofstream vanishing_csv(
        scene / "03a_manhattan_vanishing_directions.csv");
    vanishing_csv
        << "candidate,camera_x,camera_y,camera_z,inliers,mean_residual,"
           "initial_gravity_error_deg,final_gravity_error_deg,"
           "selected_initial,selected_final\n"
        << std::setprecision(12);
    for (std::size_t direction = 0; direction < vanishing_directions.size();
         ++direction) {
      const auto &candidate = vanishing_directions[direction];
      vanishing_csv
          << direction << ',' << candidate.camera_direction.x() << ','
          << candidate.camera_direction.y() << ','
          << candidate.camera_direction.z() << ',' << candidate.inliers << ','
          << candidate.mean_residual << ','
          << axialAngleDeg(candidate.camera_direction, initial_gravity) << ','
          << axialAngleDeg(candidate.camera_direction, final_gravity) << ','
          << (direction == selected_initial ? 1 : 0) << ','
          << (direction == selected_final ? 1 : 0) << '\n';
    }

    const auto &scan = observation.scan;
    const std::size_t count = scan.points.size();
    const auto plane_segmentation =
        auto_calib::segmentLidarPlanes(scan, config);
    const std::size_t normal_count = static_cast<std::size_t>(std::count(
        plane_segmentation.has_normal.begin(),
        plane_segmentation.has_normal.end(), static_cast<unsigned char>(1)));
    std::ofstream ply(scene / "04_lidar_surface_normals.ply");
    if (!ply)
      throw std::runtime_error("Cannot write debug normal PLY");
    ply << "ply\nformat ascii 1.0\ncomment normalized organized-scan "
           "normals\n"
           "element vertex "
        << scan.valid_count
        << "\nproperty float x\nproperty float y\nproperty float z\n"
           "property float nx\nproperty float ny\nproperty float nz\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\n"
           "end_header\n"
        << std::setprecision(9);
    for (std::size_t k = 0; k < count; ++k) {
      const auto &point = scan.points[k];
      if (!point.valid())
        continue;
      const auto normal = plane_segmentation.normals[k];
      const cv::Vec3b color{static_cast<unsigned char>(std::clamp(
                                (normal.z() + 1.0) * 127.5, 0.0, 255.0)),
                            static_cast<unsigned char>(std::clamp(
                                (normal.y() + 1.0) * 127.5, 0.0, 255.0)),
                            static_cast<unsigned char>(std::clamp(
                                (normal.x() + 1.0) * 127.5, 0.0, 255.0))};
      ply << point.xyz.x() << " " << point.xyz.y() << " " << point.xyz.z()
          << " " << normal.x() << " " << normal.y() << " " << normal.z() << " "
          << static_cast<int>(color[2]) << " " << static_cast<int>(color[1])
          << " " << static_cast<int>(color[0]) << "\n";
    }
    std::ofstream labels_ply(scene / "04a_lidar_plane_labels.ply");
    if (!labels_ply)
      throw std::runtime_error("Cannot write debug plane-label PLY");
    labels_ply
        << "ply\nformat ascii 1.0\ncomment fitted plane labels, units meter\n"
        << "element vertex " << scan.valid_count
        << "\nproperty float x\nproperty float y\nproperty float z\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\n"
           "end_header\n"
        << std::setprecision(9);
    for (std::size_t k = 0; k < count; ++k) {
      const auto &point = scan.points[k];
      if (!point.valid())
        continue;
      const int label = plane_segmentation.labels[k];
      const int red = label < 0 ? 90 : (label * 97) % 206 + 50;
      const int green = label < 0 ? 90 : (label * 57) % 206 + 50;
      const int blue = label < 0 ? 90 : (label * 31) % 206 + 50;
      labels_ply << point.xyz.x() << ' ' << point.xyz.y() << ' '
                 << point.xyz.z() << ' ' << red << ' ' << green << ' '
                 << blue << '\n';
    }
    std::ofstream planes_csv(scene / "04a_lidar_planes.csv");
    planes_csv << "plane,nx,ny,nz,offset_m,support_points,rms_error_m\n"
               << std::setprecision(12);
    for (std::size_t plane = 0; plane < plane_segmentation.planes.size();
         ++plane) {
      const auto &value = plane_segmentation.planes[plane];
      planes_csv << plane << ',' << value.normal.x() << ',' << value.normal.y()
                 << ',' << value.normal.z() << ',' << value.offset << ','
                 << value.support_points << ',' << value.rms_error_m << '\n';
    }
    const auto lidar_edges = auto_calib::extractLidarEdgePoints(scan, config);
    std::vector<auto_calib::PlaneIntersectionDiagnostic>
        plane_pair_diagnostics;
    const auto plane_intersections =
        auto_calib::extractLidarPlaneIntersectionSegments(
            scan, plane_segmentation, config, &plane_pair_diagnostics);
    const auto plane_boundaries =
        auto_calib::extractLidarPlaneBoundarySegments(
            scan, plane_segmentation, config);
    std::ofstream plane_pairs_csv(
        scene / "04b_lidar_plane_pair_candidates.csv");
    plane_pairs_csv << "first_plane,second_plane,boundary_contacts,"
                       "boundary_inlier_points,"
                       "plane_angle_deg,boundary_distance_p75_m,"
                       "segment_length_m,accepted,reason\n"
                    << std::setprecision(12);
    for (const auto &candidate : plane_pair_diagnostics)
      plane_pairs_csv << candidate.first_plane << ',' << candidate.second_plane
                      << ',' << candidate.boundary_contacts << ','
                      << candidate.boundary_inlier_points << ','
                      << candidate.plane_angle_deg << ','
                      << candidate.boundary_distance_p75_m << ','
                      << candidate.segment_length_m << ','
                      << (candidate.accepted ? 1 : 0) << ',' << candidate.reason
                      << '\n';
    const auto &occlusion_segments = raw_occlusion_segments[i];
    const auto &persistent_occlusions = persistent_occlusion_segments[i];
    auto calibration_segments = plane_intersections;
    calibration_segments.insert(calibration_segments.end(),
                                plane_boundaries.begin(),
                                plane_boundaries.end());
    calibration_segments.insert(calibration_segments.end(),
                                persistent_occlusions.begin(),
                                persistent_occlusions.end());
    writeSegmentMesh(scene / "04b_lidar_plane_intersection_edges",
                     plane_intersections,
                     "adjacent fitted-plane intersections");
    writeSegmentMesh(scene / "04b1_lidar_plane_boundary_edges",
                     plane_boundaries,
                     "fitted-plane boundaries against unlabelled geometry");
    writeSegmentMesh(scene / "04c_lidar_occlusion_edges", occlusion_segments,
                     "range-discontinuity silhouettes, diagnostic only");
    writeSegmentMesh(scene / "04c1_lidar_persistent_occlusion_edges",
                     persistent_occlusions,
                     "range-discontinuity silhouettes repeated across scans");
    writeSegmentMesh(scene / "04d_lidar_edges_used_for_calibration",
                     calibration_segments,
                     "plane intersections, plane boundaries, and persistent "
                     "occlusions used by calibration");
    const auto initial_projected =
        projection(observation, initial, observation.camera,
                   scene / "05_projection_initial.png",
                   "Initial RT + all LiDAR", false, config);
    const auto final_projected =
        projection(observation, visualization_transform, visualization_camera,
                   scene / "06_projection_final.png",
                   visualization_label + " + all LiDAR", false, config);
    const auto final_edges =
        projection(observation, visualization_transform, visualization_camera,
                   scene / "07_projection_final_edges.png",
                   visualization_label + " + LiDAR edges", true, config);
    const auto edge_residual = edgeResidualProjection(
        observation, visualization_transform, visualization_camera, distance,
        scene / "07a_projection_final_edge_residual.png", config);
    summary << i << ',' << scan.valid_count << ',' << normal_count << ','
            << plane_segmentation.planes.size() << ','
            << plane_intersections.size() << ',' << plane_boundaries.size()
            << ',' << occlusion_segments.size() << ','
            << persistent_occlusions.size() << ','
            << calibration_segments.size() << ',' << camera_edges << ','
            << lidar_edges.size() << ',' << initial_projected << ','
            << final_projected << ',' << final_edges << ','
            << edge_residual.mean << ',' << edge_residual.p50 << ','
            << edge_residual.p90 << ',' << edge_residual.over_30_ratio << '\n';
  }
}

nlohmann::json resultJson(const auto_calib::CalibrationResult &r,
                          double prior_x, double downward_deg,
                          double optical_roll_deg, double focal_scale) {
  return {
      {"prior_translation_x_m", prior_x},
      {"prior_downward_deg", downward_deg},
      {"prior_optical_roll_deg", optical_roll_deg},
      {"focal_scale", focal_scale},
      {"success", r.success},
      {"candidate_available", r.candidate_available},
      {"internal_gate_pass", r.internal_gate_pass},
      {"state", r.state},
      {"reason_code", r.reason_code},
      {"estimated", transformJson(r.estimated_t_camera_lidar)},
      {"diagnostic_candidate", transformJson(r.candidate_t_camera_lidar)},
      {"estimated_intrinsics", cameraJson(r.estimated_camera)},
      {"candidate_intrinsics", cameraJson(r.candidate_camera)},
      {"metrics",
       {{"camera_edge_pixels", r.metrics.camera_edge_pixels},
        {"lidar_edge_points", r.metrics.lidar_edge_points},
        {"lidar_geometry_points", r.metrics.lidar_geometry_points},
        {"lidar_signal_points", r.metrics.lidar_signal_points},
        {"nid_projected_points", r.metrics.nid_projected_points},
        {"visible_edge_points", r.metrics.visible_edge_points},
        {"occluded_edge_points", r.metrics.occluded_edge_points},
        {"edge_active_spatial_cells", r.metrics.edge_active_spatial_cells},
        {"max_coarse_visible_edge_points",
         r.metrics.max_coarse_visible_edge_points},
        {"max_coarse_nid_projected_points",
         r.metrics.max_coarse_nid_projected_points},
        {"max_coarse_edge_active_spatial_cells",
         r.metrics.max_coarse_edge_active_spatial_cells},
        {"edge_coverage_ratio", r.metrics.edge_coverage_ratio},
        {"nid_coverage_ratio", r.metrics.nid_coverage_ratio},
        {"global_edge_coverage_ratio", r.metrics.global_edge_coverage_ratio},
        {"global_nid_coverage_ratio", r.metrics.global_nid_coverage_ratio},
        {"finalist_confidence_margin", r.metrics.finalist_confidence_margin},
        {"tesl_ratio", r.metrics.tesl_ratio},
        {"absolute_support_pass", r.metrics.absolute_support_pass},
        {"edge_spatial_coverage_ratio",
         r.metrics.edge_spatial_coverage_ratio},
        {"coverage_objective", r.metrics.coverage_objective},
        {"camera_structural_lines", r.metrics.camera_structural_lines},
        {"lidar_planes", r.metrics.lidar_planes},
        {"lidar_structural_segments", r.metrics.lidar_structural_segments},
        {"lidar_occlusion_segments", r.metrics.lidar_occlusion_segments},
        {"structural_visible_segments",
         r.metrics.structural_visible_segments},
        {"structural_matched_segments",
         r.metrics.structural_matched_segments},
        {"horizontal_structural_matches",
         r.metrics.horizontal_structural_matches},
        {"vertical_structural_matches",
         r.metrics.vertical_structural_matches},
        {"structural_projected_points",
         r.metrics.structural_projected_points},
        {"total_explained_structural_length",
         r.metrics.total_explained_structural_length},
        {"asymmetric_structural_weight",
         r.metrics.asymmetric_structural_weight},
        {"multi_criteria_confidence_score",
         r.metrics.multi_criteria_confidence_score},
        {"ground_normal_valid", r.metrics.ground_normal_valid},
        {"ground_height_m", r.metrics.ground_height_m},
        {"ground_tilt_deg", r.metrics.ground_tilt_deg},
        {"final_horizontal_structural_objective",
         r.metrics.final_horizontal_structural_objective},
        {"final_vertical_structural_objective",
         r.metrics.final_vertical_structural_objective},
        {"manhattan_vertical_inliers",
         r.metrics.manhattan_vertical_inliers},
        {"manhattan_horizontal_axes", r.metrics.manhattan_horizontal_axes},
        {"initial_manhattan_objective",
         r.metrics.initial_manhattan_objective},
        {"final_manhattan_objective",
         r.metrics.final_manhattan_objective},
        {"final_manhattan_vertical_error_deg",
         r.metrics.final_manhattan_vertical_error_deg},
        {"final_manhattan_horizontal_error_deg",
         r.metrics.final_manhattan_horizontal_error_deg},
        {"multistart_candidates", r.metrics.multistart_candidates},
        {"multistart_objective_margin", r.metrics.multistart_objective_margin},
        {"selected_multistart_yaw_deg", r.metrics.selected_multistart_yaw_deg},
        {"projected_edge_points", r.metrics.projected_edge_points},
        {"projected_ratio", r.metrics.projected_ratio},
        {"initial_mean_edge_distance_px",
         r.metrics.initial_mean_edge_distance_px},
        {"final_mean_edge_distance_px", r.metrics.final_mean_edge_distance_px},
        {"initial_nid", r.metrics.initial_nid},
        {"final_nid", r.metrics.final_nid},
        {"final_range_nid", r.metrics.final_range_nid},
        {"final_normal_nid", r.metrics.final_normal_nid},
        {"final_range_entropy_ratio",
         r.metrics.final_range_entropy_ratio},
        {"final_normal_entropy_ratio",
         r.metrics.final_normal_entropy_ratio},
        {"nid_active_spatial_cells",
         r.metrics.nid_active_spatial_cells},
        {"signal_nmi_projected_points",
         r.metrics.signal_nmi_projected_points},
        {"signal_nmi_active_spatial_cells",
         r.metrics.signal_nmi_active_spatial_cells},
        {"initial_signal_nmi", r.metrics.initial_signal_nmi},
        {"final_signal_nmi", r.metrics.final_signal_nmi},
        {"final_signal_entropy_ratio",
         r.metrics.final_signal_entropy_ratio},
        {"signal_nmi_improvement_ratio",
         r.metrics.signal_nmi_improvement_ratio},
        {"nid_improvement_ratio", r.metrics.nid_improvement_ratio},
        {"initial_composite_objective", r.metrics.initial_composite_objective},
        {"final_composite_objective", r.metrics.final_composite_objective},
        {"objective_improvement_ratio", r.metrics.objective_improvement_ratio},
        {"solver_iterations", r.metrics.solver_iterations},
        {"runtime_ms", r.metrics.runtime_ms}}},
      {"solver_summary", r.solver_summary}};
}
std::size_t structuralDirectionGroups(
    const auto_calib::CalibrationResult &result) {
  return static_cast<std::size_t>(
             result.metrics.horizontal_structural_matches > 0) +
         static_cast<std::size_t>(
             result.metrics.vertical_structural_matches > 0);
}
double cameraDownwardDeg(const auto_calib::Transform &transform) {
  const Eigen::Vector3d forward =
      transform.rotation.transpose() * Eigen::Vector3d::UnitZ();
  return std::asin(std::clamp(forward.y(), -1.0, 1.0)) * 180.0 / kPi;
}
double circularYawDistanceDeg(double first, double second) {
  return std::abs(std::remainder(first - second, 360.0));
}
bool selectionEligible(const auto_calib::CalibrationResult &result,
                       std::size_t minimum_structural_direction_groups,
                       double maximum_camera_downward_deg) {
  return structuralDirectionGroups(result) >=
             minimum_structural_direction_groups &&
         cameraDownwardDeg(result.candidate_t_camera_lidar) <=
             maximum_camera_downward_deg;
}
bool manhattanStageEligible(const auto_calib::CalibrationResult &result,
                            const auto_calib::CalibrationConfig &config,
                            std::size_t observation_count) {
  if (config.manhattan_direction_weight <= 0.0 ||
      config.maximum_manhattan_vertical_error_rad <= 0.0)
    return true;
  if (result.metrics.manhattan_vertical_inliers <
      config.minimum_manhattan_vertical_inliers * observation_count)
    return false;
  return std::isfinite(result.metrics.final_manhattan_vertical_error_deg) &&
         result.metrics.final_manhattan_vertical_error_deg <=
             config.maximum_manhattan_vertical_error_rad * 180.0 / kPi;
}
void applySelectionGates(auto_calib::CalibrationResult *result,
                         std::size_t minimum_structural_direction_groups,
                         double maximum_camera_downward_deg) {
  if (!result->success)
    return;
  if (cameraDownwardDeg(result->candidate_t_camera_lidar) >
      maximum_camera_downward_deg) {
    result->success = false;
    result->reason_code = "CAMERA_DIRECTION_INFEASIBLE";
  } else if (structuralDirectionGroups(*result) <
             minimum_structural_direction_groups) {
    result->success = false;
    result->reason_code = "STRUCTURAL_DIRECTION_INSUFFICIENT";
  }
}
std::size_t writeOrientationScoreMap(
    const std::vector<auto_calib::CalibrationResult> &results,
    const std::vector<double> &downward_degrees,
    const std::vector<double> &optical_roll_degrees,
    const std::vector<double> &focal_scales, const fs::path &path,
    double *best_down_deg, double *best_yaw_deg, double *best_objective) {
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Cannot write orientation score map");
  out << "down_deg,optical_roll_deg,focal_scale,yaw_deg,raw_objective,"
         "edge_objective,nid_objective,signal_nmi_objective,"
         "structural_line_objective,horizontal_structural_objective,"
         "vertical_structural_objective,manhattan_objective,"
         "direction_prior_objective,"
         "edge_in_frame_points,visible_edge_points,"
         "occluded_edge_points,nid_projected_points,"
         "signal_nmi_projected_points,"
         "horizontal_structural_segments,vertical_structural_segments,"
         "edge_active_spatial_cells,structural_visible_segments,"
         "edge_coverage_ratio,nid_coverage_ratio,"
         "edge_spatial_coverage_ratio,coverage_objective,"
         "overlap_valid\n"
      << std::setprecision(12);
  std::size_t count = 0;
  *best_objective = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < results.size(); ++i)
    for (const auto &score : results[i].coarse_orientation_scores) {
      out << downward_degrees[i] << ',' << optical_roll_degrees[i] << ','
          << focal_scales[i] << ',' << score.yaw_offset_deg << ','
          << score.raw_objective << ',' << score.edge_objective << ','
          << score.nid_objective << ',' << score.signal_nmi_objective << ','
          << score.structural_line_objective
          << ',' << score.horizontal_structural_objective << ','
          << score.vertical_structural_objective << ','
          << score.manhattan_objective << ','
          << score.direction_prior_objective << ','
          << score.edge_in_frame_points << ','
          << score.visible_edge_points << ',' << score.occluded_edge_points
          << ',' << score.nid_projected_points << ','
          << score.signal_nmi_projected_points << ','
          << score.horizontal_structural_segments << ','
          << score.vertical_structural_segments << ','
          << score.edge_active_spatial_cells << ','
          << score.structural_visible_segments << ','
          << score.edge_coverage_ratio << ','
          << score.nid_coverage_ratio << ','
          << score.edge_spatial_coverage_ratio << ','
          << score.coverage_objective << ','
          << (score.overlap_valid ? 1 : 0)
          << '\n';
      ++count;
      if (score.raw_objective < *best_objective) {
        *best_objective = score.raw_objective;
        *best_down_deg = downward_degrees[i];
        *best_yaw_deg = score.yaw_offset_deg;
      }
    }
  return count;
}
struct BasinSelection {
  std::size_t result_index = 0;
  std::size_t column = 0;
  double yaw_deg = 0.0;
  double corrected_score = std::numeric_limits<double>::infinity();
  std::size_t basin_count = 0;
};
BasinSelection writeCorrectedScoreMap(
    const std::vector<auto_calib::CalibrationResult> &results,
    const std::vector<double> &downward_degrees, const fs::path &path,
    std::size_t minimum_structural_direction_groups, bool circular_yaw,
    double alpha = 0.8, double sigma_yaw_deg = 5.0,
    double sigma_down_deg = 5.0,
    std::vector<BasinSelection> *distinct_basins = nullptr) {
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Cannot write corrected orientation score map");
  out << "down_deg,yaw_deg,raw_objective,neighbor_mean,corrected_score\n"
      << std::setprecision(12);
  BasinSelection best;
  std::vector<std::vector<double>> corrected(results.size());
  for (std::size_t r = 0; r < results.size(); ++r) {
    const auto &scores = results[r].coarse_orientation_scores;
    corrected[r].resize(scores.size());
    for (std::size_t c = 0; c < scores.size(); ++c) {
      const auto structural_groups =
          static_cast<std::size_t>(
              scores[c].horizontal_structural_segments > 0) +
          static_cast<std::size_t>(
              scores[c].vertical_structural_segments > 0);
      const bool ground_ok = scores[c].ground_normal_valid;
      const bool tesl_ok = (scores[c].total_explained_structural_length >= 0.0);
      const double raw = structural_groups >=
                                     minimum_structural_direction_groups &&
                                 scores[c].overlap_valid &&
                                 ground_ok &&
                                 tesl_ok
                             ? scores[c].raw_objective
                             : std::numeric_limits<double>::infinity();
      double weighted_sum = 0.0;
      double weight_sum = 0.0;
      for (int dr = -1; dr <= 1; ++dr) {
        const auto rr = static_cast<int>(r) + dr;
        if (rr < 0 || rr >= static_cast<int>(results.size()))
          continue;
        for (int dc = -1; dc <= 1; ++dc) {
          if (dr == 0 && dc == 0)
            continue;
          const auto &row =
              results[static_cast<std::size_t>(rr)].coarse_orientation_scores;
          if (row.empty())
            continue;
          int cc = static_cast<int>(c) + dc;
          if (circular_yaw)
            cc = (cc + static_cast<int>(row.size())) %
                 static_cast<int>(row.size());
          else if (cc < 0 || cc >= static_cast<int>(row.size()))
            continue;
          const auto neighbor_structural_groups =
              static_cast<std::size_t>(
                  row[static_cast<std::size_t>(cc)]
                          .horizontal_structural_segments > 0) +
              static_cast<std::size_t>(
                  row[static_cast<std::size_t>(cc)]
                          .vertical_structural_segments > 0);
          if (neighbor_structural_groups >=
                  minimum_structural_direction_groups &&
              row[static_cast<std::size_t>(cc)].overlap_valid &&
              row[static_cast<std::size_t>(cc)].ground_normal_valid &&
              std::isfinite(row[static_cast<std::size_t>(cc)]
                                .raw_objective)) {
            const double dyaw =
                std::abs(scores[c].yaw_offset_deg -
                         row[static_cast<std::size_t>(cc)].yaw_offset_deg);
            const double wrapped_yaw = std::min(dyaw, 360.0 - dyaw);
            const double ddown =
                std::abs(downward_degrees[r] -
                         downward_degrees[static_cast<std::size_t>(rr)]);
            const double weight =
                std::exp(-0.5 * (std::pow(wrapped_yaw / sigma_yaw_deg, 2.0) +
                                 std::pow(ddown / sigma_down_deg, 2.0)));
            weighted_sum +=
                weight * row[static_cast<std::size_t>(cc)].raw_objective;
            weight_sum += weight;
          }
        }
      }
      const double neighbor = weight_sum > 0.0 ? weighted_sum / weight_sum
                                               : raw;
      corrected[r][c] = alpha * raw + (1.0 - alpha) * neighbor;
      out << downward_degrees[r] << ',' << scores[c].yaw_offset_deg << ','
          << scores[c].raw_objective << ',' << neighbor << ','
          << corrected[r][c] << '\n';
      if (corrected[r][c] < best.corrected_score) {
        best = {r, c, scores[c].yaw_offset_deg, corrected[r][c], 0};
      }
    }
  }
  if (!std::isfinite(best.corrected_score)) {
    if (distinct_basins)
      distinct_basins->push_back(best);
    return best;
  }

  // Identify all distinct local minima in the smoothed score map
  std::vector<BasinSelection> all_basins;
  for (std::size_t r = 0; r < corrected.size(); ++r) {
    for (std::size_t c = 0; c < corrected[r].size(); ++c) {
      const double val = corrected[r][c];
      if (!std::isfinite(val))
        continue;
      bool is_local_min = true;
      for (int dr = -1; dr <= 1 && is_local_min; ++dr) {
        const int rr = static_cast<int>(r) + dr;
        if (rr < 0 || rr >= static_cast<int>(corrected.size()))
          continue;
        for (int dc = -1; dc <= 1 && is_local_min; ++dc) {
          if (dr == 0 && dc == 0)
            continue;
          int cc = static_cast<int>(c) + dc;
          if (circular_yaw)
            cc = (cc + static_cast<int>(corrected[rr].size())) %
                 static_cast<int>(corrected[rr].size());
          else if (cc < 0 || cc >= static_cast<int>(corrected[rr].size()))
            continue;
          if (std::isfinite(corrected[rr][cc]) && corrected[rr][cc] < val)
            is_local_min = false;
        }
      }
      if (is_local_min) {
        const auto yaw =
            results[r].coarse_orientation_scores[c].yaw_offset_deg;
        all_basins.push_back({r, c, yaw, val, 1});
      }
    }
  }

  if (all_basins.empty())
    all_basins.push_back(best);
  std::sort(all_basins.begin(), all_basins.end(),
            [](const BasinSelection &a, const BasinSelection &b) {
              return a.corrected_score < b.corrected_score;
            });

  // Calculate basin extent via flood-fill for candidates
  for (auto &b_cand : all_basins) {
    const double threshold = b_cand.corrected_score + 0.02;
    std::vector<std::vector<bool>> visited(corrected.size());
    for (std::size_t r = 0; r < corrected.size(); ++r)
      visited[r].assign(corrected[r].size(), false);
    std::vector<std::pair<std::size_t, std::size_t>> queue{
        {b_cand.result_index, b_cand.column}};
    visited[b_cand.result_index][b_cand.column] = true;
    std::size_t count = 0;
    while (!queue.empty()) {
      const auto [r, c] = queue.back();
      queue.pop_back();
      ++count;
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          if (dr == 0 && dc == 0)
            continue;
          const int rr = static_cast<int>(r) + dr;
          if (rr < 0 || rr >= static_cast<int>(corrected.size()))
            continue;
          const auto &row = corrected[static_cast<std::size_t>(rr)];
          if (row.empty())
            continue;
          int cc = static_cast<int>(c) + dc;
          if (circular_yaw)
            cc = (cc + static_cast<int>(row.size())) %
                 static_cast<int>(row.size());
          else if (cc < 0 || cc >= static_cast<int>(row.size()))
            continue;
          const auto column = static_cast<std::size_t>(cc);
          if (!visited[static_cast<std::size_t>(rr)][column] &&
              row[column] <= threshold) {
            visited[static_cast<std::size_t>(rr)][column] = true;
            queue.push_back({static_cast<std::size_t>(rr), column});
          }
        }
      }
    }
    b_cand.basin_count = count;
  }

  // Deduplicate within layer by >= 30 deg yaw
  std::vector<BasinSelection> filtered_basins;
  for (const auto &b_cand : all_basins) {
    if (std::all_of(filtered_basins.begin(), filtered_basins.end(),
                    [&](const BasinSelection &acc) {
                      return circularYawDistanceDeg(acc.yaw_deg,
                                                    b_cand.yaw_deg) >= 30.0;
                    })) {
      filtered_basins.push_back(b_cand);
    }
  }

  if (filtered_basins.empty())
    filtered_basins.push_back(best);

  if (distinct_basins)
    *distinct_basins = filtered_basins;

  best = filtered_basins.front();
  return best;
}
nlohmann::json writeSignalNmiConformance(
    const std::vector<auto_calib::CalibrationObservation> &observations,
    const auto_calib::Transform &reference,
    const auto_calib::CalibrationConfig &config, const fs::path &path) {
  struct Perturbation {
    std::string axis;
    double degrees = 0.0;
    auto_calib::Transform transform;
  };
  std::vector<Perturbation> cases = {{"reference", 0.0, reference}};
  const std::vector<std::pair<std::string, Eigen::Vector3d>> axes = {
      {"lidar_x_roll", Eigen::Vector3d::UnitX()},
      {"lidar_y_yaw", Eigen::Vector3d::UnitY()},
      {"lidar_z_optical_roll", Eigen::Vector3d::UnitZ()}};
  const std::vector<double> perturbations = {-10.0, -5.0, -3.0, -1.0,
                                              1.0,   3.0,  5.0,  10.0};
  for (const auto &[name, axis] : axes)
    for (const double degrees : perturbations) {
      auto transform = reference;
      transform.rotation =
          reference.rotation *
          Eigen::AngleAxisd(radians(degrees), axis).toRotationMatrix();
      cases.push_back({name, degrees, transform});
    }
  std::vector<auto_calib::Transform> transforms;
  transforms.reserve(cases.size());
  for (const auto &candidate : cases)
    transforms.push_back(candidate.transform);
  const auto scores =
      auto_calib::evaluateSignalNmiPoses(observations, transforms, config);
  std::ofstream csv(path);
  if (!csv)
    throw std::runtime_error("Cannot write signal NMI conformance CSV");
  csv << "axis,perturbation_deg,score,delta_from_reference,entropy_ratio,"
         "projected_points,active_spatial_cells,valid,worse_than_reference\n"
      << std::setprecision(12);
  const double reference_score = scores.front().score;
  std::vector<double> valid_perturbed_scores;
  std::size_t worse = 0;
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const bool worse_than_reference =
        i > 0 && scores[i].valid && scores[i].score > reference_score;
    if (i > 0 && scores[i].valid) {
      valid_perturbed_scores.push_back(scores[i].score);
      worse += static_cast<std::size_t>(worse_than_reference);
    }
    csv << cases[i].axis << ',' << cases[i].degrees << ',' << scores[i].score
        << ',' << scores[i].score - reference_score << ','
        << scores[i].entropy_ratio << ',' << scores[i].projected_points << ','
        << scores[i].active_spatial_cells << ',' << (scores[i].valid ? 1 : 0)
        << ',' << (worse_than_reference ? 1 : 0) << '\n';
  }
  std::sort(valid_perturbed_scores.begin(), valid_perturbed_scores.end());
  const double median_perturbed =
      valid_perturbed_scores.empty()
          ? std::numeric_limits<double>::quiet_NaN()
          : valid_perturbed_scores[valid_perturbed_scores.size() / 2];
  const double worse_ratio =
      valid_perturbed_scores.empty()
          ? 0.0
          : static_cast<double>(worse) / valid_perturbed_scores.size();
  const double median_margin = median_perturbed - reference_score;
  const bool pass = scores.front().valid &&
                    valid_perturbed_scores.size() >= 18 &&
                    worse_ratio >= 0.70 && median_margin >= 0.002;
  return {{"status",
           !scores.front().valid || valid_perturbed_scores.size() < 18
               ? "INSUFFICIENT"
               : (pass ? "PASS" : "FAIL")},
          {"reference_role", "manual_RT_diagnostic_not_conformance_truth"},
          {"reference_score", reference_score},
          {"reference_entropy_ratio", scores.front().entropy_ratio},
          {"valid_perturbations", valid_perturbed_scores.size()},
          {"worse_than_reference_ratio", worse_ratio},
          {"median_perturbed_score", median_perturbed},
          {"median_margin", median_margin},
          {"activation_recommended", pass},
          {"csv", path.string()}};
}
std::size_t sceneDirectionGroups(const auto_calib::PoseSceneMetrics &metrics) {
  return static_cast<std::size_t>(metrics.horizontal_structural_matches > 0) +
         static_cast<std::size_t>(metrics.vertical_structural_matches > 0);
}
std::vector<std::string> sceneValidationFailures(
    const auto_calib::PoseSceneMetrics &metrics,
    const auto_calib::CalibrationConfig &config,
    std::size_t minimum_direction_groups) {
  std::vector<std::string> failures;
  if (metrics.visible_edge_points < config.minimum_lidar_edge_points)
    failures.push_back("EDGE_VISIBLE_INSUFFICIENT");
  if (metrics.projected_ratio < config.minimum_projected_ratio)
    failures.push_back("EDGE_OVERLAP_INSUFFICIENT");
  if (!std::isfinite(metrics.mean_edge_distance_px) ||
      metrics.mean_edge_distance_px > config.maximum_mean_edge_distance_px)
    failures.push_back("EDGE_ALIGNMENT_POOR");
  if (config.normalized_information_distance_weight > 0.0 &&
      metrics.nid_projected_points < config.minimum_nid_projected_points)
    failures.push_back("NID_OVERLAP_INSUFFICIENT");
  if (config.normalized_information_distance_weight > 0.0 &&
      metrics.nid_active_spatial_cells <
          config.minimum_nid_active_spatial_cells)
    failures.push_back("NID_SPATIAL_ENTROPY_INSUFFICIENT");
  if (metrics.structural_matched_segments <
      config.minimum_projected_structural_segments)
    failures.push_back("STRUCTURAL_OVERLAP_INSUFFICIENT");
  if (sceneDirectionGroups(metrics) < minimum_direction_groups)
    failures.push_back("STRUCTURAL_DIRECTION_INSUFFICIENT");
  if (config.manhattan_direction_weight > 0.0 &&
      metrics.manhattan_vertical_inliers <
          config.minimum_manhattan_vertical_inliers)
    failures.push_back("MANHATTAN_VERTICAL_SUPPORT_INSUFFICIENT");
  if (config.manhattan_direction_weight > 0.0 &&
      config.maximum_manhattan_vertical_error_rad > 0.0 &&
      (metrics.manhattan_vertical_error_deg < 0.0 ||
       metrics.manhattan_vertical_error_deg >
           config.maximum_manhattan_vertical_error_rad * 180.0 / kPi))
    failures.push_back("MANHATTAN_VERTICAL_ALIGNMENT_POOR");
  if (config.signal_nmi_weight > 0.0 &&
      metrics.signal_projected_points <
          config.minimum_signal_nmi_projected_points)
    failures.push_back("SIGNAL_NMI_OVERLAP_INSUFFICIENT");
  if (config.signal_nmi_weight > 0.0 &&
      metrics.signal_active_spatial_cells <
          config.minimum_signal_nmi_active_spatial_cells)
    failures.push_back("SIGNAL_NMI_ENTROPY_INSUFFICIENT");
  if (config.enable_ground_plane_constraint && !metrics.ground_normal_valid)
    failures.push_back("GEOMETRY_GROUND_INCONSISTENT");
  return failures;
}
nlohmann::json writePoseSceneValidation(
    const std::vector<auto_calib::PoseSceneMetrics> &metrics,
    const auto_calib::CalibrationConfig &config,
    std::size_t minimum_direction_groups, const fs::path &path) {
  std::ofstream csv(path);
  if (!csv)
    throw std::runtime_error("Cannot write pose-scene validation CSV");
  csv << "scene,pass,failures,visible_edges,aligned_edges,projected_ratio,"
         "edge_active_cells,mean_edge_px,edge_objective,nid_projected,nid_cells,"
         "geometry_nid,geometry_nid_objective,range_nid,normal_nid,"
         "structural_visible,structural_matched,horizontal_matches,"
         "vertical_matches,total_explained_structural_length,asymmetric_structural_weight,"
         "structural_objective,structural_score_weight,"
         "ground_valid,ground_height_m,ground_tilt_deg,"
         "manhattan_inliers,manhattan_horizontal_axes,"
         "manhattan_vertical_error_deg,manhattan_objective,signal_projected,"
         "signal_cells,signal_nmi,signal_nmi_objective\n"
      << std::setprecision(12);
  std::size_t passed = 0;
  nlohmann::json scenes = nlohmann::json::array();
  for (std::size_t i = 0; i < metrics.size(); ++i) {
    const auto failures =
        sceneValidationFailures(metrics[i], config, minimum_direction_groups);
    passed += static_cast<std::size_t>(failures.empty());
    std::ostringstream failure_text;
    for (std::size_t j = 0; j < failures.size(); ++j) {
      if (j > 0)
        failure_text << '|';
      failure_text << failures[j];
    }
    const auto &m = metrics[i];
    csv << i << ',' << (failures.empty() ? 1 : 0) << ','
        << failure_text.str() << ',' << m.visible_edge_points << ','
        << m.aligned_edge_points << ',' << m.projected_ratio << ','
        << m.edge_active_spatial_cells << ','
        << m.mean_edge_distance_px << ',' << m.edge_objective << ','
        << m.nid_projected_points << ','
        << m.nid_active_spatial_cells << ',' << m.geometry_nid << ','
        << m.geometry_nid_objective << ',' << m.range_nid << ','
        << m.normal_nid << ','
        << m.structural_visible_segments << ','
        << m.structural_matched_segments << ','
        << m.horizontal_structural_matches << ','
        << m.vertical_structural_matches << ','
        << m.total_explained_structural_length << ','
        << m.asymmetric_structural_weight << ',' << m.structural_objective
        << ',' << m.structural_score_weight << ','
        << (m.ground_normal_valid ? 1 : 0) << ','
        << m.ground_height_m << ','
        << m.ground_tilt_deg << ','
        << m.manhattan_vertical_inliers << ',' << m.manhattan_horizontal_axes
        << ',' << m.manhattan_vertical_error_deg << ','
        << m.manhattan_objective << ',' << m.signal_projected_points << ','
        << m.signal_active_spatial_cells << ',' << m.signal_nmi << ','
        << m.signal_nmi_objective << '\n';
    scenes.push_back({{"scene", i},
                      {"pass", failures.empty()},
                      {"failures", failures}});
  }
  return {{"scene_count", metrics.size()},
          {"passed_scenes", passed},
          {"pass_ratio",
           metrics.empty() ? 1.0
                           : static_cast<double>(passed) / metrics.size()},
          {"minimum_direction_groups", minimum_direction_groups},
          {"csv", path.string()},
          {"scenes", scenes}};
}
void usage() {
  std::cout
      << "run_real_calibration --input-dir PATH --output PATH "
         "[--pair-start 0 --pair-count N] "
         "[--camera-channel N] [--ldc-enabled true|false|unknown] "
         "[--zoom-focus-locked true|false|unknown] "
         "[--manual-intrinsic-json PATH] "
         "[--allow-manufacturer-fov-diagnostic true] "
         "[--manual-reference-json PATH] "
         "[--validation-pose-json PATH --validation-label LABEL] "
         "[--image-distortion-state raw|rectified|unknown] "
         "[--allow-intrinsic-refinement true|false] "
         "[--enable-experimental-joint-intrinsic true|false] "
         "[--reference-rt-perturbation-only true|false] "
         "[--search-strategy staged|legacy] "
         "[--baseline-m 0.28] [--minimum-range-m 0.30] "
         "[--camera-center-x-m X --camera-center-y-m Y "
         "--camera-center-z-m Z] [--prior-roll-deg DEG | "
         "--down-min-deg 0 --down-max-deg 90 --down-step-deg 15] "
         "[--optical-roll-min-deg -15 --optical-roll-max-deg 15 "
         "--optical-roll-step-deg 5] "
         "[--yaw-step-deg 15] [--yaw-min-deg -180 --yaw-max-deg -160] "
         "[--minimum-relative-nid-coverage 0.5 "
         "--minimum-relative-spatial-coverage 0.5 "
         "--coverage-penalty-weight 0.25] "
         "[--focal-scale 1.0 | --focal-scale-min 0.9 "
         "--focal-scale-max 1.1 --focal-scale-step 0.1] "
         "[--principal-y-offset-px 0] "
         "[--nid-weight 0.55 --signal-nmi-weight 0.0 "
         "--edge-weight 0.25 --line-weight 0.20 --manhattan-weight 0.15] "
         "[--minimum-structural-direction-groups 2] "
         "[--holdout-count 1 --minimum-scene-pass-ratio 0.8 "
         "--minimum-scene-direction-groups 1] "
         "[--maximum-camera-downward-deg 75] "
         "[--lidar-edge-local-contrast-ratio 2.0] "
         "[--plane-normal-deg 15 --plane-min-points 80 "
         "--plane-max-rms-m 0.03 --plane-neighbor-radius-cells 3 "
         "--plane-boundary-distance-m 0.10] "
         "[--camera-outward-facing true|false "
         "--direction-prior-weight 0.35] "
         "[--expected-forward-x X --expected-forward-y Y "
         "--expected-forward-z Z] "
         "[--expected-down-x X --expected-down-y Y --expected-down-z Z] "
         "[--prior-pitch-deg 0] [--prior-yaw-deg 0] [--maximum-range-m 20] "
         "[--minimum-signal 1000] [--legacy-range-offset-m M] "
         "[--debug-output PATH]\n";
}
} // namespace

int main(int argc, char **argv) {
  try {
    Args args = parseArgs(argc, argv);
    if (args.count("--tilt-zero-override") ||
        args.count("--coordinate-variant"))
      throw std::invalid_argument(
          "Legacy tilt overrides were removed; measurements[].tilt_rad "
          "and frame convention are authoritative");
    if (args.count("--help")) {
      usage();
      return 0;
    }
    if (!args.count("--input-dir") || !args.count("--output")) {
      usage();
      return 2;
    }
    const fs::path input_dir = args.at("--input-dir");
    const fs::path output_dir = args.at("--output");
    const std::string search_strategy =
        textValue(args, "--search-strategy", "staged");
    if (search_strategy != "staged" && search_strategy != "legacy")
      throw std::invalid_argument(
          "--search-strategy must be staged or legacy");
    const bool staged_search = search_strategy == "staged";
    const int camera_channel =
        static_cast<int>(value(args, "--camera-channel", 0.0));
    auto input_pairs = inputFilePairs(input_dir, camera_channel);
    const double pair_start_value = value(args, "--pair-start", 0.0);
    const double pair_count_value =
        value(args, "--pair-count", static_cast<double>(input_pairs.size()));
    if (pair_start_value < 0.0 || pair_count_value < 1.0 ||
        std::floor(pair_start_value) != pair_start_value ||
        std::floor(pair_count_value) != pair_count_value ||
        pair_start_value + pair_count_value > input_pairs.size())
      throw std::runtime_error(
          "--pair-start/--pair-count select an invalid pair range");
    const auto pair_start = static_cast<std::size_t>(pair_start_value);
    const auto pair_count = static_cast<std::size_t>(pair_count_value);
    input_pairs = std::vector<InputFilePair>(
        input_pairs.begin() + pair_start,
        input_pairs.begin() + pair_start + pair_count);
    std::vector<fs::path> images;
    std::vector<fs::path> scans;
    images.reserve(input_pairs.size());
    scans.reserve(input_pairs.size());
    for (const auto &pair : input_pairs) {
      images.push_back(pair.image);
      scans.push_back(pair.scan);
    }
    const cv::Mat first_image =
        cv::imread(images.front().string(), cv::IMREAD_COLOR);
    if (first_image.empty())
      throw std::runtime_error("Cannot read image: " + images.front().string());
    const std::string ldc_enabled = textValue(args, "--ldc-enabled", "unknown");
    if (ldc_enabled != "true" && ldc_enabled != "false" &&
        ldc_enabled != "unknown")
      throw std::invalid_argument(
          "--ldc-enabled must be true, false, or unknown");
    const bool manual_intrinsic = args.count("--manual-intrinsic-json");
    const bool allow_manufacturer_fov_diagnostic =
        textValue(args, "--allow-manufacturer-fov-diagnostic", "false") ==
        "true";
    if (!manual_intrinsic && !allow_manufacturer_fov_diagnostic)
      throw std::invalid_argument(
          "Product calibration requires --manual-intrinsic-json; use "
          "--allow-manufacturer-fov-diagnostic true only for diagnostics");
    const bool manual_reference = args.count("--manual-reference-json");
    const fs::path manual_reference_path =
        manual_reference ? fs::path(args.at("--manual-reference-json"))
                         : fs::path();
    const auto manual_reference_transform =
        manual_reference ? loadTransformJson(manual_reference_path)
                         : auto_calib::Transform{};
    const fs::path manual_intrinsic_path =
        manual_intrinsic ? fs::path(args.at("--manual-intrinsic-json")) : fs::path();
    auto camera = manual_intrinsic
                      ? cameraFromManualIntrinsic(manual_intrinsic_path,
                                                  first_image.size())
                      : cameraFromManufacturerFov(first_image.size());
    const std::string default_distortion_state =
        manual_intrinsic
            ? (ldc_enabled == "true"
                   ? "rectified"
                   : ldc_enabled == "false" ? "raw" : "unknown")
                         : "unknown";
    const std::string image_distortion_state =
        textValue(args, "--image-distortion-state", default_distortion_state);
    if (image_distortion_state != "raw" &&
        image_distortion_state != "rectified" &&
        image_distortion_state != "unknown")
      throw std::invalid_argument(
          "--image-distortion-state must be raw, rectified, or unknown");
    if (manual_intrinsic && ldc_enabled == "true" &&
        image_distortion_state == "raw")
      throw std::invalid_argument(
          "LDC=true requires rectified/unknown image distortion state");
    if (ldc_enabled == "false" && !manual_intrinsic)
      throw std::invalid_argument(
          "LDC=false requires --manual-intrinsic-json for raw image "
          "undistortion");
    if (ldc_enabled == "false" && image_distortion_state != "raw")
      throw std::invalid_argument(
          "LDC=false requires --image-distortion-state raw");
    if (ldc_enabled == "false" && camera.distortion.empty())
      throw std::invalid_argument(
          "LDC=false requires manual distortion coefficients");
    camera.image_distortion_state = image_distortion_state;
    const double principal_y_offset_px =
        value(args, "--principal-y-offset-px", 0.0);
    if (manual_intrinsic &&
        (args.count("--focal-scale") || args.count("--focal-scale-min") ||
         args.count("--focal-scale-max") || args.count("--focal-scale-step") ||
         std::abs(principal_y_offset_px) > 1e-9))
      throw std::invalid_argument(
          "Manual intrinsic is fixed; do not combine it with focal/cy search");
    const double focal_scale =
        manual_intrinsic ? 1.0 : value(args, "--focal-scale", 1.0);
    const auto focal_scale_candidates =
        manual_intrinsic
            ? std::vector<double>{1.0}
            : numericRange(args, "--focal-scale-min", "--focal-scale-max",
                           "--focal-scale-step", focal_scale);
    if (focal_scale < 0.80 || focal_scale > 1.20 ||
        std::any_of(focal_scale_candidates.begin(),
                    focal_scale_candidates.end(),
                    [](double scale) { return scale < 0.80 || scale > 1.20; }) ||
        std::abs(principal_y_offset_px) > 0.10 * first_image.rows)
      throw std::invalid_argument(
          "K profile must stay within focal scale [0.8,1.2] and cy +/-10%");
    if (!manual_intrinsic) {
      camera.camera.k(0, 0) *= focal_scale;
      camera.camera.k(1, 1) *= focal_scale;
      camera.camera.k(1, 2) += principal_y_offset_px;
      camera.profile_id += "-focal-scale-" + std::to_string(focal_scale) +
                           "-cy-offset-" +
                           std::to_string(principal_y_offset_px);
    }
    const std::string zoom_focus_locked =
        textValue(args, "--zoom-focus-locked", "unknown");
    const std::string allow_intrinsic_refinement_text =
        textValue(args, "--allow-intrinsic-refinement", "false");
    if (allow_intrinsic_refinement_text != "true" &&
        allow_intrinsic_refinement_text != "false")
      throw std::invalid_argument(
          "--allow-intrinsic-refinement must be true or false");
    const bool allow_intrinsic_refinement =
        allow_intrinsic_refinement_text == "true";
    const std::string experimental_joint_intrinsic_text = textValue(
        args, "--enable-experimental-joint-intrinsic", "false");
    if (experimental_joint_intrinsic_text != "true" &&
        experimental_joint_intrinsic_text != "false")
      throw std::invalid_argument(
          "--enable-experimental-joint-intrinsic must be true or false");
    const bool enable_experimental_joint_intrinsic =
        experimental_joint_intrinsic_text == "true";
    if (allow_intrinsic_refinement &&
        !enable_experimental_joint_intrinsic)
      throw std::invalid_argument(
          "Joint K+RT refinement is experimental; explicitly enable "
          "--enable-experimental-joint-intrinsic true");
    const std::string reference_perturbation_text = textValue(
        args, "--reference-rt-perturbation-only", "false");
    if (reference_perturbation_text != "true" &&
        reference_perturbation_text != "false")
      throw std::invalid_argument(
          "--reference-rt-perturbation-only must be true or false");
    const bool reference_perturbation_only =
        reference_perturbation_text == "true";
    if (reference_perturbation_only && !manual_reference)
      throw std::invalid_argument(
          "Reference RT perturbation mode requires --manual-reference-json");
    const double minimum_range = value(args, "--minimum-range-m", 0.30);
    const double maximum_range = value(args, "--maximum-range-m", 20.0);
    const double minimum_signal = value(args, "--minimum-signal", 1000.0);
    const double legacy_range_offset =
        value(args, "--legacy-range-offset-m",
              std::numeric_limits<double>::quiet_NaN());
    fs::create_directories(output_dir / "prepared");
    std::vector<auto_calib::CalibrationObservation> observations;
    nlohmann::json pairs = nlohmann::json::array();
    for (std::size_t i = 0; i < images.size(); ++i) {
      auto_calib::CameraModel scene_camera;
      cv::Mat image = loadImage(images[i], &camera, &scene_camera);
      LoadedScan loaded = loadScan(scans[i], minimum_range, maximum_range,
                                   minimum_signal, legacy_range_offset);
      fs::path prepared_path =
          output_dir / "prepared" / ("scene_" + std::to_string(i) + ".png");
      if (!cv::imwrite(prepared_path.string(), image))
        throw std::runtime_error("Cannot write prepared image");
      pairs.push_back({{"scene_index", i},
                       {"image", images[i].filename().string()},
                       {"scan", scans[i].filename().string()},
                       {"prepared_image", prepared_path.string()},
                       {"image_distortion_state", image_distortion_state},
                       {"distortion_applied",
                        image_distortion_state == "raw" &&
                            !camera.distortion.empty()},
                       {"preprocessing", loaded.statistics}});
      observations.push_back(
          {std::move(image), scene_camera, std::move(loaded.scan)});
    }
    auto_calib::CalibrationConfig config;
    config.enable_ceres_refinement = !staged_search;
    config.enable_experimental_joint_intrinsics =
        enable_experimental_joint_intrinsic;
    config.minimum_lidar_edge_points = 30;
    config.lidar_edge_minimum_local_contrast_ratio =
        value(args, "--lidar-edge-local-contrast-ratio", 2.0);
    config.minimum_nid_projected_points = 100;
    config.maximum_nid_points = 5000;
    config.nid_histogram_bins = 16;
    config.nid_spatial_rows = 2;
    config.nid_spatial_columns = 2;
    config.minimum_nid_tile_points = 25;
    config.minimum_nid_active_spatial_cells = 2;
    config.minimum_nid_feature_entropy_ratio = 0.08;
    config.normalized_information_distance_weight =
        value(args, "--nid-weight", 0.55);
    config.signal_nmi_weight = value(args, "--signal-nmi-weight", 0.0);
    config.edge_alignment_weight = value(args, "--edge-weight", 0.25);
    config.structural_line_weight = value(args, "--line-weight", 0.20);
    config.structural_normal_weight =
        value(args, "--line-normal-weight", 0.15);
    config.structural_line_sigma_px =
        value(args, "--line-sigma-px", 10.0);
    config.enable_normal_gated_line_matching =
        textValue(args, "--enable-normal-gated-line-matching", "true") == "true";
    config.manhattan_direction_weight =
        value(args, "--manhattan-weight", 0.15);
    const double minimum_manhattan_vertical_inliers =
        value(args, "--minimum-manhattan-vertical-inliers", 3.0);
    if (minimum_manhattan_vertical_inliers < 0.0 ||
        std::floor(minimum_manhattan_vertical_inliers) !=
            minimum_manhattan_vertical_inliers)
      throw std::runtime_error(
          "--minimum-manhattan-vertical-inliers must be a non-negative "
          "integer");
    config.minimum_manhattan_vertical_inliers =
        static_cast<std::size_t>(minimum_manhattan_vertical_inliers);
    const double maximum_manhattan_vertical_error_deg =
        value(args, "--maximum-manhattan-vertical-error-deg", 20.0);
    if (maximum_manhattan_vertical_error_deg < 0.0 ||
        maximum_manhattan_vertical_error_deg > 90.0)
      throw std::runtime_error(
          "--maximum-manhattan-vertical-error-deg must be within [0, 90]");
    config.maximum_manhattan_vertical_error_rad =
        radians(maximum_manhattan_vertical_error_deg);
    config.minimum_projected_structural_segments = 1;
    const double minimum_structural_direction_groups_value =
        value(args, "--minimum-structural-direction-groups", 2.0);
    if (minimum_structural_direction_groups_value < 0.0 ||
        minimum_structural_direction_groups_value > 2.0 ||
        std::floor(minimum_structural_direction_groups_value) !=
            minimum_structural_direction_groups_value)
      throw std::runtime_error(
          "--minimum-structural-direction-groups must be 0, 1, or 2");
    const auto minimum_structural_direction_groups =
        static_cast<std::size_t>(minimum_structural_direction_groups_value);
    const double maximum_camera_downward_deg =
        value(args, "--maximum-camera-downward-deg", 75.0);
    if (maximum_camera_downward_deg < -90.0 ||
        maximum_camera_downward_deg > 90.0)
      throw std::runtime_error(
          "--maximum-camera-downward-deg must be within [-90, 90]");
    config.lidar_plane_normal_threshold_rad =
        radians(value(args, "--plane-normal-deg", 15.0));
    const double minimum_plane_points =
        value(args, "--plane-min-points", 80.0);
    if (minimum_plane_points < 3.0)
      throw std::runtime_error("--plane-min-points must be at least 3");
    config.minimum_lidar_plane_points =
        static_cast<std::size_t>(std::lround(minimum_plane_points));
    config.maximum_lidar_plane_rms_error_m =
        value(args, "--plane-max-rms-m", 0.03);
    const double plane_neighbor_radius =
        value(args, "--plane-neighbor-radius-cells", 3.0);
    if (plane_neighbor_radius < 1.0)
      throw std::runtime_error(
          "--plane-neighbor-radius-cells must be at least 1");
    config.plane_pair_neighbor_radius_cells =
        static_cast<std::size_t>(std::lround(plane_neighbor_radius));
    config.maximum_plane_intersection_boundary_distance_m =
        value(args, "--plane-boundary-distance-m", 0.10);
    config.enable_ground_plane_constraint =
        textValue(args, "--enable-ground-plane-constraint", "true") == "true";
    config.minimum_camera_ground_height_m =
        value(args, "--ground-height-min-m", 0.8);
    config.maximum_camera_ground_height_m =
        value(args, "--ground-height-max-m", 5.0);
    config.minimum_camera_downward_pitch_deg =
        value(args, "--ground-pitch-min-deg", 5.0);
    config.maximum_camera_downward_pitch_deg =
        value(args, "--ground-pitch-max-deg", 60.0);
    config.maximum_camera_ground_tilt_deg =
        value(args, "--ground-tilt-max-deg", 85.0);
    if (config.normalized_information_distance_weight < 0.0 ||
        config.edge_alignment_weight < 0.0 ||
        config.signal_nmi_weight < 0.0 ||
        config.structural_line_weight < 0.0 ||
        config.manhattan_direction_weight < 0.0 ||
        config.normalized_information_distance_weight +
                config.signal_nmi_weight +
                config.edge_alignment_weight + config.structural_line_weight +
                config.manhattan_direction_weight <=
            0.0)
      throw std::runtime_error(
          "Invalid NID/edge/line/Manhattan objective weights");
    config.minimum_projected_ratio = 0.20;
    if (args.count("--minimum-relative-edge-coverage"))
      throw std::runtime_error(
          "--minimum-relative-edge-coverage was removed: relative edge "
          "coverage is now a soft penalty controlled by "
          "--coverage-penalty-weight");
    config.minimum_relative_nid_coverage = value(
        args, "--minimum-relative-nid-coverage", 0.50);
    config.minimum_relative_edge_spatial_coverage = value(
        args, "--minimum-relative-spatial-coverage", 0.50);
    config.coverage_penalty_weight =
        value(args, "--coverage-penalty-weight", 0.25);
    if (config.minimum_relative_nid_coverage < 0.0 ||
        config.minimum_relative_nid_coverage > 1.0 ||
        config.minimum_relative_edge_spatial_coverage < 0.0 ||
        config.minimum_relative_edge_spatial_coverage > 1.0 ||
        config.coverage_penalty_weight < 0.0)
      throw std::runtime_error("Invalid relative coverage configuration");
    config.minimum_objective_improvement_ratio = 0.05;
    config.minimum_nid_improvement_ratio =
        value(args, "--minimum-nid-improvement", 0.01);
    config.minimum_signal_nmi_improvement_ratio =
        value(args, "--minimum-signal-nmi-improvement", 0.0);
    config.residual_cap_px = 30.0;
    config.maximum_solver_iterations = 150;
    config.coarse_yaw_span_rad = kPi;
    const double yaw_step_deg = value(args, "--yaw-step-deg", 15.0);
    if (yaw_step_deg <= 0.0 || yaw_step_deg > 360.0)
      throw std::runtime_error("Invalid yaw search step");
    config.coarse_yaw_step_rad = radians(yaw_step_deg);
    if (args.count("--yaw-min-deg") || args.count("--yaw-max-deg")) {
      if (!args.count("--yaw-min-deg") || !args.count("--yaw-max-deg"))
        throw std::runtime_error(
            "--yaw-min-deg and --yaw-max-deg must be supplied together");
      const double yaw_min_deg = value(args, "--yaw-min-deg", -180.0);
      const double yaw_max_deg = value(args, "--yaw-max-deg", 180.0);
      if (yaw_max_deg < yaw_min_deg || yaw_max_deg - yaw_min_deg > 360.0)
        throw std::runtime_error("Invalid bounded yaw search range");
      config.use_coarse_yaw_bounds = true;
      config.coarse_yaw_min_rad = radians(yaw_min_deg);
      config.coarse_yaw_max_rad = radians(yaw_max_deg);
    }
    config.minimum_multistart_objective_margin = 0.02;
    config.minimum_finalist_confidence_margin =
        value(args, "--minimum-finalist-confidence-margin", 0.02);
    config.rotation_search_bound_rad = radians(10.0);
    config.translation_search_bound_m = 0.10;
    config.maximum_rotation_update_rad = kPi;
    config.maximum_translation_update_m = 0.10;
    config.rotation_prior_sigma_rad = 1.0;
    config.translation_prior_sigma_m = 0.03;
    config.prior_weight = 0.10;
    config.minimum_intrinsic_observations = 3;
    const double holdout_count_value = value(
        args, "--holdout-count", observations.size() >= 4 ? 1.0 : 0.0);
    if (holdout_count_value < 0.0 ||
        std::floor(holdout_count_value) != holdout_count_value ||
        holdout_count_value >= static_cast<double>(observations.size()))
      throw std::runtime_error(
          "--holdout-count must be a non-negative integer smaller than the "
          "observation count");
    const auto holdout_count =
        static_cast<std::size_t>(holdout_count_value);
    const auto training_end = observations.end() - holdout_count;
    const std::vector<auto_calib::CalibrationObservation>
        calibration_observations(observations.begin(), training_end);
    const std::vector<auto_calib::CalibrationObservation> holdout_observations(
        training_end, observations.end());
    const double minimum_scene_pass_ratio =
        value(args, "--minimum-scene-pass-ratio", 0.80);
    if (minimum_scene_pass_ratio < 0.0 || minimum_scene_pass_ratio > 1.0)
      throw std::runtime_error(
          "--minimum-scene-pass-ratio must be within [0, 1]");
    const double minimum_scene_direction_groups_value =
        value(args, "--minimum-scene-direction-groups", 1.0);
    if (minimum_scene_direction_groups_value < 0.0 ||
        minimum_scene_direction_groups_value > 2.0 ||
        std::floor(minimum_scene_direction_groups_value) !=
            minimum_scene_direction_groups_value)
      throw std::runtime_error(
          "--minimum-scene-direction-groups must be 0, 1, or 2");
    const auto minimum_scene_direction_groups =
        static_cast<std::size_t>(minimum_scene_direction_groups_value);
    // Three observations are required only when K is being estimated jointly
    // with RT.  With a validated manual K/distortion profile fixed, two
    // training pairs are sufficient for an RT estimate and are required for
    // the common 2-train/1-holdout reproducibility test.
    const bool profile_distortion_unknown =
        manual_intrinsic && image_distortion_state == "unknown";
    const bool manufacturer_fov_diagnostic = !manual_intrinsic;
    const bool diagnostic_only =
        manufacturer_fov_diagnostic || profile_distortion_unknown ||
        (!manual_intrinsic &&
         calibration_observations.size() < config.minimum_intrinsic_observations);
    config.optimize_camera_intrinsics =
        enable_experimental_joint_intrinsic && !diagnostic_only &&
        (!manual_intrinsic || allow_intrinsic_refinement);
    config.focal_length_relative_bound = 0.45;
    config.principal_point_bound_ratio = 0.05;
    config.intrinsic_prior_weight = 2.0;
    const double baseline = value(args, "--baseline-m", 0.28);
    const double heading_rad = radians(value(args, "--prior-pitch-deg", 0.0));
    const auto optical_roll_candidates = numericRange(
        args, "--optical-roll-min-deg", "--optical-roll-max-deg",
        "--optical-roll-step-deg",
        value(args, "--prior-yaw-deg", 0.0));
    std::vector<double> downward_degrees;
    if (args.count("--prior-roll-deg")) {
      downward_degrees.push_back(value(args, "--prior-roll-deg", 0.0));
    } else {
      const double minimum = value(args, "--down-min-deg", 0.0);
      const double maximum = value(args, "--down-max-deg", 90.0);
      const double step = value(args, "--down-step-deg", 15.0);
      if (step <= 0.0 || maximum < minimum)
        throw std::runtime_error("Invalid downward direction search range");
      for (double angle = minimum; angle <= maximum + step * 0.25;
           angle += step)
        downward_degrees.push_back(angle);
    }
    std::vector<auto_calib::CalibrationResult> results;
    std::vector<auto_calib::Transform> priors;
    std::vector<double> candidate_downward_degrees;
    std::vector<double> candidate_optical_roll_degrees;
    std::vector<double> candidate_focal_scales;
    nlohmann::json candidates = nlohmann::json::array();
    const bool explicit_camera_center = args.count("--camera-center-x-m") ||
                                        args.count("--camera-center-y-m") ||
                                        args.count("--camera-center-z-m");
    const std::vector<double> signs = explicit_camera_center
                                          ? std::vector<double>{1.0}
                                          : std::vector<double>{-1.0, 1.0};
    const bool camera_outward_facing =
        textValue(args, "--camera-outward-facing", "false") == "true";
    const bool explicit_camera_forward = args.count("--expected-forward-x") ||
                                         args.count("--expected-forward-y") ||
                                         args.count("--expected-forward-z");
    const bool explicit_camera_down = args.count("--expected-down-x") ||
                                      args.count("--expected-down-y") ||
                                      args.count("--expected-down-z");
    if (explicit_camera_forward) {
      Eigen::Vector3d expected{value(args, "--expected-forward-x", 0.0),
                               value(args, "--expected-forward-y", 0.0),
                               value(args, "--expected-forward-z", 0.0)};
      if (expected.norm() <= 1e-9)
        throw std::runtime_error("Expected camera forward must be non-zero");
      config.expected_camera_forward_lidar = expected.normalized();
      config.camera_direction_prior_weight =
          value(args, "--direction-prior-weight", 2.0);
    } else if (camera_outward_facing) {
      if (!explicit_camera_center)
        throw std::runtime_error(
            "--camera-outward-facing requires explicit camera center");
      Eigen::Vector3d expected{
          value(args, "--camera-center-x-m", 0.0), 0.0,
          value(args, "--camera-center-z-m", 0.0)};
      if (expected.norm() <= 1e-9)
        throw std::runtime_error(
            "Camera center must have a horizontal offset for outward prior");
      config.expected_camera_forward_lidar = expected.normalized();
      config.camera_direction_prior_weight =
          value(args, "--direction-prior-weight", 0.35);
    }
    if (explicit_camera_down) {
      Eigen::Vector3d expected{value(args, "--expected-down-x", 0.0),
                               value(args, "--expected-down-y", 0.0),
                               value(args, "--expected-down-z", 0.0)};
      if (expected.norm() <= 1e-9)
        throw std::runtime_error("Expected camera down must be non-zero");
      config.expected_camera_down_lidar = expected.normalized();
      config.camera_direction_prior_weight =
          value(args, "--direction-prior-weight", 2.0);
    }
    if (explicit_camera_center) {
      config.use_camera_center_prior = true;
      config.expected_camera_center_lidar = {
          value(args, "--camera-center-x-m", 0.0),
          value(args, "--camera-center-y-m", 0.0),
          value(args, "--camera-center-z-m", 0.0)};
    }
    const bool installation_pose_available =
        config.use_camera_center_prior && explicit_camera_forward &&
        explicit_camera_down;
    auto_calib::Transform installation_transform;
    if (installation_pose_available)
      installation_transform = installationTransform(
          config.expected_camera_center_lidar,
          config.expected_camera_forward_lidar,
          config.expected_camera_down_lidar);

    auto make_prior = [&](double sign, double optical_roll_deg,
                          double downward_deg) {
      const double prior_x = sign * baseline;
      auto prior = auto_calib::makeTransform(
          {prior_x, 0.0, 0.0}, {0.0, 0.0, 0.0});
      prior.rotation =
          Eigen::AngleAxisd(radians(optical_roll_deg), Eigen::Vector3d::UnitZ())
              .toRotationMatrix() *
          Eigen::AngleAxisd(radians(downward_deg), Eigen::Vector3d::UnitX())
              .toRotationMatrix() *
          Eigen::AngleAxisd(-heading_rad, Eigen::Vector3d::UnitY())
              .toRotationMatrix();
      if (explicit_camera_center)
        prior.translation_m = -prior.rotation *
                              config.expected_camera_center_lidar;
      return prior;
    };
    if (reference_perturbation_only) {
      fs::create_directories(output_dir);
      const auto perturbation = writeSignalNmiConformance(
          observations, manual_reference_transform, config,
          output_dir / "reference_rt_perturbation.csv");
      const nlohmann::json perturbation_report = {
          {"status", perturbation.at("status")},
          {"mode", "reference_rt_perturbation_only"},
          {"manual_reference_json", manual_reference_path.string()},
          {"manual_intrinsic_json",
           manual_intrinsic ? nlohmann::json(manual_intrinsic_path.string())
                            : nlohmann::json(nullptr)},
          {"intrinsics_source", camera.intrinsics_source},
          {"image_distortion_state", image_distortion_state},
          {"ldc_enabled", ldc_enabled},
          {"perturbation", perturbation},
          {"activation_policy",
           "diagnostic_evidence_only; does not activate RT or signal NMI"}};
      std::ofstream(output_dir / "reference_rt_perturbation_result.json")
          << std::setw(2) << perturbation_report << '\n';
      std::cout << std::setw(2) << perturbation_report << '\n';
      return perturbation.at("status").get<std::string>() == "PASS" ? 0 : 3;
    }
    if (args.count("--validation-pose-json")) {
      const fs::path validation_pose_path =
          args.at("--validation-pose-json");
      const auto validation_pose = loadTransformJson(validation_pose_path);
      const auto scene_metrics = auto_calib::evaluateCalibrationPoseScenes(
          observations, validation_pose, config);
      const auto scene_validation = writePoseSceneValidation(
          scene_metrics, config, minimum_scene_direction_groups,
          output_dir / "fixed_pose_scene_validation.csv");
      const bool validation_pass =
          scene_validation.at("pass_ratio").get<double>() >=
          minimum_scene_pass_ratio;
      const std::string validation_debug =
          textValue(args, "--debug-output", "");
      if (!validation_debug.empty())
        writeDebugArtifacts(observations, validation_pose, validation_pose,
                            observations.front().camera,
                            "FIXED CROSS-CONDITION RT", config,
                            validation_debug);
      const nlohmann::json validation_report = {
          {"status", validation_pass ? "INTERNAL_GATE_PASS" : "FAIL"},
          {"internal_gate_status",
           validation_pass ? "INTERNAL_GATE_PASS" : "INTERNAL_GATE_FAIL"},
          {"candidate_rt_status", "NOT_CANDIDATE_RT"},
          {"product_approved_rt_status", "NOT_PRODUCT_APPROVED_RT"},
          {"activation_allowed", false},
          {"reason_code", validation_pass
                              ? "PASS"
                              : "FIXED_POSE_VALIDATION_FAILED"},
          {"mode", "fixed_pose_cross_condition_validation"},
          {"validation_label",
           textValue(args, "--validation-label", "unspecified")},
          {"validation_pose_json", validation_pose_path.string()},
          {"t_camera_lidar", transformJson(validation_pose)},
          {"input_pair_start", pair_start},
          {"input_pair_count", pair_count},
          {"pairing_basis",
           "operator_confirmed_group_order_then_lexicographic_index"},
          {"pairs", pairs},
          {"intrinsics_source",
           manual_intrinsic ? "manual_intrinsic_fixed"
                            : "manufacturer_fov_fixed"},
          {"intrinsics", cameraJson(observations.front().camera)},
          {"image_distortion_state", image_distortion_state},
          {"ldc_enabled", ldc_enabled},
          {"minimum_scene_pass_ratio", minimum_scene_pass_ratio},
          {"scene_validation", scene_validation},
          {"algorithm",
           {{"lidar_edge_policy",
             "absolute_relative_threshold_plus_local_contrast_and_coplanar_"
             "rejection"},
            {"lidar_edge_minimum_local_contrast_ratio",
             config.lidar_edge_minimum_local_contrast_ratio},
            {"geometry_nid_channels", "range_and_surface_normal"},
            {"structural_line_feature",
             "plane_intersections_plane_boundaries_persistent_occlusions"},
            {"visibility", "quarter_resolution_z_buffer_10mm"}}},
          {"activation_policy",
           "validation_only_never_activates_or_refines_the_input_RT"}};
      std::ofstream(output_dir / "fixed_pose_validation_result.json")
          << std::setw(2) << validation_report << '\n';
      std::cout << std::setw(2) << validation_report << '\n';
      return validation_pass ? 0 : 3;
    }
    for (double sign : signs) {
      for (double candidate_focal_scale : focal_scale_candidates) {
        auto candidate_observations = calibration_observations;
        const double focal_ratio = candidate_focal_scale / focal_scale;
        for (auto &observation : candidate_observations) {
          observation.camera.k(0, 0) *= focal_ratio;
          observation.camera.k(1, 1) *= focal_ratio;
        }
        for (double optical_roll_deg : optical_roll_candidates) {
          for (double downward_deg : downward_degrees) {
        auto prior = make_prior(sign, optical_roll_deg, downward_deg);
        priors.push_back(prior);
        candidate_downward_degrees.push_back(downward_deg);
        candidate_optical_roll_degrees.push_back(optical_roll_deg);
        candidate_focal_scales.push_back(candidate_focal_scale);
        auto result = auto_calib::calibrateExtrinsicMultiScene(
            candidate_observations, prior, config);
        applySelectionGates(&result, minimum_structural_direction_groups,
                            maximum_camera_downward_deg);
        results.push_back(std::move(result));
        auto candidate = resultJson(results.back(), prior.translation_m.x(),
                                    downward_deg, optical_roll_deg,
                                    candidate_focal_scale);
        candidate["structural_direction_groups"] =
            structuralDirectionGroups(results.back());
        candidate["candidate_camera_downward_deg"] =
            cameraDownwardDeg(results.back().candidate_t_camera_lidar);
        candidate["selection_eligible"] = selectionEligible(
            results.back(), minimum_structural_direction_groups,
            maximum_camera_downward_deg);
        candidates.push_back(std::move(candidate));
          }
        }
      }
    }
    auto score = [](const auto_calib::CalibrationResult &r) {
      return r.reason_code != "COARSE_OVERLAP_INSUFFICIENT" &&
                     r.metrics.lidar_geometry_points > 0 &&
                     std::isfinite(r.metrics.final_composite_objective)
                 ? r.metrics.final_composite_objective
                 : std::numeric_limits<double>::infinity();
    };
    std::vector<std::size_t> ranking(results.size());
    for (std::size_t i = 0; i < ranking.size(); ++i)
      ranking[i] = i;
    std::sort(ranking.begin(), ranking.end(),
              [&](std::size_t a, std::size_t b) {
                if (results[a].success != results[b].success)
                  return results[a].success;
                const bool a_eligible = selectionEligible(
                    results[a], minimum_structural_direction_groups,
                    maximum_camera_downward_deg);
                const bool b_eligible = selectionEligible(
                    results[b], minimum_structural_direction_groups,
                    maximum_camera_downward_deg);
                if (a_eligible != b_eligible)
                  return a_eligible;
                return score(results[a]) < score(results[b]);
              });
    std::size_t selected = ranking.front();
    double finalist_objective_margin = 1.0;
    struct StagedFinalistRecord {
      std::size_t result_index = 0;
      double training_pass_ratio = 0.0;
    };
    std::vector<StagedFinalistRecord> staged_finalists;
    bool final_success = false;
    std::string final_reason = "SEARCH_NOT_REFINED";
    auto display_transform = results[selected].candidate_t_camera_lidar;
    auto display_camera = results[selected].candidate_camera;
    fs::create_directories(output_dir);
    const std::string debug_output = textValue(args, "--debug-output", "");
    const fs::path orientation_score_path =
        output_dir / "orientation_full_search.csv";
    const bool circular_yaw =
        !config.use_coarse_yaw_bounds ||
        config.coarse_yaw_max_rad - config.coarse_yaw_min_rad >=
            2.0 * kPi - config.coarse_yaw_step_rad * 0.5;
    double raw_best_down = 0.0, raw_best_yaw = 0.0,
           raw_best_objective = std::numeric_limits<double>::infinity();
    const std::size_t orientation_score_count = writeOrientationScoreMap(
        results, candidate_downward_degrees, candidate_optical_roll_degrees,
        candidate_focal_scales, orientation_score_path,
        &raw_best_down, &raw_best_yaw, &raw_best_objective);
    const std::size_t layer_size = downward_degrees.size();
    BasinSelection basin;
    std::size_t basin_proposal = selected;
    nlohmann::json orientation_layers = nlohmann::json::array();
    nlohmann::json search_stages = nlohmann::json::array();
    std::size_t global_max_visible_edges = 0;
    std::size_t global_max_nid_points = 0;
    std::size_t global_max_edge_cells = 0;
    double global_max_structural_length = 0.0;

    for (const auto &res : results) {
      global_max_visible_edges =
          std::max(global_max_visible_edges, res.metrics.visible_edge_points);
      global_max_nid_points =
          std::max(global_max_nid_points, res.metrics.nid_projected_points);
      global_max_edge_cells =
          std::max(global_max_edge_cells, res.metrics.edge_active_spatial_cells);
      global_max_structural_length =
          std::max(global_max_structural_length,
                   res.metrics.total_explained_structural_length);
      for (const auto &score : res.coarse_orientation_scores) {
        global_max_visible_edges =
            std::max(global_max_visible_edges, score.visible_edge_points);
        global_max_nid_points =
            std::max(global_max_nid_points, score.nid_projected_points);
        global_max_edge_cells =
            std::max(global_max_edge_cells, score.edge_active_spatial_cells);
        global_max_structural_length =
            std::max(global_max_structural_length,
                     score.total_explained_structural_length);
      }
    }

    if (staged_search) {
      struct LocalCandidate {
        auto_calib::CalibrationResult result;
        auto_calib::Transform prior;
        double down_deg = 0.0;
        double roll_deg = 0.0;
        double focal_scale = 1.0;
        double yaw_deg = 0.0;
        double objective = std::numeric_limits<double>::infinity();
        bool stage_gate_pass = false;
      };
      constexpr std::size_t kMaximumStagedBasins = 3;
      constexpr double kMinimumDistinctStagedYawDeg = 30.0;
      auto scaled_observations = [&](double focal) {
        auto out = calibration_observations;
        const double ratio = focal / focal_scale;
        for (auto &observation : out) {
          observation.camera.k(0, 0) *= ratio;
          observation.camera.k(1, 1) *= ratio;
        }
        return out;
      };
      auto stage_score = [&](const auto_calib::CalibrationResult &value) {
        return value.metrics.lidar_geometry_points > 0 &&
                       std::isfinite(value.metrics.final_composite_objective)
                   ? value.metrics.final_composite_objective
                   : std::numeric_limits<double>::infinity();
      };
      auto run_local_stage = [&](const std::vector<LocalCandidate> &seeds,
                                 double step_deg, double radius_deg,
                                 const std::string &stage_name) {
        std::vector<LocalCandidate> winners;
        const fs::path csv_path = output_dir / (stage_name + "_scores.csv");
        std::ofstream csv(csv_path);
        if (!csv)
          throw std::runtime_error("Cannot write staged search score map");
        csv << "seed,down_deg,roll_deg,yaw_center_deg,yaw_deg,objective,"
               "reason_code,stage_gate_pass\n"
            << std::setprecision(12);
        for (std::size_t seed_index = 0; seed_index < seeds.size();
             ++seed_index) {
          const auto &seed = seeds[seed_index];
          LocalCandidate winner;
          bool has_winner = false;
          const double down_min =
              std::max(0.0, seed.down_deg - radius_deg);
          const double down_max =
              std::min(90.0, seed.down_deg + radius_deg);
          const double roll_min = seed.roll_deg - radius_deg;
          const double roll_max = seed.roll_deg + radius_deg;
          for (double roll = roll_min; roll <= roll_max + step_deg * 0.25;
               roll += step_deg) {
            for (double down = down_min;
                 down <= down_max + step_deg * 0.25; down += step_deg) {
              auto stage_config = config;
              stage_config.enable_ceres_refinement = false;
              stage_config.use_coarse_yaw_bounds = true;
              stage_config.coarse_yaw_span_rad = 0.0;
              stage_config.coarse_yaw_step_rad = radians(step_deg);
              stage_config.coarse_yaw_min_rad =
                  radians(seed.yaw_deg - radius_deg);
              stage_config.coarse_yaw_max_rad =
                  radians(seed.yaw_deg + radius_deg);
              stage_config.global_reference_visible_edge_points =
                  global_max_visible_edges;
              // NID support changes substantially between unrelated camera
              // sectors.  A global maximum can reject every yaw in an otherwise
              // valid local basin, so keep the configured relative gate local
              // to this basin's yaw window.  Global edge/cell references remain
              // useful soft coverage diagnostics.
              stage_config.global_reference_nid_projected_points = 0;
              stage_config.global_reference_edge_active_cells =
                  global_max_edge_cells;
              stage_config.global_reference_structural_length =
                  global_max_structural_length;
              const double sign =
                  explicit_camera_center || seed.prior.translation_m.x() >= 0.0
                      ? 1.0
                      : -1.0;
              const auto prior = make_prior(sign, roll, down);
              auto result = auto_calib::calibrateExtrinsicMultiScene(
                  scaled_observations(seed.focal_scale), prior, stage_config);
              applySelectionGates(&result, minimum_structural_direction_groups,
                                  maximum_camera_downward_deg);
              const double objective = stage_score(result);
              const bool ground_ok = !config.enable_ground_plane_constraint ||
                                     result.metrics.ground_normal_valid;
              const bool tesl_ok =
                  (config.total_explained_structural_length_min <= 0.0 ||
                   result.metrics.total_explained_structural_length >=
                       config.total_explained_structural_length_min);
              const bool stage_gate_pass =
                  selectionEligible(result, minimum_structural_direction_groups,
                                    maximum_camera_downward_deg) &&
                  manhattanStageEligible(result, config,
                                         calibration_observations.size()) &&
                  ground_ok && tesl_ok;
              const double selected_yaw =
                  result.metrics.selected_multistart_yaw_deg;
              csv << seed_index << ',' << down << ',' << roll << ','
                  << seed.yaw_deg << ','
                  << selected_yaw << ','
                  << objective << ',' << result.reason_code << ','
                  << (stage_gate_pass ? 1 : 0) << '\n';
              if (std::isfinite(objective) &&
                  (!has_winner ||
                   (stage_gate_pass && !winner.stage_gate_pass) ||
                   (stage_gate_pass == winner.stage_gate_pass &&
                    objective < winner.objective))) {
                winner = {std::move(result), prior, down, roll,
                          seed.focal_scale, selected_yaw, objective,
                          stage_gate_pass};
                has_winner = true;
              }
            }
          }
          if (has_winner)
            winners.push_back(std::move(winner));
        }
        search_stages.push_back({{"stage", stage_name},
                                 {"step_deg", step_deg},
                                 {"radius_deg", radius_deg},
                                 {"seed_count", seeds.size()},
                                 {"winner_count", winners.size()},
                                 {"score_map", csv_path.string()}});
        return winners;
      };

      std::vector<std::pair<BasinSelection, std::size_t>> layer_basins;
      for (std::size_t start = 0, layer = 0; start < results.size();
           start += layer_size, ++layer) {
        const auto end = std::min(start + layer_size, results.size());
        if (end - start != layer_size)
          break;
        const std::vector<auto_calib::CalibrationResult> layer_results(
            results.begin() + static_cast<std::ptrdiff_t>(start),
            results.begin() + static_cast<std::ptrdiff_t>(end));
        const std::vector<double> layer_down(
            candidate_downward_degrees.begin() +
                static_cast<std::ptrdiff_t>(start),
            candidate_downward_degrees.begin() +
                static_cast<std::ptrdiff_t>(end));
        std::vector<BasinSelection> layer_distinct;
        const auto layer_basin = writeCorrectedScoreMap(
            layer_results, layer_down,
            output_dir / ("orientation_corrected_layer_" +
                          std::to_string(layer) + ".csv"),
            minimum_structural_direction_groups, circular_yaw, 0.8, 5.0,
            5.0, &layer_distinct);
        orientation_layers.push_back(
            {{"layer", layer},
             {"optical_roll_deg", candidate_optical_roll_degrees[start]},
             {"focal_scale", candidate_focal_scales[start]},
             {"selected_down_deg",
              layer_down[std::min(layer_basin.result_index,
                                  layer_down.size() - 1)]},
             {"selected_yaw_deg", layer_basin.yaw_deg},
             {"corrected_score", layer_basin.corrected_score},
             {"basin_candidate_count", layer_basin.basin_count}});
        for (const auto &b_cand : layer_distinct) {
          if (std::isfinite(b_cand.corrected_score)) {
            auto global_basin = b_cand;
            global_basin.result_index += start;
            layer_basins.push_back({global_basin, start});
          }
        }
      }
      if (!layer_basins.empty()) {
        const fs::path first_layer_map =
            output_dir / "orientation_corrected_layer_0.csv";
        if (fs::exists(first_layer_map))
          fs::copy_file(first_layer_map,
                        output_dir / "orientation_corrected_scores.csv",
                        fs::copy_options::overwrite_existing);
      }
      std::sort(layer_basins.begin(), layer_basins.end(),
                [](const auto &a, const auto &b) {
                  return a.first.corrected_score < b.first.corrected_score;
                });
      if (!layer_basins.empty()) {
        basin = layer_basins.front().first;
        basin_proposal = basin.result_index;
        std::vector<LocalCandidate> coarse_seeds;
        coarse_seeds.reserve(kMaximumStagedBasins);
        for (const auto &ranked_basin : layer_basins) {
          if (!std::all_of(
                  coarse_seeds.begin(), coarse_seeds.end(),
                  [&](const LocalCandidate &seed) {
                    return circularYawDistanceDeg(seed.yaw_deg,
                                                  ranked_basin.first.yaw_deg) >=
                           kMinimumDistinctStagedYawDeg;
                  }))
            continue;
          const auto global_index = ranked_basin.first.result_index;
          coarse_seeds.push_back(
              {results[global_index], priors[global_index],
               candidate_downward_degrees[global_index],
               candidate_optical_roll_degrees[global_index],
               candidate_focal_scales[global_index],
               ranked_basin.first.yaw_deg,
               ranked_basin.first.corrected_score});
          if (coarse_seeds.size() == kMaximumStagedBasins)
            break;
        }
        search_stages.push_back({{"stage", "top3_basin"},
                                 {"candidate_count", coarse_seeds.size()},
                                 {"selection",
                                  "contiguous_score_basin_distinct_yaw"},
                                 {"minimum_yaw_separation_deg",
                                  kMinimumDistinctStagedYawDeg}});
        const auto five_degree =
            run_local_stage(coarse_seeds, 5.0, 10.0, "search_5deg");
        const auto one_degree =
            run_local_stage(five_degree, 1.0, 5.0, "search_1deg");
        if (!one_degree.empty()) {
          std::vector<std::size_t> seed_order(one_degree.size());
          std::iota(seed_order.begin(), seed_order.end(), 0);
          std::sort(seed_order.begin(), seed_order.end(),
                    [&](std::size_t a, std::size_t b) {
                      if (one_degree[a].stage_gate_pass !=
                          one_degree[b].stage_gate_pass)
                        return one_degree[a].stage_gate_pass;
                      return one_degree[a].objective < one_degree[b].objective;
                    });
          std::vector<std::size_t> final_seed_indices;
          final_seed_indices.reserve(kMaximumStagedBasins);
          for (const std::size_t seed_index : seed_order) {
            if (!std::all_of(
                    final_seed_indices.begin(), final_seed_indices.end(),
                    [&](std::size_t selected_seed) {
                      return circularYawDistanceDeg(
                                 one_degree[seed_index].yaw_deg,
                                 one_degree[selected_seed].yaw_deg) >=
                             kMinimumDistinctStagedYawDeg;
                    }))
              continue;
            final_seed_indices.push_back(seed_index);
            if (final_seed_indices.size() == kMaximumStagedBasins)
              break;
          }
          const auto scene_pass_ratio =
              [&](const auto_calib::CalibrationResult &candidate,
                  const std::vector<auto_calib::CalibrationObservation>
                      &source_observations,
                  const auto_calib::Transform &feature_prior) {
                auto validation_observations = source_observations;
                const auto &camera = candidate.success
                                         ? candidate.estimated_camera
                                         : candidate.candidate_camera;
                const auto &transform = candidate.success
                                            ? candidate.estimated_t_camera_lidar
                                            : candidate.candidate_t_camera_lidar;
                for (auto &observation : validation_observations)
                  observation.camera = camera;
                const auto metrics = auto_calib::evaluateCalibrationPoseScenes(
                    validation_observations, transform, config,
                    &feature_prior);
                const auto passed = std::count_if(
                    metrics.begin(), metrics.end(), [&](const auto &metric) {
                      return sceneValidationFailures(
                                 metric, config,
                                 minimum_scene_direction_groups)
                          .empty();
                    });
                return metrics.empty()
                           ? 1.0
                           : static_cast<double>(passed) / metrics.size();
              };
          struct Finalist {
            std::size_t result_index = 0;
            double objective = std::numeric_limits<double>::infinity();
            double training_pass_ratio = 0.0;
            bool scene_validation_pass = false;
            auto_calib::MultiCriteriaConfidence confidence;
          };
          std::vector<Finalist> finalists;
          finalists.reserve(final_seed_indices.size());
          for (std::size_t finalist_rank = 0;
               finalist_rank < final_seed_indices.size(); ++finalist_rank) {
            const auto &seed = one_degree[final_seed_indices[finalist_rank]];
            auto final_config = config;
            final_config.enable_ceres_refinement = true;
            final_config.use_coarse_yaw_bounds = true;
            final_config.coarse_yaw_span_rad = 0.0;
            final_config.coarse_yaw_step_rad = radians(1.0);
            final_config.coarse_yaw_min_rad = radians(seed.yaw_deg - 1.0);
            final_config.coarse_yaw_max_rad = radians(seed.yaw_deg + 1.0);
            final_config.global_reference_visible_edge_points =
                global_max_visible_edges;
            final_config.global_reference_nid_projected_points =
                global_max_nid_points;
            final_config.global_reference_edge_active_cells =
                global_max_edge_cells;
            final_config.global_reference_structural_length =
                global_max_structural_length;
            // Finalists have already passed basin-local overlap checks.  Keep
            // global NID coverage in the objective/confidence, but do not use a
            // different camera sector's point count as a hard rejection here.
            final_config.minimum_relative_nid_coverage = 0.0;
            auto final_result = auto_calib::calibrateExtrinsicMultiScene(
                scaled_observations(seed.focal_scale), seed.prior,
                final_config);
            applySelectionGates(&final_result,
                                minimum_structural_direction_groups,
                                maximum_camera_downward_deg);
            const double training_pass_ratio =
                scene_pass_ratio(final_result, calibration_observations,
                                 seed.prior);
            const bool scene_validation_pass =
                final_result.success &&
                training_pass_ratio >= minimum_scene_pass_ratio;
            const auto conf = auto_calib::evaluateMultiCriteriaConfidence(
                final_result, training_pass_ratio, final_config);
            final_result.metrics.multi_criteria_confidence_score =
                conf.total_confidence;

            const std::size_t final_index = results.size();
            results.push_back(std::move(final_result));
            priors.push_back(seed.prior);
            candidate_downward_degrees.push_back(seed.down_deg);
            candidate_optical_roll_degrees.push_back(seed.roll_deg);
            candidate_focal_scales.push_back(seed.focal_scale);
            const double final_objective = stage_score(results.back());
            auto final_candidate = resultJson(
                results.back(), seed.prior.translation_m.x(), seed.down_deg,
                seed.roll_deg, seed.focal_scale);
            final_candidate["search_stage"] = "ceres_final_distinct_basin";
            final_candidate["seed_rank"] = finalist_rank + 1;
            final_candidate["seed_yaw_deg"] = seed.yaw_deg;
            final_candidate["stage_gate_pass"] = seed.stage_gate_pass;
            final_candidate["training_scene_pass_ratio"] =
                training_pass_ratio;
            final_candidate["scene_validation_pass"] = scene_validation_pass;
            final_candidate["multi_criteria_confidence_score"] =
                conf.total_confidence;
            final_candidate["scene_validation_confidence"] =
                conf.scene_validation_score;
            final_candidate["ground_geometry_confidence"] =
                conf.ground_geometry_score;
            final_candidate["tesl_confidence"] = conf.tesl_score;
            final_candidate["spatial_nid_confidence"] = conf.spatial_nid_score;
            candidates.push_back(std::move(final_candidate));
            finalists.push_back(
                {final_index, final_objective, training_pass_ratio,
                 scene_validation_pass, conf});
            staged_finalists.push_back({final_index, training_pass_ratio});
            search_stages.push_back(
                {{"stage", "ceres_final_candidate"},
                 {"seed_rank", finalist_rank + 1},
                 {"seed_yaw_deg", seed.yaw_deg},
                 {"seed_down_deg", seed.down_deg},
                 {"seed_roll_deg", seed.roll_deg},
                 {"candidate_index", final_index},
                 {"training_scene_pass_ratio", training_pass_ratio},
                 {"scene_validation_pass", scene_validation_pass},
                 {"multi_criteria_confidence_score", conf.total_confidence}});
          }

          const auto quality_tier = [&](std::size_t finalist_idx) {
            const auto &finalist = finalists[finalist_idx];
            const auto &result = results[finalist.result_index];
            return std::make_tuple(
                finalist.scene_validation_pass, result.success,
                selectionEligible(result, minimum_structural_direction_groups,
                                  maximum_camera_downward_deg),
                result.metrics.absolute_support_pass);
          };
          const auto objective_margin_between = [&](std::size_t better_idx,
                                                     std::size_t other_idx) {
            const double better = finalists[better_idx].objective;
            const double other = finalists[other_idx].objective;
            if (!std::isfinite(better) || !std::isfinite(other))
              return std::isfinite(better) ? 1.0 : -1.0;
            return (other - better) / std::max(std::abs(other), 1e-12);
          };
          const auto choose_finalist =
              [&](const std::vector<std::size_t> &pool) {
                const auto best_tier_it = std::max_element(
                    pool.begin(), pool.end(), [&](std::size_t a,
                                                  std::size_t b) {
                      return quality_tier(a) < quality_tier(b);
                    });
                const auto best_tier = quality_tier(*best_tier_it);
                std::vector<std::size_t> tier_pool;
                for (const auto idx : pool)
                  if (quality_tier(idx) == best_tier)
                    tier_pool.push_back(idx);

                std::sort(tier_pool.begin(), tier_pool.end(),
                          [&](std::size_t a, std::size_t b) {
                            if (finalists[a].objective !=
                                finalists[b].objective)
                              return finalists[a].objective <
                                     finalists[b].objective;
                            return a < b;
                          });
                const std::size_t objective_best = tier_pool.front();
                if (tier_pool.size() == 1 ||
                    objective_margin_between(objective_best, tier_pool[1]) >=
                        config.minimum_multistart_objective_margin)
                  return objective_best;

                std::vector<std::size_t> objective_ties;
                for (const auto idx : tier_pool)
                  if (idx == objective_best ||
                      objective_margin_between(objective_best, idx) <
                          config.minimum_multistart_objective_margin)
                    objective_ties.push_back(idx);

                const auto structural_length = [&](std::size_t idx) {
                  return results[finalists[idx].result_index]
                      .metrics.total_explained_structural_length;
                };
                std::sort(objective_ties.begin(), objective_ties.end(),
                          [&](std::size_t a, std::size_t b) {
                            if (structural_length(a) != structural_length(b))
                              return structural_length(a) > structural_length(b);
                            return a < b;
                          });
                constexpr double kStructuralTieBreakRatio = 0.10;
                if (objective_ties.size() == 1)
                  return objective_ties.front();
                const double best_structural =
                    structural_length(objective_ties.front());
                const double second_structural =
                    structural_length(objective_ties[1]);
                const double structural_gap =
                    (best_structural - second_structural) /
                    std::max({std::abs(best_structural),
                              std::abs(second_structural), 1e-12});
                if (structural_gap >= kStructuralTieBreakRatio)
                  return objective_ties.front();

                return *std::max_element(
                    objective_ties.begin(), objective_ties.end(),
                    [&](std::size_t a, std::size_t b) {
                      const double a_conf =
                          finalists[a].confidence.total_confidence;
                      const double b_conf =
                          finalists[b].confidence.total_confidence;
                      if (std::abs(a_conf - b_conf) > 1e-4)
                        return a_conf < b_conf;
                      if (finalists[a].objective != finalists[b].objective)
                        return finalists[a].objective > finalists[b].objective;
                      return a > b;
                    });
              };

          // Build the complete order by repeatedly applying the same staged
          // decision.  Unlike a pairwise threshold comparator, this is a strict,
          // deterministic procedure even when three near-tied candidates form
          // a non-transitive comparison cycle.
          std::vector<std::size_t> finalist_rank_order;
          std::vector<std::size_t> remaining(finalists.size());
          std::iota(remaining.begin(), remaining.end(), 0);
          while (!remaining.empty()) {
            const std::size_t winner = choose_finalist(remaining);
            finalist_rank_order.push_back(winner);
            remaining.erase(
                std::find(remaining.begin(), remaining.end(), winner));
          }

          const auto &first_finalist = finalists[finalist_rank_order.front()];
          selected = first_finalist.result_index;
          double conf_margin = 1.0;
          bool finalist_ambiguous = false;
          int separated_second_idx = -1;

          const double first_yaw =
              results[first_finalist.result_index]
                  .metrics.selected_multistart_yaw_deg;

          for (std::size_t rank = 1; rank < finalist_rank_order.size(); ++rank) {
            const auto &cand_finalist = finalists[finalist_rank_order[rank]];
            const double cand_yaw =
                results[cand_finalist.result_index]
                    .metrics.selected_multistart_yaw_deg;
            if (circularYawDistanceDeg(first_yaw, cand_yaw) >
                kFinalistSeparationAngleDeg) {
              separated_second_idx =
                  static_cast<int>(cand_finalist.result_index);
              conf_margin = first_finalist.confidence.total_confidence -
                            cand_finalist.confidence.total_confidence;
              if (std::isfinite(first_finalist.objective) &&
                  std::isfinite(cand_finalist.objective)) {
                finalist_objective_margin =
                    (cand_finalist.objective - first_finalist.objective) /
                    std::max(std::abs(cand_finalist.objective), 1e-12);
              } else {
                finalist_objective_margin =
                    std::isfinite(first_finalist.objective) ? 1.0 : -1.0;
              }
              break;
            }
          }

          if (separated_second_idx >= 0) {
            results[selected].metrics.finalist_confidence_margin = conf_margin;
            const auto &first_metrics = results[selected].metrics;
            const auto &second_metrics =
                results[separated_second_idx].metrics;

            const bool second_is_viable =
                results[separated_second_idx].success ||
                conf_margin < 0.10;

            const bool ranking_margins_insufficient =
                finalist_objective_margin <
                    config.minimum_multistart_objective_margin &&
                conf_margin < config.minimum_finalist_confidence_margin;
            const bool support_inferior =
                second_is_viable &&
                ((first_metrics.visible_edge_points <
                  0.6 * second_metrics.visible_edge_points) ||
                 (first_metrics.nid_projected_points <
                  0.6 * second_metrics.nid_projected_points));

            if (ranking_margins_insufficient || support_inferior) {
              finalist_ambiguous = true;
              results[selected].success = false;
              results[selected].internal_gate_pass = false;
              results[selected].state = "INTERNAL_GATE_FAIL";
              results[selected].reason_code = "FINALIST_AMBIGUOUS";
            }
          } else {
            results[selected].metrics.finalist_confidence_margin = 1.0;
          }

          if (results[selected].success &&
              !results[selected].metrics.absolute_support_pass) {
            results[selected].success = false;
            results[selected].internal_gate_pass = false;
            results[selected].state = "INTERNAL_GATE_FAIL";
            results[selected].reason_code = "ABSOLUTE_SUPPORT_INSUFFICIENT";
          }

          search_stages.push_back(
              {{"stage", "ceres_final_selection"},
               {"candidate_count", finalists.size()},
               {"selected_candidate", selected},
               {"selection",
                "training_scene_validation_then_internal_gate_then_absolute_"
                "support_then_significant_objective_gap_else_TESL_then_"
                "multi_criteria_confidence"},
               {"selected_confidence_score",
                first_finalist.confidence.total_confidence},
               {"finalist_objective_margin", finalist_objective_margin},
               {"finalist_confidence_margin", conf_margin},
               {"finalist_ambiguous", finalist_ambiguous},
               {"minimum_yaw_separation_deg", kMinimumDistinctStagedYawDeg}});
        }
      } else {
        selected = 0;
        basin_proposal = 0;
        final_reason = "COARSE_BASIN_NOT_FOUND";
        search_stages.push_back({{"stage", "top3_basin"},
                                 {"candidate_count", 0},
                                 {"selection", "no_fallback"}});
      }
    } else {
      std::size_t selected_layer_start = 0;
      for (std::size_t start = 0, layer = 0; start < results.size();
           start += layer_size, ++layer) {
        const auto end = std::min(start + layer_size, results.size());
        if (end - start != layer_size)
          break;
        const std::vector<auto_calib::CalibrationResult> layer_results(
            results.begin() + static_cast<std::ptrdiff_t>(start),
            results.begin() + static_cast<std::ptrdiff_t>(end));
        const std::vector<double> layer_down(
            candidate_downward_degrees.begin() +
                static_cast<std::ptrdiff_t>(start),
            candidate_downward_degrees.begin() +
                static_cast<std::ptrdiff_t>(end));
        const auto layer_basin = writeCorrectedScoreMap(
            layer_results, layer_down,
            output_dir / ("orientation_corrected_layer_" +
                          std::to_string(layer) + ".csv"),
            minimum_structural_direction_groups, circular_yaw, 0.8, 5.0,
            5.0);
        if (layer_basin.corrected_score < basin.corrected_score) {
          basin = layer_basin;
          selected_layer_start = start;
        }
      }
      const std::vector<auto_calib::CalibrationResult> selected_layer_results(
          results.begin() + static_cast<std::ptrdiff_t>(selected_layer_start),
          results.begin() + static_cast<std::ptrdiff_t>(selected_layer_start +
                                                        layer_size));
      const std::vector<double> selected_layer_down(
          candidate_downward_degrees.begin() +
              static_cast<std::ptrdiff_t>(selected_layer_start),
          candidate_downward_degrees.begin() + static_cast<std::ptrdiff_t>(
              selected_layer_start + layer_size));
      basin = writeCorrectedScoreMap(
          selected_layer_results, selected_layer_down,
          output_dir / "orientation_corrected_scores.csv",
          minimum_structural_direction_groups, circular_yaw, 0.8, 5.0, 5.0);
      basin.result_index += selected_layer_start;
      basin_proposal = basin.result_index;
      selected = basin_proposal;
      search_stages.push_back({{"stage", "legacy_basin"},
                               {"candidate_index", selected},
                               {"selection", "no_fallback"}});
    }
    if (ranking.size() != results.size()) {
      ranking.resize(results.size());
      std::iota(ranking.begin(), ranking.end(), 0);
      std::sort(ranking.begin(), ranking.end(),
                [&](std::size_t a, std::size_t b) {
                  if (results[a].success != results[b].success)
                    return results[a].success;
                  const bool a_eligible = selectionEligible(
                      results[a], minimum_structural_direction_groups,
                      maximum_camera_downward_deg);
                  const bool b_eligible = selectionEligible(
                      results[b], minimum_structural_direction_groups,
                      maximum_camera_downward_deg);
                  if (a_eligible != b_eligible)
                    return a_eligible;
                  return score(results[a]) < score(results[b]);
                });
    }
    final_success = !diagnostic_only && results[selected].success;
    final_reason = diagnostic_only
                       ? "SINGLE_OBSERVATION_DIAGNOSTIC_ONLY"
                       : (staged_search && !std::isfinite(basin.corrected_score)
                              ? "COARSE_BASIN_NOT_FOUND"
                              : results[selected].reason_code);
    display_transform = results[selected].success
                            ? results[selected].estimated_t_camera_lidar
                            : results[selected].candidate_t_camera_lidar;
    display_camera = results[selected].success
                         ? results[selected].estimated_camera
                         : results[selected].candidate_camera;
    auto training_validation_observations = calibration_observations;
    for (auto &observation : training_validation_observations)
      observation.camera = display_camera;
    auto holdout_validation_observations = holdout_observations;
    for (auto &observation : holdout_validation_observations)
      observation.camera = display_camera;
    const auto training_scene_metrics =
        auto_calib::evaluateCalibrationPoseScenes(
            training_validation_observations, display_transform, config,
            &priors[selected]);
    const auto holdout_scene_metrics =
        auto_calib::evaluateCalibrationPoseScenes(
            holdout_validation_observations, display_transform, config,
            &priors[selected]);
    const nlohmann::json training_scene_validation =
        writePoseSceneValidation(
            training_scene_metrics, config, minimum_scene_direction_groups,
            output_dir / "training_scene_validation.csv");
    const nlohmann::json holdout_scene_validation =
        holdout_scene_metrics.empty()
            ? nlohmann::json(nullptr)
            : writePoseSceneValidation(
                  holdout_scene_metrics, config,
                  minimum_scene_direction_groups,
                  output_dir / "holdout_scene_validation.csv");
    nlohmann::json finalist_holdout_validation = nlohmann::json::array();
    bool finalist_holdout_distinctive = true;
    std::size_t passing_separated_holdout_competitors = 0;
    std::size_t ambiguous_separated_holdout_competitors = 0;
    double selected_holdout_objective =
        std::numeric_limits<double>::infinity();
    double minimum_separated_holdout_objective_margin = 1.0;
    const double selected_holdout_pass_ratio =
        holdout_scene_metrics.empty()
            ? 0.0
            : holdout_scene_validation.at("pass_ratio").get<double>();
    const double selected_yaw =
        results[selected].metrics.selected_multistart_yaw_deg;

    struct FinalistHoldoutRecord {
      StagedFinalistRecord finalist;
      std::vector<auto_calib::PoseSceneMetrics> metrics;
      nlohmann::json validation;
      auto_calib::PoseObjectiveMetrics objective;
      double pass_ratio = 0.0;
      double yaw = 0.0;
      double yaw_distance = 0.0;
      bool separated = false;
      bool viable = false;
    };
    std::vector<FinalistHoldoutRecord> holdout_records;
    holdout_records.reserve(staged_finalists.size());
    for (const auto &finalist : staged_finalists) {
      const auto &candidate = results[finalist.result_index];
      const auto &candidate_transform =
          candidate.success ? candidate.estimated_t_camera_lidar
                            : candidate.candidate_t_camera_lidar;
      const auto &candidate_camera =
          candidate.success ? candidate.estimated_camera
                            : candidate.candidate_camera;
      auto candidate_holdout_observations = holdout_observations;
      for (auto &observation : candidate_holdout_observations)
        observation.camera = candidate_camera;
      const auto candidate_metrics =
          finalist.result_index == selected
              ? holdout_scene_metrics
              : auto_calib::evaluateCalibrationPoseScenes(
                    candidate_holdout_observations, candidate_transform,
                    config, &priors[finalist.result_index]);
      const fs::path validation_path =
          output_dir /
          ("finalist_holdout_candidate_" +
           std::to_string(finalist.result_index) + ".csv");
      const nlohmann::json validation =
          candidate_metrics.empty()
              ? nlohmann::json(nullptr)
              : writePoseSceneValidation(
                    candidate_metrics, config,
                    minimum_scene_direction_groups, validation_path);
      const double pass_ratio =
          candidate_metrics.empty()
              ? 0.0
              : validation.at("pass_ratio").get<double>();
      const double yaw = candidate.metrics.selected_multistart_yaw_deg;
      const double yaw_distance =
          std::isfinite(selected_yaw) && std::isfinite(yaw)
              ? circularYawDistanceDeg(selected_yaw, yaw)
              : 0.0;
      const bool separated =
          finalist.result_index != selected &&
          yaw_distance > kFinalistSeparationAngleDeg;
      const bool viable =
          candidate.internal_gate_pass &&
          candidate.metrics.absolute_support_pass &&
          finalist.training_pass_ratio >= minimum_scene_pass_ratio;
      holdout_records.push_back(
          {finalist, candidate_metrics, validation, {}, pass_ratio, yaw,
           yaw_distance, separated, viable});
    }

    std::size_t holdout_reference_visible_edges = 0;
    std::size_t holdout_reference_nid_points = 0;
    std::size_t holdout_reference_edge_cells = 0;
    for (const auto &record : holdout_records) {
      std::size_t visible_edges = 0;
      std::size_t nid_points = 0;
      std::size_t edge_cells = 0;
      for (const auto &scene : record.metrics) {
        visible_edges += scene.visible_edge_points;
        nid_points += scene.nid_projected_points;
        edge_cells += scene.edge_active_spatial_cells;
      }
      holdout_reference_visible_edges =
          std::max(holdout_reference_visible_edges, visible_edges);
      holdout_reference_nid_points =
          std::max(holdout_reference_nid_points, nid_points);
      holdout_reference_edge_cells =
          std::max(holdout_reference_edge_cells, edge_cells);
    }
    for (auto &record : holdout_records) {
      const auto &candidate = results[record.finalist.result_index];
      const auto &candidate_transform =
          candidate.success ? candidate.estimated_t_camera_lidar
                            : candidate.candidate_t_camera_lidar;
      record.objective = auto_calib::summarizeCalibrationPoseScenes(
          record.metrics, candidate_transform, config,
          holdout_reference_visible_edges, holdout_reference_nid_points,
          holdout_reference_edge_cells);
      if (record.finalist.result_index == selected)
        selected_holdout_objective = record.objective.composite_objective;
    }
    const auto objective_margin_from_selected = [&](double competitor) {
      if (!std::isfinite(selected_holdout_objective) ||
          !std::isfinite(competitor))
        return std::isfinite(selected_holdout_objective) ? 1.0 : -1.0;
      return (competitor - selected_holdout_objective) /
             std::max(std::abs(competitor), 1e-12);
    };
    for (const auto &record : holdout_records) {
      const auto &candidate = results[record.finalist.result_index];
      const bool passes_as_well_as_selected =
          selected_holdout_pass_ratio >= 1.0 && !record.metrics.empty() &&
          record.pass_ratio + 1e-12 >= selected_holdout_pass_ratio;
      const double objective_margin =
          record.finalist.result_index == selected
              ? 0.0
              : objective_margin_from_selected(
                    record.objective.composite_objective);
      const bool objective_margin_sufficient =
          record.finalist.result_index == selected ||
          objective_margin >= config.minimum_multistart_objective_margin;
      const bool ambiguous_competitor =
          record.separated && record.viable && passes_as_well_as_selected &&
          !objective_margin_sufficient;
      if (record.separated && record.viable && passes_as_well_as_selected) {
        ++passing_separated_holdout_competitors;
        minimum_separated_holdout_objective_margin =
            std::min(minimum_separated_holdout_objective_margin,
                     objective_margin);
      }
      if (ambiguous_competitor) {
        finalist_holdout_distinctive = false;
        ++ambiguous_separated_holdout_competitors;
      }
      finalist_holdout_validation.push_back(
          {{"candidate_index", record.finalist.result_index},
           {"selected", record.finalist.result_index == selected},
           {"yaw_deg", record.yaw},
           {"yaw_distance_from_selected_deg", record.yaw_distance},
           {"separated_from_selected", record.separated},
           {"training_pass_ratio", record.finalist.training_pass_ratio},
           {"internal_gate_pass", candidate.internal_gate_pass},
           {"absolute_support_pass", candidate.metrics.absolute_support_pass},
           {"viable_competitor", record.viable},
           {"holdout_pass_ratio", record.pass_ratio},
           {"holdout_objective", record.objective.composite_objective},
           {"holdout_objective_margin_from_selected", objective_margin},
           {"minimum_required_objective_margin",
            config.minimum_multistart_objective_margin},
           {"objective_margin_sufficient", objective_margin_sufficient},
           {"ambiguous_competitor", ambiguous_competitor},
           {"objective_components",
            {{"edge", record.objective.edge_objective},
             {"geometry_nid", record.objective.geometry_nid_objective},
             {"signal_nmi", record.objective.signal_nmi_objective},
             {"structural", record.objective.structural_objective},
             {"manhattan", record.objective.manhattan_objective},
             {"direction_prior",
              record.objective.direction_prior_objective},
             {"coverage", record.objective.coverage_objective}}},
           {"coverage",
            {{"visible_edges", record.objective.visible_edge_points},
             {"nid_points", record.objective.nid_projected_points},
             {"edge_cells", record.objective.edge_active_spatial_cells},
             {"edge_ratio", record.objective.edge_coverage_ratio},
             {"nid_ratio", record.objective.nid_coverage_ratio},
             {"edge_cell_ratio",
              record.objective.edge_spatial_coverage_ratio}}},
           {"validation", record.validation}});
    }
    if (final_success &&
        training_scene_validation.at("pass_ratio").get<double>() <
            minimum_scene_pass_ratio) {
      final_success = false;
      final_reason = "PER_SCENE_VALIDATION_FAILED";
    } else if (final_success && !holdout_scene_metrics.empty() &&
               holdout_scene_validation.at("pass_ratio").get<double>() <
                   1.0) {
      final_success = false;
      final_reason = "HOLDOUT_VALIDATION_FAILED";
    } else if (final_success && !finalist_holdout_distinctive) {
      final_success = false;
      final_reason = "FINALIST_HOLDOUT_AMBIGUOUS";
    }
    const bool core_internal_gate_pass =
        !diagnostic_only && results[selected].internal_gate_pass;
    const bool training_validation_pass =
        training_scene_validation.at("pass_ratio").get<double>() >=
        minimum_scene_pass_ratio;
    const bool holdout_available = !holdout_scene_metrics.empty();
    const bool holdout_validation_pass =
        !holdout_available ||
        holdout_scene_validation.at("pass_ratio").get<double>() >= 1.0;
    const bool candidate_rt_status = core_internal_gate_pass &&
                                     training_validation_pass &&
                                     holdout_available &&
                                     holdout_validation_pass &&
                                     finalist_holdout_distinctive;
    search_stages.push_back(
        {{"stage", "finalist_holdout_validation"},
         {"candidate_count", staged_finalists.size()},
         {"selected_holdout_pass_ratio", selected_holdout_pass_ratio},
         {"passing_separated_competitors",
          passing_separated_holdout_competitors},
         {"ambiguous_separated_competitors",
          ambiguous_separated_holdout_competitors},
         {"selected_holdout_objective", selected_holdout_objective},
         {"minimum_separated_holdout_objective_margin",
          minimum_separated_holdout_objective_margin},
         {"minimum_required_objective_margin",
          config.minimum_multistart_objective_margin},
         {"common_coverage_reference",
          {{"visible_edges", holdout_reference_visible_edges},
           {"nid_points", holdout_reference_nid_points},
           {"edge_cells", holdout_reference_edge_cells}}},
         {"distinctive", finalist_holdout_distinctive},
         {"selection_policy",
          "pass_ratio_tier_then_training_composite_objective_with_common_"
          "holdout_coverage_reference_and_minimum_margin"}});
    // Product activation needs independent references, repeated runs and
    // fail-safe evidence.  This executable deliberately does not infer that
    // evidence from the calibration input, so approval remains explicit.
    const bool product_approved_rt_status = false;
    const std::string lifecycle_status =
        product_approved_rt_status
            ? "PRODUCT_APPROVED_RT"
            : candidate_rt_status
                ? "CANDIDATE_RT"
                : core_internal_gate_pass && training_validation_pass
                    ? "INTERNAL_GATE_PASS"
                    : diagnostic_only ? "DIAGNOSTIC_ONLY" : "FAIL";
    const std::string visualization_pose_source =
        results[selected].success
            ? "optimized_calibration_rt"
            : "rejected_optimization_candidate";
    if (!debug_output.empty())
      writeDebugArtifacts(observations, priors[selected], display_transform,
                          display_camera, visualization_pose_source, config,
                          debug_output);
    if (installation_pose_available)
      writeDebugArtifacts(observations, installation_transform,
                          installation_transform, display_camera,
                          "mechanical_installation_prior", config,
                          output_dir / "mechanical_prior_debug");
    const bool full_search_requested =
        std::abs(yaw_step_deg - 1.0) < 1e-9 && downward_degrees.size() == 91 &&
        optical_roll_candidates.size() == 1 &&
        focal_scale_candidates.size() == 1 &&
        std::abs(downward_degrees.front()) < 1e-9 &&
        std::abs(downward_degrees.back() - 90.0) < 1e-9;
    nlohmann::json full_search_baseline = {
        {"status", full_search_requested
                       ? "FULL_SEARCH_BASELINE_DIAGNOSTIC_ONLY"
                       : "NOT_A_1_DEGREE_FULL_SEARCH"},
        {"step_size_benchmark_status", full_search_requested
                                           ? "BLOCKED_BY_REFERENCE_UNAVAILABLE"
                                           : "NOT_REQUESTED"},
        {"reason",
         full_search_requested
             ? "130333 has one observation per channel and no ground-truth RT"
             : "yaw/down grid is not 1 degree over 360x90 degrees"},
        {"yaw_step_deg", yaw_step_deg},
        {"down_step_deg", value(args, "--down-step-deg", 15.0)},
        {"orientation_candidates", orientation_score_count},
        {"raw_best_down_deg", raw_best_down},
        {"raw_best_yaw_deg", raw_best_yaw},
        {"raw_best_objective", raw_best_objective},
        {"score_map", orientation_score_path.string()},
        {"corrected_score_map",
         (output_dir / "orientation_corrected_scores.csv").string()},
        {"selection_policy",
         "top3_distinct_yaw_contiguous_basins_then_5deg_then_1deg_then_"
         "up_to_3_ceres_with_scene_validation;_no_fallback"},
        {"corrected_alpha", 0.8},
        {"minimum_structural_direction_groups",
         minimum_structural_direction_groups},
        {"maximum_camera_downward_deg", maximum_camera_downward_deg},
        {"neighbor_sigma_yaw_deg", 5.0},
        {"neighbor_sigma_down_deg", 5.0},
        {"selected_basin_down_deg",
         candidate_downward_degrees[basin.result_index]},
        {"selected_basin_optical_roll_deg",
         candidate_optical_roll_degrees[basin.result_index]},
        {"selected_basin_focal_scale",
         candidate_focal_scales[basin.result_index]},
        {"selected_basin_yaw_deg", basin.yaw_deg},
        {"selected_basin_corrected_score", basin.corrected_score},
        {"basin_proposal_candidate", basin_proposal},
        {"final_selected_candidate", selected},
        {"final_selected_down_deg", candidate_downward_degrees[selected]},
        {"final_selected_optical_roll_deg",
         candidate_optical_roll_degrees[selected]},
        {"final_selected_focal_scale", candidate_focal_scales[selected]},
        {"final_selected_yaw_deg",
         results[selected].metrics.selected_multistart_yaw_deg},
        {"basin_candidate_count", basin.basin_count}};
    full_search_baseline["orientation_layers"] = orientation_layers;
    full_search_baseline["search_strategy"] = search_strategy;
    full_search_baseline["search_stages"] = search_stages;
    std::ofstream(output_dir / "full_search_baseline_result.json")
        << std::setw(2) << full_search_baseline << '\n';
    const fs::path top_candidates_dir = output_dir / "top_candidates";
    fs::create_directories(top_candidates_dir);
    const std::size_t top_count = std::min<std::size_t>(5, ranking.size());
    for (std::size_t rank = 0; rank < top_count; ++rank) {
      const std::size_t index = ranking[rank];
      const auto candidate_transform =
          results[index].success ? results[index].estimated_t_camera_lidar
                                 : results[index].candidate_t_camera_lidar;
      auto candidate_observation = observations.front();
      candidate_observation.camera = results[index].success
                                         ? results[index].estimated_camera
                                         : results[index].candidate_camera;
      const fs::path path =
          top_candidates_dir / ("rank_" + std::to_string(rank + 1) + ".png");
      writeMatchingVisualization(
          candidate_observation, candidate_transform, config, path,
          "DIAGNOSTIC rank " + std::to_string(rank + 1) +
              " | down=" + std::to_string(candidate_downward_degrees[index]) +
              " deg | roll=" +
              std::to_string(candidate_optical_roll_degrees[index]) +
              " deg | focal=" +
              std::to_string(candidate_focal_scales[index]) + " | yaw=" +
              std::to_string(
                  results[index].metrics.selected_multistart_yaw_deg) +
              " deg");
      candidates[index]["rank"] = rank + 1;
      candidates[index]["top_candidate_visualization"] = path.string();
    }
    for (std::size_t i = 0; i < observations.size(); ++i) {
      const fs::path visualization =
          output_dir / ("matching_scene_" + std::to_string(i) + ".png");
      const std::string pose_label =
          final_success ? "PASS: optimized RT"
                        : "REJECTED CANDIDATE: RT not active";
      auto output_observation = observations[i];
      output_observation.camera = display_camera;
      const std::size_t projected =
          writeMatchingVisualization(output_observation, display_transform,
                                     config, visualization, pose_label);
      pairs[i]["matching_visualization"] = visualization.string();
      pairs[i]["visualized_projected_points"] = projected;
      const fs::path cloud_stem =
          output_dir / ("scene_" + std::to_string(i) + "_colorized_lidar");
      const std::size_t colored = writeColorizedPointCloud(
          output_observation, display_transform, cloud_stem, pose_label);
      pairs[i]["colorized_ply"] = cloud_stem.string() + ".ply";
      pairs[i]["colorized_obj"] = cloud_stem.string() + ".obj";
      pairs[i]["colorized_z_up_ply"] = cloud_stem.string() + "_z_up.ply";
      pairs[i]["colorized_z_up_obj"] = cloud_stem.string() + "_z_up.obj";
      pairs[i]["colorized_z_up_reprojection_m_ply"] =
          cloud_stem.string() + "_z_up_reprojection_m.ply";
      pairs[i]["vscode_viewer_mesh_ply"] =
          cloud_stem.string() + "_viewer_mesh.ply";
      pairs[i]["vscode_viewer_mesh_obj"] =
          cloud_stem.string() + "_viewer_mesh.obj";
      pairs[i]["vscode_viewer_z_up_mesh_ply"] =
          cloud_stem.string() + "_z_up_viewer_mesh.ply";
      pairs[i]["vscode_viewer_z_up_mesh_obj"] =
          cloud_stem.string() + "_z_up_viewer_mesh.obj";
      pairs[i]["pointcloud_3d_preview"] =
          cloud_stem.string() + "_3d_preview.png";
      pairs[i]["pointcloud_3d_preview_frame"] =
          "viewer_z_up: +x right, +y forward, +z up";
      pairs[i]["camera_colored_points"] = colored;
      pairs[i]["visualization_pose_source"] = visualization_pose_source;
      if (installation_pose_available) {
        const fs::path prior_visualization =
            output_dir /
            ("mechanical_prior_matching_scene_" + std::to_string(i) + ".png");
        const std::size_t prior_projected = writeMatchingVisualization(
            output_observation, installation_transform, config,
            prior_visualization, "MECHANICAL INSTALLATION PRIOR");
        const fs::path prior_cloud_stem =
            output_dir /
            ("mechanical_prior_scene_" + std::to_string(i) +
             "_colorized_lidar");
        const std::size_t prior_colored = writeColorizedPointCloud(
            output_observation, installation_transform, prior_cloud_stem,
            "MECHANICAL INSTALLATION PRIOR");
        pairs[i]["mechanical_prior_matching_visualization"] =
            prior_visualization.string();
        pairs[i]["mechanical_prior_projected_points"] = prior_projected;
        pairs[i]["mechanical_prior_colorized_points"] = prior_colored;
        pairs[i]["mechanical_prior_colorized_z_up_reprojection_m_ply"] =
            prior_cloud_stem.string() + "_z_up_reprojection_m.ply";
        pairs[i]["mechanical_prior_pointcloud_3d_preview"] =
            prior_cloud_stem.string() + "_3d_preview.png";
      }
      if (manual_reference) {
        const fs::path reference_visualization =
            output_dir /
            ("manual_reference_matching_scene_" + std::to_string(i) +
             ".png");
        const std::size_t reference_projected = writeMatchingVisualization(
            output_observation, manual_reference_transform, config,
            reference_visualization, "MANUAL RT REFERENCE (DIAGNOSTIC)");
        const fs::path reference_cloud_stem =
            output_dir /
            ("manual_reference_scene_" + std::to_string(i) +
             "_colorized_lidar");
        const std::size_t reference_colored = writeColorizedPointCloud(
            output_observation, manual_reference_transform,
            reference_cloud_stem, "MANUAL RT REFERENCE (DIAGNOSTIC)");
        pairs[i]["manual_reference_matching_visualization"] =
            reference_visualization.string();
        pairs[i]["manual_reference_projected_points"] = reference_projected;
        pairs[i]["manual_reference_colorized_points"] = reference_colored;
        pairs[i]["manual_reference_colorized_z_up_reprojection_m_ply"] =
            reference_cloud_stem.string() + "_z_up_reprojection_m.ply";
        pairs[i]["manual_reference_pointcloud_3d_preview"] =
            reference_cloud_stem.string() + "_3d_preview.png";
      }
      const Eigen::Vector3d camera_center =
          -display_transform.rotation.transpose() *
          display_transform.translation_m;
      const Eigen::Vector3d camera_forward =
          display_transform.rotation.transpose() * Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d viewer_center = viewerZUp(camera_center);
      const Eigen::Vector3d viewer_forward = viewerZUp(camera_forward);
      pairs[i]["diagnostic_camera_center_lidar_m"] = {
          camera_center.x(), camera_center.y(), camera_center.z()};
      pairs[i]["diagnostic_camera_optical_axis_lidar"] = {
          camera_forward.x(), camera_forward.y(), camera_forward.z()};
      pairs[i]["diagnostic_camera_center_viewer_z_up_m"] = {
          viewer_center.x(), viewer_center.y(), viewer_center.z()};
      pairs[i]["diagnostic_camera_optical_axis_viewer_z_up"] = {
          viewer_forward.x(), viewer_forward.y(), viewer_forward.z()};
      pairs[i]["diagnostic_camera_heading_viewer_z_up_deg"] =
          std::atan2(viewer_forward.y(), viewer_forward.x()) * 180.0 / kPi;
    }
    const nlohmann::json signal_nmi_conformance =
        manual_reference
            ? writeSignalNmiConformance(
                  observations, manual_reference_transform, config,
                  output_dir / "reference_rt_perturbation.csv")
            : nlohmann::json(nullptr);
    nlohmann::json report = {
        {"status", lifecycle_status},
        {"internal_gate_status",
         core_internal_gate_pass ? "INTERNAL_GATE_PASS" : "INTERNAL_GATE_FAIL"},
        {"candidate_rt_status", candidate_rt_status ? "CANDIDATE_RT" : "NOT_CANDIDATE_RT"},
        {"finalist_confidence_margin", results[selected].metrics.finalist_confidence_margin},
        {"finalist_objective_margin", finalist_objective_margin},
        {"absolute_support_pass", results[selected].metrics.absolute_support_pass},
        {"global_max_visible_edges", global_max_visible_edges},
        {"global_max_nid_points", global_max_nid_points},
        {"product_approved_rt_status",
         product_approved_rt_status ? "PRODUCT_APPROVED_RT"
                                     : "NOT_PRODUCT_APPROVED_RT"},
        {"activation_allowed", product_approved_rt_status},
        {"training_validation_pass", training_validation_pass},
        {"holdout_validation_pass", holdout_validation_pass},
        {"finalist_holdout_distinctive", finalist_holdout_distinctive},
        {"passing_separated_holdout_competitors",
         passing_separated_holdout_competitors},
        {"ambiguous_separated_holdout_competitors",
         ambiguous_separated_holdout_competitors},
        {"selected_holdout_objective", selected_holdout_objective},
        {"minimum_separated_holdout_objective_margin",
         minimum_separated_holdout_objective_margin},
        {"independent_product_approval_evidence",
         "required_outside_this_single_calibration_run"},
        {"reason_code", final_reason},
        {"candidate_gate_reason_code", results[selected].reason_code},
        {"mode", diagnostic_only
                     ? (manufacturer_fov_diagnostic
                            ? "manufacturer_fov_pose_diagnostic"
                            : profile_distortion_unknown
                            ? "fixed_K_unknown_distortion_diagnostic"
                            : "single_observation_fixed_K_pose_diagnostic")
                 : manual_intrinsic && !config.optimize_camera_intrinsics
                     ? "real_geometry_nid_edge_multistart_manual_intrinsics_fixed"
                 : enable_experimental_joint_intrinsic
                     ? "experimental_joint_intrinsics"
                     : "diagnostic_manufacturer_fov_fixed"},
        {"intrinsics_source",
         manual_intrinsic
             ? (config.optimize_camera_intrinsics
                    ? "manual_intrinsic_refined"
                    : "manual_intrinsic_fixed")
         : !config.optimize_camera_intrinsics
             ? "manufacturer_fov_fixed_for_pose_diagnostic"
         : results[selected].success
             ? "jointly_estimated_from_manufacturer_fov_initialization"
             : "joint_intrinsic_candidate_not_activated"},
        {"intrinsic_profile_id", camera.profile_id},
        {"manual_intrinsic_json",
         manual_intrinsic ? nlohmann::json(manual_intrinsic_path.string())
                          : nlohmann::json(nullptr)},
        {"manual_reference_json",
         manual_reference ? nlohmann::json(manual_reference_path.string())
                           : nlohmann::json(nullptr)},
        {"manual_distortion_model",
         manual_intrinsic ? nlohmann::json(camera.distortion_model)
                          : nlohmann::json(nullptr)},
        {"manual_distortion_coefficients",
         manual_intrinsic ? nlohmann::json(camera.distortion)
                          : nlohmann::json(nullptr)},
        {"image_distortion_state", image_distortion_state},
        {"profile_distortion_unknown", profile_distortion_unknown},
        {"manufacturer_fov_diagnostic", manufacturer_fov_diagnostic},
        {"distortion_application",
         manual_intrinsic && image_distortion_state == "raw" &&
                 !camera.distortion.empty()
             ? "manual_intrinsic_undistort"
         : manual_intrinsic && image_distortion_state == "rectified"
             ? "camera_or_operator_rectified_output"
             : manual_intrinsic && image_distortion_state == "unknown"
                 ? "not_applied_unknown_state"
                 : "not_available"},
        {"allow_intrinsic_refinement", allow_intrinsic_refinement},
        {"focal_scale", candidate_focal_scales[selected]},
        {"focal_scale_candidates", focal_scale_candidates},
        {"principal_y_offset_px", principal_y_offset_px},
        {"initial_intrinsics", cameraJson(camera.camera)},
        {"candidate_intrinsics",
         cameraJson(results[selected].candidate_camera)},
        {"active_intrinsics", cameraJson(display_camera)},
        {"manufacturer_fov_range_deg",
         {{"horizontal",
           {camera.horizontal_fov_min_deg, camera.horizontal_fov_max_deg}},
          {"vertical",
           {camera.vertical_fov_min_deg, camera.vertical_fov_max_deg}}}},
        {"camera_channel", camera_channel},
        {"ldc_enabled", ldc_enabled},
        {"zoom_focus_locked_during_capture", zoom_focus_locked},
        {"minimum_intrinsic_observations",
         config.minimum_intrinsic_observations},
        {"pairing_basis", "lexicographic filename order supplied by operator"},
        {"algorithm",
         {{"camera_feature", "gradient_magnitude"},
          {"lidar_feature", "range_discontinuity_and_surface_normal_change"},
          {"refinement",
           "fixed_visible_set_soft_histogram_NID_plus_edge_distance"},
          {"candidate_visibility", "quarter_resolution_z_buffer_10mm"},
          {"structural_line_feature",
           "LSD_2D_segments_plus_plane_intersections_plane_boundaries_and_"
           "persistent_occlusions"},
          {"plane_postprocessing",
           "neighbor_reassignment_plus_adjacent_coplanar_merge_plus_IMU_Y_"
           "height_clustering"},
          {"horizontal_plane_axis", "+Y_down_from_IMU_gated_lidar_frame"},
          {"occlusion_edge_policy",
           "single_scan_candidates_debug_only_repeated_segments_calibration"},
          {"lidar_edge_policy",
           "absolute_relative_threshold_plus_local_contrast_and_coplanar_"
           "rejection"},
          {"lidar_edge_minimum_local_contrast_ratio",
           config.lidar_edge_minimum_local_contrast_ratio},
          {"plane_normal_threshold_deg",
           config.lidar_plane_normal_threshold_rad * 180.0 / kPi},
          {"minimum_plane_points", config.minimum_lidar_plane_points},
          {"maximum_plane_rms_error_m",
           config.maximum_lidar_plane_rms_error_m},
          {"plane_pair_neighbor_radius_cells",
           config.plane_pair_neighbor_radius_cells},
          {"maximum_plane_intersection_boundary_distance_m",
           config.maximum_plane_intersection_boundary_distance_m},
          {"structural_line_residual",
           "one_to_one_direction_endpoint_distance_finite_overlap"},
          {"structural_direction_weight",
           config.structural_direction_weight},
          {"structural_endpoint_weight", config.structural_endpoint_weight},
          {"structural_overlap_weight", config.structural_overlap_weight},
          {"coarse_overlap_gate",
           "visible_edge>=100, nid_projected>=100 per observation, and "
           "configured structural direction support"},
          {"relative_coverage_policy",
           "edge support is a soft penalty; NID relative support is a hard "
           "gate only inside each staged local yaw window and a soft global "
           "finalist score; spatial support remains a hard gate"},
          {"minimum_relative_nid_coverage",
           config.minimum_relative_nid_coverage},
          {"minimum_relative_edge_spatial_coverage",
           config.minimum_relative_edge_spatial_coverage},
          {"coverage_penalty_weight", config.coverage_penalty_weight},
          {"coverage_grid",
           {config.coverage_grid_rows, config.coverage_grid_columns}},
          {"flat_geometry_rejection", "structural_score<0.05"},
          {"geometry_nid_channels",
           "range_discontinuity_and_surface_normal_change_scored_separately"},
          {"geometry_nid_spatial_grid",
           {config.nid_spatial_rows, config.nid_spatial_columns}},
          {"minimum_nid_tile_points", config.minimum_nid_tile_points},
          {"minimum_nid_active_spatial_cells",
           config.minimum_nid_active_spatial_cells},
          {"minimum_nid_feature_entropy_ratio",
           config.minimum_nid_feature_entropy_ratio},
          {"nid_weight", config.normalized_information_distance_weight},
          {"signal_nmi_policy",
           config.signal_nmi_weight > 0.0
               ? "enabled_after_manual_RT_perturbation_conformance"
               : "diagnostic_only_until_manual_RT_perturbation_conformance"},
          {"signal_nmi_feature",
           "log_signal_plus_range_squared_minus_incidence_then_range_bin_"
           "median_MAD_normalization"},
          {"signal_nmi_weight", config.signal_nmi_weight},
          {"edge_weight", config.edge_alignment_weight},
          {"structural_line_weight", config.structural_line_weight},
          {"manhattan_direction_feature",
           "LSD_vanishing_directions_plus_lidar_gravity_and_wall_axes"},
          {"manhattan_image_feature_prior_policy",
           "fixed_per_finalist_training_seed_prior_reused_for_training_and_"
           "holdout"},
          {"maximum_manhattan_vanishing_directions",
           config.maximum_manhattan_vanishing_directions},
          {"manhattan_direction_weight",
           config.manhattan_direction_weight},
          {"minimum_manhattan_vertical_inliers",
           config.minimum_manhattan_vertical_inliers},
          {"maximum_manhattan_vertical_error_deg",
           maximum_manhattan_vertical_error_deg},
          {"minimum_structural_direction_groups",
           minimum_structural_direction_groups},
          {"maximum_camera_downward_deg", maximum_camera_downward_deg},
          {"direction_prior_weight", config.camera_direction_prior_weight},
          {"expected_camera_forward_lidar",
           {config.expected_camera_forward_lidar.x(),
            config.expected_camera_forward_lidar.y(),
            config.expected_camera_forward_lidar.z()}},
          {"expected_camera_down_lidar",
           {config.expected_camera_down_lidar.x(),
            config.expected_camera_down_lidar.y(),
            config.expected_camera_down_lidar.z()}},
          {"camera_center_prior_sigma_m",
           config.use_camera_center_prior
               ? nlohmann::json(config.camera_center_prior_sigma_m)
               : nlohmann::json(nullptr)},
          {"yaw_multistart_step_deg", yaw_step_deg},
          {"yaw_search_min_deg",
           config.use_coarse_yaw_bounds
               ? nlohmann::json(value(args, "--yaw-min-deg", -180.0))
               : nlohmann::json(-180.0)},
          {"yaw_search_max_deg",
           config.use_coarse_yaw_bounds
               ? nlohmann::json(value(args, "--yaw-max-deg", 180.0))
               : nlohmann::json(180.0 - yaw_step_deg)},
          {"yaw_neighbor_topology", circular_yaw ? "circular" : "bounded"},
          {"ambiguity_margin", config.minimum_multistart_objective_margin},
          {"minimum_finalist_confidence_margin",
           config.minimum_finalist_confidence_margin},
          {"finalist_objective_margin", finalist_objective_margin},
          {"finalist_confidence_margin",
           results[selected].metrics.finalist_confidence_margin}}},
        {"pairs", pairs},
        {"input_pair_start", pair_start},
        {"input_pair_count", pair_count},
        {"training_observation_count", calibration_observations.size()},
        {"holdout_observation_count", holdout_observations.size()},
        {"minimum_scene_pass_ratio", minimum_scene_pass_ratio},
        {"minimum_scene_direction_groups",
         minimum_scene_direction_groups},
        {"training_scene_validation", training_scene_validation},
        {"holdout_scene_validation", holdout_scene_validation},
        {"finalist_holdout_validation", finalist_holdout_validation},
        {"baseline_m", explicit_camera_center ? nlohmann::json(nullptr)
                                              : nlohmann::json(baseline)},
        {"camera_center_lidar_m",
         explicit_camera_center
             ? nlohmann::json::array({value(args, "--camera-center-x-m", 0.0),
                                      value(args, "--camera-center-y-m", 0.0),
                                      value(args, "--camera-center-z-m", 0.0)})
             : nlohmann::json(nullptr)},
        {"downward_direction_candidates_deg", downward_degrees},
        {"selected_downward_direction_deg",
         candidate_downward_degrees[selected]},
        {"optical_roll_candidates_deg", optical_roll_candidates},
        {"selected_optical_roll_deg",
         candidate_optical_roll_degrees[selected]},
        {"prior_heading_image_roll_deg",
         {value(args, "--prior-pitch-deg", 0.0),
          value(args, "--prior-yaw-deg", 0.0)}},
        {"search_strategy", search_strategy},
        {"search_stages", search_stages},
        {"ceres_execution_policy",
         staged_search ? "up_to_3_distinct_yaw_finalists"
                       : "legacy_every_candidate"},
        {"joint_intrinsic_policy",
         enable_experimental_joint_intrinsic
             ? "experimental_explicit_opt_in"
             : "disabled_product_path"},
        {"camera_profile_policy",
         manual_intrinsic ? "manual_charuco_KD_fixed" : "manufacturer_fov_diagnostic_only"},
        {"candidate_results", candidates},
        {"selected_candidate", selected},
        {"basin_proposal_candidate", basin_proposal},
        {"selected_candidate_policy",
         staged_search
             ? "top3_distinct_yaw_contiguous_basins_then_5deg_then_1deg_"
               "then_up_to_3_ceres_significant_objective_else_TESL_training_"
               "scene_selection; no_fallback"
             : "legacy_contiguous_basin; no_fallback"},
        {"selected_candidate_structural_direction_groups",
         structuralDirectionGroups(results[selected])},
        {"selected_candidate_camera_downward_deg",
         cameraDownwardDeg(results[selected].candidate_t_camera_lidar)},
        {"debug_output", debug_output.empty() ? nlohmann::json(nullptr)
                                              : nlohmann::json(debug_output)},
        {"estimated_t_camera_lidar",
         transformJson(final_success
                           ? results[selected].estimated_t_camera_lidar
                           : priors[selected])},
        {"diagnostic_candidate_t_camera_lidar",
         transformJson(results[selected].candidate_t_camera_lidar)},
        {"visualization_t_camera_lidar", transformJson(display_transform)},
        {"visualization_pose_source", visualization_pose_source},
        {"manual_reference_comparison",
         manual_reference
             ? nlohmann::json(
                   {{"reference_status", "diagnostic_only"},
                    {"compared_pose_source", visualization_pose_source},
                    {"rotation_geodesic_deg",
                     auto_calib::calculatePoseError(
                         display_transform, manual_reference_transform)
                         .rotation_deg},
                    {"translation_norm_m",
                     auto_calib::calculatePoseError(
                         display_transform, manual_reference_transform)
                         .translation_m},
                    {"warning",
                     "Manual reference is diagnostic unless independently "
                     "measured and quality-approved."}})
             : nlohmann::json(nullptr)},
        {"signal_nmi_manual_rt_perturbation", signal_nmi_conformance},
        {"reference_rt_perturbation", signal_nmi_conformance},
        {"mechanical_installation_prior_t_camera_lidar",
         installation_pose_available
             ? nlohmann::json(transformJson(installation_transform))
             : nlohmann::json(nullptr)},
        {"diagnostic_only", diagnostic_only},
        {"full_search_baseline", full_search_baseline},
        {"pan_direction_top_view", "increasing pan is clockwise (measured)"},
        {"occlusion_handling", "z_buffer_nearest_surface_10mm"},
        {"acquisition_condition",
         "operator_confirmed_same_time_same_pose_static_scene"},
        {"limitations",
         {manual_intrinsic
              ? "Manual camera intrinsic is used as the active fixed profile."
              : "Manufacturer FOV initialization is diagnostic-only and cannot "
                "produce a product RT.",
          "K+D are fixed from the supplied camera profile; joint K+RT is "
          "experimental and disabled unless explicitly enabled.",
          "Lens distortion is not estimated; record and fix the camera LDC "
          "state during capture.",
          profile_distortion_unknown
              ? "LDC/raw-versus-rectified state is unknown; this result is "
                "diagnostic-only."
              : "LDC/raw-versus-rectified state is explicit.",
          config.signal_nmi_weight > 0.0
              ? "Corrected signal_strength NMI is active; verify the manual "
                "RT perturbation report before accepting its weight."
              : "Corrected signal_strength NMI is diagnostic-only until the "
                "manual RT perturbation report passes.",
          "INTERNAL_GATE_PASS is not activation; CANDIDATE_RT requires a "
          "holdout and PRODUCT_APPROVED_RT requires independent evidence."}}};
    std::ofstream(output_dir / "calibration_result.json")
        << std::setw(2) << report << '\n';
    if (installation_pose_available) {
      const Eigen::Vector3d constrained_center =
          -installation_transform.rotation.transpose() *
          installation_transform.translation_m;
      const Eigen::Vector3d constrained_forward =
          installation_transform.rotation.transpose() * Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d constrained_down =
          installation_transform.rotation.transpose() * Eigen::Vector3d::UnitY();
      const nlohmann::json constrained_rt = {
          {"status", "INPUT_CONTRACT_NOT_CALIBRATED"},
          {"reason", final_reason},
          {"t_camera_lidar", transformJson(installation_transform)},
          {"camera_center_lidar_m",
           {constrained_center.x(), constrained_center.y(),
            constrained_center.z()}},
          {"camera_forward_lidar",
           {constrained_forward.x(), constrained_forward.y(),
            constrained_forward.z()}},
          {"camera_down_lidar",
           {constrained_down.x(), constrained_down.y(), constrained_down.z()}},
          {"source", "mechanical_installation_prior"}};
      std::ofstream(output_dir / "installation_constrained_rt.json")
          << std::setw(2) << constrained_rt << '\n';
    }
    std::cout << std::setw(2) << report << '\n';
    return final_success ? 0 : 3;
  } catch (const std::exception &error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}

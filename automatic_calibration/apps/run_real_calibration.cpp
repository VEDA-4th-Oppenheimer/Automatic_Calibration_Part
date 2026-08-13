#include "auto_calib/calibration_core.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;
namespace {
constexpr double kPi = 3.14159265358979323846;
double radians(double degrees) { return degrees * kPi / 180.0; }
using Args = std::unordered_map<std::string, std::string>;
struct CameraCalibration {
  auto_calib::CameraModel camera;
  cv::Size resolution;
  std::string profile_id;
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
  std::sort(out.begin(), out.end());
  return out;
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
cv::Mat loadImage(const fs::path &path, const CameraCalibration &calibration,
                  auto_calib::CameraModel *camera) {
  cv::Mat image = cv::imread(path.string(), cv::IMREAD_COLOR);
  if (image.empty())
    throw std::runtime_error("Cannot read image: " + path.string());
  if (image.size() != calibration.resolution)
    throw std::runtime_error("All camera images must have one resolution");
  *camera = calibration.camera;
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
    const bool installed =
        status.rfind("INSTALLATION-CONSTRAINED", 0) == 0;
    const cv::Scalar camera_color =
        accepted ? cv::Scalar(0, 180, 0)
                 : (installed ? cv::Scalar(255, 255, 0)
                              : cv::Scalar(255, 0, 255));
    cv::circle(panel, center_pixel, 7, camera_color, cv::FILLED, cv::LINE_AA);
    cv::arrowedLine(panel, center_pixel, tip_pixel, camera_color, 3, cv::LINE_AA,
                    0, 0.18);
    const std::string camera_label =
        accepted ? "CALIBRATED CAM"
                 : (installed ? "INSTALLED CAMERA" : "REJECTED RT");
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
      << "04_lidar_surface_normals.ply: robust organized-scan normals\n"
      << "04a_lidar_plane_labels.ply: recovered/merged plane support labels\n"
      << "04a_lidar_planes.csv: fitted plane coefficients and quality\n"
      << "04b_lidar_plane_pair_candidates.csv: intersection rejection stage\n"
      << "04b_lidar_plane_intersection_edges.{ply,obj}: intersections of "
         "adjacent fitted planes\n"
      << "04c_lidar_occlusion_edges.{ply,obj}: range-discontinuity "
         "silhouettes (diagnostic only)\n"
      << "04d_lidar_edges_used_for_calibration.{ply,obj}: structural edges "
         "used by the objective\n"
      << "05_projection_initial.png: LiDAR points using initial RT prior\n"
      << "06_projection_final.png: LiDAR points using the reported "
         "visualization RT\n"
      << "07_projection_final_edges.png: LiDAR range edges using the "
         "visualization RT\n"
      << "Viewer mesh OBJ/PLY: units m; colorized cloud OBJ/PLY: units mm\n"
      << "debug_summary.csv: per-scene counts and projection coverage\n";
  std::ofstream summary(root / "debug_summary.csv");
  summary << "scene,valid_points,normals,planes,plane_intersection_edges,"
             "occlusion_edges,camera_edges,lidar_edges,initial_projected,"
             "final_projected,final_edges\n";
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
    const auto occlusion_segments =
        auto_calib::extractLidarOcclusionSegments(scan, config);
    writeSegmentMesh(scene / "04b_lidar_plane_intersection_edges",
                     plane_intersections,
                     "adjacent fitted-plane intersections");
    writeSegmentMesh(scene / "04c_lidar_occlusion_edges", occlusion_segments,
                     "range-discontinuity silhouettes, diagnostic only");
    writeSegmentMesh(scene / "04d_lidar_edges_used_for_calibration",
                     plane_intersections,
                     "plane-intersection edges used by calibration");
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
    summary << i << ',' << scan.valid_count << ',' << normal_count << ','
            << plane_segmentation.planes.size() << ','
            << plane_intersections.size() << ',' << occlusion_segments.size()
            << ',' << camera_edges << ',' << lidar_edges.size() << ','
            << initial_projected << ',' << final_projected << ',' << final_edges
            << '\n';
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
      {"reason_code", r.reason_code},
      {"estimated", transformJson(r.estimated_t_camera_lidar)},
      {"diagnostic_candidate", transformJson(r.candidate_t_camera_lidar)},
      {"estimated_intrinsics", cameraJson(r.estimated_camera)},
      {"candidate_intrinsics", cameraJson(r.candidate_camera)},
      {"metrics",
       {{"camera_edge_pixels", r.metrics.camera_edge_pixels},
        {"lidar_edge_points", r.metrics.lidar_edge_points},
        {"lidar_geometry_points", r.metrics.lidar_geometry_points},
        {"nid_projected_points", r.metrics.nid_projected_points},
        {"visible_edge_points", r.metrics.visible_edge_points},
        {"occluded_edge_points", r.metrics.occluded_edge_points},
        {"camera_structural_lines", r.metrics.camera_structural_lines},
        {"lidar_planes", r.metrics.lidar_planes},
        {"lidar_structural_segments", r.metrics.lidar_structural_segments},
        {"lidar_occlusion_segments", r.metrics.lidar_occlusion_segments},
        {"structural_projected_points",
         r.metrics.structural_projected_points},
        {"final_horizontal_structural_objective",
         r.metrics.final_horizontal_structural_objective},
        {"final_vertical_structural_objective",
         r.metrics.final_vertical_structural_objective},
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
        {"nid_improvement_ratio", r.metrics.nid_improvement_ratio},
        {"initial_composite_objective", r.metrics.initial_composite_objective},
        {"final_composite_objective", r.metrics.final_composite_objective},
        {"objective_improvement_ratio", r.metrics.objective_improvement_ratio},
        {"solver_iterations", r.metrics.solver_iterations},
        {"runtime_ms", r.metrics.runtime_ms}}},
      {"solver_summary", r.solver_summary}};
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
         "edge_objective,nid_objective,"
         "structural_line_objective,horizontal_structural_objective,"
         "vertical_structural_objective,direction_prior_objective,"
         "edge_in_frame_points,visible_edge_points,"
         "occluded_edge_points,nid_projected_points,"
         "horizontal_structural_segments,vertical_structural_segments,"
         "overlap_valid\n"
      << std::setprecision(12);
  std::size_t count = 0;
  *best_objective = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < results.size(); ++i)
    for (const auto &score : results[i].coarse_orientation_scores) {
      out << downward_degrees[i] << ',' << optical_roll_degrees[i] << ','
          << focal_scales[i] << ',' << score.yaw_offset_deg << ','
          << score.raw_objective << ',' << score.edge_objective << ','
          << score.nid_objective << ',' << score.structural_line_objective
          << ',' << score.horizontal_structural_objective << ','
          << score.vertical_structural_objective << ','
          << score.direction_prior_objective << ','
          << score.edge_in_frame_points << ','
          << score.visible_edge_points << ',' << score.occluded_edge_points
          << ',' << score.nid_projected_points << ','
          << score.horizontal_structural_segments << ','
          << score.vertical_structural_segments << ','
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
    double alpha = 0.5, double sigma_yaw_deg = 5.0,
    double sigma_down_deg = 5.0) {
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
          const auto cc = (static_cast<int>(c) + dc + row.size()) % row.size();
          if (std::isfinite(row[cc].raw_objective)) {
            const double dyaw =
                std::abs(scores[c].yaw_offset_deg - row[cc].yaw_offset_deg);
            const double wrapped_yaw = std::min(dyaw, 360.0 - dyaw);
            const double ddown =
                std::abs(downward_degrees[r] -
                         downward_degrees[static_cast<std::size_t>(rr)]);
            const double weight =
                std::exp(-0.5 * (std::pow(wrapped_yaw / sigma_yaw_deg, 2.0) +
                                 std::pow(ddown / sigma_down_deg, 2.0)));
            weighted_sum += weight * row[cc].raw_objective;
            weight_sum += weight;
          }
        }
      }
      const double neighbor = weight_sum > 0.0 ? weighted_sum / weight_sum
                                               : scores[c].raw_objective;
      corrected[r][c] =
          alpha * scores[c].raw_objective + (1.0 - alpha) * neighbor;
      out << downward_degrees[r] << ',' << scores[c].yaw_offset_deg << ','
          << scores[c].raw_objective << ',' << neighbor << ','
          << corrected[r][c] << '\n';
      if (corrected[r][c] < best.corrected_score) {
        best = {r, c, scores[c].yaw_offset_deg, corrected[r][c], 0};
      }
    }
  }
  if (!std::isfinite(best.corrected_score))
    return best;
  const double threshold = best.corrected_score + 0.02;
  std::vector<std::vector<bool>> visited(corrected.size());
  for (std::size_t r = 0; r < corrected.size(); ++r)
    visited[r].assign(corrected[r].size(), false);
  std::vector<std::pair<std::size_t, std::size_t>> queue{
      {best.result_index, best.column}};
  visited[best.result_index][best.column] = true;
  while (!queue.empty()) {
    const auto [r, c] = queue.back();
    queue.pop_back();
    ++best.basin_count;
    for (int dr = -1; dr <= 1; ++dr)
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0)
          continue;
        const int rr = static_cast<int>(r) + dr;
        if (rr < 0 || rr >= static_cast<int>(corrected.size()))
          continue;
        const auto &row = corrected[static_cast<std::size_t>(rr)];
        if (row.empty())
          continue;
        const auto cc = (static_cast<int>(c) + dc + row.size()) % row.size();
        if (!visited[static_cast<std::size_t>(rr)][cc] &&
            row[cc] <= threshold) {
          visited[static_cast<std::size_t>(rr)][cc] = true;
          queue.push_back({static_cast<std::size_t>(rr), cc});
        }
      }
  }
  return best;
}
void usage() {
  std::cout
      << "run_real_calibration --input-dir PATH --output PATH "
         "[--camera-channel N] [--ldc-enabled true|false|unknown] "
         "[--zoom-focus-locked true|false|unknown] "
         "[--baseline-m 0.28] [--minimum-range-m 0.30] "
         "[--camera-center-x-m X --camera-center-y-m Y "
         "--camera-center-z-m Z] [--prior-roll-deg DEG | "
         "--down-min-deg 0 --down-max-deg 90 --down-step-deg 15] "
         "[--optical-roll-min-deg -15 --optical-roll-max-deg 15 "
         "--optical-roll-step-deg 5] "
         "[--yaw-step-deg 15] [--yaw-min-deg -180 --yaw-max-deg -160] "
         "[--focal-scale 1.0 | --focal-scale-min 0.9 "
         "--focal-scale-max 1.1 --focal-scale-step 0.1] "
         "[--principal-y-offset-px 0] "
         "[--nid-weight 0.55 --edge-weight 0.25 --line-weight 0.20] "
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
    const auto images = files(input_dir, {".png", ".jpg", ".jpeg"});
    const auto scans = files(input_dir, {".json"});
    if (images.size() != scans.size() || images.empty())
      throw std::runtime_error(
          "Expected at least one sorted one-to-one image/JSON pair");
    const cv::Mat first_image =
        cv::imread(images.front().string(), cv::IMREAD_COLOR);
    if (first_image.empty())
      throw std::runtime_error("Cannot read image: " + images.front().string());
    auto camera = cameraFromManufacturerFov(first_image.size());
    const double focal_scale = value(args, "--focal-scale", 1.0);
    const auto focal_scale_candidates = numericRange(
        args, "--focal-scale-min", "--focal-scale-max",
        "--focal-scale-step", focal_scale);
    const double principal_y_offset_px =
        value(args, "--principal-y-offset-px", 0.0);
    if (focal_scale < 0.80 || focal_scale > 1.20 ||
        std::any_of(focal_scale_candidates.begin(),
                    focal_scale_candidates.end(),
                    [](double scale) { return scale < 0.80 || scale > 1.20; }) ||
        std::abs(principal_y_offset_px) > 0.10 * first_image.rows)
      throw std::invalid_argument(
          "K profile must stay within focal scale [0.8,1.2] and cy +/-10%");
    camera.camera.k(0, 0) *= focal_scale;
    camera.camera.k(1, 1) *= focal_scale;
    camera.camera.k(1, 2) += principal_y_offset_px;
    camera.profile_id += "-focal-scale-" + std::to_string(focal_scale) +
                         "-cy-offset-" +
                         std::to_string(principal_y_offset_px);
    const int camera_channel =
        static_cast<int>(value(args, "--camera-channel", 0.0));
    const std::string ldc_enabled = textValue(args, "--ldc-enabled", "unknown");
    if (ldc_enabled != "true" && ldc_enabled != "false" &&
        ldc_enabled != "unknown")
      throw std::invalid_argument(
          "--ldc-enabled must be true, false, or unknown");
    const std::string zoom_focus_locked =
        textValue(args, "--zoom-focus-locked", "unknown");
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
      cv::Mat image = loadImage(images[i], camera, &scene_camera);
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
                       {"preprocessing", loaded.statistics}});
      observations.push_back(
          {std::move(image), scene_camera, std::move(loaded.scan)});
    }
    auto_calib::CalibrationConfig config;
    config.minimum_lidar_edge_points = 30;
    config.minimum_nid_projected_points = 100;
    config.maximum_nid_points = 5000;
    config.nid_histogram_bins = 16;
    config.normalized_information_distance_weight =
        value(args, "--nid-weight", 0.55);
    config.edge_alignment_weight = value(args, "--edge-weight", 0.25);
    config.structural_line_weight = value(args, "--line-weight", 0.20);
    config.minimum_projected_structural_segments = 1;
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
    if (config.normalized_information_distance_weight < 0.0 ||
        config.edge_alignment_weight < 0.0 ||
        config.structural_line_weight < 0.0 ||
        config.normalized_information_distance_weight +
                config.edge_alignment_weight + config.structural_line_weight <=
            0.0)
      throw std::runtime_error("Invalid NID/edge/line objective weights");
    config.minimum_projected_ratio = 0.20;
    config.maximum_mean_edge_distance_px = 40.0;
    config.minimum_objective_improvement_ratio = 0.05;
    config.minimum_nid_improvement_ratio = 0.01;
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
    config.rotation_search_bound_rad = radians(10.0);
    config.translation_search_bound_m = 0.10;
    config.maximum_rotation_update_rad = kPi;
    config.maximum_translation_update_m = 0.10;
    config.rotation_prior_sigma_rad = 1.0;
    config.translation_prior_sigma_m = 0.03;
    config.prior_weight = 0.10;
    config.minimum_intrinsic_observations = 3;
    const bool diagnostic_only =
        observations.size() < config.minimum_intrinsic_observations;
    config.optimize_camera_intrinsics = !diagnostic_only;
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
    for (double sign : signs) {
      for (double candidate_focal_scale : focal_scale_candidates) {
        auto candidate_observations = observations;
        const double focal_ratio = candidate_focal_scale / focal_scale;
        for (auto &observation : candidate_observations) {
          observation.camera.k(0, 0) *= focal_ratio;
          observation.camera.k(1, 1) *= focal_ratio;
        }
        for (double optical_roll_deg : optical_roll_candidates) {
          for (double downward_deg : downward_degrees) {
        const double prior_x = sign * baseline;
        auto prior =
            auto_calib::makeTransform({prior_x, 0.0, 0.0}, {0.0, 0.0, 0.0});
        prior.rotation =
            Eigen::AngleAxisd(radians(optical_roll_deg),
                              Eigen::Vector3d::UnitZ())
                .toRotationMatrix() *
            Eigen::AngleAxisd(radians(downward_deg), Eigen::Vector3d::UnitX())
                .toRotationMatrix() *
            Eigen::AngleAxisd(-heading_rad, Eigen::Vector3d::UnitY())
                .toRotationMatrix();
        if (explicit_camera_center) {
          const Eigen::Vector3d camera_center{
              value(args, "--camera-center-x-m", 0.0),
              value(args, "--camera-center-y-m", 0.0),
              value(args, "--camera-center-z-m", 0.0)};
          prior.translation_m = -prior.rotation * camera_center;
        }
        priors.push_back(prior);
        candidate_downward_degrees.push_back(downward_deg);
        candidate_optical_roll_degrees.push_back(optical_roll_deg);
        candidate_focal_scales.push_back(candidate_focal_scale);
        results.push_back(auto_calib::calibrateExtrinsicMultiScene(
            candidate_observations, prior, config));
        candidates.push_back(resultJson(results.back(), prior.translation_m.x(),
                                        downward_deg, optical_roll_deg,
                                        candidate_focal_scale));
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
                return score(results[a]) < score(results[b]);
              });
    std::size_t selected = ranking.front();
    bool final_success = !diagnostic_only && results[selected].success;
    std::string final_reason = diagnostic_only
                                   ? "SINGLE_OBSERVATION_DIAGNOSTIC_ONLY"
                                   : results[selected].reason_code;
    auto display_transform = results[selected].success
                                 ? results[selected].estimated_t_camera_lidar
                                 : results[selected].candidate_t_camera_lidar;
    auto display_camera = results[selected].success
                              ? results[selected].estimated_camera
                              : results[selected].candidate_camera;
    const std::string debug_output = textValue(args, "--debug-output", "");
    fs::create_directories(output_dir);
    const fs::path orientation_score_path =
        output_dir / "orientation_full_search.csv";
    double raw_best_down = 0.0, raw_best_yaw = 0.0,
           raw_best_objective = std::numeric_limits<double>::infinity();
    const std::size_t orientation_score_count = writeOrientationScoreMap(
        results, candidate_downward_degrees, candidate_optical_roll_degrees,
        candidate_focal_scales, orientation_score_path,
        &raw_best_down, &raw_best_yaw, &raw_best_objective);
    const std::size_t layer_size = downward_degrees.size();
    BasinSelection basin;
    std::size_t selected_layer_start = 0;
    nlohmann::json orientation_layers = nlohmann::json::array();
    for (std::size_t start = 0, layer = 0; start < results.size();
         start += layer_size, ++layer) {
      const auto end = start + layer_size;
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
          0.5, 5.0, 5.0);
      orientation_layers.push_back(
          {{"layer", layer},
           {"optical_roll_deg", candidate_optical_roll_degrees[start]},
           {"focal_scale", candidate_focal_scales[start]},
           {"selected_down_deg", layer_down[layer_basin.result_index]},
           {"selected_yaw_deg", layer_basin.yaw_deg},
           {"corrected_score", layer_basin.corrected_score},
           {"basin_candidate_count", layer_basin.basin_count}});
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
                                                 selected_layer_start +
                                                 layer_size));
    basin = writeCorrectedScoreMap(
        selected_layer_results, selected_layer_down,
        output_dir / "orientation_corrected_scores.csv", 0.5, 5.0, 5.0);
    basin.result_index += selected_layer_start;
    selected = basin.result_index;
    final_success = !diagnostic_only && results[selected].success;
    final_reason = diagnostic_only ? "SINGLE_OBSERVATION_DIAGNOSTIC_ONLY"
                                   : results[selected].reason_code;
    display_transform = results[selected].success
                            ? results[selected].estimated_t_camera_lidar
                            : results[selected].candidate_t_camera_lidar;
    display_camera = results[selected].success
                         ? results[selected].estimated_camera
                         : results[selected].candidate_camera;
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
         "contiguous_basin_gaussian_8_neighbor_compensation"},
        {"corrected_alpha", 0.5},
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
        {"basin_candidate_count", basin.basin_count}};
    full_search_baseline["orientation_layers"] = orientation_layers;
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
    nlohmann::json report = {
        {"status", final_success ? "PASS" : "FAIL"},
        {"reason_code", final_reason},
        {"candidate_gate_reason_code", results[selected].reason_code},
        {"mode", diagnostic_only
                     ? "single_observation_fixed_K_pose_diagnostic"
                     : "real_geometry_nid_edge_multistart_joint_intrinsics"},
        {"intrinsics_source",
         !config.optimize_camera_intrinsics
             ? "manufacturer_fov_fixed_for_pose_diagnostic"
         : results[selected].success
             ? "jointly_estimated_from_manufacturer_fov_initialization"
             : "joint_intrinsic_candidate_not_activated"},
        {"intrinsic_profile_id", camera.profile_id},
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
           "LSD_2D_segments_plus_adjacent_3D_plane_intersections"},
          {"plane_postprocessing",
           "neighbor_reassignment_plus_adjacent_coplanar_merge_plus_IMU_Y_"
           "height_clustering"},
          {"horizontal_plane_axis", "+Y_down_from_IMU_gated_lidar_frame"},
          {"occlusion_edge_policy",
           "range_discontinuity_segments_debug_only_not_calibration"},
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
           "direction_plus_endpoint_distance_plus_finite_overlap"},
          {"structural_direction_weight",
           config.structural_direction_weight},
          {"structural_endpoint_weight", config.structural_endpoint_weight},
          {"structural_overlap_weight", config.structural_overlap_weight},
          {"coarse_overlap_gate",
           "visible_edge>=100 and nid_projected>=100 per observation"},
          {"flat_geometry_rejection", "structural_score<0.05"},
          {"nid_weight", config.normalized_information_distance_weight},
          {"edge_weight", config.edge_alignment_weight},
          {"structural_line_weight", config.structural_line_weight},
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
          {"ambiguity_margin", config.minimum_multistart_objective_margin}}},
        {"pairs", pairs},
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
        {"candidate_results", candidates},
        {"selected_candidate", selected},
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
         {"No manual camera intrinsic is used.",
          "At least three structurally different observations are required "
          "to jointly estimate K and RT.",
          "Lens distortion is not estimated; record and fix the camera LDC "
          "state during capture.",
          "LiDAR signal_strength is used only as an input quality filter; "
          "raw signal NMI remains disabled until conformance passes.",
          "No ground truth is available; PASS only means internal objective "
          "and prior gates passed."}}};
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

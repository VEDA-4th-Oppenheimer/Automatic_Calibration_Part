#include "auto_calib/synthetic_lidar.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <random>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
namespace auto_calib {
namespace {
constexpr std::string_view rgb_suffix = "_domain_rgb.png";
double angleAt(std::uint32_t i, double lo, double hi, std::uint32_t n) {
  return n < 2 ? (lo + hi) / 2 : lo + (hi - lo) * i / (n - 1);
}
std::uint32_t indexAt(double a, double lo, double hi, std::uint32_t n) {
  return static_cast<std::uint32_t>(std::clamp<long long>(
      std::llround((a - lo) / (hi - lo) * (n - 1)), 0, n - 1));
}
Eigen::Vector3d ray(double pan, double tilt) {
  double c = std::cos(tilt);
  return {c * std::sin(pan), -std::sin(tilt), c * std::cos(pan)};
}
void require(bool ok, const std::string &message) {
  if (!ok)
    throw std::runtime_error(message);
}
} // namespace

Eigen::Vector3d Transform::lidarToCamera(const Eigen::Vector3d &p) const {
  return rotation * p + translation_m;
}
Eigen::Vector3d Transform::cameraToLidar(const Eigen::Vector3d &p) const {
  return rotation.transpose() * (p - translation_m);
}
Transform makeTransform(const Eigen::Vector3d &t, const Eigen::Vector3d &rpy) {
  Eigen::AngleAxisd rx(rpy.x(), Eigen::Vector3d::UnitX()),
      ry(rpy.y(), Eigen::Vector3d::UnitY()),
      rz(rpy.z(), Eigen::Vector3d::UnitZ());
  return {(rz * ry * rx).toRotationMatrix(), t};
}

Frame loadStanfordFrame(const fs::path &root,
                        const std::optional<std::string> &requested) {
  fs::path rgb_dir = root / "data/rgb";
  require(fs::is_directory(rgb_dir), "Missing data/rgb: " + rgb_dir.string());
  std::vector<fs::path> candidates;
  for (const auto &e : fs::directory_iterator(rgb_dir))
    if (e.is_regular_file() &&
        e.path().filename().string().find(rgb_suffix) != std::string::npos)
      candidates.push_back(e.path());
  std::sort(candidates.begin(), candidates.end());
  require(!candidates.empty(), "No Stanford RGB frames");
  fs::path rgb = requested ? rgb_dir / (*requested + std::string(rgb_suffix))
                           : candidates.front();
  require(fs::exists(rgb), "Requested frame not found");
  std::string name = rgb.filename().string();
  std::string id = name.substr(0, name.size() - rgb_suffix.size());
  Frame f;
  f.id = id;
  f.rgb_path = rgb;
  f.depth_path = root / "data/depth" / (id + "_domain_depth.png");
  f.pose_path = root / "data/pose" / (id + "_domain_pose.json");
  f.rgb = cv::imread(f.rgb_path.string(), cv::IMREAD_COLOR);
  f.depth = cv::imread(f.depth_path.string(), cv::IMREAD_UNCHANGED);
  require(!f.rgb.empty(), "Cannot read RGB");
  require(!f.depth.empty() && f.depth.type() == CV_16UC1,
          "Depth is not 16-bit PNG");
  require(f.rgb.size() == f.depth.size(), "RGB/depth size mismatch");
  std::ifstream in(f.pose_path);
  require(bool(in), "Cannot read pose");
  nlohmann::json pose;
  in >> pose;
  auto k = pose.at("camera_k_matrix");
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      f.camera.k(r, c) = k.at(r).at(c).get<double>();
  f.camera.width = f.rgb.cols;
  f.camera.height = f.rgb.rows;
  return f;
}

std::vector<Eigen::Vector3d>
projectDepth(const cv::Mat &depth, const CameraModel &c, std::uint32_t stride) {
  require(depth.type() == CV_16UC1, "Depth type must be CV_16UC1");
  require(stride > 0, "Stride must be positive");
  require(c.k(0, 0) > 0 && c.k(1, 1) > 0, "Invalid camera K");
  std::vector<Eigen::Vector3d> out;
  out.reserve(depth.total() / (stride * stride));
  for (int v = 0; v < depth.rows; v += stride) {
    const auto *row = depth.ptr<std::uint16_t>(v);
    for (int u = 0; u < depth.cols; u += stride) {
      std::uint16_t d = row[u];
      if (d == 0 || d == 65535)
        continue;
      double z = d / 512.0;
      out.emplace_back((u - c.k(0, 2)) * z / c.k(0, 0),
                       (v - c.k(1, 2)) * z / c.k(1, 1), z);
    }
  }
  return out;
}

Scan generateScan(const std::vector<Eigen::Vector3d> &source,
                  const Transform &tf, const ScanConfig &cfg) {
  require(cfg.rows > 0 && cfg.columns > 0, "Invalid shape");
  require(cfg.pan_min < cfg.pan_max && cfg.tilt_min < cfg.tilt_max,
          "Invalid angles");
  require(cfg.min_range > 0 && cfg.min_range < cfg.max_range, "Invalid ranges");
  require(cfg.dropout >= 0 && cfg.dropout <= 1, "Invalid dropout");
  Scan scan;
  scan.config = cfg;
  scan.source_count = source.size();
  scan.points.resize(static_cast<std::size_t>(cfg.rows) * cfg.columns);
  for (std::uint32_t r = 0; r < cfg.rows; ++r)
    for (std::uint32_t c = 0; c < cfg.columns; ++c) {
      auto &p = scan.points[static_cast<std::size_t>(r) * cfg.columns + c];
      p.row = r;
      p.column = c;
      p.pan = angleAt(c, cfg.pan_min, cfg.pan_max, cfg.columns);
      p.tilt = angleAt(r, cfg.tilt_max, cfg.tilt_min, cfg.rows);
      p.timestamp = static_cast<std::int64_t>(r) * cfg.columns + c;
    }
  for (const auto &pc : source) {
    Eigen::Vector3d pl = tf.cameraToLidar(pc);
    double range = pl.norm();
    if (!std::isfinite(range) || range < cfg.min_range ||
        range > cfg.max_range || pl.z() <= 0)
      continue;
    double pan = std::atan2(pl.x(), pl.z()),
           tilt = std::atan2(-pl.y(), std::hypot(pl.x(), pl.z()));
    if (pan < cfg.pan_min || pan > cfg.pan_max || tilt < cfg.tilt_min ||
        tilt > cfg.tilt_max)
      continue;
    auto col = indexAt(pan, cfg.pan_min, cfg.pan_max, cfg.columns);
    auto row =
        cfg.rows - 1 - indexAt(tilt, cfg.tilt_min, cfg.tilt_max, cfg.rows);
    auto &p = scan.points[static_cast<std::size_t>(row) * cfg.columns + col];
    if (!p.valid() || range < p.range) {
      p.range = range;
      p.xyz = pl.cast<float>();
      p.precision = cfg.noise_stddev;
      p.flags = kValidRange | kSyntheticMeasurement;
    }
  }
  std::mt19937 rng(cfg.seed);
  std::normal_distribution<double> noise(
      0, cfg.noise_stddev > 0 ? cfg.noise_stddev : 1.0);
  std::bernoulli_distribution drop(cfg.dropout);
  for (auto &p : scan.points)
    if (p.valid()) {
      if (drop(rng)) {
        p.xyz.setConstant(std::numeric_limits<float>::quiet_NaN());
        p.range = std::numeric_limits<float>::quiet_NaN();
        p.flags = kSyntheticMeasurement;
        continue;
      }
      p.range = std::clamp(p.range + (cfg.noise_stddev > 0 ? noise(rng) : 0.0),
                           cfg.min_range, cfg.max_range);
      p.xyz = (ray(p.pan, p.tilt) * p.range).cast<float>();
      ++scan.valid_count;
    }
  return scan;
}

void writePackage(const fs::path &out, const Frame &frame, const Scan &scan,
                  const Transform &gt) {
  fs::create_directories(out / "cloud");
  fs::create_directories(out / "camera");
  fs::create_directories(out / "calibration");
  fs::create_directories(out / "qa");
  std::ofstream pcd(out / "cloud/organized_cloud.pcd");
  require(bool(pcd), "Cannot write PCD");
  pcd << "# synthetic organized pan-tilt scan\nVERSION 0.7\nFIELDS x y z range "
         "range_precision pan tilt timestamp row column quality_flags\nSIZE 4 "
         "4 4 4 4 4 4 8 4 4 4\nTYPE F F F F F F F I U U U\nCOUNT 1 1 1 1 1 1 1 "
         "1 1 1 1\nWIDTH "
      << scan.config.columns << "\nHEIGHT " << scan.config.rows
      << "\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << scan.points.size()
      << "\nDATA ascii\n"
      << std::setprecision(9);
  for (const auto &p : scan.points)
    pcd << p.xyz.x() << ' ' << p.xyz.y() << ' ' << p.xyz.z() << ' ' << p.range
        << ' ' << p.precision << ' ' << p.pan << ' ' << p.tilt << ' '
        << p.timestamp << ' ' << p.row << ' ' << p.column << ' ' << p.flags
        << '\n';
  cv::Mat ranges(scan.config.rows, scan.config.columns, CV_32FC1,
                 cv::Scalar(std::numeric_limits<float>::quiet_NaN())),
      mask(scan.config.rows, scan.config.columns, CV_8UC1, cv::Scalar(0));
  for (const auto &p : scan.points)
    if (p.valid()) {
      ranges.at<float>(p.row, p.column) = p.range;
      mask.at<std::uint8_t>(p.row, p.column) = 255;
    }
  require(cv::imwrite((out / "cloud/range_image.exr").string(), ranges),
          "Cannot write EXR");
  require(cv::imwrite((out / "cloud/validity_mask.png").string(), mask),
          "Cannot write mask");
  require(cv::imwrite((out / "camera/rgb.png").string(), frame.rgb),
          "Cannot write RGB");
  nlohmann::json rotation = nlohmann::json::array();
  for (int r = 0; r < 3; ++r)
    rotation.push_back(
        {gt.rotation(r, 0), gt.rotation(r, 1), gt.rotation(r, 2)});
  nlohmann::json truth = {
      {"parent_frame", "camera_optical"},
      {"child_frame", "lidar_scan"},
      {"convention", "p_camera = R_camera_lidar * p_lidar + t_camera_lidar"},
      {"rotation_matrix", rotation},
      {"translation_m",
       {gt.translation_m.x(), gt.translation_m.y(), gt.translation_m.z()}}};
  std::ofstream(out / "calibration/ground_truth_extrinsic.json")
      << std::setw(2) << truth << '\n';
  double ratio =
      scan.points.empty() ? 0.0 : double(scan.valid_count) / scan.points.size();
  nlohmann::json quality = {
      {"status", scan.valid_count ? "PASS" : "FAIL"},
      {"source_point_count", scan.source_count},
      {"organized_cell_count", scan.points.size()},
      {"valid_point_count", scan.valid_count},
      {"valid_ratio", ratio},
      {"limitations",
       {"camera-visible geometry only", "synthetic timestamps",
        "no signal strength", "no actuator dynamics"}}};
  std::ofstream(out / "qa/pointcloud_quality.json")
      << std::setw(2) << quality << '\n';
  YAML::Emitter y;
  y << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value << 1
    << YAML::Key << "session_id" << YAML::Value << frame.id << YAML::Key
    << "producer" << YAML::Value << "synthetic_stanford_2d3ds" << YAML::Key
    << "frames" << YAML::Value << YAML::BeginMap << YAML::Key
    << "point_cloud_frame" << YAML::Value << "lidar_scan" << YAML::Key
    << "camera_frame" << YAML::Value << "camera_optical" << YAML::EndMap
    << YAML::Key << "units" << YAML::Value << YAML::BeginMap << YAML::Key
    << "distance" << YAML::Value << "meter" << YAML::Key << "angle"
    << YAML::Value << "radian" << YAML::EndMap << YAML::Key << "scan"
    << YAML::Value << YAML::BeginMap << YAML::Key << "mode" << YAML::Value
    << "synthetic_angular_raster" << YAML::Key << "organized_shape"
    << YAML::Value << YAML::Flow << YAML::BeginSeq << scan.config.rows
    << scan.config.columns << YAML::EndSeq << YAML::EndMap << YAML::Key
    << "source_frame" << YAML::Value << frame.id << YAML::Key
    << "quality_status" << YAML::Value << (scan.valid_count ? "PASS" : "FAIL")
    << YAML::EndMap;
  std::ofstream(out / "manifest.yaml") << y.c_str() << '\n';
}
} // namespace auto_calib

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

namespace auto_calib {
constexpr std::uint32_t kValidRange = 1U;
constexpr std::uint32_t kSyntheticMeasurement = 1U << 16U;

struct CameraModel {
  Eigen::Matrix3d k = Eigen::Matrix3d::Identity();
  int width = 0;
  int height = 0;
};
struct Transform {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d lidarToCamera(const Eigen::Vector3d &p) const;
  Eigen::Vector3d cameraToLidar(const Eigen::Vector3d &p) const;
};
struct Frame {
  std::string id;
  std::filesystem::path rgb_path, depth_path, pose_path;
  CameraModel camera;
  cv::Mat rgb, depth;
};
struct ScanConfig {
  double pan_min = -0.70, pan_max = 0.70, tilt_min = -0.45, tilt_max = 0.45;
  std::uint32_t columns = 321, rows = 121, pixel_stride = 2;
  double min_range = 0.25, max_range = 20.0, noise_stddev = 0.0, dropout = 0.0;
  std::uint32_t seed = 7;
};
struct Point {
  Eigen::Vector3f xyz =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
  float range = std::numeric_limits<float>::quiet_NaN(),
        precision = std::numeric_limits<float>::quiet_NaN(),
        signal_strength = std::numeric_limits<float>::quiet_NaN();
  float pan = 0, tilt = 0;
  std::int64_t timestamp = 0;
  std::uint32_t row = 0, column = 0, flags = 0;
  bool valid() const { return (flags & kValidRange) != 0; }
};
struct Scan {
  ScanConfig config;
  std::vector<Point> points;
  std::size_t source_count = 0, valid_count = 0;
};

Frame loadStanfordFrame(const std::filesystem::path &root,
                        const std::optional<std::string> &id = {});
std::vector<Eigen::Vector3d> projectDepth(const cv::Mat &depth,
                                          const CameraModel &camera,
                                          std::uint32_t stride = 1);
Transform makeTransform(const Eigen::Vector3d &translation,
                        const Eigen::Vector3d &rpy);
Scan generateScan(const std::vector<Eigen::Vector3d> &camera_points,
                  const Transform &t_camera_lidar, const ScanConfig &config);
void writePackage(const std::filesystem::path &output, const Frame &frame,
                  const Scan &scan, const Transform &ground_truth);
} // namespace auto_calib

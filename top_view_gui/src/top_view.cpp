#include "top_view/top_view.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

namespace top_view {
namespace {
using nlohmann::json;

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}

cv::Matx33d matrixFromJson(const json &value) {
  require(value.is_array() && value.size() == 3,
          "rotation_matrix must contain three rows");
  cv::Matx33d result;
  for (int row = 0; row < 3; ++row) {
    require(value.at(row).is_array() && value.at(row).size() == 3,
            "rotation_matrix rows must contain three values");
    for (int column = 0; column < 3; ++column)
      result(row, column) = value.at(row).at(column).get<double>();
  }
  return result;
}

cv::Matx33d matrixFromQuaternion(const json &value) {
  require(value.is_array() && value.size() == 4,
          "quaternion_xyzw must contain four values");
  double x = value.at(0).get<double>();
  double y = value.at(1).get<double>();
  double z = value.at(2).get<double>();
  double w = value.at(3).get<double>();
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  require(norm > 1e-12, "quaternion_xyzw has zero norm");
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  return {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
          2.0 * (x * z + y * w),       2.0 * (x * y + z * w),
          1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
          2.0 * (x * z - y * w),       2.0 * (y * z + x * w),
          1.0 - 2.0 * (x * x + y * y)};
}

cv::Vec3d vectorFromJson(const json &value) {
  require(value.is_array() && value.size() == 3,
          "translation_m must contain three values");
  return {value.at(0).get<double>(), value.at(1).get<double>(),
          value.at(2).get<double>()};
}

const json *selectTransform(const json &document,
                            const std::string &preferred_key) {
  if (!preferred_key.empty()) {
    require(document.contains(preferred_key),
            "RT JSON does not contain requested key: " + preferred_key);
    return &document.at(preferred_key);
  }
  if (document.contains("rotation_matrix") ||
      document.contains("quaternion_xyzw"))
    return &document;
  static const std::array<const char *, 7> keys = {"estimated",
                                                   "extrinsic",
                                                   "transform",
                                                   "manual_camera_lidar",
                                                   "automatic_camera_lidar",
                                                   "manual_transform",
                                                   "automatic_transform"};
  for (const char *key : keys)
    if (document.contains(key))
      return &document.at(key);
  throw std::runtime_error(
      "Cannot find transform. Expected rotation_matrix/quaternion_xyzw or a "
      "known transform key");
}

json transformJson(const Transform &transform) {
  json rotation = json::array();
  for (int row = 0; row < 3; ++row)
    rotation.push_back({transform.rotation(row, 0), transform.rotation(row, 1),
                        transform.rotation(row, 2)});
  return {
      {"parent_frame", transform.parent_frame},
      {"child_frame", transform.child_frame},
      {"convention", "p_parent = R_parent_child * p_child + t_parent_child"},
      {"rotation_matrix", rotation},
      {"translation_m",
       {transform.translation_m[0], transform.translation_m[1],
        transform.translation_m[2]}}};
}

json matrixJson(const cv::Matx33d &matrix) {
  json result = json::array();
  for (int row = 0; row < 3; ++row)
    result.push_back({matrix(row, 0), matrix(row, 1), matrix(row, 2)});
  return result;
}

cv::Point mapPixel(double x_m, double y_m, const TopViewConfig &config) {
  return {static_cast<int>(
              std::lround((x_m - config.x_min_m) * config.pixels_per_meter)),
          static_cast<int>(
              std::lround((config.y_max_m - y_m) * config.pixels_per_meter))};
}

} // namespace

nlohmann::json readJson(const std::filesystem::path &path) {
  std::ifstream input(path);
  require(bool(input), "Cannot open JSON: " + path.string());
  json value;
  input >> value;
  return value;
}

Transform parseTransform(const nlohmann::json &document,
                         const std::string &preferred_key,
                         const std::string &default_parent,
                         const std::string &default_child) {
  const json &value = *selectTransform(document, preferred_key);
  Transform result;
  if (value.contains("rotation_matrix"))
    result.rotation = matrixFromJson(value.at("rotation_matrix"));
  else if (value.contains("quaternion_xyzw"))
    result.rotation = matrixFromQuaternion(value.at("quaternion_xyzw"));
  else
    throw std::runtime_error(
        "Transform requires rotation_matrix or quaternion_xyzw");
  result.translation_m = vectorFromJson(value.at("translation_m"));
  result.parent_frame = value.value("parent_frame", default_parent);
  result.child_frame = value.value("child_frame", default_child);
  const double determinant = cv::determinant(cv::Mat(result.rotation));
  require(std::abs(determinant - 1.0) < 1e-3,
          "Rotation matrix determinant is not close to +1");
  const cv::Matx33d orthogonality = result.rotation * result.rotation.t();
  require(cv::norm(cv::Mat(orthogonality), cv::Mat(cv::Matx33d::eye()),
                   cv::NORM_INF) < 1e-3,
          "Rotation matrix is not orthonormal");
  return result;
}

CameraModel parseCameraModel(const nlohmann::json &document) {
  const json *camera = &document;
  if (document.contains("camera"))
    camera = &document.at("camera");
  const json *intrinsic = camera;
  if (camera->contains("intrinsic"))
    intrinsic = &camera->at("intrinsic");
  CameraModel result;
  result.k = {intrinsic->at("fx").get<double>(),
              0.0,
              intrinsic->at("cx").get<double>(),
              0.0,
              intrinsic->at("fy").get<double>(),
              intrinsic->at("cy").get<double>(),
              0.0,
              0.0,
              1.0};
  if (camera->contains("resolution")) {
    const auto &resolution = camera->at("resolution");
    require(resolution.is_array() && resolution.size() == 2,
            "camera.resolution must be [width, height]");
    result.width = resolution.at(0).get<int>();
    result.height = resolution.at(1).get<int>();
  } else {
    result.width = camera->value("width", 0);
    result.height = camera->value("height", 0);
  }
  result.model = camera->value("model", "unknown");
  result.profile_id = camera->value("intrinsic_profile_id", "");
  require(result.k(0, 0) > 0.0 && result.k(1, 1) > 0.0,
          "Camera focal lengths must be positive");
  require(result.width > 0 && result.height > 0,
          "Camera resolution must be positive");
  return result;
}

Transform inverse(const Transform &transform) {
  Transform result;
  result.rotation = transform.rotation.t();
  result.translation_m = -(result.rotation * transform.translation_m);
  result.parent_frame = transform.child_frame;
  result.child_frame = transform.parent_frame;
  return result;
}

Transform compose(const Transform &parent_middle,
                  const Transform &middle_child) {
  if (!parent_middle.child_frame.empty() && !middle_child.parent_frame.empty())
    require(parent_middle.child_frame == middle_child.parent_frame,
            "Transform frame mismatch: " + parent_middle.child_frame +
                " != " + middle_child.parent_frame);
  Transform result;
  result.rotation = parent_middle.rotation * middle_child.rotation;
  result.translation_m = parent_middle.rotation * middle_child.translation_m +
                         parent_middle.translation_m;
  result.parent_frame = parent_middle.parent_frame;
  result.child_frame = middle_child.child_frame;
  return result;
}

Transform identityPlaneTransform(const std::string &parent_frame,
                                 const std::string &plane_frame) {
  Transform result;
  result.parent_frame = parent_frame;
  result.child_frame = plane_frame;
  return result;
}

cv::Size outputSize(const TopViewConfig &config) {
  require(config.x_max_m > config.x_min_m && config.y_max_m > config.y_min_m,
          "Top-view bounds must have positive width and height");
  require(config.pixels_per_meter > 0.0, "pixels_per_meter must be positive");
  const int width = static_cast<int>(
      std::lround((config.x_max_m - config.x_min_m) * config.pixels_per_meter));
  const int height = static_cast<int>(
      std::lround((config.y_max_m - config.y_min_m) * config.pixels_per_meter));
  require(width > 0 && height > 0 && width <= 16384 && height <= 16384,
          "Top-view output size is invalid or exceeds 16384 pixels");
  return {width, height};
}

cv::Matx33d imageFromTopViewHomography(const CameraModel &camera,
                                       const Transform &t_camera_plane,
                                       const TopViewConfig &config) {
  outputSize(config);
  const cv::Matx33d plane_to_normalized{
      t_camera_plane.rotation(0, 0),   t_camera_plane.rotation(0, 1),
      t_camera_plane.translation_m[0], t_camera_plane.rotation(1, 0),
      t_camera_plane.rotation(1, 1),   t_camera_plane.translation_m[1],
      t_camera_plane.rotation(2, 0),   t_camera_plane.rotation(2, 1),
      t_camera_plane.translation_m[2]};
  const cv::Matx33d map_to_plane{1.0 / config.pixels_per_meter,
                                 0.0,
                                 config.x_min_m,
                                 0.0,
                                 -1.0 / config.pixels_per_meter,
                                 config.y_max_m,
                                 0.0,
                                 0.0,
                                 1.0};
  const cv::Matx33d homography = camera.k * plane_to_normalized * map_to_plane;
  require(std::abs(cv::determinant(cv::Mat(homography))) > 1e-12,
          "Ground-plane homography is singular; check RT and plane pose");
  return homography;
}

cv::Mat renderTopView(const cv::Mat &camera_bgr, const CameraModel &camera,
                      const Transform &t_camera_plane,
                      const TopViewConfig &config) {
  require(!camera_bgr.empty(), "Camera image is empty");
  const cv::Size size = outputSize(config);
  const cv::Matx33d image_from_map =
      imageFromTopViewHomography(camera, t_camera_plane, config);
  cv::Mat result;
  cv::warpPerspective(camera_bgr, result, cv::Mat(image_from_map), size,
                      cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
                      cv::BORDER_CONSTANT, cv::Scalar(32, 32, 32));
  if (!config.draw_grid || config.grid_spacing_m <= 0.0)
    return result;

  const double x_start =
      std::ceil(config.x_min_m / config.grid_spacing_m) * config.grid_spacing_m;
  const double y_start =
      std::ceil(config.y_min_m / config.grid_spacing_m) * config.grid_spacing_m;
  for (double x = x_start; x <= config.x_max_m + 1e-9;
       x += config.grid_spacing_m) {
    const auto top = mapPixel(x, config.y_max_m, config);
    const auto bottom = mapPixel(x, config.y_min_m, config);
    const cv::Scalar color = std::abs(x) < 1e-9 ? cv::Scalar(60, 60, 255)
                                                : cv::Scalar(100, 100, 100);
    cv::line(result, top, bottom, color, std::abs(x) < 1e-9 ? 2 : 1,
             cv::LINE_AA);
  }
  for (double y = y_start; y <= config.y_max_m + 1e-9;
       y += config.grid_spacing_m) {
    const auto left = mapPixel(config.x_min_m, y, config);
    const auto right = mapPixel(config.x_max_m, y, config);
    const cv::Scalar color = std::abs(y) < 1e-9 ? cv::Scalar(60, 255, 60)
                                                : cv::Scalar(100, 100, 100);
    cv::line(result, left, right, color, std::abs(y) < 1e-9 ? 2 : 1,
             cv::LINE_AA);
  }
  cv::putText(result, "X right / Y up in reference plane", {12, 24},
              cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
  return result;
}

nlohmann::json renderMetadata(const CameraModel &camera,
                              const Transform &input_transform,
                              const Transform &plane_transform,
                              const Transform &t_camera_plane,
                              const TopViewConfig &config) {
  const auto homography =
      imageFromTopViewHomography(camera, t_camera_plane, config);
  const auto size = outputSize(config);
  return {{"schema_version", "1.0"},
          {"input_transform", transformJson(input_transform)},
          {"plane_transform", transformJson(plane_transform)},
          {"camera_plane_transform", transformJson(t_camera_plane)},
          {"camera",
           {{"model", camera.model},
            {"intrinsic_profile_id", camera.profile_id},
            {"resolution", {camera.width, camera.height}},
            {"k", matrixJson(camera.k)}}},
          {"top_view",
           {{"x_range_m", {config.x_min_m, config.x_max_m}},
            {"y_range_m", {config.y_min_m, config.y_max_m}},
            {"pixels_per_meter", config.pixels_per_meter},
            {"grid_spacing_m", config.grid_spacing_m},
            {"output_size", {size.width, size.height}},
            {"image_from_top_view_homography", matrixJson(homography)}}}};
}

} // namespace top_view

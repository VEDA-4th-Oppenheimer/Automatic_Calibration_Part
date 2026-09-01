#pragma once

#include <filesystem>
#include <nlohmann/json_fwd.hpp>
#include <opencv2/core.hpp>
#include <string>

namespace top_view {

struct Transform {
  cv::Matx33d rotation = cv::Matx33d::eye();
  cv::Vec3d translation_m{0.0, 0.0, 0.0};
  std::string parent_frame;
  std::string child_frame;
};

struct CameraModel {
  cv::Matx33d k = cv::Matx33d::eye();
  int width = 0;
  int height = 0;
  std::string model;
  std::string profile_id;
};

struct TopViewConfig {
  double x_min_m = -5.0;
  double x_max_m = 5.0;
  double y_min_m = 0.0;
  double y_max_m = 10.0;
  double pixels_per_meter = 80.0;
  double grid_spacing_m = 1.0;
  bool draw_grid = true;
};

nlohmann::json readJson(const std::filesystem::path &path);
Transform parseTransform(const nlohmann::json &document,
                         const std::string &preferred_key = {},
                         const std::string &default_parent = "camera_optical",
                         const std::string &default_child = "reference_frame");
CameraModel parseCameraModel(const nlohmann::json &document);
Transform inverse(const Transform &transform);
Transform compose(const Transform &parent_middle,
                  const Transform &middle_child);
Transform
identityPlaneTransform(const std::string &parent_frame,
                       const std::string &plane_frame = "reference_plane");
cv::Size outputSize(const TopViewConfig &config);
cv::Matx33d imageFromTopViewHomography(const CameraModel &camera,
                                       const Transform &t_camera_plane,
                                       const TopViewConfig &config);
cv::Mat renderTopView(const cv::Mat &camera_bgr, const CameraModel &camera,
                      const Transform &t_camera_plane,
                      const TopViewConfig &config);
nlohmann::json renderMetadata(const CameraModel &camera,
                              const Transform &input_transform,
                              const Transform &plane_transform,
                              const Transform &t_camera_plane,
                              const TopViewConfig &config);

} // namespace top_view

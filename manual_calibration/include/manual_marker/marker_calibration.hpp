#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace manual_marker {

struct BoardConfig {
  std::string dictionary = "DICT_5X5_100";
  int squares_x = 7;
  int squares_y = 5;
  double square_length_m = 0.04;
  double marker_length_m = 0.03;
  bool legacy_pattern = false;
  int minimum_charuco_corners = 6;
};

struct CameraModel {
  cv::Matx33d k = cv::Matx33d::eye();
  cv::Mat distortion;
  int width = 0;
  int height = 0;
  std::string model;
  std::string profile_id;
};

struct Detection {
  std::filesystem::path image_path;
  cv::Size image_size;
  cv::Mat charuco_corners;
  cv::Mat charuco_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::Mat marker_ids;
  int marker_count = 0;
  int charuco_corner_count = 0;
  bool accepted = false;
  std::string reason_code;
};

struct IntrinsicConfig {
  int minimum_frames = 10;
  double maximum_rms_px = 2.0;
};

struct IntrinsicResult {
  bool solved = false;
  std::string status = "FAIL";
  std::string reason_code = "NOT_RUN";
  CameraModel camera;
  double calibration_rms_px = 0.0;
  int input_frame_count = 0;
  int accepted_frame_count = 0;
  std::vector<double> per_view_rmse_px;
  std::vector<Detection> detections;
};

struct Transform {
  cv::Matx33d rotation = cv::Matx33d::eye();
  cv::Vec3d translation_m{0.0, 0.0, 0.0};
  std::string parent_frame;
  std::string child_frame;
};

struct PoseResult {
  bool solved = false;
  std::string status = "FAIL";
  std::string reason_code = "NOT_RUN";
  Transform t_camera_board;
  Detection detection;
  double reprojection_rmse_px = 0.0;
  double reprojection_max_px = 0.0;
  std::vector<cv::Point2f> projected_points;
};

BoardConfig loadBoardConfig(const std::filesystem::path &path);
CameraModel loadCamera(const std::filesystem::path &path);
std::vector<std::filesystem::path>
listImages(const std::filesystem::path &directory);

cv::Mat generateBoardImage(const BoardConfig &config, int width_px,
                           int height_px, int margin_px = 40,
                           int border_bits = 1);
cv::Mat generateDisplayBoardImage(const BoardConfig &config,
                                  int canvas_width_px, int canvas_height_px,
                                  int square_size_px, double pixels_per_meter,
                                  double ruler_length_m = 0.1);
Detection detect(const cv::Mat &image, const std::filesystem::path &image_path,
                 const BoardConfig &board, const CameraModel *camera = nullptr);
cv::Mat drawDetection(const cv::Mat &image, const Detection &detection,
                      const PoseResult *pose = nullptr);

IntrinsicResult
calibrateIntrinsics(const std::vector<std::filesystem::path> &images,
                    const BoardConfig &board,
                    const IntrinsicConfig &config = {});
PoseResult estimateBoardPose(const cv::Mat &image,
                             const std::filesystem::path &image_path,
                             const BoardConfig &board,
                             const CameraModel &camera,
                             double maximum_rmse_px = 3.0);

Transform inverse(const Transform &transform);
Transform compose(const Transform &parent_middle,
                  const Transform &middle_child);
double rotationDistanceDeg(const cv::Matx33d &a, const cv::Matx33d &b);
Transform transformFromFlexibleJson(const nlohmann::json &document,
                                    const std::string &default_parent = "",
                                    const std::string &default_child = "");
nlohmann::json transformToJson(const Transform &transform);
nlohmann::json intrinsicResultToJson(const IntrinsicResult &result,
                                     const BoardConfig &board);
nlohmann::json poseResultToJson(const PoseResult &result,
                                const BoardConfig &board);

} // namespace manual_marker

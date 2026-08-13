#include "manual_marker/marker_calibration.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <numeric>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <unordered_map>

namespace fs = std::filesystem;
using nlohmann::json;

namespace manual_marker {
namespace {

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}

cv::aruco::PredefinedDictionaryType dictionaryType(const std::string &name) {
  static const std::unordered_map<std::string,
                                  cv::aruco::PredefinedDictionaryType>
      types = {{"DICT_4X4_50", cv::aruco::DICT_4X4_50},
               {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
               {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
               {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
               {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
               {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
               {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
               {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
               {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}};
  const auto iterator = types.find(name);
  require(iterator != types.end(), "Unsupported ArUco dictionary: " + name);
  return iterator->second;
}

cv::Ptr<cv::aruco::CharucoBoard> makeBoard(const BoardConfig &config) {
  require(config.squares_x >= 3 && config.squares_y >= 3,
          "ChArUco board must have at least 3x3 squares");
  require(config.square_length_m > config.marker_length_m &&
              config.marker_length_m > 0.0,
          "square_length_m must be greater than marker_length_m > 0");
  const auto dictionary =
      cv::aruco::getPredefinedDictionary(dictionaryType(config.dictionary));
  auto board = cv::makePtr<cv::aruco::CharucoBoard>(
      cv::Size(config.squares_x, config.squares_y),
      static_cast<float>(config.square_length_m),
      static_cast<float>(config.marker_length_m), dictionary);
  board->setLegacyPattern(config.legacy_pattern);
  return board;
}

cv::Mat cameraMatrix(const CameraModel &camera) {
  return cv::Mat(3, 3, CV_64F, const_cast<double *>(camera.k.val)).clone();
}

cv::Mat distortionVector(const CameraModel &camera) {
  if (camera.distortion.empty())
    return cv::Mat::zeros(1, 5, CV_64F);
  cv::Mat result;
  camera.distortion.convertTo(result, CV_64F);
  return result.reshape(1, 1);
}

void objectAndImagePoints(const Detection &detection,
                          const cv::aruco::CharucoBoard &board,
                          std::vector<cv::Point3f> &object_points,
                          std::vector<cv::Point2f> &image_points) {
  const auto chessboard_corners = board.getChessboardCorners();
  object_points.clear();
  image_points.clear();
  for (std::size_t index = 0; index < detection.charuco_ids.total(); ++index) {
    const int id = detection.charuco_ids.at<int>(static_cast<int>(index));
    require(id >= 0 && static_cast<std::size_t>(id) < chessboard_corners.size(),
            "Detected ChArUco corner ID is outside board definition");
    object_points.push_back(
        chessboard_corners.at(static_cast<std::size_t>(id)));
    image_points.push_back(
        detection.charuco_corners.at<cv::Point2f>(static_cast<int>(index)));
  }
}

cv::Matx33d matrixFromJson(const json &value) {
  require(value.is_array() && value.size() == 3,
          "rotation_matrix must have 3 rows");
  cv::Matx33d result;
  for (int row = 0; row < 3; ++row) {
    require(value.at(row).is_array() && value.at(row).size() == 3,
            "rotation_matrix rows must have 3 values");
    for (int column = 0; column < 3; ++column)
      result(row, column) = value.at(row).at(column).get<double>();
  }
  return result;
}

cv::Vec3d vectorFromJson(const json &value) {
  require(value.is_array() && value.size() == 3,
          "translation_m must have 3 values");
  return {value.at(0).get<double>(), value.at(1).get<double>(),
          value.at(2).get<double>()};
}

std::array<double, 4> quaternionXyzw(const cv::Matx33d &r) {
  const double trace = r(0, 0) + r(1, 1) + r(2, 2);
  double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (r(2, 1) - r(1, 2)) / s;
    y = (r(0, 2) - r(2, 0)) / s;
    z = (r(1, 0) - r(0, 1)) / s;
  } else if (r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2)) {
    const double s = std::sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
    w = (r(2, 1) - r(1, 2)) / s;
    x = 0.25 * s;
    y = (r(0, 1) + r(1, 0)) / s;
    z = (r(0, 2) + r(2, 0)) / s;
  } else if (r(1, 1) > r(2, 2)) {
    const double s = std::sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
    w = (r(0, 2) - r(2, 0)) / s;
    x = (r(0, 1) + r(1, 0)) / s;
    y = 0.25 * s;
    z = (r(1, 2) + r(2, 1)) / s;
  } else {
    const double s = std::sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
    w = (r(1, 0) - r(0, 1)) / s;
    x = (r(0, 2) + r(2, 0)) / s;
    y = (r(1, 2) + r(2, 1)) / s;
    z = 0.25 * s;
  }
  return {x, y, z, w};
}

json boardToJson(const BoardConfig &board) {
  return {{"dictionary", board.dictionary},
          {"squares_x", board.squares_x},
          {"squares_y", board.squares_y},
          {"square_length_m", board.square_length_m},
          {"marker_length_m", board.marker_length_m},
          {"legacy_pattern", board.legacy_pattern},
          {"minimum_charuco_corners", board.minimum_charuco_corners}};
}

json detectionToJson(const Detection &detection, double per_view_rmse = -1.0) {
  json value = {{"image", detection.image_path.string()},
                {"marker_count", detection.marker_count},
                {"charuco_corner_count", detection.charuco_corner_count},
                {"accepted", detection.accepted},
                {"reason_code", detection.reason_code}};
  if (per_view_rmse >= 0.0)
    value["reprojection_rmse_px"] = per_view_rmse;
  return value;
}

} // namespace

BoardConfig loadBoardConfig(const fs::path &path) {
  std::ifstream input(path);
  require(bool(input), "Cannot open board JSON: " + path.string());
  json document;
  input >> document;
  const json &value =
      document.contains("board") ? document.at("board") : document;
  BoardConfig result;
  result.dictionary = value.value("dictionary", result.dictionary);
  result.squares_x = value.value("squares_x", result.squares_x);
  result.squares_y = value.value("squares_y", result.squares_y);
  result.square_length_m =
      value.value("square_length_m", result.square_length_m);
  result.marker_length_m =
      value.value("marker_length_m", result.marker_length_m);
  result.legacy_pattern = value.value("legacy_pattern", false);
  result.minimum_charuco_corners =
      value.value("minimum_charuco_corners", result.minimum_charuco_corners);
  makeBoard(result);
  return result;
}

CameraModel loadCamera(const fs::path &path) {
  std::ifstream input(path);
  require(bool(input), "Cannot open camera JSON: " + path.string());
  json document;
  input >> document;
  const json &camera =
      document.contains("camera") ? document.at("camera") : document;
  CameraModel result;
  result.model = camera.value("model", "");
  result.profile_id = camera.value("intrinsic_profile_id", "");
  if (camera.contains("resolution")) {
    result.width = camera.at("resolution").at(0).get<int>();
    result.height = camera.at("resolution").at(1).get<int>();
  } else {
    result.width = camera.value("image_width", 0);
    result.height = camera.value("image_height", 0);
  }
  const json &intrinsic =
      camera.contains("intrinsic") ? camera.at("intrinsic") : camera;
  result.k = {intrinsic.at("fx").get<double>(),
              0.0,
              intrinsic.at("cx").get<double>(),
              0.0,
              intrinsic.at("fy").get<double>(),
              intrinsic.at("cy").get<double>(),
              0.0,
              0.0,
              1.0};
  const auto distortion = camera.value("distortion", std::vector<double>{});
  result.distortion = cv::Mat(distortion, true).reshape(1, 1);
  require(result.width > 0 && result.height > 0,
          "Camera resolution must be positive");
  require(result.k(0, 0) > 0.0 && result.k(1, 1) > 0.0,
          "Camera focal lengths must be positive");
  return result;
}

std::vector<fs::path> listImages(const fs::path &directory) {
  require(fs::is_directory(directory),
          "Image directory does not exist: " + directory.string());
  std::vector<fs::path> result;
  for (const auto &entry : fs::directory_iterator(directory)) {
    if (!entry.is_regular_file())
      continue;
    std::string extension = entry.path().extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char value) { return std::tolower(value); });
    if (extension == ".png" || extension == ".jpg" || extension == ".jpeg" ||
        extension == ".bmp" || extension == ".tif" || extension == ".tiff")
      result.push_back(entry.path());
  }
  std::sort(result.begin(), result.end());
  return result;
}

cv::Mat generateBoardImage(const BoardConfig &config, int width_px,
                           int height_px, int margin_px, int border_bits) {
  require(width_px > 0 && height_px > 0, "Board image size must be positive");
  cv::Mat image;
  makeBoard(config)->generateImage(cv::Size(width_px, height_px), image,
                                   margin_px, border_bits);
  return image;
}

cv::Mat generateDisplayBoardImage(const BoardConfig &config,
                                  int canvas_width_px, int canvas_height_px,
                                  int square_size_px, double pixels_per_meter,
                                  double ruler_length_m) {
  require(canvas_width_px > 0 && canvas_height_px > 0,
          "Display canvas size must be positive");
  require(square_size_px > 0 && pixels_per_meter > 0.0,
          "Display pixel scale must be positive");
  require(ruler_length_m > 0.0, "Ruler length must be positive");
  const int board_width_px = config.squares_x * square_size_px;
  const int board_height_px = config.squares_y * square_size_px;
  require(board_width_px < canvas_width_px &&
              board_height_px < canvas_height_px,
          "ChArUco board must fit inside the display canvas with a margin");
  const double represented_square_m = square_size_px / pixels_per_meter;
  require(std::abs(represented_square_m - config.square_length_m) < 1e-9,
          "square_length_m does not match the display pixel scale");

  cv::Mat canvas(canvas_height_px, canvas_width_px, CV_8UC1, cv::Scalar(255));
  const cv::Mat board =
      generateBoardImage(config, board_width_px, board_height_px, 0, 1);
  const int board_x = (canvas_width_px - board_width_px) / 2;
  const int board_y = (canvas_height_px - board_height_px) / 2;
  board.copyTo(
      canvas(cv::Rect(board_x, board_y, board_width_px, board_height_px)));

  const int ruler_pixels =
      static_cast<int>(std::lround(ruler_length_m * pixels_per_meter));
  require(ruler_pixels < canvas_width_px,
          "Verification ruler does not fit inside the display canvas");
  const int ruler_x = (canvas_width_px - ruler_pixels) / 2;
  const int ruler_y = std::max(35, board_y / 2 + 20);
  cv::line(canvas, {ruler_x, ruler_y}, {ruler_x + ruler_pixels, ruler_y},
           cv::Scalar(0), 4, cv::LINE_8);
  cv::line(canvas, {ruler_x, ruler_y - 12}, {ruler_x, ruler_y + 12},
           cv::Scalar(0), 4, cv::LINE_8);
  cv::line(canvas, {ruler_x + ruler_pixels, ruler_y - 12},
           {ruler_x + ruler_pixels, ruler_y + 12}, cv::Scalar(0), 4,
           cv::LINE_8);
  const std::string label =
      std::to_string(static_cast<int>(std::lround(ruler_length_m * 1000.0))) +
      " mm verification ruler";
  cv::putText(canvas, label, {ruler_x, std::max(24, ruler_y - 20)},
              cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0), 2, cv::LINE_AA);
  return canvas;
}

Detection detect(const cv::Mat &image, const fs::path &image_path,
                 const BoardConfig &board_config, const CameraModel *camera) {
  require(!image.empty(), "Cannot detect markers in an empty image");
  const auto board = makeBoard(board_config);
  cv::aruco::CharucoParameters charuco_parameters;
  charuco_parameters.tryRefineMarkers = true;
  if (camera != nullptr) {
    charuco_parameters.cameraMatrix = cameraMatrix(*camera);
    charuco_parameters.distCoeffs = distortionVector(*camera);
  }
  cv::aruco::DetectorParameters detector_parameters;
  detector_parameters.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  cv::aruco::CharucoDetector detector(*board, charuco_parameters,
                                      detector_parameters);
  Detection result;
  result.image_path = image_path;
  result.image_size = image.size();
  detector.detectBoard(image, result.charuco_corners, result.charuco_ids,
                       result.marker_corners, result.marker_ids);
  result.marker_count = static_cast<int>(result.marker_ids.total());
  result.charuco_corner_count = static_cast<int>(result.charuco_ids.total());
  if (result.charuco_corner_count < board_config.minimum_charuco_corners) {
    result.reason_code = "CHARUCO_CORNERS_INSUFFICIENT";
  } else if (board->checkCharucoCornersCollinear(result.charuco_ids)) {
    result.reason_code = "CHARUCO_CORNERS_COLLINEAR";
  } else {
    result.accepted = true;
    result.reason_code = "OK";
  }
  return result;
}

cv::Mat drawDetection(const cv::Mat &image, const Detection &detection,
                      const PoseResult *pose) {
  cv::Mat output = image.clone();
  if (!detection.marker_ids.empty())
    cv::aruco::drawDetectedMarkers(output, detection.marker_corners,
                                   detection.marker_ids);
  if (!detection.charuco_ids.empty())
    cv::aruco::drawDetectedCornersCharuco(
        output, detection.charuco_corners, detection.charuco_ids,
        detection.accepted ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 0, 255));
  if (pose != nullptr && pose->solved) {
    for (std::size_t index = 0; index < pose->projected_points.size();
         ++index) {
      const auto observed =
          detection.charuco_corners.at<cv::Point2f>(static_cast<int>(index));
      const auto projected = pose->projected_points[index];
      cv::line(output, observed, projected, cv::Scalar(0, 220, 255), 1,
               cv::LINE_AA);
      cv::drawMarker(output, projected, cv::Scalar(220, 80, 220),
                     cv::MARKER_CROSS, 10, 2, cv::LINE_AA);
    }
  }
  cv::rectangle(output, {0, 0}, {output.cols, 42}, {0, 0, 0}, cv::FILLED);
  const std::string label =
      "ChArUco markers=" + std::to_string(detection.marker_count) +
      " corners=" + std::to_string(detection.charuco_corner_count) + " " +
      detection.reason_code;
  cv::putText(output, label, {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
              {255, 255, 255}, 2, cv::LINE_AA);
  return output;
}

IntrinsicResult calibrateIntrinsics(const std::vector<fs::path> &images,
                                    const BoardConfig &board_config,
                                    const IntrinsicConfig &config) {
  IntrinsicResult result;
  result.input_frame_count = static_cast<int>(images.size());
  if (images.empty()) {
    result.reason_code = "IMAGES_MISSING";
    return result;
  }
  cv::Size image_size;
  for (const auto &path : images) {
    const cv::Mat image = cv::imread(path.string(), cv::IMREAD_COLOR);
    if (image.empty()) {
      Detection failed;
      failed.image_path = path;
      failed.reason_code = "IMAGE_READ_FAILED";
      result.detections.push_back(failed);
      continue;
    }
    if (image_size.empty())
      image_size = image.size();
    if (image.size() != image_size) {
      Detection failed;
      failed.image_path = path;
      failed.image_size = image.size();
      failed.reason_code = "IMAGE_SIZE_MISMATCH";
      result.detections.push_back(failed);
      continue;
    }
    result.detections.push_back(detect(image, path, board_config));
  }

  std::vector<cv::Mat> all_corners, all_ids;
  for (const auto &detection : result.detections)
    if (detection.accepted) {
      all_corners.push_back(detection.charuco_corners);
      all_ids.push_back(detection.charuco_ids);
    }
  result.accepted_frame_count = static_cast<int>(all_corners.size());
  if (result.accepted_frame_count < config.minimum_frames) {
    result.reason_code = "ACCEPTED_FRAMES_INSUFFICIENT";
    return result;
  }

  const auto board = makeBoard(board_config);
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
  std::vector<cv::Mat> rvecs, tvecs;
  result.calibration_rms_px = cv::aruco::calibrateCameraCharuco(
      all_corners, all_ids, board, image_size, camera_matrix, distortion, rvecs,
      tvecs);
  result.camera.width = image_size.width;
  result.camera.height = image_size.height;
  result.camera.profile_id = "charuco-image-only";
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      result.camera.k(row, column) = camera_matrix.at<double>(row, column);
  result.camera.distortion = distortion.clone();

  std::size_t accepted_index = 0;
  for (const auto &detection : result.detections) {
    if (!detection.accepted)
      continue;
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points, projected;
    objectAndImagePoints(detection, *board, object_points, image_points);
    cv::projectPoints(object_points, rvecs.at(accepted_index),
                      tvecs.at(accepted_index), camera_matrix, distortion,
                      projected);
    double squared_sum = 0.0;
    for (std::size_t index = 0; index < projected.size(); ++index) {
      const double error = cv::norm(projected[index] - image_points[index]);
      squared_sum += error * error;
    }
    result.per_view_rmse_px.push_back(
        std::sqrt(squared_sum / static_cast<double>(projected.size())));
    ++accepted_index;
  }
  result.solved = true;
  if (result.calibration_rms_px > config.maximum_rms_px) {
    result.status = "REVIEW";
    result.reason_code = "CALIBRATION_RMS_HIGH";
  } else {
    result.status = "PASS";
    result.reason_code = "OK";
  }
  return result;
}

PoseResult estimateBoardPose(const cv::Mat &image, const fs::path &image_path,
                             const BoardConfig &board_config,
                             const CameraModel &camera,
                             double maximum_rmse_px) {
  PoseResult result;
  result.t_camera_board.parent_frame = "camera_optical";
  result.t_camera_board.child_frame = "marker_board";
  result.detection = detect(image, image_path, board_config, &camera);
  if (!result.detection.accepted) {
    result.reason_code = result.detection.reason_code;
    return result;
  }
  const auto board = makeBoard(board_config);
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  objectAndImagePoints(result.detection, *board, object_points, image_points);
  cv::Mat rvec, tvec;
  const bool solved = cv::solvePnP(
      object_points, image_points, cameraMatrix(camera),
      distortionVector(camera), rvec, tvec, false, cv::SOLVEPNP_IPPE);
  if (!solved) {
    result.reason_code = "PNP_FAILED";
    return result;
  }
  cv::solvePnPRefineLM(object_points, image_points, cameraMatrix(camera),
                       distortionVector(camera), rvec, tvec);
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      result.t_camera_board.rotation(row, column) =
          rotation.at<double>(row, column);
  for (int axis = 0; axis < 3; ++axis)
    result.t_camera_board.translation_m[axis] = tvec.at<double>(axis);
  cv::projectPoints(object_points, rvec, tvec, cameraMatrix(camera),
                    distortionVector(camera), result.projected_points);
  double squared_sum = 0.0;
  for (std::size_t index = 0; index < result.projected_points.size(); ++index) {
    const double error =
        cv::norm(result.projected_points[index] - image_points[index]);
    squared_sum += error * error;
    result.reprojection_max_px = std::max(result.reprojection_max_px, error);
  }
  result.reprojection_rmse_px = std::sqrt(
      squared_sum / static_cast<double>(result.projected_points.size()));
  result.solved = true;
  if (result.reprojection_rmse_px > maximum_rmse_px) {
    result.status = "REVIEW";
    result.reason_code = "REPROJECTION_RMS_HIGH";
  } else {
    result.status = "PASS";
    result.reason_code = "OK";
  }
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

double rotationDistanceDeg(const cv::Matx33d &a, const cv::Matx33d &b) {
  const auto delta = a * b.t();
  const double trace = delta(0, 0) + delta(1, 1) + delta(2, 2);
  return std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / CV_PI;
}

Transform transformFromFlexibleJson(const json &document,
                                    const std::string &default_parent,
                                    const std::string &default_child) {
  const json *value = &document;
  for (const char *key : {"extrinsic", "estimated", "transform",
                          "manual_transform", "automatic_transform"})
    if (value->contains(key)) {
      value = &value->at(key);
      break;
    }
  Transform result;
  result.rotation = matrixFromJson(value->at("rotation_matrix"));
  result.translation_m = vectorFromJson(value->at("translation_m"));
  result.parent_frame = value->value("parent_frame", default_parent);
  result.child_frame = value->value("child_frame", default_child);
  return result;
}

json transformToJson(const Transform &transform) {
  json rotation = json::array();
  for (int row = 0; row < 3; ++row)
    rotation.push_back({transform.rotation(row, 0), transform.rotation(row, 1),
                        transform.rotation(row, 2)});
  const auto quaternion = quaternionXyzw(transform.rotation);
  return {
      {"parent_frame", transform.parent_frame},
      {"child_frame", transform.child_frame},
      {"convention", "p_parent = R_parent_child * p_child + t_parent_child"},
      {"rotation_matrix", rotation},
      {"quaternion_xyzw",
       {quaternion[0], quaternion[1], quaternion[2], quaternion[3]}},
      {"translation_m",
       {transform.translation_m[0], transform.translation_m[1],
        transform.translation_m[2]}}};
}

json intrinsicResultToJson(const IntrinsicResult &result,
                           const BoardConfig &board) {
  json views = json::array();
  std::size_t accepted_index = 0;
  for (const auto &detection : result.detections) {
    const double error =
        detection.accepted && accepted_index < result.per_view_rmse_px.size()
            ? result.per_view_rmse_px[accepted_index++]
            : -1.0;
    views.push_back(detectionToJson(detection, error));
  }
  std::vector<double> distortion;
  if (!result.camera.distortion.empty()) {
    const cv::Mat values = result.camera.distortion.reshape(1, 1);
    for (int index = 0; index < values.cols; ++index)
      distortion.push_back(values.at<double>(0, index));
  }
  return {{"schema_version", "1.0"},
          {"method", "charuco_2d_image_only_intrinsic"},
          {"status", result.status},
          {"reason_code", result.reason_code},
          {"solved", result.solved},
          {"board", boardToJson(board)},
          {"camera",
           {{"model", result.camera.model},
            {"resolution", {result.camera.width, result.camera.height}},
            {"intrinsic_profile_id", result.camera.profile_id},
            {"intrinsic",
             {{"fx", result.camera.k(0, 0)},
              {"fy", result.camera.k(1, 1)},
              {"cx", result.camera.k(0, 2)},
              {"cy", result.camera.k(1, 2)}}},
            {"distortion_model", "opencv_radtan"},
            {"distortion", distortion}}},
          {"quality",
           {{"calibration_rms_px", result.calibration_rms_px},
            {"input_frame_count", result.input_frame_count},
            {"accepted_frame_count", result.accepted_frame_count},
            {"per_view_rmse_px", result.per_view_rmse_px}}},
          {"views", views}};
}

json poseResultToJson(const PoseResult &result, const BoardConfig &board) {
  return {{"schema_version", "1.0"},
          {"method", "charuco_2d_image_only_pose"},
          {"status", result.status},
          {"reason_code", result.reason_code},
          {"solved", result.solved},
          {"board", boardToJson(board)},
          {"extrinsic", transformToJson(result.t_camera_board)},
          {"quality",
           {{"marker_count", result.detection.marker_count},
            {"charuco_corner_count", result.detection.charuco_corner_count},
            {"reprojection_rmse_px", result.reprojection_rmse_px},
            {"reprojection_max_px", result.reprojection_max_px}}},
          {"image", result.detection.image_path.string()}};
}

} // namespace manual_marker

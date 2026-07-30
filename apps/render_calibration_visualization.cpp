#include "auto_calib/synthetic_lidar.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {
struct Args {
  fs::path dataset_root, result_json, output;
  std::string frame_id;
};

Args parse(int argc, char **argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--help") {
      std::cout << "render_calibration_visualization --dataset-root PATH "
                   "--result-json PATH --output PATH [--frame-id ID]\n";
      std::exit(0);
    }
    if (i + 1 >= argc)
      throw std::invalid_argument("Missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--dataset-root")
      args.dataset_root = value;
    else if (key == "--result-json")
      args.result_json = value;
    else if (key == "--output")
      args.output = value;
    else if (key == "--frame-id")
      args.frame_id = value;
    else
      throw std::invalid_argument("Unknown option: " + key);
  }
  if (args.dataset_root.empty() || args.result_json.empty() ||
      args.output.empty())
    throw std::invalid_argument(
        "--dataset-root, --result-json and --output are required");
  return args;
}

auto transformFromJson(const json &value) {
  auto_calib::Transform transform;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      transform.rotation(row, col) =
          value.at("rotation_matrix").at(row).at(col);
  for (int axis = 0; axis < 3; ++axis)
    transform.translation_m(axis) = value.at("translation_m").at(axis);
  return transform;
}

cv::Scalar rangeColor(float range) {
  const double normalized =
      std::clamp((static_cast<double>(range) - 0.25) / 8.0, 0.0, 1.0);
  cv::Mat color(1, 1, CV_8UC1, cv::Scalar(std::lround(normalized * 255.0)));
  cv::Mat mapped;
  cv::applyColorMap(color, mapped, cv::COLORMAP_JET);
  const auto pixel = mapped.at<cv::Vec3b>(0, 0);
  return {static_cast<double>(pixel[0]), static_cast<double>(pixel[1]),
          static_cast<double>(pixel[2])};
}

cv::Mat renderOverlay(const cv::Mat &rgb, const auto_calib::CameraModel &camera,
                      const auto_calib::Scan &scan,
                      const auto_calib::Transform &transform,
                      const cv::Scalar &tint, const std::string &label,
                      std::size_t *projected_count) {
  cv::Mat overlay = rgb.clone();
  std::size_t projected = 0;
  for (const auto &point : scan.points) {
    if (!point.valid())
      continue;
    const Eigen::Vector3d camera_point =
        transform.lidarToCamera(point.xyz.cast<double>());
    if (camera_point.z() <= 0.05)
      continue;
    const double u =
        camera.k(0, 0) * camera_point.x() / camera_point.z() + camera.k(0, 2);
    const double v =
        camera.k(1, 1) * camera_point.y() / camera_point.z() + camera.k(1, 2);
    const int x = static_cast<int>(std::lround(u));
    const int y = static_cast<int>(std::lround(v));
    if (x < 0 || y < 0 || x >= overlay.cols || y >= overlay.rows)
      continue;
    const cv::Scalar color = tint[0] >= 0 ? tint : rangeColor(point.range);
    cv::circle(overlay, {x, y}, 1, color, cv::FILLED, cv::LINE_AA);
    ++projected;
  }
  cv::Mat result;
  cv::addWeighted(rgb, 0.58, overlay, 0.42, 0.0, result);
  cv::rectangle(result, {0, 0}, {result.cols, 42}, {0, 0, 0}, cv::FILLED);
  cv::putText(result, label + " | projected=" + std::to_string(projected),
              {16, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.72, {255, 255, 255}, 2,
              cv::LINE_AA);
  if (projected_count)
    *projected_count = projected;
  return result;
}

cv::Mat labeled(const cv::Mat &image, const std::string &label) {
  cv::Mat result = image.clone();
  cv::rectangle(result, {0, 0}, {result.cols, 42}, {0, 0, 0}, cv::FILLED);
  cv::putText(result, label, {16, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.72,
              {255, 255, 255}, 2, cv::LINE_AA);
  return result;
}

struct ExportPoint {
  Eigen::Vector3d position;
  cv::Scalar color_bgr;
};

std::vector<ExportPoint>
makeExportPoints(const auto_calib::Scan &scan,
                 const auto_calib::Transform *transform = nullptr) {
  std::vector<ExportPoint> points;
  points.reserve(scan.valid_count);
  for (const auto &point : scan.points) {
    if (!point.valid())
      continue;
    Eigen::Vector3d position = point.xyz.cast<double>();
    if (transform)
      position = transform->lidarToCamera(position);
    points.push_back({position, rangeColor(point.range)});
  }
  return points;
}

void writePly(const fs::path &path, const std::vector<ExportPoint> &points,
              const std::string &frame_name) {
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Cannot write PLY: " + path.string());
  out << "ply\nformat ascii 1.0\ncomment frame " << frame_name
      << "\ncomment coordinates are meters\nelement vertex " << points.size()
      << "\nproperty float x\nproperty float y\nproperty float z\n"
         "property uchar red\nproperty uchar green\nproperty uchar blue\n"
         "end_header\n"
      << std::setprecision(9);
  for (const auto &point : points)
    out << point.position.x() << ' ' << point.position.y() << ' '
        << point.position.z() << ' ' << static_cast<int>(point.color_bgr[2])
        << ' ' << static_cast<int>(point.color_bgr[1]) << ' '
        << static_cast<int>(point.color_bgr[0]) << '\n';
}

void writeObj(const fs::path &path, const std::vector<ExportPoint> &points,
              const std::string &frame_name) {
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Cannot write OBJ: " + path.string());
  out << "# Automatic Calibration point cloud\n# frame " << frame_name
      << "\n# coordinates are meters\n"
      << std::setprecision(9);
  for (const auto &point : points)
    out << "v " << point.position.x() << ' ' << point.position.y() << ' '
        << point.position.z() << ' ' << point.color_bgr[2] / 255.0 << ' '
        << point.color_bgr[1] / 255.0 << ' ' << point.color_bgr[0] / 255.0
        << '\n';
}

} // namespace

int main(int argc, char **argv) {
  try {
    const Args args = parse(argc, argv);
    std::ifstream input(args.result_json);
    if (!input)
      throw std::runtime_error("Cannot open result JSON: " +
                               args.result_json.string());
    json report;
    input >> report;
    const auto ids = report.at("frame_ids");
    const std::string frame_id =
        args.frame_id.empty() ? ids.at(0).get<std::string>() : args.frame_id;
    const auto frame =
        auto_calib::loadStanfordFrame(args.dataset_root, frame_id);
    auto scan_config = auto_calib::ScanConfig{};
    scan_config.columns = 321;
    scan_config.rows = 121;
    scan_config.pixel_stride = 2;
    scan_config.noise_stddev = 0.005;
    scan_config.dropout = 0.01;
    scan_config.seed = 7;
    const auto ground_truth = transformFromJson(report.at("ground_truth"));
    const auto initial = transformFromJson(report.at("initial"));
    const auto estimated = transformFromJson(report.at("estimated"));
    const auto camera_points = auto_calib::projectDepth(
        frame.depth, frame.camera, scan_config.pixel_stride);
    const auto scan =
        auto_calib::generateScan(camera_points, ground_truth, scan_config);

    fs::create_directories(args.output);
    cv::imwrite((args.output / "original_rgb.png").string(),
                labeled(frame.rgb, "Original RGB"));
    std::size_t initial_projected = 0, estimated_projected = 0;
    const auto initial_image =
        renderOverlay(frame.rgb, frame.camera, scan, initial, {255, 80, 0},
                      "Initial mechanical prior (blue)", &initial_projected);
    const auto estimated_image =
        renderOverlay(frame.rgb, frame.camera, scan, estimated, {0, 220, 80},
                      "Calibrated extrinsic (green)", &estimated_projected);
    cv::imwrite((args.output / "initial_pointcloud_overlay.png").string(),
                initial_image);
    cv::imwrite((args.output / "calibrated_pointcloud_overlay.png").string(),
                estimated_image);

    const int target_width = 640;
    const double scale = static_cast<double>(target_width) / frame.rgb.cols;
    const cv::Size panel_size{target_width,
                              static_cast<int>(frame.rgb.rows * scale)};
    cv::Mat comparison(panel_size.height, panel_size.width * 3, CV_8UC3,
                       cv::Scalar(24, 24, 24));
    cv::Mat panels[] = {labeled(frame.rgb, "1. Original RGB"), initial_image,
                        estimated_image};
    for (int index = 0; index < 3; ++index) {
      cv::Mat resized;
      cv::resize(panels[index], resized, panel_size, 0, 0, cv::INTER_AREA);
      resized.copyTo(comparison(cv::Rect(index * panel_size.width, 0,
                                         panel_size.width, panel_size.height)));
    }
    cv::imwrite((args.output / "calibration_comparison.png").string(),
                comparison);

    const auto lidar_points = makeExportPoints(scan);
    const auto calibrated_camera_points = makeExportPoints(scan, &estimated);
    writePly(args.output / "pointcloud_lidar.ply", lidar_points, frame_id);
    writeObj(args.output / "pointcloud_lidar.obj", lidar_points, frame_id);
    writePly(args.output / "pointcloud_calibrated_camera.ply",
             calibrated_camera_points, frame_id);
    writeObj(args.output / "pointcloud_calibrated_camera.obj",
             calibrated_camera_points, frame_id);

    const json summary = {
        {"frame_id", frame_id},
        {"valid_scan_points", scan.valid_count},
        {"initial_projected_points", initial_projected},
        {"calibrated_projected_points", estimated_projected},
        {"files",
         {"original_rgb.png", "initial_pointcloud_overlay.png",
          "calibrated_pointcloud_overlay.png", "calibration_comparison.png",
          "pointcloud_lidar.ply", "pointcloud_lidar.obj",
          "pointcloud_calibrated_camera.ply",
          "pointcloud_calibrated_camera.obj"}},
        {"coordinate_frames",
         {{"pointcloud_lidar.ply", "lidar_scan"},
          {"pointcloud_lidar.obj", "lidar_scan"},
          {"pointcloud_calibrated_camera.ply", "camera_optical"},
          {"pointcloud_calibrated_camera.obj", "camera_optical"}}}};
    std::ofstream(args.output / "visualization_summary.json")
        << std::setw(2) << summary << '\n';
    std::cout << std::setw(2) << summary << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}

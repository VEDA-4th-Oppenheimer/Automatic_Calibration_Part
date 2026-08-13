#include "auto_calib/calibration_core.hpp"
#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>
namespace {
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
Eigen::Vector3d ray(double pan, double tilt) {
  double c = std::cos(tilt);
  return {c * std::sin(pan), -std::sin(tilt), c * std::cos(pan)};
}
auto_calib::Scan makePlaneCornerScan(bool separated_parallel_planes) {
  auto_calib::Scan scan;
  scan.config.rows = 30;
  scan.config.columns = 31;
  scan.points.resize(static_cast<std::size_t>(scan.config.rows) *
                     scan.config.columns);
  constexpr std::uint32_t middle = 15;
  constexpr double step = 0.05;
  for (std::uint32_t row = 0; row < scan.config.rows; ++row)
    for (std::uint32_t column = 0; column < scan.config.columns; ++column) {
      const double y =
          (static_cast<double>(row) - 0.5 * (scan.config.rows - 1)) * step;
      Eigen::Vector3d xyz;
      if (separated_parallel_planes) {
        xyz = {(static_cast<double>(column) - middle) * step, y,
               column <= middle ? 2.0 : 3.0};
      } else if (column <= middle) {
        xyz = {(static_cast<double>(column) - middle) * step, y, 2.0};
      } else {
        xyz = {0.0, y, 2.0 + (static_cast<double>(column) - middle) * step};
      }
      auto &point = scan.points[static_cast<std::size_t>(row) *
                                    scan.config.columns +
                                column];
      point.xyz = xyz.cast<float>();
      point.range = static_cast<float>(xyz.norm());
      point.row = row;
      point.column = column;
      point.flags = auto_calib::kValidRange;
      ++scan.valid_count;
    }
  scan.source_count = scan.valid_count;
  return scan;
}
auto_calib::Scan makeFragmentedPlaneScan() {
  auto scan = makePlaneCornerScan(true);
  for (std::uint32_t row = 0; row < scan.config.rows; ++row)
    for (std::uint32_t column = 0; column < scan.config.columns; ++column) {
      auto &point = scan.points[static_cast<std::size_t>(row) *
                                    scan.config.columns +
                                column];
      point.xyz.z() = 2.0F;
      point.range = column < scan.config.columns - 2
                        ? point.xyz.norm()
                        : point.xyz.norm() + 1.0F;
    }
  return scan;
}
auto_calib::Scan makeFragmentedHorizontalScan() {
  auto scan = makePlaneCornerScan(true);
  for (std::uint32_t row = 0; row < scan.config.rows; ++row)
    for (std::uint32_t column = 0; column < scan.config.columns; ++column) {
      auto &point = scan.points[static_cast<std::size_t>(row) *
                                    scan.config.columns +
                                column];
      point.xyz = {(static_cast<float>(column) - 15.0F) * 0.05F, 1.2F,
                   (static_cast<float>(row) - 14.5F) * 0.05F + 2.0F};
      point.range = point.xyz.norm() +
                    static_cast<float>(column / 5) * 1.0F;
    }
  return scan;
}
} // namespace
int main() {
  try {
    auto_calib::CalibrationConfig plane_cfg;
    plane_cfg.minimum_lidar_plane_points = 50;
    plane_cfg.maximum_lidar_plane_rms_error_m = 0.005;
    plane_cfg.lidar_plane_neighbor_distance_threshold_m = 0.02;
    const auto corner_scan = makePlaneCornerScan(false);
    const auto corner_planes =
        auto_calib::segmentLidarPlanes(corner_scan, plane_cfg);
    const auto corner_edges =
        auto_calib::extractLidarPlaneIntersectionSegments(
            corner_scan, corner_planes, plane_cfg);
    require(corner_planes.planes.size() == 2,
            "Perpendicular plane segmentation failed");
    require(corner_edges.size() == 1,
            "Plane intersection structural edge missing");
    require(std::abs((corner_edges.front().b - corner_edges.front().a)
                         .normalized()
                         .y()) > 0.9,
            "Plane intersection direction is incorrect");
    const auto separated_scan = makePlaneCornerScan(true);
    const auto separated_planes =
        auto_calib::segmentLidarPlanes(separated_scan, plane_cfg);
    require(separated_planes.planes.size() == 2,
            "Separated parallel planes were not segmented");
    require(auto_calib::extractLidarPlaneIntersectionSegments(
                separated_scan, separated_planes, plane_cfg)
                .empty(),
            "Parallel occlusion surfaces produced a false intersection");
    require(!auto_calib::extractLidarOcclusionSegments(separated_scan,
                                                       plane_cfg)
                 .empty(),
            "Range-discontinuity diagnostic edge missing");
    auto fragmented_cfg = plane_cfg;
    fragmented_cfg.minimum_lidar_plane_points = 100;
    const auto fragmented_scan = makeFragmentedPlaneScan();
    const auto recovered_plane =
        auto_calib::segmentLidarPlanes(fragmented_scan, fragmented_cfg);
    require(recovered_plane.planes.size() == 1,
            "Coplanar fragments were not merged");
    require(recovered_plane.planes.front().support_points ==
                fragmented_scan.valid_count,
            "Rejected coplanar points were not reassigned");
    auto horizontal_cfg = plane_cfg;
    horizontal_cfg.minimum_lidar_plane_points = 200;
    const auto horizontal_scan = makeFragmentedHorizontalScan();
    const auto horizontal_plane =
        auto_calib::segmentLidarPlanes(horizontal_scan, horizontal_cfg);
    require(horizontal_plane.planes.size() == 1,
            "Height-clustered horizontal plane was not recovered");
    require(std::abs(horizontal_plane.planes.front().normal.y()) > 0.95,
            "Recovered horizontal plane normal is incorrect");

    auto_calib::ScanConfig sc;
    sc.rows = 31;
    sc.columns = 41;
    sc.pan_min = -.35;
    sc.pan_max = .35;
    sc.tilt_min = -.25;
    sc.tilt_max = .25;
    sc.noise_stddev = 0;
    sc.dropout = 0;
    std::vector<Eigen::Vector3d> source;
    for (std::uint32_t r = 0; r < sc.rows; ++r)
      for (std::uint32_t c = 0; c < sc.columns; ++c) {
        double pan = sc.pan_min +
                     (sc.pan_max - sc.pan_min) * c / (sc.columns - 1),
               tilt = sc.tilt_max -
                      (sc.tilt_max - sc.tilt_min) * r / (sc.rows - 1);
        double range = (c > 10 && c < 30 && r > 7 && r < 23) ? 3.0 : 5.0;
        if (c > 16 && c < 24 && r > 11 && r < 19)
          range = 2.2;
        source.push_back(ray(pan, tilt) * range);
      }
    auto scan = auto_calib::generateScan(source, {}, sc);
    auto_calib::CalibrationConfig cfg;
    cfg.minimum_lidar_edge_points = 10;
    cfg.minimum_camera_edge_pixels = 10;
    auto edges = auto_calib::extractLidarEdgePoints(scan, cfg);
    require(edges.size() > 20, "LiDAR edges missing");
    auto_calib::CameraModel camera;
    camera.width = 400;
    camera.height = 400;
    camera.k << 360, 0, 200, 0, 360, 200, 0, 0, 1;
    cv::Mat image(400, 400, CV_8UC3, cv::Scalar(0, 0, 0));
    for (const auto &p : edges) {
      int u = std::lround(camera.k(0, 0) * p.x() / p.z() + camera.k(0, 2)),
          v = std::lround(camera.k(1, 1) * p.y() / p.z() + camera.k(1, 2));
      if (u >= 0 && v >= 0 && u < 400 && v < 400)
        cv::circle(image, {u, v}, 2, {255, 255, 255}, -1);
    }
    auto initial =
        auto_calib::makeTransform({.01, -.01, .015}, {.005, -.006, .008});
    cfg.maximum_mean_edge_distance_px = 25;
    cfg.minimum_projected_ratio = .5;
    cfg.maximum_solver_iterations = 30;
    auto result =
        auto_calib::calibrateExtrinsic(image, camera, scan, initial, cfg);
    require(result.metrics.camera_edge_pixels > 10, "Camera edges missing");
    require(result.metrics.final_mean_edge_distance_px <=
                result.metrics.initial_mean_edge_distance_px * 1.05,
            "Objective did not improve");
    require(result.success, "Synthetic calibration failed");
    std::vector<auto_calib::CalibrationObservation> observations = {
        {image, camera, scan}, {image, camera, scan}};
    auto multi =
        auto_calib::calibrateExtrinsicMultiScene(observations, initial, cfg);
    require(multi.success, "Multi-scene calibration failed");
    auto joint_intrinsics = cfg;
    joint_intrinsics.optimize_camera_intrinsics = true;
    joint_intrinsics.minimum_intrinsic_observations = 3;
    auto insufficient_intrinsics = auto_calib::calibrateExtrinsicMultiScene(
        observations, initial, joint_intrinsics);
    require(!insufficient_intrinsics.success &&
                insufficient_intrinsics.reason_code ==
                    "INTRINSIC_OBSERVATIONS_INSUFFICIENT",
            "Joint intrinsic observation gate failed");
    require(multi.metrics.lidar_geometry_points > 0 &&
                multi.metrics.nid_projected_points > 0 &&
                std::isfinite(multi.metrics.final_nid),
            "Geometry NID path was not evaluated");
    auto multistart_cfg = cfg;
    multistart_cfg.coarse_yaw_span_rad = 3.14159265358979323846;
    multistart_cfg.coarse_yaw_step_rad = 0.7853981633974483;
    multistart_cfg.rotation_search_bound_rad = 3.2;
    multistart_cfg.maximum_rotation_update_rad = 3.2;
    multistart_cfg.prior_weight = 0.0;
    multistart_cfg.camera_direction_prior_weight = 0.35;
    multistart_cfg.expected_camera_forward_lidar = Eigen::Vector3d::UnitZ();
    multistart_cfg.maximum_solver_iterations = 150;
    auto wrong_heading = auto_calib::makeTransform(
        initial.translation_m, {0.0, 3.14159265358979323846, 0.0});
    auto recovered_heading = auto_calib::calibrateExtrinsicMultiScene(
        observations, wrong_heading, multistart_cfg);
    require(recovered_heading.metrics.multistart_candidates >= 8,
            "Yaw multi-start candidates were not evaluated");
    require(recovered_heading.coarse_orientation_scores.size() ==
                recovered_heading.metrics.multistart_candidates,
            "Yaw candidate score map was not retained");
    require(std::any_of(recovered_heading.coarse_orientation_scores.begin(),
                        recovered_heading.coarse_orientation_scores.end(),
                        [](const auto &score) {
                          return score.visible_edge_points > 0 &&
                                 std::isfinite(score.structural_line_objective);
                        }),
            "Visibility/structural candidate diagnostics missing");
    require(std::any_of(recovered_heading.coarse_orientation_scores.begin(),
                        recovered_heading.coarse_orientation_scores.end(),
                        [](const auto &score) { return score.overlap_valid; }),
            "Yaw overlap gate rejected every valid synthetic candidate");
    require(
        std::isfinite(recovered_heading.metrics.selected_multistart_yaw_deg),
        "Selected yaw candidate was not recorded");
    require(recovered_heading.metrics.final_composite_objective <=
                recovered_heading.metrics.initial_composite_objective,
            "Yaw multi-start did not improve the composite objective");
    require(recovered_heading.success,
            "Yaw multi-start failed to recover a valid heading");
    const Eigen::Vector3d recovered_forward =
        recovered_heading.estimated_t_camera_lidar.rotation.transpose() *
        Eigen::Vector3d::UnitZ();
    require(recovered_forward.dot(Eigen::Vector3d::UnitZ()) > 0.8,
            "Camera direction prior selected the opposite reproject direction");
    const auto &direction_scores =
        recovered_heading.coarse_orientation_scores;
    const auto expected_candidate = std::min_element(
        direction_scores.begin(), direction_scores.end(),
        [](const auto &a, const auto &b) {
          return a.direction_prior_objective < b.direction_prior_objective;
        });
    const auto opposite_candidate = std::max_element(
        direction_scores.begin(), direction_scores.end(),
        [](const auto &a, const auto &b) {
          return a.direction_prior_objective < b.direction_prior_objective;
        });
    require(expected_candidate != direction_scores.end() &&
                opposite_candidate != direction_scores.end() &&
                expected_candidate->direction_prior_objective < 1e-6 &&
                opposite_candidate->direction_prior_objective > 0.9,
            "Camera direction prior did not distinguish opposite candidates");
    auto center_cfg = multistart_cfg;
    center_cfg.use_camera_center_prior = true;
    center_cfg.expected_camera_center_lidar = {0.06, -0.08, 0.01};
    center_cfg.camera_center_prior_sigma_m = 0.005;
    auto center_constrained = auto_calib::calibrateExtrinsicMultiScene(
        observations, wrong_heading, center_cfg);
    const auto &center_transform = center_constrained.success
                                       ? center_constrained.estimated_t_camera_lidar
                                       : center_constrained.candidate_t_camera_lidar;
    const Eigen::Vector3d recovered_center =
        -center_transform.rotation.transpose() * center_transform.translation_m;
    require((recovered_center - center_cfg.expected_camera_center_lidar).norm() <
                0.01,
            "Yaw search moved the physical camera center");
    observations.push_back({image, camera, scan});
    auto joint = auto_calib::calibrateExtrinsicMultiScene(observations, initial,
                                                          joint_intrinsics);
    require(joint.success, "Joint intrinsic calibration failed");
    require(std::isfinite(joint.estimated_camera.k(0, 0)) &&
                std::isfinite(joint.estimated_camera.k(1, 1)),
            "Joint intrinsic estimate is invalid");
    auto strict_objective = cfg;
    strict_objective.minimum_objective_improvement_ratio = 1.0;
    auto rejected_objective = auto_calib::calibrateExtrinsicMultiScene(
        observations, initial, strict_objective);
    require(!rejected_objective.success &&
                rejected_objective.reason_code ==
                    "OBJECTIVE_IMPROVEMENT_INSUFFICIENT",
            "Objective improvement gate failed");
    auto strict = cfg;
    strict.maximum_rotation_update_rad = 0.0;
    strict.maximum_translation_update_m = 0.0;
    auto rejected_update =
        auto_calib::calibrateExtrinsicMultiScene(observations, initial, strict);
    require(!rejected_update.success, "Unsafe update gate failed");
    auto fallback_error = auto_calib::calculatePoseError(
        rejected_update.estimated_t_camera_lidar, initial);
    require(fallback_error.translation_m < 1e-12 &&
                fallback_error.rotation_deg < 1e-12,
            "Rejected candidate did not fall back to prior");
    require(std::isfinite(
                rejected_update.candidate_t_camera_lidar.translation_m.x()),
            "Rejected diagnostic candidate was not retained");
    cv::Mat blank(400, 400, CV_8UC3, cv::Scalar(0));
    auto rejected =
        auto_calib::calibrateExtrinsic(blank, camera, scan, {}, cfg);
    require(!rejected.success &&
                rejected.reason_code == "CAMERA_EDGE_INSUFFICIENT",
            "Blank image gate failed");
    require(rejected.metrics.runtime_ms > 0.0,
            "Rejected input runtime was not recorded");
    auto error = auto_calib::calculatePoseError({}, {});
    require(error.translation_m < 1e-12 && error.rotation_deg < 1e-12,
            "Pose error failed");
    std::cout << "All Calibration Core tests passed.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Test failure: " << e.what() << '\n';
    return 1;
  }
}

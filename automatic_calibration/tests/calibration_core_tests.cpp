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
auto_calib::Scan makeGrazingPlaneScan() {
  auto_calib::Scan scan;
  scan.config.rows = 20;
  scan.config.columns = 20;
  scan.points.resize(static_cast<std::size_t>(scan.config.rows) *
                     scan.config.columns);
  for (std::uint32_t row = 0; row < scan.config.rows; ++row)
    for (std::uint32_t column = 0; column < scan.config.columns; ++column) {
      const double pan = 0.10 + 0.40 * column / (scan.config.columns - 1);
      const double tilt =
          -0.20 + 0.40 * row / (scan.config.rows - 1);
      const Eigen::Vector3d direction = ray(pan, tilt);
      const double range = 1.0 / direction.x();
      auto &point = scan.points[static_cast<std::size_t>(row) *
                                    scan.config.columns +
                                column];
      point.xyz = (range * direction).cast<float>();
      point.range = static_cast<float>(range);
      point.row = row;
      point.column = column;
      point.flags = auto_calib::kValidRange;
      ++scan.valid_count;
    }
  scan.source_count = scan.valid_count;
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
    auto one_plane_cfg = plane_cfg;
    one_plane_cfg.minimum_lidar_plane_points = 470;
    const auto one_plane_segmentation =
        auto_calib::segmentLidarPlanes(corner_scan, one_plane_cfg);
    const auto plane_boundaries =
        auto_calib::extractLidarPlaneBoundarySegments(
            corner_scan, one_plane_segmentation, one_plane_cfg);
    require(one_plane_segmentation.planes.size() == 1,
            "Plane-boundary fixture did not retain exactly one plane");
    require(!plane_boundaries.empty(),
            "Accepted-plane boundary structural edge missing");
    require(plane_boundaries.front().source ==
                auto_calib::StructuralLineSource::PlaneBoundary &&
                std::abs((plane_boundaries.front().b -
                          plane_boundaries.front().a)
                             .normalized()
                             .y()) > 0.9,
            "Plane-boundary segment metadata or direction is incorrect");

    std::vector<std::vector<auto_calib::StructuralLineSegment3d>>
        occlusion_observations(4);
    for (std::size_t observation = 0; observation < 4; ++observation) {
      const double jitter = static_cast<double>(observation) * 0.005;
      occlusion_observations[observation].push_back(
          {{jitter, 0.0, 2.0}, {jitter, 1.0, 2.0},
           auto_calib::StructuralLineSource::OcclusionCandidate, 1.0, 1});
      occlusion_observations[observation].push_back(
          {{1.0 + observation, 0.0, 1.0},
           {1.0 + observation, 0.5, 1.0},
           auto_calib::StructuralLineSource::OcclusionCandidate, 1.0, 1});
    }
    auto persistence_cfg = plane_cfg;
    persistence_cfg.minimum_persistent_occlusion_observations = 3;
    persistence_cfg.minimum_persistent_occlusion_observation_ratio = 0.75;
    const auto persistent =
        auto_calib::retainPersistentLidarOcclusionSegments(
            occlusion_observations, persistence_cfg);
    require(std::all_of(persistent.begin(), persistent.end(),
                        [](const auto &segments) {
                          return segments.size() == 1 &&
                                 segments.front().source ==
                                     auto_calib::StructuralLineSource::
                                         PersistentOcclusion &&
                                 segments.front().support_observations == 4;
                        }),
            "Persistent occlusion filtering retained transient segments");
    occlusion_observations.resize(2);
    const auto insufficient_persistence =
        auto_calib::retainPersistentLidarOcclusionSegments(
            occlusion_observations, persistence_cfg);
    require(std::all_of(insufficient_persistence.begin(),
                        insufficient_persistence.end(),
                        [](const auto &segments) { return segments.empty(); }),
            "Persistent occlusion gate accepted too few observations");
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
    auto grazing_plane = makeGrazingPlaneScan();
    auto grazing_cfg = cfg;
    grazing_cfg.lidar_edge_absolute_threshold_m = 0.02;
    grazing_cfg.lidar_edge_relative_threshold = 0.0;
    const auto grazing_segmentation =
        auto_calib::segmentLidarPlanes(grazing_plane, grazing_cfg);
    std::size_t grazing_raw_threshold_crossings = 0;
    for (std::uint32_t row = 0; row < grazing_plane.config.rows; ++row)
      for (std::uint32_t column = 0;
           column + 1 < grazing_plane.config.columns; ++column) {
        const auto index = static_cast<std::size_t>(row) *
                               grazing_plane.config.columns +
                           column;
        grazing_raw_threshold_crossings += static_cast<std::size_t>(
            std::abs(grazing_plane.points[index].range -
                     grazing_plane.points[index + 1].range) >
            grazing_cfg.lidar_edge_absolute_threshold_m);
      }
    require(grazing_raw_threshold_crossings > 100,
            "Grazing-plane regression fixture has no raw false edges");
    const auto grazing_edges = auto_calib::extractLidarEdgePoints(
        grazing_plane, grazing_segmentation, grazing_cfg);
    require(grazing_edges.empty(),
            "Coplanar grazing-range changes were misclassified as edges");
    auto invalid_edge_cfg = grazing_cfg;
    invalid_edge_cfg.lidar_edge_minimum_local_contrast_ratio = 0.5;
    auto invalid_edge_result = auto_calib::calibrateExtrinsic(
        {}, {}, grazing_plane, {}, invalid_edge_cfg);
    require(invalid_edge_result.reason_code == "INVALID_LIDAR_EDGE_CONFIG",
            "Invalid LiDAR edge configuration was not rejected");
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
    auto invalid_coverage_cfg = cfg;
    invalid_coverage_cfg.minimum_relative_nid_coverage = 1.1;
    const auto invalid_coverage_result =
        auto_calib::calibrateExtrinsicMultiScene(observations, initial,
                                                 invalid_coverage_cfg);
    require(!invalid_coverage_result.success &&
                invalid_coverage_result.reason_code ==
                    "INVALID_COVERAGE_CONFIG",
            "Invalid relative coverage configuration was not rejected");
    auto joint_intrinsics = cfg;
    joint_intrinsics.optimize_camera_intrinsics = true;
    joint_intrinsics.enable_experimental_joint_intrinsics = true;
    joint_intrinsics.minimum_intrinsic_observations = 3;
    auto blocked_joint_intrinsics = cfg;
    blocked_joint_intrinsics.optimize_camera_intrinsics = true;
    blocked_joint_intrinsics.minimum_intrinsic_observations = 3;
    const auto blocked_joint = auto_calib::calibrateExtrinsicMultiScene(
        observations, initial, blocked_joint_intrinsics);
    require(!blocked_joint.success &&
                blocked_joint.reason_code ==
                    "JOINT_INTRINSIC_EXPERIMENTAL_DISABLED",
            "Joint intrinsic research path was not disabled by default");
    auto insufficient_intrinsics = auto_calib::calibrateExtrinsicMultiScene(
        observations, initial, joint_intrinsics);
    require(!insufficient_intrinsics.success &&
                insufficient_intrinsics.reason_code ==
                    "INTRINSIC_OBSERVATIONS_INSUFFICIENT",
            "Joint intrinsic observation gate failed");
    auto score_map_only = cfg;
    score_map_only.enable_ceres_refinement = false;
    const auto score_map_result = auto_calib::calibrateExtrinsicMultiScene(
        observations, initial, score_map_only);
    require(!score_map_result.success && score_map_result.candidate_available &&
                score_map_result.state == "SCORE_MAP_ONLY" &&
                score_map_result.reason_code == "COARSE_SCORE_ONLY",
            "Coarse score-map mode did not stop before Ceres");
    require(multi.metrics.lidar_geometry_points > 0 &&
                multi.metrics.nid_projected_points > 0 &&
                std::isfinite(multi.metrics.final_nid),
            "Geometry NID path was not evaluated");
    require(multi.metrics.final_range_nid >= 0.0 &&
                multi.metrics.final_normal_nid >= 0.0 &&
                multi.metrics.nid_active_spatial_cells >= 2,
            "Range/normal spatial NID channels were not both evaluated");
    require(multi.metrics.edge_active_spatial_cells > 0 &&
                multi.metrics.max_coarse_visible_edge_points >=
                    multi.metrics.visible_edge_points &&
                multi.metrics.max_coarse_nid_projected_points >=
                    multi.metrics.nid_projected_points &&
                multi.metrics.edge_coverage_ratio > 0.0 &&
                multi.metrics.nid_coverage_ratio > 0.0 &&
                multi.metrics.edge_spatial_coverage_ratio > 0.0,
            "Relative edge/NID/spatial coverage diagnostics were not evaluated");
    require(multi.metrics.structural_matched_segments <=
                multi.metrics.structural_visible_segments &&
                multi.metrics.structural_projected_points ==
                    multi.metrics.structural_matched_segments,
            "Structural one-to-one match metrics are inconsistent");
    const auto scene_metrics = auto_calib::evaluateCalibrationPoseScenes(
        observations, multi.candidate_t_camera_lidar, cfg);
    require(scene_metrics.size() == observations.size() &&
                std::all_of(scene_metrics.begin(), scene_metrics.end(),
                            [](const auto &metrics) {
                              return metrics.visible_edge_points > 0 &&
                                     metrics.nid_projected_points > 0 &&
                                     std::isfinite(
                                         metrics.mean_edge_distance_px);
                            }),
            "Fixed-pose per-scene validation metrics were not evaluated");
    auto signal_scan = scan;
    cv::Mat signal_image(400, 400, CV_8UC3);
    for (int v = 0; v < signal_image.rows; ++v)
      for (int u = 0; u < signal_image.cols; ++u) {
        const auto intensity = static_cast<unsigned char>(
            255.0 * u / (signal_image.cols - 1));
        signal_image.at<cv::Vec3b>(v, u) =
            cv::Vec3b(intensity, intensity, intensity);
      }
    cv::rectangle(signal_image, {15, 15}, {384, 384}, {255, 255, 255}, 3,
                  cv::LINE_AA);
    for (auto &point : signal_scan.points)
      if (point.valid()) {
        const double normalized_pan =
            (point.pan - sc.pan_min) / (sc.pan_max - sc.pan_min);
        point.signal_strength =
            static_cast<float>(1000.0 * std::exp(2.0 * normalized_pan));
      }
    auto signal_cfg = cfg;
    signal_cfg.signal_nmi_weight = 0.15;
    signal_cfg.minimum_signal_nmi_projected_points = 50;
    signal_cfg.minimum_signal_entropy_ratio = 0.01;
    signal_cfg.minimum_signal_nmi_improvement_ratio = -1.0;
    std::vector<auto_calib::CalibrationObservation> signal_observations = {
        {signal_image, camera, signal_scan},
        {signal_image, camera, signal_scan}};
    const auto signal_result = auto_calib::calibrateExtrinsicMultiScene(
        signal_observations, {}, signal_cfg);
    require(signal_result.metrics.lidar_signal_points > 100 &&
                signal_result.metrics.signal_nmi_projected_points >= 100 &&
                signal_result.metrics.signal_nmi_active_spatial_cells >= 2 &&
                signal_result.metrics.final_signal_nmi < 1.0,
            "Corrected signal-strength NMI path was not evaluated");
    cv::Mat manhattan_image = image.clone();
    for (int x = 70; x <= 330; x += 65)
      cv::line(manhattan_image, {x, 40}, {x, 360}, {255, 255, 255}, 3,
               cv::LINE_AA);
    std::vector<auto_calib::CalibrationObservation> manhattan_observations = {
        {manhattan_image, camera, scan}, {manhattan_image, camera, scan},
        {manhattan_image, camera, scan}};
    auto manhattan_cfg = cfg;
    manhattan_cfg.manhattan_direction_weight = 0.25;
    manhattan_cfg.minimum_manhattan_vertical_inliers = 3;
    manhattan_cfg.maximum_manhattan_vertical_error_rad =
        10.0 * 3.14159265358979323846 / 180.0;
    const auto vanishing_diagnostics =
        auto_calib::detectManhattanVanishingDirections(
            manhattan_image, camera, manhattan_cfg);
    require(!vanishing_diagnostics.empty() &&
                vanishing_diagnostics.front().inliers >= 3,
            "Manhattan vanishing-direction diagnostics were not retained");
    const auto manhattan_result = auto_calib::calibrateExtrinsicMultiScene(
        manhattan_observations, initial, manhattan_cfg);
    require(manhattan_result.metrics.manhattan_vertical_inliers >= 9 &&
                manhattan_result.metrics.final_manhattan_vertical_error_deg >=
                    0.0 &&
                manhattan_result.metrics.final_manhattan_vertical_error_deg <
                    10.0,
            "Manhattan gravity/vertical-line constraint was not evaluated");
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
    if (!recovered_heading.success)
      std::cerr << "Yaw recovery reason=" << recovered_heading.reason_code
                << " edges=" << recovered_heading.metrics.lidar_edge_points
                << " edge_mean="
                << recovered_heading.metrics.final_mean_edge_distance_px
                << " nid_improvement="
                << recovered_heading.metrics.nid_improvement_ratio
                << " candidate_delta="
                << auto_calib::calculatePoseError(
                       recovered_heading.candidate_t_camera_lidar,
                       wrong_heading)
                       .translation_m
                << "m/"
                << auto_calib::calculatePoseError(
                       recovered_heading.candidate_t_camera_lidar,
                       wrong_heading)
                       .rotation_deg
                << "deg\n";
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
    strict_objective.maximum_solver_iterations = 100;
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

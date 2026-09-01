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

auto_calib::Scan makeFloorCeilingRoomScan() {
  auto_calib::Scan scan;
  scan.config.rows = 40;
  scan.config.columns = 40;
  scan.points.resize(static_cast<std::size_t>(scan.config.rows) *
                     scan.config.columns);
  constexpr double step = 0.05;
  for (std::uint32_t row = 0; row < scan.config.rows; ++row) {
    for (std::uint32_t column = 0; column < scan.config.columns; ++column) {
      Eigen::Vector3d xyz;
      if (row < 10) {
        // Ceiling plane: y = -0.50
        const double x = (static_cast<double>(column) - 20.0) * step;
        const double z = 2.0 + static_cast<double>(9 - row) * step;
        xyz = {x, -0.50, z};
      } else if (row >= 30) {
        // Floor plane: y = 1.00
        const double x = (static_cast<double>(column) - 20.0) * step;
        const double z = 2.0 + static_cast<double>(row - 30) * 0.08;
        xyz = {x, 1.00, z};
      } else {
        const double y = -0.50 + static_cast<double>(row - 10) * 0.075;
        if (column < 20) {
          // Front wall: z = 2.0
          const double x = (static_cast<double>(column) - 20.0) * step;
          xyz = {x, y, 2.0};
        } else {
          // Side wall: x = 0.0
          const double z = 2.0 + static_cast<double>(column - 20) * step;
          xyz = {0.0, y, z};
        }
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
  }
  scan.source_count = scan.valid_count;
  return scan;
}
} // namespace
int main() {
  try {
    {
      auto_calib::CalibrationConfig objective_cfg;
      objective_cfg.edge_alignment_weight = 0.20;
      objective_cfg.normalized_information_distance_weight = 0.30;
      objective_cfg.signal_nmi_weight = 0.10;
      objective_cfg.structural_line_weight = 0.15;
      objective_cfg.manhattan_direction_weight = 0.05;
      objective_cfg.camera_direction_prior_weight = 0.0;
      objective_cfg.coverage_penalty_weight = 0.25;
      auto_calib::PoseSceneMetrics first;
      first.visible_edge_points = 100;
      first.edge_objective = 0.10;
      first.nid_projected_points = 50;
      first.edge_active_spatial_cells = 4;
      first.geometry_nid_objective = 0.20;
      first.signal_nmi_objective = 0.30;
      first.structural_objective = 0.40;
      first.structural_score_weight = 2.0;
      first.manhattan_objective = 0.50;
      first.manhattan_vertical_inliers = 3;
      auto_calib::PoseSceneMetrics second;
      second.visible_edge_points = 300;
      second.edge_objective = 0.30;
      second.nid_projected_points = 150;
      second.edge_active_spatial_cells = 8;
      second.geometry_nid_objective = 0.40;
      second.signal_nmi_objective = 0.50;
      second.structural_objective = 0.80;
      second.structural_score_weight = 1.0;
      const auto objective = auto_calib::summarizeCalibrationPoseScenes(
          {first, second}, {}, objective_cfg, 500, 250, 16);
      require(std::abs(objective.edge_objective - 0.25) < 1e-12,
              "Fixed-pose edge objective aggregation changed");
      require(std::abs(objective.geometry_nid_objective - 0.30) < 1e-12,
              "Fixed-pose NID objective aggregation changed");
      require(std::abs(objective.structural_objective - 8.0 / 15.0) <
                  1e-12,
              "Fixed-pose structural objective aggregation changed");
      require(std::abs(objective.coverage_objective - 0.0475) < 1e-12,
              "Fixed-pose common-reference coverage changed");
      require(std::abs(objective.composite_objective - 0.296875) < 1e-12,
              "Fixed-pose composite objective must match training weights");
    }

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
    cv::Mat orthogonal_manhattan_image(400, 400, CV_8UC3,
                                       cv::Scalar(0, 0, 0));
    for (int coordinate = 70; coordinate <= 330; coordinate += 65) {
      cv::line(orthogonal_manhattan_image, {coordinate, 40},
               {coordinate, 360}, {255, 255, 255}, 3, cv::LINE_AA);
      cv::line(orthogonal_manhattan_image, {40, coordinate},
               {360, coordinate}, {255, 255, 255}, 3, cv::LINE_AA);
    }
    const auto orthogonal_directions =
        auto_calib::detectManhattanVanishingDirections(
            orthogonal_manhattan_image, camera, manhattan_cfg);
    require(std::any_of(orthogonal_directions.begin(),
                        orthogonal_directions.end(), [](const auto &item) {
                          return std::abs(item.camera_direction.x()) > 0.95;
                        }) &&
                std::any_of(orthogonal_directions.begin(),
                            orthogonal_directions.end(), [](const auto &item) {
                              return std::abs(item.camera_direction.y()) >
                                     0.95;
                            }),
            "Manhattan prior regression fixture needs two orthogonal axes");
    const auto_calib::Transform evaluated_pose;
    const auto_calib::Transform vertical_feature_prior;
    auto horizontal_feature_prior = vertical_feature_prior;
    horizontal_feature_prior.rotation =
        Eigen::AngleAxisd(-0.5 * 3.14159265358979323846,
                          Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    const std::vector<auto_calib::CalibrationObservation>
        orthogonal_observations = {
            {orthogonal_manhattan_image, camera, scan}};
    const auto vertical_prior_metrics =
        auto_calib::evaluateCalibrationPoseScenes(
            orthogonal_observations, evaluated_pose, manhattan_cfg,
            &vertical_feature_prior);
    const auto horizontal_prior_metrics =
        auto_calib::evaluateCalibrationPoseScenes(
            orthogonal_observations, evaluated_pose, manhattan_cfg,
            &horizontal_feature_prior);
    require(vertical_prior_metrics.front().manhattan_vertical_error_deg <
                    5.0 &&
                horizontal_prior_metrics.front().manhattan_vertical_error_deg >
                    80.0,
            "Fixed-pose evaluation ignored the explicit Manhattan feature "
            "prior");
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
    // Milestone 1 (R1) Tests: Dominant Ground/Ceiling Planes, Ground Normal/Height Constraints, Asymmetric Line Weighting
    auto room_scan = makeFloorCeilingRoomScan();
    auto_calib::CalibrationConfig r1_cfg;
    r1_cfg.minimum_lidar_plane_points = 50;
    r1_cfg.enable_ground_plane_constraint = true;
    r1_cfg.enable_asymmetric_line_weighting = true;
    r1_cfg.ceiling_suppression_factor = 0.40;
    r1_cfg.vertical_corner_boost_factor = 1.80;

    auto room_planes = auto_calib::segmentLidarPlanes(room_scan, r1_cfg);
    require(room_planes.planes.size() >= 3, "Room scan plane segmentation failed");

    auto dominant = auto_calib::findDominantPlanes(room_planes, r1_cfg);
    require(dominant.has_ground, "Dominant ground plane not detected");
    require(std::abs(dominant.ground_y - 1.00) < 0.15, "Ground height y estimate incorrect");
    require(dominant.ground_normal.y() > 0.90, "Ground plane normal not pointing along gravity axis");
    require(dominant.has_ceiling, "Dominant ceiling plane not detected");
    require(std::abs(dominant.ceiling_y - (-0.50)) < 0.15, "Ceiling height y estimate incorrect");
    require(dominant.ceiling_normal.y() < -0.90, "Ceiling plane normal not pointing upward");

    // Test Ground Consistency Evaluator
    auto_calib::Transform valid_tf; // C_lidar = (0, 0, 0) with pitch = 20 deg
    valid_tf.rotation =
        Eigen::AngleAxisd(20.0 * M_PI / 180.0, Eigen::Vector3d::UnitX())
            .toRotationMatrix();
    auto valid_ground_eval =
        auto_calib::evaluateGroundConsistency(valid_tf, dominant, r1_cfg);
    require(valid_ground_eval.valid, "Valid camera pose rejected by ground evaluator");
    require(std::abs(valid_ground_eval.height_m - 1.00) < 0.15, "Ground height evaluation mismatch");
    require(std::abs(valid_ground_eval.tilt_deg - 20.0) < 5.0, "Ground tilt evaluation mismatch");
    require(std::abs(valid_ground_eval.downward_pitch_deg - 20.0) < 5.0, "Downward pitch evaluation mismatch");

    // Sub-ground camera (Cy = 1.5m > 1.0m -> h_cam = -0.5m < 0.8m)
    auto_calib::Transform subground_tf = valid_tf;
    subground_tf.translation_m = {0, -1.5, 0};
    auto subground_eval = auto_calib::evaluateGroundConsistency(subground_tf, dominant, r1_cfg);
    require(!subground_eval.valid, "Sub-ground camera pose was not rejected");

    // Camera exceeding max height (5.0m)
    auto_calib::Transform too_high_tf = valid_tf;
    too_high_tf.translation_m = {0, 5.0, 0}; // h_cam = 6.0m > 5.0m
    auto too_high_eval = auto_calib::evaluateGroundConsistency(too_high_tf, dominant, r1_cfg);
    require(!too_high_eval.valid, "Too high camera pose was not rejected");

    // Upside-down camera (tilt = 180 deg > 85 deg)
    auto_calib::Transform upside_down_tf;
    upside_down_tf.rotation =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto upside_down_eval = auto_calib::evaluateGroundConsistency(upside_down_tf, dominant, r1_cfg);
    require(!upside_down_eval.valid, "Upside down camera pose was not rejected");

    // Horizontal camera (pitch = 0 deg < 5 deg)
    auto_calib::Transform horizontal_cam_tf;
    auto horizontal_cam_eval = auto_calib::evaluateGroundConsistency(horizontal_cam_tf, dominant, r1_cfg);
    require(!horizontal_cam_eval.valid, "Horizontal camera pose (pitch < 5 deg) was not rejected");

    // Over-pitched camera (pitch = 70 deg > 60 deg)
    auto_calib::Transform overpitched_tf;
    overpitched_tf.rotation =
        Eigen::AngleAxisd(70.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    auto overpitched_eval = auto_calib::evaluateGroundConsistency(overpitched_tf, dominant, r1_cfg);
    require(!overpitched_eval.valid, "Overpitched camera pose (pitch > 60 deg) was not rejected");

    // Test Asymmetric Structural Feature Weighting
    auto room_structural_lines =
        auto_calib::extractLidarPlaneIntersectionSegments(room_scan, room_planes, r1_cfg);
    require(!room_structural_lines.empty(), "No structural lines extracted from room scan");
    double min_ceiling_conf = 100.0, max_ceiling_conf = 0.0;
    double min_floor_conf = 100.0, max_floor_conf = 0.0;
    double corner_conf = 0.0;
    for (const auto &seg : room_structural_lines) {
      const Eigen::Vector3d dir = (seg.b - seg.a).normalized();
      const Eigen::Vector3d mid = 0.5 * (seg.a + seg.b);
      if (std::abs(dir.y()) < 0.2 && mid.y() < 0.0) {
        // Ceiling line (suppressed)
        min_ceiling_conf = std::min(min_ceiling_conf, seg.confidence);
        max_ceiling_conf = std::max(max_ceiling_conf, seg.confidence);
      } else if (std::abs(dir.y()) < 0.2 && mid.y() > 0.5) {
        // Floor line (boosted)
        min_floor_conf = std::min(min_floor_conf, seg.confidence);
        max_floor_conf = std::max(max_floor_conf, seg.confidence);
      } else if (std::abs(dir.y()) > 0.8) {
        // Vertical corner (strongly boosted)
        corner_conf = std::max(corner_conf, seg.confidence);
      }
    }
    require(max_ceiling_conf < 1.15, "Ceiling line suppression not applied");
    require(min_floor_conf > 2.50, "Floor line boost not applied");
    require(corner_conf >= 4.00, "Vertical corner boost not applied");
    require(min_floor_conf > max_ceiling_conf * 2.0, "Floor to ceiling confidence ratio insufficient");
    require(corner_conf > max_ceiling_conf * 3.5, "Corner to ceiling confidence ratio insufficient");

    // Test calibrateExtrinsic rejection on ground constraint violation
    cv::Mat room_img(400, 400, CV_8UC3, cv::Scalar(128, 128, 128));
    cv::line(room_img, cv::Point(200, 0), cv::Point(200, 400), cv::Scalar(255, 255, 255), 2);
    auto_calib::Transform subground_prior = valid_tf;
    subground_prior.translation_m = {0, -1.5, 0};
    auto rejected_subground =
        auto_calib::calibrateExtrinsic(room_img, camera, room_scan, subground_prior, r1_cfg);
    require(!rejected_subground.success &&
                rejected_subground.reason_code == "GEOMETRY_GROUND_INCONSISTENT",
            "Ground plane constraint gate failed on subground prior");

    auto_calib::Transform upside_down_prior = valid_tf;
    upside_down_prior.rotation =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto rejected_upside_down =
        auto_calib::calibrateExtrinsic(room_img, camera, room_scan, upside_down_prior, r1_cfg);
    require(!rejected_upside_down.success &&
                rejected_upside_down.reason_code == "GEOMETRY_GROUND_INCONSISTENT",
            "Ground plane constraint gate failed on upside down prior");

    // Challenger 2 Stress Test Suite: Parameter bounds, extreme factors & toggle
    {
      const std::vector<double> test_suppressions = {0.0, 0.01, 0.40, 1.0, 2.0, 10.0, 100.0};
      const std::vector<double> test_boosts = {0.0, 0.1, 1.0, 1.80, 5.0, 50.0, 1000.0};
      const std::vector<double> test_weights = {-1.0, 0.0, 0.5, 1.0, 5.0, 100.0};

      for (double supp : test_suppressions) {
        for (double boost : test_boosts) {
          for (double w : test_weights) {
            auto_calib::CalibrationConfig stress_cfg = r1_cfg;
            stress_cfg.ceiling_suppression_factor = supp;
            stress_cfg.vertical_corner_boost_factor = boost;
            stress_cfg.asymmetric_feature_weight_factor = w;

            auto lines = auto_calib::extractLidarPlaneIntersectionSegments(
                room_scan, room_planes, stress_cfg);
            for (const auto &l : lines) {
              require(std::isfinite(l.confidence), "Stress: confidence non-finite");
              require(l.confidence >= 0.10, "Stress: confidence below lower clamp bound 0.10");
              require(l.confidence <= 5.0, "Stress: confidence above upper clamp bound 5.0");
            }
          }
        }
      }

      // Test asymmetric line weighting toggle off
      auto_calib::CalibrationConfig disabled_cfg = r1_cfg;
      disabled_cfg.enable_asymmetric_line_weighting = false;
      auto disabled_lines = auto_calib::extractLidarPlaneIntersectionSegments(
          room_scan, room_planes, disabled_cfg);
      for (const auto &l : disabled_lines) {
        require(std::abs(l.confidence - 1.0) < 1e-6, "Disabled asymmetric weighting must yield 1.0");
      }

      // Test rotation sphere ground consistency validity
      for (double yaw_deg = -180.0; yaw_deg <= 180.0; yaw_deg += 30.0) {
        for (double pitch_deg = -90.0; pitch_deg <= 90.0; pitch_deg += 30.0) {
          auto_calib::Transform tf;
          tf.rotation =
              (Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitX()))
                  .toRotationMatrix();
          tf.translation_m = {0.0, 0.2, 0.0};
          auto eval = auto_calib::evaluateGroundConsistency(tf, dominant, r1_cfg);
          require(std::isfinite(eval.height_m) && std::isfinite(eval.tilt_deg),
                  "Tilt/height evaluation non-finite");
          const Eigen::Vector3d n_cam = tf.rotation * dominant.ground_normal;
          if (n_cam.y() <= 0.0 || eval.tilt_deg > r1_cfg.maximum_camera_ground_tilt_deg) {
            require(!eval.valid, "Inconsistent ground evaluation validity");
          }
        }
      }
    }

    // Milestone 2 (R2) Tests:
    // 1. Normal-Gated Line Matching & Dihedral Normal Retention
    {
      auto r2_lines =
          auto_calib::extractLidarPlaneIntersectionSegments(room_scan, room_planes, r1_cfg);
      require(!r2_lines.empty(), "R2: Structural lines missing from room scan");
      bool has_dihedral_normals = false;
      for (const auto &line : r2_lines) {
        if (line.has_plane_normals) {
          has_dihedral_normals = true;
          require(line.n1.norm() > 0.9 && line.n1.norm() < 1.1,
                  "R2: Plane normal n1 not unit length");
          require(line.n2.norm() > 0.9 && line.n2.norm() < 1.1,
                  "R2: Plane normal n2 not unit length");
        }
      }
      require(has_dihedral_normals,
              "R2: Plane intersection lines must retain dihedral plane normals");

      // 2. Normal-Gated Line Matching: Vertical 3D Line vs Horizontal 2D Edge
      // Create an image with ONLY horizontal lines
      cv::Mat horiz_only_img(400, 400, CV_8UC3, cv::Scalar(0, 0, 0));
      for (int y = 50; y <= 350; y += 50) {
        cv::line(horiz_only_img, cv::Point(50, y), cv::Point(350, y),
                 cv::Scalar(255, 255, 255), 2);
      }

      auto_calib::CalibrationConfig r2_gated_cfg;
      r2_gated_cfg.enable_normal_gated_line_matching = true;
      r2_gated_cfg.structural_normal_weight = 0.25;

      // When evaluating vertical 3D lines against horizontal-only 2D image,
      // normal gating must prevent false matching of vertical 3D lines to horizontal 2D lines
      std::vector<auto_calib::CalibrationObservation> vert_horiz_obs = {
          {horiz_only_img, camera, room_scan}};
      auto scene_eval_gated = auto_calib::evaluateCalibrationPoseScenes(
          vert_horiz_obs, valid_tf, r2_gated_cfg);
      require(scene_eval_gated.front().vertical_structural_matches == 0,
              "R2: Normal gating failed to reject vertical 3D line matched against horizontal 2D lines");

      // Now create an image with a matching vertical line at x = 200
      cv::Mat vert_matching_img(400, 400, CV_8UC3, cv::Scalar(0, 0, 0));
      cv::line(vert_matching_img, cv::Point(200, 50), cv::Point(200, 350),
               cv::Scalar(255, 255, 255), 2);
      std::vector<auto_calib::CalibrationObservation> vert_matching_obs = {
          {vert_matching_img, camera, room_scan}};
      auto scene_eval_matched = auto_calib::evaluateCalibrationPoseScenes(
          vert_matching_obs, valid_tf, r2_gated_cfg);
      require(scene_eval_matched.front().vertical_structural_matches >= 1,
              "R2: Normal-gated matching failed to match correctly oriented vertical line");
      require(scene_eval_matched.front().total_explained_structural_length > 50.0,
              "R2: Total explained structural length (TESL) must be positive on match");

      // 3. Coverage-Weighted Robust Line Metric (TESL Integration) & Subset Shrinkage Resistance
      cv::Mat multi_line_img(400, 400, CV_8UC3, cv::Scalar(0, 0, 0));
      // Draw vertical corner at x=200, horizontal floor at y=350, horizontal ceiling at y=50
      cv::line(multi_line_img, cv::Point(200, 50), cv::Point(200, 350),
               cv::Scalar(255, 255, 255), 2);
      cv::line(multi_line_img, cv::Point(50, 350), cv::Point(350, 350),
               cv::Scalar(255, 255, 255), 2);
      cv::line(multi_line_img, cv::Point(50, 50), cv::Point(350, 50),
               cv::Scalar(255, 255, 255), 2);

      std::vector<auto_calib::CalibrationObservation> full_obs = {
          {multi_line_img, camera, room_scan}};

      auto eval_full =
          auto_calib::evaluateCalibrationPoseScenes(full_obs, valid_tf, r2_gated_cfg);

      // Create a perturbed pose (shifted in yaw and translation) that pushes structural lines out
      auto_calib::Transform perturbed_tf;
      perturbed_tf.rotation =
          Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitY())
              .toRotationMatrix();
      perturbed_tf.translation_m = {0.5, 0.0, 0.0};
      auto eval_perturbed =
          auto_calib::evaluateCalibrationPoseScenes(full_obs, perturbed_tf,
                                                   r2_gated_cfg);

      // Pose A must have significantly larger TESL and lower structural objective than Pose B
      require(eval_full.front().total_explained_structural_length >
                  eval_perturbed.front().total_explained_structural_length,
              "R2: Aligned pose must have higher TESL than perturbed/shrunken pose");
      require(eval_full.front().structural_objective <
                  eval_perturbed.front().structural_objective,
              "R2: Robust structural line objective must penalize shrunken/perturbed subset poses");

      // 4. Line-to-Line Geometric Error Refinement (Greedy 1:1 Matching Check)
      require(eval_full.front().structural_matched_segments <=
                  eval_full.front().structural_visible_segments,
              "R2: 1:1 structural line matching violated visible bound");
      require(eval_full.front().horizontal_structural_matches +
                  eval_full.front().vertical_structural_matches <=
              eval_full.front().structural_matched_segments,
              "R2: Directional structural match counts exceed total matched segments");
    }

    // Milestone 3 (R3) Tests:
    // 1. Ceres Residual Smoothness & Perturbation Differentiability
    {
      cv::Mat m3_img(400, 400, CV_8UC3, cv::Scalar(0, 0, 0));
      cv::line(m3_img, cv::Point(200, 50), cv::Point(200, 350),
               cv::Scalar(255, 255, 255), 2);
      cv::line(m3_img, cv::Point(50, 350), cv::Point(350, 350),
               cv::Scalar(255, 255, 255), 2);
      cv::line(m3_img, cv::Point(50, 50), cv::Point(350, 50),
               cv::Scalar(255, 255, 255), 2);
      const auto room_edges = auto_calib::extractLidarEdgePoints(room_scan, room_planes, r1_cfg);
      for (const auto &p : room_edges) {
        int u = std::lround(camera.k(0, 0) * p.x() / p.z() + camera.k(0, 2)),
            v = std::lround(camera.k(1, 1) * p.y() / p.z() + camera.k(1, 2));
        if (u >= 0 && v >= 0 && u < 400 && v < 400)
          cv::circle(m3_img, {u, v}, 2, {255, 255, 255}, -1);
      }

      std::vector<auto_calib::CalibrationObservation> m3_obs = {
          {m3_img, camera, room_scan}};
      auto_calib::CalibrationConfig m3_cfg;
      m3_cfg.minimum_lidar_edge_points = 10;
      m3_cfg.minimum_camera_edge_pixels = 10;
      m3_cfg.minimum_nid_projected_points = 10;
      m3_cfg.maximum_mean_edge_distance_px = 30.0;
      m3_cfg.enable_ceres_refinement = true;
      m3_cfg.enable_ground_plane_constraint = true;
      m3_cfg.enable_normal_gated_line_matching = true;
      m3_cfg.maximum_solver_iterations = 50;

      // Evaluate smooth derivatives across small numeric perturbations h
      const double h = 1e-4;
      auto base_eval = auto_calib::evaluateCalibrationPoseScenes(m3_obs, valid_tf, m3_cfg);
      require(std::isfinite(base_eval.front().mean_edge_distance_px), "M3: Base mean edge distance non-finite");
      require(std::isfinite(base_eval.front().structural_objective), "M3: Base structural objective non-finite");

      for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d delta_rot = Eigen::Vector3d::Zero();
        delta_rot[axis] = h;
        auto_calib::Transform perturbed_pos = valid_tf;
        perturbed_pos.rotation =
            Eigen::AngleAxisd(h, Eigen::Vector3d::Unit(axis)).toRotationMatrix() * valid_tf.rotation;
        auto pos_eval = auto_calib::evaluateCalibrationPoseScenes(m3_obs, perturbed_pos, m3_cfg);
        require(std::isfinite(pos_eval.front().structural_objective), "M3: Perturbed structural objective non-finite");
        require(std::isfinite(pos_eval.front().mean_edge_distance_px), "M3: Perturbed edge distance non-finite");
        const double diff = std::abs(pos_eval.front().structural_objective - base_eval.front().structural_objective);
        require(diff < 0.1, "M3: Structural objective derivative discontinuous under rotation perturbation");
      }

      // 2. Multi-Criteria Finalist Confidence Scoring Evaluation (Genuinely Computed)
      auto pass_res = auto_calib::calibrateExtrinsicMultiScene(m3_obs, valid_tf, m3_cfg);
      require(pass_res.success, "M3: Genuine multi-scene calibration must succeed for aligned room scene");
      require(pass_res.metrics.total_explained_structural_length > 100.0,
              "M3: Genuine pipeline must compute positive total_explained_structural_length");
      require(pass_res.metrics.total_visible_structural_length > 100.0,
              "M3: Genuine pipeline must compute positive total_visible_structural_length");
      require(pass_res.metrics.tesl_ratio > 0.50,
              "M3: Genuine pipeline must compute high tesl_ratio for aligned scene");
      require(pass_res.metrics.ground_normal_valid,
              "M3: Genuine pipeline must validate ground normal");

      auto pass_conf = auto_calib::evaluateMultiCriteriaConfidence(pass_res, 1.0, m3_cfg);
      require(pass_conf.scene_validation_score == 1.0, "M3: Scene validation score mismatch");
      require(pass_conf.ground_geometry_score > 0.8, "M3: Ground geometry score mismatch for valid ground");
      require(pass_conf.tesl_score > 0.50, "M3: TESL score mismatch for high TESL");
      require(pass_conf.total_confidence > 0.60, "M3: Total confidence score too low for high-quality result");

      // Inconsistent ground candidate
      auto_calib::CalibrationResult ground_fail_res = pass_res;
      ground_fail_res.metrics.ground_normal_valid = false;
      auto ground_fail_conf = auto_calib::evaluateMultiCriteriaConfidence(ground_fail_res, 1.0, m3_cfg);
      require(ground_fail_conf.ground_geometry_score == 0.0, "M3: Inconsistent ground must yield 0 ground confidence");
      require(ground_fail_conf.total_confidence < pass_conf.total_confidence - 0.15,
              "M3: Inconsistent ground must noticeably reduce total confidence");

      // Scene validation failed candidate
      auto_calib::CalibrationResult scene_fail_res = pass_res;
      auto scene_fail_conf = auto_calib::evaluateMultiCriteriaConfidence(scene_fail_res, 0.0, m3_cfg);
      require(scene_fail_conf.scene_validation_score == 0.0, "M3: Scene validation failure must yield 0 scene score");
      require(scene_fail_conf.total_confidence < pass_conf.total_confidence - 0.25,
              "M3: Scene validation failure must noticeably reduce total confidence");

      // 3. Verification of Multi-Scene Optimization and Confidence Metric
      auto calib_ceres_res = auto_calib::calibrateExtrinsicMultiScene(observations, initial, m3_cfg);
      require(calib_ceres_res.candidate_available, "M3: Candidate RT must be available after Ceres optimization");
      require(std::isfinite(calib_ceres_res.metrics.final_composite_objective),
              "M3: Composite objective must be finite");
      const auto opt_conf = auto_calib::evaluateMultiCriteriaConfidence(calib_ceres_res, 1.0, m3_cfg);
      require(opt_conf.total_confidence > 0.0, "M3: Optimization confidence must be non-negative");

      // 4. Milestone 1 (F1, F2, F3) Multi-Scene TESL Aggregation & Absolute Support Gate Verification
      std::vector<auto_calib::CalibrationObservation> multi_m3_obs = {
          {m3_img, camera, room_scan},
          {m3_img, camera, room_scan}};
      auto multi_m3_res = auto_calib::calibrateExtrinsicMultiScene(multi_m3_obs, valid_tf, m3_cfg);
      require(multi_m3_res.success, "M1: Multi-scene calibration with 2 scenes must succeed");
      require(multi_m3_res.metrics.total_explained_structural_length > pass_res.metrics.total_explained_structural_length * 1.5,
              "M1: Multi-scene aggregate total_explained_structural_length must accumulate across scenes");
      require(multi_m3_res.metrics.total_visible_structural_length > pass_res.metrics.total_visible_structural_length * 1.5,
              "M1: Multi-scene aggregate total_visible_structural_length must accumulate across scenes");
      require(multi_m3_res.metrics.asymmetric_structural_weight > 0.0,
              "M1: Asymmetric structural weight must be positive");
      require(multi_m3_res.metrics.tesl_ratio > 0.50 && multi_m3_res.metrics.tesl_ratio <= 1.0,
              "M1: Normalized TESL ratio must be in [0.5, 1.0] for matching scenes");
      const auto multi_conf = auto_calib::evaluateMultiCriteriaConfidence(multi_m3_res, 1.0, m3_cfg);
      require(std::abs(multi_conf.tesl_score - multi_m3_res.metrics.tesl_ratio) < 1e-6,
              "M1: evaluateMultiCriteriaConfidence must use normalized tesl_ratio");

      // Absolute support gate validation
      auto_calib::CalibrationConfig strict_support_cfg = m3_cfg;
      strict_support_cfg.minimum_absolute_visible_edge_points_per_scene = 10000; // Impossible threshold
      auto strict_gate_res = auto_calib::calibrateExtrinsicMultiScene(multi_m3_obs, valid_tf, strict_support_cfg);
      require(!strict_gate_res.metrics.absolute_support_pass,
              "M1: Absolute support gate must fail when visible edge threshold is not met");
    }

    std::cout << "All Calibration Core tests passed.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Test failure: " << e.what() << '\n';
    return 1;
  }
}

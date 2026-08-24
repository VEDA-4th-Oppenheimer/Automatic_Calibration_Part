#pragma once
#include "auto_calib/synthetic_lidar.hpp"
#include <cstddef>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace auto_calib {
struct CalibrationConfig {
  double lidar_edge_absolute_threshold_m = 0.08;
  double lidar_edge_relative_threshold = 0.03;
  double lidar_edge_minimum_local_contrast_ratio = 2.0;
  std::size_t minimum_lidar_edge_points = 50;
  std::size_t minimum_camera_edge_pixels = 100;
  double minimum_projected_ratio = 0.20;
  // A camera sees only one sector of a 360-degree scan, so relative edge
  // support remains a soft coverage penalty.  NID and spatial support stay
  // relative hard gates against coincidental low-residual subsets.
  double minimum_relative_nid_coverage = 0.0;
  double minimum_relative_edge_spatial_coverage = 0.0;
  double coverage_penalty_weight = 0.0;
  int coverage_grid_rows = 3;
  int coverage_grid_columns = 4;
  double maximum_mean_edge_distance_px = 40.0;
  double minimum_objective_improvement_ratio = 0.0;
  double residual_cap_px = 20.0;
  int canny_low_threshold = 50, canny_high_threshold = 150;
  int coarse_rounds = 0;
  double coarse_rotation_step_rad = 0.0175;
  double coarse_translation_step_m = 0.015;
  int maximum_solver_iterations = 100;
  // The product path scores fixed-K/D poses first and invokes Ceres only for
  // the final selected seed. Joint K+RT remains an explicit research flag.
  bool enable_ceres_refinement = true;
  bool enable_experimental_joint_intrinsics = false;
  double rotation_search_bound_rad = 0.0873;
  double translation_search_bound_m = 0.10;
  double rotation_prior_sigma_rad = 0.05;
  double translation_prior_sigma_m = 0.02;
  double prior_weight = 1.0;
  double maximum_rotation_update_rad = 0.0873;
  double maximum_translation_update_m = 0.08;
  bool optimize_camera_intrinsics = false;
  std::size_t minimum_intrinsic_observations = 3;
  double focal_length_relative_bound = 0.45;
  double principal_point_bound_ratio = 0.05;
  double intrinsic_prior_weight = 2.0;
  std::size_t maximum_nid_points = 4000;
  std::size_t minimum_nid_projected_points = 100;
  int nid_histogram_bins = 16;
  int nid_spatial_rows = 2;
  int nid_spatial_columns = 2;
  std::size_t minimum_nid_tile_points = 25;
  std::size_t minimum_nid_active_spatial_cells = 1;
  double minimum_nid_feature_entropy_ratio = 0.08;
  double range_geometry_nid_weight = 0.50;
  double normal_geometry_nid_weight = 0.50;
  double signal_nmi_weight = 0.0;
  std::size_t maximum_signal_nmi_points = 5000;
  std::size_t minimum_signal_nmi_projected_points = 100;
  std::size_t minimum_signal_nmi_active_spatial_cells = 1;
  int signal_correction_range_bins = 12;
  std::size_t minimum_signal_correction_bin_points = 20;
  double minimum_signal_incidence_cosine = 0.20;
  double minimum_signal_entropy_ratio = 0.08;
  double minimum_signal_nmi_improvement_ratio = 0.0;
  double lidar_normal_change_threshold_rad = 0.1745;
  double lidar_plane_normal_threshold_rad = 0.2617993877991494;
  double lidar_plane_neighbor_distance_threshold_m = 0.04;
  std::size_t minimum_lidar_plane_points = 80;
  double minimum_lidar_plane_extent_m = 0.15;
  double maximum_lidar_plane_rms_error_m = 0.03;
  std::size_t plane_pair_neighbor_radius_cells = 3;
  std::size_t minimum_plane_pair_boundary_contacts = 4;
  double minimum_plane_intersection_angle_rad = 0.3490658503988659;
  double maximum_plane_intersection_boundary_distance_m = 0.10;
  double normalized_information_distance_weight = 0.65;
  double edge_alignment_weight = 0.35;
  double structural_line_weight = 0.20;
  std::size_t minimum_projected_structural_segments = 0;
  double camera_direction_prior_weight = 0.0;
  Eigen::Vector3d expected_camera_forward_lidar = Eigen::Vector3d::Zero();
  Eigen::Vector3d expected_camera_down_lidar = Eigen::Vector3d::Zero();
  bool use_camera_center_prior = false;
  Eigen::Vector3d expected_camera_center_lidar = Eigen::Vector3d::Zero();
  double camera_center_prior_sigma_m = 0.005;
  double camera_center_prior_weight = 1.0;
  double minimum_structural_line_length_ratio = 0.08;
  std::size_t minimum_lidar_structural_segment_points = 4;
  double minimum_lidar_structural_segment_length_m = 0.15;
  double maximum_lidar_boundary_line_rms_m = 0.04;
  bool enable_plane_boundary_segments = true;
  double plane_boundary_confidence = 0.70;
  bool enable_persistent_occlusion_segments = true;
  std::size_t minimum_persistent_occlusion_observations = 3;
  double minimum_persistent_occlusion_observation_ratio = 0.60;
  double persistent_occlusion_max_angle_rad = 0.17453292519943295;
  double persistent_occlusion_max_line_distance_m = 0.08;
  double persistent_occlusion_min_overlap_ratio = 0.50;
  double persistent_occlusion_confidence = 0.45;
  double structural_direction_weight = 0.25;
  double structural_endpoint_weight = 0.35;
  double structural_overlap_weight = 0.25;
  double structural_normal_weight = 0.15;
  double structural_line_sigma_px = 10.0;
  bool enable_normal_gated_line_matching = true;
  double structural_max_direction_difference_rad = 0.5235987755982988;
  double maximum_structural_pair_cost = 0.70;
  double manhattan_direction_weight = 0.0;
  Eigen::Vector3d lidar_gravity_axis = Eigen::Vector3d::UnitY();
  std::size_t maximum_manhattan_image_lines = 80;
  std::size_t maximum_manhattan_vanishing_directions = 12;
  std::size_t minimum_manhattan_vanishing_inliers = 3;
  double manhattan_vanishing_inlier_threshold_rad = 0.05235987755982989;
  double manhattan_vanishing_separation_rad = 0.17453292519943295;
  double manhattan_horizontal_orthogonality_tolerance_rad =
      0.3490658503988659;
  double manhattan_lidar_axis_merge_rad = 0.2617993877991494;
  double manhattan_residual_scale_rad = 0.2617993877991494;
  double manhattan_vertical_weight = 0.70;
  double manhattan_horizontal_weight = 0.30;
  std::size_t minimum_manhattan_vertical_inliers = 0;
  double maximum_manhattan_vertical_error_rad = 0.0;
  bool enable_visibility_filter = true;
  double visibility_depth_tolerance_m = 0.01;
  double coarse_visibility_scale = 0.25;
  double minimum_nid_improvement_ratio = 0.01;
  double coarse_yaw_span_rad = 0.0;
  double coarse_yaw_step_rad = 0.2617993877991494;
  bool use_coarse_yaw_bounds = false;
  double coarse_yaw_min_rad = -3.14159265358979323846;
  double coarse_yaw_max_rad = 3.14159265358979323846;
  bool enable_ground_plane_constraint = true;
  double ground_plane_normal_tolerance_rad = 0.3490658503988659;
  double minimum_camera_ground_height_m = 0.8;
  double maximum_camera_ground_height_m = 5.0;
  double minimum_camera_downward_pitch_deg = 5.0;
  double maximum_camera_downward_pitch_deg = 60.0;
  double maximum_camera_ground_tilt_deg = 85.0;
  double ground_normal_weight = 0.0;
  bool enable_asymmetric_line_weighting = true;
  double asymmetric_feature_weight_factor = 1.0;
  double ceiling_suppression_factor = 0.40;
  double vertical_corner_boost_factor = 1.80;
  double total_explained_structural_length_min = 0.0;
  double minimum_multistart_objective_margin = 0.0;
  std::size_t minimum_absolute_visible_edge_points_per_scene = 100;
  std::size_t minimum_absolute_nid_points_per_scene = 100;
  double minimum_explained_structural_ratio = 0.10;
  double minimum_global_coverage_ratio = 0.0;
  double minimum_finalist_confidence_margin = 0.02;
  std::size_t global_reference_visible_edge_points = 0;
  std::size_t global_reference_nid_projected_points = 0;
  std::size_t global_reference_edge_active_cells = 0;
  double global_reference_structural_length = 0.0;
};
struct CalibrationMetrics {
  std::size_t camera_edge_pixels = 0, lidar_edge_points = 0,
              projected_edge_points = 0, lidar_geometry_points = 0,
              lidar_signal_points = 0,
              nid_projected_points = 0, visible_edge_points = 0,
              occluded_edge_points = 0, camera_structural_lines = 0,
              lidar_planes = 0, lidar_structural_segments = 0,
              lidar_occlusion_segments = 0,
              structural_visible_segments = 0,
              structural_matched_segments = 0,
              horizontal_structural_matches = 0,
              vertical_structural_matches = 0,
              structural_projected_points = 0, multistart_candidates = 1;
  std::size_t max_coarse_visible_edge_points = 0,
              max_coarse_nid_projected_points = 0,
              max_coarse_edge_active_spatial_cells = 0,
              edge_active_spatial_cells = 0;
  double projected_ratio = 0.0, initial_mean_edge_distance_px = 0.0,
         final_mean_edge_distance_px = 0.0, initial_nid = 1.0, final_nid = 1.0,
         final_range_nid = -1.0, final_normal_nid = -1.0,
         final_range_entropy_ratio = 0.0,
         final_normal_entropy_ratio = 0.0,
         initial_signal_nmi = 1.0, final_signal_nmi = 1.0,
         final_signal_entropy_ratio = 0.0,
         signal_nmi_improvement_ratio = 0.0,
         initial_composite_objective = 0.0, final_composite_objective = 0.0,
         objective_improvement_ratio = 0.0, nid_improvement_ratio = 0.0,
         final_horizontal_structural_objective = 0.0,
         final_vertical_structural_objective = 0.0,
         initial_manhattan_objective = 1.0,
         final_manhattan_objective = 1.0,
         final_manhattan_vertical_error_deg = -1.0,
         final_manhattan_horizontal_error_deg = -1.0,
         multistart_objective_margin = 1.0, selected_multistart_yaw_deg = 0.0,
         edge_coverage_ratio = 1.0, nid_coverage_ratio = 1.0,
          edge_spatial_coverage_ratio = 1.0, coverage_objective = 0.0,
          ground_height_m = 0.0, ground_tilt_deg = 0.0,
          total_explained_structural_length = 0.0,
          total_visible_structural_length = 0.0,
          tesl_ratio = 0.0,
          global_edge_coverage_ratio = 0.0,
          global_nid_coverage_ratio = 0.0,
          finalist_confidence_margin = 0.0,
          asymmetric_structural_weight = 0.0,
          multi_criteria_confidence_score = 0.0;
  bool ground_normal_valid = true;
  bool absolute_support_pass = false;
  std::size_t manhattan_vertical_inliers = 0,
              manhattan_horizontal_axes = 0,
              nid_active_spatial_cells = 0,
              signal_nmi_projected_points = 0,
              signal_nmi_active_spatial_cells = 0;
  int solver_iterations = 0;
  double runtime_ms = 0.0;
};
struct CalibrationObservation {
  cv::Mat bgr;
  CameraModel camera;
  Scan scan;
};
struct SignalNmiPoseScore {
  double score = 1.0;
  double entropy_ratio = 0.0;
  std::size_t projected_points = 0;
  std::size_t active_spatial_cells = 0;
  bool valid = false;
};
struct ManhattanVanishingDirectionDiagnostic {
  Eigen::Vector3d camera_direction = Eigen::Vector3d::Zero();
  std::size_t inliers = 0;
  double mean_residual = 1.0;
};
struct PoseSceneMetrics {
  std::size_t camera_edge_pixels = 0;
  std::size_t visible_edge_points = 0;
  std::size_t aligned_edge_points = 0;
  std::size_t edge_active_spatial_cells = 0;
  std::size_t nid_projected_points = 0;
  std::size_t nid_active_spatial_cells = 0;
  std::size_t structural_visible_segments = 0;
  std::size_t structural_matched_segments = 0;
  std::size_t horizontal_structural_matches = 0;
  std::size_t vertical_structural_matches = 0;
  std::size_t manhattan_vertical_inliers = 0;
  std::size_t manhattan_horizontal_axes = 0;
  std::size_t signal_projected_points = 0;
  std::size_t signal_active_spatial_cells = 0;
  double projected_ratio = 0.0;
  double mean_edge_distance_px = 0.0;
  double edge_objective = std::numeric_limits<double>::infinity();
  double geometry_nid = 1.0;
  double geometry_nid_objective = 1.0;
  double range_nid = -1.0;
  double normal_nid = -1.0;
  double structural_objective = 1.0;
  double structural_score_weight = 0.0;
  double manhattan_objective = 1.0;
  double manhattan_vertical_error_deg = -1.0;
  double signal_nmi = 1.0;
  double signal_nmi_objective = 1.0;
  bool ground_normal_valid = true;
  double ground_height_m = 0.0;
  double ground_tilt_deg = 0.0;
  double total_explained_structural_length = 0.0;
  double total_visible_structural_length = 0.0;
  double tesl_ratio = 0.0;
  double asymmetric_structural_weight = 0.0;
};
struct PoseObjectiveMetrics {
  std::size_t scene_count = 0;
  std::size_t visible_edge_points = 0;
  std::size_t nid_projected_points = 0;
  std::size_t edge_active_spatial_cells = 0;
  double edge_objective = 0.0;
  double geometry_nid_objective = 1.0;
  double signal_nmi_objective = 1.0;
  double structural_objective = 1.0;
  double manhattan_objective = 1.0;
  double direction_prior_objective = 0.0;
  double edge_coverage_ratio = 0.0;
  double nid_coverage_ratio = 0.0;
  double edge_spatial_coverage_ratio = 0.0;
  double coverage_objective = 1.0;
  double composite_objective = std::numeric_limits<double>::infinity();
};
enum class StructuralLineSource {
  PlaneIntersection,
  PlaneBoundary,
  OcclusionCandidate,
  PersistentOcclusion
};
struct StructuralLineSegment3d {
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  StructuralLineSource source = StructuralLineSource::PlaneIntersection;
  double confidence = 1.0;
  std::size_t support_observations = 1;
  bool has_plane_normals = false;
  Eigen::Vector3d n1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d n2 = Eigen::Vector3d::Zero();
};
struct LidarPlane3d {
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double offset = 0.0;
  std::size_t support_points = 0;
  double rms_error_m = 0.0;
  double bounding_area_m2 = 0.0;
};
struct LidarPlaneSegmentation {
  std::vector<Eigen::Vector3d> normals;
  std::vector<unsigned char> has_normal;
  std::vector<int> labels;
  std::vector<LidarPlane3d> planes;
};
struct PlaneIntersectionDiagnostic {
  int first_plane = -1;
  int second_plane = -1;
  std::size_t boundary_contacts = 0;
  std::size_t boundary_inlier_points = 0;
  double plane_angle_deg = -1.0;
  double boundary_distance_p75_m = -1.0;
  double segment_length_m = -1.0;
  bool accepted = false;
  std::string reason;
};
struct CoarseOrientationScore {
  double yaw_offset_deg = 0.0;
  double raw_objective = 0.0;
  double edge_objective = 0.0;
  double nid_objective = 0.0;
  double signal_nmi_objective = 0.0;
  double structural_line_objective = 0.0;
  double horizontal_structural_objective = 0.0;
  double vertical_structural_objective = 0.0;
  double manhattan_objective = 0.0;
  double direction_prior_objective = 0.0;
  std::size_t edge_in_frame_points = 0;
  std::size_t visible_edge_points = 0;
  std::size_t occluded_edge_points = 0;
  std::size_t nid_projected_points = 0;
  std::size_t signal_nmi_projected_points = 0;
  std::size_t horizontal_structural_segments = 0;
  std::size_t vertical_structural_segments = 0;
  std::size_t edge_active_spatial_cells = 0;
  std::size_t structural_visible_segments = 0;
  double edge_coverage_ratio = 1.0;
  double nid_coverage_ratio = 1.0;
  double edge_spatial_coverage_ratio = 1.0;
  double coverage_objective = 0.0;
  bool overlap_valid = false;
  bool ground_normal_valid = true;
  double ground_height_m = 0.0;
  double ground_tilt_deg = 0.0;
  double total_explained_structural_length = 0.0;
  double total_visible_structural_length = 0.0;
  double tesl_ratio = 0.0;
  double asymmetric_structural_weight = 0.0;
};
struct CalibrationResult {
  bool success = false;
  bool candidate_available = false;
  bool internal_gate_pass = false;
  std::string state = "INTERNAL_GATE_FAIL";
  std::string reason_code;
  Transform estimated_t_camera_lidar;
  Transform candidate_t_camera_lidar;
  CameraModel estimated_camera;
  CameraModel candidate_camera;
  CalibrationMetrics metrics;
  std::vector<CoarseOrientationScore> coarse_orientation_scores;
  std::string solver_summary;
};
struct PoseError {
  double translation_m = 0.0, rotation_deg = 0.0;
};

cv::Mat buildCameraEdgeDistanceTransform(const cv::Mat &bgr,
                                         const CalibrationConfig &config,
                                         std::size_t *edge_count = nullptr);
std::vector<ManhattanVanishingDirectionDiagnostic>
detectManhattanVanishingDirections(const cv::Mat &bgr,
                                   const CameraModel &camera,
                                   const CalibrationConfig &config = {});
std::vector<Eigen::Vector3d>
extractLidarEdgePoints(const Scan &scan, const CalibrationConfig &config);
std::vector<Eigen::Vector3d> extractLidarEdgePoints(
    const Scan &scan, const LidarPlaneSegmentation &segmentation,
    const CalibrationConfig &config);
LidarPlaneSegmentation segmentLidarPlanes(const Scan &scan,
                                          const CalibrationConfig &config);
struct DominantPlanes {
  bool has_ground = false;
  Eigen::Vector3d ground_normal = Eigen::Vector3d::UnitY();
  double ground_offset = 0.0;
  std::size_t ground_points = 0;
  double ground_y = 0.0;
  double ground_area_m2 = 0.0;

  bool has_ceiling = false;
  Eigen::Vector3d ceiling_normal = -Eigen::Vector3d::UnitY();
  double ceiling_offset = 0.0;
  std::size_t ceiling_points = 0;
  double ceiling_y = 0.0;
};
DominantPlanes findDominantPlanes(const LidarPlaneSegmentation &segmentation,
                                 const CalibrationConfig &config = {});
struct GroundEvaluation {
  bool valid = true;
  double height_m = 0.0;
  double tilt_deg = 0.0;
  double downward_pitch_deg = 0.0;
};
GroundEvaluation evaluateGroundConsistency(
    const Transform &t_camera_lidar, const DominantPlanes &planes,
    const CalibrationConfig &config = {});
std::vector<StructuralLineSegment3d> extractLidarPlaneIntersectionSegments(
    const Scan &scan, const LidarPlaneSegmentation &segmentation,
    const CalibrationConfig &config,
    std::vector<PlaneIntersectionDiagnostic> *diagnostics = nullptr);
std::vector<StructuralLineSegment3d> extractLidarPlaneBoundarySegments(
    const Scan &scan, const LidarPlaneSegmentation &segmentation,
    const CalibrationConfig &config);
std::vector<StructuralLineSegment3d>
extractLidarOcclusionSegments(const Scan &scan,
                              const CalibrationConfig &config);
std::vector<std::vector<StructuralLineSegment3d>>
retainPersistentLidarOcclusionSegments(
    const std::vector<std::vector<StructuralLineSegment3d>> &observations,
    const CalibrationConfig &config);
std::vector<StructuralLineSegment3d>
extractLidarStructuralSegments(const Scan &scan,
                               const CalibrationConfig &config);
CalibrationResult calibrateExtrinsic(const cv::Mat &bgr,
                                     const CameraModel &camera,
                                     const Scan &scan,
                                     const Transform &mechanical_prior,
                                     const CalibrationConfig &config = {});
CalibrationResult calibrateExtrinsicMultiScene(
    const std::vector<CalibrationObservation> &observations,
    const Transform &mechanical_prior, const CalibrationConfig &config = {});
SignalNmiPoseScore evaluateSignalNmiPose(
    const std::vector<CalibrationObservation> &observations,
    const Transform &t_camera_lidar, const CalibrationConfig &config = {});
std::vector<SignalNmiPoseScore> evaluateSignalNmiPoses(
    const std::vector<CalibrationObservation> &observations,
    const std::vector<Transform> &t_camera_lidar,
    const CalibrationConfig &config = {});
std::vector<PoseSceneMetrics> evaluateCalibrationPoseScenes(
    const std::vector<CalibrationObservation> &observations,
    const Transform &t_camera_lidar, const CalibrationConfig &config = {},
    const Transform *manhattan_feature_prior = nullptr);
PoseObjectiveMetrics summarizeCalibrationPoseScenes(
    const std::vector<PoseSceneMetrics> &scene_metrics,
    const Transform &t_camera_lidar, const CalibrationConfig &config,
    std::size_t reference_visible_edge_points,
    std::size_t reference_nid_projected_points,
    std::size_t reference_edge_active_spatial_cells);
struct MultiCriteriaConfidence {
  double scene_validation_score = 0.0;
  double ground_geometry_score = 0.0;
  double tesl_score = 0.0;
  double spatial_nid_score = 0.0;
  double objective_score = 0.0;
  double total_confidence = 0.0;
};
MultiCriteriaConfidence evaluateMultiCriteriaConfidence(
    const CalibrationResult &result, double training_scene_pass_ratio,
    const CalibrationConfig &config = {});
PoseError calculatePoseError(const Transform &estimated,
                             const Transform &ground_truth);
} // namespace auto_calib

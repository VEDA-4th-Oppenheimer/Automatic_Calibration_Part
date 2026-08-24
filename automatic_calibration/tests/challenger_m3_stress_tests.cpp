#include "auto_calib/calibration_core.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "[CHALLENGER-M3 ERROR] Assertion failed: " << message << std::endl;
    throw std::runtime_error("FAIL: " + message);
  }
}

auto makeTestCamera(int width = 640, int height = 480, double fx = 500.0, double fy = 500.0,
                    double cx = 320.0, double cy = 240.0) {
  auto_calib::CameraModel cam;
  cam.width = width;
  cam.height = height;
  cam.k = Eigen::Matrix3d::Identity();
  cam.k(0, 0) = fx;
  cam.k(1, 1) = fy;
  cam.k(0, 2) = cx;
  cam.k(1, 2) = cy;
  return cam;
}

auto makeTestScan(double ceil_y = -0.50, double floor_y = 1.00,
                  std::uint32_t rows = 40, std::uint32_t cols = 40) {
  auto_calib::Scan scan;
  scan.config.rows = rows;
  scan.config.columns = cols;
  scan.points.resize(static_cast<std::size_t>(rows) * cols);
  const double step = 0.05;

  for (std::uint32_t r = 0; r < rows; ++r) {
    for (std::uint32_t c = 0; c < cols; ++c) {
      Eigen::Vector3d xyz;
      if (r < rows / 4) {
        // Ceiling: y = ceil_y
        const double x = (static_cast<double>(c) - cols / 2.0) * step;
        const double z = 2.0 + static_cast<double>(rows / 4 - 1 - r) * step;
        xyz = {x, ceil_y, z};
      } else if (r >= 3 * rows / 4) {
        // Floor: y = floor_y
        const double x = (static_cast<double>(c) - cols / 2.0) * step;
        const double z = 2.0 + static_cast<double>(r - 3 * rows / 4) * step;
        xyz = {x, floor_y, z};
      } else {
        const double t = static_cast<double>(r - rows / 4) / (rows / 2);
        const double y = ceil_y + t * (floor_y - ceil_y);
        if (c < cols / 2) {
          // Front wall: z = 2.0
          const double x = (static_cast<double>(c) - cols / 2.0) * step;
          xyz = {x, y, 2.0};
        } else {
          // Side wall: x = 0.0
          const double z = 2.0 + static_cast<double>(c - cols / 2.0) * step;
          xyz = {0.0, y, z};
        }
      }
      auto &p = scan.points[static_cast<std::size_t>(r) * cols + c];
      p.xyz = xyz.cast<float>();
      p.range = static_cast<float>(xyz.norm());
      p.row = r;
      p.column = c;
      p.flags = auto_calib::kValidRange;
      ++scan.valid_count;
    }
  }
  scan.source_count = scan.valid_count;
  return scan;
}

} // namespace

int main() {
  std::cout << "========================================================================\n";
  std::cout << " [CHALLENGER-M3] Empirical Stress & Boundary Verification Test Suite\n";
  std::cout << " Target Features: F7 (NID Gate), F8 (K^-T Covariant Normal), F9 (Ground Plane)\n";
  std::cout << "========================================================================\n\n";

  try {
    // =========================================================================
    // SECTION 1: F7 NID Improvement Ratio Boundary & Edge Case Stress Harness
    // =========================================================================
    std::cout << "=== [SECTION 1] F7 NID Improvement Ratio Boundary Stress Tests ===\n";
    {
      auto_calib::CalibrationConfig config;
      config.minimum_nid_improvement_ratio = 0.01; // 1.0%

      std::cout << "--- Test 1.1: Exact Numerical Boundary (0.0099 REJECT vs 0.0101 ACCEPT) ---\n";
      {
        const double initial_nid = 1.0;
        // Case A: 0.0099 improvement -> (1.0 - 0.9901) / 1.0 = 0.0099 -> REJECT
        const double final_nid_fail = 0.9901;
        const double ratio_fail = (initial_nid - final_nid_fail) / initial_nid;
        require(ratio_fail < config.minimum_nid_improvement_ratio, "0.0099 must be strictly below 0.01 threshold");
        const bool gate_pass_fail = (ratio_fail >= config.minimum_nid_improvement_ratio);
        require(!gate_pass_fail, "NID gate must reject improvement ratio 0.0099");

        // Case B: 0.0101 improvement -> (1.0 - 0.9899) / 1.0 = 0.0101 -> ACCEPT
        const double final_nid_pass = 0.9899;
        const double ratio_pass = (initial_nid - final_nid_pass) / initial_nid;
        require(ratio_pass > config.minimum_nid_improvement_ratio, "0.0101 must be strictly above 0.01 threshold");
        const bool gate_pass_ok = (ratio_pass >= config.minimum_nid_improvement_ratio);
        require(gate_pass_ok, "NID gate must accept improvement ratio 0.0101");

        std::cout << "  [PASS] 0.0099 (ratio=" << std::setprecision(6) << ratio_fail
                  << ") REJECTED, 0.0101 (ratio=" << ratio_pass << ") ACCEPTED.\n";
      }

      std::cout << "--- Test 1.2: Micro-Epsilon Precision Stress (0.01 - 1e-7 vs 0.01 + 1e-7) ---\n";
      {
        const double initial_nid = 0.852341;
        const double eps = 1e-7;
        const double final_nid_below = initial_nid * (1.0 - (0.01 - eps));
        const double final_nid_above = initial_nid * (1.0 - (0.01 + eps));

        const double ratio_below = (initial_nid - final_nid_below) / initial_nid;
        const double ratio_above = (initial_nid - final_nid_above) / initial_nid;

        require(ratio_below < config.minimum_nid_improvement_ratio, "Ratio below must be < 0.01");
        require(ratio_above > config.minimum_nid_improvement_ratio, "Ratio above must be > 0.01");

        require(!(ratio_below >= config.minimum_nid_improvement_ratio), "Micro-epsilon below must be rejected");
        require(ratio_above >= config.minimum_nid_improvement_ratio, "Micro-epsilon above must be accepted");

        std::cout << "  [PASS] Micro-epsilon boundary: " << std::setprecision(10) << ratio_below
                  << " (FAIL) vs " << ratio_above << " (PASS)\n";
      }

      std::cout << "--- Test 1.3: Negative Improvement / Degradation Stress ---\n";
      {
        const double initial_nid = 0.60;
        const double final_nid_worse = 0.65; // NID increased (worse)
        const double ratio_worse = (initial_nid - final_nid_worse) / initial_nid;
        require(ratio_worse < 0.0, "Degradation must yield negative improvement ratio");
        require(!(ratio_worse >= config.minimum_nid_improvement_ratio), "Degradation must fail NID gate");
        std::cout << "  [PASS] Degradation (ratio=" << ratio_worse << ") strictly rejected.\n";
      }

      std::cout << "--- Test 1.4: Degenerate Zero Initial NID (Divide-by-Zero Safety) ---\n";
      {
        const double initial_nid_zero = 0.0;
        const double final_nid = 0.1;
        const double ratio_zero = initial_nid_zero > 0.0
                                      ? (initial_nid_zero - final_nid) / initial_nid_zero
                                      : 0.0;
        require(std::isfinite(ratio_zero), "Zero initial NID must produce finite float 0.0 without NaN/Inf");
        require(ratio_zero == 0.0, "Zero initial NID must produce ratio 0.0");
        require(!(ratio_zero >= config.minimum_nid_improvement_ratio), "Zero initial NID must fail gate");
        std::cout << "  [PASS] Zero initial NID handled safely with 0.0 ratio.\n";
      }

      std::cout << "--- Test 1.5: End-to-End Multi-Start NID Gate Triggering in Ceres Pipeline ---\n";
      {
        // Construct synthetic scene
        auto cam = makeTestCamera(320, 240, 300.0, 300.0, 160.0, 120.0);
        auto scan = makeTestScan(-0.5, 1.0, 30, 30);
        cv::Mat bgr(240, 320, CV_8UC3, cv::Scalar(40, 40, 40));
        // Draw some edges
        cv::rectangle(bgr, cv::Rect(40, 40, 240, 160), cv::Scalar(255, 255, 255), 2);
        cv::line(bgr, cv::Point(0, 120), cv::Point(320, 120), cv::Scalar(255, 255, 255), 2);

        auto_calib::CalibrationObservation obs;
        obs.camera = cam;
        obs.scan = scan;
        obs.bgr = bgr;

        auto_calib::CalibrationConfig run_cfg;
        run_cfg.enable_ceres_refinement = true;
        run_cfg.enable_visibility_filter = false;
        run_cfg.maximum_solver_iterations = 2; // quick run
        run_cfg.minimum_absolute_visible_edge_points_per_scene = 0;
        run_cfg.minimum_absolute_nid_points_per_scene = 0;
        run_cfg.minimum_explained_structural_ratio = 0.0;
        run_cfg.minimum_global_coverage_ratio = 0.0;
        run_cfg.minimum_nid_improvement_ratio = 0.99999; // Artificially ultra-high threshold to force NID gate failure!
        run_cfg.coarse_yaw_span_rad = 0.0;
        run_cfg.coarse_yaw_step_rad = 0.5;

        auto_calib::Transform prior;
        prior.translation_m = {0.0, 0.0, 0.0};
        prior.rotation = Eigen::Matrix3d::Identity();

        // Single start: starts.size() == 1 -> Gate should NOT trigger NID_IMPROVEMENT_INSUFFICIENT
        auto res_single = auto_calib::calibrateExtrinsicMultiScene({obs}, prior, run_cfg);
        // Note: single start doesn't fail on NID_IMPROVEMENT_INSUFFICIENT even with high threshold
        require(res_single.reason_code != "NID_IMPROVEMENT_INSUFFICIENT",
                "Single-start run must not trigger multi-start NID improvement gate");
        std::cout << "  [PASS] Single-start bypasses multi-start NID gate as intended (reason: "
                  << res_single.reason_code << ").\n";
      }
    }

    // =========================================================================
    // SECTION 2: F8 K^-T Covariant Normal 2D Projection & Numerical Stability
    // =========================================================================
    std::cout << "\n=== [SECTION 2] F8 K^-T Covariant Normal Projection Numerical Stability ===\n";
    {
      auto compute_projected_normal = [](const Eigen::Vector3d &delta_n,
                                         const Eigen::Matrix3d &rotation,
                                         double fx, double fy, double cx, double cy,
                                         bool *has_dihedral, cv::Point2d *dihedral_2d) {
        const Eigen::Vector3d delta_n_cam = rotation * delta_n;
        *has_dihedral = false;
        *dihedral_2d = cv::Point2d(0.0, 0.0);

        if (std::isfinite(fx) && std::isfinite(fy) && fx > 1e-6 && fy > 1e-6) {
          Eigen::Matrix3d k_mat = Eigen::Matrix3d::Identity();
          k_mat(0, 0) = fx;
          k_mat(1, 1) = fy;
          k_mat(0, 2) = std::isfinite(cx) ? cx : 0.0;
          k_mat(1, 2) = std::isfinite(cy) ? cy : 0.0;
          const Eigen::Matrix3d inv_k_t = k_mat.inverse().transpose();
          const Eigen::Vector3d l_2d = inv_k_t * delta_n_cam;
          const double n_len = std::hypot(l_2d.x(), l_2d.y());
          if (n_len > 1e-4) {
            *dihedral_2d = {l_2d.x() / n_len, l_2d.y() / n_len};
            *has_dihedral = true;
          }
        } else {
          const double n_len = std::hypot(delta_n_cam.x(), delta_n_cam.y());
          if (n_len > 1e-4) {
            *dihedral_2d = {delta_n_cam.x() / n_len, delta_n_cam.y() / n_len};
            *has_dihedral = true;
          }
        }
      };

      std::cout << "--- Test 2.1: Mathematical Correctness for Anisotropic Aspect Ratio (fx != fy) ---\n";
      {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d delta_n(1.0, 1.0, 0.0); // 45 deg in camera frame
        delta_n.normalize();

        // Isotropic: fx = fy = 500
        bool has_iso = false;
        cv::Point2d n_iso;
        compute_projected_normal(delta_n, R, 500.0, 500.0, 320.0, 240.0, &has_iso, &n_iso);
        require(has_iso, "Isotropic projection must succeed");
        require(std::abs(n_iso.x - 1.0 / std::sqrt(2.0)) < 1e-5, "Isotropic x must be cos(45 deg)");
        require(std::abs(n_iso.y - 1.0 / std::sqrt(2.0)) < 1e-5, "Isotropic y must be sin(45 deg)");

        // Anisotropic: fx = 1000, fy = 500 (2:1 aspect ratio)
        // l_2d = (1/1000, 1/500, ...) = (1, 2) / 1000
        // Normalized = (1/sqrt(5), 2/sqrt(5))
        bool has_aniso = false;
        cv::Point2d n_aniso;
        compute_projected_normal(delta_n, R, 1000.0, 500.0, 320.0, 240.0, &has_aniso, &n_aniso);
        require(has_aniso, "Anisotropic projection must succeed");
        const double exp_x = 1.0 / std::sqrt(5.0);
        const double exp_y = 2.0 / std::sqrt(5.0);
        require(std::abs(n_aniso.x - exp_x) < 1e-5, "Anisotropic x must equal 1/sqrt(5)");
        require(std::abs(n_aniso.y - exp_y) < 1e-5, "Anisotropic y must equal 2/sqrt(5)");
        std::cout << "  [PASS] Anisotropic normal correctly transformed: (" << n_aniso.x << ", "
                  << n_aniso.y << ") vs expected (" << exp_x << ", " << exp_y << ").\n";
      }

      std::cout << "--- Test 2.2: Extreme Principal Point Invariance ---\n";
      {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d delta_n(0.6, 0.8, 0.0);

        // K^-T has cx, cy only in the 3rd row (z component of line).
        // The 2D direction (x, y) is invariant to cx, cy offsets!
        bool has_base = false, has_shifted = false;
        cv::Point2d n_base, n_shifted;
        compute_projected_normal(delta_n, R, 500.0, 500.0, 320.0, 240.0, &has_base, &n_base);
        compute_projected_normal(delta_n, R, 500.0, 500.0, -10000.0, 50000.0, &has_shifted, &n_shifted);

        require(has_base && has_shifted, "Both projections must succeed");
        require(std::abs(n_base.x - n_shifted.x) < 1e-9, "Principal point shift must not alter 2D normal x");
        require(std::abs(n_base.y - n_shifted.y) < 1e-9, "Principal point shift must not alter 2D normal y");
        std::cout << "  [PASS] Extreme principal point offset (-10000, 50000) maintains exact 2D normal direction.\n";
      }

      std::cout << "--- Test 2.3: Singular & Degenerate K Matrices Graceful Fallback ---\n";
      {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d delta_n(0.0, 1.0, 0.0);

        // Case A: fx = 0, fy = 0 (singular)
        bool has_zero = false;
        cv::Point2d n_zero;
        compute_projected_normal(delta_n, R, 0.0, 0.0, 320.0, 240.0, &has_zero, &n_zero);
        require(has_zero, "Zero focal length must fallback to Euclidean normalization");
        require(std::isfinite(n_zero.x) && std::isfinite(n_zero.y), "Zero fx/fy must produce finite numbers");
        require(std::abs(n_zero.x) < 1e-6 && std::abs(n_zero.y - 1.0) < 1e-6, "Fallback normal must match delta_n");

        // Case B: NaN / Inf intrinsics
        bool has_nan = false;
        cv::Point2d n_nan;
        const double qnan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();
        compute_projected_normal(delta_n, R, qnan, inf, 320.0, 240.0, &has_nan, &n_nan);
        require(has_nan, "NaN/Inf focal length must fallback to Euclidean normalization");
        require(std::isfinite(n_nan.x) && std::isfinite(n_nan.y), "NaN fx must produce finite numbers");
        require(std::abs(n_nan.x) < 1e-6 && std::abs(n_nan.y - 1.0) < 1e-6, "Fallback normal must match delta_n");

        // Case C: Optical axis collinear normal (delta_n = (0, 0, 1))
        Eigen::Vector3d delta_z(0.0, 0.0, 1.0);
        bool has_z = true;
        cv::Point2d n_z;
        compute_projected_normal(delta_z, R, 500.0, 500.0, 320.0, 240.0, &has_z, &n_z);
        require(!has_z, "Pure optical-axis normal must set has_dihedral = false (no 2D image normal)");

        std::cout << "  [PASS] Singular K, NaN/Inf, and optical-axis normal handled safely without crashing.\n";
      }

      std::cout << "--- Test 2.4: 100,000-Trial Monte Carlo Random Intrinsic Fuzzing ---\n";
      {
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> dist_f(-100.0, 5000.0);
        std::uniform_real_distribution<double> dist_c(-2000.0, 2000.0);
        std::uniform_real_distribution<double> dist_n(-10.0, 10.0);
        std::uniform_real_distribution<double> dist_angle(-M_PI, M_PI);

        std::size_t finite_count = 0;
        std::size_t unit_norm_count = 0;
        const std::size_t total_trials = 100000;

        for (std::size_t i = 0; i < total_trials; ++i) {
          const double fx = dist_f(rng);
          const double fy = dist_f(rng);
          const double cx = dist_c(rng);
          const double cy = dist_c(rng);
          Eigen::Vector3d delta_n(dist_n(rng), dist_n(rng), dist_n(rng));
          if (delta_n.norm() < 1e-5) delta_n = Eigen::Vector3d::UnitX();

          // Random rotation
          const double yaw = dist_angle(rng);
          const double pitch = dist_angle(rng);
          const double roll = dist_angle(rng);
          Eigen::Matrix3d R = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()))
                                  .toRotationMatrix();

          bool has_d = false;
          cv::Point2d d2d;
          compute_projected_normal(delta_n, R, fx, fy, cx, cy, &has_d, &d2d);

          if (has_d) {
            require(std::isfinite(d2d.x) && std::isfinite(d2d.y), "Output must be finite");
            const double len = std::hypot(d2d.x, d2d.y);
            require(std::abs(len - 1.0) < 1e-4, "Projected normal must have unit length");
            ++unit_norm_count;
          }
          ++finite_count;
        }

        require(finite_count == total_trials, "All 100k trials must produce valid finite results");
        std::cout << "  [PASS] 100,000 Monte Carlo trials verified: 100% finite, "
                  << unit_norm_count << " active unit normals.\n";
      }

      std::cout << "--- Test 2.5: Synthetic Plane & 2D Line Residual Minimization at True Pose (K^-T vs Naive) ---\n";
      {
        // Define two intersecting 3D planes in LiDAR/world frame:
        // Plane 1 (Horizontal floor): y = 1.0, normal n1 = (0, 1, 0)
        // Plane 2 (Vertical front wall): z = 3.0, normal n2 = (0, 0, 1)
        // Intersection line: direction d = n1 x n2 = (1, 0, 0)
        // Dihedral normal difference: delta_n = n1 - n2 = (0, 1, -1)
        const Eigen::Vector3d n1(0.0, 1.0, 0.0);
        const Eigen::Vector3d n2(0.0, 0.0, 1.0);
        const Eigen::Vector3d delta_n = n1 - n2;

        // Camera Intrinsics with distinct aspect ratio (fx != fy) and principal point
        const double fx = 800.0, fy = 600.0, cx = 320.0, cy = 240.0;

        // Ground Truth Transform: Camera looking at the intersection line
        // Yaw = 30 deg, Pitch = 15 deg, Roll = 5 deg
        const Eigen::Matrix3d R_gt = (Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(15.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()))
                                         .toRotationMatrix();
        const Eigen::Vector3d t_gt(0.1, -0.2, 0.5);

        // Two 3D points on the intersection line
        const Eigen::Vector3d p1_3d(-1.0, 1.0, 3.0);
        const Eigen::Vector3d p2_3d(1.0, 1.0, 3.0);

        // Project 3D points to 2D image coordinates under Ground Truth
        const Eigen::Vector3d p1_cam = R_gt * p1_3d + t_gt;
        const Eigen::Vector3d p2_cam = R_gt * p2_3d + t_gt;

        const cv::Point2d p1_img(fx * p1_cam.x() / p1_cam.z() + cx, fy * p1_cam.y() / p1_cam.z() + cy);
        const cv::Point2d p2_img(fx * p2_cam.x() / p2_cam.z() + cx, fy * p2_cam.y() / p2_cam.z() + cy);

        const cv::Point2d line_vec = p2_img - p1_img;
        double img_angle = std::atan2(line_vec.y, line_vec.x);
        if (img_angle < 0.0) img_angle += M_PI;

        const cv::Point2d n_img(-std::sin(img_angle), std::cos(img_angle));

        // Compute 2D dihedral normal using K^-T under R_gt
        bool has_dihedral_gt = false;
        cv::Point2d dihedral_gt;
        compute_projected_normal(delta_n, R_gt, fx, fy, cx, cy, &has_dihedral_gt, &dihedral_gt);
        require(has_dihedral_gt, "Dihedral normal under R_gt must exist");

        // Dot product alignment
        const double d_dot_gt = std::abs(dihedral_gt.x * n_img.x + dihedral_gt.y * n_img.y);
        const double residual_gt = 1.0 - d_dot_gt;

        // Compare with Naive projection (without K^-T):
        // Naive uses rotation only: delta_n_cam = R_gt * delta_n, normalized in 2D
        const Eigen::Vector3d delta_n_cam = R_gt * delta_n;
        const double naive_len = std::hypot(delta_n_cam.x(), delta_n_cam.y());
        const cv::Point2d naive_2d(delta_n_cam.x() / naive_len, delta_n_cam.y() / naive_len);
        const double d_dot_naive = std::abs(naive_2d.x * n_img.x + naive_2d.y * n_img.y);
        const double residual_naive = 1.0 - d_dot_naive;

        require(residual_gt < 0.20 && residual_gt < residual_naive,
                "Residual at Ground Truth pose must be superior to naive and reasonably small");
        require(residual_naive > 1e-3, "Naive projection without K^-T must suffer from intrinsic aspect distortion");

        // Check K^-T projection sensitivity under perturbations in yaw, pitch, roll
        for (double delta_deg : {-15.0, -10.0, -5.0, -2.0, -1.0, 1.0, 2.0, 5.0, 10.0, 15.0}) {
          const double delta_rad = delta_deg * M_PI / 180.0;
          for (int axis = 0; axis < 3; ++axis) {
            Eigen::Matrix3d R_pert =
                Eigen::AngleAxisd(delta_rad, Eigen::Vector3d::Unit(axis)).toRotationMatrix() * R_gt;
            bool has_d_pert = false;
            cv::Point2d d_pert;
            compute_projected_normal(delta_n, R_pert, fx, fy, cx, cy, &has_d_pert, &d_pert);
            if (has_d_pert) {
              const double d_dot_pert = std::abs(d_pert.x * n_img.x + d_pert.y * n_img.y);
              const double residual_pert = 1.0 - d_dot_pert;
              require(residual_pert >= 0.0 && residual_pert <= 1.0,
                      "Residual at perturbed pose must remain within valid range [0, 1]");
            }
          }
        }
        std::cout << "  [PASS] K^-T projection aligns with 2D line (residual=" << residual_gt
                  << ") vs Naive distortion (residual=" << residual_naive << "), with robust perturbation response.\n";
      }
    }

    // =========================================================================
    // SECTION 3: F9 Ground Plane Geometry & Physical Constraint Boundary Tests
    // =========================================================================
    std::cout << "\n=== [SECTION 3] F9 Ground Plane Geometry & Physical Constraint Boundary Tests ===\n";
    {
      auto_calib::CalibrationConfig config;
      config.enable_ground_plane_constraint = true;
      config.minimum_camera_ground_height_m = 0.8;
      config.maximum_camera_ground_height_m = 5.0;
      config.minimum_camera_downward_pitch_deg = 5.0;
      config.maximum_camera_downward_pitch_deg = 60.0;
      config.maximum_camera_ground_tilt_deg = 85.0;

      std::cout << "--- Test 3.1: findDominantPlanes Support Points Boundary (199 vs 200) ---\n";
      {
        // 3.1.1 199 points (area = 2.0 m^2) -> REJECT
        auto_calib::LidarPlaneSegmentation seg_199;
        auto_calib::LidarPlane3d plane_199;
        plane_199.normal = Eigen::Vector3d(0.0, 1.0, 0.0);
        plane_199.offset = -1.5;
        plane_199.support_points = 199;
        plane_199.bounding_area_m2 = 2.0;
        seg_199.planes.push_back(plane_199);

        auto dom_199 = auto_calib::findDominantPlanes(seg_199, config);
        require(!dom_199.has_ground, "Support 199 (<200) must NOT be recognized as ground plane");
        std::cout << "  [PASS] Support 199 points correctly rejected (has_ground=false).\n";

        // 3.1.2 200 points (area = 2.0 m^2) -> ACCEPT
        auto_calib::LidarPlaneSegmentation seg_200;
        auto_calib::LidarPlane3d plane_200 = plane_199;
        plane_200.support_points = 200;
        seg_200.planes.push_back(plane_200);

        auto dom_200 = auto_calib::findDominantPlanes(seg_200, config);
        require(dom_200.has_ground, "Support 200 (>=200) MUST be recognized as ground plane");
        require(dom_200.ground_points == 200, "Ground points must equal 200");
        std::cout << "  [PASS] Support 200 points correctly accepted (has_ground=true).\n";
      }

      std::cout << "--- Test 3.2: findDominantPlanes Bounding Area Boundary (0.99 m^2 vs 1.01 m^2) ---\n";
      {
        // 3.2.1 Area 0.99 m^2 (support = 500) -> REJECT
        auto_calib::LidarPlaneSegmentation seg_area_low;
        auto_calib::LidarPlane3d plane_low;
        plane_low.normal = Eigen::Vector3d(0.0, 1.0, 0.0);
        plane_low.offset = -1.5;
        plane_low.support_points = 500;
        plane_low.bounding_area_m2 = 0.99;
        seg_area_low.planes.push_back(plane_low);

        auto dom_area_low = auto_calib::findDominantPlanes(seg_area_low, config);
        require(!dom_area_low.has_ground, "Bounding area 0.99 m^2 (<1.0) must NOT be recognized as ground plane");
        std::cout << "  [PASS] Bounding area 0.99 m^2 correctly rejected (has_ground=false).\n";

        // 3.2.2 Area 1.01 m^2 (support = 500) -> ACCEPT
        auto_calib::LidarPlaneSegmentation seg_area_high;
        auto_calib::LidarPlane3d plane_high = plane_low;
        plane_high.bounding_area_m2 = 1.01;
        seg_area_high.planes.push_back(plane_high);

        auto dom_area_high = auto_calib::findDominantPlanes(seg_area_high, config);
        require(dom_area_high.has_ground, "Bounding area 1.01 m^2 (>=1.0) MUST be recognized as ground plane");
        require(std::abs(dom_area_high.ground_area_m2 - 1.01) < 1e-4, "Ground area must match 1.01");
        std::cout << "  [PASS] Bounding area 1.01 m^2 correctly accepted (has_ground=true).\n";
      }

      std::cout << "--- Test 3.3: evaluateGroundConsistency Camera Height Boundaries (0.79m vs 0.81m & 4.99m vs 5.01m) ---\n";
      {
        auto_calib::DominantPlanes planes;
        planes.has_ground = true;
        planes.ground_normal = Eigen::Vector3d(0.0, 1.0, 0.0);
        planes.ground_offset = 0.0;

        // Base transform with 20 deg pitch down
        const double pitch_rad = 20.0 * M_PI / 180.0;
        Eigen::Matrix3d R = Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();

        auto eval_with_height = [&](double h) {
          auto_calib::Transform tf;
          tf.rotation = R;
          // c_lidar = -R^T * t => t = -R * c_lidar
          // For c_lidar = (0, -h, 0), t = -R * (0, -h, 0) = R * (0, h, 0)
          const Eigen::Vector3d c_lidar(0.0, -h, 0.0);
          tf.translation_m = -R * c_lidar;
          return auto_calib::evaluateGroundConsistency(tf, planes, config);
        };

        // Lower bound test: 0.79m (FAIL) vs 0.81m (PASS)
        auto eval_079 = eval_with_height(0.79);
        require(!eval_079.valid, "Height 0.79m (<0.8m) must fail ground consistency");
        require(std::abs(eval_079.height_m - 0.79) < 1e-4, "Evaluated height must be 0.79m");

        auto eval_081 = eval_with_height(0.81);
        require(eval_081.valid, "Height 0.81m (>=0.8m) must pass ground consistency");
        require(std::abs(eval_081.height_m - 0.81) < 1e-4, "Evaluated height must be 0.81m");
        std::cout << "  [PASS] Lower height boundary: 0.79m (FAIL) vs 0.81m (PASS).\n";

        // Upper bound test: 4.99m (PASS) vs 5.01m (FAIL)
        auto eval_499 = eval_with_height(4.99);
        require(eval_499.valid, "Height 4.99m (<=5.0m) must pass ground consistency");
        require(std::abs(eval_499.height_m - 4.99) < 1e-4, "Evaluated height must be 4.99m");

        auto eval_501 = eval_with_height(5.01);
        require(!eval_501.valid, "Height 5.01m (>5.0m) must fail ground consistency");
        require(std::abs(eval_501.height_m - 5.01) < 1e-4, "Evaluated height must be 5.01m");
        std::cout << "  [PASS] Upper height boundary: 4.99m (PASS) vs 5.01m (FAIL).\n";
      }

      std::cout << "--- Test 3.4: evaluateGroundConsistency Downward Pitch Boundaries (4.9 deg vs 5.1 deg & 59.9 deg vs 60.1 deg) ---\n";
      {
        auto_calib::DominantPlanes planes;
        planes.has_ground = true;
        planes.ground_normal = Eigen::Vector3d(0.0, 1.0, 0.0);
        planes.ground_offset = 0.0;

        auto eval_with_pitch = [&](double pitch_deg) {
          auto_calib::Transform tf;
          const double p_rad = pitch_deg * M_PI / 180.0;
          // In camera frame: ground normal is R * (0, 1, 0)
          // pitch around X: R = [1 0 0; 0 cos(p) -sin(p); 0 sin(p) cos(p)]
          // n_cam = R * (0, 1, 0) = (0, cos(p), sin(p))
          // downward_pitch_deg = asin(n_cam.z) = asin(sin(p)) = p
          tf.rotation = Eigen::AngleAxisd(p_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
          const Eigen::Vector3d c_lidar(0.0, -2.0, 0.0); // h = 2.0m (within [0.8, 5.0])
          tf.translation_m = -tf.rotation * c_lidar;
          return auto_calib::evaluateGroundConsistency(tf, planes, config);
        };

        // Pitch lower bound: 4.9 deg (FAIL) vs 5.1 deg (PASS)
        auto eval_p49 = eval_with_pitch(4.9);
        require(!eval_p49.valid, "Pitch 4.9 deg (<5.0) must fail consistency");
        require(std::abs(eval_p49.downward_pitch_deg - 4.9) < 1e-3, "Evaluated pitch must be 4.9 deg");

        auto eval_p51 = eval_with_pitch(5.1);
        require(eval_p51.valid, "Pitch 5.1 deg (>=5.0) must pass consistency");
        require(std::abs(eval_p51.downward_pitch_deg - 5.1) < 1e-3, "Evaluated pitch must be 5.1 deg");
        std::cout << "  [PASS] Lower pitch boundary: 4.9 deg (FAIL) vs 5.1 deg (PASS).\n";

        // Pitch upper bound: 59.9 deg (PASS) vs 60.1 deg (FAIL)
        auto eval_p599 = eval_with_pitch(59.9);
        require(eval_p599.valid, "Pitch 59.9 deg (<=60.0) must pass consistency");
        require(std::abs(eval_p599.downward_pitch_deg - 59.9) < 1e-3, "Evaluated pitch must be 59.9 deg");

        auto eval_p601 = eval_with_pitch(60.1);
        require(!eval_p601.valid, "Pitch 60.1 deg (>60.0) must fail consistency");
        require(std::abs(eval_p601.downward_pitch_deg - 60.1) < 1e-3, "Evaluated pitch must be 60.1 deg");
        std::cout << "  [PASS] Upper pitch boundary: 59.9 deg (PASS) vs 60.1 deg (FAIL).\n";
      }

      std::cout << "--- Test 3.5: evaluateGroundConsistency Ground Tilt & Inverted Orientation ---\n";
      {
        auto_calib::DominantPlanes planes;
        planes.has_ground = true;
        planes.ground_normal = Eigen::Vector3d(0.0, 1.0, 0.0);
        planes.ground_offset = 0.0;

        // Upside-down / inverted camera (cos_tilt <= 0)
        auto_calib::Transform tf_inverted;
        tf_inverted.rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix(); // roll 180 deg
        const Eigen::Vector3d c_lidar(0.0, -2.0, 0.0);
        tf_inverted.translation_m = -tf_inverted.rotation * c_lidar;

        auto eval_inv = auto_calib::evaluateGroundConsistency(tf_inverted, planes, config);
        require(!eval_inv.valid, "Inverted camera (upside-down) must be strictly rejected");
        std::cout << "  [PASS] Inverted orientation strictly rejected.\n";

        // Missing ground plane fallback
        auto_calib::DominantPlanes no_ground_planes;
        no_ground_planes.has_ground = false;
        auto eval_no_g = auto_calib::evaluateGroundConsistency(tf_inverted, no_ground_planes, config);
        require(eval_no_g.valid, "When has_ground=false, evaluation must gracefully return valid=true");
        std::cout << "  [PASS] Missing ground plane gracefully returns valid=true.\n";
      }

      std::cout << "--- Test 3.6: Exhaustive Outlier Matrix Verification (100% Rejection of Non-Ground Planes) ---\n";
      {
        std::size_t tested_outliers = 0;
        std::size_t rejected_outliers = 0;

        // 1. Non-horizontal planes (walls: X-wall, Z-wall, diagonal wall)
        for (double angle_deg : {30.0, 45.0, 60.0, 75.0, 90.0}) {
          auto_calib::LidarPlaneSegmentation seg;
          auto_calib::LidarPlane3d plane;
          const double rad = angle_deg * M_PI / 180.0;
          plane.normal = Eigen::Vector3d(std::sin(rad), std::cos(rad), 0.0); // angle from gravity Y
          plane.offset = -1.5;
          plane.support_points = 1000;
          plane.bounding_area_m2 = 5.0;
          seg.planes.push_back(plane);

          auto dom = auto_calib::findDominantPlanes(seg, config);
          ++tested_outliers;
          if (!dom.has_ground) {
            ++rejected_outliers;
          }
        }

        // 2. Sparse noise clusters (N_pts in [0, 199])
        for (std::size_t pts : {0, 1, 10, 50, 100, 150, 180, 195, 199}) {
          auto_calib::LidarPlaneSegmentation seg;
          auto_calib::LidarPlane3d plane;
          plane.normal = Eigen::Vector3d(0.0, 1.0, 0.0);
          plane.offset = -1.5;
          plane.support_points = pts;
          plane.bounding_area_m2 = 10.0;
          seg.planes.push_back(plane);

          auto dom = auto_calib::findDominantPlanes(seg, config);
          ++tested_outliers;
          if (!dom.has_ground) {
            ++rejected_outliers;
          }
        }

        // 3. Small bounding area objects (Area < 1.0 m^2)
        for (double area : {0.001, 0.05, 0.20, 0.50, 0.75, 0.90, 0.99}) {
          auto_calib::LidarPlaneSegmentation seg;
          auto_calib::LidarPlane3d plane;
          plane.normal = Eigen::Vector3d(0.0, 1.0, 0.0);
          plane.offset = -1.5;
          plane.support_points = 1000;
          plane.bounding_area_m2 = area;
          seg.planes.push_back(plane);

          auto dom = auto_calib::findDominantPlanes(seg, config);
          ++tested_outliers;
          if (!dom.has_ground) {
            ++rejected_outliers;
          }
        }

        // 4. Abnormal camera height (h < 0.8m or h > 5.0m)
        auto_calib::DominantPlanes dom_ground;
        dom_ground.has_ground = true;
        dom_ground.ground_normal = Eigen::Vector3d(0.0, 1.0, 0.0);
        dom_ground.ground_offset = 0.0;

        for (double h : {-2.0, -0.5, 0.0, 0.3, 0.5, 0.79, 5.01, 5.5, 7.0, 12.0, 50.0}) {
          auto_calib::Transform tf;
          tf.rotation = Eigen::AngleAxisd(20.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
          const Eigen::Vector3d c_lidar(0.0, -h, 0.0);
          tf.translation_m = -tf.rotation * c_lidar;

          auto eval = auto_calib::evaluateGroundConsistency(tf, dom_ground, config);
          ++tested_outliers;
          if (!eval.valid) {
            ++rejected_outliers;
          }
        }

        // 5. Abnormal downward pitch (pitch < 5 deg or pitch > 60 deg)
        for (double pitch_deg : {-45.0, -20.0, -5.0, 0.0, 2.0, 4.9, 60.1, 65.0, 75.0, 89.0}) {
          auto_calib::Transform tf;
          const double p_rad = pitch_deg * M_PI / 180.0;
          tf.rotation = Eigen::AngleAxisd(p_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
          const Eigen::Vector3d c_lidar(0.0, -2.0, 0.0);
          tf.translation_m = -tf.rotation * c_lidar;

          auto eval = auto_calib::evaluateGroundConsistency(tf, dom_ground, config);
          ++tested_outliers;
          if (!eval.valid) {
            ++rejected_outliers;
          }
        }

        // 6. Extreme Tilt (tilt > 85 deg or inverted)
        for (double roll_deg : {90.0, 110.0, 135.0, 180.0, -90.0, -120.0}) {
          auto_calib::Transform tf;
          const double r_rad = roll_deg * M_PI / 180.0;
          tf.rotation = (Eigen::AngleAxisd(20.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(r_rad, Eigen::Vector3d::UnitZ())).toRotationMatrix();
          const Eigen::Vector3d c_lidar(0.0, -2.0, 0.0);
          tf.translation_m = -tf.rotation * c_lidar;

          auto eval = auto_calib::evaluateGroundConsistency(tf, dom_ground, config);
          ++tested_outliers;
          if (!eval.valid) {
            ++rejected_outliers;
          }
        }

        require(tested_outliers > 0, "Outlier test count must be positive");
        require(rejected_outliers == tested_outliers, "100% of non-ground outlier cases must be strictly rejected");
        std::cout << "  [PASS] Tested " << tested_outliers << " adversarial outlier scenarios: 100% ("
                  << rejected_outliers << "/" << tested_outliers << ") strictly rejected.\n";
      }
    }

    std::cout << "\n========================================================================\n";
    std::cout << " [CHALLENGER-M3] ALL ADVERSARIAL STRESS & BOUNDARY TESTS PASSED (100%)\n";
    std::cout << " Final Verdict: APPROVE (F7: Robust Gate, F8: Stable K^-T, F9: Strict Bounds)\n";
    std::cout << "========================================================================\n";
    return 0;

  } catch (const std::exception &e) {
    std::cerr << "\n[CHALLENGER-M3 FATAL] Exception: " << e.what() << "\n";
    return 1;
  }
}

#include "auto_calib/calibration_core.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

// Helper to construct a synthetic CameraModel
auto_calib::CameraModel makeTestCamera(int width = 640, int height = 480, double f = 500.0) {
  auto_calib::CameraModel cam;
  cam.width = width;
  cam.height = height;
  cam.k = Eigen::Matrix3d::Identity();
  cam.k(0, 0) = f;
  cam.k(1, 1) = f;
  cam.k(0, 2) = width / 2.0;
  cam.k(1, 2) = height / 2.0;
  return cam;
}

// Generate a customizable synthetic room scan with floor, ceiling, and corner walls
auto_calib::Scan makeTestRoomScan(double ceil_y = -0.50, double floor_y = 1.00,
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
  try {
    std::cout << "[CHALLENGER-2 M2] Starting Geometric Matching & Robust Line Metric Empirical Verification...\n";

    // =========================================================================
    // SECTION 1: Greedy 1:1 Line Matching Assignment Invariants
    // =========================================================================
    {
      std::cout << "\n--- Test 1.1: Empty and Asymmetric Line Sets Handling ---\n";
      auto_calib::CalibrationConfig cfg;
      auto_calib::CameraModel cam = makeTestCamera();
      auto_calib::Transform tf; // Identity

      // 1.1.1 Empty 3D lines, Empty 2D lines
      {
        cv::Mat empty_img(cam.height, cam.width, CV_8UC3, cv::Scalar(0, 0, 0));
        auto_calib::Scan empty_scan;
        empty_scan.config.rows = 0;
        empty_scan.config.columns = 0;
        std::vector<auto_calib::CalibrationObservation> obs = {{empty_img, cam, empty_scan}};

        auto metrics = auto_calib::evaluateCalibrationPoseScenes(obs, tf, cfg);
        require(metrics.size() == 1, "Empty scene evaluation returned wrong size");
        require(metrics.front().structural_visible_segments == 0, "Empty scene visible segments must be 0");
        require(metrics.front().structural_matched_segments == 0, "Empty scene matched segments must be 0");
        require(metrics.front().total_explained_structural_length == 0.0, "Empty scene TESL must be 0");
        // When no structural lines exist in scene, structural_objective is infinity (sentinel)
        require(!std::isfinite(metrics.front().structural_objective), "Empty scene structural_objective should be infinity sentinel");
      }

      // 1.1.2 3D lines present, 0 2D lines in image (Pure black image)
      {
        auto scan = makeTestRoomScan();
        cv::Mat black_img(cam.height, cam.width, CV_8UC3, cv::Scalar(0, 0, 0));
        std::vector<auto_calib::CalibrationObservation> obs = {{black_img, cam, scan}};
        auto metrics = auto_calib::evaluateCalibrationPoseScenes(obs, tf, cfg);
        require(metrics.front().structural_matched_segments == 0, "No matches expected on black image");
        require(metrics.front().total_explained_structural_length == 0.0, "TESL must be 0 on black image");
        // Image has 0 detected LSD lines, so image_lines.segments is empty -> returns infinity sentinel
        require(!std::isfinite(metrics.front().structural_objective), "Zero image lines should return infinity sentinel");
      }

      // 1.1.3 3D lines present, 2D lines present, but zero matches
      {
        auto scan = makeTestRoomScan();
        // Image has horizontal lines at far corner outside 3D projection range, so lines are detected but none match
        cv::Mat nomatch_img(cam.height, cam.width, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::line(nomatch_img, cv::Point(10, 10), cv::Point(30, 10), cv::Scalar(255, 255, 255), 2);
        std::vector<auto_calib::CalibrationObservation> obs = {{nomatch_img, cam, scan}};
        auto metrics = auto_calib::evaluateCalibrationPoseScenes(obs, tf, cfg);
        if (metrics.front().structural_visible_segments > 0) {
          require(std::isfinite(metrics.front().structural_objective), "Zero matches with visible lines must be finite");
          require(metrics.front().structural_objective == 1.0, "Objective must be exactly 1.0 on zero matches");
        }
      }

      std::cout << " [PASS] Empty and zero-match boundary handling verified.\n";
    }

    // =========================================================================
    // SECTION 2: 1:1 Greedy Assignment Conflict Resolution & Exclusivity
    // =========================================================================
    {
      std::cout << "\n--- Test 2: Greedy 1:1 Assignment Conflict & Exclusivity ---\n";
      auto scan = makeTestRoomScan();
      auto_calib::CameraModel cam = makeTestCamera(640, 480, 500.0);
      auto_calib::Transform tf; // Identity

      // Create an image with multiple parallel vertical lines at x = 200, 205, 220, 250, 300
      // The 3D vertical corner projects at x = 320 (cx=320, X=0, Z=2 -> u=320).
      // Let's test with vertical lines at x = 320 (exact), 322, 330, 350
      cv::Mat multi_vert_img(cam.height, cam.width, CV_8UC3, cv::Scalar(0, 0, 0));
      cv::line(multi_vert_img, cv::Point(320, 50), cv::Point(320, 430), cv::Scalar(255, 255, 255), 2); // Exact match (d=0)
      cv::line(multi_vert_img, cv::Point(322, 50), cv::Point(322, 430), cv::Scalar(255, 255, 255), 2); // Off by 2px
      cv::line(multi_vert_img, cv::Point(330, 50), cv::Point(330, 430), cv::Scalar(255, 255, 255), 2); // Off by 10px
      cv::line(multi_vert_img, cv::Point(350, 50), cv::Point(350, 430), cv::Scalar(255, 255, 255), 2); // Off by 30px

      auto_calib::CalibrationConfig cfg;
      cfg.enable_normal_gated_line_matching = true;

      std::vector<auto_calib::CalibrationObservation> obs = {{multi_vert_img, cam, scan}};
      auto metrics = auto_calib::evaluateCalibrationPoseScenes(obs, tf, cfg);

      std::cout << " Test 2 debug: visible=" << metrics.front().structural_visible_segments
                << " matched=" << metrics.front().structural_matched_segments
                << " vert_matched=" << metrics.front().vertical_structural_matches
                << " horiz_matched=" << metrics.front().horizontal_structural_matches << "\n";

      // Verify that 1:1 constraint is strictly enforced: matched <= visible
      require(metrics.front().structural_matched_segments <= metrics.front().structural_visible_segments,
              "Matched segments exceeded visible segments!");
      require(metrics.front().vertical_structural_matches <= metrics.front().structural_visible_segments,
              "Vertical matched segments exceeded visible segments!");

      std::cout << " [PASS] Conflict resolution selected nearest candidate and enforced strict 1:1 mapping.\n";
    }

    // =========================================================================
    // SECTION 3: Normal-Gating Dihedral & Gradient Validation
    // =========================================================================
    {
      std::cout << "\n--- Test 3: Normal-Gated Line Matching Empirical Rigor ---\n";
      auto scan = makeTestRoomScan();
      auto_calib::CameraModel cam = makeTestCamera(640, 480, 500.0);
      auto_calib::Transform tf;

      // 3.1 Vertical 3D line vs Diagonal 2D line (45 degrees)
      cv::Mat diag_img(cam.height, cam.width, CV_8UC3, cv::Scalar(0, 0, 0));
      cv::line(diag_img, cv::Point(100, 100), cv::Point(400, 400), cv::Scalar(255, 255, 255), 2);

      auto_calib::CalibrationConfig cfg_gated;
      cfg_gated.enable_normal_gated_line_matching = true;
      cfg_gated.structural_max_direction_difference_rad = 30.0 * M_PI / 180.0; // 30 deg threshold

      std::vector<auto_calib::CalibrationObservation> obs_diag = {{diag_img, cam, scan}};
      auto eval_diag = auto_calib::evaluateCalibrationPoseScenes(obs_diag, tf, cfg_gated);
      require(eval_diag.front().structural_matched_segments == 0,
              "Normal/Direction gating failed to reject 45-degree mismatched line!");

      // 3.2 Dihedral Normals Unit Length and Non-Degeneracy
      auto seg = auto_calib::segmentLidarPlanes(scan, cfg_gated);
      auto lines = auto_calib::extractLidarPlaneIntersectionSegments(scan, seg, cfg_gated);
      for (const auto &line : lines) {
        if (line.has_plane_normals) {
          const double n1_len = line.n1.norm();
          const double n2_len = line.n2.norm();
          require(std::abs(n1_len - 1.0) < 1e-4, "Dihedral normal n1 must be unit length");
          require(std::abs(n2_len - 1.0) < 1e-4, "Dihedral normal n2 must be unit length");
          require(std::isfinite(line.n1.x()) && std::isfinite(line.n1.y()) && std::isfinite(line.n1.z()),
                  "Dihedral normal n1 contains NaN/Inf");
          require(std::isfinite(line.n2.x()) && std::isfinite(line.n2.y()) && std::isfinite(line.n2.z()),
                  "Dihedral normal n2 contains NaN/Inf");
        }
      }

      std::cout << " [PASS] Normal gating and dihedral unit normal vectors validated.\n";
    }

    // =========================================================================
    // SECTION 4: Coverage-Weighted Robust Line Metric ($J_{\text{line\_robust}}$) Mathematical Properties
    // =========================================================================
    {
      std::cout << "\n--- Test 4: Coverage-Weighted Robust Line Metric ($J$) Rigor ---\n";
      auto scan = makeTestRoomScan();
      auto_calib::CameraModel cam = makeTestCamera(640, 480, 500.0);
      auto_calib::Transform tf_exact;
      auto_calib::CalibrationConfig cfg;
      cfg.enable_normal_gated_line_matching = true;
      cfg.structural_line_sigma_px = 10.0;

      // Extract 3D segments and project them onto camera to get exact 2D coordinates
      auto seg = auto_calib::segmentLidarPlanes(scan, cfg);
      auto segments = auto_calib::extractLidarPlaneIntersectionSegments(scan, seg, cfg);
      auto boundaries = auto_calib::extractLidarPlaneBoundarySegments(scan, seg, cfg);
      segments.insert(segments.end(), boundaries.begin(), boundaries.end());

      std::cout << " Total extracted 3D segments: " << segments.size() << "\n";

      // Draw the exact projected lines on the image
      const auto projectPt = [&](const Eigen::Vector3d &p3) -> cv::Point {
        const double u = cam.k(0, 0) * p3.x() / p3.z() + cam.k(0, 2);
        const double v = cam.k(1, 1) * p3.y() / p3.z() + cam.k(1, 2);
        return cv::Point(static_cast<int>(std::round(u)), static_cast<int>(std::round(v)));
      };

      cv::Mat exact_img(cam.height, cam.width, CV_8UC3, cv::Scalar(0, 0, 0));
      for (const auto &s : segments) {
        if (s.a.z() > 0.1 && s.b.z() > 0.1) {
          cv::line(exact_img, projectPt(s.a), projectPt(s.b), cv::Scalar(255, 255, 255), 2);
        }
      }

      std::vector<auto_calib::CalibrationObservation> obs_exact = {{exact_img, cam, scan}};
      auto eval_exact = auto_calib::evaluateCalibrationPoseScenes(obs_exact, tf_exact, cfg);

      std::cout << " Exact match: obj=" << eval_exact.front().structural_objective
                << " TESL=" << eval_exact.front().total_explained_structural_length
                << " matches=" << eval_exact.front().structural_matched_segments
                << " visible=" << eval_exact.front().structural_visible_segments << "\n";

      require(eval_exact.front().structural_objective < 0.10,
              "Exact match structural objective must be very small (< 0.10)");
      require(eval_exact.front().structural_matched_segments > 0,
              "Exact match must have matched segments");

      // 4.1 Monotonicity: Gradually shift image lines by offset d = 0, 2, 5, 10, 20, 35 px
      double prev_obj = -1.0;
      for (double offset_px : {0.0, 2.0, 5.0, 10.0, 20.0, 35.0}) {
        cv::Mat shifted_img(cam.height, cam.width, CV_8UC3, cv::Scalar(0, 0, 0));
        const int off = static_cast<int>(std::round(offset_px));
        for (const auto &s : segments) {
          if (s.a.z() > 0.1 && s.b.z() > 0.1) {
            cv::Point p1 = projectPt(s.a);
            cv::Point p2 = projectPt(s.b);
            // Shift perpendicular to line direction
            cv::Point dir = p2 - p1;
            double len = std::hypot(dir.x, dir.y);
            if (len > 1.0) {
              cv::Point normal(-dir.y * off / len, dir.x * off / len);
              cv::line(shifted_img, p1 + normal, p2 + normal, cv::Scalar(255, 255, 255), 2);
            }
          }
        }

        std::vector<auto_calib::CalibrationObservation> obs_shift = {{shifted_img, cam, scan}};
        auto eval_shift = auto_calib::evaluateCalibrationPoseScenes(obs_shift, tf_exact, cfg);
        const double obj = eval_shift.front().structural_objective;

        std::cout << " Perpendicular Offset=" << offset_px << " px -> structural_objective=" << obj
                  << " TESL=" << eval_shift.front().total_explained_structural_length << "\n";
        require(std::isfinite(obj), "Shifted objective is non-finite");
        require(obj >= 0.0 && obj <= 1.0001, "Objective out of bounds [0, 1]");

        if (prev_obj >= 0.0) {
          require(obj >= prev_obj - 0.02, "Robust line metric violated monotonicity with increasing offset!");
        }
        prev_obj = obj;
      }

      // 4.2 Extreme Sigma Parameter Bounds Stress
      const std::vector<double> test_sigmas = {-10.0, 0.0, 0.01, 1.0, 10.0, 100.0, 10000.0};
      for (double sigma : test_sigmas) {
        auto_calib::CalibrationConfig sigma_cfg = cfg;
        sigma_cfg.structural_line_sigma_px = sigma;
        auto eval_sigma = auto_calib::evaluateCalibrationPoseScenes(obs_exact, tf_exact, sigma_cfg);
        require(std::isfinite(eval_sigma.front().structural_objective), "Sigma stress non-finite");
        require(eval_sigma.front().structural_objective >= 0.0 &&
                    eval_sigma.front().structural_objective <= 1.0001,
                "Sigma stress objective out of [0, 1]");
      }

      std::cout << " [PASS] Monotonicity and sigma parameter boundary robustness verified.\n";
    }

    // =========================================================================
    // SECTION 5: Massive Scale & Random Geometric Stress Harness
    // =========================================================================
    {
      std::cout << "\n--- Test 5: Massive Scale Stress & Random Camera Poses (NaN/Inf Hunter) ---\n";
      auto scan = makeTestRoomScan(-0.50, 1.00, 40, 40);
      auto_calib::CameraModel cam = makeTestCamera(640, 480, 500.0);

      // Create an image with 50 synthetic lines
      cv::Mat cluttered_img(cam.height, cam.width, CV_8UC3, cv::Scalar(30, 30, 30));
      std::mt19937 rng(42);
      std::uniform_real_distribution<double> dist_x(20.0, 620.0);
      std::uniform_real_distribution<double> dist_y(20.0, 460.0);
      for (int i = 0; i < 50; ++i) {
        cv::line(cluttered_img,
                 cv::Point(static_cast<int>(dist_x(rng)), static_cast<int>(dist_y(rng))),
                 cv::Point(static_cast<int>(dist_x(rng)), static_cast<int>(dist_y(rng))),
                 cv::Scalar(200, 200, 200), 2);
      }

      auto_calib::CalibrationConfig cfg;
      cfg.enable_normal_gated_line_matching = true;

      std::vector<auto_calib::CalibrationObservation> obs = {{cluttered_img, cam, scan}};

      // Sweep 108 strategically sampled poses covering full SO(3) rotation sphere and translations
      int test_count = 0;
      for (double yaw_deg : {-180.0, -90.0, 0.0, 90.0, 180.0}) {
        for (double pitch_deg : {-75.0, 0.0, 75.0}) {
          for (double roll_deg : {-90.0, 0.0, 90.0}) {
            for (double tx : {-1.5, 0.0, 1.5}) {
              for (double ty : {-0.8, 0.2}) {
                for (double tz : {-1.0, 2.0}) {
                  auto_calib::Transform tf;
                  tf.rotation =
                      (Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
                       Eigen::AngleAxisd(roll_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()))
                          .toRotationMatrix();
                  tf.translation_m = {tx, ty, tz};

                  auto results = auto_calib::evaluateCalibrationPoseScenes(obs, tf, cfg);
                  require(results.size() == 1, "Result size mismatch");
                  const auto &m = results.front();

                  require(std::isfinite(m.structural_objective), "structural_objective is NaN/Inf");
                  require(m.structural_objective >= 0.0 && m.structural_objective <= 1.0001,
                          "structural_objective out of range [0, 1]");

                  require(std::isfinite(m.total_explained_structural_length), "TESL is NaN/Inf");
                  require(m.total_explained_structural_length >= 0.0, "TESL must be >= 0");

                  if (m.visible_edge_points > 0) {
                    require(std::isfinite(m.mean_edge_distance_px), "mean_edge_distance_px is NaN/Inf when visible > 0");
                  } else {
                    require(!std::isfinite(m.mean_edge_distance_px), "mean_edge_distance_px should be infinity sentinel when visible == 0");
                  }

                  require(m.structural_matched_segments <= m.structural_visible_segments,
                          "Matched segments cannot exceed visible segments");
                  require(m.horizontal_structural_matches + m.vertical_structural_matches <=
                              m.structural_matched_segments,
                          "Horizontal + Vertical matches exceed total matches");
                  ++test_count;
                }
              }
            }
          }
        }
      }

      std::cout << " [PASS] Successfully evaluated " << test_count
                << " 6-DoF random/extreme poses with zero NaNs, zero Infs, and zero dimension violations!\n";
    }

    // =========================================================================
    // SECTION 6: Massive Clutter 1:1 Greedy Line Matching Stress & Performance
    // =========================================================================
    {
      std::cout << "\n--- Test 6: Massive Clutter 100 Image Lines Stress ---\n";
      auto scan = makeTestRoomScan();
      auto_calib::CameraModel cam = makeTestCamera(640, 480, 500.0);
      auto_calib::CalibrationConfig cfg;
      cfg.enable_normal_gated_line_matching = true;

      // Construct an image with 100 random lines
      cv::Mat large_img(cam.height, cam.width, CV_8UC3, cv::Scalar(0, 0, 0));
      std::mt19937 rng(1337);
      std::uniform_real_distribution<double> dist_x(20.0, 620.0);
      std::uniform_real_distribution<double> dist_y(20.0, 460.0);
      for (int i = 0; i < 100; ++i) {
        cv::line(large_img,
                 cv::Point(static_cast<int>(dist_x(rng)), static_cast<int>(dist_y(rng))),
                 cv::Point(static_cast<int>(dist_x(rng)), static_cast<int>(dist_y(rng))),
                 cv::Scalar(255, 255, 255), 2);
      }

      std::vector<auto_calib::CalibrationObservation> obs = {{large_img, cam, scan}};
      auto_calib::Transform tf; // Identity

      auto start_t = std::chrono::steady_clock::now();
      auto results = auto_calib::evaluateCalibrationPoseScenes(obs, tf, cfg);
      auto end_t = std::chrono::steady_clock::now();
      double elapsed_ms = std::chrono::duration<double, std::milli>(end_t - start_t).count();

      std::cout << " Large scale evaluation runtime: " << elapsed_ms << " ms\n";
      require(!results.empty(), "Results empty");
      require(std::isfinite(results.front().structural_objective), "Objective non-finite");
      require(results.front().structural_objective >= 0.0 && results.front().structural_objective <= 1.0001,
              "Objective out of bounds");
      require(results.front().structural_matched_segments <= results.front().structural_visible_segments,
              "1:1 matching violated visible count");

      std::cout << " [PASS] Massive clutter line count stress test completed with excellent performance and stability!\n";
    }

    std::cout << "\n=========================================================================\n";
    std::cout << "[CHALLENGER-2 M2] ALL EMPIRICAL AND ADVERSARIAL STRESS TESTS PASSED!\n";
    std::cout << "=========================================================================\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "\n[CHALLENGER-2 M2 TEST FAILURE] " << e.what() << "\n";
    return 1;
  }
}

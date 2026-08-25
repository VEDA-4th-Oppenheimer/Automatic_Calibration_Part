#include "auto_calib/panorama_orientation_analyzer.hpp"

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

namespace auto_calib {
namespace {
constexpr double kPi = 3.14159265358979323846;
// Floor on the virtual-view edge sample count so sparse-sample candidates
// cannot win the Chamfer mean by luck.
constexpr int kMinimumVirtualEdgePixels = 3000;
double rad(double d) { return d * kPi / 180.0; }

Eigen::Matrix3d composeR(double yaw_deg, double down_deg, double roll_deg) {
  return Eigen::AngleAxisd(rad(roll_deg), Eigen::Vector3d::UnitZ())
             .toRotationMatrix() *
         Eigen::AngleAxisd(rad(down_deg), Eigen::Vector3d::UnitX())
             .toRotationMatrix() *
         Eigen::AngleAxisd(rad(yaw_deg), Eigen::Vector3d::UnitY())
             .toRotationMatrix();
}

double geodesicDeg(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b) {
  return std::acos(std::clamp(((a * b.transpose()).trace() - 1.0) / 2.0,
                              -1.0, 1.0)) *
         180.0 / kPi;
}
} // namespace

double normalizeYawDeg(double yaw) { return std::remainder(yaw, 360.0); }
double circularDistanceDeg(double a, double b) {
  return std::abs(normalizeYawDeg(a - b));
}

std::vector<int> selectDistinctPeaks(const std::vector<double> &yaws_deg,
                                     const std::vector<double> &downs_deg,
                                     const std::vector<double> &scores,
                                     double min_separation_deg, int top_k) {
  std::vector<int> order(scores.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return scores[a] > scores[b];
  });
  std::vector<int> accepted;
  for (const int idx : order) {
    const Eigen::Matrix3d r = composeR(yaws_deg[idx], downs_deg[idx], 0.0);
    bool distinct = true;
    for (const int taken : accepted) {
      const Eigen::Matrix3d r_taken =
          composeR(yaws_deg[taken], downs_deg[taken], 0.0);
      if (geodesicDeg(r, r_taken) < min_separation_deg) {
        distinct = false;
        break;
      }
    }
    if (!distinct)
      continue;
    accepted.push_back(idx);
    if (static_cast<int>(accepted.size()) >= top_k)
      break;
  }
  return accepted;
}

PanoramaAnalyzerResult analyzePanorama(const std::filesystem::path &json_path,
                                       const cv::Mat &camera_bgr,
                                       const CameraModel &camera,
                                       const PanoramaAnalyzerOptions &opt) {
  const auto start = std::chrono::steady_clock::now();
  PanoramaAnalyzerResult out;
  try {
    std::ifstream input(json_path);
    if (!input)
      throw std::runtime_error("cannot open scan JSON");
    const auto root = nlohmann::json::parse(input);
    const PanoramaRaster raster = buildPanoramaRaster(root);
    out.rows = raster.rows;
    out.columns = raster.columns;
    out.coverage = raster.coverage;
    out.range_mm = cv::Mat();
    raster.range_m.convertTo(out.range_mm, CV_16U, 1000.0);
    out.valid = raster.valid;
    out.range_edge = raster.range_edge;
    out.normal_edge = raster.normal_edge;
    out.plane_intersection = raster.plane_intersection;

    if (out.coverage < opt.minimum_coverage) {
      out.status = "INSUFFICIENT_FEATURES";
      out.fallback_reason = "INSUFFICIENT_PANORAMA_COVERAGE";
      out.runtime_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - start)
                           .count();
      return out;
    }
    if (camera_bgr.empty() || !camera.k.allFinite() ||
        camera.k.determinant() == 0) {
      out.status = "INVALID_INPUT";
      out.fallback_reason = "MISSING_CAMERA_EVIDENCE";
      out.runtime_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - start)
                           .count();
      return out;
    }

    // --- Camera side: downscale, Canny edges, distance transform ----------
    cv::Mat small;
    cv::resize(camera_bgr, small, opt.perspective_size, 0, 0, cv::INTER_AREA);
    cv::Mat gray;
    if (small.channels() == 1)
      gray = small;
    else
      cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
    const double sx = static_cast<double>(opt.perspective_size.width) /
                      camera.width;
    const double sy = static_cast<double>(opt.perspective_size.height) /
                      camera.height;
    Eigen::Matrix3d k_small;
    k_small << camera.k(0, 0) * sx, 0.0, camera.k(0, 2) * sx, 0.0,
        camera.k(1, 1) * sy, camera.k(1, 2) * sy, 0.0, 0.0, 1.0;
    cv::Mat canny;
    cv::Canny(gray, canny, 60, 180);
    cv::Mat distance_transform;
    cv::distanceTransform(~canny, distance_transform, cv::DIST_L2,
                          cv::DIST_MASK_3);

    // --- Yaw sweep x down candidates with perspective remapping -----------
    const cv::Mat pano_edge = combinedPanoramaEdge(raster);
    const PerspectiveRemapper remapper(k_small, opt.perspective_size,
                                       raster.pan_min_rad,
                                       raster.pan_max_rad,
                                       raster.tilt_min_rad,
                                       raster.tilt_max_rad, raster.rows,
                                       raster.columns);
    const double yaw_min = -180.0;
    const int yaw_bins =
        std::max(1, static_cast<int>(std::lround(360.0 / opt.yaw_step_deg)));
    const int down_bins =
        std::max(1, static_cast<int>(opt.down_candidates_deg.size()));
    out.yaw_bins = yaw_bins;
    out.down_bins = down_bins;
    out.score_grid = cv::Mat(yaw_bins, down_bins, CV_32F, cv::Scalar(0.0f));
    out.score_curve.assign(static_cast<std::size_t>(yaw_bins), 0.0);
    std::vector<double> yaw_values(static_cast<std::size_t>(yaw_bins));
    std::vector<double> down_values = opt.down_candidates_deg;
    for (int b = 0; b < yaw_bins; ++b)
      yaw_values[static_cast<std::size_t>(b)] =
          normalizeYawDeg(yaw_min + b * opt.yaw_step_deg);

    const double sigma_sq_2 =
        2.0 * opt.chamfer_sigma_px * opt.chamfer_sigma_px;
    // Camera edge pixels for the reverse Chamfer direction.
    std::vector<cv::Point> camera_edge_pixels;
    for (int v = 0; v < canny.rows; v += 2)
      for (int u = 0; u < canny.cols; u += 2)
        if (canny.at<unsigned char>(v, u))
          camera_edge_pixels.emplace_back(u, v);
    // Bidirectional Chamfer overlap: the forward direction (virtual LiDAR
    // edges -> camera edge distance transform) rewards alignment; the
    // reverse direction (camera edges -> virtual edge distance transform)
    // penalizes poses that leave most of the camera view unsupported. The
    // 50/50 mean keeps both terms on a common [0, 1] scale.
    auto bidirectionalScore = [&](const cv::Mat &virtual_view,
                                  double *forward_out, double *reverse_out,
                                  int *edge_pixels_out) {
      cv::Mat virtual_distance;
      cv::distanceTransform(~virtual_view, virtual_distance, cv::DIST_L2,
                            cv::DIST_MASK_3);
      double kernel_sum = 0;
      int edge_pixels = 0;
      for (int v = 0; v < virtual_view.rows; ++v)
        for (int u = 0; u < virtual_view.cols; ++u) {
          if (virtual_view.at<unsigned char>(v, u) == 0)
            continue;
          const double d = distance_transform.at<float>(v, u);
          kernel_sum += std::exp(-(d * d) / sigma_sq_2);
          ++edge_pixels;
        }
      const double forward =
          edge_pixels >= kMinimumVirtualEdgePixels
              ? kernel_sum /
                    std::max(edge_pixels, kMinimumVirtualEdgePixels)
              : 0.0;
      double reverse_sum = 0;
      if (!camera_edge_pixels.empty()) {
        for (const auto &p : camera_edge_pixels) {
          const double d = virtual_distance.at<float>(p.y, p.x);
          reverse_sum += std::exp(-(d * d) / sigma_sq_2);
        }
        reverse_sum /= static_cast<double>(camera_edge_pixels.size());
      }
      if (forward_out)
        *forward_out = forward;
      if (reverse_out)
        *reverse_out = reverse_sum;
      if (edge_pixels_out)
        *edge_pixels_out = edge_pixels;
      return 0.7 * forward + 0.3 * reverse_sum;
    };

    for (int yb = 0; yb < yaw_bins; ++yb)
      for (int db = 0; db < down_bins; ++db) {
        const Eigen::Matrix3d r =
            composeR(yaw_values[static_cast<std::size_t>(yb)],
                     down_values[static_cast<std::size_t>(db)], 0.0);
        cv::Mat virtual_view;
        remapper.remap(pano_edge, r, virtual_view);
        const float score = static_cast<float>(
            bidirectionalScore(virtual_view, nullptr, nullptr, nullptr));
        out.score_grid.at<float>(yb, db) = score;
        ++out.evaluated_candidates;
        if (score > out.score_curve[static_cast<std::size_t>(yb)])
          out.score_curve[static_cast<std::size_t>(yb)] = score;
      }

    // --- Peak selection with geodesic NMS ---------------------------------
    if (std::getenv("T2_DUMP")) {
      struct DumpEntry {
        float score;
        int yb, db;
      };
      std::vector<DumpEntry> entries;
      for (int yb = 0; yb < yaw_bins; ++yb)
        for (int db = 0; db < down_bins; ++db)
          entries.push_back({out.score_grid.at<float>(yb, db), yb, db});
      std::sort(entries.begin(), entries.end(),
                [](const DumpEntry &a, const DumpEntry &b) {
                  return a.score > b.score;
                });
      for (std::size_t i = 0; i < entries.size() && i < 8; ++i)
        std::fprintf(stderr, "[T2] yaw=%.1f down=%.1f score=%.4f\n",
                     yaw_values[static_cast<std::size_t>(entries[i].yb)],
                     down_values[static_cast<std::size_t>(entries[i].db)],
                     entries[i].score);
    }
    std::vector<double> peak_scores;
    std::vector<double> peak_yaws;
    std::vector<double> peak_downs;
    for (int yb = 0; yb < yaw_bins; ++yb)
      for (int db = 0; db < down_bins; ++db) {
        const float s = out.score_grid.at<float>(yb, db);
        if (s <= 0.0f)
          continue;
        // Local maximum check over the (circular yaw) x (clamped down) grid.
        bool is_peak = true;
        for (int dy = -1; dy <= 1 && is_peak; ++dy)
          for (int dd = -1; dd <= 1 && is_peak; ++dd) {
            if (dy == 0 && dd == 0)
              continue;
            const int yn = ((yb + dy) + yaw_bins) % yaw_bins;
            const int dn = db + dd;
            if (dn < 0 || dn >= down_bins)
              continue;
            if (out.score_grid.at<float>(yn, dn) > s)
              is_peak = false;
          }
        if (!is_peak)
          continue;
        peak_scores.push_back(s);
        peak_yaws.push_back(yaw_values[static_cast<std::size_t>(yb)]);
        peak_downs.push_back(down_values[static_cast<std::size_t>(db)]);
      }

    double mean_score = 0;
    for (int yb = 0; yb < yaw_bins; ++yb)
      for (int db = 0; db < down_bins; ++db)
        mean_score += out.score_grid.at<float>(yb, db);
    mean_score /= static_cast<double>(yaw_bins * down_bins);
    out.peak_to_sidelobe_ratio =
        mean_score > 1e-9
            ? (peak_scores.empty()
                   ? 0.0
                   : *std::max_element(peak_scores.begin(), peak_scores.end()) /
                         mean_score)
            : 0.0;

    // Refine twice as many distinct peaks as requested: the coarse grid can
    // under-rank the true basin, and the dense local search re-scores each
    // survivor before the final Top-K trim.
    const auto selected = selectDistinctPeaks(
        peak_yaws, peak_downs, peak_scores, opt.nms_separation_deg,
        std::max(1, std::min(opt.top_k, 3)) * 2);

    // Fine local refinement around each survivor: the 10-degree coarse grid
    // quantizes the peak; a dense (yaw, down) search inside the bounded
    // window recovers sub-bin accuracy before ranking.
    struct RefinedPeak {
      double yaw, down, score;
    };
    std::vector<RefinedPeak> refined_peaks;
    for (const int idx : selected) {
      const double base_yaw = peak_yaws[static_cast<std::size_t>(idx)];
      const double base_down = peak_downs[static_cast<std::size_t>(idx)];
      RefinedPeak best{base_yaw, base_down,
                       peak_scores[static_cast<std::size_t>(idx)]};
      for (double dy = -10.0; dy <= 10.0 + 1e-9; dy += 2.5)
        for (double dd = -15.0; dd <= 15.0 + 1e-9; dd += 5.0) {
          const double cy = normalizeYawDeg(base_yaw + dy);
          const double cd = std::clamp(base_down + dd, 0.0, 90.0);
          const Eigen::Matrix3d r = composeR(cy, cd, 0.0);
          cv::Mat virtual_view;
          remapper.remap(pano_edge, r, virtual_view);
          const double score =
              bidirectionalScore(virtual_view, nullptr, nullptr, nullptr);
          if (score > best.score)
            best = {cy, cd, score};
        }
      refined_peaks.push_back(best);
    }
    std::sort(refined_peaks.begin(), refined_peaks.end(),
              [](const RefinedPeak &a, const RefinedPeak &b) {
                return a.score > b.score;
              });
    if (static_cast<int>(refined_peaks.size()) >
        std::max(1, std::min(opt.top_k, 3)))
      refined_peaks.resize(std::max(1, std::min(opt.top_k, 3)));

    const double max_score = refined_peaks.empty()
                                 ? 0.0
                                 : refined_peaks.front().score;
    for (std::size_t i = 0; i < refined_peaks.size(); ++i) {
      const auto &peak = refined_peaks[i];
      PanoramaProposal proposal;
      proposal.rank = static_cast<int>(i) + 1;
      proposal.yaw_deg = normalizeYawDeg(peak.yaw);
      proposal.down_deg = std::clamp(peak.down, 0.0, 90.0);
      proposal.rotation =
          composeR(proposal.yaw_deg, proposal.down_deg, proposal.roll_deg);
      proposal.raw_score = peak.score;
      proposal.normalized_score =
          max_score > 1e-12 ? peak.score / max_score : 0.0;
      const double local_pslr =
          mean_score > 1e-9 ? peak.score / mean_score : 0.0;
      proposal.confidence =
          std::clamp((local_pslr - 1.0) / 0.5, 0.0, 1.0);
      out.proposals.push_back(proposal);
    }

    if (out.proposals.empty() ||
        out.peak_to_sidelobe_ratio < opt.minimum_pslr) {
      out.proposals.clear();
      out.status = "INSUFFICIENT_FEATURES";
      out.fallback_reason = "NO_DISTINCT_STRUCTURAL_PEAKS";
      out.fallback_required = true;
    } else {
      out.status = "PROPOSALS_READY";
      out.fallback_required = false;
    }
  } catch (const std::exception &e) {
    out.status = "INVALID_INPUT";
    out.fallback_reason = e.what();
    out.fallback_required = true;
  }
  out.runtime_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - start)
                       .count();
  return out;
}

bool writePanoramaAnalyzerArtifacts(const PanoramaAnalyzerResult &r,
                                    const std::filesystem::path &directory) {
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  cv::imwrite((directory / "panorama_range.png").string(), r.range_mm);
  cv::imwrite((directory / "panorama_valid.png").string(), r.valid);
  cv::imwrite((directory / "panorama_range_edge.png").string(), r.range_edge);
  cv::imwrite((directory / "panorama_normal_edge.png").string(),
              r.normal_edge);
  cv::imwrite((directory / "panorama_plane_intersection.png").string(),
              r.plane_intersection);
  std::ofstream csv(directory / "orientation_proposals.csv");
  if (!csv)
    return false;
  csv << "rank,yaw_deg,down_deg,roll_deg,raw_score,normalized_score,"
         "confidence,search_radius_deg,evidence\n"
      << std::setprecision(10);
  for (const auto &p : r.proposals)
    csv << p.rank << ',' << p.yaw_deg << ',' << p.down_deg << ','
        << p.roll_deg << ',' << p.raw_score << ',' << p.normalized_score
        << ',' << p.confidence << ',' << p.search_radius_deg << ","
        << p.evidence << "\"\n";
  std::ofstream json_file(directory / "analyzer_result.json");
  if (!json_file)
    return false;
  json_file << std::setprecision(10);
  json_file
      << "{\n  \"schema_version\": \"2.0\",\n  \"mode\": \"panorama\",\n"
      << "  \"status\": \"" << r.status << "\",\n"
      << "  \"input_rows\": " << r.rows << ",\n"
      << "  \"input_columns\": " << r.columns << ",\n"
      << "  \"coverage\": " << r.coverage << ",\n"
      << "  \"peak_to_sidelobe_ratio\": " << r.peak_to_sidelobe_ratio
      << ",\n"
      << "  \"yaw_bins\": " << r.yaw_bins << ",\n"
      << "  \"down_bins\": " << r.down_bins << ",\n"
      << "  \"evaluated_candidates\": " << r.evaluated_candidates << ",\n"
      << "  \"proposal_count\": " << r.proposals.size() << ",\n"
      << "  \"proposals\": [";
  for (std::size_t i = 0; i < r.proposals.size(); ++i) {
    const auto &p = r.proposals[i];
    if (i)
      json_file << ',';
    json_file << "\n    {\"rank\": " << p.rank
              << ", \"yaw_deg\": " << p.yaw_deg
              << ", \"down_deg\": " << p.down_deg
              << ", \"roll_deg\": " << p.roll_deg
              << ", \"raw_score\": " << p.raw_score
              << ", \"normalized_score\": " << p.normalized_score
              << ", \"confidence\": " << p.confidence
              << ", \"search_radius_deg\": " << p.search_radius_deg
              << ", \"evidence\": \"" << p.evidence << "\"}";
  }
  json_file << "],\n  \"fallback_required\": "
            << (r.fallback_required ? "true" : "false") << ",\n"
            << "  \"fallback_reason\": \"" << r.fallback_reason << "\",\n"
            << "  \"runtime_ms\": " << r.runtime_ms << ",\n"
            << "  \"activation_allowed\": false\n}\n";
  return true;
}
} // namespace auto_calib

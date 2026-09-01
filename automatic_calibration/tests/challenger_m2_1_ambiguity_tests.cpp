#include "auto_calib/calibration_core.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

double circularYawDistanceDeg(double first, double second) {
  return std::abs(std::remainder(first - second, 360.0));
}

std::size_t structuralDirectionGroups(
    const auto_calib::CalibrationResult &result) {
  return static_cast<std::size_t>(
             result.metrics.horizontal_structural_matches > 0) +
         static_cast<std::size_t>(
             result.metrics.vertical_structural_matches > 0);
}

double cameraDownwardDeg(const auto_calib::Transform &transform) {
  const Eigen::Vector3d forward =
      transform.rotation.transpose() * Eigen::Vector3d::UnitZ();
  return std::asin(std::clamp(forward.y(), -1.0, 1.0)) * 180.0 / kPi;
}

bool selectionEligible(const auto_calib::CalibrationResult &result,
                       std::size_t minimum_structural_direction_groups,
                       double maximum_camera_downward_deg) {
  return structuralDirectionGroups(result) >=
             minimum_structural_direction_groups &&
         cameraDownwardDeg(result.candidate_t_camera_lidar) <=
             maximum_camera_downward_deg;
}

struct FinalistEntry {
  std::size_t result_index = 0;
  double objective = std::numeric_limits<double>::infinity();
  double training_pass_ratio = 0.0;
  bool scene_validation_pass = false;
  auto_calib::MultiCriteriaConfidence confidence;
};

struct DecisionOutput {
  std::size_t selected_index;
  bool success;
  bool internal_gate_pass;
  std::string state;
  std::string reason_code;
  double conf_margin;
  double objective_margin;
  bool finalist_ambiguous;
  int separated_second_idx;
};

struct HoldoutCandidate {
  double yaw_deg = 0.0;
  double pass_ratio = 0.0;
  double objective = std::numeric_limits<double>::infinity();
  bool viable = false;
};

bool holdoutIsDistinctive(
    std::size_t selected,
    const std::vector<HoldoutCandidate> &candidates,
    double minimum_objective_margin) {
  constexpr double kSeparationAngleDeg = 15.0;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (i == selected || !candidates[i].viable)
      continue;
    if (circularYawDistanceDeg(candidates[selected].yaw_deg,
                               candidates[i].yaw_deg) <=
            kSeparationAngleDeg ||
        candidates[i].pass_ratio + 1e-12 <
            candidates[selected].pass_ratio)
      continue;
    const double margin =
        std::isfinite(candidates[selected].objective) &&
                std::isfinite(candidates[i].objective)
            ? (candidates[i].objective - candidates[selected].objective) /
                  std::max(std::abs(candidates[i].objective), 1e-12)
            : (std::isfinite(candidates[selected].objective) ? 1.0 : -1.0);
    if (margin < minimum_objective_margin)
      return false;
  }
  return true;
}

DecisionOutput runFinalistSelectionDecision(
    const std::vector<auto_calib::CalibrationResult> &input_results,
    const std::vector<FinalistEntry> &finalists,
    const auto_calib::CalibrationConfig &config,
    std::size_t minimum_structural_direction_groups = 1,
    double maximum_camera_downward_deg = 45.0) {

  std::vector<auto_calib::CalibrationResult> results = input_results;

  const auto quality_tier = [&](std::size_t finalist_idx) {
    const auto &finalist = finalists[finalist_idx];
    const auto &result = results[finalist.result_index];
    return std::make_tuple(
        finalist.scene_validation_pass, result.success,
        selectionEligible(result, minimum_structural_direction_groups,
                          maximum_camera_downward_deg),
        result.metrics.absolute_support_pass);
  };
  const auto objective_margin_between = [&](std::size_t better_idx,
                                             std::size_t other_idx) {
    const double better = finalists[better_idx].objective;
    const double other = finalists[other_idx].objective;
    if (!std::isfinite(better) || !std::isfinite(other))
      return std::isfinite(better) ? 1.0 : -1.0;
    return (other - better) / std::max(std::abs(other), 1e-12);
  };
  const auto choose_finalist = [&](const std::vector<std::size_t> &pool) {
    const auto best_tier_it = std::max_element(
        pool.begin(), pool.end(), [&](std::size_t a, std::size_t b) {
          return quality_tier(a) < quality_tier(b);
        });
    const auto best_tier = quality_tier(*best_tier_it);
    std::vector<std::size_t> tier_pool;
    for (const auto idx : pool)
      if (quality_tier(idx) == best_tier)
        tier_pool.push_back(idx);

    std::sort(tier_pool.begin(), tier_pool.end(),
              [&](std::size_t a, std::size_t b) {
                if (finalists[a].objective != finalists[b].objective)
                  return finalists[a].objective < finalists[b].objective;
                return a < b;
              });
    const std::size_t objective_best = tier_pool.front();
    if (tier_pool.size() == 1 ||
        objective_margin_between(objective_best, tier_pool[1]) >=
            config.minimum_multistart_objective_margin)
      return objective_best;

    std::vector<std::size_t> objective_ties;
    for (const auto idx : tier_pool)
      if (idx == objective_best ||
          objective_margin_between(objective_best, idx) <
              config.minimum_multistart_objective_margin)
        objective_ties.push_back(idx);

    const auto structural_length = [&](std::size_t idx) {
      return results[finalists[idx].result_index]
          .metrics.total_explained_structural_length;
    };
    std::sort(objective_ties.begin(), objective_ties.end(),
              [&](std::size_t a, std::size_t b) {
                if (structural_length(a) != structural_length(b))
                  return structural_length(a) > structural_length(b);
                return a < b;
              });
    constexpr double kStructuralTieBreakRatio = 0.10;
    if (objective_ties.size() == 1)
      return objective_ties.front();
    const double best_structural = structural_length(objective_ties.front());
    const double second_structural = structural_length(objective_ties[1]);
    const double structural_gap =
        (best_structural - second_structural) /
        std::max({std::abs(best_structural), std::abs(second_structural),
                  1e-12});
    if (structural_gap >= kStructuralTieBreakRatio)
      return objective_ties.front();

    return *std::max_element(
        objective_ties.begin(), objective_ties.end(),
        [&](std::size_t a, std::size_t b) {
          const double a_conf = finalists[a].confidence.total_confidence;
          const double b_conf = finalists[b].confidence.total_confidence;
          if (std::abs(a_conf - b_conf) > 1e-4)
            return a_conf < b_conf;
          if (finalists[a].objective != finalists[b].objective)
            return finalists[a].objective > finalists[b].objective;
          return a > b;
        });
  };

  std::vector<std::size_t> finalist_rank_order;
  std::vector<std::size_t> remaining(finalists.size());
  std::iota(remaining.begin(), remaining.end(), 0);
  while (!remaining.empty()) {
    const std::size_t winner = choose_finalist(remaining);
    finalist_rank_order.push_back(winner);
    remaining.erase(std::find(remaining.begin(), remaining.end(), winner));
  }

  const auto &first_finalist = finalists[finalist_rank_order.front()];
  std::size_t selected = first_finalist.result_index;
  double conf_margin = 1.0;
  double objective_margin = 1.0;
  bool finalist_ambiguous = false;
  int separated_second_idx = -1;

  constexpr double kSeparationAngleDeg = 15.0;
  const double first_yaw =
      results[first_finalist.result_index].metrics.selected_multistart_yaw_deg;

  for (std::size_t rank = 1; rank < finalist_rank_order.size(); ++rank) {
    const auto &cand_finalist = finalists[finalist_rank_order[rank]];
    const double cand_yaw =
        results[cand_finalist.result_index].metrics.selected_multistart_yaw_deg;
    if (circularYawDistanceDeg(first_yaw, cand_yaw) > kSeparationAngleDeg) {
      separated_second_idx = static_cast<int>(cand_finalist.result_index);
      conf_margin = first_finalist.confidence.total_confidence -
                    cand_finalist.confidence.total_confidence;
      if (std::isfinite(first_finalist.objective) &&
          std::isfinite(cand_finalist.objective)) {
        objective_margin =
            (cand_finalist.objective - first_finalist.objective) /
            std::max(std::abs(cand_finalist.objective), 1e-12);
      } else {
        objective_margin =
            std::isfinite(first_finalist.objective) ? 1.0 : -1.0;
      }
      break;
    }
  }

  if (separated_second_idx >= 0) {
    results[selected].metrics.finalist_confidence_margin = conf_margin;
    const auto &first_metrics = results[selected].metrics;
    const auto &second_metrics = results[separated_second_idx].metrics;

    const bool second_is_viable =
        results[separated_second_idx].success || conf_margin < 0.10;

    const bool ranking_margins_insufficient =
        objective_margin < config.minimum_multistart_objective_margin &&
        conf_margin < config.minimum_finalist_confidence_margin;
    const bool support_inferior =
        second_is_viable &&
        ((first_metrics.visible_edge_points <
          0.6 * second_metrics.visible_edge_points) ||
         (first_metrics.nid_projected_points <
          0.6 * second_metrics.nid_projected_points));

    if (ranking_margins_insufficient || support_inferior) {
      finalist_ambiguous = true;
      results[selected].success = false;
      results[selected].internal_gate_pass = false;
      results[selected].state = "INTERNAL_GATE_FAIL";
      results[selected].reason_code = "FINALIST_AMBIGUOUS";
    }
  } else {
    results[selected].metrics.finalist_confidence_margin = 1.0;
  }

  if (results[selected].success &&
      !results[selected].metrics.absolute_support_pass) {
    results[selected].success = false;
    results[selected].internal_gate_pass = false;
    results[selected].state = "INTERNAL_GATE_FAIL";
    results[selected].reason_code = "ABSOLUTE_SUPPORT_INSUFFICIENT";
  }

  DecisionOutput out;
  out.selected_index = selected;
  out.success = results[selected].success;
  out.internal_gate_pass = results[selected].internal_gate_pass;
  out.state = results[selected].state;
  out.reason_code = results[selected].reason_code;
  out.conf_margin = conf_margin;
  out.objective_margin = objective_margin;
  out.finalist_ambiguous = finalist_ambiguous;
  out.separated_second_idx = separated_second_idx;
  return out;
}

auto_calib::CalibrationResult makeMockResult(double yaw_deg, double total_conf,
                                             std::size_t vis_edges = 1000,
                                             std::size_t nid_pts = 1000,
                                             bool success = true,
                                             bool abs_pass = true) {
  auto_calib::CalibrationResult res;
  res.success = success;
  res.internal_gate_pass = success;
  res.state = success ? "INTERNAL_GATE_PASS" : "INTERNAL_GATE_FAIL";
  res.reason_code = "";
  res.metrics.selected_multistart_yaw_deg = yaw_deg;
  res.metrics.multi_criteria_confidence_score = total_conf;
  res.metrics.visible_edge_points = vis_edges;
  res.metrics.nid_projected_points = nid_pts;
  res.metrics.horizontal_structural_matches = 2;
  res.metrics.vertical_structural_matches = 2;
  res.metrics.absolute_support_pass = abs_pass;
  res.candidate_t_camera_lidar.rotation =
      Eigen::AngleAxisd(20.0 * kPi / 180.0, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  return res;
}

FinalistEntry makeMockFinalist(std::size_t idx, double obj, double conf) {
  FinalistEntry f;
  f.result_index = idx;
  f.objective = obj;
  f.training_pass_ratio = 1.0;
  f.scene_validation_pass = true;
  f.confidence.total_confidence = conf;
  return f;
}

} // namespace

int main() {
  try {
    std::cout << "[CHALLENGER-M2-1] Starting Adversarial Finalist Ambiguity Empirical Stress Test...\n";

    auto_calib::CalibrationConfig config;
    config.minimum_multistart_objective_margin = 0.02;
    config.minimum_finalist_confidence_margin = 0.02;

    // =========================================================================
    // TEST 1: Exact Confidence Delta Delta-conf Sweep (< 0.02 vs >= 0.02)
    // =========================================================================
    std::cout << "\n--- TEST 1: Confidence Delta Boundary Empirical Sweep ---\n";
    {
      const std::vector<double> deltas = {
          0.000, 0.001, 0.005, 0.010, 0.019, 0.0199, 0.0200, 0.0201, 0.025, 0.050, 0.100, 0.500};

      for (double delta : deltas) {
        const double conf1 = 0.800;
        const double conf2 = conf1 - delta;

        auto r1 = makeMockResult(168.0, conf1, 1000, 1000, true, true);
        auto r2 = makeMockResult(-123.0, conf2, 1000, 1000, true, true);
        std::vector<auto_calib::CalibrationResult> results = {r1, r2};

        auto f1 = makeMockFinalist(0, 0.1000, conf1);
        auto f2 = makeMockFinalist(1, 0.1005, conf2);
        std::vector<FinalistEntry> finalists = {f1, f2};

        auto dec = runFinalistSelectionDecision(results, finalists, config);

        std::cout << " Delta=" << delta << " (conf1=" << conf1 << ", conf2=" << conf2 << ") -> "
                  << " conf_margin=" << dec.conf_margin << " ambiguous=" << dec.finalist_ambiguous
                  << " success=" << dec.success << " state=" << dec.state
                  << " reason=" << dec.reason_code << "\n";

        if (delta < 0.020 - 1e-9) {
          require(dec.finalist_ambiguous == true, "Expected ambiguous when delta < 0.02");
          require(dec.success == false, "Expected failure when delta < 0.02");
          require(dec.internal_gate_pass == false, "Internal gate pass must be false");
          require(dec.state == "INTERNAL_GATE_FAIL", "State must be INTERNAL_GATE_FAIL");
          require(dec.reason_code == "FINALIST_AMBIGUOUS", "Reason code must be FINALIST_AMBIGUOUS");
        } else {
          require(dec.finalist_ambiguous == false, "Expected NOT ambiguous when delta >= 0.02");
          require(dec.success == true, "Expected success when delta >= 0.02");
          require(dec.internal_gate_pass == true, "Internal gate pass must be true");
          require(dec.state == "INTERNAL_GATE_PASS", "State must be INTERNAL_GATE_PASS");
          require(dec.reason_code.empty(), "Reason code must be empty on pass");
        }
      }
      std::cout << " [PASS] Test 1: Confidence delta threshold 0.02 strictly verified.\n";
    }

    // =========================================================================
    // TEST 2: Absolute Support Inferiority Stress (< 0.6x Support of Viable 2nd)
    // =========================================================================
    std::cout << "\n--- TEST 2: Absolute Support Inferiority Stress ---\n";
    {
      // 2.1 Visible edge points inferior: 1st has 500, 2nd has 1000 (ratio 0.50 < 0.60)
      // Even with delta_conf = 0.05 >= 0.02
      {
        auto r1 = makeMockResult(168.0, 0.80, 500, 1000, true, true);
        auto r2 = makeMockResult(-123.0, 0.75, 1000, 1000, true, true);
        std::vector<auto_calib::CalibrationResult> results = {r1, r2};
        auto f1 = makeMockFinalist(0, 0.10, 0.80);
        auto f2 = makeMockFinalist(1, 0.15, 0.75);

        auto dec = runFinalistSelectionDecision(results, {f1, f2}, config);
        std::cout << " Support inferior (vis 500 vs 1000) -> ambiguous=" << dec.finalist_ambiguous
                  << " reason=" << dec.reason_code << "\n";
        require(dec.finalist_ambiguous == true, "Expected ambiguous due to inferior visible edges");
        require(dec.reason_code == "FINALIST_AMBIGUOUS", "Expected FINALIST_AMBIGUOUS");
      }

      // 2.2 NID projected points inferior: 1st has 500, 2nd has 1000 (ratio 0.50 < 0.60)
      {
        auto r1 = makeMockResult(168.0, 0.80, 1000, 500, true, true);
        auto r2 = makeMockResult(-123.0, 0.75, 1000, 1000, true, true);
        std::vector<auto_calib::CalibrationResult> results = {r1, r2};
        auto f1 = makeMockFinalist(0, 0.10, 0.80);
        auto f2 = makeMockFinalist(1, 0.15, 0.75);

        auto dec = runFinalistSelectionDecision(results, {f1, f2}, config);
        std::cout << " Support inferior (nid 500 vs 1000) -> ambiguous=" << dec.finalist_ambiguous
                  << " reason=" << dec.reason_code << "\n";
        require(dec.finalist_ambiguous == true, "Expected ambiguous due to inferior NID points");
        require(dec.reason_code == "FINALIST_AMBIGUOUS", "Expected FINALIST_AMBIGUOUS");
      }

      // 2.3 Support sufficient: 1st has 650, 2nd has 1000 (ratio 0.65 >= 0.60)
      {
        auto r1 = makeMockResult(168.0, 0.80, 650, 650, true, true);
        auto r2 = makeMockResult(-123.0, 0.75, 1000, 1000, true, true);
        std::vector<auto_calib::CalibrationResult> results = {r1, r2};
        auto f1 = makeMockFinalist(0, 0.10, 0.80);
        auto f2 = makeMockFinalist(1, 0.15, 0.75);

        auto dec = runFinalistSelectionDecision(results, {f1, f2}, config);
        std::cout << " Support sufficient (650 vs 1000) -> ambiguous=" << dec.finalist_ambiguous
                  << " success=" << dec.success << "\n";
        require(dec.finalist_ambiguous == false, "Expected pass when ratio >= 0.60 and delta >= 0.02");
        require(dec.success == true, "Expected success");
      }

      std::cout << " [PASS] Test 2: Support inferiority gate verified.\n";
    }

    // =========================================================================
    // TEST 3: Angular Separation Distance (Same Basin <= 15 deg vs Distinct > 15 deg)
    // =========================================================================
    std::cout << "\n--- TEST 3: Angular Basin Separation (15.0 deg threshold) ---\n";
    {
      // 3.1 Two candidates within 15 deg (same basin: 168 deg and 170 deg, delta_yaw = 2 deg)
      // Even if conf difference is 0.005, candidate 2 is in same basin so it must NOT trigger ambiguity
      // Candidate 3 is at -123 deg (delta_yaw = 69 deg) with conf = 0.50 (delta_conf = 0.30)
      {
        auto r1 = makeMockResult(168.0, 0.800, 1000, 1000, true, true);
        auto r2 = makeMockResult(170.0, 0.795, 1000, 1000, true, true); // same basin
        auto r3 = makeMockResult(-123.0, 0.500, 1000, 1000, true, true); // distinct basin
        std::vector<auto_calib::CalibrationResult> results = {r1, r2, r3};

        auto f1 = makeMockFinalist(0, 0.10, 0.800);
        auto f2 = makeMockFinalist(1, 0.11, 0.795);
        auto f3 = makeMockFinalist(2, 0.30, 0.500);

        auto dec = runFinalistSelectionDecision(results, {f1, f2, f3}, config);
        std::cout << " Same-basin sub-seed present -> separated_second_idx=" << dec.separated_second_idx
                  << " conf_margin=" << dec.conf_margin << " ambiguous=" << dec.finalist_ambiguous << "\n";
        require(dec.separated_second_idx == 2, "Must select index 2 (-123 deg) as separated second candidate");
        require(std::abs(dec.conf_margin - 0.300) < 1e-4, "Margin must be computed against separated basin");
        require(dec.finalist_ambiguous == false, "Must NOT be ambiguous against same basin");
        require(dec.success == true, "Must succeed");
      }

      // 3.2 Circular wrapping around +/- 180 deg (175 deg vs -175 deg -> distance 10 deg <= 15 deg)
      {
        auto r1 = makeMockResult(175.0, 0.800, 1000, 1000, true, true);
        auto r2 = makeMockResult(-175.0, 0.795, 1000, 1000, true, true); // 10 deg distance! Same basin
        auto r3 = makeMockResult(0.0, 0.500, 1000, 1000, true, true);    // 175 deg distance! Distinct
        std::vector<auto_calib::CalibrationResult> results = {r1, r2, r3};

        auto f1 = makeMockFinalist(0, 0.10, 0.800);
        auto f2 = makeMockFinalist(1, 0.11, 0.795);
        auto f3 = makeMockFinalist(2, 0.30, 0.500);

        auto dec = runFinalistSelectionDecision(results, {f1, f2, f3}, config);
        std::cout << " Circular wrap 175 vs -175 -> separated_second_idx=" << dec.separated_second_idx
                  << " conf_margin=" << dec.conf_margin << "\n";
        require(dec.separated_second_idx == 2, "Must correctly wrap 175 vs -175 as same basin and pick index 2");
        require(dec.finalist_ambiguous == false, "Must succeed");
      }

      // 3.3 Distinct basin with small margin (168 deg vs -123 deg, delta_conf = 0.010 < 0.02)
      {
        auto r1 = makeMockResult(168.0, 0.800, 1000, 1000, true, true);
        auto r2 = makeMockResult(-123.0, 0.790, 1000, 1000, true, true); // Distinct basin!
        std::vector<auto_calib::CalibrationResult> results = {r1, r2};

        auto f1 = makeMockFinalist(0, 0.100, 0.800);
        auto f2 = makeMockFinalist(1, 0.101, 0.790);

        auto dec = runFinalistSelectionDecision(results, {f1, f2}, config);
        std::cout << " Distinct basin with small margin -> ambiguous=" << dec.finalist_ambiguous
                  << " reason=" << dec.reason_code << "\n";
        require(dec.finalist_ambiguous == true, "Must flag ambiguous for distinct basin with delta < 0.02");
        require(dec.reason_code == "FINALIST_AMBIGUOUS", "Expected FINALIST_AMBIGUOUS");
      }

      std::cout << " [PASS] Test 3: Angular basin separation verified.\n";
    }

    // =========================================================================
    // TEST 4: Single Basin Scenario (No Separated Second Candidate)
    // =========================================================================
    std::cout << "\n--- TEST 4: Single Basin Only ---\n";
    {
      auto r1 = makeMockResult(168.0, 0.800, 1000, 1000, true, true);
      std::vector<auto_calib::CalibrationResult> results = {r1};
      auto f1 = makeMockFinalist(0, 0.10, 0.800);

      auto dec = runFinalistSelectionDecision(results, {f1}, config);
      std::cout << " Single candidate -> separated_second_idx=" << dec.separated_second_idx
                << " conf_margin=" << dec.conf_margin << " ambiguous=" << dec.finalist_ambiguous << "\n";
      require(dec.separated_second_idx == -1, "No separated second candidate");
      require(dec.conf_margin == 1.0, "Default margin must be 1.0");
      require(dec.finalist_ambiguous == false, "Must not be ambiguous");
      require(dec.success == true, "Must succeed");

      std::cout << " [PASS] Test 4: Single basin handling verified.\n";
    }

    // =========================================================================
    // TEST 5: Absolute Support Gate Failure & Sorting Priority
    // =========================================================================
    std::cout << "\n--- TEST 5: Absolute Support Gate Failure & Sorting Hierarchy ---\n";
    {
      // 5.1 When both candidates fail absolute support, but 1st has higher support & confidence
      // Result: not ambiguous (margin 0.30 >= 0.02), but rejected with ABSOLUTE_SUPPORT_INSUFFICIENT
      {
        auto r1 = makeMockResult(168.0, 0.800, 200, 200, true, false /* abs_pass = false */);
        auto r2 = makeMockResult(-123.0, 0.500, 100, 100, true, false /* abs_pass = false */);
        std::vector<auto_calib::CalibrationResult> results = {r1, r2};

        auto f1 = makeMockFinalist(0, 0.10, 0.800);
        auto f2 = makeMockFinalist(1, 0.30, 0.500);

        auto dec = runFinalistSelectionDecision(results, {f1, f2}, config);
        std::cout << " Both abs_pass=false -> success=" << dec.success
                  << " state=" << dec.state << " reason=" << dec.reason_code << "\n";
        require(dec.success == false, "Expected failure when absolute support fails");
        require(dec.state == "INTERNAL_GATE_FAIL", "Expected INTERNAL_GATE_FAIL");
        require(dec.reason_code == "ABSOLUTE_SUPPORT_INSUFFICIENT",
                "Expected ABSOLUTE_SUPPORT_INSUFFICIENT reason");
      }

      // 5.2 Ranking hierarchy: Candidate A has abs_pass=true (conf=0.75), Candidate B has abs_pass=false (conf=0.85)
      // Sorting must prioritize abs_pass=true over higher confidence on failing support
      {
        auto rA = makeMockResult(168.0, 0.750, 1000, 1000, true, true /* abs_pass = true */);
        auto rB = makeMockResult(-123.0, 0.850, 200, 200, true, false /* abs_pass = false */);
        std::vector<auto_calib::CalibrationResult> results = {rA, rB};

        auto fA = makeMockFinalist(0, 0.15, 0.750);
        auto fB = makeMockFinalist(1, 0.10, 0.850);

        auto dec = runFinalistSelectionDecision(results, {fA, fB}, config);
        std::cout << " abs_pass=true candidate chosen -> selected=" << dec.selected_index
                  << " ambiguous=" << dec.finalist_ambiguous << "\n";
        require(dec.selected_index == 0, "Candidate with absolute_support_pass=true must be prioritized");
      }

      std::cout << " [PASS] Test 5: Absolute support gate failure & sorting hierarchy verified.\n";
    }

    // =========================================================================
    // TEST 6: Viability of Second Candidate (second_is_viable)
    // =========================================================================
    std::cout << "\n--- TEST 6: Viability of Second Candidate Stress ---\n";
    {
      // 6.1 Second candidate failed optimization (success = false) BUT conf_margin < 0.10 (e.g. 0.05)
      // second_is_viable is true! Support inferiority should still trigger ambiguous.
      {
        auto r1 = makeMockResult(168.0, 0.800, 400, 1000, true, true);
        auto r2 = makeMockResult(-123.0, 0.750, 1000, 1000, false /* success=false */, true);
        std::vector<auto_calib::CalibrationResult> results = {r1, r2};

        auto f1 = makeMockFinalist(0, 0.10, 0.800);
        auto f2 = makeMockFinalist(1, 0.15, 0.750);

        auto dec = runFinalistSelectionDecision(results, {f1, f2}, config);
        std::cout << " Second failed but conf_margin=0.05 (< 0.10) & 1st support inferior -> ambiguous="
                  << dec.finalist_ambiguous << "\n";
        require(dec.finalist_ambiguous == true, "Should trigger ambiguous because second is viable (margin < 0.10)");
      }

      // 6.2 Second candidate failed optimization (success = false) AND conf_margin >= 0.10 (e.g. 0.30)
      // second_is_viable is false. Support comparison is not triggered against a dead candidate.
      {
        auto r1 = makeMockResult(168.0, 0.800, 400, 1000, true, true);
        auto r2 = makeMockResult(-123.0, 0.500, 1000, 1000, false /* success=false */, true);
        std::vector<auto_calib::CalibrationResult> results = {r1, r2};

        auto f1 = makeMockFinalist(0, 0.10, 0.800);
        auto f2 = makeMockFinalist(1, 0.40, 0.500);

        auto dec = runFinalistSelectionDecision(results, {f1, f2}, config);
        std::cout << " Second failed and conf_margin=0.30 (>= 0.10) -> ambiguous=" << dec.finalist_ambiguous
                  << " success=" << dec.success << "\n";
        require(dec.finalist_ambiguous == false, "Should NOT trigger ambiguous against non-viable candidate");
        require(dec.success == true, "Must succeed");
      }

      std::cout << " [PASS] Test 6: Second candidate viability semantics verified.\n";
    }

    // =========================================================================
    // TEST 7: Matching Objective Must Lead Confidence Across Distinct Basins
    // =========================================================================
    std::cout << "\n--- TEST 7: Objective-first finalist ranking ---\n";
    {
      // Reproduces the build21 failure shape: the physically plausible basin
      // has the lower matching objective, while a false basin has slightly
      // higher coverage-derived confidence.  Select the lower objective for
      // diagnostics, then fail closed because the independent confidence
      // signal disagrees.
      auto aligned = makeMockResult(168.0, 0.755, 709, 314, true, true);
      auto false_basin = makeMockResult(85.0, 0.783, 1093, 364, true, true);
      std::vector<auto_calib::CalibrationResult> results = {aligned, false_basin};
      auto aligned_finalist = makeMockFinalist(0, 0.736, 0.755);
      auto false_finalist = makeMockFinalist(1, 0.769, 0.783);

      const auto dec = runFinalistSelectionDecision(
          results, {aligned_finalist, false_finalist}, config);
      require(dec.selected_index == 0,
              "Lower matching objective must lead higher confidence across basins");
      require(!dec.finalist_ambiguous,
              "A decisive objective gap with adequate support must resolve confidence disagreement");
      require(dec.success,
              "Decisive matching evidence must keep the internal candidate valid");
      std::cout << " [PASS] Test 7: Decisive objective resolves confidence disagreement.\n";
    }

    // =========================================================================
    // TEST 8: Near-tied Objective Uses Absolute Structural Evidence
    // =========================================================================
    std::cout << "\n--- TEST 8: Near-tied objective TESL tie-break ---\n";
    {
      // Reproduces build20: the false basin is only 0.14% lower in objective,
      // far below the configured 2% separation, while the aligned basin
      // explains far more structural length.
      auto aligned = makeMockResult(166.0, 0.755, 699, 293, true, true);
      auto false_basin = makeMockResult(65.0, 0.774, 959, 440, true, true);
      aligned.metrics.total_explained_structural_length = 4631.0;
      false_basin.metrics.total_explained_structural_length = 1723.0;
      std::vector<auto_calib::CalibrationResult> results = {aligned, false_basin};
      auto aligned_finalist = makeMockFinalist(0, 0.7622, 0.755);
      auto false_finalist = makeMockFinalist(1, 0.7611, 0.774);

      const auto dec = runFinalistSelectionDecision(
          results, {aligned_finalist, false_finalist}, config);
      require(dec.selected_index == 0,
              "Near-tied objective must prefer substantially stronger TESL evidence");
      require(dec.finalist_ambiguous,
              "Conflicting confidence must still reject the single-scene RT");
      require(dec.reason_code == "FINALIST_AMBIGUOUS",
              "Near-tie disagreement must report FINALIST_AMBIGUOUS");
      std::cout << " [PASS] Test 8: Near-tied objective uses TESL and fails closed.\n";
    }

    // =========================================================================
    // TEST 9: Multi-scene Objective Dominance with Small Confidence Margin
    // =========================================================================
    std::cout << "\n--- TEST 9: Multi-scene objective dominance ---\n";
    {
      auto aligned = makeMockResult(167.0, 0.760, 2558, 1093, true, true);
      auto false_basin = makeMockResult(87.0, 0.746, 2971, 1078, true, true);
      aligned.metrics.total_explained_structural_length = 13219.0;
      false_basin.metrics.total_explained_structural_length = 7766.0;
      std::vector<auto_calib::CalibrationResult> results = {aligned, false_basin};
      auto aligned_finalist = makeMockFinalist(0, 0.7530, 0.760);
      auto false_finalist = makeMockFinalist(1, 0.8180, 0.746);

      const auto dec = runFinalistSelectionDecision(
          results, {aligned_finalist, false_finalist}, config);
      require(dec.selected_index == 0,
              "Multi-scene lower objective basin must be selected");
      require(dec.objective_margin > 0.02,
              "Fixture must retain a decisive objective gap");
      require(dec.conf_margin < 0.02,
              "Fixture must retain a sub-threshold confidence margin");
      require(!dec.finalist_ambiguous && dec.success,
              "Decisive objective plus adequate support must pass internal ambiguity gate");
      std::cout << " [PASS] Test 9: Multi-scene objective dominance retained.\n";
    }

    // =========================================================================
    // TEST 10: Three-way Near-tie Ranking Must Be Order-independent
    // =========================================================================
    std::cout << "\n--- TEST 10: Deterministic three-way ranking ---\n";
    {
      auto a = makeMockResult(0.0, 0.60, 1000, 1000, true, true);
      auto b = makeMockResult(120.0, 0.80, 1000, 1000, true, true);
      auto c = makeMockResult(-120.0, 0.70, 1000, 1000, true, true);
      a.metrics.total_explained_structural_length = 100.0;
      b.metrics.total_explained_structural_length = 80.0;
      c.metrics.total_explained_structural_length = 60.0;
      const std::vector<auto_calib::CalibrationResult> results = {a, b, c};
      const std::vector<FinalistEntry> base = {
          makeMockFinalist(0, 1.030, 0.60),
          makeMockFinalist(1, 1.015, 0.80),
          makeMockFinalist(2, 1.000, 0.70)};

      // A pairwise threshold comparator can cycle here: A beats B on TESL,
      // B beats C on TESL, while C beats A on the objective.  The explicit
      // staged selector must return B for every input ordering.
      std::vector<int> order = {0, 1, 2};
      do {
        std::vector<FinalistEntry> permuted;
        for (const int idx : order)
          permuted.push_back(base[static_cast<std::size_t>(idx)]);
        const auto dec =
            runFinalistSelectionDecision(results, permuted, config);
        require(dec.selected_index == 1,
                "Three-way finalist selection must not depend on input order");
      } while (std::next_permutation(order.begin(), order.end()));
      std::cout << " [PASS] Test 10: All 6 finalist orders select the same basin.\n";
    }

    // =========================================================================
    // TEST 11: Hold-out Must Distinguish Separated Viable Finalists
    // =========================================================================
    std::cout << "\n--- TEST 11: Candidate-specific hold-out distinctiveness ---\n";
    {
      const std::vector<HoldoutCandidate> ambiguous = {
          {167.0, 1.0, 0.400, true},
          {87.0, 1.0, 0.405, true},
          {170.0, 1.0, 0.100, true}};
      require(!holdoutIsDistinctive(
                  0, ambiguous, config.minimum_multistart_objective_margin),
              "A near-tied separated viable finalist must fail closed");

      const std::vector<HoldoutCandidate> distinctive = {
          {167.0, 1.0, 0.350, true},
          {87.0, 1.0, 0.500, true},
          {170.0, 1.0, 0.100, true}};
      require(holdoutIsDistinctive(
                  0, distinctive, config.minimum_multistart_objective_margin),
              "A decisively worse separated finalist must not block the selected pose");

      const std::vector<HoldoutCandidate> lower_pass_competitor = {
          {167.0, 1.0, 0.400, true}, {87.0, 0.5, 0.100, true}};
      require(holdoutIsDistinctive(
                  0, lower_pass_competitor,
                  config.minimum_multistart_objective_margin),
              "A separated finalist in a lower pass-ratio tier must not block the selected pose");

      const std::vector<HoldoutCandidate> nonviable_competitor = {
          {167.0, 1.0, 0.400, true}, {87.0, 1.0, 0.100, false}};
      require(holdoutIsDistinctive(
                  0, nonviable_competitor,
                  config.minimum_multistart_objective_margin),
              "A finalist that failed training/core gates is not a viable hold-out competitor");
      std::cout << " [PASS] Test 11: Hold-out pass-ratio tier and objective margin verified.\n";
    }

    std::cout << "\n=========================================================================\n";
    std::cout << "[CHALLENGER-M2-1] ALL ADVERSARIAL FINALIST AMBIGUITY TESTS PASSED (100%)!\n";
    std::cout << "=========================================================================\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "\n[CHALLENGER-M2-1 FAILURE] " << e.what() << "\n";
    return 1;
  }
}

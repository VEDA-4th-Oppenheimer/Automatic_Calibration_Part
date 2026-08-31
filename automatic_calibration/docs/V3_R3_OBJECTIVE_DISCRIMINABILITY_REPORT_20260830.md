# V3-R3 Manual Reference Objective Discriminability Audit

> Offline diagnostic only. No analyzer, objective, weight, threshold, optimizer, fallback, K/D, or product path was modified.

## 1. Scope and production path

- Branch/base: `exp-v3-r3-objective-discriminability` / `8a18332`.
- Evaluated builds: `build45, build46, build48, build49, build50`; build47 excluded; build51 reference-generation data excluded.
- Manual RT: `/workspace/manual_calibration/output/manual_projection_refiner_build51_20260830/manual_reference_candidate/manual_reference_rt.json`; `reference_ground_truth=false`, evaluator-only.
- Camera center contract applied to every audit pose: `(0.0, -0.08105, 0.0)` m; `t=-R*C_lidar`.
- Each pose was evaluated by the existing `--validation-pose-json` fixed-pose path, which calls the production `evaluateCalibrationPoseScenes()` implementation.
- The production source does not serialize an aggregate fixed-pose composite, so the evaluator reproduces the source `summarizeCalibrationPoseScenes()` formula and records the formula/evidence explicitly.

## 2. Candidate set

Reference, yaw/down/optical-roll ±1/5/10° perturbations, analyzer Top-3, persisted bounded 5°/1° winners, Ceres diagnostic candidates, operational estimated poses, the visualization pose, and a left-composed 180° wrong branch were evaluated.

## 3. Build-level summary

| build | reference composite | local ±1° minimum | reference vs 180° margin | nearest analyzer error | production rank change | classification |
|---|---:|---|---:|---:|---:|---|
| build45 | 0.809778398425212 | False | 0.6909241371057879 | 13.009982304100115 | 1 | REFERENCE_LOCAL_MINIMUM_NOT_DISTINCTIVE+MIXED_SCORE_CONFLICT |
| build46 | 0.7705885706204383 | False | 0.7140081582580617 | 13.723223425799096 | 0 | REFERENCE_LOCAL_MINIMUM_NOT_DISTINCTIVE+MIXED_SCORE_CONFLICT |
| build48 | 0.812706697981872 | False | 0.7472153153196279 | 29.930912365617292 | 0 | REFERENCE_LOCAL_MINIMUM_NOT_DISTINCTIVE+MIXED_SCORE_CONFLICT |
| build49 | 0.8006668630056741 | False | 0.6941481784728258 | 30.39251700297973 | 5 | REFERENCE_LOCAL_MINIMUM_NOT_DISTINCTIVE+COVERAGE_BIAS_DOMINATES+MIXED_SCORE_CONFLICT |
| build50 | 0.9242289765010024 | False | 0.2436802079129975 | 44.284553082297876 | 6 | REFERENCE_LOCAL_MINIMUM_NOT_DISTINCTIVE+COVERAGE_BIAS_DOMINATES |

## 4. Numeric interpretation

The objective is minimized. Every margin below is `other - reference`; positive means the Reference has the lower objective.

| build | Reference | automatic final | auto-reference margin | auto reference error (deg) | nearest analyzer error (deg) | Reference vs 180° margin |
|---|---:|---:|---:|---:|---:|---:|
| build45 | 0.809778 | 0.795421 | -0.014358 | 31.364358 | 13.009982 | 0.690924 |
| build46 | 0.770589 | 0.745741 | -0.024848 | 12.452443 | 13.723223 | 0.714008 |
| build48 | 0.812707 | 0.755638 | -0.057068 | 26.189521 | 29.930912 | 0.747215 |
| build49 | 0.800667 | 0.698830 | -0.101837 | 20.776462 | 30.392517 | 0.694148 |
| build50 | 0.924229 | 1.036402 | 0.112173 | 168.263640 | 44.284553 | 0.243680 |

### build45 13° versus selected 31° branch

- The nearest Top-3 proposal is `build45_analyzer_top3_rank_2` at `13.009982°`; its production objective is `0.832724`.
- The selected operational pose is `build45_operational_estimated_rt_selected` at `31.364358°`; its production objective is `0.795421`.
- Selected-minus-Reference composite delta is `-0.014358`. The negative value means the unchanged production objective prefers the selected branch.
- The exact weighted component deltas are in `build45_score_inversion_trace.json`; a negative delta means the selected branch has the lower/better contribution.

### Component consistency against automatic final

| component | Reference preferred build count | weight | interpretation |
|---|---:|---:|---|
| edge_alignment | 5/5 | 0.250000 | reference-favoring |
| geometry_nid | 1/5 | 0.550000 | conflicted/non-discriminative |
| signal_nmi | 5/5 | 0.000000 | diagnostic-only; weight is zero |
| structural_line | 2/5 | 0.200000 | conflicted/non-discriminative |
| manhattan | 1/5 | 0.150000 | conflicted/non-discriminative |
| direction_prior | 0/5 | 0.000000 | diagnostic-only; weight is zero |
| coverage_penalty | 4/5 | 0.250000 | reference-favoring |

## 5. Required questions

1. **Reference local minimum:** see the per-build `local_perturbation_landscape.csv`; the answer is not inferred from candidate count. The aggregate condition is recorded as `reference_local_minimum_in_multiple_builds` in `component_discriminability_summary.json`.
2. **180° branch:** `pairwise_reference_margins.csv` and `wrong_branch_comparison.json` report the signed `wrong-reference` margin. Positive means the Reference is preferred; negative means the wrong branch wins.
3. **build45 inversion:** `build45_score_inversion_trace.json` reports the closest analyzer proposal, the selected automatic result, every weighted component delta, and the exact production/common-support ranking.
4. **Consistent components:** `component_discriminability_summary.json` contains per-build and five-build counts for Reference preferred vs automatic/analyzer/wrong-branch comparisons.
5. **Wrong-branch-favoring components:** the same summary lists negative Reference margins by component; no component is silently discarded.
6. **Coverage bias:** production-support and common-support ranks are side-by-side in `common_support_comparison.csv`; a rank change is reported rather than corrected.
7. **Next target:** this audit does not authorize a score change. If the stop conditions below are true, the next review target is the objective/component design before increasing candidate count.

## 5. Common support limitation

Strict point-ID intersection cannot be computed from the existing runner output because it contains support counts, not visible point IDs. It is therefore recorded as `NOT_COMPARABLE`. The common-support comparison uses the maximum observed support counts across the audit candidates and the exact production coverage penalty formula; it is not presented as a strict point intersection.

The fixed-pose production CSV does not serialize `range_entropy_ratio`, `normal_entropy_ratio`, `signal_entropy_ratio`, or `manhattan_horizontal_error_deg`. These fields are explicitly present in `objective_components.csv` with `NOT_SERIALIZED_BY_FIXED_POSE_VALIDATION`; no value was inferred from another pose or imputed.

## 6. Stop decision

- Overall: **SOL_REVIEW_REQUIRED_STOP_BEFORE_SCORE_MODIFICATION**.
- Objective/score review required: `True`.
- No score weight, threshold, analyzer, optimizer, fallback, K/D, or product RT was changed.
- Manual Reference remains diagnostic only; this report does not declare ground truth, product PASS, or Phase 1 GO.

## 7. Validation

- Validation status: `PASS`.
- Production scoring path available: `False`.
- Automatic-output immutability checks: `True`.
- Finite JSON/CSV and runner checks: `True`.

## 8. Evidence

- `audit_candidate_manifest.csv` — pose/source/provenance manifest.
- `objective_components.csv` / `weighted_contributions.csv` — production/common-support components and contributions.
- `pairwise_reference_margins.csv` / `local_perturbation_landscape.csv` — signed comparisons.
- Candidate funnel loss is outside this audit; V3-R3 classifies objective discriminability only.

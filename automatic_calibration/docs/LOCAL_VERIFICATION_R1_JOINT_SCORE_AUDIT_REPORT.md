# V3 Local Verification R1 — Joint Score Audit

- Date: 2026-08-31
- Source: `automatic_calibration\generated\reference_anchored_local_verification_20260831`
- Execution: existing CSV join only; no grid search, threshold/weight change, or Ceres
- Candidate key: `(yaw_offset_deg, downward_offset_deg, optical_roll_offset_deg)`
- Builds: build45, build46, build48, build49

## Method

Per-build rank percentile is `(rank - 1) / (N - 1)`; lower is better. Joint rank is sorted by median percentile and then worst-build percentile. A basin is one-grid-cell Chebyshev adjacency to a build's rank-1 candidate; this is an audit definition and does not change a production threshold.

## production

- Classification: **OBJECTIVE_DRIFT_TO_SEARCH_BOUNDARY**
- Joint winner: `1/1/5`; boundary=`True`; median percentile=`0.003383`; worst-build percentile=`0.124812`
- Nominal `(0,0,0)` joint rank: **356**; median percentile=`0.284586`
- Winner basin support: `build45` (1/4)
- Per-build rank-1 winners: build45=(1,0,5); build46=(1,-1,5); build48=(1,3,0); build49=(4,5,5)

| Joint rank | Candidate `(yaw/down/roll)` | Boundary | Median pct | Worst pct | Basin support | Per-build ranks |
|---:|---|:---:|---:|---:|---:|---|
| 1 | `1/1/5` | True | 0.003383 | 0.124812 | 1/4 | build45:4, build46:6, build48:5, build49:167 |
| 2 | `1/0/5` | True | 0.004135 | 0.115038 | 2/4 | build45:1, build46:4, build48:9, build49:154 |
| 3 | `1/-1/5` | True | 0.009023 | 0.124060 | 2/4 | build45:3, build46:1, build48:23, build49:166 |
| 4 | `1/2/3` | False | 0.009398 | 0.222556 | 0/4 | build45:13, build46:8, build48:14, build49:297 |
| 5 | `1/2/4` | False | 0.009774 | 0.132331 | 0/4 | build45:5, build46:11, build48:17, build49:177 |
| 6 | `1/1/4` | False | 0.009774 | 0.212030 | 1/4 | build45:2, build46:2, build48:26, build49:283 |
| 7 | `0/-5/3` | True | 0.010526 | 0.674436 | 0/4 | build45:11, build46:17, build48:13, build49:898 |
| 8 | `1/3/4` | False | 0.012406 | 0.054887 | 0/4 | build45:16, build46:13, build48:19, build49:74 |
| 9 | `0/-5/2` | True | 0.015038 | 0.770677 | 0/4 | build45:22, build46:20, build48:15, build49:1026 |
| 10 | `0/-4/4` | False | 0.015414 | 0.568421 | 0/4 | build45:29, build46:14, build48:7, build49:757 |

## Interpretation

The joint winner is on the configured local boundary. This is evidence that the local window is insufficient or the objective drifts toward its edge; it is not an accepted RT update.

## nid_off

- Classification: **OBJECTIVE_DRIFT_TO_SEARCH_BOUNDARY**
- Joint winner: `1/1/5`; boundary=`True`; median percentile=`0.003008`; worst-build percentile=`0.145865`
- Nominal `(0,0,0)` joint rank: **314**; median percentile=`0.254511`
- Winner basin support: `build45` (1/4)
- Per-build rank-1 winners: build45=(1,0,5); build46=(1,-1,5); build48=(1,3,0); build49=(4,5,5)

| Joint rank | Candidate `(yaw/down/roll)` | Boundary | Median pct | Worst pct | Basin support | Per-build ranks |
|---:|---|:---:|---:|---:|---:|---|
| 1 | `1/1/5` | True | 0.003008 | 0.145865 | 1/4 | build45:4, build46:6, build48:3, build49:195 |
| 2 | `1/0/5` | True | 0.003383 | 0.136090 | 2/4 | build45:1, build46:4, build48:7, build49:182 |
| 3 | `1/-1/5` | True | 0.006391 | 0.107519 | 2/4 | build45:3, build46:1, build48:16, build49:144 |
| 4 | `1/1/4` | False | 0.008647 | 0.209774 | 1/4 | build45:2, build46:2, build48:23, build49:280 |
| 5 | `1/2/3` | False | 0.008647 | 0.222556 | 0/4 | build45:14, build46:8, build48:11, build49:297 |
| 6 | `1/2/4` | False | 0.009398 | 0.140602 | 0/4 | build45:5, build46:9, build48:18, build49:188 |
| 7 | `1/3/4` | False | 0.013534 | 0.077444 | 0/4 | build45:19, build46:13, build48:19, build49:104 |
| 8 | `0/-5/3` | True | 0.015414 | 0.699248 | 0/4 | build45:21, build46:21, build48:22, build49:931 |
| 9 | `0/-2/2` | False | 0.015789 | 0.443609 | 0/4 | build45:10, build46:19, build48:25, build49:591 |
| 10 | `0/-4/3` | False | 0.016541 | 0.603759 | 0/4 | build45:18, build46:28, build48:6, build49:804 |

## Interpretation

The joint winner is on the configured local boundary. This is evidence that the local window is insufficient or the objective drifts toward its edge; it is not an accepted RT update.

## Decision boundary

This audit does not execute calibration and does not alter score weights, thresholds, candidate generation, Ceres, fallback behavior, or product RT. Manual Reference remains provisional; no product PASS, Ground Truth, or RT promotion is declared.

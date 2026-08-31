# V3-R3-R1 Exact Common-Support Objective Audit

> Offline diagnostic only. The production objective, weights, thresholds, analyzer, Top-K, optimizer and fallback were not changed.

## Scope

- Branch/base: `exp-v3-r3-r1-exact-common-support` / `3eef74c`.
- Evaluated builds: `build45, build46, build48, build49, build50`; build47 and build51 were excluded by the V3-R3 scope.
- Manual Reference: `/host/develop/manual_calibration/output/manual_projection_refiner_build51_20260830/manual_reference_candidate/manual_reference_rt.json`; `reference_ground_truth=false`.
- Camera-center contract: `(0.0, -0.08105, 0.0)` m; every audit pose uses `t=-R*C_lidar`.
- Stable point ID: organized `(row,column)` represented by the existing point array index; no coordinate-rounded or generated UUID was used.
- Both variable-support and exact-support values come from the C++ fixed-pose audit path. Python performs only ID intersections and pairwise subtraction.

## Candidate restriction

Per build the audit retained Manual Reference, operational selected pose, visualization pose, nearest analyzer proposal, lowest production-objective analyzer proposal, 180-degree branch, and the best persisted Reference ±1-degree perturbation. Duplicate poses reuse the same variable/common run.

## Build summary (Reference vs auto-selected/visualization pose)

| build | variable geometry margin | exact geometry margin | variable composite margin | exact composite margin | Reference NID points | selected NID points | common geometry IDs | Reference→common point reduction | classification |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| build45 | -0.08774153974975596 | 0.09385407467072837 | -0.014357587926259407 | 0.09721287795588662 | 444.0 | 175.0 | 58 | 86.93693693693693% | GEOMETRY_NID_SAMPLE_COUNT_BIAS |
| build46 | -0.0717599002176087 | -0.042701492569046096 | -0.02484760474673975 | -0.011438837506671296 | 433.0 | 259.0 | 205 | 52.65588914549654% | GEOMETRY_NID_TRUE_MISRANK |
| build48 | -0.034640492195620265 | 0.025982640615317076 | -0.05706829944495817 | -0.016998375200380722 | 463.0 | 331.0 | 183 | 60.475161987041034% | GEOMETRY_NID_SAMPLE_COUNT_BIAS |
| build49 | -0.033584357614348415 | 0.027937778882027753 | -0.10183690465195949 | -0.019551570537798102 | 382.0 | 239.0 | 130 | 65.96858638743456% | GEOMETRY_NID_SAMPLE_COUNT_BIAS |
| build50 | 0.0316663241888131 | 0.0 | 0.11217331987937573 | -0.02301347684641275 | 432.0 | 322.0 | 0 | 100.0% | COMMON_SUPPORT_TOO_SMALL |

Margins are `other - Reference`; positive means Reference has the lower objective because the production terms are minimized. Exact support is pair-specific; a zero/very small intersection is not evidence of equal pose quality.

## Component margins for the selected pose

| build | exact common geometry | exact edge | exact structural | exact Manhattan | exact composite | common geometry IDs |
|---|---:|---:|---:|---:|---:|---:|
| build45 | 0.093854 | 0.198468 | -0.077031 | 0.075883 | 0.097213 | 58 |
| build46 | -0.042701 | 0.111895 | 0.001124 | -0.107677 | -0.011439 | 205 |
| build48 | 0.025983 | 0.150793 | -0.167189 | -0.236995 | -0.016998 | 183 |
| build49 | 0.027938 | 0.099167 | -0.026858 | -0.408547 | -0.019552 | 130 |
| build50 | 0.000000 | 0.000000 | 0.092276 | -0.276458 | -0.023013 | 0 |

Positive margin means the Reference is preferred. `common_geometry_nid_point_count=0` makes geometry/edge exact-support comparisons non-comparable for that pair; the CSV still retains the actual C++ result and support count.

## build45 score inversion

The 31.36-degree auto-selected pose and the 13.01-degree analyzer/objective-winner proposal are compared using the same variable-support C++ scorer. The following values are `13.01-degree proposal - 31.36-degree pose`; positive means the 31.36-degree pose has the lower (better) objective:

| composite | edge | geometry NID | structural | Manhattan | coverage |
|---:|---:|---:|---:|---:|---:|
| 0.037303 | 0.067069 | 0.084060 | 0.000878 | -0.103163 | -0.041594 |

- Variable-support Reference→selected geometry margin: `-0.087742`; exact common-support margin: `0.093854`.
- Variable-support Reference→selected composite margin: `-0.014358`; exact common-support margin: `0.097213`.
- Variable→exact selected pair rank: `{'variable_rank': 1, 'exact_pair_rank': {'reference': 1, 'selected': 2, 'tie': False}}`.
- Interpretation: variable support favors the 31.36-degree pose, while the same-ID comparison favors the Reference; this is direct evidence that changing support can drive the observed geometry-NID inversion. It does not prove the Reference is ground truth.
- Full per-point membership and C++ objective evidence: `build45_exact_support_trace.json`.

## build49/build50 exact-support follow-up

- build49 selected pair: variable geometry margin `-0.033584` → exact `0.027938`; the rank direction reverses after exact IDs.
- build50 selected pair: common geometry IDs `0`; it is `COMMON_SUPPORT_TOO_SMALL` for the selected pose and must not be interpreted as a geometry ranking.
- For build50, the nearest analyzer proposal has a nonzero but limited exact intersection recorded in `build50_exact_support_trace.json`; all pair counts and margins are retained in `common_support_pairwise_margins.csv`.
- This replaces the prior count-only `NOT_COMPARABLE` statement only for the point-ID intersections actually measured here; it does not make a small intersection statistically sufficient.

## Required questions

1. Reference local minimum: the selected best ±1-degree perturbation is slightly better than the Reference in the recorded composite landscape in these builds, so a strict local minimum is not established. Those perturbations are explicitly manual uncertainty probes; this is not a Reference rejection.
2. 180-degree branch: variable-support composite prefers the Reference in all five builds. Exact geometry/edge support is zero for the 180-degree pair, so those terms are `NOT_COMPARABLE`; the remaining exact pose terms still retain the measured margins.
3. build45 inversion: the 31.36-degree pose wins its aggregate variable-support objective because its edge and geometry contributions improve enough to offset worse Manhattan/coverage contributions. Exact common IDs reverse the geometry preference.
4. Consistent component behavior: edge alignment favors the Reference for the selected pairs whenever a nonzero common support exists; geometry favors the Reference in three of four nonempty selected pairs and favors the other pose in build46; structural and Manhattan terms are scene-dependent rather than consistent Reference selectors.
5. Wrong-branch preference: no consistent 180-degree preference for the wrong branch is observed in variable-support composite. Several non-180 candidate comparisons show component conflict, especially Manhattan versus edge/NID.
6. Coverage: support counts differ substantially (for example build45 Reference NID 444 versus selected 175 in variable support), and exact filtering changes coverage counts. This audit demonstrates support sensitivity but does not isolate coverage as an independent causal term.
7. Candidate generation versus objective: do not change candidate generation or production score from this audit alone. The evidence supports a later review of support normalization/common-support or coverage semantics, but no score modification is implemented here.

## Decision

- Overall diagnostic classification: **GEOMETRY_NID_SAMPLE_COUNT_BIAS**.
- Score-change proposal gate: `NO_GEOMETRY_TERM_CHANGE_FROM_THIS_AUDIT`. No score modification was implemented.
- Because only one of the five selected pairs is an exact geometry true misrank and build50 selected support is empty, the >=3-build geometry-misrank condition for implementing a geometry-NID change is not met.
- Manual ±1-degree poses are uncertainty probes, not ground truth.
- This audit does not promote the Reference, declare product PASS, or authorize Phase 1.

## Evidence and validation

- `support_membership_<build>_<candidate>.csv`: C++ projection/depth/in-frame/z-buffer/use/rejection for each score-term source point.
- `exact_common_support_ids.csv`: pair- and term-specific point-ID intersections.
- `variable_vs_common_scores.csv`: C++ production variable-support versus C++ exact-filtered outputs.
- `histogram_support_statistics.csv`: C++ NID sample count, occupied bins and entropy ratio.
- No-op validation: `PASS`; existing persisted V3-R3 evidence hashes were unchanged.
- Fixed-pose audit/control no-op details: `PASS`; R/t serialization, gate status/reason and scene CSV were compared with the audit-only field and run paths excluded.
- All generated audit values were checked for finite serialization; runner nonzero status is expected for fixed-pose quality-gate failures and is retained as evidence.

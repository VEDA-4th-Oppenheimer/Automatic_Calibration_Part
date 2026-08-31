# V3-R3-R1-R1 Exact Final NID Sample Support Report

- Date: 2026-08-31
- Branch: `exp-v3-r3-r1-r1-exact-final-sample-support`
- Base commit: `c4b110d`
- Mode: audit-only; production objective and selection are unchanged
- Reference ground truth: `false`
- Product PASS/reference promotion: `false`

## Purpose

V3-R3-R1 intersected source point IDs before the production spatial balancing stage. This run first obtains the final range/normal NID sample IDs from the C++ scorer for the Reference and operational selected pose, then forces their per-term intersections through the same C++ histogram implementation.

## Inputs and scope

- R3 candidate evidence: `/workspace/automatic_calibration/generated/v3_r3_objective_discriminability_20260830`
- Previous R3-R1 evidence (read-only hash check): `/workspace/automatic_calibration/generated/v3_r3_r1_exact_common_support_20260831`
- Manual Reference: `/workspace/develop/manual_calibration/output/manual_projection_refiner_build51_20260830/manual_reference_candidate/manual_reference_rt.json`
- Builds: build45, build46, build48, build49, build50
- Exactly two poses per build: Manual Reference and operational selected
- build50 with zero final common support is excluded from evidence

## Final sample support comparison

`margin_selected_minus_reference` is selected geometry-NID objective minus Reference geometry-NID objective; positive means Reference is lower/better.

| build | common geometry IDs | common range | common normal | ref range/normal | selected range/normal | ref NID | selected NID | margin | comparable | classification |
|---|---:|---:|---:|---:|---:|---:|---:|---:|:---:|---|
| build45 | 58 | 58 | 58 | 58/58 | 58/58 | 0.8994821799949322 | 0.8811192871551219 | -0.03269699393168546 | True | GEOMETRY_NID_DISCRIMINABILITY_FAILURE |
| build46 | 205 | 205 | 205 | 205/205 | 205/205 | 0.9461736214723713 | 0.9362614435873639 | -0.018659031221847644 | True | GEOMETRY_NID_DISCRIMINABILITY_FAILURE |
| build48 | 183 | 183 | 183 | 183/183 | 183/183 | 0.9187276652137666 | 0.9349019252896649 | 0.0299810870811833 | True | SAMPLE_SUPPORT_BIAS_CONFIRMED |
| build49 | 130 | 130 | 130 | 130/130 | 130/130 | 0.9299842784943568 | 0.9222936557321562 | -0.014245170842884458 | True | GEOMETRY_NID_DISCRIMINABILITY_FAILURE |
| build50 | 0 | 0 | 0 | None/None | None/None | None | None | None | False | COMMON_SUPPORT_TOO_SMALL |

## Result

- Overall classification: `GEOMETRY_NID_DISCRIMINABILITY_FAILURE`
- Comparable builds: `4`
- Variable-support ranking flips: `1`
- Corrected exact-support Reference wins: `1`
- Corrected exact-support selected wins: `3`
- build50 zero common sample is not counted as support for either conclusion.

## Validation

- Exact range/normal sample-count equality: `True`
- Audit/control no-op: `PASS`
- Previous V3-R3-R1 evidence unchanged: `True`
- Finite/proper-pose checks: `True`
- Core tests: `PASS`
- Analyzer tests: `PASS`

## Interpretation

This evidence only answers whether the previous NID comparison was affected by different final histogram sample populations. It does not modify the production score, weights, thresholds, optimizer, analyzer, fallback, K/D, or product RT policy. A score change remains blocked pending Sol review.

## Execution evidence

- Runtime rows: `19`
- Detailed C++ support records are in each `runner_runs` directory; `sample_point_indices` in `fixed_pose_support_audit.json` is the authoritative final NID sample list.

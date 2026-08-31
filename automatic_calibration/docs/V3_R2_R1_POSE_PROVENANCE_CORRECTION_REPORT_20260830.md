# V3-R2-R1 Candidate Pose Provenance Correction Report

> Correction status: COMPLETED as an offline diagnostic/evaluator correction.
> Calibration core, analyzer, optimizer, score, threshold, fallback, K/D, and runner decisions were not changed.

## 1. Scope and correction

The previous V3-R2 evaluator labeled the serialized `estimated` pose as the Ceres finalist. This is incorrect on a FAIL result. This report keeps four distinct roles: `ceres_diagnostic_candidate`, `candidate_gate_result`, `operational_estimated_rt`, and `visualization_rt`. `final_product_candidate` is emitted only for a PASS-like status.

Manual Reference was used only after execution: `/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/manual_calibration/output/manual_projection_refiner_build51_20260830/manual_reference_candidate/manual_reference_rt.json`. `reference_ground_truth=false`; the reference is an offline diagnostic comparison target, not ground truth.

## 2. Role contract

| Role | JSON source | Meaning |
|---|---|---|
| `ceres_diagnostic_candidate` | `candidate_results[i].diagnostic_candidate` | Pose Ceres actually optimized. |
| `candidate_gate_result` | `internal_gate_pass`, `success`, `state`, `reason_code` | Gate transition metadata; not a pose selection. |
| `operational_estimated_rt` | `candidate_results[i].estimated` / top-level `estimated_t_camera_lidar` | Post-gate/caller-facing operational value; on FAIL it is not an accepted product RT. |
| `visualization_rt` | `visualization_t_camera_lidar` | Pose used for the overlay, with `visualization_pose_source`. |
| `final_product_candidate` | top-level estimated only when PASS-like | Absent/`NOT_AVAILABLE` on FAIL. |

## 3. Recomputed reference errors

The values below are read from the persisted R1 JSON and compared to the manual reference by the evaluator only.

| Build | Status | Analyzer proposal min | Bounded 5° seed | Bounded 1° seed | Ceres diagnostic min | Operational estimated | Visualization | Product candidate |
|---|---|---:|---:|---:|---:|---:|---:|---|
| build45 | INTERNAL_GATE_PASS | 13.009982° | 15.583290° | 15.583290° | 15.790462° | 31.364358° | 31.364358° | available |
| build46 | INTERNAL_GATE_PASS | 13.723223° | 12.457788° | 12.457788° | 12.452443° | 12.452443° | 12.452443° | available |
| build48 | INTERNAL_GATE_PASS | 29.930912° | 24.764400° | 26.572223° | 26.189521° | 26.189521° | 26.189521° | available |
| build49 | INTERNAL_GATE_PASS | 30.392517° | 23.766728° | 21.286793° | 20.776462° | 20.776462° | 20.776462° | available |
| build50 | FAIL | 44.284553° | 42.740196° | 41.790554° | 41.723008° | 168.263640° | 41.723008° | NOT_AVAILABLE (FAIL) |

The minimum analyzer proposal rows are the persisted proposal records. Bounded rows are diagnostic pose completions from the saved search seed fields; they are not new optimization runs.

## 4. Build50 correction

- Analyzer rank-3 proposal reference error: **44.284553°**.
- Candidate 0 Ceres diagnostic pose reference error: **41.723008°**.
- Candidate 0 operational estimated pose reference error: **168.263640°**.
- Visualization pose reference error: **41.723008°**, because `visualization_pose_source=rejected_optimization_candidate` and the visualization uses the diagnostic candidate.
- Candidate 0 was `internal_gate_pass=false`, `success=false`, `state=INTERNAL_GATE_FAIL`, `reason_code=OBJECTIVE_IMPROVEMENT_INSUFFICIENT`.
- Candidate 1 diagnostic/operational errors are 174.751464° / 167.511649°.
- Status is `FAIL`; therefore no accepted/product candidate is present. The 168.263640° value is the safe/operational estimated return, not the Ceres result and not a product PASS.

### Build50 flow interpretation

The analyzer did produce a candidate basin, but build50's persisted analyzer status is `NOT_A_1_DEGREE_FULL_SEARCH` with reason `yaw/down grid is not 1 degree over 360x90 degrees`; `basin_candidate_count=2`, `bounded_internal_yaw_candidates=38`, and `orientation_analyzer_fallback_triggered=false`. Thus analyzer search coverage is limited by the recorded strategy, but it does not explain the old 168° Ceres label. The diagnostic Ceres pose was slightly better than the rank-3 proposal (44.284553° → 41.723008°), then failed the internal quality gate. The runner exposed a different `estimated` pose after gate handling, which explains the apparent 168° regression in the old evaluator. No fallback was triggered. That is a safety/acceptance boundary (the result remained FAIL), while the absence of a recovery branch is a separate recovery-capability gap; this audit does not modify it.

## 5. PASS-build provenance

For build45, build46, build48, and build49, the selected candidate is internally gate-passing; its diagnostic, estimated, and visualization poses are equal in the persisted evidence. The new provenance files still keep their roles separate and do not promote any RT beyond the recorded runner status.

## 6. Validation

- Evaluator self-test: `{'pose_contract': True, 'rotation_proper': True, 'camera_center_contract': True, 'circular_yaw_distance': True}`.
- Finite/proper-pose checks: `True`; invalid rows: `[]`.
- Overall correction validation: **PASS**.
- Existing R2 evidence remains preserved; this correction adds a new output directory and does not delete historical audit records.

## 7. Limitations

This is a provenance/evaluator correction, not an algorithm improvement. It does not change the selected RT, gate, status, fallback, or calibration result. A FAIL result must remain FAIL even when an operational estimated pose is serialized. Product acceptance and ground truth remain outside this diagnostic report.

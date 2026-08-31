# V3-R1 Proposal Pose Contract and Camera-Center Lock Report

Date: 2026-08-30
Branch: `exp-v3-proposal-pose-center-lock`
Base: `06ed877`

## 1. Executive result

This work is diagnostic only. It does not produce a product RT, ground truth,
or a Phase 0C/Phase 1 approval.

The explicit camera-center constraint is now enforced as a hard geometric
contract. Across the ten B0/V3 automatic runs, the recovered camera center is
within `1.5965e-12 mm` of `[0, -0.08105, 0] m` and the translation norm is
`81.05 mm`. The previously observed build49 V3 center error of approximately
`139.046 mm` is therefore removed by the hard constraint.

The V3 analyzer still does not identify the offline manual-reference basin:
all five evaluated builds have `recall@1`, `recall@3`, and `recall@5` equal to
false. V3 reduces the bounded scene-evaluation count by `71.43%` for build45,
46, and 48, and by `77.38%` for build49 and 50, but this reduction is not an
accuracy or total-projection reduction claim. Build50 remains a wrong-basin
case. The analyzer proposal/score path therefore remains unresolved for
product use.

## 2. Scope and input contract

### Code and branch history

The implementation was split into the following commits:

| Commit | Scope |
|---|---|
| `d72d795` | Serialize analyzer proposal full-pose diagnostics at the runner boundary. |
| `90ad774` | Add offline manual-reference basin-recall evaluator and evidence outputs. |
| `12a3de7` | Enforce the explicit camera center with a rotation-only Ceres manifold and tests. |
| final commit | Preserve runner pipeline runtime during `--reuse-existing` aggregation and add this report. |

No push was performed.

### Inputs

- Data root: `/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/data/jenkins-capture/scene0`
- Operational intrinsic: `/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json`
- Offline reference: `/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/manual_calibration/output/manual_projection_refiner_build51_20260830/manual_reference_candidate/manual_reference_rt.json`
- Evaluation output: `/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/analyzer_eval_v3_worktree/automatic_calibration/generated/v3_r1_proposal_pose_center_lock_20260830/`
- Camera center contract: `C_lidar = [0, -0.08105, 0] m`
- Camera channel: CH1
- Intrinsics: clean18 operational profile, raw distortion, intrinsic refinement disabled
- `range_offset_m`: `0.084`, applied once by the runner

Build roles were recorded in `input_manifest.csv`:

- evaluated: build45, build46, build48, build49, build50
- excluded diagnostic: build47 (`ACQUISITION_BOUNDARY_INCOMPLETE`)
- reference-source only: build51

The automatic B0 and V3 runs used the same input pair, camera center, K/D,
distortion state, intrinsic-refinement setting, scene-pass policy, and
direction-prior weight. The only intentional mode differences were:

- B0: `--search-strategy legacy --orientation-analyzer off`
- V3: `--search-strategy staged --orientation-analyzer hybrid`

The manual reference was never passed to the automatic runner, analyzer,
optimizer, score, boundary, or fallback path. It was loaded by the Python
evaluator for offline comparison only.

## 3. Stage A — Proposal full-pose contract

`run_real_calibration.cpp` now records, for each analyzer proposal, the
proposal ID/rank, analyzer yaw/down/roll and uncertainty, raw/basin scores,
confidence, source stage, and the full pose fields:

- `rotation_matrix_camera_lidar`
- `translation_m_camera_lidar`
- `camera_center_lidar_m`
- transform convention
- coordinate-contract version

The convention is:

```text
p_camera = R_camera_lidar * p_lidar + t_camera_lidar
C_lidar = -R_camera_lidar^T * t_camera_lidar
```

The runner uses its existing `make_prior` construction for the diagnostic
proposal pose and applies the configured center contract as
`t = -R * C`. It does not use evaluator-side yaw conversion, manual-reference
injection, or any score/ranking change. The bounded search and candidate
selection behavior remain unchanged by this serialization.

The build45 audit/control check was run with identical options except output
directory and audit mode. It showed:

- selected candidate unchanged
- status/reason unchanged
- final R/t bitwise equal
- 3 proposals in each audit record
- all proposal poses finite and contract-valid
- maximum proposal orthonormal error: `3.33e-16`
- determinant range: `[0.9999999999999998, 1.0000000000000002]`
- maximum proposal camera-center error: `2.80e-14 mm`

Evidence directories:

- `automatic_calibration/generated/v3_r1_stage_a_audit_build45/`
- `automatic_calibration/generated/v3_r1_stage_a_control_build45/`

## 4. Stage B — Manual-reference basin recall

The reference is explicitly marked `reference_ground_truth=false` and
`manual_reference_injected=false`. The diagnostic tolerance for the recall
classification is 10 degrees. The proposal evaluator compares every V3
Top-K proposal with the reference rotation and separately records center
error, pose-contract validity, and orthonormality.

| Build | Rank-1 reference rotation error | Final reference rotation error | Recall @1/@3/@5 | Top-K within 10 deg | Failure classification |
|---|---:|---:|---|---|---|
| build45 | 34.8123 deg | 31.3644 deg | false / false / false | no | `ANALYZER_REFERENCE_BASIN_MISSED` |
| build46 | 55.4836 deg | 12.4524 deg | false / false / false | no | `ANALYZER_REFERENCE_BASIN_MISSED` |
| build48 | 170.2000 deg | 26.1895 deg | false / false / false | no | `ANALYZER_REFERENCE_BASIN_MISSED` |
| build49 | 134.9227 deg | 20.7765 deg | false / false / false | no | `ANALYZER_REFERENCE_BASIN_MISSED` |
| build50 | 133.1660 deg | 168.2636 deg | false / false / false | no | `ANALYZER_REFERENCE_BASIN_MISSED` |

`wrong_180_branch_present=true` was recorded for every build. A smaller final
error than the proposal error does not change the classification: because the
reference basin was absent from the analyzer Top-K, the result is not evidence
that bounded refinement reliably recovered the correct basin.

### Build50 failure trace

The build50 V3 proposal set was:

| Rank | yaw | down | optical roll | raw score | basin score | confidence | reference rotation error |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | -60 deg | 18 deg | 4.2707 deg | 0.79121 | 0.75747 | 0.14486 | 133.1660 deg |
| 2 | 16 deg | 32 deg | 4.2707 deg | 0.74782 | 0.73898 | 0.13415 | 150.3450 deg |
| 3 | -151 deg | 44 deg | 0 deg | 0.76819 | 0.72747 | 0.19927 | 44.2846 deg |

The selected final pose was approximately `yaw=-152 deg`, `down=39.5882
deg`, `roll=0 deg`, with `168.2636 deg` reference rotation error and
`OBJECTIVE_IMPROVEMENT_INSUFFICIENT`. `fallback_triggered=false` is the
observed runner behavior; this experiment has no automatic fallback threshold
that turns this diagnostic condition into a full-search fallback. It must not
be interpreted as approval.

## 5. Stage C — Camera-center hard constraint

The previous soft/independent translation behavior was replaced for the
explicit-center path by a Ceres `Manifold` with three rotational degrees of
freedom. Every candidate state maintains:

```text
t_camera_lidar = -R_camera_lidar * C_lidar
```

Consequences:

- translation is not an independent optimization variable when the explicit
  center option is enabled;
- the relation is restored after every rotation update;
- translation is not normalized after an independent solve;
- the no-explicit-center path remains separate;
- score weights, thresholds, K/D, analyzer logic, and coordinate contract were
  not changed.

Before this change, the Stage-B build49 V3 record had camera-center error
`0.1390459833 m` (`139.046 mm`). After the change, the final B0/V3 records
showed:

- translation norm: `81.05 mm` within floating-point serialization error;
- maximum camera-center error: `1.5965e-12 mm`;
- all R matrices finite, orthonormal, and determinant `+1`;
- range offset recorded as exactly one application.

This resolves the physical-center contract only. It does not validate the
orientation or the cross-modal basin selected by the analyzer.

## 6. Stage D — B0/V3 results

### Automatic result comparison

| Build | B0 selected (yaw/down/roll) | B0 status | B0 ref rot err | V3 selected (yaw/down/roll) | V3 status | V3 ref rot err | V3 center err |
|---|---|---|---:|---|---|---:|---:|
| build45 | 75 / 15 / 0 | FAIL | 168.8458 deg | 140 / 41.9342 / 0 | `INTERNAL_GATE_PASS` | 31.3644 deg | 1.524e-12 mm |
| build46 | 75 / 30 / 0 | FAIL | 168.4274 deg | 158 / 33.0653 / 6.2006 | `INTERNAL_GATE_PASS` | 12.4524 deg | 1.527e-12 mm |
| build48 | 75 / 30 / 0 | FAIL | 168.4274 deg | 140 / 30.7806 / 8.5273 | `INTERNAL_GATE_PASS` | 26.1895 deg | 1.597e-12 mm |
| build49 | 75 / 45 / 0 | `INTERNAL_GATE_PASS` | 93.5093 deg | 152 / 39.1998 / 0 | `INTERNAL_GATE_PASS` | 20.7765 deg | 1.553e-12 mm |
| build50 | 165 / 30 / 0 | FAIL | 168.4274 deg | -152 / 39.5882 / 0 | FAIL | 168.2636 deg | 1.539e-12 mm |

The V3 mean reference rotation error is `51.8093 deg`, compared with
`153.5274 deg` for B0. This is a diagnostic comparison against a provisional
offline reference, not an accuracy claim.

### Search work and runtime

| Builds | B0 orientation candidates | V3 orientation candidates | B0 scene evaluations | V3 bounded scene evaluations | Reduction |
|---|---:|---:|---:|---:|---:|
| build45/46/48 | 168 | 9 | 168 | 48 | 71.43% |
| build49/50 | 168 | 6 | 168 | 38 | 77.38% |

The mean bounded-evaluation reduction is `73.81%`. The analyzer separately
reported 360 evaluated signature yaw positions per build and
`expensive_projection_evaluations=0`; that counter is not combined with the
bounded runner count. Accordingly, the evidence records
`total_projection_reduction_claim=NOT_COMPUTED`.

The persisted pipeline runtimes recovered from the runner logs are:

- B0 mean: `76.344 s`
- V3 mean: `84.483 s`
- V3 analyzer mean: `12.740 s`
- V3 wall-time overhead versus B0 mean: approximately `10.66%`

Thus, reducing bounded candidate evaluations did not yet reduce total wall
time in this host run. Analyzer runtime and bounded projection work must be
optimized or measured separately before making a product performance claim.

## 7. Evidence and validation

The primary evidence archive is:

`automatic_calibration/generated/v3_r1_proposal_pose_center_lock_20260830/`

It contains:

- `proposal_full_pose.csv`
- `proposal_vs_reference.csv`
- `reference_basin_recall.json`
- `final_rt_vs_reference.csv`
- `camera_center_contract.csv`
- `build50_failure_trace.json`
- `runtime_comparison.csv`
- `validation_checks.json`
- `input_manifest.csv`
- B0/V3/fixed-reference run logs and projection/debug artifacts

The final `validation_checks.json` is `PASS` for the evaluator-owned checks:

- evaluator self-tests: all five pass;
- proposal count: 15;
- proposal full poses finite: true;
- proposal pose contract: true;
- maximum orthonormal error: `3.3307e-16`;
- determinant range: `[0.9999999999999998, 1.0000000000000002]`;
- maximum proposal center error: `2.7972e-14 mm`;
- runner JSON finite: true;
- evaluator-owned CSV finite: true.

The following regression command was rerun in the Ubuntu Docker environment:

```text
ctest --test-dir automatic_calibration/.v3-r1-build \
  -R 'automatic_calibration_core_tests|hybrid_orientation_analyzer_tests' \
  --output-on-failure
```

Result: `2/2` tests passed in `33.54 s`. Python syntax compilation also
passed. The build directory used for this isolated experiment registered
additional legacy tests whose executables were not built; therefore this
report does not claim that the full CTest registration passed. Missing legacy
targets were not masked as V3-R1 success.

## 8. Interpretation and remaining work

### What was improved

1. Proposal records now carry a common camera–LiDAR full-pose contract, making
   basin recall measurable without changing analyzer ranking.
2. The physical camera-center contract is enforced exactly in the explicit
   center path; the old large center violation is eliminated.
3. B0 and V3 comparison counts distinguish analyzer work from bounded scene
   evaluations, avoiding the earlier overclaim of total projection reduction.
4. Runtime is preserved from the runner's measured `pipeline_runtime_ms` when
   evidence is re-aggregated.

### What remains unresolved

1. The analyzer Top-K did not contain the offline reference basin in any of
   the five builds. This is an analyzer proposal-coverage/identification issue
   before it is a Ceres refinement issue.
2. Build50 can finish on a wrong branch with `fallback=false` because the
   current experiment records fallback state but does not add a new fallback
   policy.
3. The camera-center constraint removes translation ambiguity at the configured
   physical center but cannot correct a wrong orientation.
4. The offline manual reference is not independently surveyed ground truth;
   all rotation-error and recall numbers are conditional diagnostic evidence.
5. V3 bounded evaluation reduction currently has analyzer overhead and did not
   improve total wall time in this run.

### Recommended next gate

Keep product activation, Phase 0C promotion, and Phase 1 on hold. The next
experiment should validate analyzer basin coverage and an explicit fail-safe
policy using an independent reference or a synthetic pose oracle. It should
measure whether the correct basin is present before changing score weights or
thresholds. No such score or fallback redesign is included in V3-R1.

## 9. Final status

```text
status = DIAGNOSTIC_ONLY
reference_ground_truth = false
product_rt_available = false
manual_reference_injected = false
phase0c_status = HOLD
phase1_status = HOLD
```

V3-R1 demonstrates a valid full-pose serialization and a functioning hard
camera-center constraint. It does not demonstrate reliable automatic RT
selection, and no result from this report may be promoted to product RT,
ground truth, or approval evidence.

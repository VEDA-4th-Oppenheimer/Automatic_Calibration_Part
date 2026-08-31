# V3 Hybrid Analyzer — Manual Reference Offline Evaluation

Date: 2026-08-30
Branch: `exp-v3-manual-reference-evaluation`
Base V3 commit: `21f816a` (`Add multi-scene analyzer consensus fail-safe`)

## 1. Scope and safety boundary

This evaluation treats the Manual Projection Refiner result as a provisional
manual reference candidate only. It was not passed to the analyzer, coarse
search, candidate generator, optimizer, initial pose, or search boundary. The
automatic runs were executed without `--manual-reference-json`; the reference
was used only after each run for offline comparison and in the separate fixed
pose validation path.

The reference was generated from build51. Therefore build51 is reference-source
data, not a hold-out. Build47 was excluded because its prior diagnostic status
was `ACQUISITION_BOUNDARY_INCOMPLETE`.

Reference contract:

- status: `MANUAL_FIXED_BASELINE_REFERENCE_CANDIDATE`
- evaluation status: `MANUAL_REFERENCE_RT`
- `reference_ground_truth=false`
- `product_rt=false`
- allowed use: offline diagnostic evaluation only
- camera center in LiDAR frame: `[0, -0.08105, 0] m`
- translation norm: `81.050000 mm`
- convention: `p_camera = R_camera_lidar * p_lidar + t_camera_lidar`
- reference SHA-256: `258db63236087e0b6fb3195ce590ea28645da51897ebeea55870221ad82df575`

The full matrix, translation, quaternion, camera center, K/D and hashes are in
`generated/v3_manual_reference_evaluation_20260830/manual_reference_snapshot.json`.

## 2. Inputs and pairing

All evaluated pairs were selected deterministically from one package containing
one `*_CH1.jpg` and one `*_pan_tilt_lidar.json`. The same CH1 channel, fixed
clean18 K/D, raw image-distortion state and `range_offset_m=0.084` contract were
used for both automatic modes.

| build | role | image SHA-256 prefix | JSON SHA-256 prefix | valid samples |
|---|---|---:|---:|---:|
| build45 | evaluated | `bf3a50e276d9` | `a2470c360800` | 40183 |
| build46 | evaluated | `7dda8e6f39bd` | `7a5183788ff0` | 40188 |
| build47 | excluded diagnostic | `aa4a60b9d52b` | `bbad3290ebcf` | 40183 |
| build48 | evaluated | `ca5a2b21e252` | `d3e66a44fd36` | 40188 |
| build49 | evaluated | `60889ddff29e` | `46a3b111a51e` | 40187 |
| build50 | evaluated | `6484ca9597f7` | `77aaf8017b09` | 40182 |
| build51 | reference source only | `5b2372d319af` | `655f1598dfa1` | 40189 |

The complete paths, full hashes and scan metadata are in
`generated/v3_manual_reference_evaluation_20260830/input_manifest.csv`.

## 3. Phase A — fixed Manual Reference RT

The exact same reference RT was applied without re-estimation to build45, 46,
48, 49 and 50. The runner's internal fixed-pose objective gate passed for all
five pairs. This is not a target-corner or ground-truth gate; `in_frame` means
only that the pinhole projection landed inside the image.

| build | fixed status | front/projected | in-frame | behind camera | core pass ratio |
|---|---|---:|---:|---:|---:|
| build45 | `INTERNAL_GATE_PASS` | 29300 | 3494 | 10883 | 1.0 |
| build46 | `INTERNAL_GATE_PASS` | 29302 | 3510 | 10886 | 1.0 |
| build48 | `INTERNAL_GATE_PASS` | 29303 | 3495 | 10885 | 1.0 |
| build49 | `INTERNAL_GATE_PASS` | 29310 | 3488 | 10877 | 1.0 |
| build50 | `INTERNAL_GATE_PASS` | 29304 | 3496 | 10878 | 1.0 |

Visual inspection of the fixed-reference `06_projection_final.png` outputs for
build45, 46, 48, 49 and 50 shows the projected point field in the board/room
region rather than an obvious 180-degree opposite projection. The target moves
within the images, so this supports repeatability of the provisional manual
overlay only; it does not establish absolute correctness. The complete debug
images and logs remain under
`generated/v3_manual_reference_evaluation_20260830/phase_a_fixed_reference/`.

## 4. Phase B — independent B0 and V3 runs

Common options were identical. Only output directory and the search mode were
different:

```text
--manual-intrinsic-json .../camera_intrinsic.json
--ldc-enabled false --image-distortion-state raw
--allow-intrinsic-refinement false
--camera-center-x-m 0 --camera-center-y-m -0.08105 --camera-center-z-m 0
--direction-prior-weight 0 --pair-count 1 --holdout-count 0
```

Definitions used here:

- B0: `--search-strategy legacy --orientation-analyzer off`
- V3: `--search-strategy staged --orientation-analyzer hybrid`

### 4.1 Final automatic result vs provisional reference

Rotation error is `acos((trace(R_auto R_ref^T)-1)/2)`. Center error is the
LiDAR-frame distance between `-R_auto^T t_auto` and `-R_ref^T t_ref`.

| build | B0 status | B0 rot error | V3 status | V3 rot error | V3 center error |
|---|---|---:|---|---:|---:|
| build45 | `FAIL` / `OBJECTIVE_IMPROVEMENT_INSUFFICIENT` | 168.846° | `INTERNAL_GATE_PASS` | 31.941° | 14.516 mm |
| build46 | `FAIL` / `MULTISTART_AMBIGUOUS` | 168.427° | `INTERNAL_GATE_PASS` | 12.451° | 0.870 mm |
| build48 | `FAIL` / `MULTISTART_AMBIGUOUS` | 168.427° | `INTERNAL_GATE_PASS` | 26.147° | 1.406 mm |
| build49 | `INTERNAL_GATE_PASS` | 94.458° | `INTERNAL_GATE_PASS` | 20.999° | 139.046 mm |
| build50 | `FAIL` / `MULTISTART_AMBIGUOUS` | 168.427° | `FAIL` / `OBJECTIVE_IMPROVEMENT_INSUFFICIENT` | 168.264° | 0.000 mm* |

`*` build50's zero center error is a consequence of the selected result
remaining on the fixed camera-center contract; it is not evidence that its
rotation is correct.

V3 was closer to the provisional reference than B0 on 4/5 builds, but none of
the five V3 results met the diagnostic `<=2°` rotation criterion. Three V3
results were within 30°; build50 remained on a roughly 180°-separated branch.
This is an improvement in orientation proximity, not a validated automatic RT.

### 4.2 Analyzer proposals and recall limitation

The analyzer produced `PROPOSALS_READY` for all five builds, with 360
signature-yaw evaluations and no runtime fallback. Rank-1 proposal values were:

| build | rank-1 yaw | rank-1 down | rank-1 roll | analyzer runtime |
|---|---:|---:|---:|---:|
| build45 | 135° | 37° | 0° | 15.269 s |
| build46 | -139° | 44° | 0° | 15.496 s |
| build48 | -2° | 75° | 0° | 12.826 s |
| build49 | -59° | 14° | 6.471° | 12.364 s |
| build50 | -60° | 18° | 4.271° | 13.129 s |

Exact `reference_basin_recall@1/@3/@5` is **not computable from the current
artifact contract**. The analyzer JSON emits yaw/down/roll proposal priors, but
not a full proposal `R,t` in the same LiDAR-camera convention. Therefore a
provisional camera RT cannot be transformed into a valid proposal-distance
comparison without inventing a coordinate mapping. This is recorded explicitly
in `proposal_vs_reference.csv`; the final RT-vs-reference values above are an
indirect post-run comparison, not Top-K recall.

The observed build50 V3 result is a diagnostic warning: the analyzer's
rank-1 proposal did not trigger fallback, while the final RT was 168.264° from
the provisional reference. It is consistent with a missed reference basin or
an untriggered fail-safe, but the present proposal output cannot prove which
stage caused the branch selection.

### 4.3 Candidate count and runtime

| build | B0 orientation candidates | V3 analyzer candidates | B0 projection evaluations | V3 projection evaluations | reduction |
|---|---:|---:|---:|---:|---:|
| build45 | 168 | 9 | 168 | 48 | 71.43% |
| build46 | 168 | 9 | 168 | 48 | 71.43% |
| build48 | 168 | 9 | 168 | 48 | 71.43% |
| build49 | 168 | 6 | 168 | 38 | 77.38% |
| build50 | 168 | 6 | 168 | 38 | 77.38% |

Mean projection-evaluation reduction was 73.81%. Measured whole-run wall time
was 97.096 s for B0 versus 95.463 s for V3, only a 1.68% mean reduction. V3
was faster on build48/49/50 but slower on build45/46. Analyzer time was
13.62–17.71% of V3 wall time on these single-pair runs, exceeding the desired
10% budget on every build.

The measured wall times are preserved in
`generated/v3_manual_reference_evaluation_20260830/runtime_measurements.json`;
the comparison table is `runtime_comparison.csv`.

## 5. Failure classification

- `REFERENCE_REPROJECTION_NOT_REPEATABLE`: not triggered by the fixed overlay
  review; all five fixed runs landed in the image and passed the existing core
  gate. This is not a ground-truth claim.
- B0: mostly `SCORE_RANKING_AMBIGUOUS`/objective-gate failure, with final poses
  far from the provisional reference on all five builds.
- V3 build45/46/48/49: internal gate passed but reference rotation remained
  12.45–31.94°, so `QUALITY_GATE_REJECTED_GOOD_RT` is not established; the
  result is simply not within the diagnostic reference criterion.
- V3 build50: `ANALYZER_REFERENCE_BASIN_MISSED` is the leading diagnostic
  hypothesis, with `FALLBACK_NOT_TRIGGERED` as a secondary symptom. It is not
  proven because proposal transforms are not emitted.
- `INTRINSIC_OR_DISTORTION_CONFLICT`: not isolated by this run; all modes used
  the same clean18 K/D and raw distortion state.
- `INPUT_PAIR_OR_CONTRACT_ERROR`: not observed; hashes, pairing and common
  options were recorded.

## 6. Conclusion

For this evidence set, V3 is **better than B0 for orientation proximity and
projection workload**, but it is not yet accurate enough to be treated as a
reference-accurate automatic calibration. The key gains are 71–77% fewer
recorded orientation/projection evaluations and 4/5 improved final rotation
comparisons. The key losses are analyzer overhead above 10%, no build meeting
the <=2° diagnostic criterion, no computable Top-K reference recall, and a
build50 branch failure without fallback.

No product PASS, Ground Truth, product RT, Phase 0C reference, or Phase 1 GO is
declared. The manual RT remains a provisional offline diagnostic candidate.

## 7. Verification status

- Evaluator self-test: PASS (`evaluator_self_test.json`), covering identity
  rotation, camera-center contract and one-time range-offset bookkeeping.
- `calibration_core_tests`: PASS.
- `hybrid_orientation_analyzer_tests`: PASS.
- Full CTest: 10/12 passed. The two failures were legacy real-data tests
  `verify_20260818_staged` and `verify_20260819_staged`; both could not find
  their historical `/workspace/data/real_calibration/...` fixtures in this
  isolated V3 worktree. They are fixture-path/infrastructure failures, not
  failures of the requested build45–50 evaluation.
- JSON finite check: PASS. Evaluator-owned summary CSVs are finite. Existing
  runner diagnostic score CSVs contain `nan` cells for rejected candidates;
  they are preserved as raw execution evidence and were not rewritten or
  masked. This is recorded as `validation_checks.json` rather than being
  presented as an all-files-finite PASS.

## 8. Recommended next Sol review items

1. Add a diagnostic-only full `R,t` to each analyzer proposal so basin recall
   can be computed without inventing an angle convention.
2. Define and test a fail-closed rule for a proposal/final RT that is far from
   an independent offline diagnostic reference; do not tune production scores
   from this provisional reference.
3. Profile the 12.4–15.5 s analyzer cost and reduce it below 10% of total
   runtime before considering embedded deployment.
4. Reconcile the meaning of B0/V3 projection counters with the actual expensive
   scene evaluations and preserve both counters distinctly.
5. Add a target-corner/geometry validation metric; in-frame counts alone are
   insufficient for reprojection correctness.
6. Keep build51 out of hold-out statistics and repeat with an independent,
   stationary target reference before any promotion.
7. Preserve the current clean18 K/D as diagnostic-default only; do not select a
   product profile from this comparison.
8. Investigate build50's no-fallback path with the emitted analyzer proposal
   evidence before changing analyzer or objective code.

## 9. Evidence index

- Evaluator: `automatic_calibration/scripts/run_manual_reference_evaluation.py`
- Reference snapshot: `automatic_calibration/generated/v3_manual_reference_evaluation_20260830/manual_reference_snapshot.json`
- Input manifest: `automatic_calibration/generated/v3_manual_reference_evaluation_20260830/input_manifest.csv`
- Fixed-reference metrics: `automatic_calibration/generated/v3_manual_reference_evaluation_20260830/fixed_reference_reprojection_metrics.csv`
- B0 results: `automatic_calibration/generated/v3_manual_reference_evaluation_20260830/b0_results.csv`
- V3 results: `automatic_calibration/generated/v3_manual_reference_evaluation_20260830/v3_analyzer_results.csv`
- Proposal evidence: `automatic_calibration/generated/v3_manual_reference_evaluation_20260830/proposal_vs_reference.csv`
- Runtime comparison: `automatic_calibration/generated/v3_manual_reference_evaluation_20260830/runtime_comparison.csv`
- Full logs and per-build analyzer artifacts: `automatic_calibration/generated/v3_manual_reference_evaluation_20260830/b0/` and `.../v3/`

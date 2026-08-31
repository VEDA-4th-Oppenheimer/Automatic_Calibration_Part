# V3-R6-R2 Directional Proposal-Only Shadow Integration (2026-08-31)

- Verdict: **DIRECTIONAL_PROPOSAL_SHADOW_NOT_ACCEPTED**
- Branch/base: `exp-v3-r6-r2-directional-proposal-shadow` / `d038e21`
- This is a diagnostic shadow path. It does not modify the production result, fallback, weights, thresholds, K/D, or coordinate contract.

## Scope and invariants

- R5-D evidence: `/workspace/automatic_calibration/generated/v3_r5_objective_ablation_20260831/objective_ablation_comparison.csv`
- Existing analyzer proposals were the only directional-score inputs; Manual Reference was loaded after execution for offline error measurement only.
- The directional distance-transform cache was built once per observation in the C++ batch evaluator and reused for all proposals in that batch.
- At most one valid, distinct proposal was sent to the existing R5-D Ceres configuration per build.
- Directional score was not used in the composite objective or Ceres residual. A non-converged shadow candidate was never treated as operational.
- Runtime overhead is the additional shadow work divided by the immutable R5-D wall time; the production baseline itself was not rerun or modified.

## Per-build result

| build | directional rank | diagnostic error | operational error | baseline operational error | baseline status | shadow status | termination |
| --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| build45 | 1 | 168.375642 | n/a | 7.032299 | INTERNAL_GATE_PASS | FAIL | GATE_REJECTED |
| build46 | 1 | 12.777206 | 12.777206 | 10.015828 | INTERNAL_GATE_PASS | INTERNAL_GATE_PASS | CONVERGED_AND_GATE_PASS |
| build48 | 1 | 170.200048 | n/a | 21.887468 | INTERNAL_GATE_PASS | FAIL | GATE_REJECTED |
| build49 | 1 | 28.710572 | 28.710572 | 19.151480 | INTERNAL_GATE_PASS | INTERNAL_GATE_PASS | CONVERGED_AND_GATE_PASS |
| build50 | 1 | 154.636306 | n/a | 168.263640 | FAIL | FAIL | GATE_REJECTED |

## Required interpretation

- `diagnostic_rotation_error_deg` is the Ceres candidate pose, whether or not it passed the internal gate.
- `operational_rotation_error_deg` is populated only for a shadow candidate accepted by the copied gate. A FAIL row has no accepted shadow RT.
- For build50, the immutable R5-D FAIL remains authoritative; the shadow record is evaluation-only and cannot promote it to PASS.
- Existing R5-D finalist candidates remain the authoritative production finalist union. The shadow proposal is an additional diagnostic member, never a replacement.

## Runtime

| build | baseline wall ms | directional batch ms | shadow Ceres ms | shadow total ms | overhead |
| --- | ---: | ---: | ---: | ---: | ---: |
| build45 | 52386.519411 | 5286.755590 | 5789.626763 | 11076.382353 | 0.211436 |
| build46 | 56067.621159 | 4913.434178 | 5764.571278 | 10678.005456 | 0.190449 |
| build48 | 50788.378815 | 4467.123307 | 5773.685311 | 10240.808618 | 0.201637 |
| build49 | 48465.143801 | 5311.406043 | 5385.830688 | 10697.236731 | 0.220720 |
| build50 | 40496.401012 | 3598.544795 | 4427.906732 | 8026.451527 | 0.198202 |

## Validation

- Baseline status/reason/RT unchanged by design: `True`
- build50 baseline FAIL retained: `True`
- Shadow non-convergence excluded from operational output: `True`
- New 90/180-degree wrong branch: `['build45', 'build48']`
- Diagnostic improvements >=2 degrees: `[]`
- Maximum runtime overhead: `0.220720`
- Projection-evaluation count: the existing Ceres result does not serialize a projection counter, so it is recorded as `null` with an explicit `NOT_SERIALIZED_BY_EXISTING_CERES_PATH` status.
- Finite/proper/camera-center checks: `True`
- Targeted Core/Analyzer tests: `True`

## Decision

- No product PASS, Ground Truth, product RT, or Phase 1 GO is declared.
- If the shadow gate is not accepted, the next action is Sol review of proposal generation/selection or Ceres recovery; no weight, threshold, fallback, or iteration change is made here.

## Evidence

- output root: `/workspace/automatic_calibration/generated/v3_r6_r2_directional_proposal_shadow_20260831_final_r1`
- directional_proposal_shadow_summary.csv
- candidate_lineage.csv
- optimizer_termination.csv
- runtime_comparison.csv
- validation_checks.json
- directional_scores/<build>/directional_proposal_evaluation.json
- shadow/<build>/shadow_calibration_result.json and matching/3D preview artifacts

# E2E Test Infra: Automatic Calibration System

## Test Philosophy
- Multi-tier validation: Synthetic unit tests -> Component tests -> Real dataset multi-scene staged calibration -> Cross-epoch Hold-out validation.
- Requirement-driven: derive directly from ORIGINAL_REQUEST.md (§R1-§R4 and Acceptance Criteria).

## Feature Inventory Coverage Matrix
| # | Feature | Requirement | Tier 1 (Unit) | Tier 2 (Boundary) | Tier 3 (Integration) | Tier 4 (Real World) |
|---|---------|-------------|:-------------:|:-----------------:|:--------------------:|:-------------------:|
| F1 | Ground/Ceiling Normal & Height | R1 | ✓ | ✓ | ✓ | ✓ (20260818, 20260819) |
| F2 | Asymmetric Feature Weighting | R1 | ✓ | ✓ | ✓ | ✓ (20260818, 20260819) |
| F3 | Normal-Gated Line Matching | R2 | ✓ | ✓ | ✓ | ✓ (20260818, 20260819) |
| F4 | Coverage-Weighted Metric (TESL) | R2 | ✓ | ✓ | ✓ | ✓ (20260818, 20260819) |
| F5 | Staged Multi-Basin Filtering | R3 | ✓ | ✓ | ✓ | ✓ (20260818, 20260819) |
| F6 | Ceres 6-DoF Smooth Refinement | R3 | ✓ | ✓ | ✓ | ✓ (20260818, 20260819) |
| F7 | Product Policy & Hold-out Gate | R4 | ✓ | ✓ | ✓ | ✓ (20260818, 20260819) |

## Test Execution Commands
1. **CTest Suite (Docker)**:
   docker exec auto-calib-dev bash -c ninja -C /workspace-build && ctest --test-dir /workspace-build --output-on-failure
2. **20260818 Dataset Real Calibration**:
   docker exec auto-calib-dev /workspace-build/bin/run_real_calibration --input-dir /workspace/data/real_calibration/session-const-env/repeat_test_sample/20260818 --output /workspace/automatic_calibration/generated/test_20260818_staged --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json --image-distortion-state raw --camera-channel 1 --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 --search-strategy staged --holdout-count 1
3. **20260819 Dataset Real Calibration (Cross-epoch / Symmetry Ambiguity Check)**:
   docker exec auto-calib-dev /workspace-build/bin/run_real_calibration --input-dir /workspace/data/real_calibration/session-const-env/repeat_test_sample/20260819 --output /workspace/automatic_calibration/generated/test_20260819_staged --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json --image-distortion-state raw --camera-channel 1 --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 --search-strategy staged --holdout-count 1

## Acceptance Criteria
- CTest 100% PASS.
- 20260818 and 20260819 select true Yaw basin (~169°~170°) consistently.
- Ceres 6-DoF solver returns CONVERGENCE.
- Internal gate and Candidate RT conditions satisfied.

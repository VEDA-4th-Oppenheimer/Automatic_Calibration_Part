# Project: Automatic Calibration System Enhancement (Yaw Ambiguity Removal & Staged 6-DoF Optimization)

## Architecture
- **Core Library (libautomatic_calibration_core)**:
  - include/auto_calib/calibration_core.hpp: Data structures (StructuralLineSegment3d, LidarPlane3d, CalibrationConfig, CalibrationMetrics, PoseSceneMetrics), API prototypes.
  - src/calibration_core.cpp: Plane segmentation, normal estimation, line extraction, 2D LSD matching, NID/Signal NMI, Manhattan vanishing directions, Ceres 6-DoF residuals, cost functions.
  - src/synthetic_lidar.cpp: Synthetic scene and scan generation.
- **Application CLI & Staged Runner**:
  - pps/run_real_calibration.cpp: Multi-scene staged search runner (Coarse -> Top-3 Basin -> 5° Local -> 1° Fine -> Ceres Refinement -> Multi-criteria Gate -> Candidate RT).
  - pps/render_calibration_visualization.cpp: 2D/3D reprojection and colorized mesh generation.
- **Test Infrastructure & Verification**:
  - 	ests/calibration_core_tests.cpp, 	ests/synthetic_lidar_tests.cpp: CTest test suites.
  - Docker container uto-calib-dev with Ninja/CMake.
  - Real datasets: data/real_calibration/session-const-env/repeat_test_sample/20260818 and 20260819.

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| F1 | Ground/Ceiling Normal & Height Consistency | Extract dominant floor/ceiling planes and enforce optical axis down-angle consistency and signed camera height constraint ({\text{ground}} > C_y$). | M1 | ORIGINAL_REQUEST §R1 |
| F2 | Asymmetric Structural Feature Weighting | Weight 3D line segments by distinctiveness (length, vertical corner junctions, distance from ceiling) to suppress repetitive symmetric patterns. | M1 | ORIGINAL_REQUEST §R1 |
| F3 | Normal-Gated Line Matching Cost | Incorporate 3D plane normal difference projection and 2D image gradient alignment into line-to-line matching cost. | M2 | ORIGINAL_REQUEST §R2 |
| F4 | Coverage-Weighted Robust Line Metric (TESL) | Replace naive average edge/line distance with Total Explained Structural Length (TESL) to prevent subset shrinkage and artificial minima. | M2 | ORIGINAL_REQUEST §R2 |
| F5 | Staged Multi-Basin Pre-filtering & Scoring | Enforce TESL and geometric validity in top-N basin selection, 5° local, and 1° fine search stages. | M3 | ORIGINAL_REQUEST §R3 |
| F6 | Ceres 6-DoF Residual Smoothing & Finalist Selection | Apply smooth Huber-like loss to residuals, maintain Z-buffer freeze during optimization, and select finalist via multi-criteria confidence scoring. | M3 | ORIGINAL_REQUEST §R3 |
| F7 | Docker/CTest Regression & Real Dataset Validation | 100% pass on CTest, cross-epoch validation on 20260818/20260819 datasets, and comprehensive documentation update. | M4 | ORIGINAL_REQUEST §R4 |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | R1: Geometric Constraints & Asymmetric Feature Weighting | Ground plane height/normal validation, gravity axis consistency, asymmetric 3D line weighting in calibration_core. | None | PLANNED |
| M2 | R2: 2D-3D Structure Line & Surface Cost Enhancement | Normal-Gated line matching, Coverage-Weighted Robust Line Metric (TESL), direct surface alignment in calibration_core. | M1 | PLANNED |
| M3 | R3: Ceres 6-DoF & Staged Pipeline Tuning | Multi-basin filtering, smooth loss functions, multi-criteria finalist scoring in un_real_calibration & calibration_core. | M2 | PLANNED |
| M4 | R4: E2E Regression, Real Data Testing & Documentation | Docker/CTest 100% PASS, 20260818 & 20260819 calibration validation, result analysis, docs/ updates. | M3 | PLANNED |

## Interface Contracts
### calibration_core.hpp <-> un_real_calibration.cpp
- PoseSceneMetrics: extended with ground_normal_valid, ground_height_m, ground_tilt_deg, 	otal_explained_structural_length, symmetric_structural_weight.
- CalibrationConfig: configuration fields for ground plane tolerance, asymmetric feature weighting factor, TESL threshold, and normal-gated line matching.
- evaluateStructuralLines: returns matched line metrics including TESL and robust coverage ratio.
- compositeObjective: evaluates updated weighted sum with robust line coverage and geometric penalties.

## Code Layout
- include/auto_calib/calibration_core.hpp: Public interfaces and data structures.
- src/calibration_core.cpp: Core algorithmic implementations (Planes, Lines, Normals, Costs, Ceres).
- pps/run_real_calibration.cpp: CLI and staged pipeline coordinator.
- 	ests/calibration_core_tests.cpp: Unit tests and synthetic scenario regression tests.
- docs/: Product calibration policy, implementation changelogs, architecture documentation.

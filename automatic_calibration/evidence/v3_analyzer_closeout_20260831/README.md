# V3 Analyzer Experiment Archive

- Archive date: 2026-08-31
- Base: `develop` (`f684cd6`)
- Source snapshot: `exp-v3-r7-multi-capture-consensus-audit` (`2ab3c97`)
- Scope: experiment plans, analysis reports, execution logs, and six key PNGs only

## Final status

```text
GLOBAL_TARGETLESS_INITIALIZATION=NOT_PRODUCT_READY
ANALYZER_EXPERIMENT=FROZEN
MANUAL_REFERENCE=PROVISIONAL_ENGINEERING_NOMINAL
RECOMMENDED_PRODUCT_PATH=REFERENCE_ANCHORED_LOCAL_VERIFICATION
PRODUCT_RT_PROMOTED=false
```

The archive intentionally contains no experimental C++/Python implementation,
no CMake change, and no product-path modification. The authoritative experiment
conclusion is `docs/V3_ANALYZER_EXPERIMENT_CLOSEOUT_20260831.md`.

## Evidence map

| Directory | Contents |
|---|---|
| `manual_reference/` | Fixed Manual Reference offline evaluation |
| `r1/` | Proposal full-pose and camera-center contract |
| `r2/`, `r2_r1/` | Candidate funnel, lineage, and pose provenance correction |
| `r3/` | Objective component discriminability audit |
| `r3_r1/`, `r3_r1_r1/` | Exact common/final sample-support audits |
| `r4/` | Geometry NID ablation |
| `r5/` | Structural/Manhattan factorial ablation |
| `r6/`, `r6_r1/`, `r6_r2/` | Directional-edge experiments and shadow integration |
| `r7/` | Same-installation multi-capture consensus audit |
| `result_logs/` | Representative B0/V3/NID-off/directional-edge execution logs |
| `key_images/` | Six representative projection, failure, and analyzer images |

## Key images

| File | Meaning |
|---|---|
| `01_manual_reference_build45_projection.png` | Fixed Manual Reference projection |
| `02_v3_build45_matching.png` | Representative V3 matching result |
| `03_v3_build50_wrong_branch.png` | V3 wrong-basin/fail-closed case |
| `04_nid_off_build45_matching.png` | Geometry NID-off ablation result |
| `05_directional_edge_build45_regression.png` | Directional-edge E2E regression |
| `06_analyzer_build45_lidar_panorama.png` | Analyzer LiDAR range panorama |

## Excluded data

- raw camera images and LiDAR JSON/PCD
- CSV and JSON result dumps
- generated visualization except the six key PNGs listed above
- PLY/OBJ/MTL visualization
- repeated runner work directories and full debug dumps
- large per-point support membership files when the summarized result and
  validation output are sufficient to reproduce the recorded decision

Those files remain local and are not required to understand the accepted or
rejected experiment decisions. No file in this archive is a product RT or
ground-truth declaration.

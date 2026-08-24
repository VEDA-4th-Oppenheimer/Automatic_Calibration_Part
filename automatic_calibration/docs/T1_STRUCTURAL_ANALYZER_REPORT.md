# T1 Structural Orientation Analyzer

This branch contains an analyzer-only, deterministic orientation proposal experiment. It uses
OpenCV LSD line segments and finite-difference normals from an organized `Point` grid. Up to three
separated yaw proposals are emitted with a ±10° bounded-search radius. Empty or degenerate evidence
returns `INSUFFICIENT_FEATURES` and `fallback_required=true`; no fallback search is invoked here.

`activation_allowed` is always `false`, and no calibration objective, gate, coordinate convention,
or production executable path is changed.

## Build and test

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build -R structural_orientation_analyzer_tests --output-on-failure
```

With the project Docker image and read-only production data mount, a reproducible build22
analyzer-only run is:

```sh
WT=/workspace
IMG=$WT/data/jenkins-capture/scene0/calib_dataset_build22_20260823_231014
/workspace-build/bin/run_structural_orientation_analyzer \
  "$IMG/20260823_230009_CH1.jpg" \
  "$IMG/calib-20260824-080033_sweep-000001_pan_tilt_lidar.json" \
  "$WT/automatic_calibration/generated/analyzer_eval/t1_build22"
```

The runner validates organized row/column bounds and duplicate cells, applies the JSON frame
formula exactly (including the documented range offset), and writes `analyzer_result.json` and
`orientation_proposals.csv`. Exit code is 0 for proposals, 3 for analyzer fallback, and 2 for
invalid input. This is analyzer-only; it does not call B0 search or alter production behavior.

The synthetic test checks fail-safe image shortage, organized-grid processing, deterministic Top-K
size, and the activation safety flag. Docker execution is required for the project's real-data
commands; this minimal analyzer has no JSON/package loader or production integration in this phase.

## Limitations

Vanishing evidence is represented by LSD direction clusters (not a full projective vanishing-point
solver), normals are local finite differences, and down/roll remain zero seeds. The source exposes
artifact writing for `analyzer_result.json` and `orientation_proposals.csv`; full B0 bounded-search
and fallback wiring remains intentionally outside this analyzer-only commit.

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

The synthetic test checks fail-safe image shortage, organized-grid processing, deterministic Top-K
size, and the activation safety flag. Docker execution is required for the project's real-data
commands; this minimal analyzer has no JSON/package loader or production integration in this phase.

## Limitations

Vanishing evidence is represented by LSD direction clusters (not a full projective vanishing-point
solver), normals are local finite differences, and down/roll remain zero seeds. The source exposes
artifact writing for `analyzer_result.json` and `orientation_proposals.csv`; full B0 bounded-search
and fallback wiring remains intentionally outside this analyzer-only commit.

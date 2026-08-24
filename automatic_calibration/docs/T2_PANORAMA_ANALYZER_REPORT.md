# T2 Panorama Analyzer

Implemented as an opt-in research library; production calibration and activation remain unchanged (`activation_allowed=false`). It validates organized JSON row/column mapping, preserves the native pan/tilt contract, builds range/valid/edge channels, uses circular azimuth signatures, and emits up to three deterministic yaw proposals. Invalid shape, duplicate cells, missing cells, low coverage, or absent structural proposals set `fallback_required=true` with a reason for the existing B0 full search.

Build and test:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2 --target panorama_orientation_analyzer_tests
ctest --test-dir build -R panorama_orientation_analyzer_tests --output-on-failure
```

Standalone CH1 build22 run (the input bind mount is read-only and output is worktree-local):

```sh
docker run --rm --cpus=2 -v "$WT":/workspace -v "$WT/build":/workspace-build \
  -v "$PROD/data/jenkins-capture":/workspace/data/jenkins-capture:ro \
  -w /workspace auto-calib-dev:ubuntu-latest \
  /workspace-build/bin/run_panorama_analyzer \
  --scan /workspace/data/jenkins-capture/scene0/calib_dataset_build22_20260823_231014/calib-20260824-080033_sweep-000001_pan_tilt_lidar.json \
  --image /workspace/data/jenkins-capture/scene0/calib_dataset_build22_20260823_231014/20260823_230009_CH1.jpg \
  --output /workspace/generated/t2_build22
```

The runner writes `analyzer_result.json`, `orientation_proposals.csv`, and native panorama PNGs. Exit code 3 means analyzer fallback is required; it does not invoke or alter B0.

Limitations: this minimal experiment does not alter `run_real_calibration`, does not invoke Ceres/NID, and its normal/plane channels are conservative range-edge proxies. Perspective confirmation and B0 handoff remain integration work; signal strength and grayscale/NMI are intentionally excluded. The runner is standalone and does not claim a product calibration result.

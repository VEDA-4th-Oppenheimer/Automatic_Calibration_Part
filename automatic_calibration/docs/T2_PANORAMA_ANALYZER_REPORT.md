# T2 Panorama Analyzer

Implemented as an opt-in research library; production calibration and activation remain unchanged (`activation_allowed=false`). It validates organized JSON row/column mapping, preserves the native pan/tilt contract, builds range/valid/edge channels, uses circular azimuth signatures, and emits up to three deterministic yaw proposals. Invalid shape, duplicate cells, missing cells, low coverage, or absent structural proposals set `fallback_required=true` with a reason for the existing B0 full search.

Build and test:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2 --target panorama_orientation_analyzer_tests
ctest --test-dir build -R panorama_orientation_analyzer_tests --output-on-failure
```

Limitations: this minimal experiment does not alter `run_real_calibration`, does not invoke Ceres/NID, and its normal/plane channels are conservative range-edge proxies. Perspective confirmation and B0 handoff remain integration work; signal strength and grayscale/NMI are intentionally excluded.

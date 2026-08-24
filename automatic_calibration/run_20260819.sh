#!/bin/bash
set -e
/workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/real_calibration/session-const-env/repeat_test_sample/20260819 \
  --output /workspace/automatic_calibration/generated/verify_challenger_20260819 \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --ldc-enabled false \
  --camera-channel 1 \
  --camera-center-x-m 0.05928 \
  --camera-center-y-m -0.08105 \
  --camera-center-z-m 0.0 \
  --search-strategy staged \
  --holdout-count 1

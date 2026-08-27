#!/usr/bin/env bash
set -u

runner=${RUNNER:-/workspace/build-v3/bin/run_hybrid_orientation_analyzer}
intrinsic=${INTRINSIC:-/workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json}
output_root=${1:-/workspace/automatic_calibration/generated/v3_hybrid_eval}
mkdir -p "$output_root"
summary="$output_root/summary.csv"
printf '%s\n' 'environment,case,reference_yaw_deg,exit_code,status,fallback_required,runtime_ms,rank1_yaw_deg,recall_at_3,minimum_basin_error_deg' > "$summary"

run_case() {
  local environment=$1 case_name=$2 reference=$3 image=$4 scan=$5
  local output="$output_root/$case_name"
  mkdir -p "$output"
  "$runner" --image "$image" --scan "$scan" --intrinsic-json "$intrinsic" --output "$output" > "$output/run.log" 2>&1
  local rc=$?
  python3 - "$environment" "$case_name" "$reference" "$rc" "$output/analyzer_result.json" >> "$summary" <<'PY'
import json, math, sys
environment, case, reference, rc, path = sys.argv[1:]
reference = float(reference)
with open(path, encoding="utf-8") as f:
    result = json.load(f)
proposals = result.get("proposals", [])
def distance(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)
errors = [distance(float(p["yaw_deg"]), reference) for p in proposals]
rank1 = proposals[0]["yaw_deg"] if proposals else ""
recall = int(bool(errors) and min(errors) <= 10.0)
minimum = min(errors) if errors else math.inf
print(environment, case, reference, rc, result.get("status", ""),
      str(result.get("fallback_required", True)).lower(),
      result.get("runtime_ms", 0.0), rank1, recall, minimum, sep=",")
PY
}

repeat_root=/workspace/data/real_calibration/session-const-env/repeat_test_sample/20260818
run_case A repeat_20260818_0 169 \
  "$repeat_root/20260818-143751-CH1.jpg" \
  "$repeat_root/calib-20260818-143748_sweep-000001_pan_tilt_lidar.json"
run_case A repeat_20260818_1 169 \
  "$repeat_root/20260818-145847-CH1.jpg" \
  "$repeat_root/calib-20260818-145912_sweep-000001_pan_tilt_lidar.json"
run_case A repeat_20260818_2 169 \
  "$repeat_root/20260818-151305-CH1.jpg" \
  "$repeat_root/calib-20260818-151312_sweep-000001_pan_tilt_lidar.json"

jenkins_root=/workspace/data/jenkins-capture/scene0
run_case B jenkins_build22 177 \
  "$jenkins_root/calib_dataset_build22_20260823_231014/20260823_230009_CH1.jpg" \
  "$jenkins_root/calib_dataset_build22_20260823_231014/calib-20260824-080033_sweep-000001_pan_tilt_lidar.json"
run_case B jenkins_build23 177 \
  "$jenkins_root/calib_dataset_build23_20260823_232209/20260823_231209_CH1.jpg" \
  "$jenkins_root/calib_dataset_build23_20260823_232209/calib-20260824-081228_sweep-000001_pan_tilt_lidar.json"
run_case B jenkins_build24 177 \
  "$jenkins_root/calib_dataset_build24_20260823_233514/20260823_232509_CH1.jpg" \
  "$jenkins_root/calib_dataset_build24_20260823_233514/calib-20260824-082527_sweep-000001_pan_tilt_lidar.json"

column -s, -t "$summary" 2>/dev/null || cat "$summary"

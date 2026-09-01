# Reference-Anchored Local Verification MVP

작성일: 2026-08-31  
상태: 실험/제품 후보 경로 검증용, 제품 RT 승격 아님

## 목적

전역 targetless analyzer가 안정적인 full-pose basin을 생성하지 못해 동결된
상태에서, 수동으로 확보한 `Manual Reference RT`를 engineering nominal으로
사용하고 그 주변만 자동으로 확인한다. 이 경로는 360° cold-start 자동
캘리브레이션이 아니다.

Manual Reference는 `reference_ground_truth=false`, `product_rt=false`인
provisional 값이다. 따라서 이 경로의 PASS/후보 결과는 제품 승인이나 절대
정확도 증거가 아니다.

## 입력 계약

- 카메라: 기존 clean18 K/D JSON을 고정 사용한다.
- 입력 영상: `--image-distortion-state raw`이면 runner가 기존 K/D로 한 번
  undistort한다.
- LiDAR 좌표: 기존 JSON의 `frame`/`range_formula` 계약을 그대로 사용한다.
- `sensor.range_offset_m`은 JSON measurement를 XYZ로 만드는 시점에 정확히
  한 번 적용한다.
- pose 계약: `p_camera = R * p_lidar + t`.
- 카메라 중심: `C_lidar=(0,-0.08105,0)m`.
- 모든 후보의 이동은 자유롭게 최적화하지 않고 `t=-R*C_lidar`로 계산한다.
- 회전은 기존 runner의 parent-frame convention을 재사용한다.

## Opt-in CLI

기존 `staged`/`legacy` 동작은 그대로 유지된다. 아래 전략을 명시할 때만
새 경로가 실행된다.

```text
--search-strategy reference-anchored-local
--reference-rt <manual_reference_rt.json>
--manual-intrinsic-json <camera_intrinsic.json>
--image-distortion-state raw
--ldc-enabled unknown
--camera-center-x-m 0 --camera-center-y-m -0.08105 --camera-center-z-m 0
--local-yaw-half-range-deg 5
--local-down-half-range-deg 5
--local-roll-half-range-deg 5
--local-step-deg 1
```

기본 범위에서는 11×11×11=1,331개의 pose를 생성한다. build45, build46,
build48, build49는 각각 독립적으로 평가하고 build50은 local search나
consensus에 포함하지 않는 hold-out이다. 전체 입력 폴더는 실제 디렉터리명과
manifest를 확인해 build 순서를 `45,46,48,49,50`으로 고정한다.

## 실행 순서

1. Reference RT, K/D, camera-center, pairing, range-offset metadata를
   검증한다.
2. 각 training build에서 1,331개 pose를 기존
   `evaluateCalibrationPoseScenes()`/`summarizeCalibrationPoseScenes()`로
   평가한다.
3. 후보별로 같은 pose 집합에서 projectable한 LiDAR point-ID 교집합을
   만들어 고정 support로 사용한다. ID는 organized array index이며 새
   UUID나 좌표 반올림 ID를 만들지 않는다.
4. A는 기존 production objective를 사용한다. B는 동일 scene metrics를
   기존 summarizer에 넣되 NID weight만 0으로 한 shadow 진단이다. B는
   선택·Ceres·출력 RT에 영향을 주지 않는다.
5. A 순위 상위 3개만 기존 Ceres refinement에 전달한다. Ceres 결과의
   proper rotation, convergence, camera-center 1 mm gate, local boundary
   hit를 검사한다.
6. 조건을 통과한 training 결과만 rotation consensus 후보가 된다. 가장
   큰 cluster가 회전 2° 이하, translation 5 mm 이하로 최소 3개이면
   quaternion 평균을 구하고 다시 `t=-R*C`를 적용한다.
7. consensus가 있을 때만 build50에 고정 적용한다. build50은 RT를
   재최적화하지 않으며 product candidate로 반환하지 않는다.
8. local search 실패 시 global analyzer fallback을 자동 실행하지 않고
   nominal Manual Reference를 last-known-good visualization pose로 둔다.

네 training build는 계산 시간을 줄이기 위해 독립 작업으로 병렬 처리할 수
있지만, 각 build의 score와 Ceres 호출은 서로 공유하지 않는다.

## 판정

- `VERIFIED_NO_UPDATE`: nominal 유지가 충분하고 local 보정이 필요하지 않음.
- `LOCAL_UPDATE_CANDIDATE`: training build 최소 3개가 같은 local basin을
  지지하고 engineering gate를 통과함.
- `LOCAL_UPDATE_REJECTED`: basin 반복성, boundary, Ceres 또는 support가
  모호함.
- `CALIBRATION_UNAVAILABLE`: 입력·K/D·좌표계·finite·camera-center 계약이
  성립하지 않음.

이 상태들은 제품 PASS와 다르다. `product_rt_promoted=false`를 항상
기록한다. build50의 고정 진단이 좋아 보여도 hold-out 경로는
`FAIL_CLOSED_DIAGNOSTIC_ONLY`로 남기고 제품 활성화를 하지 않는다.

## 산출물

`generated/reference_anchored_local_verification_20260831/` 아래에 다음을
남긴다.

- `reference_rt_snapshot.json`, `input_manifest.csv`
- build45/46/48/49별 `local_candidate_scores.csv`,
  `local_search_summary.json`, `matching_scene_0.png`,
  `scene_0_colorized_lidar_3d_preview.png`, `debug/`
- `build50/` 고정 hold-out visualization과 summary
- `per_build_rt_comparison.csv`, `repeatability_summary.json`,
  `holdout_build50_summary.json`, `runtime_comparison.csv`,
  `validation_checks.json`
- `REFERENCE_ANCHORED_LOCAL_VERIFICATION_REPORT_20260831.md`

Generated/raw 데이터는 Git에 커밋하지 않는다. 커밋 대상은 runner의 opt-in
코드와 이 문서뿐이며, 제품 기본 경로의 weight, threshold, analyzer,
optimizer, K/D, fallback 정책은 이 MVP로 변경하지 않는다.

## 제한과 다음 검토

이 경로는 Manual Reference에 anchor되어 있으므로 전역 targetless 초기화의
대체 증명이 아니다. 독립 survey/CAD 기준, 더 다양한 정적 장면, 제품 장치
내 입력 파이프라인 검증이 추가되기 전에는 `LOCAL_UPDATE_CANDIDATE`도
engineering diagnostic 수준으로만 취급한다.

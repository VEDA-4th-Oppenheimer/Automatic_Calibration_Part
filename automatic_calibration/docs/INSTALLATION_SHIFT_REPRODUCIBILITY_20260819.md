# 2026-08-19 설치 위치 변경 재현성 검증

작성일: 2026-08-19  
시험 범위: CH1 개별 시험, CH4 개별 시험  
상태: **CH1 후보 검증 진행 중 / CH4 최신 2-train·1-hold-out 내부 통과 / 제품 승격 보류**

## 1. 목적

기존 고정환경과 다른 설치 위치에서 자동 외부 파라미터 탐색이 실행되는지 확인하고,
설치 변경 후 선택되는 yaw/down/optical-roll 후보가 안정적으로 반복되는지 확인한다.
기존 설치의 RT 또는 camera-center prior는 강제로 사용하지 않았다.

2026-08-20 운영자 확인에 따라 설치 변경의 의미를 다음과 같이 정정한다. 카메라만
LiDAR에서 분리해 이동한 것이 아니라, 카메라와 LiDAR가 부착된 강체 모듈 전체를
모니터암으로 이동·회전했다. 따라서 방/세계 좌표에서 모듈 pose는 바뀌지만
`T_camera_lidar`와 LiDAR frame의 camera-center는 바뀌면 안 된다. 본 문서의
`--baseline-m 0` 결과는 prior 제거 진단으로만 유지하며, 최종 cross-epoch 판정은
[`INSTALLATION_EPOCH_REPRODUCIBILITY_20260818_20260819.md`](INSTALLATION_EPOCH_REPRODUCIBILITY_20260818_20260819.md)를
따른다.

## 2. 입력 확인

입력 경로:

```text
data/real_calibration/session-const-env/repeat_test_sample/20260819/
```

확인된 파일 수:

| 구분 | 수량 |
|---|---:|
| CH1 image | 5 |
| CH4 image | 6 |
| LiDAR JSON | 3 |

실행기는 image와 JSON이 같은 개수의 lexicographic 1:1 pair여야 하므로, 이번 재실행은
각 채널의 확인된 3개 image와 3개 JSON을 pair로 구성했다. 마지막 JSON은 CH1/CH4
이미지에 공통으로 대응하는 LiDAR scan으로 사용했다.

### CH1 pair

```text
20260819-184325-CH1.jpg ↔ calib-20260819-184249_sweep-000001_pan_tilt_lidar.json
20260819-184342-CH1.jpg ↔ calib-20260819-185256_sweep-000001_pan_tilt_lidar.json
20260819-200910-CH1.jpg ↔ calib-20260819-200851_sweep-000001_pan_tilt_lidar.json
```

### CH4 pair

```text
20260819-184401-CH4.jpg ↔ calib-20260819-184249_sweep-000001_pan_tilt_lidar.json
20260819-184405-CH4.jpg ↔ calib-20260819-185256_sweep-000001_pan_tilt_lidar.json
20260819-200913-CH4.jpg ↔ calib-20260819-200851_sweep-000001_pan_tilt_lidar.json
```

각 채널의 나머지 image는 JSON scan이 부족해 자동 실행에 포함하지 않았다. 3쌍 실행은
최소 training 관측 수를 충족하지만 독립 hold-out은 아직 없다.

## 3. 공통 실행 조건

- LDC: `false`
- image 상태: `raw`
- Manual intrinsic 고정, intrinsic refinement 비활성
- 기존 Manual RT/reference JSON 미사용
- 기존 설치 camera-center `(0.05928,-0.08105,0)` 미사용
- direction prior weight: `0`
- LiDAR JSON 좌표계와 pan 방향 계약 사용
- 결과 RT는 diagnostic candidate이며 활성 RT로 승격하지 않음

주의: camera-center를 생략한 초기 v1/v15 실행에는 실행기의 기본
`--baseline-m 0.28`이 적용됐다. 설치 변경 후 28 cm baseline을 보장할 수 없으므로,
이 두 결과는 baseline 민감도 확인용으로만 해석한다. 이후 중립 재실행에서는
`--baseline-m 0`으로 해당 prior를 제거했다.

사용한 intrinsic:

```text
CH1: manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json
CH4: manual_calibration/output/session-const-env/intrinsic-ch4-20260819-full/camera_intrinsic.json
```

## 4. CH1 결과

### 4.1 5° 전체 탐색

산출물:

```text
automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v1/
```

| 항목 | 결과 |
|---|---:|
| orientation candidates | 19,152 |
| raw best yaw/down | `-115° / 20°` |
| refined down | `20.236°` |
| selected yaw | `-115°` |
| selected optical roll | `+10°` |
| final composite objective | `0.5902` |
| mean edge distance | `8.04 px` |
| geometry NID | `0.9172` |
| training scene gate | `2/2 PASS` |
| overall status | `FAIL: SINGLE_OBSERVATION_DIAGNOSTIC_ONLY` |

### 4.2 15° yaw/down + roll ±15° 비교

산출물:

```text
automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v15_roll/
```

| 항목 | 결과 |
|---|---:|
| orientation candidates | 2,352 |
| raw best yaw/down | `-105° / 30°` |
| refined down | `15.348°` |
| selected yaw | `-120°` |
| selected optical roll | `+15°` (탐색 경계) |
| final composite objective | `0.6017` |
| mean edge distance | `7.12 px` |
| geometry NID | `0.9140` |
| training scene gate | `2/2 PASS` |
| overall status | `FAIL: SINGLE_OBSERVATION_DIAGNOSTIC_ONLY` |

두 탐색 간 yaw는 `-115~-120°`로 비슷하지만, down은 약 `15.3~20.2°`, optical roll은
`+10~+15°`로 변한다. 15° 실행에서 roll이 경계값에 붙었으므로 실제 roll을 확정한
것으로 해석하지 않는다.

### 4.3 baseline prior 제거 재실행

산출물:

```text
automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v15_roll_nobaseline/
```

`--baseline-m 0`과 동일한 yaw/down 및 roll 탐색을 적용했다.

| 항목 | 결과 |
|---|---:|
| raw best yaw/down | `-120° / 15°` |
| refined down | `14.771°` |
| selected yaw | `-120°` |
| selected optical roll | `+15°` (탐색 경계) |
| estimated translation | `(-9.3, 12.8, 2.8) mm` |
| final composite objective | `0.6265` |
| mean edge distance | `7.23 px` |
| geometry NID | `0.9278` |
| training scene gate | `2/2 PASS` |
| overall status | `FAIL: SINGLE_OBSERVATION_DIAGNOSTIC_ONLY` |

baseline을 제거해도 yaw는 `-120°`, down은 약 `15°`로 유지되지만 optical roll은
`+15°` 경계에 머문다. translation은 기존 `0.28 m` prior에 고정되지 않고 원점 근처로
이동했으나, 실제 새 설치 중심 offset을 측정한 값이 아니므로 물리적 정답으로 보지 않는다.

### 4.4 3쌍 독립 재실행

산출물:

```text
automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v3/
```

새 CH1 image와 공통 LiDAR JSON을 추가해 총 3쌍으로 재실행했다.

| 항목 | 결과 |
|---|---:|
| raw best yaw/down | `-120° / 15°` |
| refined down | `16.285°` |
| selected yaw | `-120°` |
| selected optical roll | `+10°` |
| final composite objective | `0.6409` |
| mean edge distance | `10.55 px` |
| geometry NID | `0.9345` |
| training scene gate | `3/3 PASS` |
| overall status | `PASS` |

이는 CH1의 3개 training 관측에 대한 내부 gate PASS다. 독립 hold-out을 사용한
재현성 승인과 제품 RT 승격은 별도 단계로 남아 있다.

### CH1 투영 확인

3쌍 최종 실행의 주요 확인 파일:

- `automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v3/matching_scene_0.png`
- `automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v3/scene_0_colorized_lidar_3d_preview.png`
- `automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v3/scene_0_colorized_lidar_z_up_viewer_mesh.ply`
- `automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v3/scene_0_colorized_lidar_z_up_viewer_mesh.obj`
- `automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v3/training_scene_validation.csv`

`matching_scene_*.png`는 최종 CH1 후보를 각 장면에 투영한 결과다. 3개 training scene의
품질 gate는 통과했지만, 독립 hold-out/ground truth가 없어 설치 변경 후 재현성을 최종
승인한 것으로 해석하지 않는다.

## 5. CH4 개별 결과

산출물:

```text
automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch4_v3/
```

CH1과 동일하게 15° yaw/down + roll ±15°를 사용하고, 설치 변경 후 기존 28 cm
baseline prior를 제거한 독립 실행 결과다. CH4 결과는 CH1의 보조 점수나 방향 prior로
사용하지 않았다.

| 항목 | 결과 |
|---|---:|
| raw best yaw/down | `-90° / 60°` |
| refined down | `0.149°` |
| selected yaw | `+150°` |
| selected optical roll | `+15°` (탐색 경계) |
| estimated translation | `(-0.2, -0.9, 0.6) mm` |
| final composite objective | `0.7255` |
| mean edge distance | `27.91 px` |
| geometry NID | `0.8396` |
| training scene gate | `1/3 PASS` |
| overall status | `FAIL: PER_SCENE_VALIDATION_FAILED` |

CH4는 CH1과 독립적으로 판정한다. 세 번째 pair를 추가한 뒤 CH4 scene 0/1에서
`NID_OVERLAP_INSUFFICIENT`가 발생했고, scene 1에서는
`MANHATTAN_VERTICAL_ALIGNMENT_POOR`도 발생했다. CH4 결과는 CH1 결과를 보정하거나
보조 점수로 합치지 않는다.

초기 `repeat_test_sample_20260819_install_shift_ch4_v1/` 실행은 생략한 baseline 옵션의
기본값 `0.28 m`가 적용된 비교용 결과이므로 최종 CH4 판정에서 제외했다.

## 6. 판정과 남은 조건

기존 CH4 `repeat_test_sample_20260819_install_shift_ch4_v3`는 3개 scene 중 2개가
per-scene 품질 게이트를 통과하지 못했다. 이후 CH4 전용 PASS K·D, 고정 camera-center,
최신 staged/coverage 정책과 2-train·1-hold-out 분할로 재실행한 결과는
`CANDIDATE_RT`, training `2/2`, hold-out `1/1`이다. 따라서 기존 FAIL은 최신 실행의
현재 판정이 아니라 비교 이력으로 유지한다.

최신 CH4 실행은 내부 hold-out까지 통과했지만 `PRODUCT_APPROVED_RT`는 아니다. ArUco
검출이 제공하는 `T_camera_marker_board`만으로는 `T_lidar_marker_board`가 없어 절대
camera–LiDAR RT 오차를 계산할 수 없기 때문이다.

1. CH1과 CH4 각각 최소 2개 training + 1개 독립 hold-out
2. 각 pair의 실제 대응 관계를 기록한 manifest
3. CH4에서 구조선/NID가 충분한 추가 독립 pair 1~2개
4. LiDAR에서 식별 가능한 기준 보드와 `T_lidar_marker_board` 또는 독립 manual RT
5. 설치 변경 후 camera-center 또는 기계 offset 측정값

현재 CH1 후보(`yaw 약 -120°, down 약 16°, roll +10°`)는 내부 gate를 통과했지만
제품 RT로 확정하지 않는다. CH4의 기존 v3 후보(`yaw 약 +150°, down 약 0°,
roll +15°`)는 per-scene FAIL인 비교 이력으로 유지한다. 최신 CH4 후보는
`yaw seed 약 97°, down 11°, roll 9°`이며 2-train·1-hold-out을 통과했지만,
`T_lidar_marker_board` 또는 독립 manual RT가 없어 제품 승격하지 않는다. 모듈의 강체
장착은 유지됐으므로 8월 18일에 사용한 camera-center `(0.05928,-0.08105,0) m`를
동일하게 적용해 CH1을 재실행하고, 독립 기준과의 cross-epoch 검증을 계속한다.

## 7. 수정 로그

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-19 | 설치 위치 변경 데이터 구성 확인, CH1·CH4 개별 탐색 실행 |
| 2026-08-19 | image/JSON 개수 불일치와 2쌍 진단 한계를 기록하고 최종 RT 승격 보류 |
| 2026-08-19 | 기본 0.28 m baseline 영향 확인 후 `--baseline-m 0` 중립 재실행 추가 |
| 2026-08-19 | CH4를 CH1 보조 진단에서 분리하고 baseline 없는 독립 결과로 갱신 |
| 2026-08-19 | 공통 LiDAR JSON과 CH1/CH4 이미지 각 1개를 추가해 3쌍 독립 재실행 |
| 2026-08-19 | CH1 3/3 내부 PASS, CH4 1/3 per-scene FAIL 결과 반영 |
| 2026-08-20 | 카메라·LiDAR 강체 모듈 전체 이동 조건을 반영하고 동일 RT 재현성 시험으로 해석 정정 |
| 2026-08-20 | 8월 18일 RT 고정 적용 `3/3 PASS`, 동일 조건 19일 재추정 후보의 역방향 `0/4 FAIL` 결과를 비교 문서에 반영 |
| 2026-08-20 | CH4 전용 PASS K·D로 전체 장면 ArUco 검출 및 pose 일관성 확인 |
| 2026-08-20 | CH4 최신 staged 실행: training `2/2`, hold-out `1/1`, `CANDIDATE_RT`, activation 금지 |

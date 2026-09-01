# CH1 A4 ChArUco 부착 검증 기록

작성일: 2026-08-18  
범위: CH1 단독 검증  
상태: 보드 인식 PASS, 고정환경 hold-out PASS, 운영 RT 확정 전

고정환경 수집 조건과 파일 분할에 대한 운영자 확인은
[`CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md`](CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md)에
별도로 기록했다.

## 1. 목적과 범위

0815에 사용하던 고정 환경에 A4 ChArUco 보드를 추가로 부착한 뒤 CH1 영상에서
보드가 검출되는지 확인하고, 같은 고정 환경의 LiDAR scan을 사용해 자동 캘리브레이션
진단까지 수행했다. 사용자의 요청에 따라 CH2~CH4는 이번 기록과 판정에서 제외한다.

이번 시험은 보드 인식 검증과 자동 파이프라인 연결 확인을 위한 것이다. 최종 RT를
승인하는 conformance 시험으로 해석하지 않는다.

## 2. 입력 데이터

### CH1 이미지

```text
data/real_calibration/session-const-env/auto_data/image_storage/
  20260818-131423-CH1.jpg
```

- 해상도: `2592 x 1520`
- 0815와 같은 설치·환경에서 A4 보드만 추가한 영상
- 카메라 프로파일: `PNM-C16083RVQ`

### LiDAR scan

이번 단일 보드 인식 probe에는 동일한 timestamp의 scan 파일이 없어서, 고정 환경의
0815 연속 scan 중 하나를 진단용으로 사용했다.

```text
data/real_calibration/session-const-env/repeat_test_sample/20260815/
  calib-20260814-232414_sweep-000001_pan_tilt_lidar.json
```

따라서 이 단일 probe의 image-scan 연결은 운영자가 정한 "고정 환경 진단용" pairing이며,
최종 정확도 검증에 사용하지 않는다. 이후 20260818 네 세트는 별도 확인된 고정환경
epoch의 반복 데이터로 분리해 기록했다.

## 3. A4 ChArUco 보드 인식 결과

실제 출력물에서 측정한 물리 치수를 사용했다.

설정 파일:

```text
output/pdf/charuco_a4_board_config.json
```

| 항목 | 실측값 |
|---|---:|
| dictionary | `DICT_5X5_100` |
| 배열 | `7 x 5` |
| 한 칸 | `27 mm` |
| 마커 | `20 mm` |
| 유효 보드 크기 | `189 x 135 mm` |

CH1 검출 결과:

| 항목 | 결과 |
|---|---:|
| ArUco marker | `16 / 17` |
| ChArUco corner | `22` |
| reprojection RMSE | `1.2826 px` |
| 최대 reprojection 오차 | `2.6274 px` |
| reason code | `OK` |
| status | `PASS` |

산출물:

```text
manual_calibration/output/session-const-env/validation/
  20260818-131423-CH1/marker_pose_overlay.png
  20260818-131423-CH1/marker_pose_result.json
```

판정: CH1 영상에서 A4 보드는 인식된다. 다만 17개 marker와 24개 corner를 모두
확보하지 못했으므로, 이후 반복 촬영에서는 보드를 더 크게·정면에 가깝게 배치한다.

## 4. CH1 자동 캘리브레이션 진단

적용한 주요 조건:

- manual intrinsic 고정
- `--ldc-enabled false`
- `--image-distortion-state raw`
- LiDAR frame camera-center: `(0.05928, -0.08105, 0) m`
- range offset: `0.084 m`
- yaw coarse step: `5 deg`
- down search: `0~30 deg`, `5 deg`
- optical roll search: `-15~15 deg`, `5 deg`
- direction prior weight: `0`

결과 디렉터리:

```text
automatic_calibration/generated/ch1_20260818_aruco_probe/
```

주요 결과:

| 항목 | 결과 |
|---|---:|
| 입력 pair | `1` |
| raw best yaw | `160 deg` |
| raw best down | `20 deg` |
| 선택 optical roll | `0 deg` |
| NID improvement | 약 `3.07%` |
| training scene validation | `1 / 1 PASS` |
| 전체 status | `FAIL` |
| reason code | `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY` |

`NID_IMPROVEMENT_INSUFFICIENT` gate도 함께 기록됐다. 최적화 후보는 내부 목적함수상
선택됐지만 reference RT와 hold-out이 없기 때문에 활성 RT로 채택하지 않았다.
산출물의 `matching_scene_0.png`, `06_projection_final.png`, 3D preview에는
`REJECTED CANDIDATE`가 표시된다.

## 5. 결론

1. CH1의 A4 보드 자체는 정상 인식된다.
2. 이번 단일 자동 실행의 FAIL은 보드 인식 실패가 아니라 입력 pair가 하나여서 최종
   RT 승인 조건을 충족하지 못한 결과다. 파일 시각 차이는 고정환경 probe의 제한사항이지
   보드 검출 실패 원인이 아니다.
3. 이번 실행에서 얻은 `yaw 160 deg / down 20 deg`는 진단 후보일 뿐이며 운영 RT로
   사용하지 않는다.
4. CH2~CH4는 이번 검증 범위에서 제외했다.

## 6. 다음 실행 조건

최종 CH1 자동 캘리브레이션을 진행하려면 같은 설치 상태에서 다음을 확보한다.

- CH1 이미지와 LiDAR JSON의 실제 측정 pair 최소 3개
- 각 pair의 파일명 또는 별도 manifest를 통한 명시적 pairing
- 보드 전체가 보이는 CH1 영상, 가능하면 17 marker / 24 corner
- zoom, focus, LDC 상태 고정

그 후 `--pair-count 3` 이상으로 재실행하고, 결과 RT의 반복성·hold-out 투영·품질
게이트를 함께 판정한다.

## 7. 산출물 목록

- `manual_calibration/output/session-const-env/validation/20260818-131423-CH1/marker_pose_overlay.png`
- `manual_calibration/output/session-const-env/validation/20260818-131423-CH1/marker_pose_result.json`
- `automatic_calibration/generated/ch1_20260818_aruco_probe/calibration_result.json`
- `automatic_calibration/generated/ch1_20260818_aruco_probe/matching_scene_0.png`
- `automatic_calibration/generated/ch1_20260818_aruco_probe/scene_0_colorized_lidar_3d_preview.png`

## 8. 동일 고정환경 CH1 image-scan 3쌍 재검증

같은 날 새로 측정한 다음 3쌍을 추가했다.

| scene | CH1 이미지 | LiDAR JSON | 파일 시각 차이 |
|---:|---|---|---:|
| 0 | `20260818-143751-CH1.jpg` | `calib-20260818-143748_sweep-000001_pan_tilt_lidar.json` | 3 s |
| 1 | `20260818-145847-CH1.jpg` | `calib-20260818-145912_sweep-000001_pan_tilt_lidar.json` | 25 s |
| 2 | `20260818-151305-CH1.jpg` | `calib-20260818-151312_sweep-000001_pan_tilt_lidar.json` | 7 s |

입력 디렉터리:

```text
data/real_calibration/session-const-env/repeat_test_sample/20260818/
```

### 8.1 ChArUco 검출

| scene | marker | corner | RMSE | max error | 상태 |
|---:|---:|---:|---:|---:|---|
| 0 | 17/17 | 24/24 | 1.243 px | 1.917 px | PASS |
| 1 | 17/17 | 24/24 | 1.277 px | 2.142 px | PASS |
| 2 | 17/17 | 24/24 | 1.325 px | 2.430 px | PASS |

세 영상 모두 왼쪽 벽에 부착한 A4 보드를 완전 검출했다. 검출 산출물은 다음 경로에
저장했다.

```text
manual_calibration/output/session-const-env/validation/20260818-repeat/
```

### 8.2 자동 캘리브레이션 결과

결과 디렉터리:

```text
automatic_calibration/generated/ch1_20260818_three_pair_v1/
```

| 항목 | 결과 |
|---|---:|
| 전체 status | `PASS` |
| candidate gate | `PASS` |
| training scene | `3/3 PASS` |
| coarse yaw | `170 deg` |
| coarse down | `20 deg` |
| optical roll | `0 deg` |
| refined down | `19.9989 deg` |
| objective improvement | `15.53%` |
| NID improvement | `1.08%` |
| mean edge distance | `19.92 px` |
| projected ratio | `0.7794` |
| structural matches | `71` |
| Manhattan vertical error | `5.37 deg` |

추정한 `T_camera_lidar`는 다음과 같다.

```text
R = [-0.98481974,  0.00002141,  0.17358018]
    [ 0.05938464,  0.93969911,  0.33680714]
    [-0.16310593,  0.34200232, -0.92543550]

t = [0.05837995, 0.07264168, 0.03739286] m
```

signal-strength NMI의 Manual RT perturbation 진단도 `PASS`했고, 24개 perturbation 중
`79.17%`가 reference보다 나쁜 점수를 냈다. 이 결과는 signal NMI 활성화 가능성을
지지하지만, 현재 결과에서 signal NMI weight는 여전히 `0`이다.

### 8.3 Manual RT 비교와 제한

기존 Manual RT와 비교하면 다음 차이가 난다.

| 비교 항목 | 차이 |
|---|---:|
| 회전 geodesic | `10.6906 deg` |
| translation norm | `0.13917 m` |

자동 결과는 현재 내부 gate를 통과했지만 Manual RT와 차이가 크다. Manual RT도
독립 conformance truth가 아니라 진단 reference이고, 이번 세 쌍은 사실상 같은 구조와
시야를 반복한 데이터다. 따라서 현재 PASS는 "세 입력의 내부 목적함수와 gate 통과"를
뜻하며 외부 파라미터의 절대 정확도 확정을 뜻하지 않는다.

네 번째 쌍을 추가해 고정 RT hold-out을 수행했으므로 현재 결과는
`candidate PASS / fixed-environment hold-out PASS`로 기록한다. 이 결과만으로 Manual RT와의
절대 정확도 또는 다른 설치 epoch에 대한 일반화를 확정하지 않으며, 기존 운영 RT를 자동으로
대체하지 않는다.

## 9. 독립 CH1 hold-out 고정 RT 검증

기존 3쌍으로 추정한 RT를 다시 최적화하지 않고, 추가된 네 번째 CH1 image-scan pair에
그대로 적용했다.

| 구분 | 파일 |
|---|---|
| CH1 이미지 | `20260818-155208-CH1.jpg` |
| LiDAR JSON | `calib-20260818-154229_sweep-000001_pan_tilt_lidar.json` |
| 입력 선택 | `--pair-start 3 --pair-count 1` |
| 고정 RT | `ch1_20260818_three_pair_v1/calibration_result.json` |

파일명 시각은 `9분 39초` 차이가 난다. JSON에는 wall-clock 대신 monotonic timestamp만
있으므로 파일 내부 정보만으로 동시 측정 여부를 교차검증할 수는 없다. 다만 사용자가
해당 시간 동안 카메라, LiDAR actuator 및 장면이 모두 고정돼 있었음을 확인했다
(`2026-08-18`). 따라서 이 두 파일을 현재 고정환경 RT 재현성을 평가하는 유효한 독립
hold-out pair로 확정한다. 이 확인은 정적 장면 검증에는 충분하지만 센서 하드웨어 시간
동기화를 검증했다는 의미는 아니다.

### 9.1 ChArUco 검출

| marker | corner | RMSE | max error | 상태 |
|---:|---:|---:|---:|---|
| 17/17 | 24/24 | 1.316 px | 2.133 px | PASS |

검출 산출물:

```text
manual_calibration/output/session-const-env/validation/20260818-repeat/
  20260818-155208-CH1/
```

### 9.2 RT 고정 검증 결과

`--validation-pose-json` 모드로 후보 탐색, Ceres 최적화 및 RT 갱신을 모두 비활성화했다.

| 항목 | 결과 |
|---|---:|
| 전체 status | `PASS` |
| 장면 통과 | `1/1` |
| visible / aligned edge | `241 / 187` |
| projected ratio | `0.7759` |
| mean edge distance | `20.57 px` |
| edge p50 / p90 | `12.37 / 55.00 px` |
| 30 px 초과 edge | `21.99%` |
| geometry NID | `0.9482` |
| structural matched | `21/21` |
| horizontal / vertical match | `7 / 14` |
| Manhattan vertical error | `11.21 deg` |

전체 LiDAR 투영을 직접 확인하면 벽, 왼쪽 캐비닛, 책상 평면의 배치와 방향은 영상의
대응 표면을 따른다. edge residual은 중앙값 `12.37 px`로 대체로 근접하지만 일부
캐비닛·벽 모서리에서 큰 오차가 남아 p90은 `55 px`다. 따라서 이 결과는 방향이 다른
바닥으로 투영되던 과거 실패와는 구분되지만, pixel-level 정답이라고 해석하지 않는다.

산출물:

```text
automatic_calibration/generated/ch1_20260818_holdout_155208_fixed_v1/
  fixed_pose_validation_result.json
  fixed_pose_scene_validation.csv
  debug/scene_0/06_projection_final.png
  debug/scene_0/07_projection_final_edges.png
  debug/scene_0/07a_projection_final_edge_residual.png
```

### 9.3 현재 판정

- 현재 고정 설치·조명 조건에 대한 독립 hold-out 검증은 `PASS`다.
- 기존 3쌍에서만 우연히 맞은 RT가 아니라 네 번째 pair에도 동일 RT가 유지됐다.
- 파일명 시각 차이는 고정환경 사용자 확인으로 hold-out 판정에 수용했다. Manual RT 대비
  회전 `10.69 deg`, 이동 `139.17 mm` 차이는 남아 있으므로 절대 정확도 인증과 현재
  조건의 재현성 검증은 구분한다.
- 현재 상태는 `candidate PASS / fixed-environment hold-out PASS`이며, 장치 이동 후에도
  같은 RT를 재사용한다는 의미는 아니다.

## 10. 고정환경 확인 및 문서 연결

2026-08-18 네 세트는 카메라·LiDAR·actuator와 장면을 고정한 상태에서 수집했다는
운영자 확인을 반영한다. 이 확인에 따라 scene 0~2는 RT 추정, scene 3은 고정 RT
hold-out으로 유지한다. 파일명 시각이 다르거나 JSON이 연속 sweep이라는 이유만으로
pair를 폐기하지 않지만, 내부 timestamp가 없으므로 시간 동기화 시험으로 해석하지
않는다.

자세한 입력 목록, pairing 규칙, installation epoch 규칙은
[`CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md`](CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md)를
기준 문서로 사용한다.

## 11. 최신 4세트 재실행 확인

최근 추가된 파일을 포함해 동일한 명령을 다시 실행했다. 입력 디렉터리의 네 세트를
파일명 순서로 읽고, 마지막 세트를 hold-out으로 분리했다.

```text
입력: data/real_calibration/session-const-env/repeat_test_sample/20260818/
분할: scene 0~2 training (3개) / scene 3 hold-out (1개)
출력: automatic_calibration/generated/ch1_20260818_four_pair_recheck_v2/
```

이번 재실행에서 사용한 추가 세트는 다음과 같다.

| 구분 | 파일 |
|---|---|
| CH1 이미지 | `20260818-155208-CH1.jpg` |
| LiDAR JSON | `calib-20260818-154229_sweep-000001_pan_tilt_lidar.json` |

주요 조건은 기존 검증과 동일하다.

- `LDC=false`, Manual ChArUco `K + distortion` 고정, raw 영상 undistort
- camera center `(0.05928, -0.08105, 0) m`
- `yaw step=5°`, `down=0~30° step=5°`, optical roll `-15~15° step=5°`
- `holdout-count=1`, `minimum-scene-pass-ratio=1.0`

### 11.1 재실행 결과

| 항목 | 결과 |
|---|---:|
| 전체 status | `PASS` |
| training | `3/3 PASS` |
| hold-out | `1/1 PASS` |
| 선택 down | `20.0°` (refined `19.9989°`) |
| optical roll | `0.0°` |
| hold-out visible/aligned edge | `241 / 187` |
| hold-out projected ratio | `0.775934` |
| hold-out mean edge distance | `20.574 px` |
| hold-out geometry NID | `0.948202` |
| hold-out structural match | `21/21` |
| hold-out Manhattan vertical error | `11.210°` |

hold-out CSV는 이전에 생성한 고정 RT 검증 CSV와 행 단위로 동일했다. 따라서 최근 파일
추가가 기존 RT를 바꾸거나 결과를 악화시키지 않았고, 같은 고정 installation epoch에서
RT가 반복 적용되는 것을 재현했다. 다만 이 데이터는 동일한 환경·설치 상태의 반복이므로
다른 설치 위치나 새로운 구조에 대한 일반화 검증으로 해석하지 않는다.

재실행 산출물:

```text
automatic_calibration/generated/ch1_20260818_four_pair_recheck_v2/
  calibration_result.json
  training_scene_validation.csv
  holdout_scene_validation.csv
  matching_scene_0.png ... matching_scene_3.png
  scene_0_colorized_lidar_3d_preview.png ... scene_3_colorized_lidar_3d_preview.png
```

## 수정 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-18 | 최신 운영자 확인을 반영해 20260818 현재 네 세트를 고정환경·동일 installation epoch로 재확정하고, 고정환경의 의미와 새 epoch 분리 조건을 명시 |
| 2026-08-18 | 20260818 네 세트를 동일 고정환경 데이터로 명시하고, 단일 probe의 timestamp 제한과 네 번째 hold-out 판정을 분리해 기록 |
| 2026-08-18 | 최근 추가된 네 번째 CH1 image–scan을 포함해 3 training + 1 hold-out으로 재실행하고 `PASS` 및 기존 hold-out 수치 일치를 확인 |

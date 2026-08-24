# CV 자동 캘리브레이션 진행 현황 및 설계 요약

문서 목적: Confluence `VPT / CV` 폴더에 게시할 수 있는 프로젝트 진행 현황 원본

작성일: 2026-08-19  
최종 수정일: 2026-08-19  
대상 범위: Calibration Core, CH1 자동 캘리브레이션, LiDAR JSON 계약, 고정환경 검증

## 1. 프로젝트 목적

카메라 영상과 1D LiDAR pan-tilt sweep만으로 카메라–LiDAR 외부 파라미터
`T_camera_lidar = (R, t)`를 자동 추정하고, 실제 actuator가 완성되기 전에도 반복 가능한
conformance test를 수행하는 것이 목표다.

현재 범위는 다음과 같다.

- CH1을 우선 검증한다. CH2~CH4는 CH1 검증 완료 후 확장한다.
- 카메라 내부 파라미터는 자동 추정하지 않고, Manual ChArUco로 검증한 `K + distortion`을
  고정한다.
- 카메라는 Hanwha Vision PNM-C16083RVQ를 사용한다.
- 현재 운용 경로는 LDC 미사용 상태에서 raw image에 Manual distortion 보정을 적용한다.
- 현재 자동 RT는 고정 설치·조명 환경에서 재현성을 검증하는 후보이며, 절대 ground truth로
  확정된 값은 아니다.

## 2. 시스템 구성

### 2.1 카메라

- 모델: Hanwha Vision PNM-C16083RVQ
- 채널: CH1 우선
- 상하/좌우 반전: 사용 안 함
- 복도뷰 회전: 0°
- zoom/focus: 측정 세션 동안 고정
- LDC: 카메라에서 사용하지 않음. 실행 시 `--ldc-enabled false`와 Manual intrinsic을
  명시한다.

현재 사용 중인 Manual intrinsic profile:

```text
resolution = 2592 x 1520
fx = 2033.901952
fy = 2037.779638
cx = 1337.029701
cy = 745.370056
distortion = [-0.5653174395, 0.3445938561, -0.0039145366,
               0.0008182749, -0.1080941249]
profile = charuco-pass-clean18-20260814
```

### 2.2 LiDAR와 actuator

- LiDAR: TOFSense F2P
- 입력: pan/tilt sweep JSON, schema 1.2
- JSON의 `signal_strength`는 원본값을 유지한다.
- actuator가 제공하는 pan/tilt 값은 좌표 변환 직전에 사용한다.
- pan 증가 방향은 실제 장치 기준 Top-view 시계 방향으로 확인했다.

### 2.3 설치 치수

현재 고정환경에서 카메라는 LiDAR 위에 있고, LiDAR 중심축에서 수평으로 59.28 mm 떨어져
있다. 카메라와 LiDAR 중심의 수직 차이는 81.05 mm다. JSON `+Y=down` 기준으로 실행에
사용한 camera-center prior는 다음과 같다.

```text
camera_center_lidar = (+0.05928, -0.08105, 0.0) m
```

이 값은 초기 translation prior/진단값이며, 최종 외부 파라미터 `t`는 calibration이
추정한다.

## 3. 좌표계 계약

LiDAR JSON의 `frame.range_formula`를 유일한 입력 좌표 계약으로 사용한다.

```text
r = distance_m + sensor.range_offset_m
x = r*cos(tilt)*sin(pan)
y = -r*sin(tilt)
z = r*cos(tilt)*cos(pan)
```

- 좌표계: right-handed
- `+x`: right
- `+y`: down
- `+z`: forward
- `pan+`: right
- `tilt_rad=0`: 수평
- `tilt_rad=-π/2`: 아래 방향
- `mechanism.tilt_zero=nadir`: 좌표각의 원점이 아니라 기구 홈 메타데이터
- 내부 계산 단위: meter

카메라 변환은 다음 계약으로 고정한다.

```text
p_camera = R_camera_lidar * p_lidar + t_camera_lidar
```

PLY/OBJ viewer 변환은 계산 좌표와 분리한다.

```text
viewer_z_up = (lidar_x, lidar_z, -lidar_y)
```

## 4. Calibration Core 처리 흐름

```text
JSON scan
  → frame.range_formula 기반 3D point 생성
  → range discontinuity + surface-normal 변화 계산
  → 평면 region growing / plane fitting / plane merge
  → plane intersection, plane-boundary, persistent occlusion 구조선 추출
  → camera grayscale / gradient / Canny edge 생성
  → yaw × down × optical-roll 후보 투영
  → z-buffer 가시성 필터
  → spatial NID + edge distance + structural/Manhattan residual 계산
  → 인접 8개 후보를 포함한 contiguous basin 선택
  → Ceres refinement
  → training gate + fixed-RT holdout gate
```

목적함수는 geometry NID, edge, 구조선, Manhattan 방향을 함께 사용한다.
`signal_strength` NMI는 별도 perturbation 진단은 구현됐지만, 실데이터 conformance가
충분히 확인될 때까지 기본 weight `0`으로 유지한다.

구조선은 raw range gap만으로 확정하지 않는다. 같은 평면 내부의 깊이 변화는 제거하고,
평면 교차선·승인 평면과 geometry 경계·반복 폐색선만 calibration 구조선으로 승격한다.
이 변경으로 책상 상판, 장애물, 사람의 단순 실루엣이 벽–바닥 경계와 동일하게 취급되는
문제를 줄였다.

## 5. 주요 변경 및 실패 분석

### 초기 실패

- 좌표계 식을 JSON 계약과 다르게 적용해 2D 투영이 바닥 또는 반대 방향으로 고정됐다.
- `tilt_zero=nadir`를 계약각의 0°로 오해할 수 있는 문서 표현이 문제였다.
- 평면·천장·바닥 위주 환경은 회전 대칭성이 높아 잘못된 yaw도 높은 점수를 얻었다.
- 45° coarse search는 방향 후보 간격이 너무 커 실제 basin을 놓칠 수 있었다.

### 현재 반영한 수정

- JSON `range_formula`를 adapter에서 검증하고, `tilt_rad`를 0° 수평 / 음수 하향으로
  처리한다.
- pan 증가 방향을 Top-view 시계 방향으로 반영했다.
- yaw/down을 독립적으로 탐색하고, 공통 `prior-roll=90°` 가정을 제거했다.
- coarse yaw는 실험에서 5° 간격을 사용하고, 주변 후보 8개의 Gaussian 보정과 contiguous
  basin을 이용한다.
- 평면·normal·구조선·z-buffer·holdout gate를 목적함수 및 검증 산출물에 반영했다.
- PNM 카메라의 수동 내부 파라미터와 raw distortion 보정 경로를 고정했다.

## 6. 자동화 테스트 상태

Docker Ubuntu 환경에서 CMake/CTest를 실행했다.

```text
100% tests passed, 0 tests failed out of 5
```

테스트는 평면 교차선 생성, 평행면 오검출 방지, 폐색 진단선 분리, 평면 조각 병합,
수평면 IMU-Y 복구를 포함한다.

## 7. 2026-08-18 CH1 자동 캘리브레이션

입력 디렉터리:

```text
data/real_calibration/session-const-env/repeat_test_sample/20260818/
```

초기 3쌍:

```text
20260818-143751-CH1.jpg ↔ calib-20260818-143748_...json
20260818-145847-CH1.jpg ↔ calib-20260818-145912_...json
20260818-151305-CH1.jpg ↔ calib-20260818-151312_...json
```

세 영상 모두 ChArUco `17/17 marker`, `24/24 corner`를 검출했다. 3쌍을 사용한 자동
캘리브레이션은 내부 training gate를 통과했다.

선택 후보:

```text
yaw = 170°
down = 20°
optical roll = 0°
refined down = 19.9989°
```

추정 RT:

```text
R = [-0.9848197395,  0.0000214074,  0.1735801838]
    [ 0.0593846361,  0.9396991087,  0.3368071407]
    [-0.1631059338,  0.3420023166, -0.9254355028]

t = [0.0583799531, 0.0726416842, 0.0373928642] m
```

결과 경로:

```text
automatic_calibration/generated/ch1_20260818_three_pair_v1/
```

이 결과는 당시 holdout이 없었기 때문에 `candidate PASS / verification pending`으로
보류했다.

## 8. 네 번째 pair 고정 RT holdout 검증

추가된 네 번째 pair:

```text
20260818-155208-CH1.jpg
calib-20260818-154229_sweep-000001_pan_tilt_lidar.json
```

파일명 시각 차이는 9분 39초지만, 사용자가 그 시간 동안 카메라·LiDAR actuator·장면이
고정됐음을 확인했다. 따라서 같은 고정환경의 유효한 독립 holdout으로 판정했다.

### 결과

| 항목 | 결과 |
|---|---:|
| ChArUco | 17/17 marker, 24/24 corner |
| ChArUco RMSE / max | 1.316 / 2.133 px |
| fixed RT scene pass | 1/1 |
| projected ratio | 0.7759 |
| mean edge distance | 20.57 px |
| edge p50 / p90 | 12.37 / 55.00 px |
| 30 px 초과 edge | 21.99% |
| geometry NID | 0.9482 |
| structural match | 21/21 |
| Manhattan vertical error | 11.21° |

최종 투영을 육안 확인한 결과 벽·왼쪽 캐비닛·책상 평면의 방향과 위치는 유지됐다. 다만
일부 모서리의 residual은 남아 있으므로 pixel-level 정답이나 절대 ground truth로 해석하지
않는다.

산출물:

```text
automatic_calibration/generated/ch1_20260818_holdout_155208_fixed_v1/
  fixed_pose_validation_result.json
  fixed_pose_scene_validation.csv
  debug/scene_0/06_projection_final.png
  debug/scene_0/07_projection_final_edges.png
  debug/scene_0/07a_projection_final_edge_residual.png
```

현재 판정은 `candidate PASS / fixed-environment hold-out PASS`다. 장치가 이동하거나
zoom/focus/LDC 상태가 바뀌면 동일 RT를 재사용할 수 있다는 의미는 아니다.

### 8.1 네 세트 전체 재실행 확인

추가 pair를 포함한 네 세트를 다시 실행해 scene 0~2를 training, scene 3을 holdout으로
분리했다. 결과는 다음 경로에 저장했다.

```text
automatic_calibration/generated/ch1_20260818_four_pair_recheck_v2/
```

| 항목 | 결과 |
|---|---:|
| 전체 status | `PASS` |
| training | `3/3 PASS` |
| holdout | `1/1 PASS` |
| 선택 down / refined down | `20.0° / 19.9989°` |
| optical roll | `0.0°` |
| holdout mean edge distance | `20.574 px` |
| holdout geometry NID | `0.948202` |
| holdout structural match | `21/21` |

고정 RT holdout 결과와 네 세트 전체 재실행 결과의 holdout CSV가 행 단위로 일치했다.
따라서 새 파일 추가가 기존 CH1 RT를 변경하거나 결과를 악화시키지 않았으며, 현재
고정된 installation epoch에서 RT가 반복 적용됨을 재확인했다. 이 결과는 설치가 변경된
환경으로의 일반화나 Manual RT의 절대 정확도를 의미하지 않는다.

## 9. Manual RT와의 비교

자동 RT와 기존 Manual reference의 차이는 다음과 같다.

```text
rotation geodesic difference = 10.6906°
translation difference       = 139.17 mm
```

Manual RT 자체도 독립적으로 승인된 ground truth가 아니므로, 이 차이만으로 자동 RT를
실패로 판정하지 않는다. 다만 절대 정확도 승인에는 다음 중 하나가 추가로 필요하다.

- LiDAR 좌표계에서 ChArUco 보드 pose를 독립 측정
- 센서–보드–카메라의 물리 기준을 포함한 marker 기반 reference 구성
- 설치를 변경한 독립 환경에서 동일 RT의 재현성 검증

## 10. 현재 완료 / 미완료

### 완료

- Ubuntu/Docker 기반 Calibration Core 및 CMake/CTest 구성
- LiDAR JSON adapter와 좌표계 계약 검증
- Manual intrinsic + raw distortion 보정 경로
- range/normal geometry NID, edge, 구조선, Manhattan, z-buffer
- 인접 후보 basin 보정 및 Ceres refinement
- CH1 고정환경 3쌍 추정 + 네 번째 pair fixed-RT holdout PASS
- 네 세트 전체 재실행(`3 training + 1 holdout`) 및 기존 결과 checksum/CSV 일치 확인
- 단계별 debug image/PLY/OBJ/CSV/JSON 산출

### 미완료

- Manual RT의 독립 ground truth 품질 승인
- 설치 이동 후 재현성 검증
- CH2~CH4 확장 검증
- actuator 최종 firmware/시간 동기화 및 실제 스트리밍 연동
- signal-strength NMI를 실운영 목적함수에 활성화할 conformance 근거 확보
- 카메라 LDC 상태를 장치 API 또는 설정 export로 자동 확인

## 11. 권장 다음 단계

1. 현재 CH1 fixed-environment 결과를 baseline으로 보관한다.
2. 장치를 한 번 재설치한 뒤 CH1 image–scan pair를 추가하고 같은 RT를 고정 검증한다.
3. 설치 변경 데이터에서 FAIL이면 RT가 환경 종속인지, actuator 원점/좌표 계약이 달라졌는지
   먼저 확인한다.
4. CH1이 안정화된 후 동일 절차로 CH2~CH4를 진행한다.
5. conformance 승인 전에는 `calibration_result.json`의 RT를 운영값으로 자동 교체하지
   않고, 검증 리포트와 함께 승인한다.

## 12. 변경 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-13 | Calibration Core, 좌표계, 초기 실데이터 실패 상태 정리 |
| 2026-08-14 | Manual intrinsic/왜곡 보정, geometry NID, 구조선, holdout gate 반영 |
| 2026-08-18 | A4 ChArUco 보드 부착, CH1 3쌍 자동 추정, 네 번째 pair 추가 |
| 2026-08-18 | 기존 RT 고정 holdout `1/1 PASS`, projected/edge residual 확인 |
| 2026-08-19 | 고정환경 확인을 반영해 현재 상태를 `fixed-environment hold-out PASS`로 확정 |

## 13. 저장소 내 상세 문서

- `automatic_calibration/docs/CURRENT_PROGRESS_AND_STATUS.md`
- `automatic_calibration/docs/CH1_ARUCO_VALIDATION_20260818.md`
- `automatic_calibration/docs/CALIBRATION_CORE_ARCHITECTURE.md`
- `automatic_calibration/docs/PAN_TILT_LIDAR_JSON_INTERFACE.md`
- `automatic_calibration/docs/REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md`
- `manual_calibration/docs/SESSION_CONST_ENV_CALIBRATION_RECORD.md`

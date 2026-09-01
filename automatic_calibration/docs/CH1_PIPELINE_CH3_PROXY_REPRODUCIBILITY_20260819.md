# CH1 파이프라인 CH3 임시 재현성 시험

작성일: 2026-08-19  
시험 성격: CH3 자동 캘리브레이션 개발이 아닌 CH1 파이프라인의 교차 채널 임시 재현성 확인  
판정: **PROVISIONAL PASS / 정밀도 및 독립 기준 RT 미확정**

## 1. 목적

CH1에서 고정한 자동 캘리브레이션 알고리즘, 탐색 범위, 목적함수와 품질 게이트를
변경하지 않고 CH3 시야에 적용해 다음을 확인한다.

1. CH1에서만 우연히 통과한 알고리즘인지 확인한다.
2. 반대 방향을 보는 채널에서도 gross yaw/down 방향을 복구하는지 확인한다.
3. 과거의 바닥·반대 벽 투영 false positive가 반복되는지 확인한다.

이 시험은 CH3 제품 캘리브레이션 완료를 의미하지 않는다. CH3는 CH1 알고리즘의
독립 입력 역할만 한다.

## 2. 입력과 pairing

8월 14일 CH3 조명 ON 영상 3장과 같은 고정환경의 연속 LiDAR sweep 3개를 파일명
순서로 연결했다.

| scene | CH3 이미지 | LiDAR JSON |
|---:|---|---|
| 0 | `20260814-230710-CH3.jpg` | `calib-20260814-232414_sweep-000001_pan_tilt_lidar.json` |
| 1 | `20260814-230743-CH3.jpg` | `calib-20260814-233403_sweep-000001_pan_tilt_lidar.json` |
| 2 | `20260814-230747-CH3.jpg` | `calib-20260814-234352_sweep-000001_pan_tilt_lidar.json` |

실행 입력은 원본을 복사하지 않고 다음 심볼릭 링크 디렉터리로 구성했다.

```text
data/real_calibration/session-const-env/repeat_test_sample/ch3_proxy_20260814/
```

세 장 모두 RT 추정에 사용했다. 따라서 별도 hold-out은 없으며 시간 동기화 시험으로도
해석하지 않는다.

## 3. 고정한 CH1 파이프라인 조건

- 카메라: PNM-C16083RVQ, `2592 x 1520`
- LDC: `false`
- 영상 상태: `raw`, Manual ChArUco distortion으로 undistort
- 내부 파라미터: CH1 `charuco-pass-clean18-20260814` 고정
- intrinsic refinement: 비활성
- yaw: `-180~175°`, `5°` 간격
- down: `0~30°`, `5°` 간격
- optical roll: `-15~15°`, `5°` 간격
- direction prior weight: `0`
- training scene pass ratio: `1.0`
- signal NMI weight: `0`

알고리즘과 게이트는 CH1 시험과 동일하다. 센서의 실제 위치 차이만 반영하기 위해
카메라 중심은 CH3 물리 prior `(-0.05928, -0.08105, 0) m`를 사용했다.

실행 명령:

```bash
/workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/real_calibration/session-const-env/repeat_test_sample/ch3_proxy_20260814 \
  --output /workspace/automatic_calibration/generated/ch1_pipeline_ch3_proxy_20260814_v1 \
  --camera-channel 3 \
  --ldc-enabled false \
  --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --allow-intrinsic-refinement false \
  --camera-center-x-m -0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --down-min-deg 0 --down-max-deg 30 --down-step-deg 5 \
  --optical-roll-min-deg -15 --optical-roll-max-deg 15 --optical-roll-step-deg 5 \
  --yaw-step-deg 5 --direction-prior-weight 0 \
  --minimum-scene-pass-ratio 1.0 --minimum-scene-direction-groups 1 \
  --debug-output /workspace/automatic_calibration/generated/ch1_pipeline_ch3_proxy_20260814_v1/debug
```

## 4. 결과

### 4.1 Search boundary와 방향 결정 방식

이번 실행의 자세 탐색 범위는 다음과 같다.

| 축/변수 | 탐색 범위 | 간격 | 후보 수 | 비고 |
|---|---:|---:|---:|---|
| yaw | `-180~175°` | `5°` | 72 | LiDAR 수직축을 중심으로 360° 전체 탐색 |
| down | `0~30°` | `5°` | 7 | CH1에서 확인한 설치 범위를 사용 |
| optical roll | `-15~15°` | `5°` | 7 | 영상 회전·상하·좌우 반전 미사용 조건 |
| focal scale | `1.0` | 고정 | 1 | Manual intrinsic 고정 |

따라서 coarse 자세 평가는 다음과 같이 총 3,528개다.

```text
72 yaw × 7 down × 7 optical roll = 3,528 coarse poses
```

각 down/optical-roll 조합에서 yaw 72개를 먼저 평가하고, 가장 점수가 낮은 yaw를
시작점으로 Ceres가 RT를 연속값으로 미세 조정한다. 이번 실행에는 별도 1° fine grid를
사용하지 않았다. `20.0671°`처럼 정수가 아닌 최종 down 값은 Ceres 연속 최적화의
결과다. 연속 최적화의 회전 탐색 bound는 선택한 coarse start 주변 angle-axis 성분별
`±10°`, translation bound는 `±0.10 m`다. 카메라 중심에는 실측 위치 prior가 별도로
적용된다.

이번 실행은 **yaw에 대해서는 blind 360° search**지만 전체 3축 blind search는 아니다.
down과 optical roll은 CH1 설치·영상 설정을 기준으로 제한했다. 실제 카메라의 down이
30°보다 크거나 영상이 크게 회전된 상태라면 현재 범위 밖의 해는 찾을 수 없다.

#### 4.1.1 2D–3D 후보 평가 순서

방향 후보마다 원본 이미지 자체를 회전시키는 것이 아니라, 후보 `T_camera_lidar`를
LiDAR 특징에 적용한 뒤 카메라 내부 파라미터로 2D 이미지 평면에 투영한다.

```text
2D 이미지 특징 추출
  ├─ Canny edge와 edge distance map
  ├─ gradient magnitude
  ├─ LSD 구조선
  └─ 소실점/Manhattan 방향
             ↕ 후보별 비교
3D LiDAR 특징 추출
  ├─ 거리 불연속 edge
  ├─ surface-normal 변화
  ├─ 평면 경계·평면 교차선
  └─ 반복 폐색선과 LiDAR 중력/벽축
             ↓
yaw/down/roll 후보별 3D→2D 투영과 점수 계산
             ↓
연속된 저점수 basin 및 gate를 만족하는 후보 선택
             ↓
Ceres 연속 RT refinement
```

모든 원본 LiDAR 포인트를 matching edge로 사용하지는 않는다. 추출된 edge·geometry·
구조선 특징을 후보 점수에 사용하고, 전체 점군은 z-buffer 가시성 판정과 최종 색상
PLY/OBJ 생성에 사용한다. 카메라 뒤 또는 영상 범위 밖에 있는 점과 z-buffer에서 가려진
점은 후보 평가에서 제외한다.

이번 실행의 복합 목적함수 입력 가중치는 다음과 같다.

| 목적함수 성분 | 가중치 |
|---|---:|
| Geometry NID | `0.55` |
| 2D edge distance | `0.25` |
| 구조선 대응 | `0.20` |
| Manhattan 방향 | `0.15` |
| signal-strength NMI | `0.00` |
| 카메라 방향 prior | `0.00` |

따라서 단일 이미지 edge 점수만으로 방향을 결정하지 않는다. coarse overlap 단계에서
화면 안의 visible LiDAR edge와 geometry feature가 충분하지 않은 방향은 먼저 제거하고,
남은 후보는 geometry NID, edge, 구조선, Manhattan 방향을 함께 평가한다. 이후 인접
후보의 점수를 반영한 연속 basin을 proposal로 사용하지만, 최종 선택은 refinement와
품질 gate를 통과한 후보를 우선한다.

#### 4.1.2 Manual 입력과 방향 prior 구분

이번 CH3 임시 시험에서 사용한 수동 정보는 다음 두 종류뿐이다.

1. CH1에서 구한 `K + distortion`: raw 영상의 왜곡 보정과 3D→2D projection에 사용
2. CH3 카메라 중심 `(-0.05928, -0.08105, 0) m`: translation/camera-center prior에 사용

카메라 중심 `C_lidar`는 각 회전 후보에서 다음과 같이 translation으로 변환된다.

```text
t_camera_lidar = -R_camera_lidar × C_lidar
```

이는 카메라 위치를 제한하지만 광축 방향을 지정하지 않는다. CH3의 수평 X 부호가
CH1과 반대라는 위치 정보는 들어가지만, “CH3 광축은 CH1에서 180° 반대”라는 조건은
들어가지 않았다.

이번 실행에서 사용하지 않은 값은 다음과 같다.

- CH1 Manual `T_camera_lidar`
- CH3 Manual `T_camera_lidar`
- CH1 Automatic RT를 CH3 초기 방향으로 사용하는 설정
- CH3 사전 heading
- `expected-camera-forward/down`
- `camera-outward-facing` 방향 prior
- `manual-reference-json`

결과 JSON의 실제 상태도 다음과 같다.

```text
direction_prior_weight = 0
expected_camera_forward_lidar = (0, 0, 0)
expected_camera_down_lidar = (0, 0, 0)
manual_reference_json = null
```

`--camera-channel 3`은 결과 provenance에 채널 번호를 기록하며, 채널 번호를 특정
heading으로 변환하는 hard-coded table은 현재 실행 경로에 없다.

#### 4.1.3 약 180° 관계의 출처

알고리즘은 실행 전에 CH1과 CH3가 반대 방향이라는 값을 알지 못했다. CH3에서 360°
yaw 탐색과 refinement가 끝난 후 두 최종 RT에서 top-view heading을 계산해 비교했다.

```text
CH1 final top-view heading = -99.996°
CH3 final top-view heading =  71.844°
원형 heading 차이          = 171.840°
전체 회전 geodesic 차이    = 169.552°
```

즉 약 180° 관계는 입력 prior가 아니라 **계산 완료 후 얻은 사후 비교 결과**다. CH3의
coarse yaw가 `-20°`, CH1이 `170°`로 선택된 것도 같은 경향을 보인다. 다만 coarse yaw는
down/roll과 결합되기 전의 내부 후보 좌표이므로 물리 방향 비교에는 최종 top-view
heading을 사용한다.

현재 값은 반대 시야라는 장치 배치 설명과 일관되지만 승인된 CH3 Manual RT 또는
독립적인 광축 실측값으로 검증한 ground truth는 아니다. 그러므로 “알고리즘이 약
반대 방향을 독립적으로 선택했다”까지는 확인됐지만, `171.840°` 자체를 실제 설치각
정답으로 확정하지 않는다.

### 4.2 전체 판정

| 항목 | CH3 임시 시험 | CH1 기준 시험 |
|---|---:|---:|
| 상태 | `PASS` | `PASS` |
| training scene | `3/3 PASS` | `3/3 PASS` |
| hold-out | 없음 | `1/1 PASS` |
| 선택 yaw | `-20°` | `170°` |
| 선택 down | `20°` | `20°` |
| refined down | `20.0671°` | `19.9989°` |
| optical roll | `-5°` | `0°` |
| 목적함수 개선률 | `5.0533%` | `15.5314%` |
| geometry NID 개선률 | `1.6895%` | `1.0792%` |
| 평균 edge 거리 | `31.527 px` | `19.919 px` |
| projected edge ratio | `0.6653` | `0.7794` |
| 구조선 대응 | `58/64` | `71/72` |
| Manhattan vertical error | `3.372°` | `5.373°` |
| multistart margin | `0.03849` | `0.06012` |

CH3 top-view camera heading은 `71.844°`, CH1은 `-99.996°`다. 차이는
`171.840°`이고 전체 회전 geodesic 차이는 `169.552°`다. 반대 시야를 대체 입력으로
사용한 목적과 일치하는 gross orientation이며, refined down 차이는 `0.068°`다.

### 4.3 장면별 게이트

| scene | 결과 | projected ratio | mean edge | geometry NID | 구조선 대응 | vertical error |
|---:|---|---:|---:|---:|---:|---:|
| 0 | PASS | 0.6515 | 33.107 px | 0.9067 | 19/20 | 2.206° |
| 1 | PASS | 0.6735 | 31.051 px | 0.9081 | 21/22 | 1.073° |
| 2 | PASS | 0.6715 | 30.365 px | 0.8956 | 18/22 | 6.838° |

## 5. 시각 판정

- 세 장 모두 LiDAR 점이 영상의 방·책상·의자·벽 구조 전반에 투영된다.
- 과거 실패처럼 바닥이나 촬영 반대편 한 면에만 집중되는 현상은 보이지 않는다.
- 세 sweep에서 top/front/side의 카메라 방향이 동일하게 재현된다.
- 반면 edge residual은 CH1보다 크다. 장면별 p90 edge 오차는 약 `81~92 px`이고
  약 `32~35%`의 평가 edge가 30 px보다 멀다.
- 따라서 gross 방향 복구는 성공했지만 pixel-level 정밀 정합이 확인된 것은 아니다.

주요 산출물:

```text
automatic_calibration/generated/ch1_pipeline_ch3_proxy_20260814_v1/
  calibration_result.json
  training_scene_validation.csv
  matching_scene_0.png ... matching_scene_2.png
  scene_0_colorized_lidar_3d_preview.png ... scene_2_colorized_lidar_3d_preview.png
  scene_*_colorized_lidar_z_up_reprojection_m.ply
  debug/scene_*/05_projection_initial.png
  debug/scene_*/06_projection_final.png
  debug/scene_*/07_projection_final_edges.png
  debug/scene_*/07a_projection_final_edge_residual.png
```

## 6. 8월 18일 CH3 Manual 자료의 역할

`20260818-131432-CH3.jpg`에서는 A4 ChArUco marker `17개`, corner `24개`가
검출됐고, CH1 내부 파라미터를 넣은 단일 영상 pose의 reprojection RMSE는
`0.735 px`였다. 이 결과는 CH1 K를 임시로 재사용할 수 있다는 호환성 진단에는
도움이 되지만 CH3 K 또는 `T_camera_lidar`의 정답을 확정하지 않는다.

현재 8월 18일 산출물에는 `T_camera_marker_board`만 있고 같은 보드의 승인된
`T_lidar_marker_board`가 없다. 따라서 이번 실행에는 잘못된 CH1 Manual RT를 넣지
않았고 `--manual-reference-json`도 사용하지 않았다.

## 7. 결론

CH1에서 고정한 알고리즘을 별도 CH3 시야에 그대로 적용했을 때 3/3 내부 장면 게이트를
통과했고, CH1과 거의 반대인 heading 및 동일한 down을 복구했다. 그러므로
**CH1 파이프라인의 교차 채널 임시 재현성은 PROVISIONAL PASS**로 판정한다.

다음 이유로 제품 conformance PASS 또는 CH3 calibration 완료로 승격하지 않는다.

1. CH3 전용 intrinsic이 아니라 CH1 intrinsic을 임시 재사용했다.
2. 세 장 모두 추정에 사용돼 독립 hold-out이 없다.
3. 승인된 CH3 Manual `T_camera_lidar` 기준이 없다.
4. 목적함수 개선률이 최소 게이트 5%보다 `0.053%p`만 높고 edge 정밀도는 CH1보다 낮다.

후속 시험에서는 알고리즘·임계값을 바꾸지 않은 채 CH3의 독립 image-scan 한 쌍 또는
승인된 Manual CH3 RT를 추가해 현재 RT를 고정 검증한다.

## 수정 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-19 | CH1 파이프라인을 고정한 CH3 3쌍 임시 재현성 실행 결과와 시각 판정 최초 작성 |
| 2026-08-19 | yaw/down/optical-roll search boundary, 2D–3D 후보 평가 순서, Manual 입력 범위, 약 180° 관계가 사후 결과임을 명시 |

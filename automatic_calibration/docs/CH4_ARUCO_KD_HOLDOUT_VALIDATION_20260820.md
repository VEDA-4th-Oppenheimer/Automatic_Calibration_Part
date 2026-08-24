# CH4 ArUco K·D 및 자동 RT Hold-out 검증

- 작성일: 2026-08-20
- 대상 채널: PNM-C16083RVQ CH4
- 데이터: `repeat_test_sample/20260819`
- 결론: **CH4 전용 K·D와 2 training + 1 hold-out으로 `CANDIDATE_RT` 생성**
- 제품 적용: **금지 (`activation_allowed=false`)**

## 1. 검증 목적

CH1 전체 장면에서 A4 ChArUco 보드가 검출되지 않아, CH4 전용 ChArUco 사진으로
구한 K·D를 고정하고 CH4 image–LiDAR pair만으로 자동 RT를 다시 추정했다. CH1의
intrinsic 또는 RT는 사용하지 않았다.

## 2. CH4 intrinsic profile

사용 파일:

```text
manual_calibration/output/session-const-env/intrinsic-ch4-20260819-full/camera_intrinsic.json
```

보드가 충분히 보이는 CH4 사진 18장으로 구한 PASS profile이다.

| 항목 | 값 |
|---|---:|
| Calibration RMS | `1.5427 px` |
| fx | `3125.0812` |
| fy | `3140.1864` |
| cx | `1177.2851` |
| cy | `713.8399` |
| 해상도 | `2592 × 1520` |
| 왜곡 모델 | `opencv_radtan` |
| D | `[0.009664, -2.569339, -0.003587, -0.046293, 4.735001]` |

41장을 필터링 없이 사용한 기존 CH4 profile은 RMS `4.0848 px`로 REVIEW였으므로
이번 실행에서 사용하지 않았다.

## 3. 전체 장면의 A4 ChArUco 검출

보드 설정은 `DICT_5X5_100`, 7×5, marker `20 mm`, square `27 mm`다.

| CH4 image | marker | ChArUco corner | RMS | 판정 |
|---|---:|---:|---:|---|
| `20260819-184401-CH4.jpg` | 6 | 6 | `0.4895 px` | PASS |
| `20260819-184405-CH4.jpg` | 6 | 6 | `0.6342 px` | PASS |
| `20260819-200913-CH4.jpg` | 2 | 1 | - | FAIL: `CHARUCO_CORNERS_INSUFFICIENT` |

앞의 두 프레임에서 구한 `T_camera_marker_board` 차이는 translation `0.358 mm`,
rotation `0.073°`다. 두 프레임이 거의 같은 장면이므로 이는 짧은 시간의 검출
일관성 증거이며 독립적인 카메라–LiDAR RT ground truth는 아니다.

주의할 점은 한 영상에 같은 ID 배열을 가진 A4 보드 복사본이 여러 장 보이고 일부는
프레임 경계에서 잘려 있다는 것이다. 현재 검출기는 우측 하단의 일부 보드를 선택했다.
향후 기준 촬영에서는 한 화면에 보드 한 장만 두거나 각 보드에 서로 다른 ID 범위를
사용해야 한다.

검출 산출물:

```text
manual_calibration/output/session-const-env/validation/20260819-a4-ch4-scene/
```

## 4. 자동 RT 입력 구성

| 역할 | image | LiDAR JSON |
|---|---|---|
| training 0 | `20260819-184401-CH4.jpg` | `calib-20260819-184249_sweep-000001_pan_tilt_lidar.json` |
| training 1 | `20260819-184405-CH4.jpg` | `calib-20260819-185256_sweep-000001_pan_tilt_lidar.json` |
| hold-out | `20260819-200913-CH4.jpg` | `calib-20260819-200851_sweep-000001_pan_tilt_lidar.json` |

공통 조건:

- CH4 manual K·D 고정
- raw image에 manual D로 한 번만 왜곡 보정
- `ldc=false`, zoom/focus locked
- camera center in LiDAR frame: `(0.05928, -0.08105, 0) m`
- yaw 전체 탐색, coarse `5°`
- down `0~30° / 5°`
- optical roll `-15~15° / 5°`
- staged 탐색: coarse → contiguous basin → 5° → 1° → Ceres
- 마지막 한 pair는 후보 선택에 사용하지 않고 hold-out 검증만 수행

## 5. 자동 RT 결과

산출물:

```text
automatic_calibration/generated/ch4_20260819_staged_kd_holdout_20260820_v1/
```

최종 상태는 `CANDIDATE_RT`, reason `PASS`, `activation_allowed=false`다.

| 항목 | 결과 |
|---|---:|
| selected yaw seed | `97°` |
| selected down | `11°` |
| selected optical roll | `9°` |
| training | `2/2 PASS` |
| hold-out | `1/1 PASS` |
| final composite objective | `0.65587` |
| final mean edge distance | `33.22 px` |
| geometry NID | `0.86453` |
| Manhattan vertical error | `6.53°` |
| edge coverage | `0.9930` |
| NID coverage | `0.9912` |

`selected yaw seed=97°`는 내부 카메라 자세 조합의 탐색 변수다. viewer Z-up에서 최종
카메라 광축의 top-view heading은 `-173.01°`이므로, 3D preview 화살표와 `97°`를
같은 방위각으로 직접 비교하면 안 된다.

`T_camera_lidar` 계약은 `p_camera = R * p_lidar + t`이며 결과는 다음과 같다.

```text
R = [
  [-0.1498594591, -0.1536348129,  0.9766977459],
  [ 0.1681541594,  0.9695009312,  0.1783034580],
  [-0.9743029925,  0.1909562482, -0.1194545523]
]

t_m = [-0.0035686915, 0.0686085790, 0.0732343092]
```

장면별 결과:

| scene | 역할 | visible/aligned edge | mean edge | geometry NID | Manhattan vertical | 판정 |
|---:|---|---:|---:|---:|---:|---|
| 0 | training | `509/349` | `36.17 px` | `0.86243` | `2.82°` | PASS |
| 1 | training | `485/347` | `30.13 px` | `0.86664` | `10.24°` | PASS |
| 2 | hold-out | `480/363` | `27.59 px` | `0.91476` | `2.62°` | PASS |

## 6. 판정 범위

이번 결과로 다음은 확인됐다.

1. CH4 전용 K·D가 정상 로드되고 raw 영상 왜곡 보정에 적용됐다.
2. 현재 3쌍에서는 같은 RT가 두 training과 미사용 hold-out의 내부 품질 gate를 통과했다.
3. 기존 CH4 `1/3 FAIL` 결과와 달리, 현재 camera-center prior 및 최신 staged/coverage
   정책에서는 유효한 `CANDIDATE_RT`를 생성했다.

그러나 카메라에서 검출한 ArUco만으로는 `T_camera_lidar`의 절대 정답을 만들 수 없다.
현재 ArUco가 제공하는 것은 `T_camera_marker_board`뿐이며, 같은 기준의
`T_lidar_marker_board`가 없기 때문이다. 따라서 이번 PASS는 내부 2D–3D 품질 및
hold-out PASS이지 `PRODUCT_APPROVED_RT`가 아니다.

제품 승격 전에는 다음 중 하나가 추가로 필요하다.

- LiDAR에서도 식별 가능한 단일 기준 보드의 `T_lidar_marker_board` 측정
- 독립 manual/external RT 기준과의 회전·이동 오차 비교
- 다른 시각·다른 scan의 추가 CH4 hold-out에서 같은 RT 반복 확인

## 7. 주요 확인 파일

- `calibration_result.json`: 최종 상태, RT, 검색 단계 및 lifecycle
- `matching_scene_0.png`, `matching_scene_1.png`: training 투영
- `matching_scene_2.png`: hold-out 투영
- `scene_0_colorized_lidar_3d_preview.png`: 3D 방향 확인
- `training_scene_validation.csv`, `holdout_scene_validation.csv`: 장면별 gate

## 8. 수정 로그

| 날짜 | 내용 |
|---|---|
| 2026-08-20 | CH4 전용 PASS K·D 확인 및 전체 장면 ArUco 검출 수행 |
| 2026-08-20 | 2 training + 1 hold-out 최신 staged 자동 RT 검증 수행 |
| 2026-08-20 | `CANDIDATE_RT`, training `2/2`, hold-out `1/1`, activation 금지 상태 기록 |

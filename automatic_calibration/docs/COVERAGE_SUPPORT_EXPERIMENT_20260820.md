# Coverage 기반 후보 선택 수정 및 2026-08-19 실데이터 실험

작성일: 2026-08-20  
대상: `repeat_test_sample/20260819` CH1 3쌍  
목적: 방향은 대체로 맞지만 실제 투영이 카메라 기준 오른쪽으로 더 이동해야 하는
상황에서, 일부 엣지만 우연히 맞춘 local minimum을 제거하고 올바른 yaw basin을
선택하는지 확인한다.

## 1. 문제와 수정 이유

19일 독립 재추정 후보는 내부 품질 게이트를 통과했지만, 18일 RT를 19일 데이터에
고정 적용한 결과와 비교하면 회전 차이가 약 `79.5°`였다. 두 후보의 지지 범위를
비교하면 독립 후보가 훨씬 적은 LiDAR 구조만 사용했다.

| 항목 | 18일 RT 고정 적용 | 19일 독립 후보 |
|---|---:|---:|
| scene별 visible edge | `498~509` | `110~165` |
| scene별 구조선 visible | `21~29` | `12~15` |
| 평균 edge 거리 | `17.1~25.0 px` | `6.8~11.3 px` |

기존 점수는 `LiDAR edge → camera edge distance`의 평균과 NID를 중심으로 계산했다.
따라서 카메라 영상의 한 영역에 들어오는 소수의 포인트만 잘 맞으면, 나머지 LiDAR
구조가 화면 밖으로 빠진 후보도 좋은 점수를 받을 수 있었다. 기존 `visible_edge >=
100` 하한은 이 데이터에서 `110~165` 포인트 후보를 제거하지 못했다.

초기 수정은 다음 세 가지를 함께 적용했다.

1. **상대 edge coverage**: 같은 down/optical-roll layer에서 yaw 후보가 보이는 edge
   수를 해당 layer의 최대값으로 나눈다.
2. **상대 NID coverage**: 같은 layer에서 NID에 투영된 geometry point 수를 최대값으로
   나눈다.
3. **영상 공간 coverage**: 영상 3×4 격자에서 보이는 LiDAR edge가 차지하는 셀 수를
   계산하고 같은 layer의 최대 셀 수와 비교한다.

후보의 평균 오차만 비교하지 않고 다음 penalty도 추가한다.

```text
coverage_objective =
    ((1 - edge_coverage)^2
   + (1 - nid_coverage)^2
   + (1 - spatial_coverage)^2) / 3

coarse_objective += coverage_penalty_weight * coverage_objective
```

초기 구현은 세 상대 coverage 모두를 hard gate로 사용했다. 그러나 카메라가 보지 않는
360° scan sector까지 layer 최대 support와 비교하면 실제 정합 방향도 탈락할 수 있음이
확인됐다. 현재 edge coverage는 아래 penalty만 적용하며, hard gate는 상대 NID와 영상
공간 coverage에만 남긴다. Ceres 안에서 z-buffer/coverage를 매 iteration 재구성하지는
않는다. 해당 값은 불연속적이므로 coarse 후보에서 가시 집합을 고정해 계산한다.

## 2. 구현 변경

### 2.1 Core 설정과 진단값

다음 설정을
[`calibration_core.hpp`](../include/auto_calib/calibration_core.hpp)에 추가했다.

```text
minimum_relative_nid_coverage
minimum_relative_edge_spatial_coverage
coverage_penalty_weight
coverage_grid_rows = 3
coverage_grid_columns = 4
```

`CalibrationMetrics`에는 다음 결과를 추가했다.

```text
max_coarse_visible_edge_points
max_coarse_nid_projected_points
max_coarse_edge_active_spatial_cells
edge_active_spatial_cells
edge_coverage_ratio
nid_coverage_ratio
edge_spatial_coverage_ratio
coverage_objective
```

`CoarseOrientationScore`에도 후보별 coverage와 penalty를 저장하므로
`orientation_full_search.csv`에서 후보가 왜 탈락했는지 확인할 수 있다.

### 2.2 평가 순서

변경된 실행 순서는 다음과 같다.

```text
카메라 edge/LiDAR edge/NID/구조선 추출
        ↓
같은 down·roll layer의 전체 yaw 후보 평가
        ↓
layer 내부 최대 support 계산
        ↓
후보별 상대 coverage와 penalty 계산
        ↓
absolute overlap + relative NID/spatial coverage gate
        ↓
유효한 연속 후보의 8-neighbor 보정 점수
        ↓
Ceres fine refinement
        ↓
최종 coverage 및 평균 edge/NID/구조선 품질 gate
```

`run_real_calibration.cpp`의 8-neighbor 보정도 `overlap_valid`가 false인 후보를
이웃 평균 계산에서 제외하도록 수정했다. 따라서 저coverage 후보가 주변의 좋은
후보 점수에 편승할 수 없다.

### 2.3 CLI 옵션

```text
--minimum-relative-nid-coverage 0.50
--minimum-relative-spatial-coverage 0.50
--coverage-penalty-weight 0.25
```

애플리케이션 기본값은 상대 NID/spatial gate `0.50 / 0.50`, coverage penalty `0.25`다.
Core 라이브러리의 기본 threshold와 penalty는 `0`으로 두어 단일-start 진단 및 기존
synthetic 호출의 의미를 보존한다. `--minimum-relative-edge-coverage`는 실제 정합
방향을 하드 탈락시킨 문제 때문에 제거됐으며, 이를 전달하면 명시적으로 오류를 낸다.

## 3. 실험 조건

입력은 기존 19일 CH1 3쌍 staging input이다.

```text
automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v3/input
```

공통 조건:

| 항목 | 값 |
|---|---|
| camera center | `(0.05928, -0.08105, 0) m` |
| intrinsic | Manual CH1 fixed profile |
| distortion | `raw` image + manual coefficients undistort |
| LDC | `false` |
| yaw | `-180~+175°`, 5° |
| down | `0~30°`, 5° |
| optical roll | `-15~+15°`, 5° |
| holdout | 0 (3쌍 모두 training; 본 실험은 후보 선택 진단) |

실행 결과는 다음 디렉터리에 보관했다.

```text
automatic_calibration/generated/ch1_20260819_coverage_legacy_v2/
automatic_calibration/generated/ch1_20260819_coverage_penalty_v2/
automatic_calibration/generated/ch1_20260819_coverage_gate50_v2/
automatic_calibration/generated/ch1_20260819_coverage_gate70_v2/
```

각 디렉터리의 `calibration_result.json`, `orientation_full_search.csv`,
`orientation_corrected_scores.csv`, `matching_scene_*.png`를 함께 확인한다.

## 4. coverage 설정별 결과

### 4.1 기존 방식: penalty/gate 없음

```text
minimum relative coverage = 0/0/0
coverage penalty = 0
```

| 항목 | 결과 |
|---|---:|
| status | `PASS` |
| selected yaw | `-115°` |
| selected down | `20°` |
| selected optical roll | `+10°` |
| final visible edge | `417` |
| layer maximum visible edge | `3374` |
| edge coverage | `0.124` |
| NID coverage | `0.476` |
| spatial coverage | `0.444` |
| mean edge distance | `8.70 px` |

평균 edge 거리는 좋지만, 실제로는 layer에서 가능한 support의 12.4%만 사용했다.
이 결과가 기존 false PASS의 재현이다.

### 4.2 penalty만 적용

```text
minimum relative coverage = 0/0/0
coverage penalty = 0.25
```

| 항목 | 결과 |
|---|---:|
| status | `PASS` |
| selected yaw | `+160°` |
| selected down | `20°` |
| selected optical roll | `0°` |
| final visible edge | `1898` |
| layer maximum visible edge | `3366` |
| edge coverage | `0.564` |
| NID coverage | `0.650` |
| spatial coverage | `0.889` |
| mean edge distance | `27.91 px` |

평균 edge 거리만 보면 기존 후보보다 나쁘지만, 훨씬 넓은 LiDAR 구조를 사용한다.
18일 RT의 yaw가 약 `+170°`였다는 점을 고려하면 방향 basin은 의도한 방향으로
이동했다.

### 4.3 상대 coverage 0.5 gate

```text
minimum relative coverage = 0.50/0.50/0.50
coverage penalty = 0.25
```

결과는 penalty-only와 동일한 `+160° / down 20° / roll 0°` 후보였다.

```text
edge coverage   = 0.564 >= 0.50
NID coverage    = 0.650 >= 0.50
spatial coverage= 0.889 >= 0.50
```

따라서 저coverage `-115°` 후보는 coarse 단계에서 제외되고, 오른쪽 방향의 basin이
선택됐다. 단, 최종 평균 edge 오차가 `27.91 px`로 높으므로 이 데이터만으로 제품 RT로
승격하지 않고 독립 hold-out 및 manual/기계 기준과 함께 판단한다.

### 4.4 상대 coverage 0.7 gate

```text
minimum relative coverage = 0.70/0.70/0.70
coverage penalty = 0.25
```

| 항목 | 결과 |
|---|---:|
| 최종 status | `FAIL: EDGE_ALIGNMENT_POOR` |
| 선택 후보 yaw | `+125°` |
| down/roll | `30° / -15°` (탐색 경계) |
| edge coverage | `0.754` |
| NID coverage | `0.714` |
| spatial coverage | `0.917` |
| mean edge distance | `75.43 px` |

coverage는 높지만 영상 엣지와의 실제 위치 정합이 나빠졌다. 현재 19일 장면에서는
`0.7`을 제품 기본값으로 사용하지 않는다. 이 결과는 coverage를 무조건 크게 만드는
것도 정답이 아니며, coverage와 위치 정합을 함께 gate해야 한다는 실험 근거다.

## 5. cross-epoch 회귀 확인

수정된 binary로 18일 RT를 19일 CH1 3쌍에 고정 적용했다.

산출물:

```text
automatic_calibration/generated/ch1_20260818_rt_on_20260819_fixed_coverage_v2/
```

| scene | status | visible/aligned | projected ratio | mean edge | geometry NID |
|---:|---|---:|---:|---:|---:|
| 0 | PASS | `509/419` | `0.8232` | `17.11 px` | `0.9320` |
| 1 | PASS | `508/395` | `0.7776` | `22.04 px` | `0.9435` |
| 2 | PASS | `498/361` | `0.7249` | `25.02 px` | `0.9128` |

고정 RT는 `3/3 PASS`로 유지됐다. 따라서 coverage 변경이 기존 cross-epoch 고정 검증을
깨뜨리지는 않았다.

## 6. 결론과 후속 조치

- 19일 기존 독립 후보가 잘못된 이유는 2D edge가 없어서가 아니라, 낮은 coverage의
  부분집합에 평균 점수가 끌려갔기 때문이다.
- coverage penalty와 0.5 relative gate는 선택 후보를 `-115°`에서 `+160°`로 이동시켜
  18일 RT(`약 +170°`)와 같은 방향 basin으로 유도했다.
- `0.7`은 support는 확보하지만 현재 장면에서 edge 정합을 희생해 FAIL했다. 현재
  권장 운영값은 0.5이며, 독립 hold-out에서 threshold sweep을 다시 확인해야 한다.
- 19일 3쌍을 모두 training으로 사용한 결과는 coverage 선택 실험의 진단으로만
  취급한다. 제품 RT 승인 전에는 training에 사용하지 않은 hold-out을 별도로
  확인해야 한다.
- 카메라가 벽과 평행하지 않은 것은 방향 모호성을 키우는 조건이지만, 해당 장면의
  edge/구조선 검출 자체가 실패한 것은 아니다. coverage gate는 비스듬한 설치를
  금지하지 않고, 일부 영역만 맞춘 후보를 제거하는 방식이다.

## 7. 수정 로그

| 날짜 | 변경 |
|---|---|
| 2026-08-20 | `Evaluation`에 3×4 영상 공간 occupancy 계산 추가 |
| 2026-08-20 | 같은 yaw layer 기준 edge/NID/spatial 상대 coverage 계산 추가 |
| 2026-08-20 | coverage penalty와 coarse/final 상대 coverage gate 추가 |
| 2026-08-20 | orientation CSV/JSON/scene validation에 coverage 진단값 추가 |
| 2026-08-20 | 8-neighbor 보정 점수에서 `overlap_valid=false` 후보 제외 |
| 2026-08-20 | 19일 CH1 coverage 0/0.25/0.5/0.7 실험 실행 |
| 2026-08-20 | 새 코드로 18일 RT→19일 고정 적용 `3/3 PASS` 회귀 확인 |
| 2026-08-20 | Manual K 고정 시 2-train/1-hold-out 경로를 허용하고 pair 3 독립 검증 실행 |

## 8. 2-train/1-hold-out 검증 (pair 3 제외)

사용자 요청에 따라 19일 CH1의 3쌍 중 **pair 1·2만 RT 추정에 사용하고 pair 3은
추정이 끝난 뒤 한 번만 검증**했다. 여기서 pair 번호는 실행 staging의
`image_0/scan_0`, `image_1/scan_1`, `image_2/scan_2` 순서이며, `image_2/scan_2`가
hold-out이다. 따라서 hold-out 영상의 edge/NID/구조선 점수는 후보 선택과 Ceres
refinement의 입력에 포함되지 않는다.

원본 pair 3은 `20260819-200910-CH1.jpg`와
`calib-20260819-200851_sweep-000001_pan_tilt_lidar.json`이다.

산출물:

```text
automatic_calibration/generated/ch1_20260819_coverage_gate50_holdout_pair3_v1/
```

실행 조건은 coverage 실험과 동일하고, 추가로 `--holdout-count 1`을 지정했다.
Manual CH1 내부 파라미터를 고정(`--allow-intrinsic-refinement false`)했기 때문에
2개의 training pair로 RT만 추정할 수 있도록 실행 경로를 수정했다. 내부 K와 RT를
동시에 추정하는 모드에서는 기존처럼 3개 이상의 관측이 필요하다.

### 8.1 판정 결과

| 구분 | 사용 목적 | scene 수 | PASS | 평균 visible/aligned | 평균 edge 거리 | 평균 geometry NID |
|---|---|---:|---:|---:|---:|---:|
| training pair 1~2 | RT 추정 + training gate | 2 | `2/2` | `565/473` | `16.29 px` | `0.8824` |
| hold-out pair 3 | 최종 검증만 | 1 | `1/1` | `557/482` | `15.14 px` | `0.8963` |

전체 결과는 `status=PASS`, `reason_code=PASS`였다. Hold-out의 세부 결과는
[`holdout_scene_validation.csv`](../generated/ch1_20260819_coverage_gate50_holdout_pair3_v1/holdout_scene_validation.csv),
training 결과는
[`training_scene_validation.csv`](../generated/ch1_20260819_coverage_gate50_holdout_pair3_v1/training_scene_validation.csv)에
기록되어 있다.

### 8.2 추정된 RT와 해석

선택된 coarse basin은 다음과 같다.

```text
yaw                  +155°
downward direction     +5°
optical roll            0°
refined camera-down     +6.705°
t_camera_lidar       (0.05155, 0.07864, 0.03531) m
```

두 training pair에서만 추정한 RT를 pair 3에 적용했을 때 edge projected ratio가
`0.8654`, geometry NID가 `0.8963`, 평균 edge 거리가 `15.14 px`로 유지됐다. 이는
현재 고정 설치·동일 조명 조건 안에서 pair 3으로의 재현성이 확인됐다는 뜻이다.

다만 hold-out이 한 쌍이고 동일 설치 epoch의 데이터이므로, 이 결과만으로 제품 RT를
최종 승격하지 않는다. 다음 단계는 설치를 다시 조정한 독립 epoch에서도 같은 방식으로
training/hold-out을 분리하고, 18일 RT 고정 적용 및 marker/jig 기준과 함께 비교하는
것이다.

## 9. 검증 실행 회귀

hold-out 실행에 사용한 binary를 Docker의 Ubuntu 환경에서 다시 빌드하고 전체 CTest를
실행했다.

```text
100% tests passed, 0 tests failed out of 5
Total Test time = 6.04 sec
```

## 10. 최신 코드 재검증 주의 (2026-08-20)

위 실험 표는 당시 coverage 실험 staging과 실행 조건의 결과다. 최신 커밋 `f92626e`에서
동일한 20260819 CH1 staging에 `--holdout-count 1`, relative NID/spatial coverage `0.5`,
penalty `0.25`를 다시 적용한 산출물은
`generated/resume_verify_20260820_20260819_ch1_coverage_holdout/`이다.

최신 재실행은 training `2/2`, hold-out `1/1`을 통과했지만 선택 yaw가 `-118°`로
남았다. 즉 coverage penalty/gate가 후보의 저coverage degeneracy를 줄이는 데는
도움이 되지만, 같은 고정환경 epoch 안에서 다른 basin을 완전히 제거하지는 못한다.
이 결과가 이전 표의 `+160°` 선택과 다르므로 두 결과를 하나의 재현성 근거로 합치지
않는다. 현재 canonical 판정은 [`INSTALLATION_EPOCH_REPRODUCIBILITY_20260818_20260819.md`](INSTALLATION_EPOCH_REPRODUCIBILITY_20260818_20260819.md)의
최신 코드 재실행 절을 따르며, coverage만으로 제품 RT를 승인하지 않는다.

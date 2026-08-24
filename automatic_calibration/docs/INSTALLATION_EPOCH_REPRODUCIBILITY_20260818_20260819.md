# 2026-08-18~19 강체 모듈 이동 재현성 및 제품 RT 승격 보류 분석

작성일: 2026-08-20  
대상 데이터: `repeat_test_sample/20260818`, `repeat_test_sample/20260819`  
대상 채널: CH1 우선, CH4 독립 판정  
상태: **8월 18일 RT cross-epoch PASS / 8월 19일 독립 후보 거절 / 자동 선택 안정화 필요**

## 1. 확정된 설치 조건

2026-08-18과 2026-08-19 사이에 카메라만 LiDAR로부터 분리해 옮긴 것이 아니다.
카메라와 LiDAR는 하나의 강체 모듈로 부착돼 있으며, 모니터암을 옆으로 이동하고 모듈
전체의 방향을 회전했다. 센서 사이의 브래킷과 상대 장착은 변경하지 않았다.

따라서 다음 두 종류의 변환을 구분해야 한다.

| 구분 | 8월 18일→19일 변화 |
|---|---|
| 모듈의 방/세계 좌표 위치·방향 | 변경됨 |
| 카메라–LiDAR 상대 외부 파라미터 `T_camera_lidar` | 변경되면 안 됨 |
| LiDAR 좌표계에서 본 카메라 중심 `C_lidar` | 변경되면 안 됨 |

카메라와 LiDAR에 동일한 강체 이동 `G`가 적용되면 세계 좌표에서 두 센서의 pose는
달라지지만 상대 변환에서는 `G`가 소거된다.

```text
T_camera_lidar(19)
  = inverse(T_world_camera(19)) * T_world_lidar(19)
  = inverse(G * T_world_camera(18)) * (G * T_world_lidar(18))
  = T_camera_lidar(18)
```

따라서 8월 19일은 새로운 제품 RT를 만들어야 하는 별도 장착 상태가 아니라, 동일한
제품 RT가 모듈의 세계 pose 변화에도 재현되는지 확인하는 cross-epoch 시험이다.

## 2. Camera-center 기계값의 의미와 적용

현재 기계 모델에서 사용하는 값은 다음과 같다.

```text
C_lidar = (0.05928, -0.08105, 0) m
```

LiDAR scan frame이 `+x right, +y down, +z forward`일 때 다음 의미다.

| 성분 | 의미 |
|---|---|
| `x=+0.05928 m` | 카메라 광학 중심이 LiDAR 원점보다 오른쪽으로 59.28 mm |
| `y=-0.08105 m` | 카메라 광학 중심이 LiDAR 원점보다 위로 81.05 mm |
| `z=0 m` | 앞뒤 offset을 0으로 둔 기계 모델 |

센서 중심 사이의 직선거리는 약 `100.4 mm`다. 이 값은 최종 RT JSON의 translation
`t` 자체가 아니라 LiDAR frame에서 본 카메라 광학 중심이다.

```text
t_camera_lidar = -R_camera_lidar * C_lidar
```

모듈 전체를 이동·회전해도 `C_lidar`는 유지해야 한다. 따라서 8월 19일 실행에서도
브래킷이 유지됐다는 전제하에 이 vector를 사용해야 한다.

## 3. 현재 실행 조건의 불일치

### 3.1 8월 18일 CH1

산출물:

```text
automatic_calibration/generated/ch1_20260818_four_pair_recheck_v2/
```

주요 조건과 결과:

| 항목 | 값 |
|---|---:|
| camera center | `(0.05928, -0.08105, 0) m` |
| training | `3/3 PASS` |
| hold-out | `1/1 PASS` |
| yaw | 약 `170°` |
| down | 약 `19.999°` |
| optical roll | `0°` |
| 추정 translation 크기 | 약 `100.4 mm` |

### 3.2 8월 19일 CH1

산출물:

```text
automatic_calibration/generated/repeat_test_sample_20260819_install_shift_ch1_v3/
```

주요 조건과 결과:

| 항목 | 값 |
|---|---:|
| camera center | 미입력 |
| baseline | `0 m` |
| training | `3/3 PASS` |
| hold-out | 없음 |
| yaw | 약 `-120°` |
| down | 약 `16.285°` |
| optical roll | `+10°` |
| 추정 translation 크기 | 약 `7.1 mm` |

8월 19일 실행의 `baseline=0`은 기존 설치 prior를 제거한 중립 진단에는 사용할 수 있지만,
실제 강체 모듈의 기계 구조를 반영한 8월 18일 결과와 동일 조건으로 비교할 수 없다.
특히 실제 센서 중심 거리가 약 100.4 mm인데 선택 RT의 translation 크기가 약 7.1 mm인
점은 물리 모델과 일치하지 않는다.

## 4. 날짜별 Automatic RT 차이

현재 저장된 두 CH1 RT를 직접 비교하면 다음 차이가 난다.

| 지표 | 차이 |
|---|---:|
| rotation geodesic | `75.113°` |
| translation vector norm | `97.331 mm` |

강체 브래킷이 유지됐다면 이 차이는 실제 모듈의 세계 pose 변화로 설명할 수 없다.
`T_camera_lidar`는 세계 pose와 무관하기 때문이다. 현재 차이는 주로 다음 항목을 의심해야
한다.

1. 8월 18일과 19일의 camera-center 조건 불일치
2. 반복·대칭 구조에서 다른 orientation basin을 선택한 local minimum
3. 8월 19일의 독립 hold-out 부재
4. image–scan pairing의 외부 확인 정보 부족
5. 장면마다 유효한 2D–3D 구조선/NID overlap 차이

따라서 8월 19일 CH1의 내부 `PASS`는 제품 RT 재현 성공을 뜻하지 않는다.

## 5. ArUco/ChArUco가 검증하는 범위

8월 18일 영상에서는 A4 ChArUco가 완전히 검출됐다.

| 항목 | 결과 |
|---|---:|
| marker | `17/17` |
| corner | `24/24` |
| reprojection RMSE | 약 `1.24~1.33 px` |

이 검출로 직접 얻는 값은 다음과 같다.

```text
T_camera_marker_board
```

이는 카메라의 board pose 및 카메라 내부 파라미터/왜곡 보정 상태를 확인하는 유효한
카메라 측 기준이다. 그러나 이미지에서 marker가 보이는 것만으로 다음 값은 생기지 않는다.

```text
T_lidar_marker_board
```

평평한 종이에 인쇄된 흑백 패턴은 일반적으로 LiDAR depth에서 식별 가능한 3D 요철이나
경계를 만들지 않는다. 현재 Automatic Calibration도 ArUco ID와 corner를 직접 사용하지
않고 camera gradient, Canny edge, LSD 구조선을 사용한다. 따라서 영상의 marker line이
강하게 검출돼도 대응되는 LiDAR 구조가 없으면 다른 벽·책상·바닥 edge와 잘못 대응할 수
있다.

ArUco를 절대 RT 기준으로 사용하려면 다음 관계가 완성돼야 한다.

```text
T_camera_lidar_reference
  = T_camera_marker_board * inverse(T_lidar_marker_board)
```

`T_lidar_marker_board`는 다음 중 하나로 독립 취득해야 한다.

- LiDAR에서 외곽과 pose를 식별할 수 있는 두께 있는 rigid target
- 정밀 CAD/jig 및 실측 좌표
- LiDAR-visible board frame이나 별도 3D 기준점
- 품질이 검증된 screen/board plane과 외곽선 pose 추정

현재 marker 검출 PASS는 카메라 측 pose 품질을 증명하지만, 단독으로
`T_camera_lidar`의 절대 정확도를 증명하지는 않는다.

## 6. 제품 RT로 승격하지 않는 직접 사유

현재 제품 RT 승격을 보류하는 이유는 다음과 같다.

1. 동일 강체 모듈인데 8월 18일과 19일 CH1 RT가 `75.113° / 97.331 mm` 차이 난다.
2. 두 날짜에 서로 다른 camera-center 조건을 사용해 동일 조건 재현성 시험이 아니다.
3. 8월 19일 CH1은 3개 장면을 모두 추정에 사용했고 독립 hold-out이 없다.
4. ArUco 검출값이 독립적인 `T_lidar_marker_board`와 연결되지 않았다.
5. 8월 18일 PASS도 같은 설치·같은 공간의 내부 gate 및 1개 hold-out PASS이며 절대
   ground truth PASS는 아니다.
6. 8월 19일 CH4는 `1/3 PASS`로 per-scene 품질 게이트 자체를 통과하지 못했다.

즉 현재 문제는 단순히 데이터 개수가 적다는 것만이 아니다. 물리적으로 동일해야 하는
RT가 날짜 간 반복되지 않았으며, 어떤 결과가 정답인지 판별할 독립 기준이 아직 완성되지
않았다.

## 7. 수정·검증 우선순위

### 7.1 8월 19일 CH1 동일 조건 재실행

8월 18일과 동일하게 다음 조건을 적용한다.

```text
camera center = (0.05928, -0.08105, 0) m
LDC = false
동일 CH1 intrinsic + distortion
raw image undistort
yaw/down/roll 탐색 범위와 간격 동일
동일 scene 품질 gate
```

이 실행은 8월 19일 `baseline=0` 결과를 대체하는 동일 조건 재현성 시험이다.

### 7.2 8월 18일 RT의 8월 19일 고정 적용

8월 18일 RT를 최적화하지 않고 8월 19일 CH1의 모든 유효 pair에 고정 적용한다.

- 통과: 같은 RT가 다른 세계 pose에서도 유지됨을 지지
- 실패: RT 재현 실패이며 pairing, 좌표계, intrinsic, 목적함수 또는 기계 모델을 점검

반대로 8월 19일 재추정 RT를 8월 18일 hold-out에 고정 적용하는 양방향 검증도 수행한다.

### 7.3 독립 hold-out 확보

8월 19일 CH1은 최소 한 개의 추가 image–scan pair를 확보하고, 해당 pair를 RT 탐색에
사용하지 않은 고정 hold-out으로 둔다. CH4는 별도 채널로 계속 독립 판정한다.

### 7.4 ArUco 기준 확장

현재 A4 marker는 camera-side pose에 사용하고, LiDAR에서도 pose가 결정되는 target 또는
실측 `T_lidar_marker_board`를 추가한다. 그 후 Automatic RT와 rotation/translation 오차를
직접 비교한다.

## 8. 2026-08-20 동일 조건 실행 결과

### 8.1 8월 18일 RT를 8월 19일에 고정 적용

8월 18일 CH1 RT를 후보 탐색이나 Ceres 재최적화 없이 8월 19일 세 pair에 그대로
적용했다.

산출물:

```text
automatic_calibration/generated/ch1_20260818_rt_on_20260819_fixed_v1/
```

| scene | 통과 | visible/aligned edge | projected ratio | mean edge | geometry NID | 구조선 match | 수직 오차 |
|---:|---|---:|---:|---:|---:|---:|---:|
| 0 | PASS | `509/419` | `0.8232` | `17.11 px` | `0.9320` | `16/28` | `8.73°` |
| 1 | PASS | `508/395` | `0.7776` | `22.04 px` | `0.9435` | `15/29` | `8.69°` |
| 2 | PASS | `498/361` | `0.7249` | `25.02 px` | `0.9128` | `14/21` | `9.32°` |

결과는 `3/3 PASS`다. 이는 8월 18일 RT가 모듈 전체를 옮기고 회전한 8월 19일
장면에서도 유지된다는 cross-epoch 재현성 근거다.

### 8.2 8월 19일 동일 기계조건 독립 재추정

8월 19일 세 pair에 8월 18일과 동일한 조건을 적용했다.

```text
camera center = (0.05928, -0.08105, 0) m
yaw = 360° / 5° step
down = 0~30° / 5° step
optical roll = -15~15° / 5° step
direction prior weight = 0
```

산출물:

```text
automatic_calibration/generated/ch1_20260819_same_mechanical_5deg_v1/
```

내부 결과는 `3/3 PASS`였지만 다음의 다른 orientation basin을 선택했다.

| 항목 | 8월 18일 RT | 8월 19일 독립 RT |
|---|---:|---:|
| yaw | 약 `+170°` | `-115°` |
| down | `19.999°` | `20.156°` |
| optical roll | `0°` | `+10°` |
| rotation 차이 | 기준 | `79.542°` |
| translation 차이 | 기준 | `83.340 mm` |

두 실행 모두 동일 camera-center를 사용했으므로 translation 크기는 기계 baseline을
따르지만, 회전이 다르면 `t=-R*C`의 방향도 달라진다. 따라서 translation 차이는 별도
센서 이동이 아니라 잘못 선택된 회전 basin의 결과다.

### 8.3 서로 다른 RT가 모두 PASS한 원인

8월 19일 장면에서 두 RT의 support를 비교하면 다음 차이가 난다.

| 지표 | 8월 18일 고정 RT | 8월 19일 독립 RT |
|---|---:|---:|
| visible edge/scene | `498~509` | `110~165` |
| NID projected/scene | `753~784` | `423~480` |
| visible 구조선/scene | `21~29` | `12~15` |
| mean edge distance | `17.11~25.02 px` | `6.80~11.35 px` |

8월 19일 독립 후보는 훨씬 적은 LiDAR edge와 구조만 화면에 남긴 뒤, 그 작은 부분집합의
edge distance를 낮췄다. 현재 gate는 visible edge가 절대 최소값 `100` 이상이면
통과할 수 있어 `110~165`개의 저coverage 후보를 제거하지 못한다. projected ratio도
선택된 visible subset을 분모로 계산하므로 높은 값만으로 넓은 장면 coverage를 보장하지
않는다.

즉 현재 목적함수와 gate는 다음 degeneracy를 허용한다.

```text
잘못된 방향
  → 화면에 투영되는 LiDAR 구조 수 감소
  → 우연히 가까운 일부 edge만 유지
  → 평균 edge 오차 감소
  → 내부 PASS
```

### 8.4 역방향 고정 RT 검증

8월 19일 독립 RT를 8월 18일 네 pair에 고정 적용했다.

산출물:

```text
automatic_calibration/generated/ch1_20260819_rt_on_20260818_fixed_v1/
```

결과는 `0/4 FAIL`이다. 네 scene 모두 visible edge가 `0`이었고 다음 주요 실패가
발생했다.

- `EDGE_VISIBLE_INSUFFICIENT`
- `EDGE_OVERLAP_INSUFFICIENT`
- `EDGE_ALIGNMENT_POOR`
- `NID_OVERLAP_INSUFFICIENT`
- `NID_SPATIAL_ENTROPY_INSUFFICIENT`

반면 8월 18일 RT는 8월 19일에서 `3/3 PASS`했다. 따라서 두 내부 PASS 후보 중
cross-epoch 일반화가 확인된 것은 8월 18일 RT뿐이다.

### 8.5 최종 판정

- **8월 18일 RT:** 현재 강체 모듈의 우선 운영 후보. 8월 18일 training `3/3`,
  hold-out `1/1`, 8월 19일 cross-epoch `3/3`을 통과했다.
- **8월 19일 독립 RT:** 내부 training `3/3` 표시는 났지만 역방향 `0/4 FAIL`이므로
  거절한다.
- **Automatic 선택 로직:** 동일 모듈에서 약 `79.5°` 다른 후보를 다시 선택했으므로,
  coverage-aware gate를 추가하기 전에는 자동 산출 RT를 무조건 제품 RT로 활성화할 수
  없다.

필요한 코드 수정 방향은 후보별 visible edge/NID/구조선의 절대 support를 같은 장면에서
관측된 최대 support와 비교하고, coverage가 충분한 후보끼리만 edge/NID 점수를 비교하는
것이다. 평균 edge distance가 낮더라도 support를 크게 줄인 후보는 선택 대상에서
제외해야 한다.

## 9. 승인 판단 기준

제품 RT 승격은 최소한 다음 조건을 모두 만족한 뒤 수행한다.

- 8월 18일과 19일을 동일한 기계 prior 및 camera profile로 처리
- 날짜별 training gate PASS
- 날짜별 독립 hold-out PASS
- 8월 18일 RT를 19일에 고정 적용한 cross-epoch 검증 PASS — **현재 충족**
- 독립 재추정 RT가 cross-epoch 역방향 검증도 통과 — **현재 미충족**
- 두 날짜 RT 차이가 사전에 정의한 rotation/translation 허용오차 이내
- 독립 marker/jig 기준과의 오차가 제품 요구사항 이내

허용오차 수치는 현재 문서에서 임의로 완화하지 않는다. 실제 top-view 활용 요구와 설치
공차를 기준으로 제품 요구사항에서 별도로 확정한다.

## 10. 수정 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-20 | 분석 범위를 2026-08-18/19 데이터로 제한하고 날짜별 결과 비교 작성 |
| 2026-08-20 | 카메라와 LiDAR가 강체 모듈로 함께 이동·회전했다는 설치 조건 반영 |
| 2026-08-20 | camera-center 유지 조건, ArUco 검증 한계 및 제품 RT 승격 보류 사유 기록 |
| 2026-08-20 | 8월 18일 RT→19일 `3/3 PASS`, 19일 독립 RT→18일 `0/4 FAIL` 양방향 검증 기록 |
| 2026-08-20 | 저coverage 후보가 평균 edge 오차를 낮춰 내부 PASS한 selection degeneracy 분석 추가 |

## 11. 2026-08-20 coverage 기반 후보 선택 수정

상대 edge/NID/영상 공간 coverage와 coverage penalty를 구현하고, 같은 19일 CH1 입력으로
coverage 미사용·penalty-only·0.5 gate·0.7 gate를 비교했다. 새 코드로 18일 RT의 19일
고정 적용 `3/3 PASS`도 회귀 확인했다.

코드 변경, CLI 옵션, 결과 표와 산출물 경로는
[`COVERAGE_SUPPORT_EXPERIMENT_20260820.md`](COVERAGE_SUPPORT_EXPERIMENT_20260820.md)에
상세히 기록했다. 현재 권장 실험값은 relative coverage `0.5`, penalty `0.25`이며,
제품 RT 승격 전 독립 hold-out에서 다시 검증한다.

## 12. 2026-08-20 최신 코드 재실행 결과

최신 커밋 `f92626e`를 Docker Ubuntu 빌드에서 재실행했다. 8월 18일은 네 쌍 중
세 쌍을 training, 한 쌍을 hold-out으로 사용했고, 8월 19일은 staging한 CH1 세 쌍 중
두 쌍을 training, 마지막 쌍을 hold-out으로 사용했다. 두 실행 모두 Manual CH1
`K+D`, `LDC=false`, raw image undistort, camera center
`(0.05928,-0.08105,0) m`, yaw 5°, down 0~30°/5°, optical roll ±15°/5° 조건이다.

### 12.1 최신 독립 재추정

| epoch | 산출물 | 선택 RT | training | hold-out | 상태 |
|---|---|---|---:|---:|---|
| 20260818 | `generated/resume_verify_20260820_20260818/` | yaw `169°`, down `21°`, roll `3°` | 3/3 | 1/1 | `CANDIDATE_RT` |
| 20260819 | `generated/resume_verify_20260820_20260819_ch1/` | yaw `-118°`, down `22°`, roll `13°` | 2/2 | 1/1 | `CANDIDATE_RT` |

두 RT의 회전 geodesic 차이는 `76.445°`, translation 차이 norm은 `78.25 mm`다. 강체
모듈 전체만 이동·회전했고 센서 상대 장착이 유지됐다면 이 차이는 실제 `T_camera_lidar`
변화로 해석할 수 없다.

### 12.2 양방향 fixed-RT 교차 검증

- 8월 18일 RT를 8월 19일에 고정 적용:
  `generated/resume_verify_20260820_20260819_fixed_0818_v2/` → **3/3 PASS**.
  각 장면에서 visible/aligned edge는 `516/427`, `516/403`, `509/375`였고, 평균
  edge 거리는 `16.53`, `21.47`, `23.26 px`였다.
- 8월 19일 독립 RT를 8월 18일에 고정 적용:
  `generated/resume_verify_20260820_20260818_fixed_0819_v2/` → **0/4 FAIL**.
  네 장면 모두 visible edge가 0이며 `EDGE_VISIBLE_INSUFFICIENT`,
  `EDGE_OVERLAP_INSUFFICIENT`, `NID_OVERLAP_INSUFFICIENT` 등이 발생했다.

따라서 현재 데이터에서 cross-epoch 일반화가 확인된 후보는 8월 18일 RT뿐이며, 8월
19일 독립 후보는 자동 선택의 대체 basin으로 거절한다.

### 12.3 coverage 옵션 재검증

상대 NID/공간 coverage `0.5`, coverage penalty `0.25`, 2-train/1-hold-out을 최신
코드로 재실행한 산출물은
`generated/resume_verify_20260820_20260819_ch1_coverage_holdout/`이다. 이 실행도
`CANDIDATE_RT`와 training `2/2`, hold-out `1/1`을 기록했지만 선택 RT는 여전히
`yaw=-118°/down=22°/roll=13°`였다. 따라서 coverage gate만으로는 동일 epoch의
hold-out을 통과하는 잘못된 basin을 제거하지 못한다. 현재 제품 경로에서는
cross-epoch fixed-RT 검증과 독립 기준이 필수이며, coverage 결과만으로 RT를 승격하지
않는다.

### 12.4 시각 확인 파일

- 8월 19일 독립 후보(부분 영역에 치우친 투영):
  `generated/resume_verify_20260820_20260819_ch1_coverage_holdout/matching_scene_0.png`
- 8월 18일 RT 고정 적용(전체 LiDAR 구조 투영):
  `generated/resume_verify_20260820_20260819_fixed_0818_v2/debug/scene_0/06_projection_final.png`

최종 상태는 두 경우 모두 `activation_allowed=false`이며, 제품 RT 승격은 보류한다.

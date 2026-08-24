# Automatic Calibration 테스트 진행 및 실패 분석 리포트

- 문서 상태: 실데이터 검증 중간 결론
- 작성일: 2026-08-07
- 최종 수정일: 2026-08-18
- 적용 범위: Calibration Core, Stanford 합성 검증, real session-001~003 및 고정환경 130333/20260818 CH1
- 최종 판정: **합성 검증 PASS, CH1 v11 내부 gate PASS, 제품 RT/conformance 승인 보류**

> 2026-08-20 이후의 제품 운용 기준은 [`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)다.
> 이 리포트의 제조사 FOV K·K+RT 공동 추정 관련 문장은 과거 실험 이력으로 보존하며,
> 현재 MVP는 Manual ChArUco `K+D` 고정과 `R,t` 전용 추정을 사용한다.

## 1. 목적

본 문서는 지금까지 수행한 2D camera–3D pan-tilt LiDAR 자동 외부 파라미터 calibration 테스트를 정리하고, 실데이터에서 matching이 실패한 원인을 입력 데이터, 센서 취득, 좌표계 및 알고리즘 관점으로 분리한다. 또한 다음 반복시험에서 무엇을 고정하고 무엇을 변경해야 하는지 결정 기준을 제공한다.

이 문서에서 `PASS`는 수치 게이트만 통과했다는 뜻이 아니라 재투영 영상의 물리적 대응, 반복 측정 재현성 및 독립 검증까지 만족하는 경우로 제한한다. 이 기준에 따라 session-003의 200 bps 01 수치 PASS는 false positive로 재분류한다.

## 2. 현재 구현 범위

현재 Automatic Calibration Core에는 다음 기능이 구현되어 있다.

- Organized pan-tilt range image와 3D point cloud 입력
- Camera Gaussian/Canny edge 및 edge distance transform 생성
- LiDAR 인접 셀 range discontinuity edge 추출
- Mechanical prior 기반 camera FoV 사전 선택
- Ceres 기반 6-DoF 단일/다중 장면 최적화
- 유효 edge 수, 투영률, 평균 edge 거리 및 prior 이동량 품질 게이트
- 실패 시 기존 mechanical prior 유지
- 결과 JSON 및 실제 데이터 2D–3D 재투영 비교 이미지 출력
- PLY/OBJ 기반 point cloud 진단 출력

실제 실행기는 JPG와 PNG, 단일 관측, LiDAR frame의 3축 camera center 입력을 지원하도록 보완했다. OpenSDK/CV5 이식과 Jenkins 연동은 본 실데이터 정확도 문제와 분리하여 진행한다.

## 3. 테스트 진행 요약

### 3.1 빌드 및 단위 테스트

2026-08-06~07 기준 workspace 전체 CTest 5개가 통과했다.

| 테스트 | 결과 |
|---|---|
| Automatic synthetic LiDAR | PASS |
| Automatic Calibration Core | PASS |
| Manual marker calibration | PASS |
| Top-view core | PASS |
| Top-view GUI smoke | PASS |

이는 코드 경로가 실행되고 정의된 단위 동작을 만족한다는 뜻이며, 실제 센서 extrinsic 정확도를 보증하지 않는다.

### 3.2 Stanford 2D-3D-Semantics 합성 검증

| 구분 | 결과 | 주요 수치 |
|---|---|---|
| 단일 장면 | FAIL | 최종 회전 오차 2.306°, 이동 오차 61.7 mm로 50 mm 기준 초과 |
| 다중 5장면 | PASS | 회전 오차 0.789°, 이동 오차 39.3 mm, projected ratio 68.7% |

합성 다중 장면은 알려진 ground truth와 정확한 intrinsic/depth를 사용하므로 Core의 기본 최적화 가능성은 확인됐다. 반면 단일 장면은 목적함수가 감소하고 Core 내부 게이트가 통과했어도 ground-truth 이동 오차가 기준을 초과했다. 이는 한 장면의 edge 비용만으로 translation이 약하게 관측된다는 초기 경고다.

### 3.3 Real session-001

- 입력: PNM-C16083RVQ 이미지 2장, LiDAR scan 2개
- 실행: 기본 후보 및 heading/roll/down 방향 탐색 총 44개 결과
- 결과: 44개 전부 FAIL
- 실패 분포: `EDGE_ALIGNMENT_POOR` 32개, `OVERLAP_INSUFFICIENT` 12개
- 최선 후보: 평균 edge 거리 65.44 px, projected ratio 51.47%, LiDAR edge 1,358개

최적화는 수렴했지만 실제 영상 구조와 LiDAR 깊이 경계가 일치하지 않았다. 수학적 solver convergence와 올바른 물리 정합은 별개임을 확인했다.

### 3.4 Real session-002

- 입력: LiDAR scan 1개와 임시 Galaxy S25 사진 3장
- 조건: 센서를 바닥에 두고 천장을 관측, 사진은 수동 회전 후 재시험
- 결과: `OVERLAP_INSUFFICIENT`
- 이미지 회전 전: 평균 edge 거리 195.36 px, projected ratio 17.95%
- 이미지 회전 후: 평균 edge 거리 176.59 px, projected ratio 16.16%
- 전체 유효 point 약 30,960개 중 영상 내부 투영 약 8,195개(26.5%)

추가 분석 결과 사진 사이 영상 모서리 이동량이 약 101~181 px로 나타났다. 동일 위치에서 촬영했더라도 카메라 자세가 달랐으므로 하나의 고정 `T_camera_lidar`를 세 사진에 공동 적용할 수 없다. 같은 LiDAR scan을 복제해 서로 다른 카메라 자세의 사진과 묶은 입력은 smoke test로만 유효하며 calibration 입력으로는 무효다.

JSON에는 scan pan 범위 0~359.1°와 mechanism pan 범위 0~179°가 함께 존재했다. 이 불일치는 producer의 각도 생성 또는 metadata 계약 확인 전까지 좌표계 신뢰도를 제한한다.

### 3.5 Real session-003 200/400 bps

- LiDAR: 책상 위에서 천장을 향해 scan
- Camera: LiDAR보다 약 16 cm 위에서 촬영
- 사진: 4개 각도, 두 속도 폴더에 byte-identical 사진 사용
- 1~3번 JPEG: EXIF orientation 180°
- 4번 JPEG: 정상 orientation
- OpenCV: EXIF orientation을 자동 반영하므로 raw pixel 뒤집힘은 calibration 입력에서 정규화됨

#### LiDAR 취득 비교

| 항목 | 200 bps | 400 bps |
|---|---:|---:|
| 측정시간 | 1,226.4 s | 616.9 s |
| 유효 point | 31,044 | 30,987 |
| 유효률 | 99.50% | 99.32% |
| 평균 signal strength | 30,830.5 | 30,979.7 |
| 평균 range | 1.974 m | 1.984 m |

동일 organized cell 30,987개의 속도 간 range 비교 결과는 다음과 같다.

| 지표 | 결과 |
|---|---:|
| Pearson correlation | 0.871 |
| 절대 range 차이 평균 | 14.1 cm |
| 절대 range 차이 중앙값 | 7.0 cm |
| 절대 range 차이 90백분위 | 27.0 cm |
| 5 cm 이내 일치율 | 39.7% |

정지 장면임에도 속도 조건에 따라 point cloud 차이가 크다. Calibration 전에 actuator angle/time alignment, 방향 전환 backlash, scan 중 진동, sample 병합 및 producer 좌표화를 별도 검증해야 한다.

#### 동일 후보 RT의 사진별 matching 결과

| 속도 | 사진 | 계산상 판정 | 평균 edge 거리 | projected ratio | 최종 판정 |
|---|---|---|---:|---:|---|
| 200 bps | 01 | PASS | 35.34 px | 67.3% | **FAIL / false positive** |
| 200 bps | 02 | OVERLAP_INSUFFICIENT | 182.10 px | 14.4% | FAIL |
| 200 bps | 03 | OVERLAP_INSUFFICIENT | 238.95 px | 17.6% | FAIL |
| 200 bps | 04 | EDGE_ALIGNMENT_POOR | 49.18 px | 54.3% | FAIL |
| 400 bps | 01 | EDGE_ALIGNMENT_POOR | 47.67 px | 66.9% | FAIL |
| 400 bps | 02 | OVERLAP_INSUFFICIENT | 202.97 px | 12.4% | FAIL |
| 400 bps | 03 | EDGE_ALIGNMENT_POOR | 201.29 px | 31.6% | FAIL |
| 400 bps | 04 | EDGE_ALIGNMENT_POOR | 67.74 px | 48.6% | FAIL |

200 bps 01은 초기 35.362 px에서 최종 35.338 px로 0.024 px, 약 0.07%만 개선됐다. 재투영 영상에서는 동일 물리 경계에 정렬되지 않았으며 같은 자세의 400 bps scan에서도 재현되지 않았다. 따라서 활성 calibration으로 사용할 수 없고 `FAIL / VISUAL_ALIGNMENT_REJECTED` 또는 `FAIL / FALSE_ALIGNMENT`로 다뤄야 한다.

### 3.6 고정환경 2026-08-11 130333 천장 설치 재시험 (폐기된 초기 해석)

> 이 절은 당시 실행 기록 보존용이다. `camera center Y=+0.08305 m`와
> `tilt_zero=forward` 해석은 이후 정정되었으며 현재 실행 기준으로 사용하지 않는다.

실제 설치는 카메라가 아래를 보고 LiDAR의 `tilt=0°`가 수평 벽,
`tilt=-90°`가 바닥을 향한다. 기존 실행은 `prior-roll=0°`여서 LiDAR의
바닥축 `+Y`를 카메라 optical 전방 `+Z`로 옮기는 90° 회전이 누락됐다.
최적화 허용 범위가 초기값 주변 ±5°이므로 이 오류는 solver가 복구할 수 없다.

다음 조건으로 다시 계산했다.

- camera downward prior: `roll=+90°`
- camera center Y: `+0.08305m`(당시 잘못 적용; 현재 기준은 `-0.08105m`)
- 채널별 X 부호와 heading 탐색
- 표시용 point cloud: `viewer_z_up=(lidar_x,lidar_z,-lidar_y)`

| 채널 | camera center X | heading | 평균 edge 거리 | projected ratio | 판정 |
|---|---:|---:|---:|---:|---|
| CH1 | +0.05928m | 140° | 44.74 px | 51.64% | FAIL / EDGE_ALIGNMENT_POOR |
| CH2 | +0.05928m | 100° | 102.72 px | 21.99% | FAIL / EDGE_ALIGNMENT_POOR |
| CH3 | -0.05928m | 120° | 28.51 px | 77.20% | 기존 PASS / **false positive** |
| CH4 | -0.05928m | -55° | 50.70 px | 44.15% | FAIL / EDGE_ALIGNMENT_POOR |

CH3의 수치 PASS는 이후 3D top-view에서 실제 촬영 방향과 반대 영역에 영상이
입혀진 것이 확인되어 폐기했다. CH1·2·4도 채널별 실제 렌즈 optical center와
optical axis 확인, 독립 frame 재투영 및 구조 edge가 풍부한 추가 장면이 필요하다.

### 3.7 CH3 반대 방향 재검증 및 false PASS 차단

2026-08-11 업데이트된 LiDAR ICD에서 `lidar_scan`은 `+X=right`, `+Y=down`,
`+Z=forward`, 거리 단위는 m이며 실제 range는
`distance_m + sensor.range_offset_m`로 정의됐다. 130333 입력은 구형 schema 1.1로
`sensor.range_offset_m`가 없으므로 이번 replay에서 확인된 84 mm를
`--legacy-range-offset-m 0.084`로 명시했다. 새 schema 입력은 헤더 값이 없으면
실행을 거절하도록 변경했다.

사용자가 제공한 3D 투영 화면과 실제 top-view 방향을 비교한 결과 CH3의 실제
영상 방향은 기존 120° 후보와 약 180° 반대였다. 동일 입력에 기존 120°와 반대
후보 -60°를 다시 적용한 결과는 다음과 같다.

| CH3 후보 | 초기 평균 edge 거리 | 최종 평균 edge 거리 | 개선률 | projected edge ratio | 판정 |
|---|---:|---:|---:|---:|---|
| 120° | 28.5003 px | 28.5173 px | -0.0596% | 76.13% | **FAIL / OBJECTIVE_IMPROVEMENT_INSUFFICIENT** |
| -60° | 157.4902 px | 138.0899 px | 12.318% | 15.76% | **FAIL / OVERLAP_INSUFFICIENT** |

120°는 평균 거리가 오히려 증가했는데도 이전 절대 기준만으로 PASS가 됐던
false alignment다. Core와 실제 실행기에 최소 objective 개선률 5% 게이트를
추가해 같은 결과가 다시 PASS되지 않도록 수정하고 회귀 테스트를 추가했다.

-60°는 사용자가 확인한 물리적 방향 후보지만 LiDAR depth edge 330개 중 영상
안에서 유효하게 평가된 점이 52개뿐이고 평균 거리도 138.09 px다. 따라서 현재
자료만으로 정답 RT로 채택할 수 없다. 또한 현재 `prior-roll=90°`는 광축을 LiDAR
`+Y`(수직 아래)로 둔다. 이 조건에서 heading 180° 변경은 광축 방향 변경이 아니라
영상 평면의 180° 회전에 가깝다. PNM-C16083RVQ 개별 렌즈 모듈이 실제로 수직
아래가 아닌 사선 방향을 본다면 채널별 optical tilt/axis가 별도로 필요하거나,
이를 찾기 위한 360° yaw multi-start는 2026-08-11 추가했다. yaw로 설명되지 않는 렌즈별
optical tilt까지 자동 탐색하는 단계는 아직 남아 있다.

### 3.8 Manual K+D profile 고정 (현재 제품 경로)

기존 130333/CH3 replay에서 제조사 FOV K를 사용한 결과는 과거 진단 기록으로 유지한다.
현재 제품 경로는 동일 channel/resolution/zoom/focus의
`manual_calibration/output/<session>/intrinsic/camera_intrinsic.json`에서 ChArUco로
측정한 `K + distortion`을 읽고 고정한다. raw image이면 같은 profile로 undistort한 뒤
`R,t` 외부 파라미터만 탐색한다.

설치 장소가 바뀌어도 광학 profile이 같으면 K/D는 재사용할 수 있지만, zoom/focus/LDC/
해상도/채널이 바뀌면 새 profile이 필요하다. 제품 승인 실행에서
`--allow-intrinsic-refinement true`는 사용하지 않는다. profile이 없거나 상태가
불명확하면 pose 후보를 진단할 수는 있지만 결과는 `DIAGNOSTIC_ONLY`이며 활성 RT로
승격하지 않는다.

### 3.9 2026-08-11 변경사항 재검토 및 현재 이슈 (2026-08-12 기록)

2026-08-11에 반영한 변경 및 검증 결과는 다음과 같다.

- `prior-roll=+90°`를 천장 설치 카메라의 수직 하향 mechanical prior로 추가했다. 이는 실제 채널별 optical axis 측정값이 아니라, 카메라가 아래를 본다는 설치 설명을 LiDAR `+Y`(down)와 camera `+Z`(forward)에 연결한 공통 가정이다.
- 360° yaw coarse multi-start 간격을 45°에서 15°로 변경했다. 실제 실행 시 후보는 8개에서 24개로 증가하며, 선택된 후보는 이후 Ceres가 연속 각도로 refinement한다.
- `--debug-output`을 추가해 입력 이미지, camera gradient, edge distance, LiDAR surface-normal PLY, 초기/최종 투영 및 edge 투영을 단계별로 저장한다.
- 사람이 포함된 091522 데이터를 사용한 3세트 smoke test는 `MULTISTART_AMBIGUOUS`로 거절했다. 동일 CH1 이미지를 여러 LiDAR scan에 재사용했으므로 최종 calibration 근거로 사용하지 않는다.
- 사람이 없는 130333 데이터는 CH1~CH4를 재실행했으나 채널별 image–LiDAR 쌍이 1세트뿐이어서 모두 `INTRINSIC_OBSERVATIONS_INSUFFICIENT`로 중단됐다. 따라서 15° yaw 탐색과 Ceres는 이 데이터에서 실행되지 않았고, 출력 이미지는 공통 prior 투영 진단이다.

현재 확인된 핵심 이슈는 다음과 같다.

1. 네 채널에 공통 `prior-roll=90°`를 적용해 모든 광축을 수직 아래로 시작시킨다. 실제 PNM-C16083RVQ의 채널별 렌즈가 사선 방향을 보는 경우, yaw만 바꿔서는 바닥 중심 투영을 해결할 수 없다. 채널별 optical tilt/axis 탐색이 필요하다.
2. 130333 JSON의 `mechanism.tilt_zero`는 `nadir`로 기록되어 있지만 현재 ICD와 adapter 식은 `tilt=0°` 수평 전방, `tilt=-90°` 아래를 요구한다. producer metadata와 좌표변환 계약을 먼저 확정하고 불일치 시 입력을 거절해야 한다.
3. 단일 관측에서는 K와 RT 공동 추정이 불가능하므로 자동 보정 결과를 만들 수 없다. 사람 없는 장면에서 채널별 최소 3세트, 권장 5세트의 실제 image–LiDAR 쌍이 필요하다.
4. 현재 투영 시 모든 LiDAR 점을 그리므로 카메라에서 가려진 벽 점과 장애물 뒤 점을 제거하지 않는다. 향후 camera-view z-buffer/occlusion 처리가 필요하다.
5. `04_lidar_surface_normals.ply`의 색상은 normal 방향 시각화이며 벽·바닥·장애물의 의미론적 분류가 아니다. 평면 방향과 장애물 경계의 가중치를 분리해야 한다.

따라서 현재 실데이터 calibration의 최종 상태는 **RT 미확정**이다. 15° 변경은 탐색 해상도를 개선했지만, 공통 하향 prior·tilt 계약 불일치·관측 부족을 해결하지 않으므로 단독으로 바닥 투영 문제를 해결하지 않는다.

## 4. 실패 내용 분석

### 4.1 입력 및 실험 구성 문제

#### 카메라–LiDAR 비강체 조건

외부 파라미터는 두 센서가 강체로 고정됐을 때 하나의 값으로 정의된다. session-002와 session-003의 손-held 다각도 사진은 사진마다 optical center와 orientation이 바뀐다. 같은 위치 또는 같은 천장을 촬영했다는 사실만으로 동일 extrinsic이 되지 않는다.

#### 임시 camera intrinsic

Galaxy S25 입력은 EXIF의 35 mm 환산 초점거리로 `fx=fy=2555.56 px`를 근사하고 distortion을 0으로 가정했다. 이는 smoke test용이며 정확한 RT 계산에 사용할 수 없다. 특히 영상 가장자리의 왜곡은 수십 pixel 이상의 오차로 이어질 수 있다.

#### 장면 퇴화

평평한 천장은 LiDAR range가 거의 일정하고 실제 depth discontinuity가 적다. 반면 카메라에는 조명, 타일 선, 그림자와 질감 edge가 많이 나타난다. 두 modality의 edge가 동일한 물체 경계를 의미하지 않으므로 우연한 최소점이 발생한다. 반복적이고 대칭적인 구조는 heading과 translation 모호성을 추가한다.

#### 넓은 scan과 prior 의존 FoV

카메라와 실제로 겹치지 않는 LiDAR 영역이 많고, 초기 RT가 틀리면 prior 기반 FoV 선택도 잘못된 영역을 선택한다. session-002에서는 point의 약 73.5%가 영상 밖으로 투영됐다. 단순 interpolation은 이 기하학적 overlap 문제를 해결하지 못한다.

### 4.2 Point cloud 취득 문제

- 200/400 bps 사이 동일 셀 range 재현성이 낮음
- scan 속도에 따른 진동, angle/time alignment 또는 moving-window 병합 영향 미분리
- mechanism 180°와 scan 360° metadata 불일치 사례 존재
- 긴 scan 시간 동안 actuator와 환경의 안정성 검증 부족
- F2P `signal_strength`와 `range_precision`을 최적화 weight로 아직 반영하지 않음

현재 유효 point 비율이 99% 이상이라는 사실은 거리와 ray 방향이 정확하다는 뜻이 아니다. `valid=true`는 packet/범위 검사 통과를 나타낼 뿐 기구학 정확도와 scan 재현성을 보증하지 않는다.

### 4.3 알고리즘 및 품질 게이트 문제

현재 목적함수는 LiDAR depth edge가 어떤 camera edge에 가까운지를 최소화한다. 동일 물체의 동일 경계인지, edge 방향과 near/far 구조가 일치하는지는 확인하지 않는다. Camera edge가 매우 많으면 틀린 RT도 낮은 평균 거리를 얻을 수 있다.

기존 Core는 최종 목적함수가 초기값의 105%보다 커질 때만
`OBJECTIVE_NOT_IMPROVED`로 거절했다. 따라서 거의 개선되지 않았거나 소폭
악화된 결과도 다른 절대 게이트를 통과하면 PASS가 될 수 있었다. 현재는 실제
실행기에 최소 개선률 5%를 적용하고 미달 시
`OBJECTIVE_IMPROVEMENT_INSUFFICIENT`로 거절한다. session-003 및 130333 CH3의
false PASS가 이 회귀 조건의 근거다.

계획서에는 다음 품질 조건이 정의돼 있으나 현재 Core에 모두 구현되지는 않았다.

- coarse 대비 fine objective의 유의미한 개선
- inlier ratio
- multi-start 결과 pose 편차
- edge 방향 및 구조선 일치
- 여러 독립 장면에서의 반복 관측
- 실제 장면 분리 검증 frame의 재투영 오차

실패 시 `estimated_t_camera_lidar`가 안전하게 prior로 돌아가는 계약은 유효하다. 단, 시각화에서 prior를 최적화 결과처럼 오인하지 않도록 `FAIL: showing prior RT`를 명시해야 한다.

### 4.4 Interpolation 평가

영상 interpolation은 회전과 resize의 계단 현상을 줄이고, point splatting은 시각화를 보기 쉽게 만들 수 있다. 그러나 잘못된 RT, 비강체 촬영, 빈 depth 구조 또는 speed-dependent point cloud를 수정하지 못한다. Range image의 깊이 경계를 가로질러 보간하면 존재하지 않는 point와 평면을 만들어 matching을 더 악화시킬 수 있다.

따라서 interpolation은 작은 결측의 edge-preserving 보정과 시각화에만 사용하고, 최적화에서는 원본 측정, 유효 mask 및 uncertainty를 유지한다.

## 5. 계획서 기준 대비 현재 상태

| 계획서 초기 성공 기준 | 현재 상태 | 판정 |
|---|---|---|
| 자연 장면 edge 재투영 RMSE 3 px 이하 | 현재 구현 mean edge distance 35.34~238.95 px(정의가 달라 직접 RMSE 비교 불가) | 검증 지표 미충족 |
| 회전 반복 표준편차 0.2° 이하 | 20260818 동일 epoch에서 3개 추정용 입력과 1개 고정 RT hold-out은 확보했지만 RT 분산은 아직 산출하지 않음 | 부분 검증 / 분산 미평가 |
| 이동 반복 표준편차 10 mm 이하 | 20260818 동일 epoch에서 3개 추정용 입력과 1개 고정 RT hold-out은 확보했지만 RT 분산은 아직 산출하지 않음 | 부분 검증 / 분산 미평가 |
| ±2°/±50 mm 초기 오차 복원 90% 이상 | 합성 일부 검증, 실데이터 미검증 | 미충족 |
| 잘못된 결과 활성화 0건 | false PASS 발견, 최소 objective 개선률 5% 게이트 및 회귀 테스트 추가 | 부분 충족 / 독립 검증 필요 |
| 동일 입력 offline replay 재현 | 동일 입력 solver 재현 가능, 센서 반복성 미충족 | 부분 충족 |

현재 단계는 Calibration Core MVP의 합성 가능성 검증까지 완료됐고, 실제 장비 accuracy validation은 미완료다. Calibration Core 개발 완료 또는 제품 적용 가능 상태로 판단하지 않는다.

## 6. 결정 및 권고 방향

### 6.1 즉시 결정

1. session-001~003의 모든 실데이터 결과를 FAIL로 유지한다.
2. session-003 200 bps 01의 계산상 PASS를 false-positive regression case로 등록한다.
3. 고정환경 130333 CH3의 기존 120° PASS도 false positive로 폐기한다.
4. 독립 frame 검증이 추가될 때까지 자동 PASS 결과를 활성 calibration으로 사용하지 않는다.
5. 카메라와 LiDAR를 강체로 고정하지 않은 사진은 extrinsic 산출 입력에서 거절한다.
6. 단순 interpolation 추가를 근본 해결책으로 채택하지 않는다.

### 6.2 실험 환경 재구성

- 카메라와 LiDAR를 하나의 plate/frame에 강체 고정
- LiDAR frame 기준 camera center `(x,y,z)`와 장착 roll/pitch/yaw 측정
- 두 센서의 +X/+Y/+Z 방향을 사진과 CAD로 기록
- 동일 channel/resolution/zoom/focus/LDC profile의 Manual ChArUco K+D 고정
- target-based/ChArUco 촬영은 profile 생성·독립 reference가 필요할 때만 별도 사용
- 센서 묶음 전체 이동은 허용하되 센서 사이 상대 pose 변경 금지
- 평평한 천장 대신 방 모서리, 높이가 다른 상자와 비대칭 다중 평면 사용
- 같은 설치에서 200 bps 5회, 400 bps 5회 반복
- 진단 reference로 stop-and-measure scan 추가

### 6.3 알고리즘 방향

제품 목표는 targetless로 유지하되, 개발 기준값 확보를 위해 target-based reference를 병행한다.

1. 비대칭 multi-plane target 또는 camera marker와 LiDAR plane/edge를 이용해 기준 RT를 산출한다.
2. 동일 frame의 reference RT와 targetless RT를 비교한다.
3. Edge-only 비용을 단독 판정 기준에서 제외한다.
4. 최소 objective 개선률, yaw multi-start 모호성 gate와 camera-view z-buffer는 추가했다. symmetric matching, edge orientation 및 독립 pose 일관성 gate는 후속 구현한다.
5. 여러 장면이 동일 RT로 수렴할 때만 PASS로 판정한다.
6. NMI는 F2P signal strength의 거리 보정, entropy, saturation 및 반복성이 확인된 경우에만 보조항으로 평가한다.

Target-based 방식은 최종 제품 운용을 대체하기 위한 것이 아니라 센서 좌표계, projection과 targetless 결과를 검증할 ground truth/reference 확보 수단이다.

### 6.4 Geometry NID + yaw multi-start 구현 및 CH3 재확인 (2026-08-11)

Edge-only false minimum을 줄이기 위해 다중 장면 Calibration Core를 다음과 같이 변경했다.

- RGB gradient magnitude와 LiDAR range discontinuity/surface-normal 변화량의 soft-histogram NID
- `0.70 × NID² + 0.30 × normalized edge distance²` 복합 목적함수
- 당시 검증은 LiDAR 수직축 기준 45° 간격 8개 yaw 후보의 360° multi-start로 수행했으며, 현재 실행 기본값은 15° 간격 24개 후보로 변경했다
- 멀리 떨어진 후보 점수 차이가 2% 미만이면 `MULTISTART_AMBIGUOUS`
- NID 투영 수·개선률 부족 시 보정값 미적용
- 원시 `signal_strength` NMI는 conformance 전까지 비활성

합성 회귀에서 180° 잘못된 초기 heading을 복구했으며 전체 CTest를 통과했다. 고정환경 `130333/CH3` 한 쌍은 새 실행기로 재확인했지만 `INTRINSIC_OBSERVATIONS_INSUFFICIENT`로 정상 거절됐다. 이는 알고리즘 정합 실패가 아니라 수동 intrinsic 없이 K와 RT를 공동 추정하기 위한 최소 3관측 조건 때문이다. 따라서 이 한 쌍의 시각화는 prior 투영 진단일 뿐 자동 보정 결과가 아니다.

재실행 결과: `automatic_calibration/generated/real_session_const_20260811_130333_geometry_nid/ch3/`


### 6.5 공통 90° prior 제거 및 단일 관측 진단 재실행 (2026-08-12)

기록된 현 이슈 중 공통 하향 prior, 관측 부족 조기 중단, 좌표계 계약 미검증 및 가림 미처리를 다음과 같이 수정했다.

- `prior-roll=90°` 기본값을 제거하고 down 0~90°(15° 간격) × yaw 360°(15° 간격)의 2차원 초기 방향 탐색으로 변경
- 단일 image–LiDAR 쌍은 제조사 FOV 기반 K를 고정한 pose 진단을 실행하되 `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL`로 강제하고 활성 RT는 prior 유지
- 실패 후보 RT를 `diagnostic_candidate_t_camera_lidar`로 분리하고 상위 5개 후보의 점수·방향·투영 이미지를 저장
- `mechanism.tilt_zero=forward` 계약 불일치를 기본 거절하고, producer 확인 후 명시적 override만 허용·기록
- 2D 투영과 colorized PLY/OBJ에 최근접 표면 z-buffer(10 mm 허용)를 적용

130333 CH1~CH4 한 쌍씩 재실행한 결과 모든 채널은 의도대로 FAIL을 유지했다. 선택 down 후보는 CH1 15°, CH2 0°, CH3 30°, CH4 0°로, 네 채널을 모두 바닥으로 강제하던 공통 90° 현상은 제거됐다. 그러나 네 채널 모두 내부 후보 게이트가 `NID_OVERLAP_INSUFFICIENT`이므로 이 방향과 RT는 정답이 아니다. 최소 3개, 권장 5개 이상의 서로 다른 구조 관측이 들어오기 전까지 **RT 미확정** 상태를 유지한다.

재실행 결과: `automatic_calibration/generated/real_session_const_20260811_direction_search/ch1/` ~ `ch4/`

### 6.6 Pan/Tilt sweep과 초기 방향 탐색의 관계 (2026-08-12)

실제 130333 JSON을 수치로 확인한 결과 `tilt_min_rad=-1.570796`, `tilt_max_rad=0.0`, 측정값 최소·최대도 약 `-90.0°~0.0°`였다. JSON의 `units.angle=radian`은 올바르며, 도 단위 환산은 `deg=rad×180/π`이다. 따라서 현재 파일의 범위는 `-90~0°`이고, 팀에서 말한 장치 전체 허용범위 `-90~+90°`와는 다른 개념이다.


현재 LiDAR 좌표식은 `x=d cos(tilt) sin(pan)`, `y=-d sin(tilt)`, `z=d cos(tilt) cos(pan)`이다. 따라서 pan 회전축은 LiDAR `+Y`이며 Core의 15° yaw multi-start 축(`UnitY`)과 축 방향은 일치한다. 단, pan 360° sweep은 한 번의 point cloud에 모든 방위각을 포함한다는 뜻이고, yaw multi-start는 카메라-LiDAR 외부 회전 후보를 고르는 절차이므로 서로 같은 변수는 아니다.

130333 JSON 파일의 취득 범위는 `pan=0~약 360°`, `tilt=-90~0°`이며, 장치가 지원하는 전체 tilt 범위(-90~+90°)와는 구분해야 한다. 이 파일에는 +tilt 구간 측정값이 없다.  `mechanism.tilt_zero`는 `nadir`로 기록되어 있다. 현재 adapter 식은 `tilt_zero=forward` 계약을 전제로 하므로, 이번 CH1~CH4 재실행은 `--tilt-zero-override forward`를 사용한 진단 실행이다. 이는 producer 좌표계가 확정됐다는 의미가 아니며, 제품 calibration 결과로 사용할 수 없다.

현재 down 후보 `0~90°(15° 간격)`는 JSON tilt 샘플을 자동 변환한 값이 아니라 카메라 optical axis 외부 회전의 초기 후보다. 따라서 실제 tilt sweep 범위와 관련은 있지만 동일한 값은 아니다. CH2의 `down=0°` 선택은 현 목적함수에서 가장 낮은 후보였다는 뜻일 뿐이며, 단일 관측·NID overlap 부족으로 RT는 미확정이다. 다음 단계는 `tilt_zero` 의미와 `tilt=0/-90°` 실제 광선 방향을 producer와 확정한 뒤 adapter 식과 down 후보 생성을 함께 검증하는 것이다.

### 6.7 130333 1° full-search 선행 게이트 실행 (2026-08-12)

CH1~CH4 각각 yaw 360°와 down 0~90°를 1° 간격으로 전수 평가해 채널당
32,760개 raw orientation score를 생성했다. 아래 튜플은 혼동을 피하기 위해
`(down_deg, yaw_deg)` 순서로 표기한다. Raw 최저 방향은 CH1 `(15°,-42°)`,
CH2 `(12°,-43°)`, CH3 `(2°,-36°)`, CH4 `(3°,-43°)`였다. 모든 채널의 내부
후보 gate는 `NID_OVERLAP_INSUFFICIENT`였고, 단일 관측과 reference RT 부재로
`FULL_SEARCH_BASELINE_DIAGNOSTIC_ONLY / BLOCKED_BY_REFERENCE_UNAVAILABLE`로
판정했다. 따라서 coarse step 최적화와 인접 후보 가중치 시험은 실행하지 않았다.

결과: `automatic_calibration/generated/real_session_const_20260811_full_search_1deg/`

### 6.8 20260818 CH1 고정환경 회차 확인 (2026-08-18)

운영자가 `data/real_calibration/session-const-env/repeat_test_sample/20260818/`의
수집 조건을 재확인했다. 네 image와 네 LiDAR JSON은 카메라, LiDAR, pan-tilt actuator,
보드 및 장면을 수집 중 이동시키지 않은 동일 installation epoch
(`session-const-env-20260818-ch1-fixed`)에 속한다. 따라서 파일명 시각이 다르다는 이유로
서로 다른 외부 파라미터를 적용하는 데이터로 분류하지 않는다.

이번 회차 입력은 다음과 같다.

| scene | CH1 image | LiDAR JSON | 역할 |
|---:|---|---|---|
| 0 | `20260818-143751-CH1.jpg` | `calib-20260818-143748_sweep-000001_pan_tilt_lidar.json` | RT 추정 |
| 1 | `20260818-145847-CH1.jpg` | `calib-20260818-145912_sweep-000001_pan_tilt_lidar.json` | RT 추정 |
| 2 | `20260818-151305-CH1.jpg` | `calib-20260818-151312_sweep-000001_pan_tilt_lidar.json` | RT 추정 |
| 3 | `20260818-155208-CH1.jpg` | `calib-20260818-154229_sweep-000001_pan_tilt_lidar.json` | 고정 RT hold-out |

여기서 image–scan **pairing**은 센서가 동시 촬영됐다는 뜻이 아니라 offline replay를
위한 결정론적 입력 연결이다. JSON은 연속 sweep이고 wall-clock 촬영시각을 제공하지
않으므로 시간 동기화 conformance로 해석하지 않는다. 정적 장면과 동일한 센서 상대
자세의 반복성을 확인하는 회차로만 사용한다.

동일 epoch의 scene 0~2로 추정한 RT를 scene 3에 재최적화 없이 적용한 결과는
`fixed-environment hold-out PASS`다. 다만 이는 한 설치 epoch에서의 재투영 재현성
확인이지, 서로 다른 장소·설치 epoch에 대한 일반화나 Manual RT와의 절대 정확도 인증이
아니다. 따라서 운영 RT 교체는 보류하고, actuator·zoom·focus·LDC·영상 방향을 변경한
뒤에는 새 `installation_epoch`로 분리한다.

세부 파일 목록, 고정 조건, pairing 규칙 및 산출물은
[`CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md`](CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md)를
기준 문서로 사용한다.

## 7. 다음 테스트 절차

### Phase A — Point cloud 단독 검증

1. 고정 평면에서 200/400 bps 각 5회 취득
2. cell별 range bias, 표준편차 및 속도 간 차이 계산
3. 좌→우/우→좌 sweep 차이와 row 전환 구간 분석
4. encoder timestamp와 LiDAR timestamp 정렬 검증
5. mechanism pan 범위와 JSON pan 좌표 계약 확정

Point cloud가 센서 사양과 프로젝트 noise gate를 만족하지 않으면 calibration을 실행하지 않는다.

### Phase B — 투영 경로 검증

1. 동일 channel/resolution/zoom/focus의 Manual ChArUco K+D profile과 고정된 LDC 상태의 pixel 좌표 일관성 검증
2. 필요 시 알려진 3D target point를 개발 reference RT로만 투영
3. 이미지 회전 전후 camera model과 pixel 좌표 일관성 확인
4. `T_camera_lidar` 방향과 inverse 사용 여부 확인
5. 채널별 렌즈 optical axis/tilt를 측정값 또는 coarse multi-start로 확정

### Phase C — Calibration 반복성 검증

1. 구조가 풍부한 장면 5개 이상 취득
2. 동일 mechanical prior로 multi-scene calibration
3. 10회 반복 RT 평균과 표준편차 계산
4. 200/400 bps의 RT 교차 검증
5. 학습/최적화에 사용하지 않은 독립 frame으로 재투영 검증

### Phase D — 회귀 및 Jenkins 등록

- session-003 false PASS를 반드시 FAIL로 만드는 negative test
- 낮은 overlap, edge 부족, 잘못된 intrinsic, EXIF 회전, metadata 불일치 test
- 합성 ground-truth positive test 유지
- 실데이터 reference RT 허용오차 test 추가

## 8. 관련 산출물

- [Calibration Core 아키텍처](CALIBRATION_CORE_ARCHITECTURE.md)
- [Pan-Tilt LiDAR JSON 인터페이스](PAN_TILT_LIDAR_JSON_INTERFACE.md)
- [Jenkins Conformance Test 계획](JENKINS_CONFORMANCE_TEST_PLAN.md)
- [Targetless 방법 및 논문 검토](TARGETLESS_CALIBRATION_METHOD_REVIEW.md)
- [전체 프로젝트 계획서](../../../project_plan/1d_lidar_pan_tilt_camera_auto_calibration_plan.md)
- [Point cloud 이후 Calibration 상세 계획](../../../project_plan/02_post_point_cloud_calibration_detailed_plan.md)
- Session-003 matching 이미지: `archive/non_fixed_environment_20260811/generated/real_session_003_matching/`
- Session-003 orientation 탐색: `archive/non_fixed_environment_20260811/generated/real_session_003_orientation_probe/`
- Session-003 yaw 세부 탐색: `archive/non_fixed_environment_20260811/generated/real_session_003_yaw_refine/`
- 고정환경 130333 축 보정 결과: `automatic_calibration/generated/real_session_const_20260811_130333_ceiling_corrected/best/`
- 고정환경 CH3 V2.1/84 mm 및 판정 게이트 재검증: `automatic_calibration/generated/real_session_const_20260811_130333_v21_offset/ch3_heading_120_gated/`
- 고정환경 CH3 반대 방향(-60°) 결과: `automatic_calibration/generated/real_session_const_20260811_130333_v21_offset/ch3_heading_minus60/`

## 9. 수정 이력

| 버전 | 날짜 | 변경 내용 |
|---|---|---|
| 0.1 | 2026-08-07 | Stanford 합성 검증과 real session-001~003 진행 내역, 실패 원인, false PASS 재분류 및 재시험 방향 최초 작성 |
| 0.2 | 2026-08-11 | 비고정 환경 session-001~003 원본 및 생성 결과를 보관 디렉터리로 이동하고 관련 산출물 경로 갱신 |
| 0.3 | 2026-08-11 | 천장 설치 90° 축 변환, Z-up 시각화 및 고정환경 130333 채널별 재시험 결과 추가 |
| 0.4 | 2026-08-11 | LiDAR ICD V2.1 range offset 반영, CH3 180° 반대 방향 재검증, 기존 PASS 폐기, 최소 objective 개선률 5% 게이트와 회귀 테스트 기록 |
| 0.5 | 2026-08-11 | Automatic 경로의 manual intrinsic 의존 제거, 제조사 FOV 초기화와 K-RT 공동 최적화, 최소 3관측 fail-safe 및 논문 기반 방법 검토 연결 |
| 0.6 | 2026-08-11 | geometry NID·edge 복합 refinement, 360° yaw multi-start, 모호성/NID gate 및 CH3 재실행 결과 반영 |
| 0.7 | 2026-08-12 | 15° yaw coarse search, 단계별 debug artifact, 사람 없는 130333 CH1~CH4 재실행 결과 및 공통 prior-roll/tilt 계약/occlusion 현재 이슈 기록 |
| 0.8 | 2026-08-12 | 공통 90° prior 제거, down×yaw 탐색, 단일 관측 고정-K 진단 전용 처리, 후보 RT 분리, tilt_zero 계약 검증, z-buffer 및 CH1~4 재실행 결과 반영 |
| 0.9 | 2026-08-12 | pan 360°와 UnitY yaw 축의 관계, JSON tilt 범위/`nadir` 불일치, CH2 후보의 제한적 의미 및 후속 좌표계 확정 절차 기록 |
| 0.10 | 2026-08-12 | 130333 CH1~CH4 1° full-search 32,760 후보 실행, NID overlap 실패와 후속 step-size 시험 중단 기록 |
| 0.11 | 2026-08-12 | B 방식(8-neighbor 보정점수와 contiguous basin) 구현 및 CH1~CH4 1° 재시험 기록 |
| 0.12 | 2026-08-13 | 설치 제약 기반 Top X-Y 개선 과정과 Front X-Z 잔여 오차 분석, 1° 국소 탐색·연속 3D/2D 선분 방향/끝점/겹침 비용·수평/수직 점수 분리·제한 K profile 3×3 CH1 실행 결과 기록 |
| 0.13 | 2026-08-13 | 기존 `04b`가 평면 교차선이 아니라 폐색 윤곽이었다는 구현 불일치를 수정하고, robust normal·평면 분할·평면 교차선·폐색선 분리 및 단계별 산출물/합성 테스트 추가 |
| 0.14 | 2026-08-13 | 현재까지의 좌표계, 자동 캘리브레이션, 구조선 변경, Docker/CTest 검증, 130333 CH1 결과와 미완료 작업을 `CURRENT_PROGRESS_AND_STATUS.md`로 통합 정리 |
| 0.15 | 2026-08-13 | 미분류 평면 재할당, 공면 병합, IMU-Y 수평면 높이 복구와 130333 CH1 재시험 결과 기록 |
| 0.16 | 2026-08-13 | FAIL 3D 산출물이 탈락 후보 RT를 사용하던 오류 수정, 설치 중심·광축·영상 아래 벡터 기반 재투영 및 OBJ 카메라 광축 마커 추가 |
| 0.17 | 2026-08-13 | VS Code Viewer mesh의 mm 좌표를 m로 변경하고 PLY/OBJ 단위 계약을 명시 |
| 0.18 | 2026-08-20 | 제품 운용 정책 정정: Manual ChArUco K+D 고정, K+RT 공동 추정 보류, 내부 PASS와 제품 승인 상태 분리 |
| 0.18 | 2026-08-13 | 지금까지의 좌표계·데이터·목적함수·탐색·구조선·투영·K profile 시도를 시간순으로 통합하고, 유효 결론·폐기 가정·남은 게이트를 명시 |
| 0.19 | 2026-08-14 | range/normal spatial NID, 평면 경계·반복 폐색 구조선, 1:1 대응, Manhattan 수직축, signal NMI 진단, 장면별/hold-out gate 및 v6~v9 결과 추가 |

## 10. 현재까지 시도한 내용 요약

### 10.1 데이터와 환경

- Stanford 2D-3D-Semantics Dataset을 actuator 완성 전 검증 데이터로 검토했다.
- 실제 데이터는 `session-const-env/2026-08-11/130333`을 기준으로 사용했다.
- 카메라와 LiDAR의 설치 높이·횡방향 offset을 고정하고 CH1~CH4를 반복 평가했다.
- 카메라는 천장을 향해 설치되고 LiDAR는 반대 방향으로 천장을 스캔하는 설치 방향을 반영했다.

### 10.2 좌표계·센서 처리 시도

- LiDAR JSON의 pan/tilt/distance를 Cartesian point cloud로 변환했다.
- 천장 설치 방향과 카메라 좌표계의 180°/90° 방향 혼동을 점검했다.
- 공통 `prior-roll=90°` 가정을 제거하고 down(0~90°)과 yaw(0~359°)를 독립 탐색하도록 변경했다.
- 제조사 사양 기반 내부 파라미터를 기본값으로 사용하고, 수동 checkerboard 내부 캘리브레이션 의존은 자동 경로에서 제거했다.
- range offset, tilt-zero 방향, 카메라 중심 offset, LDC/zoom/focus 고정 조건을 명시적으로 입력했다.

### 10.3 매칭 알고리즘 시도

- image edge와 LiDAR surface normal을 이용한 geometry/NID 기반 매칭을 적용했다.
- 장애물·벽·바닥이 동일 surface로 섞이는 문제를 확인하기 위해 point-cloud normal 및 중간 산출물을 저장했다.
- 초기 45° coarse search를 15° coarse search로 줄였고, 이후 1° full-search를 기준선으로 설정했다.
- raw score 단독 선택 대신 인접 8개 후보의 평균을 반영하고 yaw 원형 연결 및 contiguous basin을 적용하는 B 방식을 구현했다.
- 최종 결과를 PNG, PLY, OBJ, CSV로 생성해 2D 이미지와 3D 투영을 직접 비교할 수 있게 했다.

### 10.4 테스트 결과와 확인된 실패

- 1° full-search는 채널당 360×91=32,760개 후보를 생성했다.
- CH1~CH4 모두 `NID_OVERLAP_INSUFFICIENT` 또는 단일 관측 진단 모드로 인해 자동 PASS가 되지 않았다.
- B 방식 선택 결과는 CH1 `(15°, -42°)`, CH2 `(13°, -50°)`, CH3 `(2°, -36°)`, CH4 `(3°, -43°)`이며 튜플은 `(down, yaw)` 순서다.
- 인접 후보 보정으로 단일 후보에 의한 불안정성은 완화했지만, 같은 벽면 또는 바닥 쪽으로 투영되는 근본 문제는 해결되지 않았다.
- 주요 미해결 원인은 단일 장면의 반복 구조, 실제 RT 기준값 부재, 카메라/LiDAR 좌표축·tilt-zero 계약 미확정, LiDAR 관측과 이미지 시야의 overlap 부족이다.

### 10.5 현재 판단과 다음 단계

- 현재 결과는 알고리즘 PASS가 아니라 후보 탐색·시각화 파이프라인의 진단 결과다.
- coarse step 크기 최적화는 1° full-search가 reference RT와 다중 장면 기준을 통과한 뒤 진행해야 한다.
- 다음 검증에는 서로 다른 yaw/tilt의 다중 장면, 실제 또는 합성 ground-truth RT, 좌표계 계약 확정, 벽·바닥·장애물 분리 검증이 필요하다.




## 10.6 LDC unknown and Gaussian weighting test (2026-08-12)

- LDC was not found in the camera web settings, so the default is unknown; SSDR is a separate image dynamic-range function.
- LDC values are validated as true, false, or unknown; an unknown state is never assumed to be OFF.
- Eight-neighbor compensation uses Gaussian yaw/down distance weighting with sigma_yaw=5 deg, sigma_down=5 deg, and alpha=0.5.
- CH1 test: ldc_enabled=unknown, basin down=15 deg, yaw=-42 deg, basin size 2, diagnostic-only FAIL because the input has one observation and no reference RT.
- CTest: 5 of 5 passed.
- Full compute-saving coarse-to-fine separation is not complete because the current API runs Ceres before basin selection.


## 10.7 CH1 고정환경 좌표 변환 비교 시험 (폐기된 진단 실험, 2026-08-12)

고정환경 기준은 `data/real_calibration/session-const-env/2026-08-11/130333`이며 CH1만 사용했다. LiDAR JSON metadata는 `tilt_zero=nadir`, 유효 tilt는 -90°~0°다. `--ldc-enabled`는 생략해 unknown으로 기록했다.

세 변환 케이스를 동일한 1° yaw/down 탐색으로 비교했다.

| 케이스 | 정의 | 선택 basin (down, yaw) | corrected score | basin 수 |
|---|---|---:|---:|---:|
| current | 기존 tilt 식 유지 | (15°, -42°) | 0.810089 | 2 |
| tilt_sign_flip | tilt 부호 반전 | (16°, 1°) | 0.576399 | 1 |
| nadir_reference | nadir 각도로 재표현 | (15°, -42°) | 0.810089 | 2 |

세 케이스 모두 단일 관측 진단 FAIL이며, score가 낮다는 이유만으로 물리적 정합을 확정하지 않는다. `nadir_reference`가 current와 같은 것은 수식의 대수적 재표현이기 때문이다.

## 10.8 Device V2 ICD 좌표계 출력 정합 (철회된 중간 변경, 2026-08-12)

Confluence Device V2 1D LiDAR 계약을 기준으로 사람이 확인하는 PLY/OBJ 출력 프레임을 `device_v2_icd`로 변경했다. 출력 식은 `x=d*cos(phi)*cos(theta)`, `y=d*cos(phi)*sin(theta)`, `z=d*sin(phi)`이며 단위는 mm이다. 내부 Calibration Core projection은 기존 optical-style frame을 유지하고, 출력 시 명시적인 내부→ICD 변환을 적용한다.

변경 후 `scene_0_colorized_lidar.ply`, `_z_up.ply`, OBJ 헤더에 frame과 units를 기록한다. 이는 기존 top-view에서 ICD 축과 표시 축이 뒤섞일 수 있었던 문제를 줄인다. 단, Core 내부 회전 prior까지 ICD 축으로 직접 변경한 것은 아니므로, 실제 CH1 optical-axis 비교는 ICD viewer 좌표에서 별도 frustum을 함께 그려 검증해야 한다.

CTest 5/5 및 CH1 smoke test를 실행했다. smoke test는 단일 관측 진단 모드의 예상 종료코드 3이며 ICD PLY/OBJ 산출물은 정상 생성되었다.

## 10.9 ICD 식과 내부 식의 문서 혼재 수정 (철회된 중간 변경, 2026-08-12)

기존 문서 본문에는 내부 optical frame 식과 Device V2 ICD 식이 함께 존재했으나 서로 다른 좌표계라는 설명이 충분하지 않았다. 이로 인해 `x=range*cos(tilt)*sin(pan)`과 `x_icd=range*cos(phi)*cos(theta)`가 같은 식처럼 보이는 문제가 있었다.

수정 후 외부 JSON/PLY/OBJ의 기준은 Device V2 ICD 식으로 고정하고, Calibration Core 내부는 다음 adapter로 명시했다.

```text
p_internal = (y_icd, -z_icd, x_icd)
p_icd = (z_internal, x_internal, -y_internal)
```

따라서 두 식은 동일한 좌표계의 대체식이 아니며, ICD→internal 변환 후에만 현재 projection 코드와 연결된다.

## 10.10 기구각/계약각 및 방위 기준 정정 (2026-08-12)

이전 분석에서 `mechanism.tilt_zero=nadir`를 계약각 원점으로 해석한 것은 오류였다. 실제 JSON에서 `mechanism.tilt_zero`는 모터 기구축 홈 메타데이터이고, 좌표 계산은 `measurements[].tilt_rad` 및 `frame.range_formula`를 사용한다. 130333에서는 계약 tilt=0°가 수평, -90°가 아래 방향이며 별도 +90° 보정을 하지 않는다.

또한 pan/theta=0은 카메라 CH1 optical axis와 일치해야 하는 값이 아니다. pan 기구 홈 offset은 센서 방위 기준일 뿐이며, 카메라와의 상대 회전은 외부 캘리브레이션 대상이다. 현재 미확정 항목은 pan 증가 방향의 손대칭(시계/반시계)이다. 평면·대칭 환경으로는 이를 검증할 수 없으므로 비대칭 지형지물로 별도 시험해야 한다.

카메라 중심 offset은 JSON에 포함되지 않으며 `t_camera_lidar` 추정 대상이다. `range_offset_m`만 축교점과 LiDAR 발광면의 거리 보정으로 적용한다.

이에 따라 PLY/OBJ 출력은 Device V2 JSON의 `frame.range_formula`와 동일한 `lidar_scan` 좌표계로 되돌렸고, 파일 단위는 meter로 기록한다. 이전의 ICD frame/mm 출력 변경은 JSON 계약과 혼재를 만들 수 있어 철회했다.


## 10.11 PLY/OBJ viewer 출력 수정 (2026-08-12)

내부 Calibration 계산 단위는 meter로 유지하되, PLY/OBJ와 `_z_up` export는 viewer 호환을 위해 mm로 출력하도록 수정했다. 파일 header에 `units mm`를 기록한다. 이전 meter export에서 일부 viewer가 point cloud를 지나치게 작게 표시하거나 보이지 않는 문제가 발생할 수 있었다. preview의 좌표 변환도 `_z_up` 파일과 동일한 `viewer_z_up=(x,z,-y)`로 통일했다.

CH1 smoke test 산출물은 `automatic_calibration/generated/real_session_const_20260811_ch1_ply_fixed/`에 생성했고 CTest 5/5를 통과했다.

## 10.12 현재 기준 재감사 및 CH1 재시험 (2026-08-12)

현재 권위 기준은 JSON `frame` 계약이다. `mechanism.tilt_zero=nadir`는 기구축 홈
메타데이터이며, 좌표 계산에는 `measurements[].tilt_rad`를 그대로 사용한다. 제품
실행 경로에서 `tilt_sign_flip`, `nadir_reference`, `tilt-zero-override`를 제거하고,
지원하지 않는 handedness/convention 입력을 거절하도록 수정했다.

고정환경 실측값에서 카메라는 LiDAR보다 천장에 83.05 mm 가까우므로 내부
`+y=down` 기준 camera center Y를 `-0.08305 m`로 정정했다. CH1을 yaw/down 1°
전체 탐색으로 재실행한 결과는 다음과 같다.

| 항목 | 결과 |
|---|---|
| camera center 입력 | `(0.05928, -0.08305, 0) m` |
| 선택 basin `(down, yaw)` | `(0°, -40°)` |
| viewer top-view optical-axis heading | 약 `+50.4°` |
| 전체/카메라 색상점 | `40,307 / 893` |
| 판정 | `FAIL / SINGLE_OBSERVATION_DIAGNOSTIC_ONLY` |
| 내부 후보 gate | `NID_OVERLAP_INSUFFICIENT` |

3D preview에 최종 진단 후보의 카메라 중심(주황색)과 optical axis(빨간 화살표)를
추가했다. 이 결과는 사용자 표시의 실제 CH1 방향(오른쪽)과 일치하지 않는다. 따라서
현재 잔여 문제는 PLY 축 표시가 아니라 단일 관측·낮은 overlap·반복 구조에서 목적함수가
물리적으로 틀린 basin을 선택하는 문제다. 이 후보는 활성 RT로 사용하지 않는다.

다음 필수 검증은 pan 증가 방향의 물리적 손대칭 conformance test와 비대칭 구조를 포함한
동기화 관측 3쌍 이상이다. 이 조건 전에는 objective 가중치나 coarse step을 조정해도
정확한 자동 캘리브레이션으로 판정하지 않는다.

## 10.13 CH1 overlap gate 및 구조 특징 수정 재시험 (2026-08-12)

실장 확인으로 pan 증가는 Top-view 기준 시계 방향으로 확정했다. 후보별 edge/NID
화면 내 투영 수를 기록하고, 저-overlap 후보를 점수 계산 단계에서 무효화했다. 또한
구조 변화값이 거의 0인 평면 내부 포인트를 NID 입력에서 제외하고, 실패 후보의
Top-view 표기를 `CAM`에서 `REJECTED RT`로 변경했다. 폐기된
`coordinate_contract_expected_tilt_zero=forward` 필드도 제거했다.

첫 구현에서 360° 스캔 전체를 분모로 사용한 20% overlap 기준은 단일 카메라 FOV의
정상 후보까지 모두 제거했다. 이를 edge 100개 이상과 NID 200개 이상의 절대 투영 수
조건으로 교정했다. 교정 후 32,760개 중 28,856개 후보가 gate를 통과했다.

| 후보 | down | yaw | raw objective | edge in-frame | NID projected |
|---|---:|---:|---:|---:|---:|
| 최종 오답 후보 | 43° | 68° | 0.89948 | 2,013 | 205 |
| 실제 방향 주변 | 2° | -90° | 0.93009 | 123 | 202 |

실제 방향 후보도 검색되고 gate를 통과했으나 목적함수가 오답 후보를 더 유사하다고
평가했다. 따라서 현재 주원인은 1° 탐색 누락이나 이웃 후보 보정이 아니라 단일 장면의
NID/edge 목적함수 식별력 부족이다. 결과는 `OVERLAP_INSUFFICIENT` 진단 전용이며
RT에 활성화되지 않는다. 다음 구현은 목적함수와 시각화에 동일한 z-buffer 가시성
처리를 적용하고, plane intersection/긴 구조선 대응을 추가하는 것이다.

추가로 NID 20%/edge 80% 진단 검색을 수행했으나 선택 방향은 `down=18도,
yaw=127도`로 이동했고 실제 방향으로 수렴하지 않았다. 따라서 단순 가중치 조정으로
해결할 수 없으며, 양 센서에서 공통으로 관측되는 구조선과 가시 표면을 명시적으로
대응시켜야 한다.

## 10.14 JSON 변경 우선순위와 목적함수 수정 설계 (2026-08-12)

schema 1.2의 추가 메타데이터는 향후 권장사항이며 현재 수정의 선행조건이 아니다.
기존 130333 schema 1.1 JSON은 원본을 변경하지 않고 legacy range offset 입력과 현재
계약각으로 계속 사용한다. 이번 오정합을 해결하기 위해 JSON에 임의의 160도 보정이나
pan zero offset을 추가하지 않는다.

### A. 후보별 z-buffer 가시성

1. coarse RT 후보로 전체 LiDAR 점을 카메라에 투영한다.
2. 영상 해상도의 depth buffer에서 픽셀별 최소 camera-Z를 계산한다.
3. 최소 깊이보다 10 mm 이상 뒤인 점을 가림점으로 제외한다.
4. 남은 가시 구조점으로 edge/NID/coverage를 다시 계산한다.
5. CSV에 `visible`, `occluded`, `edge_score`, `nid_score`, `coverage_penalty`를 분리 저장한다.

Ceres 내부에서 매 잔차 평가마다 z-buffer를 바꾸면 목적함수가 불연속이 되므로 사용하지
않는다. refinement는 현재 RT에서 가시 집합을 고정해 Ceres를 실행하고, 종료 후 z-buffer를
다시 만든다. 이를 2~3회 외부 반복하며 RT 변화가 작으면 종료한다.

### B. 2D/3D 구조선 대응

1. 카메라 영상은 OpenCV LSD로 선분을 검출한다.
2. 짧은 선, 글자/라벨 밀집 영역, 약한 선을 제거하고 길이·방향·교차점 정보를 보존한다.
3. organized LiDAR cloud는 normal 기반으로 주요 평면을 분리한다.
4. 지지점이 충분하고 서로 평행하지 않은 평면 쌍의 교차선을 3D 구조선으로 만든다.
5. 후보 RT로 3D 선분 양 끝점을 투영한다.
6. 투영선과 2D 선의 수직거리, 방향차, 겹치는 길이를 잔차로 계산한다.

초기 결합 목적함수는 다음 형태로 시작하되 합성/hold-out 데이터로 가중치를 결정한다.

```text
J = w_edge * J_edge_visible
  + w_nid  * J_nid_visible
  + w_line * J_line
  + w_cov  * J_coverage
  + w_prior * J_mechanical_prior
```

구조선 대응 수가 부족하면 `J_line`을 억지로 사용하지 않고
`STRUCTURAL_LINE_INSUFFICIENT` 진단을 기록한다. 실제 CH1 방향 주변 후보와 현재 오답
후보의 항목별 점수를 먼저 비교한 후에만 기본 가중치를 변경한다.

### 10.14.1 1차 구현 반영 (2026-08-12)

- coarse 후보별 축소 해상도 z-buffer를 만들고 edge/NID를 가시 포인트만으로 평가한다.
- 선택 후보의 visible 집합을 고정해 Ceres refinement에 사용하고, 최종 RT 평가는
  z-buffer를 다시 생성해 수행한다.
- OpenCV LSD의 장거리 2D 선분 distance transform과 LiDAR normal/range 변화가 큰
  구조 포인트 사이의 구조선 비용을 목적함수에 추가했다.
- `orientation_full_search.csv`에 edge/NID/structure-line 개별 비용과
  visible/occluded 포인트 수를 기록한다.
- 명시적 평면 교차선의 방향 및 겹침 길이 항은 아직 추가하지 않았다. 1차 결과에서
  CH1 실제 방향 식별력이 부족할 때 다음 단계로 적용한다.

15° yaw/down 진단 검색 결과, z-buffer와 개별 점수 출력은 정상 동작했으며 회귀 테스트
5/5를 통과했다. 그러나 선택 진단 후보는 `(down=0°, yaw=30°)`이고 viewer heading은
약 `120.9°`로 실제 CH1 방향과 여전히 불일치했다. 결과는
`SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / OVERLAP_INSUFFICIENT`로 비활성화했다.

이 결과는 단순한 구조 변화 포인트-to-LSD-line 거리만으로는 평행한 책상·캐비닛·벽 선을
구별하지 못한다는 것을 보여준다. 다음 단계에서는 구조선 항을 주요 평면 분할 → 평면
교차 3D 선분 → 2D 선분과의 방향 차이 및 겹침 길이 비용으로 교체해야 한다. 또한 현재
coarse z-buffer의 동일 픽셀 충돌이 적어 edge occlusion 수가 거의 0이므로, 가시성 효과는
edge 표본보다 dense geometry/NID에서 주로 검증해야 한다.

### 10.14.2 CH1 설치 방향 prior 및 재투영 방향 회귀시험 (2026-08-12)

설치 사진에서 CH1 렌즈 중심은 LiDAR 축의 `+X` 쪽에 있고 광축도 바깥쪽 `+X`를
향한다. 기존 코드는 렌즈 중심 offset을 translation 초기화에만 사용하여, 반복 구조가
더 높은 오답 점수를 만들면 반대 방향도 선택할 수 있었다.

`--camera-outward-facing true`를 지정하면 수평 camera center 벡터를 정규화하여 기대
광축으로 사용하고 다음 약한 기구 prior를 coarse 목적함수와 Ceres refinement에 동일하게
적용하도록 수정했다.

```text
J_direction = (1 - dot(camera_forward_lidar, expected_forward_lidar)) / 2
```

CH1에는 `expected_forward_lidar=(+1,0,0)`, weight `0.35`를 적용했다. 실제 방향의
`yaw=-90°` 후보가 기존 NID 최소 200점 gate에서 179점으로 탈락하던 문제는 단일 카메라
FOV에 맞춰 최소 100점으로 조정했다.

5° yaw / 15° down 재시험 결과:

| 항목 | 수정 전 | 수정 후 |
|---|---:|---:|
| 선택 yaw | `+30°` | `-90°` |
| viewer top-view heading | `120.9°` | `0.57°` |
| 기대 CH1 방향 | `+X` | `+X` |
| 방향 일치 | 아니오 | 예 |

합성 회귀 테스트에는 복구된 camera forward와 기대 방향의 내적이 `0.8`보다 커야 한다는
조건을 추가했고 전체 CTest 5/5를 통과했다. 실데이터는 단일 관측이고 projected ratio가
낮으므로 계속 `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / OVERLAP_INSUFFICIENT`이며 RT를
활성화하지 않는다. 이번 변경은 재투영 방위를 설치 사진과 일치시킨 것이며 픽셀 단위 RT
정확도를 증명한 것은 아니다.

### 10.14.3 설치 사진 재검토 및 물리 RT 정정 (2026-08-12)

추가 설치 사진과 사용자가 표시한 Top-view를 재검토한 결과, 앞서 사용한
`camera center offset 방향 = optical axis 방향` 가정은 잘못됐다. CH1 렌즈 중심은
LiDAR 축에서 `(0.05928,-0.08305,0) m`에 있지만 광축은 별도로 LiDAR `-Z`, 영상 아래
방향은 LiDAR `+Y`를 향한다. viewer Z-up에서는 광축이 Top-view 아래쪽이다.

수정 내용:

1. camera center, camera forward, camera down을 서로 독립된 설치 제약으로 분리했다.
2. coarse yaw 후보마다 고정된 물리 중심 `C`로 `t=-R*C`를 다시 계산한다.
3. Ceres translation prior를 `t` 성분 차이가 아니라 `C=-R^T*t`의 실측 중심 오차로
   변경했다.
4. 광축과 영상 아래 방향을 함께 목적함수에 넣어 광축 주위 roll 대칭을 제거했다.
5. Ceres 회전 refinement는 선택된 coarse 후보를 기준으로 ±10° 안에서만 수행한다.
6. LSD 구조선을 8개 방향 bin으로 분리하고, 투영된 LiDAR 구조 접선과 방향이 맞는
   2D 선분에만 Chamfer 비용을 계산한다.
7. projected ratio 분모를 360° 전체 edge가 아니라 현재 후보의 visible edge로 정정했다.

5° yaw/down 재실행 결과:

| 물리 검증 항목 | 입력/기대 | 결과 |
|---|---:|---:|
| camera center X | `0.05928 m` | `0.059233 m` |
| camera center Y | `-0.08305 m` | `-0.083075 m` |
| camera center Z | `0 m` | `-0.000085 m` |
| camera forward | `(0,0,-1)` | `(0.0628,-0.0365,-0.9974)` |
| camera down | `(0,1,0)` | `(-0.0142,0.9992,-0.0375)` |
| viewer Top-view heading | `-90°` | `-86.40°` |

방향과 설치 중심은 사진/실측과 일치한다. 다만 130333은 단일 장면이고 제조사 FOV
기반의 고정 K를 사용하므로 픽셀 단위 정확도 및 자동 활성화 조건은 아직 충족하지
못했다. 결과는 `installation_constrained_rt.json`에 별도 저장하며 상태는
`DIAGNOSTIC_NOT_ACTIVE`다. 자동 RT 활성화에는 동일 설치에서 구조가 다른 동기 관측
3쌍 이상과 실제 카메라 profile/LDC 검증이 필요하다.

### 10.14.4 Top X-Y 개선 과정과 Front X-Z 잔여 오차 분석 (2026-08-13)

#### A. Top X-Y가 설치 사진과 유사해진 과정

Top X-Y 개선은 search step 하나의 효과가 아니라 다음 오류를 순서대로 제거한 결과다.

1. JSON의 실제 좌표 계약을 사용했다.
   `x=r*cos(tilt)*sin(pan)`, `y=-r*sin(tilt)`,
   `z=r*cos(tilt)*cos(pan)`을 읽고 viewer에는 `(x,z,-y)`로만 변환했다.
2. 실측 pan 증가 방향이 Top-view 시계 방향임을 좌표 해석에 반영했다.
3. 카메라 중심과 광축을 분리했다. 중심은 `(0.05928,-0.08305,0) m`, CH1 광축은
   LiDAR `-Z`로 설정했다.
4. coarse 후보가 회전할 때 카메라 중심이 함께 이동하던 오류를 수정했다. 각 후보마다
   동일한 물리 중심 `C`를 유지하며 `t=-R*C`로 다시 계산한다.
5. Ceres에서도 단순 `t` 차이가 아니라 `C=-R^T*t`가 실측 중심에서 벗어나는 정도를
   5 mm sigma로 제약했다.
6. 후보별 z-buffer로 가시점만 평가하고, 선택 후보 주변 ±10°에서 연속 RT를
   refinement했다.
7. 2D LSD 구조선과 LiDAR 구조 접선을 8개 방향 bin으로 나눠 같은 방향의 구조만
   비교했다.

그 결과 viewer Top-view heading은 기대 `-90°`에 대해 `-86.40°`, 카메라 중심 오차는
약 `0.10 mm`가 되어 사용자가 표시한 CH1 방위와 거의 같아졌다. 이 결과가 Top X-Y
화살표와 투영 영역이 매우 유사해진 직접적인 이유다.

#### B. Front X-Z가 책상 edge와 맞지 않는 이유

최종 검색은 이미 `yaw-step=5°`, `down-step=5°`로 수행했다. down별 최상 점수는 다음과
같았으며, `0°`가 최저였다.

| down | best yaw | raw objective | edge | NID | directional line |
|---:|---:|---:|---:|---:|---:|
| 0° | -170° | 0.8246 | 0.4806 | 0.9643 | 0.7942 |
| 5° | -170° | 0.8541 | 0.5507 | 0.9687 | 0.8044 |
| 10° | -165° | 0.8694 | 0.4752 | 0.9680 | 0.7715 |
| 15° | -165° | 0.9308 | 0.5098 | 0.9602 | 0.8707 |

따라서 현재 Front X-Z 오차는 **5° 후보 간격이 커서 올바른 down을 건너뛴 현상으로
보기 어렵다.** 선택 후보 이후 Ceres가 연속 각도로 광축을 조정했고 최종 광축의 수직
성분도 약 `-2.1°`다. 1° full search는 확인 시험으로 의미가 있지만 근본 해결책은 아니다.

현재 더 큰 원인은 다음과 같다.

1. 단일 장면에서 수직 구조의 식별력이 부족하다. 벽·캐비닛·책상의 평행한 edge가 많아
   서로 다른 높이에서도 비슷한 점수를 만든다.
2. 현재 structure-line cost는 선의 방향과 점-to-line 거리는 보지만, **같은 3D 선분과
   같은 2D 선분의 양 끝점 및 겹침 길이**를 직접 대응시키지 않는다.
3. 제조사 FOV 중앙값으로 만든 `fy=2163.97`, `cy=760`을 고정하고 있다. 실제 zoom/focus,
   채널 crop 또는 LDC profile과 다르면 수직 위치/스케일 오차가 Front X-Z에서 크게
   나타난다.
4. `LDC=unknown`이라 영상이 rectified인지 raw distortion인지 확정되지 않았다.
5. LiDAR의 책상 edge는 range/normal 변화 포인트 집합일 뿐, 하나의 연속된 3D 책상
   선분으로 모델링되지 않았다.

#### C. 권장 수정 및 시험 순서

1. **5°→1° 국소 시험:** 전체 360×90을 다시 계산하기보다 현재 후보 주변
   `yaw=-180~-160°`, `down=0~10°`를 1°로 탐색한다. step 민감도 확인용이다.
2. **수직 residual 분리 출력:** 후보마다 horizontal/vertical line cost, 책상 높이 방향
   residual, projected vertical coverage를 별도 CSV로 기록한다.
3. **3D 선분 추출:** plane segmentation으로 벽-바닥, 벽-책상 또는 책상 상판 경계의
   3D 양 끝점을 만든다.
4. **2D-3D 선분 대응:** 투영선과 LSD 선의 수직거리뿐 아니라 방향 차이, 양 끝점 거리,
   겹침 길이를 함께 최소화한다.
5. **카메라 profile 확인:** 실제 CH1 stream resolution/zoom/focus/LDC 상태와 동일한
   Manual ChArUco K+D를 고정한다. manufacturer profile 후보는 민감도 진단으로만 남긴다.
6. 위 조건에서 구조가 다른 동기 관측으로 R,t를 추정하고, 사용하지 않은 hold-out으로 검증한다.

판정 기준은 Top X-Y 방향 일치만이 아니라 Front X-Z에서 책상/캐비닛의 동일 edge가
겹치고, hold-out 장면에서도 같은 RT가 유지되는 것이다.

### 10.14.5 권장 순서 구현 및 CH1 실행 결과 (2026-08-13)

130333 CH1에 대해 위 순서를 코드와 시험에 반영했다. 이번 결과도 reference RT가 없는
단일 관측이므로 제품 RT가 아니라 진단 후보이며 상태는
`SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL`이다.

#### 구현 변경

1. `--yaw-min-deg`, `--yaw-max-deg`를 추가해 전체 360° 재탐색 없이 국소 yaw 범위를
   지정할 수 있게 했다.
2. organized scan에서 인접 range discontinuity가 4점 이상, 0.15 m 이상 연속되는
   구간을 3D 선분으로 추출했다.
3. 카메라는 LSD의 실제 2D 선분 양 끝점을 보존한다.
4. 새 구조선 비용은 방향 차이 0.25, 끝점의 선까지 거리 0.40, 유한 구간 겹침 부족
   0.35를 합산한다. 이 비용은 coarse 점수뿐 아니라 Ceres refinement에도 연결했다.
5. `orientation_full_search.csv`에 수평/수직 구조선 비용과 각 방향의 평가 선분 수를
   분리 기록한다.
6. `--focal-scale`, `--principal-y-offset-px`를 추가해 manual intrinsic 없이 제조사
   FOV K 주변의 제한된 profile 민감도 시험을 할 수 있게 했다.
7. `04b_lidar_structural_segments.{ply,obj}`로 추출된 3D 선분을 선분별 색상과 실제
   edge 연결로 확인할 수 있게 했다.

#### 1° 국소 시험

시험 범위는 yaw `-180~-160°`, down `0~10°`, 두 축 모두 1°이며 총 231개 후보다.

| 항목 | 결과 |
|---|---:|
| 보정 basin down / yaw | `0° / -171°` |
| raw best down / yaw | `1° / -171°` |
| viewer Top heading | `-83.26°` |
| 기대 Top heading | `-90°` |
| 추출 3D 구조선 | 282 |
| 검출 2D LSD 구조선 | 123 |
| 최종 화면 내 구조선 | 11 |
| 최종 수평 / 수직 구조선 | `0 / 11` |
| 최종 composite objective | 0.74412 |
| NID 개선률 | -4.43% |

Top 방위는 기대 방향 주변을 유지했지만 NID는 오히려 악화되어 후보 gate는
`NID_IMPROVEMENT_INSUFFICIENT`다. 특히 Front X-Z의 책상 높이를 직접 제약해야 할
수평 선분이 최종 후보에서 하나도 남지 않았고, coarse 최상점 주변에서도 대부분
수평 1개 대 수직 9~12개였다. 따라서 5° 간격이 주원인이라는 가설은 기각된다.
현재 front 오차의 직접 원인은 **FOV 안에 투영되어 가시성 검사를 통과한 연속 수평
3D 구조선이 부족하여 수직 구조선이 목적함수를 지배하는 것**이다.

#### 제한 K profile 시험

제조사 FOV 중앙값 K에 focal scale `{0.9,1.0,1.1}`, `cy` offset
`{-76,0,+76}px`를 적용한 3×3 진단을 동일 1° 국소 범위에서 수행했다.

| focal scale | cy offset | down / yaw | objective | Top heading |
|---:|---:|---:|---:|---:|
| 1.1 | +76 px | `0° / -174°` | **0.72171** | -84.75° |
| 1.0 | +76 px | `0° / -172°` | 0.73083 | -81.99° |
| 1.1 | 0 px | `0° / -174°` | 0.73258 | -85.04° |
| 1.0 | 0 px | `0° / -171°` | 0.74412 | -83.26° |

점수는 K profile에 민감하지만 가장 낮은 점수도 수평 구조선이 0개이고 reference RT가
없다. 따라서 `focal=1.1, cy=+76`을 실제 K로 선택하지 않는다. 이 시험은 zoom/focus와
principal point 불확실성이 front 투영 오차에 영향을 준다는 증거이지 K 식별 결과가
아니다.

#### 산출물과 다음 게이트

- 국소 시험: `generated/real_session_const_20260811_ch1_local1deg_segments/`
- K profile 3×3: `generated/real_session_const_20260811_ch1_k_profile/`
- 점수 분해: 각 결과의 `orientation_full_search.csv`
- 3D 구조선: `debug/scene_0/04b_lidar_structural_segments.{ply,obj}`

현 historical 데이터에는 CH1 image–scan 동기 관측이 1쌍뿐이라 외부 RT 검증이 불가능했다.
현재는 Manual K+D를 고정하고 동일 설치/채널에서 책상 edge가 서로 다른 영상 위치와 깊이에
나타나는 image+JSON을 수집해 R,t를 추정한 후 hold-out 투영을 수행한다. 다음
수집에서는 카메라 FOV 안에 벽–책상 상판 경계가 충분히 길게 들어오고 LiDAR에서도 그
경계가 연속 range discontinuity로 보이는 구도를 우선한다.

### 10.14.6 3D 구조선 정의 수정 (2026-08-13)

#### 확인된 구현 불일치

이전 `04b_lidar_structural_segments`는 surface normalization 결과에서 두 평면이
맞닿는 선을 추출하지 않았다. 실제 구현은 organized scan의 인접 cell 사이 range가
`max(80 mm, 3%)`보다 급변하는 위치를 이어 붙였다. 따라서 282개 선분 대부분이 짧은
장애물 실루엣·폐색 경계였으며, “벽-바닥/벽-책상 평면 교차선”이라는 문서 설명과
일치하지 않았다. 282개 중 210개가 0.5 m 미만이고 1 m 이상은 17개뿐이었던 것도 이
구현 특성과 일치한다.

#### 수정된 처리 계약

1. range discontinuity 반대편 점을 사용하지 않는 one-sided normal을 계산한다.
2. normal 각도와 이웃의 접평면 거리를 이용해 organized grid를 region growing한다.
3. 최소 지지점·두 축 extent·PCA point-to-plane RMS 조건을 통과한 영역만 평면으로
   승인한다.
4. 경계의 미분류 normal 띠를 건너뛸 수 있도록 organized grid 3-cell 반경에서 승인
   평면 쌍을 찾고 수학적 교차선을 계산한다.
5. 중복을 제거한 관측 경계점 중 교차선 100 mm 이내의 지지점이 8개 이상이고, 그
   inlier로 자른 길이가 150 mm 이상일 때만 구조선으로 사용한다.
6. raw range discontinuity는 `occlusion edge`로 이름을 바꾸고 진단 파일에만 저장한다.
   Calibration의 구조선 목적함수에는 평면 교차선만 입력한다.

새 중간 산출물은 다음과 같다.

- `04_lidar_surface_normals.ply`: 실제 평면 분할과 동일한 robust normal
- `04a_lidar_plane_labels.ply`: 평면별 색상, 회색은 승인되지 않은 점
- `04b_lidar_plane_intersection_edges.{ply,obj}`: 승인 평면 교차선
- `04c_lidar_occlusion_edges.{ply,obj}`: 폐색/거리 급변 실루엣, 진단 전용
- `04d_lidar_edges_used_for_calibration.{ply,obj}`: 목적함수 입력(현재 `04b`와 동일)

합성 회귀 테스트에는 직교한 두 평면에서 교차선 1개가 생성되는 경우와, 떨어진 두
평행면에서 교차선은 생성되지 않고 폐색선만 생성되는 경우를 추가했다. Docker CMake
빌드와 전체 CTest는 5/5 PASS했다.

동일 130333 CH1을 `yaw=-180~-160°`, `down=0~10°`, 두 축 1°로 재실행한 결과는 다음과
같다.

| 항목 | 수정 후 결과 |
|---|---:|
| 유효 point / normal | 40,307 / 37,808 |
| 승인 평면 | 22 |
| 평면 라벨 승인 point | 13,517 (33.5%) |
| 평면 쌍 후보 | 11 |
| 승인 평면 교차선 | **1** |
| 폐색선(진단 전용) | 282 |
| 화면 내 구조선 | 0 |
| 선택 down / yaw | `0° / -171°` |
| NID 개선률 | -4.11% |
| 최종 상태 | `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL` |

승인된 교차선은 평면 12와 16의 88.83° 교차이며, 100 mm 이내 고유 지지점 13개,
길이 0.324 m다. 전체 3-cell 주변점의 p75 거리는 0.339 m였으므로 모든 주변점의 75%를
요구하지 않고 실제 inlier로 선분을 자르는 방식이 필요했다. 평면 0과 1은 88.27°지만
inlier가 2개뿐이라 거절했다.

이 변경으로 282개 장애물/폐색 윤곽이 구조선 목적함수를 지배하던 오류는 제거됐다.
하지만 현재 CH1 시야에는 승인 교차선이 투영되지 않아 구조선 항이 자세를 제약하지
못한다. 따라서 이번 결과는 알고리즘 분리의 성공이지 RT 정합 성공이 아니다. 다음
데이터는 카메라 영상에 보이는 벽-바닥 또는 벽-책상 접촉선이 LiDAR에서도 8점 이상
연속 관측되도록 구도를 잡아야 한다.

산출물은
`generated/real_session_const_20260811_ch1_plane_intersections/`에 있으며, 탈락 근거는
`debug/scene_0/04b_lidar_plane_pair_candidates.csv`에서 확인한다. 이전 282개 구조선
결과와 점수는 새 알고리즘의 검증 결과로 재사용하지 않는다.

### 10.14.7 미분류 점·수평 가구면 복구 (2026-08-13)

`04a_lidar_plane_labels.ply`에서 벽은 검출됐지만 사용자가 표시한 책상 영역 대부분이
회색으로 남았다. 이전 region growing은 최소 점 수를 통과하지 못한 연결 성분을
`visited` 상태로 버려 이후 단계에서 다시 사용할 수 없었다. 높이 분포를 확인한 결과
`y=1.05~1.18 m`에 수직 normal을 가진 점이 수백 개 있어 책상 상판 후보 데이터 자체는
존재했다.

다음 후처리를 추가했다.

1. 승인 평면에 인접한 회색 점을 normal 각도와 point-to-plane 40 mm gate로 반복
   재할당한다.
2. grid에서 서로 이웃하고 normal·offset이 같은 공면 조각을 병합한다.
3. 스캔 전 IMU 수평 gate로 `+Y down`이 보장된다는 ICD를 이용해 수직 normal 점을
   높이별로 묶고, PCA extent/RMS 조건을 다시 적용한다.
4. 수평면이 기존 라벨을 흡수한 뒤 80점 미만으로 남은 잔여 평면은 제거한다.

동일 130333 CH1 국소 1° 시험 결과는 다음과 같다.

| 항목 | 이전 | 복구 후 |
|---|---:|---:|
| 승인 평면 | 22 | 13 |
| 평면 라벨 point | 13,517 (33.5%) | 20,474 (50.8%) |
| 평면 쌍 후보 | 11 | 21 |
| 승인 평면 교차선 | 1 | 6 |
| 폐색선 | 282 | 282 |
| 화면 내 구조선 | 0 | 0 |

복구된 수평면 후보에는 `y=1.123 m` 219점, `y=1.071 m` 133점,
`y=1.372 m` 91점이 있다. 앞의 두 평면은 책상 높이 후보지만 semantic label을 사용하지
않으므로 설치 사진·라벨 PLY와 함께 확인해야 한다. 수직면과 이 후보들의 승인 교차선은
늘었지만 선택 RT에서 카메라 화면 안에 들어온 구조선은 여전히 0이다. 따라서 이번 변경은
3D 구조 추출 개선이며 2D–3D 자동 RT 성공을 의미하지 않는다.

합성 시험에는 최소 점 수보다 작은 공면 조각 복구와 모든 region이 탈락한 수평면의
높이 복구를 추가했고 Docker 전체 CTest 5/5가 통과했다. 최종 산출물은
`generated/real_session_const_20260811_ch1_horizontal_recovery/`에 저장했다.

### 10.14.8 실제 설치 방향 기반 3D 재투영 (2026-08-13)

기존 FAIL 산출물은 설치 방향 prior가 입력되어도 최종 3D 색상과 카메라 화살표에
`candidate_t_camera_lidar`를 사용했다. 130333 CH1 후보 광축은
`(0.117,-0.050,-0.992)`로 실제 입력 광축 `(0,0,-1)`과 약 7.4° 차이가 있었다.

카메라 중심 `C`, 광축 `f`, 영상 아래 `d`가 명시되면 `right=d×f`로 카메라 frame을
직접 구성하고 `t=-R·C`를 계산하도록 변경했다. CH1 설치 계약으로 얻은 값은
`R=diag(-1,1,-1)`, `t=(0.05928,0.08305,0) m`이며, 역변환 검증 결과 중심·광축·down이
입력값과 정확히 일치한다.

FAIL 상태의 최적화 후보는 계속 비활성 진단값으로 보존하지만, 2D projection,
colorized PLY/OBJ, 3-view preview와 debug `06/07`은 설치 계약 RT를 사용한다. Viewer
mesh에는 청록색 카메라 중심/1 m 광축 마커를 넣었다. 새 CH1 결과는 1,479점을 화면에
투영했고 Top-view heading은 `-90°`다. 전체 CTest 5/5가 통과했다.

산출물: `generated/real_session_const_20260811_ch1_installed_reprojection/`

Viewer mesh 단위도 정리했다. `*_viewer_mesh.obj/.ply`는 VS Code clipping 범위에 맞춰
미터로 저장하고 OBJ는 외부 `.mtl` 없이 형상만 제공한다. 색상이 필요하면 PLY를 사용하며,
일반 `scene_0_colorized_lidar.obj/.ply`는 밀리미터를 유지한다.
교차선 `04b_lidar_plane_intersection_edges.ply`는 원래부터 미터 단위다.

### 10.14.9 설치 prior 분리 및 down/optical-roll 복구 (2026-08-13)

#### 원인 정정

`real_session_const_20260811_ch1_installed_reprojection`은 FAIL 후보 대신
`forward=(0,0,-1)`, `down=(0,1,0)` 설치 RT를 일반 결과에 표시했다. 따라서 yaw가
근사해 보였지만 광축 하향각은 강제로 0°였고, 이는 책상 윗면을 내려다보는 CH1 영상과
모순된다. 이 파일은 자동 정합 결과가 아니라 mechanical prior였다.

일반 투영은 다시 최적화 후보 전용으로 돌리고 mechanical prior는 별도 접두사 파일로
분리했다. 두 결과 모두 FAIL/진단 표시를 유지하며 어느 쪽도 활성 RT가 아니다.

#### 구조선 0건의 직접 원인과 수정

3D 평면 교차선의 fitted endpoint가 카메라 화면 밖에 있으면 실제 선이 화면을 가로질러도
전체를 폐기했다. 선분을 33점으로 샘플링한 뒤 화면 안의 가시 구간만 평가하도록 수정해
CH1에서 구조선 0개가 최대 2개로 복구됐다. 복구선은 모두 벽–수평면 계열 수평선이고
수직선은 0개다. 구조선 목적함수가 활성인데 최종 가시 구조선이 0개이면
`STRUCTURAL_OVERLAP_INSUFFICIENT`로 거절하도록 PASS gate도 추가했다.

#### 5° coarse와 1° fine 결과

| 항목 | 5° coarse | 1° fine |
|---|---:|---:|
| down | 15° | 14° |
| optical roll | 15° | 17° |
| yaw | -165° | -169° |
| focal scale | 1.0 | 1.0 |
| 화면 내 구조선 | 2 | 2 |
| edge mean | 33.38 px | 34.07 px |
| NID 개선률 | -1.22% | -1.89% |
| 결과 | FAIL | FAIL |

coarse는 down `-30~30°`, roll `-15~15°`, focal `0.9/1.0/1.1`을 비교했고,
fine은 coarse 주변 down/roll/yaw를 1°로 좁혔다. focal 0.9/1.0/1.1의 최저 인접후보
보정 점수는 각각 `0.728239/0.714862/0.718370`이다. down 0° 고정 문제는 제거됐지만
NID가 개선되지 않았으므로 현재 `14°/17°/-169°`는 정답 RT가 아니라 다음 수집 범위를
정하는 진단 후보다.

산출물:

- `generated/real_session_const_20260811_ch1_candidate_prior_split/`
- `generated/real_session_const_20260811_ch1_vertical_coarse_5deg/`
- `generated/real_session_const_20260811_ch1_vertical_fine_1deg/`

### 10.14.10 지금까지 시도한 내용 누적 요약 (2026-08-13)

이번 프로젝트에서 실제로 수행한 시도와 판정을 다음처럼 고정 기록한다.

| 순서 | 시도 | 결과 | 최종 해석 |
|---:|---|---|---|
| 1 | Docker/CMake/CTest 개발환경과 Core 실행 | CTest 5/5 PASS | 코드 회귀는 통과했지만 실센서 정확도와 별개 |
| 2 | Stanford 단일·다중 장면 synthetic 검증 | 단일 FAIL, 다중 5장면 PASS | 다중 관측과 ground truth가 있으면 기본 최적화 가능 |
| 3 | session-001~003 실제 데이터와 이미지 flip/회전·200/400 bps 비교 | 물리 투영 불일치, 수치 PASS 일부 폐기 | false PASS 방지 필요 |
| 4 | 고정환경 130333 CH1~CH4 반복 기준 확보 | 채널별 한 쌍뿐 | 재현성 기준은 얻었으나 K–RT 식별 불가 |
| 5 | JSON ICD 좌표식·tilt 의미·range offset 정정 | adapter와 producer 식 일치 | `tilt_zero=nadir`는 기구 home으로만 유지 |
| 6 | 천장 설치 Z-up 변환·camera center/광축 marker·OBJ/PLY 단위 수정 | Top-view 방향 확인 가능 | 시각화 단위와 계산 단위 분리 |
| 7 | manual K 배제, 제조사 FOV K 초기화, LDC unknown 기록 | 자동 취지 유지 | 실제 channel FOV/zoom/focus/LDC 미확정 |
| 8 | edge-only → NID+edge 복합 비용 | 대칭·반복 구조에서 yaw 오선택 지속 | 구조 정보가 추가로 필요 |
| 9 | yaw 45° → 15°/5° coarse, 1° 국소·360×90 baseline | reference RT가 없어 conformance 보류 | 계산량 시험보다 정답 기준 선행 |
| 10 | B 방식 인접 8후보 Gaussian 보정+contiguous basin | 연속 고득점 영역 반영 | raw 최저점보다 안정적이나 정답 보장 아님 |
| 11 | range edge를 평면 분할·교차선 기반 구조선으로 교체 | 폐색선 282개를 calibration에서 제외, 교차선 1→6 | 구조선 의미는 개선됐으나 가시선 부족 |
| 12 | 미분류점 재할당·공면 병합·IMU-Y 수평면 복구 | 라벨 point 33.5%→50.8% | 책상 높이 후보가 보존됐지만 semantic 확정 아님 |
| 13 | FAIL 후보 RT와 mechanical prior 투영 분리 | 산출물 출처 혼동 제거 | FAIL 결과는 제품 RT가 아님 |
| 14 | 교차선 33점 visible-segment sampling | 화면 내 구조선 0→최대 2 | endpoint 밖 문제는 수정, 수직선은 여전히 부족 |
| 15 | direction prior=0의 down/pitch·optical roll 독립 탐색과 focal profile | coarse `(15°,15°,1.0)`, fine `(14°,17°,1.0)` | NID 악화로 모두 FAIL |
| 16 | focal scale 0.9/1.0/1.1 민감도 | 1.0 보정점수 최저 | 실제 K 확정 증거가 아니며 단일 관측 K–RT 결합 문제 존재 |

#### 현재 유효한 결론

1. 현재 LiDAR 좌표 변환은 JSON `frame.range_formula`와 `measurements[].tilt_rad`를
   기준으로 한다. `sensor.range_offset_m`가 없는 schema 1.1은 실행 인자를 요구한다.
2. 이전의 공통 `prior-roll=90°`, nadir 기준 계약각 해석, 폐색선 전부를 구조선으로 쓰는
   방식, FAIL 후보를 최종 투영에 쓰는 방식은 폐기했다.
3. 130333 CH1의 수평 yaw는 근사할 수 있지만 세로 투영은 down/roll·K·LDC·구조선 부족이
   얽혀 있다. 현재 탐색값은 `down≈14~15°`, `roll≈17°`이며 정답 RT가 아니다.
4. 모든 130333 단일 관측 결과는 `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL`이다.
   내부 objective가 낮아도 실제 물리 정합 또는 conformance PASS로 해석하지 않는다.

#### 남은 승인 조건

- 동일 고정환경 CH1의 서로 다른 구조 관측 image+JSON 최소 3쌍
- 실제 channel FOV/zoom/focus 및 LDC 상태 확인
- 벽–책상/벽–바닥 경계가 2D와 LiDAR 양쪽에서 충분한 길이로 보이는 데이터
- reference RT 또는 독립 측정값을 확보한 후 1° full-search·coarse step benchmark 수행

### 10.14.11 실측 센서 수직 순서 반영 CH1 재실행 (2026-08-14)

설치 측정에서 지면 기준 센서 순서가 **LiDAR → 카메라**임을 확정했다. 천장 기준
LiDAR 635 mm, 카메라 551.95 mm이므로 카메라는 LiDAR보다 83.05 mm 위에 있다.
JSON frame의 `+Y=down` 계약을 적용하면 카메라 중심은 다음과 같이 표현된다.

```text
C_L = (+0.05928, -0.08305, 0) m
```

이 값을 `--camera-center-x/y/z-m`으로 넣고 CH1 조명 켜짐 repeat sample을 5° yaw/down
coarse로 재실행했다. 기본 `baseline=0.28 m`는 사용하지 않았고, LDC는 `unknown`,
legacy range offset은 `0.084 m`로 기록했다.

| 항목 | 결과 |
|---|---|
| 출력 | `generated/repeat_test_sample_20260814/bright_center_constrained/` |
| camera center | `(0.05928, -0.08305, 0) m` |
| 후보 수 | yaw/down 1,368개 (5° grid) |
| contiguous 선택 | `down=80°`, yaw `165°` (grid 좌표) |
| 최종 상태 | `FAIL / MULTISTART_AMBIGUOUS` |
| 시각화 pose | `REJECTED CANDIDATE` (활성 RT 아님) |

따라서 수직 offset의 부호/위치 입력은 올바르게 반영되었지만, 단일 고정 장면에서
NID·edge·구조선 목적함수가 방향을 유일하게 식별하지 못하는 문제가 남아 있다. 이
실행은 센서 중심 문제와 방향/관측성 문제를 분리하는 진단이며, 결과 RT를 제품에
적용하지 않는다.

### 10.14.12 모델링 치수 재확인 및 81.05 mm 정정 실행 (2026-08-14)

모델링 도면에서 LiDAR 회전 중심선과 CH1 카메라 중심의 수평 거리는 59.28 mm다. 이
수평 성분은 기존 실행에도 `--camera-center-x-m 0.05928`로 이미 반영되어 있었다.
운영자가 수직 차이를 83.05 mm에서 81.05 mm로 정정했으므로 현재 설치 계약은 다음과
같다.

```text
LiDAR frame = +x right, +y down, +z forward
C_L = (+0.05928, -0.08105, 0) m
|C_L| = 0.100415 m
```

이 값은 각 방향 후보의 `t=-R*C_L`과 Ceres 5 mm camera-center prior에 사용한다.
geometry-first v4와 같은 조명 ON 5세트·5° yaw/down 조건으로 다시 실행한 결과,
yaw/down 선택은 `165°/20°`로 유지됐고 최종 상태도
`FAIL / NID_IMPROVEMENT_INSUFFICIENT`였다. edge mean은 25.36→26.12 px,
NID 개선률은 -0.610→-0.506%로 변했다. 따라서 2 mm 정정은 올바른 translation과
시차 계산에 필요하지만 현재 방향 식별 실패의 주원인은 아니다.

산출물:

```text
generated/repeat_test_sample_20260814_light_on_geometry_first_5deg_v5_offset_81p05mm/
```

### 10.14.13 Geometry 식별력·수직 구조 보강 및 v9 결과 (2026-08-14)

기존 실패에는 서로 다른 두 문제가 겹쳐 있었다.

1. 전역 geometry NID는 위치를 버린 histogram이라, 반복되는 벽·바닥 분포를 다른 방향에
   놓아도 점수가 비슷했다.
2. 영상 소실점 후보를 지지선 수 상위 3개만 보존해 실제 수직군이 탈락했고, 후속 down
   탐색도 5~20°로 제한되어 실제 수직 증거에 도달하지 못했다.

이를 위해 range/normal 분리 2×2 spatial NID, entropy gate, 평면 경계·반복 폐색 구조선,
2D–3D 선분 1:1 대응, 영상 소실점–LiDAR 중력/벽축 Manhattan 잔차를 추가했다. 소실점은
최대 12개를 유지하고 각 장면의 후보·지지선·중력축 오차를
`03a_manhattan_vanishing_directions.csv`에 기록한다.

v8의 장면 0/2/3/4에서 반복된 수직 후보는 camera direction
`(-0.099, 0.827, 0.553)` 부근이었다. `asin(0.553)=33.6°`이므로 영상은 기존 제한보다
큰 하향각을 지지한다. down 25~40°, yaw 155~185°를 5°로 다시 탐색한 v9 결과는 다음과
같다.

| 항목 | v8 | v9 |
|---|---:|---:|
| grid down | 15° | 30° |
| refinement down | 14.37° | 27.81° |
| 장면별 수직 오차 | 10.23~20.40° | 3.86~7.88° |
| 학습 장면 통과 | 1/4 | 4/4 |
| aggregate edge mean | 약 31.99 px | 35.96 px |
| NID 개선 | 약 +3.8% | +1.7475% |
| 최종 판정 | `PER_SCENE_VALIDATION_FAILED` | `HOLDOUT_VALIDATION_FAILED` |

v9 hold-out의 수직 오차는 7.84°로 정상 범위지만 edge mean이 `40.8389 px`여서 40 px
gate를 0.8389 px 초과했다. 즉 최신 FAIL 원인은 더 이상 수직군 부족이 아니라 독립
scan에 대한 edge 일반화다. 기준을 느슨하게 바꾸지 않고 FAIL을 유지한다.

또한 최종 후보 선택 후 장면별 gate 결과를 다시 덮어써 PASS로 기록하던 실행기 순서
결함을 수정했다. 같은 v8 조건을 재실행하면 candidate gate가 PASS여도 최종 결과는
`PER_SCENE_VALIDATION_FAILED`로 정확히 기록된다.

F2P signal NMI는 거리 제곱·입사각·range-bin median/MAD 보정까지 구현했지만 Manual RT
perturbation에서 worse ratio `0.5833`, median margin `0.000633`으로 식별력이 부족했다.
가중치는 계속 0이며 보조 진단 파일로만 사용한다.

다음 시험은 v9 주변 `down=27~33°`, `yaw=165~171°`의 1° fine grid다. 이후 독립
hold-out을 최소 3쌍으로 늘려 특정 scan 우연인지 구조적 오차인지 구분한다.

### 10.14.14 1° fine 탐색과 공면 range-edge 오검출 수정 (2026-08-14)

v9 주변을 1°로 탐색한 v10은 `yaw=167°`, grid down `28°`, refinement down
`28.6895°`를 선택했다. bounded yaw 구간의 양 끝을 이웃으로 잘못 연결하던 score-map
topology도 수정해 전체 360°일 때만 순환하도록 했다. 학습은 4/4 PASS였지만 hold-out
edge mean이 `40.4238 px`라서 `HOLDOUT_VALIDATION_FAILED`였다.

새 `07a_projection_final_edge_residual.png`에서 hold-out의 p50은 `15.30 px`인데 p90은
`110.01 px`였다. 일부 true edge만 잘 맞고 책상 상판·벽 내부의 많은 점이 큰 오차를
만드는 long-tail 분포였다. raw range gap만 사용해 비스듬한 공면의 완만한 거리 변화까지
edge로 선택한 것이 직접 원인이다.

공통 edge 추출기에 다음을 적용했다.

- 같은 plane label의 인접점 제외
- 호환 normal과 40 mm 이하 상호 접평면 거리인 점 제외
- 현재 gap이 같은 축의 앞뒤 gap보다 기본 2배 이상인 local contrast gate
- invalid ratio 입력 거절 및 실행 리포트에 정책·비율 기록

동일 조건 v11에서는 장면별 LiDAR edge가 약 6.3k에서 2.9k로 줄었다. 선택 방향은
`yaw=167°`, grid down `28°`, refinement down `27.3755°`이고 학습 4/4와 hold-out
1/1이 모두 통과했다. hold-out 주요 값은 다음과 같다.

| 항목 | 값 |
|---|---:|
| visible/aligned edge | 134 / 76 |
| projected ratio | 0.567164 |
| mean edge distance | 37.1998 px |
| geometry NID / active cells | 0.951842 / 7 |
| 구조선 visible/matched | 18 / 18 |
| 수평/수직 대응 | 4 / 13 |
| Manhattan 수직 오차 | 6.8348° |
| 내부 상태 | PASS |

Top-view 광축과 2D 책상/벽 투영도 이전 바닥·반대 방향 오선택보다 물리적으로 일관된다.
그러나 hold-out이 한 장뿐이고 독립 ground truth가 없다. Manual 예비 RT와도 회전
`8.5588°`, translation norm `0.12646 m` 차이가 난다. 그러므로 이 결과는 코드·현재
분할의 최초 실데이터 내부 gate PASS이지 제품 RT/conformance PASS가 아니다. 다음 승인
조건은 구조와 조명이 다른 독립 hold-out 3쌍 이상에서 RT 반복성을 검증하는 것이다.

설정 검증과 실행 metadata를 보완한 최종 코드 v12에서도 같은 RT, 학습 4/4,
hold-out 1/1 PASS가 재현됐다. v11/v12의 CH1 matching 및 3D preview checksum도 동일하다.
재현 산출물은
`generated/repeat_test_sample_20260814_light_on_structural_nid_manhattan_v12_fine1deg_coplanar_edge/`
에 있다.

### 10.14.15 조명 ON RT의 나이트비전 교차검증 — 참고 진단 (2026-08-14)

나이트비전은 현재 제품 요구사항 및 승인 gate 범위가 아니다. 이 절의 FAIL은 조건 변화
민감도를 관찰한 참고 진단이며 조명 ON v12 내부 PASS를 취소하지 않는다.

`repeat_test_sample`의 뒤 5쌍이 조명 OFF/나이트비전임을 운영자가 확인했다. 먼저 해당
5쌍에서 RT를 다시 추정한 v13은 training 1/4, holdout 0/1로 실패했다. 이어 조명 ON
v12 RT를 전혀 변경하지 않고 나이트비전 5쌍 모두에 적용하는 고정 pose 검증 경로를
추가했다.

고정 RT v14도 1/5만 통과했고 나머지는 모두 `EDGE_ALIGNMENT_POOR`였다. 실패 장면의
mean edge distance는 `44.41~54.39 px`로 40 px gate를 넘었다. 조명 ON 대비
나이트비전 Canny edge 수는 평균 `53.8%` 감소했다. v13 거절 후보와 v12 RT 차이는
회전 `3.148°`, 이동 `3.566 mm`로 비교적 작았다.

따라서 참고 실행의 실패는 RT가 완전히 다른 방향으로 이동한 것보다 나이트비전의 2D
edge 분포 변화에 edge-distance gate가 민감한 증상이다. 현재 개발은 이를 수정 대상으로
삼지 않는다. 조명 ON 상태에서 구조가 다른 독립 holdout 최소 3쌍을 확보해 v12 RT의
반복성을 먼저 검증하며, day/night 공통 처리는 후속 선택 과제로 보류한다.

산출물:

```text
generated/repeat_test_sample_20260814_night_vision_fixed_light_v12_rt_v14/
  fixed_pose_validation_result.json
  fixed_pose_scene_validation.csv
  debug/scene_*/07a_projection_final_edge_residual.png
```

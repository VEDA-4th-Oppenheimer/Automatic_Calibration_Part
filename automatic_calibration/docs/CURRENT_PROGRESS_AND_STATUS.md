# Automatic Calibration 현재 진행 현황

작성일: 2026-08-13
최종 수정일: 2026-08-13
범위: Calibration Core, LiDAR JSON 변환, 2D–3D 매칭 진단, 고정환경 130333 CH1 시험

## 1. 프로젝트 목적

actuator가 완성되기 전에도 카메라 영상과 1D LiDAR pan/tilt scan을 이용해 카메라–LiDAR
외부 파라미터(RT)를 검증할 수 있도록 Calibration Core를 개발한다. 최종 목표는
실제 장치에서 반복 가능한 자동 캘리브레이션과 conformance test에 사용할 수 있는
구조다.

현재 결과는 센서·actuator 통합 완료품이 아니며, 단일 관측 결과를 활성 RT로 승인하지
않는 진단 단계다.

## 2. 지금까지 완료한 내용

### 개발 환경과 프로젝트 구조

- `develop/automatic_calibration` 아래 Core, 실행기, 테스트, 문서를 분리했다.
- Ubuntu latest 기반 Docker 개발환경과 CMake/Ninja 빌드 구성을 사용한다.
- WSL의 Docker CLI 통합이 일시적으로 보이지 않는 경우에도 Windows Docker CLI로 동일
  Compose 컨테이너를 빌드·검증할 수 있음을 확인했다.
- 기존 변경사항과 사용자가 보관한 데이터는 유지하고, Calibration Core 관련 파일만
  추가·수정했다.

### 입력과 좌표계

- LiDAR JSON의 frame 계약을 사용한다.

  ```text
  x = r*cos(tilt)*sin(pan)
  y = -r*sin(tilt)
  z = r*cos(tilt)*cos(pan)
  ```

- JSON의 `measurements[].tilt_rad`는 0°=수평, -90°=아래 방향으로 처리한다.
- `mechanism.tilt_zero=nadir`는 기구 홈 메타데이터로만 사용한다.
- `pan` 증가 방향은 실제 장치 측정값에 따라 Top-view 기준 시계 방향으로 기록했다.
- 외부 파라미터는 다음 계약으로 고정한다.

  ```text
  p_camera = R_camera_lidar * p_lidar + t_camera_lidar
  ```

- 130333 레거시 JSON에는 `sensor.range_offset_m`가 없으므로 원본을 수정하지 않고 실행
  시 `--legacy-range-offset-m 0.084`를 명시했다.
- 고정환경의 카메라 중심 prior는 측정값 `x=0.05928 m, y=-0.08305 m, z=0 m`을
  진단용으로 사용했다.

### 카메라 처리

- 제조사 FOV에서 초기 K를 구성한다.
- 자동 경로에서 checkerboard/Charuco 수동 내부 캘리브레이션 값은 사용하지 않는다.
- RGB grayscale, Sobel gradient, Canny edge distance transform을 생성한다.
- LDC 상태는 확인 전까지 `unknown`으로 기록하며, zoom/focus 고정 조건을 결과 metadata에
  저장한다.

### 매칭과 최적화

- LiDAR range discontinuity와 robust surface normal 변화를 NID용 geometry feature로
  계산한다.
- 16×16 soft joint histogram 기반 NID와 edge distance를 복합 목적함수로 사용한다.
- yaw 후보는 360° 범위를 탐색할 수 있고, 실험 시 `--yaw-step-deg 1`과 제한 범위를
  지정할 수 있다.
- down 후보와 yaw 후보를 독립적으로 평가한다. 과거의 공통 `prior-roll=90°` 가정은
  제거했다.
- 후보 선택 시 raw score만 사용하지 않고 인접 8개 후보의 Gaussian 보정 점수와
  contiguous basin을 계산한다.
- Ceres 기반 6-DoF refinement와 z-buffer 가시성 필터를 사용한다.
- 실패 후보는 활성 결과로 내보내지 않고 mechanical prior를 유지한다.

## 3. 구조선 추출 변경

### 이전 문제

기존 `04b_lidar_structural_segments`는 평면이 만나는 선을 추출한 것이 아니라 인접
cell의 range 급변을 선분으로 연결했다. 이 때문에 벽·바닥 경계와 책상·사람·장애물의
폐색 윤곽이 같은 구조선으로 취급되었다.

### 현재 처리

1. range discontinuity를 넘지 않는 one-sided normal을 계산한다.
2. normal 방향, 접평면 거리, grid 연결성을 이용해 평면을 region growing한다.
3. PCA 평면 fitting 후 최소 점 수, extent, RMS 조건으로 평면을 승인한다.
4. 승인 평면에 인접한 미분류 점을 재할당하고 이웃한 공면 조각을 병합한다.
5. IMU 좌표의 `+Y down`을 이용해 분절된 수평면을 높이별로 보완 검출한다.
6. 3-cell 반경 내 승인 평면 쌍을 찾고, 평면 각도와 실제 교차선 근처 inlier를 검사한다.
7. 고유 inlier가 최소 8개이고 선분 길이가 150 mm 이상일 때만 평면 교차선을 구조선으로
   사용한다.
8. range discontinuity 선분은 `occlusion edge`로 분리하여 진단용으로만 저장한다.

Calibration의 구조선 목적함수에는 평면 교차선만 들어간다.

## 4. 검증 결과

### 자동화 테스트

Docker 컨테이너에서 CMake 빌드와 CTest를 수행했다.

```text
100% tests passed, 0 tests failed out of 5
```

추가한 합성 회귀 테스트는 다음을 검증한다.

- 직교한 두 평면에서 평면 교차선 1개 생성
- 떨어진 평행면에서 평면 교차선은 생성하지 않음
- 같은 평행면 구성에서 폐색 진단선은 별도로 생성
- 최소 점 수보다 작은 공면 조각의 재할당·병합
- region growing이 모두 탈락한 수평면의 IMU-Y 높이 복구

### 고정환경 130333 CH1

입력 디렉터리:

```text
data/real_calibration/session-const-env/2026-08-11/130333
```

시험 범위는 `yaw=-180~-160°`, `down=0~10°`, 두 축 1° 간격이다. 이 시험은 360×90
전체 1° full search가 아니며, 후보 방향 확인용 국소 진단이다.

| 항목 | 결과 |
|---|---:|
| 유효 LiDAR point | 40,307 |
| robust normal | 37,808 |
| 승인 평면 | 13 |
| 평면 라벨 승인 point | 20,474 (50.8%) |
| 평면 쌍 후보 | 21 |
| 승인 평면 교차선 | 6 |
| 폐색선(진단 전용) | 282 |
| 선택 방향 | down 0°, yaw -171° |
| 선택 교차선 길이 | 0.324 m |
| 화면 내 구조선 | 0 |
| NID 개선률 | -4.11% |
| 최종 상태 | `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL` |

해석:

- 기존 282개 폐색 윤곽을 구조선으로 사용하던 문제는 제거됐다.
- `y=1.123 m`(219점), `y=1.071 m`(133점)의 수평 가구면 후보가 복구됐다. 실제 책상
  상판인지 여부는 라벨 PLY와 설치 사진을 함께 확인해야 한다.
- 교차선은 6개로 늘었지만 현재 CH1 영상 안의 유효 구조선은 여전히 0이어서 RT를
  제약하지 못했다.
- 따라서 현재 결과는 알고리즘 분리와 데이터 진단에는 성공했지만 자동 RT 승인에는
  실패한 상태다.
- 단일 image+JSON 관측이므로 내부 파라미터와 RT를 제품값으로 확정할 수 없다.

## 5. 디버그 산출물

결과 디렉터리:

`generated/real_session_const_20260811_ch1_horizontal_recovery/`

장면별 주요 파일:

- `04_lidar_surface_normals.ply`: robust normal
- `04a_lidar_plane_labels.ply`: 평면별 색상 라벨
- `04a_lidar_planes.csv`: 평면 계수, 지지점 수, RMS
- `04b_lidar_plane_pair_candidates.csv`: 평면 쌍 승인/탈락 근거
- `04b_lidar_plane_intersection_edges.ply/.obj`: 실제 평면 교차선
- `04c_lidar_occlusion_edges.ply/.obj`: 폐색 윤곽
- `04d_lidar_edges_used_for_calibration.ply/.obj`: 목적함수 입력 구조선
- `05_projection_initial.png`: 초기 RT 투영
- `06_projection_final.png`: 최종 진단 후보 투영
- `07_projection_final_edges.png`: range edge 투영
- `debug_summary.csv`: 단계별 점·선분 개수

## 6. 현재 미완료와 리스크

- CH1에서 검출된 교차선 6개가 카메라 화면에 투영되지 않아 구조선 비용이 비활성이다.
- 승인 평면 라벨 비율은 33.5%에서 50.8%로 개선됐지만 절반가량은 여전히 미분류다.
- 단일 관측만으로는 K와 RT를 분리 식별할 수 없다.
- 카메라 LDC 실제 상태와 렌즈 왜곡 계수는 아직 확정하지 않았다.
- LiDAR intensity/signal strength는 현재 품질 필터에만 사용하며 NID에는 넣지 않았다.
- 130333 레거시 JSON은 range offset을 실행 인자로 전달해야 한다. 향후 producer가
  `sensor.range_offset_m`를 JSON에 기록하는 것이 권장된다.
- 360×90 1° full search와 coarse step-size benchmark는 아직 이번 문서의 CH1 국소 시험으로
  대체되지 않는다.

## 7. 다음 작업 순서

1. `04b_lidar_plane_pair_candidates.csv`와 `04a_lidar_plane_labels.ply`를 확인해 실제
   벽–바닥/벽–책상 경계가 평면으로 함께 검출되는지 확인한다.
2. 카메라 영상 안에 긴 평면 경계가 보이고 LiDAR에서도 8점 이상 연속 관측되는 구도로
   동일 설치 image+JSON을 최소 3쌍 수집한다.
3. CH1 단일 장면이 아닌 3개 이상 장면의 공동 K+RT 최적화를 수행한다.
4. hold-out 장면에서 같은 RT로 Top X-Y와 Front X-Z 투영을 검증한다.
5. 그 후에 yaw/roll coarse 간격 시험과 1° full-search conformance를 진행한다.
6. 실제 actuator 팀과 pan 기준 방향, tilt 부호, range offset, timestamp pairing을
   최종 ICD로 고정한다.

## 8. 관련 문서

- [Calibration Core 아키텍처](CALIBRATION_CORE_ARCHITECTURE.md)
- [실데이터 실패 분석 리포트](REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md)
- [LiDAR JSON 인터페이스](PAN_TILT_LIDAR_JSON_INTERFACE.md)
- [Yaw–Roll 탐색 간격 시험 계획](ORIENTATION_SEARCH_STEP_SIZE_TEST_PLAN.md)

## 9. 설치 계약 기반 3D 재투영 수정 (2026-08-13)

단일 관측이 FAIL인 경우에도 이전 3D 산출물은 최적화에서 탈락한 후보 RT를 사용했다.
130333 CH1에서는 입력한 광축 `(0,0,-1)` 대신 후보 광축
`(0.117,-0.050,-0.992)`가 사용되어 실제 설치 방향과 약 7.4° 차이가 났다.

현재는 카메라 중심, 광축, 영상 아래 방향이 모두 명시되면 다음 설치 RT를 직접 구성해
FAIL 진단의 2D/3D 재투영에 사용한다.

```text
camera center = (0.05928, -0.08305, 0) m
camera forward = (0, 0, -1)
camera down = (0, 1, 0)
R_camera_lidar = diag(-1, 1, -1)
t_camera_lidar = (0.05928, 0.08305, 0) m
```

최적화 후보는 `diagnostic_candidate_t_camera_lidar`에 분리 보존하고, 실제 시각화 RT는
`visualization_t_camera_lidar`, 출처는
`operator_measured_installation_contract`로 기록한다. Viewer OBJ/PLY에는 청록색으로
카메라 중심부터 1 m 길이의 실제 광축 마커를 추가했다.

재실행 결과 Top-view heading은 정확히 `-90°`, 카메라 화면에 투영된 점은 1,479개다.
산출물은 `generated/real_session_const_20260811_ch1_installed_reprojection/`에 있다.
이 결과는 설치 방향을 정확히 표현하지만, 제조사 FOV 기반 K와 LDC unknown 상태이므로
픽셀 단위 자동 캘리브레이션 성공을 의미하지 않는다.

### Viewer mesh 단위

VS Code 3D Viewer용 `*_viewer_mesh.obj/.ply`는 clipping 호환성을 위해 좌표를 미터로
저장한다. Viewer OBJ는 외부 `.mtl` 파일 없이 형상만 제공하고, 색상이 필요한 경우 PLY를
사용한다. 일반 `scene_0_colorized_lidar.obj/.ply`는 기존 계약대로 밀리미터를 유지하고,
`04b_lidar_plane_intersection_edges.ply`는 미터다. Viewer에서 확인할 때는
`*_viewer_mesh` 또는 `04b` 파일을 사용한다.

이미지 색상이 입혀진 LiDAR 점을 미터 단위로 직접 확인하려면
`scene_0_colorized_lidar_z_up_reprojection_m.ply`를 사용한다. 모든 LiDAR 점의 이미지
투영 색상을 보존하며, 이미지에서 색을 얻지 못한 점은 회색으로 표시한다.

## 10. CH1 세로 자세 복구 시험 (2026-08-13)

### 산출물 분리 정정

9절의 “FAIL 시 설치 계약 RT를 일반 시각화에 사용” 정책은 폐기했다. 설치 prior가
거절된 최적화 후보를 덮어써 원인을 오해하게 만들었기 때문이다.

- `matching_scene_0.png`, 일반 colorized PLY/OBJ, `debug/06~07`: 최적화 후보
- `mechanical_prior_matching_scene_0.png`, `mechanical_prior_scene_0_*`: 설치 prior
- `calibration_result.json`: 두 RT와 출처를 각각 기록
- FAIL 후보는 계속 비활성이며 제품 RT로 사용하지 않음

### 구조선 투영 복구

평면 교차선 6개가 검출됐지만 기존 평가는 선분 양 끝점과 중앙점이 모두 화면 안에
들어오는 경우만 허용해 `structural_projected_points=0`이 됐다. fitted 교차선에서 33개
점을 샘플링하고 화면 안에서 z-buffer를 통과한 첫·마지막 점으로 보이는 구간을 구성하도록
수정했다. fitted plane RMS를 고려해 가시성 tolerance는 최소 30 mm를 사용한다.

130333 CH1에서는 화면 내 구조선이 최대 2개로 복구됐다. 두 선 모두 수평 계열이며,
LiDAR plane segmentation에서 카메라가 보는 유효 수직 교차선은 얻지 못했다. 따라서
수평 구조선은 down/roll을 일부 제약하지만 완전한 3축 방향 식별에는 부족하다.

### coarse-to-fine 결과

설치 forward/down은 비교용으로만 사용하고 방향 prior weight를 0으로 설정했다.

| 단계 | 탐색 범위 | 선택 초기 후보 | 판정 |
|---|---|---|---|
| 5° coarse | yaw -180~-160°, down -30~30°, roll -15~15°, focal 0.9/1.0/1.1 | down 15°, roll 15°, yaw -165°, focal 1.0 | FAIL |
| 1° fine | yaw -170~-160°, down 10~20°, roll 10~20°, focal 1.0 | down 14°, roll 17°, yaw -169° | FAIL |

coarse focal별 최저 인접후보 보정 점수는 0.9=`0.728239`, 1.0=`0.714862`,
1.1=`0.718370`으로 focal 1.0이 가장 낮았다. fine 후보는 2개 수평 구조선,
edge mean `34.07 px`, projected ratio `0.565`를 얻었지만 NID가 `-1.89%` 악화돼
`NID_IMPROVEMENT_INSUFFICIENT`로 거절됐다. 단일 관측 진단이므로 이 게이트와 무관하게
최종 상태는 `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL`이다.

산출물:

- `generated/real_session_const_20260811_ch1_candidate_prior_split/`
- `generated/real_session_const_20260811_ch1_vertical_coarse_5deg/`
- `generated/real_session_const_20260811_ch1_vertical_fine_1deg/`

### 현재 결론

기존 완전 수평 광축 가정은 잘못됐고, 데이터는 약 14~15°의 하향 성분을 지지한다.
optical roll도 17° 주변 후보가 낮은 복합 점수를 보였지만 NID가 함께 개선되지 않았으므로
확정 RT가 아니다. 구조선이 하나도 보이지 않는 후보는 새
`STRUCTURAL_OVERLAP_INSUFFICIENT` 게이트로 PASS할 수 없다. 다음 승인 시험에는 동일
CH1·고정 설치에서 구조가 다른 동기 image+JSON 최소 3쌍이 필요하다.

## 11. 지금까지 시도한 내용 누적 기록 (2026-08-13)

아래 표는 결과가 좋았는지와 관계없이 실제로 수행한 시도와 현재 판정을 시간순으로
정리한 것이다. `PASS`는 합성 회귀 또는 내부 게이트 통과를 뜻하며, 실제 RT가 맞다는
뜻으로 사용하지 않는다.

| 단계 | 시도/목적 | 적용한 내용 | 결과 및 현재 판정 |
|---|---|---|---|
| 1 | 개발환경·Core 실행 확인 | Ubuntu latest Docker, CMake/Ninja, CTest 5개 | CTest 5/5 PASS. 센서 정확도 보증 아님 |
| 2 | Stanford 데이터셋 검증 | 단일/다중 장면 synthetic LiDAR–image 최적화 | 단일 장면 FAIL(이동 오차 기준 초과), 다중 5장면 PASS(회전 0.789°, 이동 39.3 mm). 합성 가능성만 확인 |
| 3 | 실제 session-001~003 시험 | 이미지 flip/회전, 200/400 bps, 채널별 투영 및 후보 비교 | 일부 수치 PASS가 있었으나 물리 투영 불일치로 false PASS 재분류, 결과 전부 FAIL |
| 4 | 고정환경 130333 확보 | CH1~CH4, 설치 위치·높이·횡방향 중심을 고정 | 반복 가능한 기준 데이터 확보. 채널당 동기 image+JSON이 1쌍이라 제품 calibration 불가 |
| 5 | JSON 좌표계 정정 | `x=r cos(tilt) sin(pan)`, `y=-r sin(tilt)`, `z=r cos(tilt) cos(pan)` 적용 | producer 계약과 adapter 식을 일치시킴 |
| 6 | tilt 의미 분리 | `tilt_rad`는 0° 수평, -90° nadir; `mechanism.tilt_zero=nadir`는 기구 home으로만 해석 | 잘못된 +90° 보정과 tilt 원점 오독 제거 |
| 7 | 센서 offset·방향 반영 | legacy range offset 0.084 m, camera center `(0.05928,-0.08305,0)` m, pan 증가 시계 방향 | 입력 좌표와 설치 metadata를 진단 경로에 기록 |
| 8 | 천장 설치 시각화 | Z-up Top/Front/Side view, 카메라 중심·광축 marker, mm/m Viewer mesh 분리 | OBJ/PLY를 확인 가능하게 수정하고 `.mtl` 의존 제거 |
| 9 | 카메라 K 정책 | checkerboard/Charuco 수동 K를 배제하고 제조사 FOV 중앙값으로 초기화 | 자동 취지 유지. 실제 zoom/focus·LDC profile은 미확정 |
| 10 | LDC·focus 처리 | LDC는 `unknown`, zoom/focus lock 상태를 metadata에 기록 | 렌즈 왜곡은 추정하지 않음. 가장자리 오차 원인으로 남음 |
| 11 | edge-only 초기 매칭 | LiDAR range discontinuity와 Canny/edge distance 사용 | 벽·바닥·책상·장애물 폐색이 같은 edge로 섞여 잘못된 방향 선택 |
| 12 | NID+edge 복합 목적함수 | surface normal/range geometry와 16×16 soft NID를 추가 | 반복·대칭 공간에서 yaw가 비슷해도 NID가 개선되지 않는 사례 지속 |
| 13 | yaw 탐색 변경 | 45° coarse → 15°/5° coarse, 1° 국소 및 360×90 1° baseline 실행 | 1° full search는 reference RT 부재로 conformance gate 보류 |
| 14 | 후보 점수 보정(B 방식) | 후보 raw score + 인접 8개 Gaussian 보정, contiguous basin 선택 | raw 단일 최저점보다 연속 영역을 반영. 정답 보장은 아님 |
| 15 | 구조선 정의 수정 | range edge를 구조선으로 쓰던 방식 폐기, robust normal→평면 분할→평면 교차선으로 변경 | 폐색선 282개는 진단 전용, 승인 평면 교차선만 calibration 입력 |
| 16 | 미분류/수평면 복구 | 인접 평면 재할당, 공면 병합, IMU `+Y down` 높이 clustering | CH1 승인 point 33.5%→50.8%, 교차선 1→6 |
| 17 | 설치 prior 투영 분리 | FAIL 후보를 설치 RT로 덮어쓰던 경로 폐기, 최적화 후보와 mechanical prior 파일 분리 | 투영 결과의 출처 혼동 제거 |
| 18 | 구조선 가시성 수정 | 양 endpoint가 화면 밖이면 선 전체를 버리던 로직을 33점 visible segment sampling으로 변경 | 화면 내 구조선 0→최대 2. 수직 구조선은 여전히 0 |
| 19 | 세로 자세 coarse-to-fine | direction prior=0, down/pitch와 optical roll 독립 탐색, focal 0.9/1.0/1.1 비교 | 5°: down 15°/roll 15°; 1°: down 14°/roll 17°/yaw -169°. NID 악화로 FAIL |
| 20 | 현재 내부 파라미터 진단 | fine RT 주변에서 focal scale 비교 | coarse 보정점수는 1.0이 최저였으나 K 정답 증거 아님. 단일 관측 K–RT 분리 불가 |

### 현재 유효한 결론

- 좌표계와 JSON 변환은 현재 문서 계약을 기준으로 처리한다. producer가 `sensor.range_offset_m`
  를 JSON에 넣는 것은 후속 권장사항이다.
- 실제 투영에서 yaw만 비슷하고 세로 크기·위치가 틀린 원인은 단일 원인이 아니다. 현재
  확인된 후보는 optical down/roll, 제조사 FOV 기반 K, LDC/렌즈 왜곡, 구조선 부족이다.
- `down≈14~15°`, `roll≈17°`는 탐색 후보이지 정답 RT가 아니다. NID가 개선되지 않았고
  화면 내 구조선도 2개뿐이다.
- 현재 모든 130333 단일 장면 결과는 `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL`이다.
  제품 RT 또는 conformance PASS로 승격하지 않는다.

### 폐기된 가정·결과

- 공통 `prior-roll=90°` 고정
- `mechanism.tilt_zero=nadir`를 계약각 원점으로 읽는 해석
- range discontinuity 282개를 전부 구조선으로 사용하는 방식
- FAIL 후보 RT를 최종 colorized 투영에 표시하는 방식
- 단일 관측에서 수치 objective PASS를 실제 정합 PASS로 해석하는 방식

### 남은 작업

1. 동일 고정환경 CH1에서 책상 상판·벽 경계가 화면과 LiDAR에 함께 길게 보이는 동기
   image+JSON을 최소 3쌍 수집한다.
2. 해당 3쌍 이상으로 K와 RT를 공동 최적화하고, 별도 hold-out 장면에 재투영한다.
3. 실제 카메라 채널별 FOV/zoom/focus와 LDC 상태를 확인해 K·왜곡 profile을 고정한다.
4. 비대칭 구조물로 pan 부호·θ=0 기준을 독립 검증한다.
5. reference RT가 확보된 후에야 coarse 간격 효율과 1° full-search conformance를 판정한다.

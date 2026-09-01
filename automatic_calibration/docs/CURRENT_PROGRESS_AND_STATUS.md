# Automatic Calibration 현재 진행 현황

작성일: 2026-08-13
최종 수정일: 2026-08-27
범위: Calibration Core, LiDAR JSON 변환, 2D–3D 매칭 진단, 고정환경 CH1 시험 및 제품 운용 정책

날짜별 작업·문제·수정·잔여 이슈는
[`DAILY_WORK_LOG.md`](DAILY_WORK_LOG.md)에서 관리한다.

2026-08-27 V3 analyzer 이후 확인한 camera/LiDAR edge 의미 불일치, geometry NID의 한계,
targetless 문헌 비교와 다음 구현 순서는
[`CROSS_MODAL_EDGE_SCORING_ANALYSIS_AND_IMPLEMENTATION_PLAN_20260827.md`](CROSS_MODAL_EDGE_SCORING_ANALYSIS_AND_IMPLEMENTATION_PLAN_20260827.md)를
기준으로 한다. 현재 결론은 search step 추가 축소보다 channel-separated fine score와
관측성 gate를 먼저 구현하는 것이다.

2026-08-24 finalist별 hold-out 재검증과 최신 판정은
[`FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md`](FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md)를
우선한다. 2026-08-23 코드·결과 재감사는 실험 이력으로
[`CALIBRATION_LOGIC_EVALUATION_AND_IMPROVEMENT_20260823.md`](CALIBRATION_LOGIC_EVALUATION_AND_IMPROVEMENT_20260823.md)에
보존한다.

현재 MVP 정책은 [`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)를 기준으로
한다. 이 문서의 이전 실행 결과는 실험 이력으로 보존하며, 아래 최신 정책 섹션이 과거의
제조사 FOV K·K+RT 공동 추정·단일 내부 `PASS` 해석을 대체한다.

Manual ChArUco/태블릿 display geometry를 이용한 별도 기준·진단 경로의 최신 결과는
[`../../manual_calibration/docs/SESSION_CONST_ENV_CALIBRATION_RECORD.md`](../../manual_calibration/docs/SESSION_CONST_ENV_CALIBRATION_RECORD.md)에 기록한다. 이 문서의 Automatic 결과와 혼동하지 않도록 `T_camera_lidar` 예비값은 별도 status로 관리한다.

다른 Manual 세션의 결과를 Automatic prior/reference/holdout으로 분리해 사용하는 규칙은
[`MANUAL_REFERENCE_PRIOR_WORKFLOW.md`](MANUAL_REFERENCE_PRIOR_WORKFLOW.md)를 따른다.

## 1. 프로젝트 목적

actuator가 완성되기 전에도 카메라 영상과 1D LiDAR pan/tilt scan을 이용해 카메라–LiDAR
외부 파라미터(RT)를 검증할 수 있도록 Calibration Core를 개발한다. 최종 목표는
실제 장치에서 반복 가능한 자동 캘리브레이션과 conformance test에 사용할 수 있는
구조다.

현재는 다중 training에서 RT 후보를 만들고 분리 finalist를 동일 hold-out의 연속
목적함수로 비교할 수 있다. build17~21 최신 상태는 `CANDIDATE_RT / PASS`지만
센서·actuator 통합 완료품이나 `PRODUCT_APPROVED_RT`가 아니며
`activation_allowed=false`다.

조명 켜짐 고정환경 repeat sample의 `PASS`가 실제 reprojection과 일치하지 않는
false positive로 확인되었다. 상세 원인과 문헌 비교, 수정 순서는
[BRIGHT_LIGHT_FALSE_PASS_ANALYSIS.md](BRIGHT_LIGHT_FALSE_PASS_ANALYSIS.md)에 기록했다.
특히 해당 실행은 실측 camera center가 아닌 기본 `baseline=0.28 m`를 사용했으므로
결과 RT를 제품 calibration 값으로 사용하지 않는다.

2026-08-14에는 Manual ChArUco intrinsic을 명시적으로 입력하고 raw image distortion을
undistort하는 경로를 구현했다. `repeat_test_sample` 10세트를 실행한 결과는
`FAIL / NID_IMPROVEMENT_INSUFFICIENT`이며, 거절 후보와 예비 Manual RT의 차이는
65.4376°/0.142727 m였다. 입력 image–LiDAR 대응 manifest가 없어 이 결과는
진단용으로만 취급한다. 상세 명령·산출물은
[REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md](REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md)에 기록했다.

2026-08-14에는 당시 설치 측정값을 반영해 CH1을 추가 재실행했다. 지면 기준 센서 순서는
LiDAR → 카메라이며, 해당 실행에는 JSON `+Y=down` 좌표계의 camera center
`(+0.05928, -0.08305, 0) m`를 사용했다. 5° coarse 탐색 결과는
`FAIL / MULTISTART_AMBIGUOUS`였고 `down=80°` 후보가 contiguous basin으로 선택됐다.
이는 중심 오프셋 부호가 맞다는 것을 확인하지만, 단일 장면의 방향 식별과 reprojection이
해결됐다는 뜻은 아니다. 결과 이미지는 `REJECTED CANDIDATE` 진단용으로만 보관한다.
상세 결과는 [BRIGHT_LIGHT_FALSE_PASS_ANALYSIS.md](BRIGHT_LIGHT_FALSE_PASS_ANALYSIS.md)의
재실행 절을 참조한다.

같은 날 바닥 방향 고착 원인을 수정하고 조명 ON 5세트만 분리해 재검증했다. coarse
basin에서 수평·수직 구조 방향군을 모두 요구하고, 인접 후보 보정 비율을 50%에서 20%로
낮췄으며, 실패 basin이 refined 후보의 최종 gate를 덮어쓰지 않도록 변경했다. 5° 탐색의
선택 자세는 `down=20.37°`, `yaw=165°`이고 Manual reference와 회전 차이는 12.31°로
줄었다. geometry-first 가중치 시험은 11.17°까지 줄었지만 NID가 약 0.61% 악화되어 모두
`FAIL`을 유지했다. Automatic과 예비 Manual RT의 투영 모두 완전 정합이 아니므로 Manual
RT도 ground truth로 승격하지 않았다. 명령·수치·2D/3D 비교 경로는
[REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md](REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md)에 기록했다.

이후 모델링 치수를 다시 확인해 현재 camera center prior를
`(+0.05928, -0.08105, 0) m`로 정정했다. 59.28 mm 수평 성분은 기존 실행에도 이미
반영되어 있었고, 81.05 mm는 수직 성분이다. 동일 geometry-first 5° 조건의 v5 재실행은
동일한 yaw/down grid를 선택했고 `FAIL / NID_IMPROVEMENT_INSUFFICIENT`를 유지했다.
따라서 정정값은 향후 translation prior의 기준으로 사용하되 방향 실패의 해결로 해석하지
않는다.

같은 날 geometry 식별력을 보강해 range/normal spatial NID, 평면 경계·반복 폐색 구조선,
2D–3D 선분 1:1 대응, 영상 소실점–LiDAR 중력축 Manhattan 잔차, 장면별 hold-out gate를
추가했다. 소실점 후보를 3개에서 최대 12개로 늘리고 down 탐색을 25~40°로 옮긴 v9에서는
학습 4장면의 수직 오차가 3.86~7.88°로 줄어 모두 통과했다. hold-out 한 장면의 edge
mean이 40.8389 px로 40 px 기준을 초과해 최종 상태는
`FAIL / HOLDOUT_VALIDATION_FAILED`다. 상세 결과는
[REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md](REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md)의 11절에 기록한다.

이어 v9 주변을 1°로 탐색한 v10도 hold-out edge `40.4238 px`로 실패했다. 단계별 잔차
그림에서 비스듬한 책상 상판·벽 내부가 range edge로 잘못 선택된 것을 확인해, 같은 평면
label·접평면 호환성·앞뒤 local contrast를 이용한 edge 필터를 추가했다. 동일 조건 v11은
`yaw=167°`, grid down `28°`, refinement down `27.3755°`를 선택했고 학습 4/4,
hold-out 1/1을 통과했다. hold-out edge는 `37.1998 px`, 수직 오차는 `6.8348°`다.
이는 최초의 **실데이터 내부 gate PASS**이지만 독립 ground truth와 hold-out 3개 이상이
없으므로 제품 RT/conformance 승인 상태는 아니다. 리포트 metadata를 보완한 최종 코드
v12에서도 같은 RT·수치·시각화 checksum으로 재현됐다.

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
- 고정환경의 카메라 중심 prior는 정정 측정값 `x=0.05928 m, y=-0.08105 m, z=0 m`을
  진단용으로 사용했다.

### 카메라 처리

- PNM-C16083RVQ CH1의 현재 운용 경로는 LDC 미사용(`false`)으로 고정한다.
- 동일 해상도·zoom·focus에서 구한 Manual ChArUco `K + distortion`을 고정하고 raw
  영상을 `cv::undistort`한 뒤 Automatic RT만 탐색한다.
- `LDC=false`인데 Manual intrinsic, 왜곡 계수 또는 `image-distortion-state=raw`가
  누락되면 실행을 중단한다. 제조사 FOV K는 별도 비교·초기화 경로에만 사용한다.
- RGB grayscale, Sobel gradient, Canny edge distance transform을 생성한다.
- zoom/focus 고정 조건과 사용한 Manual intrinsic profile을 결과 metadata에 저장한다.

### 매칭과 최적화

- LiDAR range discontinuity와 robust surface-normal 변화를 별도 geometry 채널로 계산한다.
- 2×2 spatial cell별 16×16 soft NID를 계산하고 최소 투영점·entropy·active-cell gate를
  적용한다.
- 평면 교차선, 평면–미분류 경계선, 여러 관측에서 반복되는 폐색선을 구조선으로 사용하고
  2D LSD 선분과 1:1 대응한다.
- 영상 소실점과 LiDAR 중력축·수평 벽축의 Manhattan 방향 잔차를 분리한다.
- 보정된 `signal_strength` NMI는 perturbation conformance 파일만 만들며 현재 가중치는 0이다.
- yaw 후보는 360° 범위를 탐색할 수 있고, 실험 시 `--yaw-step-deg 1`과 제한 범위를
  지정할 수 있다.
- down 후보와 yaw 후보를 독립적으로 평가한다. 과거의 공통 `prior-roll=90°` 가정은
  제거했다.
- 후보 선택 시 raw score만 사용하지 않고 인접 8개 후보의 Gaussian 보정 점수와
  contiguous basin을 계산한다.
- Ceres 기반 6-DoF refinement와 z-buffer 가시성 필터를 사용한다.
- 학습 후보를 고른 뒤 각 학습 장면과 별도 hold-out 장면을 동일 RT로 다시 평가한다.
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
7. 고유 inlier가 최소 8개이고 선분 길이가 150 mm 이상일 때 평면 교차선을 만든다.
8. 승인 평면과 미분류 geometry의 경계를 PCA 선분으로 맞춰 plane-boundary 구조선을 만든다.
9. range discontinuity는 단일 scan에서는 진단용으로 두고 여러 관측에서 반복되는 선만
   persistent occlusion 구조선으로 승격한다.
10. 3D 구조선과 2D LSD 선분을 방향·끝점·겹침 비용으로 1:1 할당한다.

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
- `04b1_lidar_plane_boundary_edges.ply/.obj`: 평면–미분류 geometry 경계선
- `04c_lidar_occlusion_edges.ply/.obj`: 폐색 윤곽
- `04c1_lidar_persistent_occlusion_edges.ply/.obj`: 여러 관측에서 반복된 폐색 구조선
- `04d_lidar_edges_used_for_calibration.ply/.obj`: 목적함수 입력 구조선
- `03a_manhattan_vanishing_directions.csv`: 소실점 후보와 중력축 각도 오차
- `05_projection_initial.png`: 초기 RT 투영
- `06_projection_final.png`: 최종 진단 후보 투영
- `07_projection_final_edges.png`: range edge 투영
- `debug_summary.csv`: 단계별 점·선분 개수

## 6. 현재 미완료와 리스크

- 과거 130333 단일 장면에서는 교차선이 화면에 투영되지 않았지만, 최신 repeat v9에서는
  구조선 89개가 1:1 대응됐다. 데이터 세대가 다르므로 두 결과를 직접 합치지 않는다.
- 승인 평면 라벨 비율은 33.5%에서 50.8%로 개선됐지만 절반가량은 여전히 미분류다.
- 단일 관측만으로는 K와 RT를 분리 식별할 수 없다.
- 카메라 LDC 실제 상태와 렌즈 왜곡 계수는 아직 확정하지 않았다.
- 보정된 LiDAR `signal_strength` NMI 경로는 구현됐지만 Manual RT perturbation
  conformance가 실패해 목적함수 가중치는 0이다.
- 130333 레거시 JSON은 range offset을 실행 인자로 전달해야 한다. 향후 producer가
  `sensor.range_offset_m`를 JSON에 기록하는 것이 권장된다.
- 360×90 1° full search와 coarse step-size benchmark는 아직 이번 문서의 CH1 국소 시험으로
  대체되지 않는다.

## 7. 다음 작업 순서

1. 동일 고정환경에서 학습과 독립된 image+JSON hold-out을 3쌍 이상 추가한다. 정적 장면은
   결정론적 association으로도 검증할 수 있고, 움직임이 있는 장면은 시간 근접 pair를 사용한다.
2. 조명·사람 유무가 다른 hold-out에서도 v11 RT와 gate가 반복되는지 확인한다.
3. 독립 reference RT가 확보되면 yaw/down 1° full-search와 coarse step-size 효율 시험을 진행한다.
4. signal NMI는 반복성이 확인되기 전까지 진단 채널로만 유지한다.
5. 실제 actuator 팀과 pan 기준 방향, tilt 부호, range offset, timestamp pairing을
   최종 ICD로 고정한다.

## 8. 관련 문서

- [Calibration Core 아키텍처](CALIBRATION_CORE_ARCHITECTURE.md)
- [실데이터 실패 분석 리포트](REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md)
- [LiDAR JSON 인터페이스](PAN_TILT_LIDAR_JSON_INTERFACE.md)
- [Yaw–Roll 탐색 간격 시험 계획](ORIENTATION_SEARCH_STEP_SIZE_TEST_PLAN.md)
- [Manual session-const-env 작업 기록](../../manual_calibration/docs/SESSION_CONST_ENV_CALIBRATION_RECORD.md)
- [20260818 CH1 고정환경 데이터 수집 확인](CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md)

## 9. 설치 계약 기반 3D 재투영 수정 (2026-08-13)

단일 관측이 FAIL인 경우에도 이전 3D 산출물은 최적화에서 탈락한 후보 RT를 사용했다.
130333 CH1에서는 입력한 광축 `(0,0,-1)` 대신 후보 광축
`(0.117,-0.050,-0.992)`가 사용되어 실제 설치 방향과 약 7.4° 차이가 났다.

현재는 카메라 중심, 광축, 영상 아래 방향이 모두 명시되면 다음 설치 RT를 직접 구성해
FAIL 진단의 2D/3D 재투영에 사용한다.

```text
camera center = (0.05928, -0.08105, 0) m
camera forward = (0, 0, -1)
camera down = (0, 1, 0)
R_camera_lidar = diag(-1, 1, -1)
t_camera_lidar = (0.05928, 0.08105, 0) m
```

최적화 후보는 `diagnostic_candidate_t_camera_lidar`에 분리 보존하고, 실제 시각화 RT는
`visualization_t_camera_lidar`, 출처는
`operator_measured_installation_contract`로 기록한다. Viewer OBJ/PLY에는 청록색으로
카메라 중심부터 1 m 길이의 실제 광축 마커를 추가했다.

2026-08-13의 이전 83.05 mm 재실행 결과 Top-view heading은 정확히 `-90°`, 카메라
화면에 투영된 점은 1,479개다.
산출물은 `generated/real_session_const_20260811_ch1_installed_reprojection/`에 있다.
이 결과는 설치 방향을 확인하는 진단 자료다. 과거 실행은 제조사 FOV 기반 K와 LDC
unknown 상태였으므로 픽셀 단위 자동 캘리브레이션 성공을 의미하지 않는다. 현재 제품
경로에서는 Manual ChArUco K+D profile을 고정하고 같은 distortion contract로 재검증한다.

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

기존 완전 수평 광축 가정은 잘못됐고, 당시 단일 장면 탐색은 약 14~15° 후보를 지지했다.
이 값은 이후 repeat 다중 장면 v9의 27.81° 후보로 대체된 과거 진단값이다.
optical roll도 17° 주변 후보가 낮은 복합 점수를 보였지만 NID가 함께 개선되지 않았으므로
확정 RT가 아니다. 구조선이 하나도 보이지 않는 후보는 새
`STRUCTURAL_OVERLAP_INSUFFICIENT` 게이트로 PASS할 수 없다. 다음 승인 시험에는 동일
CH1·고정 설치에서 구조가 다른 image+JSON 최소 3쌍이 필요하다. 장면이 움직이는 경우에만
시간 근접 pairing을 필수로 한다.

## 11. 지금까지 시도한 내용 누적 기록 (2026-08-14)

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
| 7 | 센서 offset·방향 반영 | legacy range offset 0.084 m, 현재 camera center `(0.05928,-0.08105,0)` m, pan 증가 시계 방향 | 입력 좌표와 설치 metadata를 진단 경로에 기록 |
| 8 | 천장 설치 시각화 | Z-up Top/Front/Side view, 카메라 중심·광축 marker, mm/m Viewer mesh 분리 | OBJ/PLY를 확인 가능하게 수정하고 `.mtl` 의존 제거 |
| 9 | 카메라 K 정책 | 현재 CH1은 동일 zoom/focus에서 구한 Manual ChArUco K와 왜곡 계수를 고정 | Automatic은 보정된 영상에서 RT만 탐색하며 Manual RT는 입력하지 않음 |
| 10 | LDC·focus 처리 | 현재 CH1은 `LDC=false`, raw 영상에 `cv::undistort` 적용 | Manual profile 또는 왜곡 계수가 누락되면 실행 중단 |
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
| 21 | spatial geometry NID | range/normal 채널 분리, 2×2 cell, entropy/active-cell gate | v9 NID +1.7475%, 평탄한 전역 histogram보다 공간 식별력 개선 |
| 22 | 구조선 증거 확대 | 평면 경계, 반복 폐색선, 2D–3D 1:1 대응 | v9 선택 후보에서 수평 13개·수직 76개 대응(aggregate) |
| 23 | Manhattan 수직 제약 | 소실점 후보 3→12, LiDAR 중력축/벽축 잔차와 후보 CSV | 수직 오차 약 67.8°→3.86~7.88° |
| 24 | 장면별/hold-out gate | 최종 선택 RT를 각 장면에 재평가하고 PASS 덮어쓰기 결함 수정 | 학습 4/4 PASS, hold-out edge 40.8389 px로 최종 FAIL |
| 25 | signal NMI 진단 | 거리·입사각·range-bin 보정과 Manual RT perturbation | worse ratio 0.5833, median margin 0.000633로 conformance FAIL; weight 0 유지 |
| 26 | 1° 국소 fine search | down 27~33°, yaw 165~171°를 1°로 비교하고 bounded yaw 이웃 적용 | yaw 167°/down 28° 선택, hold-out edge 40.4238 px로 v10 FAIL |
| 27 | 공면 range-edge 제거 | same-plane/접평면 호환점 제외, 앞뒤 gap 대비 2× local contrast, 잔차 heatmap | edge 6.3k→2.9k, 학습 4/4·hold-out 1/1 내부 gate PASS |

### 현재 유효한 결론

- 좌표계와 JSON 변환은 현재 문서 계약을 기준으로 처리한다. producer가 `sensor.range_offset_m`
  를 JSON에 넣는 것은 후속 권장사항이다.
- 실제 투영에서 yaw만 비슷하고 세로 크기·위치가 틀린 원인은 단일 원인이 아니다. 과거
  확인된 후보는 optical down/roll, FOV K, LDC/렌즈 왜곡, 구조선 부족이다. 현재는 Manual
  ChArUco K+D를 고정해 K 오차가 RT로 흡수되는 경로를 차단하고 optical down/roll과 구조선
  관측성을 별도로 검증한다.
- 최신 repeat 다중 장면 후보는 grid down 28°, refinement down 27.3755°, yaw 167°다.
  학습 4장면과 hold-out 1장면은 내부 gate를 통과했다. 그러나 Manual 예비 RT와 회전
  8.5588° 차이가 있고 독립 ground truth가 아니므로 정답 RT로 승인하지 않는다.
- 현재 모든 130333 단일 장면 결과는 `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL`이다.
  제품 RT 또는 conformance PASS로 승격하지 않는다.

### 폐기된 가정·결과

- 공통 `prior-roll=90°` 고정
- `mechanism.tilt_zero=nadir`를 계약각 원점으로 읽는 해석
- range discontinuity 282개를 전부 구조선으로 사용하는 방식
- FAIL 후보 RT를 최종 colorized 투영에 표시하는 방식
- 단일 관측에서 수치 objective PASS를 실제 정합 PASS로 해석하는 방식

### 남은 작업

1. 동일 고정환경·동일 조명에서 날짜/수집 회차가 다른 hold-out을 3쌍 이상 추가한다.
2. 고정 조건의 반복 수집에서 RT 반복성과 단계별 correspondence를 비교한다.
3. 비대칭 구조물로 pan 부호·θ=0 기준을 독립 검증한다.
4. reference RT 확보 후 coarse 간격 효율과 1° full-search conformance를 판정한다.

## 12. 권장 구현 단계 완료 기록 (2026-08-14)

### 구현 범위

이번 단계에서는 geometry 식별력과 수직 방향 제약을 보강하고, 그 결과를 독립 장면에
적용해 false PASS를 차단하는 데 집중했다.

| 구분 | 반영 내용 | 확인 위치 |
|---|---|---|
| Geometry NID | range/normal 채널 분리, 2×2 spatial cell, entropy/active-cell gate | `calibration_core.cpp`, `calibration_result.json` |
| 3D 구조 | 평면 교차선, 평면–미분류 경계, 반복 폐색선 | `debug/scene_*/04b*`, `04c1*`, `04d*` |
| 2D–3D 대응 | LSD 선분과 방향·끝점·중첩 비용의 1:1 assignment | candidate metrics, `orientation_full_search.csv` |
| 수직 방향 | 영상 소실점과 LiDAR 중력축/벽축 Manhattan 잔차 | `03a_manhattan_vanishing_directions.csv` |
| 자세 탐색 | bounded yaw와 down을 1° fine grid로 평가 | `orientation_corrected_scores.csv` |
| edge 정제 | 공면점 제외와 앞뒤 gap 대비 2× local contrast | `07a_projection_final_edge_residual.png` |
| 품질 판정 | training/hold-out 장면별 고정 RT 재평가 | `training_scene_validation.csv`, `holdout_scene_validation.csv` |

### 실패에서 수정까지

```text
v9  : 5° 탐색, hold-out edge 40.8389 px → FAIL
v10 : 1° 탐색, hold-out edge 40.4238 px → FAIL
       └─ 책상 상판·벽 내부의 공면 range 변화가 가짜 edge로 확인됨
v11 : 공면/local-contrast edge 필터 적용, hold-out 37.1998 px → 내부 PASS
v12 : 설정 검증·리포트 metadata 추가 후 동일 RT·수치·checksum 재현 → 내부 PASS
```

v12 선택 자세는 yaw `167°`, grid down `28°`, refinement down `27.3755°`다. 학습은
4/4, hold-out은 1/1을 통과했고 전체 Docker CTest도 5/5 PASS했다. v12 결과 JSON에는
다음 재현성 정보가 포함된다.

```text
lidar_edge_policy = absolute_relative_threshold_plus_local_contrast_and_coplanar_rejection
lidar_edge_minimum_local_contrast_ratio = 2.0
yaw_neighbor_topology = bounded
```

### 판정과 인계 사항

- 현재 상태는 **실데이터 내부 품질 gate PASS**다.
- 독립 ground truth가 없고 hold-out이 한 장뿐이므로 제품 RT/conformance PASS가 아니다.
- Manual 예비 RT는 진단 비교값이며 Automatic 목적함수나 초기 RT로 사용하지 않았다.
- `signal_strength` NMI는 perturbation 식별력 검증 실패로 가중치 0을 유지한다.
- 다음 승인 시험에는 고정 설치·고정 환경·조명 ON에서 날짜/수집 회차가 다른 독립
  hold-out 최소 3쌍이 필요하다.

최종 산출물:

```text
automatic_calibration/generated/
  repeat_test_sample_20260814_light_on_structural_nid_manhattan_v12_fine1deg_coplanar_edge/
```

세부 수치와 v6~v12 비교는
[`REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md`](REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md),
실패 원인 누적 분석은
[`REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md`](REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md)를 따른다.

## 13. 조명 ON–나이트비전 교차조건 참고 진단 (2026-08-14)

> 범위 정정: 나이트비전은 현재 제품 요구사항과 승인 조건이 아니다. 아래 실행은 고정
> RT 검증 경로가 조건 변화에서 어떻게 동작하는지 확인한 참고 진단이며, 조명 ON 주
> 개발 경로의 상태를 낮추거나 차단하지 않는다.

### 데이터 분류와 검증 방식

운영자 확인에 따라 `repeat_test_sample`을 다음처럼 분류했다. 파일명 시각은 동기화
근거로 사용하지 않고, 확정된 그룹 순서 안에서 사전식 index로 image와 scan을 묶었다.

| 조건 | 선택 범위 | 용도 |
|---|---:|---|
| 조명 ON 컬러 | 앞 5쌍 (`pair-start=0`) | 4 training + 1 holdout으로 v12 RT 추정 |
| 조명 OFF 나이트비전 | 뒤 5쌍 (`pair-start=5`) | 별도 RT 재추정 및 v12 RT 고정 교차검증 |

조명 변화와 RT 변화를 분리하기 위해 `--validation-pose-json` 경로를 추가했다. 이
경로는 입력 RT를 전혀 최적화하지 않고 선택된 모든 장면에 그대로 적용한 뒤 장면별
gate만 계산한다. 결과는 `fixed_pose_validation_result.json`과
`fixed_pose_scene_validation.csv`로 남긴다.

### 결과

| 시험 | RT 처리 | 장면 통과 | 상태 |
|---|---|---:|---|
| v12 조명 ON | 앞 4쌍 추정, 1쌍 holdout | training 4/4, holdout 1/1 | 내부 gate PASS |
| v13 나이트비전 | 앞 4쌍 재추정, 1쌍 holdout | training 1/4, holdout 0/1 | FAIL |
| v14 나이트비전 | **v12 RT 고정**, 5쌍 전체 평가 | 1/5 | FAIL |

v14의 실패 장면 0, 1, 2, 4는 모두 `EDGE_ALIGNMENT_POOR`였다. 평균 edge distance는
각각 `44.41`, `54.39`, `48.83`, `46.96 px`로 gate `40 px`를 넘었고, 통과한 장면 3은
`35.90 px`였다. 조명 ON의 Canny edge는 장면당 평균 약 `45,425 px`, 나이트비전은
약 `20,974 px`로 `53.8%` 감소했다.

나이트비전 v13의 거절된 최적화 후보는 조명 ON v12 RT와 회전 `3.148°`, 이동
`3.566 mm` 차이였다. 차이가 크지는 않지만 v13 후보 자체가 품질 gate를 통과하지
못했으므로 반복 가능한 RT로 승인할 수 없다. 현재 증거는 큰 RT 점프보다 주·야간 전환에
따른 2D edge 분포 변화와 고정 `40 px` edge gate의 modality 민감도를 더 강하게 지지한다.

### 판정과 현재 개발 범위

- 나이트비전 5쌍과 v13/v14 FAIL은 참고 결과로만 보존하며 현재 conformance gate에
  포함하지 않는다.
- 조명 ON v12의 내부 gate PASS 상태는 유지된다. 다만 독립 ground truth와 날짜/수집
  회차가 다른 holdout이 부족해 제품 RT 승인 상태는 아니다.
- 다음 필수 데이터는 같은 rigid camera–LiDAR 설치·공간 구조·**조명 ON** 조건에서
  별도로 수집한 image+scan 최소 3쌍이다. 자세한 범위는 14절을 따른다.
- day/night 공통 gradient 정규화, IR profile K 검증 및 나이트비전 전용 threshold는
  현재 범위 밖의 후속 선택 과제로 보류한다.

산출물:

```text
automatic_calibration/generated/
  repeat_test_sample_20260814_night_vision_structural_nid_manhattan_v13_fine1deg/
  repeat_test_sample_20260814_night_vision_fixed_light_v12_rt_v14/
```

## 14. 20260815 고정환경 반복 데이터 범위와 설치 교란 정정 (2026-08-15)

### 현재 시험 목적

이번 단계의 공식 시험 조건은 다음과 같이 고정한다.

| 항목 | 현재 조건 |
|---|---|
| camera–LiDAR 설치 | rigid 관계 고정 |
| 설치 장소와 공간 구조 | 고정 |
| 공식 시험 조명 | 조명 ON 고정 |
| 검증 대상 | 날짜/수집 회차가 달라도 동일 RT가 반복되는지 |
| 현재 비대상 | 다른 공간·구조·조명에 대한 일반화, 나이트비전 |

> **후속 운영자 정정:** `20260813`과 `20260815` 사이 actuator 문제로 장치를 건드렸으며
> camera–LiDAR 상대 자세가 조금 변했을 가능성이 있다. 따라서 위 표의 “rigid 관계 고정”은
> 두 날짜 사이에서 검증되지 않았고, 두 회차를 동일 RT의 엄격한 반복성 시험으로 사용할 수
> 없다. `20260815` 회차 내부의 연속 수집은 같은 상태로 취급한다.

당초 “독립 hold-out”은 같은 고정 조건의 날짜 분리 회차로 계획했으나, 설치 교란 정정 후
`20260815`는 `20260813`과 별도의 **installation epoch**로 분류한다. 두 회차 비교는
교차 회차 진단으로 보존하되 시간 반복성 conformance 근거로 사용하지 않는다.

### `repeat_test_sample/20260815` 현황

확인 시점의 파일 구성은 다음과 같다.

```text
20260815/
  20260814-230655-CH1.jpg
  20260814-230728-CH1.jpg
  20260814-230731-CH1.jpg
  20260815-181310-CH1.jpg
  20260815-181315-CH1.jpg
  20260815-181318-CH1.jpg
  calib-20260814-232414_sweep-000001_pan_tilt_lidar.json
  calib-20260814-233403_sweep-000001_pan_tilt_lidar.json
  calib-20260814-234352_sweep-000001_pan_tilt_lidar.json
```

- LiDAR JSON 3개는 같은 정적 환경에서 연속 측정한 반복 scan이다.
- 앞의 조명 ON 사진 3장은 환경 변화가 없는 주말에 시간별로 획득한 camera snapshot이며
  공식 반복 시험 image 후보로 사용한다.
- 조명 OFF 사진 3장은 같은 시간대 조건 비교용 환경 참고 사진이다. 야간/나이트비전
  데이터로 분류하지 않으며 현재 calibration PASS/FAIL 입력에서 제외한다.
- 장면이 고정되어 있으므로 image와 JSON 사이의 촬영 시각 동기화 또는 물리적 1:1
  pairing은 요구하지 않는다. 다만 현재 실행기 입력 형식 때문에 조명 ON image와 JSON을
  사전식 순서로 한 번씩 연결해 3개 observation으로 구성한다. 이는 계산상 association일
  뿐 실제 동시 획득 관계를 뜻하지 않는다.
- 3개 image × 3개 scan의 9개 조합은 같은 증거를 중복 계산해 표본 수를 부풀리므로
  독립 observation으로 만들지 않는다.
- 현재 실행기는 지정한 한 디렉터리의 image/JSON 개수가 같아야 하고 하위 폴더를 재귀
  탐색하지 않는다. 조명 OFF 참고 사진은 별도 하위 디렉터리로 분리하고 실행 시 조명 ON
  3쌍이 있는 디렉터리를 직접 `--input-dir`로 지정한다.

### 다음 검증 순서

1. 조명 ON image 3개와 연속 scan JSON 3개를 선택하고 camera zoom/focus/profile 불변을
   확인한다. 사전식 index 연결은 재현 가능한 실행을 위한 형식일 뿐 시간 pairing이 아니다.
2. v12 RT를 변경하지 않고 새 3쌍에 적용한 결과는 교차 회차 민감도 진단으로 평가한다.
3. 새 수집 회차만으로 RT를 다시 추정하고 v12와 회전 geodesic·translation 차이를
   기록하되, 차이를 순수 알고리즘 반복 오차로 해석하지 않는다.
4. 이후 장치를 전혀 건드리지 않은 하나의 installation epoch 안에서 날짜 분리 회차를
   추가해 RT 분산을 누적한다.

새 영상은 기존 `20260813` 영상과 보이는 화각/자세 차이가 있고 실제 장치 접촉도
보고됐다. camera 또는 LiDAR가 상대적으로 움직였으면 extrinsic RT가 실제로 변한다.
반대로 camera와 LiDAR 전체가 하나의 rigid assembly로 함께 움직였다면 두 센서 사이 RT는
원칙적으로 유지되므로, 관측 차이는 목적함수 불확실성·가림/화각 변화와 분리해 분석해야
한다. zoom이 바뀌었으면 기존 Manual intrinsic profile도 재사용할 수 없다.

## 15. 20260815 조명 ON 교차 회차 진단 결과 (2026-08-15, 해석 정정)

### 입력 구성과 공통 조건

원본 데이터는 이동하거나 이름을 변경하지 않았다. 현재 실행기가 image/JSON 개수가
같은 단일 디렉터리를 요구하므로 Docker 내부 `/tmp/auto_calib_20260815_light_on_v15`에
심볼릭 링크만 만들어 다음과 같이 결정론적으로 연결했다. 이 표는 계산 입력 association이며
실제 동시 촬영·측정 관계를 뜻하지 않는다.

| scene | 조명 ON image | 연속 scan JSON |
|---:|---|---|
| 0 | `20260814-230655-CH1.jpg` | `calib-20260814-232414_sweep-000001_pan_tilt_lidar.json` |
| 1 | `20260814-230728-CH1.jpg` | `calib-20260814-233403_sweep-000001_pan_tilt_lidar.json` |
| 2 | `20260814-230731-CH1.jpg` | `calib-20260814-234352_sweep-000001_pan_tilt_lidar.json` |

공통으로 Manual intrinsic을 고정하고 raw image에 distortion correction을 적용했다.
camera center prior는 `(0.05928, -0.08105, 0) m`, LDC 상태는 `false`, 탐색 범위는
yaw `165~171°`/down `27~33°`/roll `0°`, 간격은 `1°`다. 목적함수 가중치는 geometry
NID `0.35`, edge `0.25`, structural line `0.25`, Manhattan direction `0.15`이고
signal NMI 가중치는 `0`이다.

### v15: 기존 v12 RT 고정 교차 회차 진단

v12 RT를 전혀 최적화하지 않고 새 3개 observation에 그대로 적용했다.

| scene | visible/aligned edge | aligned ratio | mean edge | geometry NID | 구조선 대응 | 수직 오차 | 결과 |
|---:|---:|---:|---:|---:|---:|---:|---|
| 0 | 380/252 | 0.6632 | 25.9134 px | 0.954940 | 25/25 | 4.0758° | PASS |
| 1 | 143/95 | 0.6643 | 23.0789 px | 0.954377 | 22/22 | 3.3905° | PASS |
| 2 | 125/99 | 0.7920 | 19.1769 px | 0.962132 | 25/25 | 3.0148° | PASS |

결과 수치는 `3/3 PASS`다. 다만 두 회차 사이 설치 교란 가능성이 뒤늦게 확인됐으므로
이는 동일 RT의 외부 반복성 PASS가 아니다. 기존 v12 RT가 작은 설치 변화 가능성이 있는
새 회차에서도 현재 gate를 통과했다는 **교차 회차 강건성/민감도 진단**이다. 동시에 현재
gate가 이 정도 차이를 구분하지 못한다는 의미일 수 있으므로 conformance 근거로 쓰지 않는다.

### v16: 20260815 installation epoch의 RT 재추정

새 데이터만으로 동일한 1° 탐색과 연속 refinement를 다시 수행했다. 세 observation은
모두 최적화 입력이며 별도 hold-out은 없다.

| scene | visible/aligned edge | aligned ratio | mean edge | p50 / p90 | 30 px 초과 | geometry NID | 결과 |
|---:|---:|---:|---:|---:|---:|---:|---|
| 0 | 372/295 | 0.7930 | 18.6291 px | 13.0 / 46.1 px | 20.91% | 0.936950 | PASS |
| 1 | 142/111 | 0.7817 | 16.1479 px | 11.0 / 38.3 px | 21.83% | 0.938573 | PASS |
| 2 | 121/96 | 0.7934 | 17.9024 px | 12.0 / 43.3 px | 20.66% | 0.942369 | PASS |

선택 자세는 yaw `166°`, grid down `29°`, refined down `27.0676°`, roll `0°`이며
training scene `3/3 PASS`다. v12와 v16의 최종 RT 차이는 다음과 같다.

| 비교 항목 | v12 | v16 | 차이 |
|---|---:|---:|---:|
| 선택 yaw | 167° | 166° | -1° |
| refined down | 27.3755° | 27.0676° | -0.3078° |
| 회전 geodesic | - | - | **1.6873°** |
| translation | `(0.05327, 0.06846, 0.05064) m` | `(0.05545, 0.06800, 0.04892) m` | **2.8198 mm** |

이 차이는 `20260813→20260815` 사이 실제 상대 자세 변화, 장면/가림 변화에 따른 최적화
불확실성, 또는 두 영향의 합일 수 있다. 별도 ground truth가 없으므로 `1.6873° /
2.8198 mm`를 실제 장치 이동량이나 알고리즘 반복 오차 중 하나로 단정할 수 없다.

최종 투영을 직접 확인한 결과 캐비닛–벽 수직 경계, 책상 상판 전면선, 바닥/벽 접점의
방향은 세 scene에서 일관된다. 큰 잔차는 캐비닛 내부 반복선, 케이블, 열린 책상 내부,
가림 경계처럼 LiDAR 구조선과 camera edge가 1:1로 대응하지 않는 부분에 집중된다.

### 정정 판정과 남은 절차

- `20260815` 데이터 자체의 최적화 품질 gate는 `3/3 PASS`지만, **`20260813→20260815`
  동일 RT 시간 반복성은 미검증**이다.
- v12 고정 적용 `3/3 PASS`는 설치가 같았다는 증거가 아니다. 현재 gate의 작은 자세 변화
  허용 가능성을 함께 보여주는 교차 회차 진단이다.
- 회전 `1.6873°`, 이동 `2.8198 mm`는 실제 설치 변화와 추정 변동이 혼합될 수 있는
  관측값이며 제품 conformance 허용치로 선언하지 않는다.
- 세 image가 같은 정적 화각에서 가까운 시간에 획득됐고 scan도 연속 수집이므로 표본은
  상관돼 있다. 다른 날짜의 추가 회차로 RT 분산을 누적해야 한다.
- signal-strength NMI perturbation 진단은 이번에도 `FAIL`이고
  `activation_recommended=false`이므로 가중치 `0`을 유지한다.
- 동일 코드 기준 Docker CTest는 `5/5 PASS`다.
- 다음 단계는 새 기준 installation epoch를 지정하고, camera·LiDAR·actuator를 건드리지
  않은 상태로 날짜 분리 회차를 누적하는 것이다. actuator를 다시 건드린 경우 새 epoch로
  번호를 올리고, 가능하면 전후 기구 측정값 또는 Manual RT를 별도 reference로 남긴다.
- 다른 공간/구조/조명 일반화는 별도 시험이다.

산출물:

```text
automatic_calibration/generated/
  repeat_test_sample_20260815_light_on_fixed_v12_rt_v15/
    fixed_pose_validation_result.json
    fixed_pose_scene_validation.csv
    debug/scene_*/06_projection_final.png
    debug/scene_*/07a_projection_final_edge_residual.png
  repeat_test_sample_20260815_light_on_repeat_estimation_v16/
    calibration_result.json
    training_scene_validation.csv
    debug/debug_summary.csv
    debug/scene_*/06_projection_final.png
    debug/scene_*/07a_projection_final_edge_residual.png
```

## 16. 20260815 추가 저조도 사진 진단 (2026-08-15)

### 데이터와 시험 분리

`20260815` 폴더에 `18:28:18`, `18:28:21`, `18:28:23` CH1 사진 3장이 추가됐다.
직접 확인한 결과 조명 ON `23:06` 사진과 달리 `18:13` 사진군과 같은 조명 OFF 저조도
영상이며, 해상도는 기존 Manual intrinsic과 같은 `2592×1520`이다. 나이트비전 제품
시험으로 분류하지 않고 현재 조명 ON 주 경로와 분리된 참고 진단으로 실행했다.

새 18:28 사진 3개와 기존 연속 scan JSON 3개를 사전식으로 한 번씩 연결했다. 조명 조건
실패의 반복성을 확인하기 위해 18:13 사진 3개도 같은 scan에 연결해 대조군으로 실행했다.
같은 scan을 재사용했으므로 6개의 독립 observation으로 합산하지 않는다.

### 실행 결과

| 실행 | 영상 | RT 처리 | 장면 통과 | 상태 |
|---|---|---|---:|---|
| v17 | 18:28 조명 OFF | 조명 ON v16 RT 고정 | 0/3 | FAIL |
| v18 | 18:28 조명 OFF | RT 재추정 | 0/3 | FAIL, 후보 거절 |
| v19 | 18:13 조명 OFF | 조명 ON v16 RT 고정 | 0/3 | FAIL |

v17과 v19의 모든 장면에서 `EDGE_ALIGNMENT_POOR`와
`MANHATTAN_VERTICAL_ALIGNMENT_POOR`가 발생했다. 대부분의 장면에는
`EDGE_OVERLAP_INSUFFICIENT`도 함께 발생했다.

| 영상군/scene | aligned ratio | mean edge | Manhattan 수직 오차 |
|---|---:|---:|---:|
| 18:28 / 0 | 0.1129 | 226.8820 px | 50.9878° |
| 18:28 / 1 | 0.1056 | 196.3935 px | 50.8897° |
| 18:28 / 2 | 0.1157 | 179.8525 px | 51.1303° |
| 18:13 / 0 | 0.1344 | 190.0897 px | 51.1928° |
| 18:13 / 1 | 0.1549 | 191.8950 px | 51.2378° |
| 18:13 / 2 | 0.2479 | 180.9749 px | 51.1223° |

v18이 선택한 진단 후보는 yaw `169°`, grid down `27°`, refined down `27.0031°`였지만
`MANHATTAN_VERTICAL_ALIGNMENT_POOR`로 거절됐다. 조명 ON v16 RT 대비 회전 차이는
`2.1646°`, translation 차이는 `3.6400 mm`다. 이 값은 거절 후보의 진단값이며 활성
calibration RT로 사용하지 않는다.

### 원인 분석과 판정

- 조명 ON v16의 camera edge는 총 `162,746 px`, 구조선은 `316`개였으나 18:28
  저조도 v18에서는 각각 `10,417 px`, `72`개로 감소했다. 감소율은 `93.60%`,
  `77.22%`다.
- 밝은 영상에서 약 `4.66°`였던 선택 수직 방향 오차가 저조도에서 `51.63°`가 됐다.
  `03a_manhattan_vanishing_directions.csv`에서도 밝은 scene 0은 56-inlier 방향과
  약 5° 수직 오차를 찾았지만, 저조도는 14-inlier의 잘못된 방향을 선택했다.
- geometry NID는 저조도에서도 약 `0.95~0.97`로 높았지만 edge/구조선 대응은 실패했다.
  따라서 geometry NID 단독 점수로 PASS시키지 않고 복합 gate가 거절한 것은 정상이다.
- 모든 LiDAR 점을 그린 `06_projection_final.png`는 표면을 채우므로 얼핏 맞아 보일 수
  있다. 반면 `07a`에서 30 px 초과 edge가 v18 기준 `79.7~84.1%`로, 실제 구조 대응은
  성립하지 않았다.
- 18:13과 18:28 두 회차가 같은 실패 형태를 보여 단일 노출의 우연이 아니라 현재
  저조도 feature pipeline의 조건 의존성으로 판단한다.

현재 제품 범위가 조명 ON이므로 threshold를 완화하거나 v18 거절 RT를 채택하지 않는다.
저조도 지원이 요구될 때만 별도 camera profile/K 검증, 명암 정규화 또는 CLAHE A/B,
저조도용 edge/line 검출기와 독립 conformance 데이터를 별도 과제로 진행한다. 현재
`20260815` installation epoch의 활성 후보는 조명 ON에서 통과한 v16이다.

산출물:

```text
automatic_calibration/generated/
  repeat_test_sample_20260815_light_off_1828_fixed_v16_rt_v17/
  repeat_test_sample_20260815_light_off_1828_reestimation_v18/
  repeat_test_sample_20260815_light_off_1813_fixed_v16_rt_v19/
```

## 17. CH1 A4 ChArUco 부착 검증 (2026-08-18)

0815와 같은 고정 환경에 실측 A4 ChArUco 보드(한 칸 27 mm, marker 20 mm)를 부착한
CH1 영상을 추가했다. CH1에서 `16/17` marker, `22` ChArUco corner가 검출됐고
reprojection RMSE `1.2826 px`로 보드 인식은 PASS했다. CH2~CH4는 이번 범위에서
제외했다.

동일 timestamp LiDAR가 없어 0815 고정환경 scan 1개를 진단용으로 연결해 자동
캘리브레이션을 실행했다. 후보는 yaw `160°`, down `20°`, optical roll `0°`였으나
입력 pair가 1개라 최종 상태는 `FAIL / SINGLE_OBSERVATION_DIAGNOSTIC_ONLY`다.
해당 후보 RT는 운영 RT로 채택하지 않는다. 상세 기록과 산출물은
[`CH1_ARUCO_VALIDATION_20260818.md`](CH1_ARUCO_VALIDATION_20260818.md)에 있다.

### 17.1 동일 고정환경 3쌍 재실행

`repeat_test_sample/20260818`에 CH1 image-scan 3쌍을 추가했다. A4 보드는 세 이미지
모두 `17/17 marker`, `24/24 corner`로 완전 검출됐다. 자동 캘리브레이션은 yaw `170°`,
down `20°`, optical roll `0°` 후보를 선택했고 training scene `3/3` 및 전체 내부 gate를
통과했다.

결과는 `automatic_calibration/generated/ch1_20260818_three_pair_v1/`에 있다. 기존
Manual RT와는 회전 `10.6906°`, translation `139.17 mm` 차이가 나며 hold-out은 0개다.
따라서 상태를 `candidate PASS / verification pending`으로 유지하고 운영 RT를 교체하지
않는다. 상세 수치와 다음 검증 조건은
[`CH1_ARUCO_VALIDATION_20260818.md`](CH1_ARUCO_VALIDATION_20260818.md)의 8절에 기록했다.

### 17.2 네 번째 pair 고정 RT hold-out 검증

추가된 `20260818-155208-CH1.jpg`와
`calib-20260818-154229_sweep-000001_pan_tilt_lidar.json`을 네 번째 pair로 선택했다.
ChArUco는 `17/17 marker`, `24/24 corner`, RMSE `1.316 px`로 통과했다.

`ch1_20260818_three_pair_v1` RT를 고정하고 재최적화 없이 검증한 결과는 `1/1 PASS`다.
projected ratio는 `0.7759`, mean edge distance는 `20.57 px`, structural match는 `21/21`,
Manhattan vertical error는 `11.21 deg`였다. 직접 확인한 투영도 벽·캐비닛·책상 평면의
방향과 배치가 유지됐다. 다만 edge p90은 `55 px`이고 30 px 초과 비율은 `21.99%`라
pixel-level 정답으로 확정하지 않는다.

산출물은
`automatic_calibration/generated/ch1_20260818_holdout_155208_fixed_v1/`에 있다. 파일명
시각은 9분 39초 차이가 나지만, 사용자가 해당 시간 동안 카메라·LiDAR actuator·장면이
모두 고정돼 있었음을 `2026-08-18` 확인했다. 따라서 현재 고정환경 RT 재현성을 평가하는
유효한 독립 hold-out pair로 확정한다. 현재 상태를
`candidate PASS / fixed-environment hold-out PASS`로 갱신한다. 이 판정은 센서 시간
동기화 검증을 뜻하지 않으며, Manual RT와의 절대 차이 검증은 별도 미완료 항목으로
유지한다. 상세 내용은
[`CH1_ARUCO_VALIDATION_20260818.md`](CH1_ARUCO_VALIDATION_20260818.md)의 9절에 기록했다.

### 17.3 20260818 고정환경 수집 확인 정정

운영자 확인에 따라 `repeat_test_sample/20260818`의 CH1 네 세트는 카메라·LiDAR·
pan-tilt actuator와 장면을 수집 중 이동시키지 않은 동일 installation epoch
(`session-const-env-20260818-ch1-fixed`)의 고정환경 데이터로 확정한다. 최근 추가된
파일도 이 epoch에 포함한다. 따라서 scene 0~2를 RT 추정에 사용하고 scene 3을 고정
RT hold-out으로 사용하는 분할을 유지한다. 날짜 또는 파일명 시각이 다르다는 이유로
이 세트를 서로 다른 설치로 분류하지 않는다.

다만 JSON에는 wall-clock 촬영시각이 없고 monotonic timestamp만 있으므로 이 판정은
정적 장면·상대 자세의 반복성에 대한 것이며, 카메라와 LiDAR의 시간 동기화를 의미하지
않는다. 조명 ON을 공식 조건으로 유지하고, actuator·zoom·focus·LDC·영상 방향을
변경하면 새 `installation_epoch`로 분리한다.

동시 촬영을 뜻하는 시간 동기화 검증과는 구분하며, 세부 파일 목록과 pairing 규칙은
[`CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md`](CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md)에
기록했다.

### 17.4 최신 파일 포함 재실행 확인

최근 추가된 `20260818-155208-CH1.jpg`와
`calib-20260818-154229_sweep-000001_pan_tilt_lidar.json`을 포함해 네 세트를 다시
실행했다. 파일명 순서 기준 scene 0~2를 training, scene 3을 hold-out으로 사용했고,
결과는 `automatic_calibration/generated/ch1_20260818_four_pair_recheck_v2/`에 저장했다.

`PASS`, training `3/3`, hold-out `1/1`, 선택 down `20.0°`(refined `19.9989°`),
optical roll `0.0°`가 확인됐다. hold-out은 visible/aligned edge `241/187`, mean edge
distance `20.574 px`, geometry NID `0.948202`, structural match `21/21`로 기존
`ch1_20260818_holdout_155208_fixed_v1` 결과와 동일했다. 최신 파일이 기존 RT의 반복성
판정을 변경하지 않았다는 뜻이다. 이는 동일 고정환경 epoch 내부의 재현성 확인이며,
다른 설치·구조로의 일반화나 Manual RT와의 절대 정확도 인증은 아니다.

### 17.5 최신 운영자 확인 반영

2026-08-18 최신 확인에 따라 `repeat_test_sample/20260818` 디렉터리의 현재 네 세트는
수집 회차 전체가 고정된 환경이었다는 전제로 유지한다. 카메라·LiDAR·pan-tilt actuator의
상대 설치, 카메라 영상 설정, A4 ChArUco 보드와 정적 장면을 수집 중 변경하지 않은 동일
installation epoch다. 따라서 이 세트의 scene 0~2 학습 및 scene 3 고정 RT hold-out 분할은
유효하다.

이 확인은 정적 장면에서의 외부 파라미터 반복성에 한정하며, JSON에 wall-clock 촬영시각이
없으므로 센서 시간 동기화를 의미하지 않는다. actuator·zoom·focus·LDC·영상 방향 또는
장면이 바뀌면 새 `installation_epoch`로 분리한다. 상세 입력 목록과 pairing 규칙은
[`CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md`](CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md)를
기준으로 한다.

## 18. 2026-08-20 MVP 제품 정책 정정

최근 논의에 따라 다음 항목을 현재 구현·문서의 기준으로 확정한다.

### 18.1 K/D와 LDC

- 제품 실행은 동일 channel/resolution/zoom/focus의 **Manual ChArUco `K + distortion` profile**을 입력으로 받는다.
- ChArUco는 제품 실행 중 매번 촬영하는 표적이 아니라 profile을 사전에 만들고 검증하는 절차다.
- Automatic은 profile의 `K,D`를 변경하지 않고 `R,t` 외부 파라미터만 추정한다.
- 현재 조명 ON 실험은 raw image에 Manual D를 적용한 undistort 경로를 사용했다. LDC가
  `true`인 rectified profile을 사용할 때는 그 출력에 맞는 K를 별도 등록하고 중복 보정하지 않는다.
- LDC 또는 raw/rectified 상태가 확정되지 않은 실행은 제품 승인 대상이 아니라 `DIAGNOSTIC_ONLY`다.

### 18.2 K+RT 공동 추정 보류

코드에 남아 있는 `--allow-intrinsic-refinement` 및 관련 API는 논문 재현·민감도 진단을
위한 연구 경로다. 현재 데이터로는 K와 RT의 관측성 및 ground truth를 동시에 검증할 수
없으므로 MVP 제품 경로에서 비활성화하고, Manual K/D 고정 외부 RT 추정만 진행한다.

### 18.3 초기 방향 탐색과 품질 판정

coarse yaw/down/optical-roll 탐색, 8-neighbor 보정점수, contiguous basin은 방향 후보를
진단하고 Ceres `R,t`의 시작점을 만드는 단계다. 구조가 반복되는 천장·바닥·벽 장면에서는
낮은 raw/NID score가 실제 정합을 보장하지 않으므로 탐색 결과만으로 RT를 활성화하지 않는다.

현재 결과 상태는 다음 세 등급으로 구분한다.

| 상태 | 의미 | 제품 RT 승격 |
|---|---|---:|
| `INTERNAL_GATE_PASS` | 입력 score·overlap·구조 gate 통과 | 불가 |
| `CANDIDATE_RT` | training/제한 hold-out에서 재현된 후보 | 불가 |
| `PRODUCT_APPROVED_RT` | 독립 기준·반복성·실패 안전성까지 통과 | 가능 |

2026-08-20 CH1의 2-train/1-hold-out 결과는 `CANDIDATE_RT`이며 제품 승인으로 해석하지
않는다. gate 실패 시 후보 산출물과 원인을 보존하고 기존 active RT를 유지한다. threshold를
낮추거나 실패 후보를 강제 승격하지 않는다.

### 18.4 다음 승인 조건과 Top-view 경계

제품 승격 전 최소 조건은 독립 hold-out 3쌍 이상, 동일 설치 반복 10회의 회전 표준편차
`≤0.2°`·이동 표준편차 `≤10 mm`, 초기 perturbation 복원률 `≥90%`, 독립 `T_camera_lidar`
기준과의 오차, false activation 0건이다. 최종 Top-view/Qt 시각화는 별도 Qt 세션에서
진행하며 Calibration Core의 품질 승인과 분리한다.

상세 정책은 [`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md), 방법 비교는
[`TARGETLESS_CALIBRATION_METHOD_REVIEW.md`](TARGETLESS_CALIBRATION_METHOD_REVIEW.md),
Manual profile 연계는 [`MANUAL_REFERENCE_PRIOR_WORKFLOW.md`](MANUAL_REFERENCE_PRIOR_WORKFLOW.md)를
참조한다.

### 18.5 2026-08-20 구현 반영

위 정책을 실행기에 반영했다. 기본 경로는 coarse score map → 서로 30° 이상 떨어진
top-3 contiguous basin → 5° local search → 1° local search → 최대 3개 final Ceres
순서이며, 실패 시 다른 후보를 fallback으로 고르지 않는다. `CalibrationResult`와 최종 JSON은
`INTERNAL_GATE_PASS`, `CANDIDATE_RT`, `PRODUCT_APPROVED_RT`를 별도 필드로 기록한다.
K+RT 공동 추정은 명시적인 experimental flag 없이는 실행되지 않으며, 수동 ChArUco K+D
profile이 제품 경로의 필수 입력이다. Manual reference 주변 signal-strength NMI
perturbation은 `--reference-rt-perturbation-only true`로 별도 진단한다.

구현 상세·실행 예·검증 이력은
[`CALIBRATION_CORE_IMPLEMENTATION_CHANGELOG_20260820.md`](CALIBRATION_CORE_IMPLEMENTATION_CHANGELOG_20260820.md)에 기록했다.

### 18.6 2026-08-20 Manhattan 잔차·staged A/B 결과

Manhattan 방향 잔차의 `15°` hard clipping을 연속 Huber형 잔차로 변경하고, staged
5°/1° 후보 선택에 Manhattan·구조 gate 우선순위를 추가했다. Docker CTest 5개는 모두
통과했다.

동일한 20260818 CH1 네 쌍과 동일한 Manual K+D/카메라 중심/탐색 범위로 legacy와 staged를
비교한 결과는 둘 다 `EDGE_ALIGNMENT_POOR`였다. legacy는 평균 edge `138.32 px`, staged는
`120.89 px`였고, staged는 edge/NID/spatial coverage를 각각 `0.979/0.995/0.969`까지
확보했지만 영상 엣지 위치 gate를 통과하지 못했다. training 및 hold-out은 두 전략 모두
`0/3`, `0/1`이다.

따라서 이번 수정은 큰 Manhattan 오차가 포화되어 Ceres가 회복하지 못하는 문제와 staged
후보의 무조건 선택 문제를 보완한 것이며, 현재 입력의 제품 RT 승인까지 완료한 것은
아니다. A/B 산출물은
`generated/ab_legacy_20260820/` 및 `generated/ab_staged_20260820/`에 보관했다. 기존
`ch1_20260818_four_pair_recheck_v2` PASS는 coverage gate 도입 전 생성된 결과라 이번
A/B의 직접 회귀 기준으로 사용하지 않는다. 다음은 coverage/edge 목적함수와 실제 영상
엣지의 위치 불일치를 별도로 분석한 뒤 재검증하는 단계다.
현재 score map에서 기존에 양호했던 yaw `170°` 후보는 직접 edge support가 `739`개였지만
같은 layer 최대값 대비 edge coverage가 `0.212`로 계산되어 `minimum_relative_edge_coverage=0.50`
에 의해 coarse 후보에서 제외됐다. 따라서 다음 수정은 search step을 더 줄이는 것보다
카메라 유효 시야를 반영한 coverage 기준을 재설계하는 것이 우선이다.

### 18.7 2026-08-20 edge soft 정책과 cross-epoch 한계

상기 18.6의 원인을 반영해 relative edge coverage hard gate를 제거했다. edge support는
`coverage_penalty_weight=0.25`의 soft penalty로만 남기고, relative NID/영상 공간
coverage `0.5`와 absolute overlap·구조·Manhattan gate는 유지했다. 이전
`--minimum-relative-edge-coverage` CLI 옵션은 삭제됐다.

staged 검색은 서로 다른 local 해를 끝까지 비교하기 위해 yaw `30°` 이상 떨어진
contiguous basin을 최대 3개 유지한다. 각 후보를 5° → 1° → Ceres까지 독립 실행하고,
**training scene validation → core gate → objective**로 최종 후보를 고른다. hold-out은
후보 선택에 사용하지 않고 선택된 RT에 대해서만 한 번 검증한다.

동일한 20260818 CH1 네 쌍에서는 yaw `169°`, down `21°`, roll `3°`를 선택해 training
`3/3`, hold-out `1/1`, 평균 edge `21.05 px`로 `CANDIDATE_RT`를 회복했다. 이는 hard
gate가 배제했던 기존 `≈170°` 방향과 일치한다.

하지만 20260819 CH1 세 쌍은 자체 training `2/2`·hold-out `1/1`을 통과해도 선택 RT가
20260818 대비 회전 `76.45°` 차이다. 사용자가 확인한 조건처럼 camera와 LiDAR가 같은
강체 모듈로 함께 이동했다면 두 extrinsic은 원칙적으로 같아야 하므로, 이 `CANDIDATE_RT`는
제품 승격 근거가 아니다. 현재 `activation_allowed=false`라 장치 RT는 갱신되지 않는다.
다음 승인 단계에는 독립 marker/manual RT 또는 승인된 이전 RT와의 post-search 회전
일관성 검증을 추가해야 한다. 이는 targetless 탐색을 reference에 고정하는 것이 아니라
후보를 제품 RT로 올리기 전의 독립 품질 검증이다.

### 18.8 2026-08-20 CH4 전용 K·D 및 hold-out 검증

CH1 전체 장면에서 A4 ChArUco가 검출되지 않아 CH4를 독립 채널로 검증했다. CH4의
보드 충분 영상 18장으로 만든
`manual_calibration/output/session-const-env/intrinsic-ch4-20260819-full/camera_intrinsic.json`
을 고정 사용했다. profile RMS는 `1.5427 px`이며 `fx=3125.0812`, `fy=3140.1864`,
`cx=1177.2851`, `cy=713.8399`, distortion model은 `opencv_radtan`이다. 41장을
필터링 없이 사용한 RMS `4.0848 px` REVIEW profile은 사용하지 않았다.

전체 장면 CH4 ArUco 검출은 `20260819-184401` 및 `184405`에서 각각 marker `6`,
ChArUco corner `6`, RMSE `0.4895/0.6342 px`로 PASS했다. 두 camera–board pose의
차이는 translation `0.358 mm`, rotation `0.073°`다. `200913`은 marker `2`, corner
`1`로 insufficient였다. 영상 안에 같은 ID 보드 복사본이 여러 장 있고 일부가
프레임 경계에서 잘려 있으므로 이 결과는 camera-side pose 일관성 진단으로만 본다.

CH4 image–LiDAR 3쌍은 다음처럼 2개 training과 1개 hold-out으로 구성했다.

```text
training: 20260819-184401-CH4.jpg ↔ calib-20260819-184249_*.json
training: 20260819-184405-CH4.jpg ↔ calib-20260819-185256_*.json
hold-out: 20260819-200913-CH4.jpg ↔ calib-20260819-200851_*.json
```

최신 staged 실행 산출물은
`automatic_calibration/generated/ch4_20260819_staged_kd_holdout_20260820_v1/`다.
Manual K·D 고정, raw+D undistort, `ldc=false`, camera-center
`(0.05928,-0.08105,0)m`, yaw 5°, down `0~30°/5°`, optical roll `±15°/5°`를
사용했다. 결과는 `CANDIDATE_RT`, training `2/2 PASS`, hold-out `1/1 PASS`이며
선택 방향은 yaw seed `97°`, down `11°`, optical roll `9°`다. 최종 평균 edge는
`33.22 px`, geometry NID `0.8645`, Manhattan vertical error `6.53°`, edge coverage
`0.9930`, NID coverage `0.9912`다.

이 결과는 이전 CH4 v3의 `1/3 per-scene FAIL`을 개선한 내부 검증 결과다. 그러나
ArUco가 제공하는 `T_camera_marker_board`만으로 `T_lidar_marker_board`를 알 수 없기
때문에 절대 camera–LiDAR RT ground truth나 제품 승인으로 해석하지 않는다. 현재
`activation_allowed=false`를 유지한다. 상세 조건·행렬·투영 파일은
[`CH4_ARUCO_KD_HOLDOUT_VALIDATION_20260820.md`](CH4_ARUCO_KD_HOLDOUT_VALIDATION_20260820.md)에
기록했다.

### 18.9 2026-08-21 Gemini 고도화 로직 평가 및 수정 우선순위

Gemini가 추가한 basin 탐색, 지면/천장 판정, 비대칭 구조선 가중치, normal-gated
matching, TESL, multi-criteria confidence를 코드·테스트·실데이터 결과 기준으로
재평가했다. 독립 Docker clean build와 Core/M2 테스트는 통과했지만, 20260819에서
`yaw=-123°`의 희소 false basin이 `CANDIDATE_RT`로 선택되었다. 같은 실행의
`yaw=165°` 후보가 visible edge와 설치 조건 측면에서 더 타당했으므로 최종 RT 자동화
완료로 판정하지 않는다.

핵심 문제는 다음과 같다.

1. 다중 장면 구조선 집계에서 TESL/asymmetric 값이 누락되어 finalist TESL이 0으로
   기록된다.
2. finalist마다 별도로 상대 coverage를 정규화해 visible edge가 적은 후보가 유리해질
   수 있다.
3. staged NID 개선 기준이 기존 1%에서 0%로 완화되었다.
4. CTest가 물리 RT 정답을 검사하지 않아 false pass를 잡지 못한다.
5. normal 사영식, ground plane 의미 판정, ignored dataset 경로는 추가 검증이 필요하다.

수정 우선순위와 승인 조건은
[`GEMINI_LOGIC_EVALUATION_AND_REMEDIATION_PLAN_20260821.md`](GEMINI_LOGIC_EVALUATION_AND_REMEDIATION_PLAN_20260821.md)에
기록했다. 해당 수정 및 재검증 전까지 `activation_allowed=false`와
`NOT_PRODUCT_APPROVED_RT`를 유지한다.

### 18.10 2026-08-21 Jenkins scene0 CH1 4묶음 반복성 검증

`data/jenkins-capture/scene0`의 같은 설치·다른 시각 Jenkins 패키지 4개를 CH1 기준으로
검증했다. 기존 runner가 중첩 패키지와 multi-channel 파일, `manifest.json`을 직접
처리하지 못해 CH1 suffix filter, LiDAR JSON suffix filter, 부모 패키지 pairing,
scan-name 시간순 정렬을 추가했다. 수정 후 원본 scene0 루트를 직접 입력해 빌드와
실데이터 실행을 완료했다.

시간순 앞 3개를 training, 마지막 1개를 hold-out으로 둔 결합 실행은 yaw `170°`, down
`29°`, roll `-1°`를 선택했고 training `3/3`, hold-out `1/1`로 `CANDIDATE_RT`를
반환했다. 2D/3D 시각화에서도 결합 방향은 장면과 대체로 일치했다. 그러나 마지막
hold-out CH1 영상은 세 번째 training 영상과 SHA-256이 같아 독립 LiDAR sweep만 검증한
`limited hold-out`이다.

세 training pair를 각각 단독 실행한 결과는 yaw `-128°`, `-190°`, `-190°`였고 모두
내부 PASS였다. pair 0↔1 회전 geodesic은 `64.548°`, pair 0↔2는 `68.564°`, pair 1↔2도
`8.190°`다. 따라서 같은 설치에서 같은 RT가 나와야 한다는 반복성 조건은 FAIL이다.
pair 0에서는 잘못된 `-128°` 후보와 `-185°` 후보의 confidence 차이가 약 `0.00622`뿐인데
finalist ambiguity gate가 없어 잘못된 후보가 선택됐다. 또한 scene CSV에는 TESL 값이
있지만 finalist aggregate는 0으로 남는 기존 문제도 재확인했다.

결합 결과는 진단용 후보로만 보존하고 `activation_allowed=false`를 유지한다. 상세
데이터 해시, 명령, R·t, pairwise 차이, 시각 판정과 산출물은
[`JENKINS_SCENE0_CH1_REPRODUCIBILITY_20260821.md`](JENKINS_SCENE0_CH1_REPRODUCIBILITY_20260821.md)에
기록했다.

### 18.11 2026-08-23 build20/build21 원인 수정 및 최신 상태

build20/build21 수정 영상과 build17~21 batch를 이용해 현재 문서·코드·결과를 다시
감사했다. build21은 full coarse에서 정상 `yaw≈165°` 후보를 찾았으나, 서로 다른 FOV의
global maximum NID를 local hard gate 기준으로 사용해 정상 basin이
`COARSE_OVERLAP_INSUFFICIENT`로 제거됐다. 단순히 relative NID threshold를 0으로 낮춘
A/B에서는 `yaw≈85°` false basin이 confidence 우선으로 내부 PASS해 임계값 완화가 해결책이
아님을 확인했다.

다음 변경을 반영했다.

- 5°/1° NID relative hard gate를 basin-local reference로 제한
- global NID는 final objective/confidence의 soft 진단값으로 유지
- 최대 3개 Ceres finalist를 objective 2% 유의차 → near-tie TESL 10% → confidence로 선택
- objective와 confidence margin이 모두 부족할 때 ambiguity fail
- 경쟁 후보 대비 visible/NID support 60% 미만이면 fail
- 3-candidate threshold 순환을 막는 명시적 결정론 선택과 6-permutation 회귀 추가

최종 build17~19 training + 수정 build20~21 hold-out 실행은 `yaw=167°`,
`down≈37.16°`, `roll=7°`, training `3/3`, hold-out `2/2`로 `CANDIDATE_RT`를 반환했다.
objective margin은 `7.954%`, confidence margin은 `1.344%`다. 2D/3D 시각 검토에서
gross 방향 반전은 제거됐지만 모든 영상 경계에 정밀하게 일치하는 ground truth 수준은
아니다.

현재 판정은 다음과 같다.

- Calibration Core: 후보 자동 추정·내부 gate 가능
- 현재 최고 수명주기: `CANDIDATE_RT`
- 제품 RT 자동화: 미완료
- `product_approved_rt_status=NOT_PRODUCT_APPROVED_RT`
- `activation_allowed=false`

이전 `169°/23°/5°` 후보와 최신 후보는 회전 약 `14.299°`, 이동 약 `20.91 mm` 차이가
있으므로 algorithm version 없이 RT만 교체하면 안 된다. 독립 물리 reference, 원본·동시
hold-out 3쌍 이상, 동일 설치 10회 반복성, candidate별 hold-out margin이 다음 P0/P1이다.

상세 보고서:
[`CALIBRATION_LOGIC_EVALUATION_AND_IMPROVEMENT_20260823.md`](CALIBRATION_LOGIC_EVALUATION_AND_IMPROVEMENT_20260823.md)

### 18.12 2026-08-24 finalist별 hold-out 식별성 검증

사용자가 수정했다고 알린 build20/build21 CH1은 2026-08-23 최신 수정본과 raw SHA-256,
prepared pixel SHA-256이 모두 같았다. 동일 입력과 동일 search logic으로 build17~21을
재실행했고, RT와 full/5°/1° score map도 이전 deterministic 결과와 동일했다.

이번에는 선택된 167° 후보뿐 아니라 training/core/absolute support를 통과한 87°와
−106° finalist도 build20·21에 고정 적용했다. 세 후보 모두 hold-out `2/2 PASS`였다.
선택 yaw와의 거리는 각각 80°와 87°이므로 같은 basin으로 볼 수 없다. 따라서 기존
`CANDIDATE_RT`는 hold-out이 후보를 식별한다는 근거가 부족했고, binary 중간 판정은
아래와 같다.

- training: `3/3 PASS`
- 선택 후보 hold-out: `2/2 PASS`
- 동일 hold-out을 통과한 separated viable finalist: `2개`
- status: `INTERNAL_GATE_PASS`
- reason: `FINALIST_HOLDOUT_AMBIGUOUS`
- candidate: `NOT_CANDIDATE_RT`
- product: `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`

실패 시 `estimated_t_camera_lidar`는 안전상 prior로 남고 167° RT는
`diagnostic_candidate_t_camera_lidar`와 `visualization_t_camera_lidar`에만 기록된다.
자세한 코드 변경, 후보별 CSV와 다음 고도화 순서는
[`FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md`](FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md)를
따른다.

### 18.13 2026-08-24 학습 동일 hold-out 목적함수 적용

§18.12의 binary pass-ratio는 167°/87°/−106° 후보가 모두 build20·21에서
`2/2 PASS`라 후보를 구분하지 못했다. 별도 가중치를 만들지 않고 Core 학습에서 사용하던
Edge 0.25, geometry NID 0.55, 구조선 0.20, Manhattan 0.15, coverage 0.25
목적함수를 fixed-pose hold-out 평가에 그대로 적용했다. Coverage 분모는 후보별 자기
최대가 아니라 같은 hold-out 모든 finalist의 공통 최대 support다.

같은 pass-ratio tier의 분리 viable 후보는 기존 objective margin 2% 기준으로 비교한다.
build17~21에서 선택 167° 목적함수는 `0.763763`, 87°는 `0.816782`, −106°는
`0.871447`이었다. 최소 우위 `6.491%`로 최신 판정은 `CANDIDATE_RT / PASS`다.
전역/5°/1° score map과 RT 해시는 이전 실행과 같아 탐색 자체는 변경되지 않았다.

독립 회귀에서 20260818은 training `FINALIST_AMBIGUOUS`를 유지했고, 20260819는 yaw
80°가 165° 경쟁 후보보다 hold-out 목적함수 `9.882%` 우수해 `CANDIDATE_RT`가 됐다.
두 candidate 모두 물리 참값 승인이 아니므로 `NOT_PRODUCT_APPROVED_RT`와
`activation_allowed=false`를 유지한다. 빠른 회귀는 `9/9 PASS (88.52 s)`였고,
20260818/19는 Jenkins weekly expected-rejection/candidate-regression으로 분리했다.

### 18.14 2026-08-24 Manhattan prior 일관성 수정과 1회 검증

추가 코드 감사에서 training은 각 finalist의 `seed.prior`로 영상 수직 소실점 축을
선택하지만 fixed-pose/hold-out은 refined candidate RT로 그 축을 다시 선택한다는 차이를
발견했다. 후보마다 평가 특징 자체가 바뀔 수 있는 불공정 비교이므로 다음처럼 수정했다.

- training, scene pass-ratio, hold-out 모두 finalist별 동일 `seed.prior`로 Manhattan
  영상 특징을 고정한다.
- refined RT는 투영과 잔차 계산에만 사용한다.
- 명시적 prior 선택 회귀와 결과 JSON policy 필드를 추가했다.

수정본으로 build17~21, 20260818, 20260819를 각각 1회 실행했다. 세 실행 모두 수정 전과
선택 `R,t`, 상태, hold-out 목적함수, full/5°/1° score map이 완전히 같았다. 최신 결과는
build17~21 `CANDIDATE_RT/PASS`(167°, 최소 margin 6.491%), 20260818
`FAIL/FINALIST_AMBIGUOUS`, 20260819 `CANDIDATE_RT/PASS`(80°, margin 9.882%)다.
빠른 회귀는 `9/9 PASS (77.19 s)`, 새 prior 회귀를 포함한 Core는 `1/1 PASS
(5.51 s)`였다. 결과 불변은 현재 fixture에서 같은 소실점 축이 선택됐다는 뜻이며,
일관성 결함을 그대로 둘 근거는 아니다. 제품 상태는 계속
`NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`다.

상세 근거와 최신 산출물은
[`FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md`](FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md)의
§9를 따른다.

### 18.15 2026-08-24 1회 제한 해제와 weekly 계약 검증

초기 prior 수정 비교에 적용했던 fixture별 1회 제한을 해제하고, 20260818/19를 정식
weekly CTest로 다시 실행했다. 기존에는 예상 거절만 exit와 reason을 함께 검사하고
성공 케이스는 exit 0만 확인했으므로, 두 테스트를 공통
`verify_real_calibration_result.cmake` 경로로 통합했다.

- 빠른 회귀: `9/9 PASS (52.26 s)`
- weekly: `2/2 PASS`, 병렬 real time `1240.91 s`
- 20260818: exit 3, `FAIL / FINALIST_AMBIGUOUS`, `NOT_CANDIDATE_RT`
- 20260819: exit 0, `CANDIDATE_RT / PASS`, `CANDIDATE_RT`
- 공통 검사: status/reason, candidate/product 상태, `activation_allowed=false`, Manhattan
  feature-prior policy
- score map: prior-locked 실행과 full/5°/1° SHA-256 동일

따라서 현재 자동 테스트는 성공과 안전 거절을 모두 정상 결과로 구분할 수 있다. 그러나
이는 물리 RT 참값의 정확도 승인이 아니므로 두 결과 모두
`NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`다.

### 18.16 2026-08-24 Jenkins build22~24 CH1 ChArUco reference 준비

`data/jenkins-capture/scene0`에 추가된 build22~24의 CH1 image와 LiDAR scan을 감사했다.
세 package 모두 서로 다른 image/scan hash, `checksum_error_count=0`, valid point
40,183개 이상을 가지며 같은 고정 장면이다. 시간 순서와 hold-out 누수 방지를 기준으로
build22·23을 training, build24를 development temporal hold-out으로 고정했다. build24는
automatic 최적화에는 사용하지 않지만 이미 영상과 marker 검출을 확인했으므로 sealed
product hold-out으로 부르지 않는다.

CH1에는 같은 ID의 A4 ChArUco가 우측 모니터 앞과 파란 의자 위에 각각 한 장씩 있다.
전체 frame에서는 작은 marker와 ID 중복 때문에 검출되지 않았으나, 보드별 ROI와 crop
principal-point 보정을 적용하면 총 6개 pose가 모두 PASS했다.

- monitor ROI `2090,700,500,650`: 최대 반복 차이 `1.094355° / 10.795 mm`
- chair ROI `1200,1200,800,320`: 최대 반복 차이 `0.922760° / 4.229 mm`
- 구현: `estimate_marker_pose --roi x,y,width,height`
- 출력: `manual_calibration/output/jenkins-scene0/ch1-build22-24-charuco/`

이 결과로 `T_camera_marker_board` camera-side reference는 확보했다. 그러나 같은 보드의
독립 `T_lidar_marker_board`가 없으므로 전체 `T_camera_lidar` ground truth 상태는
`RT_REFERENCE_INCOMPLETE`다. Marker pose를 targetless 후보 선택에 주입하지 않고,
LiDAR-visible board pose가 추가된 뒤 다음 식으로 독립 RT reference를 만들어 build24에서
비교한다.

```text
T_camera_lidar_reference
  = T_camera_marker_board * inverse(T_lidar_marker_board)
```

현재 제품 상태는 계속 `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`다. 상세
역할, 입력 수치, ROI overlay, 명령과 후속 순서는
[`JENKINS_SCENE0_CH1_BUILD22_24_CHARUCO_REFERENCE_PLAN_20260824.md`](JENKINS_SCENE0_CH1_BUILD22_24_CHARUCO_REFERENCE_PLAN_20260824.md)를
따른다.

### 18.17 2026-08-24 CH1 build5~24 전체 활용 및 실행 계획 확정

현재 `scene0`의 12개 package를 모두 활용하되 하나의 optimizer 입력으로 섞지 않도록
역할을 재분류했다.

```text
Case A baseline: build5/8/9 training → build10 limited hold-out
Case B stress:   build17/18/19 diagnostic training → build20/21 dynamic hold-out
Case C primary:  build22/23 clean training → build24 development hold-out
Cross-check:     Case C RT를 Case A/B 전체에 재추정 없이 fixed 적용
```

CH1의 파란 의자/우측 모니터 보드를 build5~24 전부 ROI audit한 결과 24 target 중 22개가
PASS했다. 예상 거절은 편집된 build17 monitor의 0 corner와 강한 원근의 build18 chair 5
corner다. 기준을 낮춰 두 입력을 통과시키지 않으며 ChArUco는 camera-side 진단에만 쓴다.

Luna가 장시간 실행을 중복 시작하거나 pair index, exit 3, product 승인 상태를 오해하지 않도록
경로·명령·예상 결과·중단 조건·보고 형식을 단일 계획서에 고정했다. Docker CLI와 입력 경로,
incremental build, `manual_marker_tests` 1/1 PASS까지 검증했으며 full calibration 실행은 아직
시작하지 않았다.

상세 계획은
[`CH1_ALL_BUILD_DATA_UTILIZATION_AND_LUNA_EXECUTION_PLAN_20260824.md`](CH1_ALL_BUILD_DATA_UTILIZATION_AND_LUNA_EXECUTION_PLAN_20260824.md)를
따른다. 성공 가능한 최고 상태는 계속 `CANDIDATE_RT`, `NOT_PRODUCT_APPROVED_RT`,
`activation_allowed=false`다.

### 18.18 2026-08-24 CH1 전체 계획 실행 결과

Luna 실행 계획에 기록한 Step 0~7을 실제 Docker에서 완료했다.

| 단계 | 결과 |
|---|---|
| CH1 ChArUco 24-target audit | `22 PASS + 2 EXPECTED FAIL`; build17 monitor와 build18 chair는 corner 부족으로 fail-closed |
| Case C build22·23→24 | exit 0, `CANDIDATE_RT/PASS`, training 2/2, hold-out 1/1, margin 2.9436% |
| Case A build5/8/9→10 | exit 3, `FAIL/FINALIST_AMBIGUOUS`, training 3/3, hold-out 1/1, margin 1.8209% |
| Case B build17/18/19→20/21 | exit 0, `CANDIDATE_RT/PASS`, training 3/3, hold-out 2/2, margin 6.4912% |
| Case C RT fixed→Case A | exit 0, `INTERNAL_GATE_PASS`, 4/4 scene |
| Case C RT fixed→Case B | exit 0, `INTERNAL_GATE_PASS`, 5/5 scene |

Case C 후보의 선택 방향은 downward 약 `42°`, optical roll `3°`, seed yaw `-183°`이며,
Case B 후보는 downward 약 `37.16°`, optical roll `7°`, seed yaw `167°`이다. Case A는
hold-out에서 finalist 목적함수 margin이 2%에 못 미쳐 안전하게 후보 승격을 거절했다.

고정 RT 결과는 재추정 없이 입력 RT를 적용한 내부 일관성 검증이므로
`PRODUCT_APPROVED_RT`가 아니다. 상세 결과와 모든 JSON/CSV/PNG/PLY/OBJ 위치는
[`CH1_ALL_BUILD_EXECUTION_REPORT_20260824.md`](CH1_ALL_BUILD_EXECUTION_REPORT_20260824.md)를
따른다.

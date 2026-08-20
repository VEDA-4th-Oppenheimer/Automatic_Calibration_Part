# Automatic Calibration

저가 TOFSense F2P 1D LiDAR와 pan-tilt actuator로 정적 자연 장면을 scan하고,
카메라 영상과 depth edge를 targetless 방식으로 정합해 `T_camera_lidar`를 구하는
제품 개발 경로다.

현재 구현:

- Stanford 2D-3D-S 기반 synthetic pan-tilt producer
- Organized point cloud/range image
- Camera edge–LiDAR depth discontinuity 목적함수
- Single/multi-scene Ceres 6-DoF 최적화
- Ground-truth conformance와 overlay/PLY/OBJ 출력

실제 F2P/actuator producer, camera adapter, NMI와 Open Platform 검증은 후속 단계다.

## 빌드

Workspace 전체 또는 이 디렉터리만 독립 빌드할 수 있다.

```bash
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

전체 workspace 빌드 시 실행 파일은 `build/bin`에 생성된다. 이 디렉터리에서 직접
빌드할 때는 `cmake -S automatic_calibration -B build/automatic -G Ninja`를 사용할 수
있다.

## Synthetic scan

```bash
build/bin/generate_synthetic_scan \
  --dataset-root /path/to/area_1 \
  --output automatic_calibration/generated/area1_smoke \
  --columns 321 --rows 121 --pixel-stride 2 \
  --noise-stddev-m 0.005 --dropout 0.01 --seed 7
```

## Calibration

```bash
build/bin/run_synthetic_calibration --help
build/bin/run_multi_synthetic_calibration --help
```

### 천장 설치 실데이터

`run_real_calibration`은 공통 `roll=90°`를 더 이상 가정하지 않는다. 기본 실행은
카메라 optical axis의 down 방향을 0~90°에서 15° 간격으로, yaw를 360°에서
15° 간격으로 탐색한다. `--prior-roll-deg DEG`를 지정한 경우에만 down 방향을
하나로 고정한다. `camera-center-*-m`은 LiDAR frame(`+x right, +y down,
+z forward`)에서 본 각 채널 렌즈 중심이다.

```bash
build/bin/run_real_calibration \
  --input-dir /path/to/three-or-more-image-scan-pairs \
  --output automatic_calibration/generated/result \
  --camera-channel 3 \
  --ldc-enabled unknown \
  --zoom-focus-locked true \
  --camera-center-x-m 0.05928 \
  --camera-center-y-m -0.08305 \
  --camera-center-z-m 0 \
  --expected-forward-x 0 --expected-forward-y 0 --expected-forward-z -1 \
  --expected-down-x 0 --expected-down-y 1 --expected-down-z 0 \
  --direction-prior-weight 0 \
  --down-min-deg -30 --down-max-deg 30 --down-step-deg 5 \
  --optical-roll-min-deg -15 --optical-roll-max-deg 15 \
  --optical-roll-step-deg 5 \
  --focal-scale-min 0.9 --focal-scale-max 1.1 --focal-scale-step 0.1 \
  --legacy-range-offset-m 0.084
```

카메라 중심 offset과 광축은 독립된 값이다. CH1 설치 사진 기준 광축은 LiDAR `-Z`,
영상 아래 방향은 LiDAR `+Y`로 입력한다. 두 방향 벡터는 기계 설치 prior를 별도
시각화하기 위한 값이다. 자동 방향 탐색에서는 `--direction-prior-weight 0`으로 두고
down과 optical roll을 독립적으로 찾는다. 방향을 계측해 신뢰할 수 있을 때만 0보다 큰
가중치를 사용한다. 실측 camera center는 회전 후보마다 `t=-R*C`로 다시 계산하며
Ceres에서도 5 mm sigma 내에 유지한다.

실패 시 `matching_scene_*.png`와 일반 colorized PLY는 거절된 최적화 후보를 표시하고,
`mechanical_prior_*` 파일은 설치 prior를 표시한다. 두 자세가 서로를 덮어쓰지 않으며
`installation_constrained_rt.json`도 진단용으로만 저장된다.

1° full-search 진단은 `--yaw-step-deg 1 --down-step-deg 1`을 추가한다. 결과의
`orientation_full_search.csv`에는 yaw 360 × down 91 = 32,760개 raw score가,
`full_search_baseline_result.json`에는 후속 step-size 시험 허용 여부가 기록된다.
Reference RT가 없는 단일 관측은 `BLOCKED_BY_REFERENCE_UNAVAILABLE`로 유지한다.

이미 확인된 방향 주변만 1°로 재검증하려면 yaw 경계를 함께 지정할 수 있다.

```bash
--yaw-min-deg -180 --yaw-max-deg -160 --yaw-step-deg 1 \
--down-min-deg 0 --down-max-deg 10 --down-step-deg 1
```

구조선 목적함수는 인접한 승인 LiDAR 평면의 실제 교차선을 사용한다. fitted 선분의 양
끝이 화면 밖이어도 화면 안에 보이는 구간을 샘플링해 2D LSD 선분과 방향·거리·유한
구간 겹침을 평가한다. range discontinuity는 폐색 진단선으로만 저장한다. 후보별
수평/수직 비용과 유효 선분 수는 `orientation_full_search.csv`에 따로 기록된다. 실제
zoom/focus profile 민감도는 manual intrinsic 대신 제조사 FOV K 주변
`--focal-scale-min/max/step`(전체 허용 0.8~1.2)으로 비교한다.

`run_real_calibration`은 manual intrinsic 파일을 사용하지 않는다. 제조사 FOV
범위로 K를 초기화하고 구조가 다른 3개 이상의 관측에서 `fx,fy,cx,cy,R,t`를
공동 최적화한다. 계산 원본 frame의 `.ply/.obj`와 사람이 확인할 Z-up
`_z_up.ply/.obj`를 함께 생성하며, 3D PNG 미리보기는 Z-up으로 표시한다.
원본 파일은 point primitive이므로 면만 지원하는 VS Code 3D Viewer에서는
`_viewer_mesh.{ply,obj}` 또는 `_z_up_viewer_mesh.{ply,obj}`를 연다. 이 파일은 최대
12,000점을 반경 25 mm의 삼각형 tetra mesh로 변환한 호환용 미리보기이며 계산
입력이 아니다. 해당 확장은 back-face를 제거하므로 면 winding은 바깥쪽으로 만들고
vertex normal을 함께 기록한다.

### 단계별 디버그 산출물

중간 단계의 결과를 확인하려면 `--debug-output`을 추가한다. 일반 결과 디렉터리와
분리된 디버그 디렉터리에 장면별 이미지와 3D 데이터를 생성한다.

```bash
build/bin/run_real_calibration \
  --input-dir /path/to/three-or-more-image-scan-pairs \
  --output automatic_calibration/generated/result \
  --debug-output automatic_calibration/generated/result/intermediate
```

각 `intermediate/scene_N`에는 다음 파일이 생성된다.

- `01_input.png`: 입력 이미지
- `02_camera_gradient.png`: Sobel gradient magnitude (NID의 카메라 특징)
- `03_camera_edge_distance.png`: Canny edge distance transform
- `04_lidar_surface_normals.ply`: 거리 급변을 가로지르지 않고 계산한 surface normal
- `04a_lidar_plane_labels.ply`: 같은 평면으로 분할된 point를 동일 색으로 표시
- `04a_lidar_planes.csv`: 평면 normal·offset·지지점 수·RMS 오차
- `04b_lidar_plane_pair_candidates.csv`: 평면 쌍별 승인/탈락 단계와 근거
- `04b_lidar_plane_intersection_edges.{ply,obj}`: 인접한 두 평면의 실제 교차선
- `04c_lidar_occlusion_edges.{ply,obj}`: 거리 급변으로 생긴 장애물 실루엣(진단 전용)
- `04d_lidar_edges_used_for_calibration.{ply,obj}`: 목적함수에 실제 사용한 구조선
- `05_projection_initial.png`: 초기 RT prior를 적용한 LiDAR 투영
- `06_projection_final.png`: 최종 RT 또는 비활성 진단 후보 RT를 적용한 전체 LiDAR 투영
- `07_projection_final_edges.png`: 최종 RT를 적용한 LiDAR range edge 투영
- `debug_summary.csv`: valid point, normal, plane, 평면 교차선, 폐색선, 초기·최종 투영 개수

관측이 3개 미만이면 제조사 FOV 기반 K를 고정하고 pose 탐색은 진단 목적으로 실행하지만,
결과는 항상 `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL`이다. 이때 활성 RT는 prior를
유지하고 `diagnostic_candidate_t_camera_lidar`와 `top_candidates/rank_1.png`~
`rank_5.png`만 검토용으로 제공한다. 투영은 10 mm 허용 z-buffer로 가려진 뒤쪽 점을
제외한다. 투영점 개수는 화면 안에 보이는 점 수이며 매칭 정확도를 의미하지 않는다.

`mechanism.tilt_zero=nadir`는 모터 기구축 홈 메타데이터다. 좌표 계산에는
`measurements[].tilt_rad`를 그대로 사용하며, 130333 계약각은 0°=수평,
-90°=아래 방향이다. pan 360° sweep과 카메라 외부 yaw 후보는 동일한 변수가 아니다.
실장 확인 결과 pan 값 증가는 Top-view 기준 시계 방향이다.

고정환경 실측에서 카메라는 LiDAR보다 천장에 83.05 mm 더 가까우므로 `+y=down`
프레임에서는 camera center Y가 `-0.08305 m`다. X 부호는 실제 CH1 렌즈가 축의
어느 쪽에 있는지 설치 사진/실측으로 확인한 값을 사용한다.

다중 장면 경로는 카메라 gradient와 LiDAR range/surface-normal 특징의 NID를 70%,
edge 정합을 30%로 사용하는 복합 목적함수다. 초기 수평 방향은 15° 간격으로 360°를
탐색한다. 멀리 떨어진 두 방향의 점수가 비슷하거나 NID·중첩률·개선률이 부족하면
결과를 적용하지 않고 `MULTISTART_AMBIGUOUS`, `NID_OVERLAP_INSUFFICIENT` 등의
reason code와 기존 mechanical prior를 반환한다.

## 결과 시각화

```bash
build/bin/render_calibration_visualization \
  --dataset-root /path/to/area_1 \
  --result-json automatic_calibration/generated/calibration_core_multi_validation/calibration_result.json \
  --output automatic_calibration/generated/calibration_core_multi_validation/visualization
```

상세 문서:

- [`docs/CURRENT_PROGRESS_AND_STATUS.md`](docs/CURRENT_PROGRESS_AND_STATUS.md)
- [`docs/CALIBRATION_CORE_ARCHITECTURE.md`](docs/CALIBRATION_CORE_ARCHITECTURE.md)
- [`docs/SYNTHETIC_DATA_ARCHITECTURE.md`](docs/SYNTHETIC_DATA_ARCHITECTURE.md)
- [`docs/PAN_TILT_LIDAR_JSON_INTERFACE.md`](docs/PAN_TILT_LIDAR_JSON_INTERFACE.md)
- [`docs/JENKINS_CONFORMANCE_TEST_PLAN.md`](docs/JENKINS_CONFORMANCE_TEST_PLAN.md)
- [`docs/TARGETLESS_CALIBRATION_METHOD_REVIEW.md`](docs/TARGETLESS_CALIBRATION_METHOD_REVIEW.md)
- [`docs/OPENSDK_RT_INTEGRATION_HANDOFF.md`](docs/OPENSDK_RT_INTEGRATION_HANDOFF.md) — OpenSDK/CV5 영상·LiDAR 파이프라인, RT 계약, Core 실행 및 안전한 결과 적용 인계서

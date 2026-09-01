# Automatic Calibration

현재 실데이터 MVP 운용 정책은 [`docs/PRODUCT_CALIBRATION_POLICY.md`](docs/PRODUCT_CALIBRATION_POLICY.md)에
고정되어 있다. 제품 경로는 Manual ChArUco로 사전에 확보한 `K + distortion`을 고정하고
`R,t`만 추정한다. K+RT 공동 추정과 제조사 FOV K는 연구·진단용으로만 남겨 둔다.
2026-08-24 finalist별 hold-out에 학습과 같은 연속 목적함수와 공통 coverage 기준을
적용한 build17~21 결과는 최소 경쟁 margin 6.491%로 `CANDIDATE_RT / PASS`다. 제품
승인은 아니므로 `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`를 유지한다.
Manhattan 영상 특징은 finalist별 training seed prior로 고정해 hold-out에서도 동일한
소실점 축을 평가한다. 2026-08-24 일관성 수정 후 1회 재검증에서도 `R,t`와 판정은
수정 전과 동일했다.
최신 근거와 남은 승인 조건은
[`docs/FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md`](docs/FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md)를
따른다. 같은 날의 `FINALIST_HOLDOUT_AMBIGUOUS` 기록은 binary pass-ratio만 비교했던
중간 실험 이력이다.

저가 TOFSense F2P 1D LiDAR와 pan-tilt actuator로 정적 자연 장면을 scan하고,
카메라 영상과 depth edge를 targetless 방식으로 정합해 `T_camera_lidar`를 구하는
제품 개발 경로다.

현재 구현:

- Stanford 2D-3D-S 기반 synthetic pan-tilt producer
- Organized point cloud/range image
- Camera edge–LiDAR depth discontinuity 목적함수
- Single/multi-scene Ceres 6-DoF 최적화
- Ground-truth conformance와 overlay/PLY/OBJ 출력

실제 F2P/actuator producer, camera adapter, signal-strength NMI와 Open Platform 검증은 후속 단계다.

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

빠른 PR/daily 회귀와 장시간 실데이터 weekly 회귀는 분리한다.

```bash
ctest --test-dir build -E '^verify_' --output-on-failure
ctest --test-dir build -L weekly -j2 --output-on-failure
```

Weekly의 `verify_20260818_staged`는 성공 RT가 아니라 exit 3과
`FINALIST_AMBIGUOUS`를 기대하는 안전 거절 테스트다. `verify_20260819_staged`는
`CANDIDATE_RT / PASS`를 기대한다. 두 테스트 모두
`tests/verify_real_calibration_result.cmake`를 통해 exit code뿐 아니라 JSON의 상태,
reason, candidate/product 수명주기, `activation_allowed=false`와 Manhattan
feature-prior 정책을 검사한다.

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

기본 `--search-strategy staged`는 이 coarse score map에서 raw score와 인접 8개 후보의
가중치를 반영해 상위 3개 contiguous basin을 고른 뒤, basin별 5° → 1° local search를
수행한다. 서로 다른 yaw basin의 최종 seed를 최대 3개까지 Ceres `R,t` refinement하고,
training scene validation → Core gate → multi-criteria confidence → objective 순서로
선택한다. 모든 finalist가 실패하면 coarse 후보를 fallback으로 승격하지 않는다.
`--search-strategy legacy`는 비교용 진단 경로다.

Flat image–scan 디렉터리뿐 아니라 Jenkins 패키지들이 한 단계 아래에 있는 dataset
root도 입력할 수 있다. `--camera-channel N`으로 `_CHN` 영상만 고르고,
`*_pan_tilt_lidar.json`만 scan으로 사용하며 `manifest.json`은 제외한다. 중첩 입력은
같은 부모 패키지 안에서만 pairing하고 scan 파일명 기준으로 정렬한다.

```bash
build/bin/run_real_calibration \
  --input-dir /path/to/three-or-more-image-scan-pairs \
  --output automatic_calibration/generated/result \
  --camera-channel 3 \
  --ldc-enabled unknown \
  --zoom-focus-locked true \
  --manual-intrinsic-json manual_calibration/output/<session>/intrinsic/camera_intrinsic.json \
  --manual-reference-json manual_calibration/output/<session>/reference/T_camera_lidar.json \
  --image-distortion-state raw \
  --allow-intrinsic-refinement false \
  --camera-center-x-m 0.05928 \
  --camera-center-y-m -0.08105 \
  --camera-center-z-m 0 \
  --expected-forward-x 0 --expected-forward-y 0 --expected-forward-z -1 \
  --expected-down-x 0 --expected-down-y 1 --expected-down-z 0 \
  --direction-prior-weight 0 \
  --down-min-deg -30 --down-max-deg 30 --down-step-deg 5 \
  --optical-roll-min-deg -15 --optical-roll-max-deg 15 \
  --optical-roll-step-deg 5 \
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

구조선 목적함수는 승인 LiDAR 평면의 교차선, 승인 평면과 미분류 geometry의 경계선,
여러 scan에서 반복되는 폐색선을 사용한다. fitted 선분의 양 끝이 화면 밖이어도 화면 안에
보이는 구간을 샘플링하고, 2D LSD 선분과 방향·거리·유한 구간 겹침 비용으로 1:1 대응한다.
단일 scan의 range discontinuity는 폐색 진단선으로만 저장한다. 후보별 수평/수직 비용과
유효 선분 수는 `orientation_full_search.csv`에 따로 기록된다. 영상 소실점 후보와 LiDAR
중력축/벽축의 각도는 `debug/scene_*/03a_manhattan_vanishing_directions.csv`에서 확인한다.
후보가 일부 LiDAR edge만 맞춘 local minimum이 되지 않도록 같은 yaw layer의 최대
support 대비 edge/NID/영상 공간 coverage와 penalty를 함께 평가한다. 카메라는 360°
LiDAR의 한 sector만 보므로 edge coverage는 hard gate가 아닌 soft penalty로만 사용한다.
상대 NID·영상 공간 coverage는 기본 `0.5` hard gate, coverage penalty는 기본 `0.25`이며,
`orientation_full_search.csv`의 `edge_coverage_ratio`, `nid_coverage_ratio`,
`edge_spatial_coverage_ratio`, `coverage_objective`로 확인할 수 있다. 이전
`--minimum-relative-edge-coverage` 옵션은 제거됐다. 2026-08-20 CH1 실험의 전체 비교표는
[`COVERAGE_SUPPORT_EXPERIMENT_20260820.md`](docs/COVERAGE_SUPPORT_EXPERIMENT_20260820.md)에
기록했다.
제조사 FOV K 주변 `--focal-scale-min/max/step`(허용 0.8~1.2)은 연구·민감도 진단에서만
사용한다. 제품 승인 실행에서는 동일 채널·해상도·zoom/focus/LDC 상태의 Manual ChArUco
profile을 반드시 입력한다.

`run_real_calibration`은 `--manual-intrinsic-json`으로 같은 해상도·zoom/focus
프로파일에서 측정한 ChArUco `K + distortion`을 읽는다. 제품 경로에서는
`fx,fy,cx,cy`와 왜곡 계수를 항상 고정하고 `R,t`만 미세 조정한다.
`--allow-intrinsic-refinement true`는 연구·진단용으로만 허용하며 제품 승인 실행에는
사용하지 않는다. JSON에
왜곡 계수가 있고 `--image-distortion-state raw`이면 입력 image를 먼저
undistort하고, `rectified`이면 이미 LDC/외부 보정된 영상으로 간주해 중복 보정을
하지 않는다. `unknown`이면 raw/rectified 계약이 확정되지 않은 것이므로 보정하지 않고
결과를 `DIAGNOSTIC_ONLY`로 남긴다. 이 상태의 RT는 후보나 제품 활성값으로 승격하지 않는다.

Manual K+D를 고정한 제품 경로에서는 관측을 `training`과 `hold-out`으로 분리할 수
있다. 예를 들어 3쌍 중 2쌍으로 RT만 추정하고 `--holdout-count 1`로 마지막 1쌍을
추정 완료 후 검증한다. 이 경우 K+RT 공동 추정의 최소 3관측 규칙을 적용하지 않으며,
hold-out 장면은 후보 점수·Ceres 최적화에 들어가지 않는다. 결과는
`training_scene_validation.csv`, `holdout_scene_validation.csv`와
`calibration_result.json`의 `training_observation_count`/
`holdout_observation_count`로 확인한다. 2026-08-20 CH1의 실제 2-train/1-hold-out
실행은 [`COVERAGE_SUPPORT_EXPERIMENT_20260820.md`](docs/COVERAGE_SUPPORT_EXPERIMENT_20260820.md)
§8에 기록되어 있다.

staged 경로는 선택 RT뿐 아니라 최대 3개 separated Ceres finalist를 같은 hold-out에
고정 적용한다. training/core/absolute-support를 통과한 분리 후보를 pass-ratio tier로
나눈 뒤, 같은 tier에서는 학습과 같은 Edge·geometry NID·구조선·Manhattan·coverage
목적함수로 비교한다. 공통 hold-out coverage 기준에서 선택 후보의 목적함수 우위가 기존
2% margin보다 작으면 `FINALIST_HOLDOUT_AMBIGUOUS`와 `NOT_CANDIDATE_RT`를 반환한다.
후보별 결과는
`finalist_holdout_candidate_<index>.csv`와 JSON의 `finalist_holdout_validation`에서
확인한다. 거절 시 `estimated_t_camera_lidar`는 안전 prior이며 거절된 최적화 자세는
`diagnostic_candidate_t_camera_lidar`에만 남는다. 후보 통과 시에도 제품 활성화는
별도 `product_approved_rt_status`와 `activation_allowed`를 따라야 한다.

LDC는 렌즈 왜곡을 보정해 영상을 출력하는 기능이지 카메라 내부 파라미터를
없애는 기능이 아니다. LDC가 켜져도 rectified 영상의 픽셀을 LiDAR에 투영하려면
그 출력 프로파일의 K가 필요하다. 따라서 LDC를 사용할 수 없는 카메라는
manual `K + distortion`을 사용하거나 동일 K로 영상을 undistort해야 한다.
실수로 보정되지 않은 raw 영상을 사용하는 것을 막기 위해 `--ldc-enabled false`는
`--manual-intrinsic-json`, 왜곡 계수, `--image-distortion-state raw`가 모두 없으면
실행을 중단한다.

내부 `PASS`는 제품 승인을 뜻하지 않는다. `INTERNAL_GATE_PASS`와 `CANDIDATE_RT`는
독립 hold-out·반복성·기준 RT 검증 전까지 활성값으로 승격하지 않으며, 실패 시 기존
active RT를 유지한다. 상세 상태 구분과 실패 처리는
[`PRODUCT_CALIBRATION_POLICY.md`](docs/PRODUCT_CALIBRATION_POLICY.md)를 따른다.

이미 승인 후보로 얻은 RT가 다른 조명 조건에서도 유지되는지만 검사할 때는
`--validation-pose-json`을 사용한다. 이 모드는 입력 RT를 초기값으로 쓰는 것이 아니라
**완전히 고정**하며, 후보 탐색·Ceres 최적화·RT 갱신을 수행하지 않는다.

```bash
build/bin/run_real_calibration \
  --input-dir /path/to/image-scan-pairs \
  --output automatic_calibration/generated/condition_b_fixed_rt \
  --pair-start 5 --pair-count 5 \
  --camera-channel 1 \
  --ldc-enabled false --zoom-focus-locked true \
  --manual-intrinsic-json /path/to/camera_intrinsic.json \
  --image-distortion-state raw \
  --validation-pose-json /path/to/reference/calibration_result.json \
  --validation-label condition_b_using_reference_RT \
  --minimum-scene-pass-ratio 1.0 \
  --debug-output automatic_calibration/generated/condition_b_fixed_rt/debug
```

결과는 `fixed_pose_validation_result.json`과
`fixed_pose_scene_validation.csv`에 기록된다. `--debug-output`을 지정하면 고정 RT의
장면별 투영과 edge 잔차도 생성된다. 이 결과가 FAIL이어도 RT를 새 값으로 대체하지
않으며, holdout을 보고 gate 임계값을 완화해서는 안 된다.

계산 원본 frame의 `.ply/.obj`와 사람이 확인할 Z-up
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
- `03a_manhattan_vanishing_directions.csv`: 영상 소실점 후보와 초기·최종 중력축 오차
- `04_lidar_surface_normals.ply`: 거리 급변을 가로지르지 않고 계산한 surface normal
- `04a_lidar_plane_labels.ply`: 같은 평면으로 분할된 point를 동일 색으로 표시
- `04a_lidar_planes.csv`: 평면 normal·offset·지지점 수·RMS 오차
- `04b_lidar_plane_pair_candidates.csv`: 평면 쌍별 승인/탈락 단계와 근거
- `04b_lidar_plane_intersection_edges.{ply,obj}`: 인접한 두 평면의 실제 교차선
- `04b1_lidar_plane_boundary_edges.{ply,obj}`: 승인 평면과 미분류 geometry의 경계선
- `04c_lidar_occlusion_edges.{ply,obj}`: 거리 급변으로 생긴 장애물 실루엣(진단 전용)
- `04c1_lidar_persistent_occlusion_edges.{ply,obj}`: 여러 관측에서 반복된 폐색 구조선
- `04d_lidar_edges_used_for_calibration.{ply,obj}`: 목적함수에 실제 사용한 구조선
- `05_projection_initial.png`: 초기 RT prior를 적용한 LiDAR 투영
- `06_projection_final.png`: 최종 RT 또는 비활성 진단 후보 RT를 적용한 전체 LiDAR 투영
- `07_projection_final_edges.png`: 최종 RT를 적용한 LiDAR range edge 투영
- `07a_projection_final_edge_residual.png`: Canny 거리별 edge 잔차(초록 ≤10 px, 노랑 ≤30 px, 빨강 >30 px)
- `debug_summary.csv`: valid point, normal, plane, 구조선, 투영 개수와 edge mean/p50/p90

`07a`는 원인 확인용 full-resolution nearest-pixel 진단이다. 최종 품질 gate는 Core의
bilinear distance와 quarter-resolution z-buffer를 사용하므로 `07a`의 수치와 소수점까지
같지는 않다.

관측이 3개 미만이면 Manual ChArUco K+D profile을 고정하고 pose 탐색은 진단 목적으로 실행하지만,
결과는 항상 `SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL`이다. 이때 활성 RT는 prior를
유지하고 `diagnostic_candidate_t_camera_lidar`와 `top_candidates/rank_1.png`~
`rank_5.png`만 검토용으로 제공한다. 투영은 10 mm 허용 z-buffer로 가려진 뒤쪽 점을
제외한다. 투영점 개수는 화면 안에 보이는 점 수이며 매칭 정확도를 의미하지 않는다.

`mechanism.tilt_zero=nadir`는 모터 기구축 홈 메타데이터다. 좌표 계산에는
`measurements[].tilt_rad`를 그대로 사용하며, 130333 계약각은 0°=수평,
-90°=아래 방향이다. pan 360° sweep과 카메라 외부 yaw 후보는 동일한 변수가 아니다.
실장 확인 결과 pan 값 증가는 Top-view 기준 시계 방향이다.

고정환경 실측에서 지면 기준 센서 순서는 **LiDAR → 카메라**이고, 카메라는 LiDAR보다
81.05 mm 위에 있다. 모델링 도면에서 CH1 렌즈 중심은 LiDAR 회전 중심선의 오른쪽으로
59.28 mm 떨어져 있다. 따라서 LiDAR frame의 `+x=right`, `+y=down`을 적용한 현재
camera center prior는 다음과 같다.

```text
C_L = (+0.05928, -0.08105, 0) m
|C_L| = 0.100415 m
```

59.28 mm와 81.05 mm는 각각 수평·수직 성분이며 둘 중 하나를 전체 센서 간 거리로
사용하지 않는다. `z=0`은 두 중심의 전후 깊이가 같다는 현재 설치 가정이다. X 부호와
Z offset은 최종 조립 좌표계에서 다시 실측하면 해당 값을 우선한다.

다중 장면 경로는 카메라 gradient와 LiDAR range/surface-normal 특징의 NID를 70%,
edge 정합을 30%로 사용하는 복합 목적함수다. 초기 coarse 방향은 15° 간격으로 360°를
탐색하고 이후 staged local search로 세분화한다. 멀리 떨어진 두 방향의 점수가 비슷하거나 NID·중첩률·개선률이 부족하면
결과를 적용하지 않고 `MULTISTART_AMBIGUOUS`, `NID_OVERLAP_INSUFFICIENT` 등의
reason code와 기존 mechanical prior를 반환한다.

Reference RT의 signal-strength NMI 식별력을 별도로 확인하려면
`--manual-reference-json PATH --reference-rt-perturbation-only true`를 사용한다. 이
모드는 `reference_rt_perturbation.csv`를 생성하는 진단 전용 실행이며 RT 활성화와 NMI
가중치 변경을 수행하지 않는다. 결과 lifecycle 상태는 일반 `PASS`가 아니라
`INTERNAL_GATE_PASS`/`CANDIDATE_RT`/`PRODUCT_APPROVED_RT`로 분리해 해석한다.

## 결과 시각화

```bash
build/bin/render_calibration_visualization \
  --dataset-root /path/to/area_1 \
  --result-json automatic_calibration/generated/calibration_core_multi_validation/calibration_result.json \
  --output automatic_calibration/generated/calibration_core_multi_validation/visualization
```

상세 문서:

- [`docs/OPENSDK_RT_INTEGRATION_HANDOFF.md`](docs/OPENSDK_RT_INTEGRATION_HANDOFF.md) — OpenSDK/CV5 영상·LiDAR 파이프라인, RT 계약, Core 실행 및 안전한 결과 적용 인계서
- [`docs/CURRENT_PROGRESS_AND_STATUS.md`](docs/CURRENT_PROGRESS_AND_STATUS.md)
- [`docs/CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md`](docs/CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md)
- [`docs/CH1_ARUCO_VALIDATION_20260818.md`](docs/CH1_ARUCO_VALIDATION_20260818.md)
- [`docs/REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md`](docs/REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md)
- [`docs/REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md`](docs/REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md)
- [`docs/CALIBRATION_CORE_ARCHITECTURE.md`](docs/CALIBRATION_CORE_ARCHITECTURE.md)
- [`docs/SYNTHETIC_DATA_ARCHITECTURE.md`](docs/SYNTHETIC_DATA_ARCHITECTURE.md)
- [`docs/PAN_TILT_LIDAR_JSON_INTERFACE.md`](docs/PAN_TILT_LIDAR_JSON_INTERFACE.md)
- [`docs/MANUAL_REFERENCE_PRIOR_WORKFLOW.md`](docs/MANUAL_REFERENCE_PRIOR_WORKFLOW.md)
- [`docs/JENKINS_CONFORMANCE_TEST_PLAN.md`](docs/JENKINS_CONFORMANCE_TEST_PLAN.md)
- [`docs/TARGETLESS_CALIBRATION_METHOD_REVIEW.md`](docs/TARGETLESS_CALIBRATION_METHOD_REVIEW.md)

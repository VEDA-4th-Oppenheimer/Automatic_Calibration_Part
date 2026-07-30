# `develop` 코드 분석·리뷰 및 성능 시험 보고서

- 분석/측정일: 2026-07-28 (Asia/Seoul)
- 대상: `develop/`의 C++17 합성 LiDAR 생성기와 Calibration Core
- 빌드 유형: `RelWithDebInfo`
- 실행 환경: Docker on WSL2, Linux 6.18.33.2, x86_64, 8 vCPU, 15.52 GiB RAM
- 도구 버전: CMake 4.2.3, GCC 15.2.0, Ceres 2.2.0

## 1. 결론 요약

이 프로젝트는 Stanford 2D-3D-Semantics의 RGB/depth를 가상 pan-tilt LiDAR 입력으로
변환하고, 카메라 edge와 LiDAR range discontinuity를 맞추어 카메라-LiDAR 6-DoF
extrinsic을 추정하는 Calibration Core MVP다.

구조는 공개 API(`include`), 구현(`src`), 실행 프로그램(`apps`), 테스트(`tests`)로
명확히 나뉘어 있다. 실패한 최적화 후보 대신 입력 mechanical prior를 반환하는 fail-safe
계약도 구현과 테스트가 일치한다. 2026-07-28 실측에서 모든 단위 테스트가 통과했고,
5장면 기본 conformance도 통과했다.

다만 production 적용 전에 다음 항목은 우선 수정해야 한다.

1. `projected_ratio`가 실제 영상 내부 투영 여부가 아니라 residual `< 30` 여부를 세므로,
   영상 안에 투영됐지만 edge에서 30px 이상 떨어진 점을 비투영점으로 오분류한다.
2. 단일 장면에서만 `coarse_rounds`가 적용되고 다중 장면 API에서는 무시된다.
3. 음수 noise, 비정상 Canny/solver/prior 설정, 매우 큰 unsigned CLI 값 등 설정 검증이
   부족하여 잘못된 입력이 과도한 할당, 정의하기 어려운 최적화, 부정확한 metadata로
   이어질 수 있다.
4. `ubuntu:latest`는 빌드 재현성을 보장하지 못한다.
5. 테스트는 정상 경로와 일부 fail-safe만 다루며 I/O 실패, 잘못된 설정, 경계 투영,
   재현성, 다장면별 실패 원인 등을 충분히 회귀 검증하지 않는다.

## 2. 디렉터리와 파일 구조

```text
develop/
├── CMakeLists.txt                         빌드 타깃과 의존성
├── Dockerfile                            Ubuntu native 개발 이미지
├── compose.yaml                          소스·데이터셋·빌드 볼륨 연결
├── README.md                             개발/빌드/실행 시작점
├── include/auto_calib/
│   ├── synthetic_lidar.hpp               합성 scan 데이터 모델과 공개 API
│   └── calibration_core.hpp              보정 설정·결과·공개 API
├── src/
│   ├── synthetic_lidar.cpp               Stanford 로더, depth 투영, scan/패키지 생성
│   └── calibration_core.cpp              edge 추출, Ceres 최적화, 품질 게이트
├── apps/
│   ├── generate_synthetic_scan.cpp       합성 scan 패키지 생성 CLI
│   ├── run_synthetic_calibration.cpp     단일 장면 conformance CLI
│   └── run_multi_synthetic_calibration.cpp
│                                             다중 장면 공동 최적화 CLI
├── tests/
│   ├── synthetic_lidar_tests.cpp         depth/transform/topology 단위 테스트
│   └── calibration_core_tests.cpp        edge/최적화/fail-safe 단위 테스트
├── docs/
│   ├── SYNTHETIC_DATA_ARCHITECTURE.md
│   ├── CALIBRATION_CORE_ARCHITECTURE.md
│   └── JENKINS_CONFORMANCE_TEST_PLAN.md
└── generated/                            기존 실행 결과
```

`CMakeLists.txt`는 `src/synthetic_lidar.cpp`와 `src/calibration_core.cpp`를
`synthetic_lidar` 정적 라이브러리 하나로 묶는다. 세 CLI와 두 테스트는 이 라이브러리를
링크한다. 라이브러리 이름은 현재 역할보다 좁아서, 이후에는 `auto_calib_core`처럼
보정 기능까지 드러내는 이름이 더 정확하다.

주요 외부 의존성은 다음과 같다.

| 의존성 | 사용 위치 | 목적 |
|---|---|---|
| Eigen3 | 데이터 모델, 두 구현부 | 3D 벡터, 회전 행렬, angle-axis |
| OpenCV | 두 구현부 | PNG/EXR I/O, grayscale, blur, Canny, distance transform |
| Ceres | `calibration_core.cpp` | 6변수 비선형 최소제곱 |
| nlohmann-json | 로더, writer, CLI | pose 입력과 결과 JSON |
| yaml-cpp | `synthetic_lidar.cpp` | manifest 출력 |

## 3. 데이터 계약과 좌표계

핵심 변환은 다음과 같다.

```text
p_camera = R_camera_lidar × p_lidar + t_camera_lidar
```

- 카메라 optical frame: `+x` 오른쪽, `+y` 아래, `+z` 전방
- 거리: meter
- 내부 각도: radian
- 보고용 회전 오차: degree
- pan: `atan2(x, z)`, 오른쪽이 양수
- tilt: `atan2(-y, hypot(x, z))`, 위쪽이 양수

`Transform::lidarToCamera()`는 위 식을 그대로 계산하고,
`Transform::cameraToLidar()`는 회전 행렬의 직교성을 이용해
`Rᵀ(p_camera - t)`로 역변환한다.

Stanford depth는 radial range가 아닌 z-depth다.

```text
z = raw / 512
x = (u - cx) × z / fx
y = (v - cy) × z / fy
```

`raw == 0` 또는 `65535`는 결측으로 제외한다.

## 4. 전체 실행 흐름

```text
Stanford RGB/depth/pose
        │
        ├─ loadStanfordFrame(): RGB, 16-bit depth, K 로드
        │
        ├─ projectDepth(): depth pixel → camera 3D points
        │
        ├─ generateScan(): camera → LiDAR 변환, angular raster,
        │                  nearest return, noise/dropout
        │
        ├─ writePackage(): PCD, EXR, mask, RGB, GT, QA, manifest
        │
        └─ Calibration Core
             ├─ RGB → Gaussian blur → Canny → edge distance transform
             ├─ organized scan → 인접 range 불연속 → 3D edge points
             ├─ mechanical prior + 6-DoF Ceres 최적화
             └─ overlap/alignment/update/convergence 품질 게이트
                    ├─ PASS: 후보 extrinsic 반환
                    └─ FAIL: 입력 mechanical prior 반환
```

## 5. 파일·함수별 상세 분석

### 5.1 `include/auto_calib/synthetic_lidar.hpp`

#### 상수와 데이터 구조

- `kValidRange`: 점에 유효 range가 있음을 나타내는 bit 0.
- `kSyntheticMeasurement`: 합성 데이터임을 나타내는 bit 16.
- `CameraModel`: 3×3 intrinsic `k`, 영상 폭과 높이.
- `Transform`: `R_camera_lidar`와 `t_camera_lidar`.
- `Frame`: Stanford frame ID, 원본 경로, 카메라 모델, RGB/depth `cv::Mat`.
- `ScanConfig`: angular 범위, 격자 크기, depth sampling stride, 거리 범위,
  Gaussian noise, dropout, seed.
- `Point`: xyz/range/precision/pan/tilt/timestamp/grid index/flags.
- `Scan`: 설정, organized points, 입력 3D 점 수와 최종 유효 점 수.

`Point::valid()`는 `flags & kValidRange`만 확인한다. xyz/range가 finite인지까지 보장하지
않으므로 외부 producer가 `kValidRange`를 잘못 설정하면 후단으로 NaN이 전달될 수 있다.

### 5.2 `src/synthetic_lidar.cpp`

#### `angleAt(i, lo, hi, n)` / `indexAt(a, lo, hi, n)`

angular grid index와 각도를 상호 변환한다. `angleAt()`은 축의 cell이 하나면 범위 중앙을
반환한다. `indexAt()`은 실수 각도를 가장 가까운 cell로 반올림하고 `[0, n-1]`로 clamp한다.
`generateScan()`이 rows/columns를 0보다 크게 검증한 뒤 호출하므로 `n == 0`은 정상 경로에
들어오지 않는다.

#### `ray(pan, tilt)`

pan/tilt를 LiDAR frame 단위 광선으로 변환한다.

```text
x = cos(tilt) sin(pan)
y = -sin(tilt)
z = cos(tilt) cos(pan)
```

noise가 range에 적용된 뒤 xyz를 다시 광선 위에 배치할 때 사용한다.

#### `makeTransform(translation, rpy)`

roll, pitch, yaw의 회전을 각각 X/Y/Z `AngleAxis`로 만들고 `Rz × Ry × Rx` 순서로
합성한다. 즉 고정축 관점의 roll-pitch-yaw 계약이다. actuator/다른 SDK와 연결할 때
회전 순서를 반드시 맞춰야 한다.

#### `loadStanfordFrame(root, id)`

1. `root/data/rgb` 존재 여부를 확인한다.
2. `_domain_rgb.png`가 들어간 파일을 정렬한다.
3. ID가 없으면 정렬상 첫 frame, 있으면 해당 frame을 선택한다.
4. 같은 ID로 depth PNG와 pose JSON 경로를 구성한다.
5. RGB와 `CV_16UC1` depth를 읽고 크기 일치를 검증한다.
6. pose의 `camera_k_matrix`를 `Eigen::Matrix3d`로 복사한다.

함수는 누락 파일, 잘못된 depth type, JSON key/type 오류를 예외로 보고한다. RGB 후보
필터가 suffix로 끝나는지를 보지 않고 문자열 포함 여부만 보므로, 비표준 백업 파일도
후보가 될 수 있다.

#### `projectDepth(depth, camera, stride)`

stride 간격으로 depth를 순회하고 유효 pixel을 카메라 3D 점으로 역투영한다.
시간 복잡도와 출력 메모리는 대략 `O(width × height / stride²)`다.

검증되는 것은 depth type, 양의 stride, 양의 `fx/fy`다. `cx/cy`, camera width/height,
finite intrinsic은 검증하지 않는다.

#### `generateScan(source, t_camera_lidar, config)`

1. shape, angular 범위, range, dropout을 검증한다.
2. `rows × columns`의 organized point 배열을 만들고 각 cell의 metadata를 채운다.
3. 각 camera point를 `cameraToLidar()`로 변환한다.
4. range, `z > 0`, pan/tilt FOV를 검사한다.
5. angular cell로 양자화하고 같은 cell에서는 가장 가까운 point만 유지한다.
6. seeded RNG로 dropout과 Gaussian range noise를 적용한다.
7. 최종 range와 cell의 광선으로 xyz를 다시 계산한다.

대략적인 시간 복잡도는 `O(source point 수 + rows × columns)`, scan 저장 공간은
`O(rows × columns)`이다.

현재 `noise_stddev >= 0` 검증이 없다. 음수면 실제 noise는 적용되지 않지만
`Point::precision`에는 음수가 기록된다. 이는 계산과 metadata가 불일치하는 입력 오류다.

#### `writePackage(output, frame, scan, ground_truth)`

다음 파일을 기록한다.

```text
cloud/organized_cloud.pcd
cloud/range_image.exr
cloud/validity_mask.png
camera/rgb.png
calibration/ground_truth_extrinsic.json
qa/pointcloud_quality.json
manifest.yaml
```

PCD는 ASCII이므로 사람이 확인하기 쉽지만 파일 크기와 기록 시간이 binary PCD보다 크다.
OpenCV 파일 기록 실패는 확인하지만 ground-truth/QA/manifest `ofstream` 상태는 확인하지
않는다. 디스크 부족이나 권한 문제에서 함수가 성공한 것처럼 끝날 수 있다.

### 5.3 `include/auto_calib/calibration_core.hpp`

#### `CalibrationConfig`

- LiDAR edge: 절대/상대 range 차이 threshold와 최소 점 수
- 카메라 edge: Canny threshold와 최소 pixel 수
- 품질 gate: 최소 projected ratio, 최대 평균 edge 거리
- 최적화: residual cap, coarse search, iteration, 탐색 bound
- prior: 회전/평행이동 sigma와 weight
- fail-safe: prior 대비 최대 허용 update

단위가 이름에 포함되어 있어 API 자체 설명력은 좋은 편이다. 그러나 생성 시 전체 설정을
검증하는 함수가 없고 각 사용처에서 일부만 검사한다.

#### `CalibrationMetrics` / `CalibrationResult`

입력 feature 수, projected ratio, 전후 평균 residual, solver iteration과 Core runtime을
기록한다. 실패 시에도 `estimated_t_camera_lidar`는 입력 prior로 초기화되므로 호출자가
실패 후보를 실수로 적용할 위험을 줄인다.

### 5.4 `src/calibration_core.cpp`

#### `toParameters()` / `fromParameters()`

`Transform`과 Ceres의 6변수 배열을 변환한다.

```text
[angle_axis_x, angle_axis_y, angle_axis_z, tx, ty, tz]
```

#### `bilinear(image, u, v)`

float distance transform을 bilinear interpolation한다. 호출 전
`u < cols - 1`, `v < rows - 1`가 확인되므로 네 이웃 접근은 정상 범위다.

#### `residualForPoint(parameters, point, camera, distance)`

1. Ceres angle-axis로 LiDAR 점을 회전한다.
2. translation을 더해 camera 좌표로 만든다.
3. `z <= 0.05m`이면 residual 50을 반환한다.
4. intrinsic으로 `(u, v)`에 투영한다.
5. 영상 밖이면 `30 + 경계까지 거리`를 반환한다.
6. 영상 안이면 distance transform을 bilinear sampling한다.

이 함수가 edge alignment 목적함수와 최종 지표 계산의 중심이다.

#### `EdgeCost`

모든 LiDAR edge point residual을 계산하고 `residual_cap_px`로 상한을 둔다.
전체 edge 수 제곱근의 역수로 scale하여 장면/점 수가 늘어도 prior와 데이터 항의 상대
영향이 과도하게 변하지 않도록 한다.

`DynamicNumericDiffCostFunction<CENTRAL>`은 6개 변수의 수치 미분을 위해 residual
함수를 반복 평가한다. 구현은 단순하지만 edge 수와 iteration 수가 커지면 계산량이
선형 이상으로 체감 증가한다. 이 부분은 analytic/autodiff 가능한 영상 gradient 기반
cost로 바꾸면 최적화 여지가 크다.

#### `PriorCost`

후보와 mechanical prior의 차이를 회전 sigma와 translation sigma로 각각 정규화한다.
`prior_weight`가 0이면 사실상 prior residual은 사라지지만 search bound는 계속 적용된다.
sigma가 0 또는 음수인 설정은 검증되지 않아 0으로 나누거나 의미가 역전될 수 있다.

#### `evaluate()`

모든 점의 평균 uncapped residual과 projected 수를 계산한다. 현재 projected 판정이
`residual < 30`이다. 그러나 영상 내부 점도 edge에서 30px 이상 떨어질 수 있으므로
projected/overlap 의미와 일치하지 않는다. 투영 가능 여부와 edge 거리를 별도로 반환해야
한다.

#### `coarseSearch()`

각 round마다 회전 3축과 이동 3축을 순서대로 ±step 탐색하는 coordinate descent다.
prior bound 밖 후보는 제외하고 round마다 step을 절반으로 줄인다.

이 함수는 단일 장면 `calibrateExtrinsic()`에서만 호출된다. 다중 장면 함수는 같은
`CalibrationConfig.coarse_rounds`를 받아도 coarse search를 하지 않는다.

#### `buildCameraEdgeDistanceTransform()`

BGR 또는 grayscale 입력을 받아 5×5 Gaussian blur, Canny, inverse, precise L2 distance
transform 순서로 처리한다. 다른 channel 수는 거부한다.

#### `extractLidarEdgePoints()`

organized shape를 확인한 뒤 모든 수평/수직 이웃을 비교한다.

```text
threshold = max(absolute_threshold,
                relative_threshold × min(range_a, range_b))
```

range 차이가 threshold보다 크면 양쪽 점을 edge로 선택한다. `vector<bool>`로 중복을
제거한 뒤 xyz를 double로 반환한다. 시간과 임시 메모리는 `O(rows × columns)`이다.

#### `calibrateExtrinsic()`

1. 결과의 estimated transform을 prior로 초기화한다.
2. intrinsic, camera edge 수, LiDAR edge 수를 gate한다.
3. prior의 초기 평균 residual을 계산한다.
4. 선택적으로 coarse search한다.
5. Ceres problem에 edge residual과 prior residual을 추가한다.
6. 각 6변수에 prior 중심 lower/upper bound를 설정한다.
7. `DENSE_QR`, Ceres `num_threads=1`로 최적화한다.
8. projected ratio, 최종 residual, iteration, solver summary를 기록한다.
9. optimizer/overlap/alignment/objective/update gate를 순서대로 적용한다.
10. 모두 통과한 경우에만 candidate를 결과에 저장한다.

Core `runtime_ms`는 feature 생성부터 quality gate까지 포함하지만, 원본 파일 로딩,
depth projection, synthetic scan 생성, package 기록은 포함하지 않는다.

#### `calibrateExtrinsicMultiScene()`

각 장면의 edge distance와 LiDAR edge를 준비하고 하나의 6-DoF parameter block에
장면별 residual block을 연결한다. 평균 residual과 projected ratio는 전체 edge 수로
가중 집계한다. 한 장면이라도 입력 gate에 실패하면 전체가 실패한다.

다중 장면 API 자체는 장면 수가 1이어도 실행된다. 최소 2장면 규칙은 CLI에서만 검사한다.
공개 API 계약상 정말 다장면만 허용할 것이라면 Core에서도 검증해야 한다.

#### `calculatePoseError()`

translation은 두 벡터의 L2 거리, rotation은
`R_estimated × R_ground_truthᵀ`의 angle-axis 각도로 계산한다.

### 5.5 실행 프로그램

#### `generate_synthetic_scan`

필수 인자는 `--dataset-root`, `--output`이다. frame, scan shape/FOV, ground truth,
noise/dropout/seed를 받고 `load → project → generate → write`를 실행한다.

#### `run_synthetic_calibration`

한 frame에서 합성 scan을 만들고 알려진 ground truth에 delta를 더한 initial prior로
단일 장면 보정을 실행한다. Core gate와 별도로 ground-truth 회전/translation 허용오차를
검사한다. Core가 성공해도 정확도 tolerance를 넘으면 exit code 3과
`GROUND_TRUTH_TOLERANCE_EXCEEDED`를 반환한다.

#### `run_multi_synthetic_calibration`

쉼표로 구분된 두 개 이상의 frame ID를 로드하고 동일 ground truth로 각 scan을 생성한 뒤
하나의 extrinsic을 공동 최적화한다. 각 장면 패키지도 출력하므로 Core 시간보다 end-to-end
시간과 디스크 사용량이 훨씬 크다.

세 CLI의 parser는 단순하고 중복되어 있다. 알 수 없는 option도 값만 있으면 허용하고,
음수 문자열을 `stoul()`로 읽은 값이나 지나치게 큰 rows/columns를 상한 검증하지 않는다.
공통 parser/validator로 합치는 것이 안전하다.

### 5.6 테스트

`synthetic_lidar_tests`가 검증하는 항목:

- raw depth 1024가 2m로 변환되는지
- 결측 depth가 제외되는지
- transform 왕복 오차
- organized grid 크기와 중앙 cell
- 동일 ray의 nearest return

`calibration_core_tests`가 검증하는 항목:

- 인공 range discontinuity의 LiDAR edge 추출
- 단일/다중 최적화 성공
- 목적함수 비악화
- 과도 update 거절과 prior fallback
- blank camera 거절
- 실패 경로 runtime 기록
- 동일 pose 오차 0

현재 테스트 executable은 assertion framework 없이 예외와 단일 `main()`을 사용한다.
실패 위치와 여러 케이스의 독립 실행성이 제한된다. Dockerfile에 이미 GTest가 있으므로
각 계약을 독립 `TEST`로 옮기면 회귀 진단이 쉬워진다.

## 6. 코드 리뷰 결과

### 우선순위 High

#### H1. projected ratio 의미 오류

- 위치: `src/calibration_core.cpp:42-60`, `90-104`
- 현상: `evaluate()`가 `r < 30`을 영상 내부 투영으로 간주한다.
- 문제: 영상 내부에서 edge distance가 30px 이상이면 비투영으로 집계된다. 반대로
  projected ratio가 overlap과 alignment 품질을 혼합한 값이 된다.
- 영향: `OVERLAP_INSUFFICIENT`가 실제 FOV overlap 부족이 아니라 edge alignment 불량으로
  발생할 수 있고, metric 이름과 문서 의미가 달라진다.
- 권고: residual 함수가 `{distance, in_image}`를 반환하거나 별도의
  `projectPoint()`를 두어 기하학적 투영 여부와 edge residual을 분리한다.
- 추가 테스트: 영상 내부이지만 edge에서 30px 이상 떨어진 점, 정확히 경계에 있는 점,
  `z <= 0.05`인 점을 각각 검증한다.

#### H2. 설정값 검증 부족

- 위치: `src/synthetic_lidar.cpp:107-166`,
  `src/calibration_core.cpp:202-290`, `296-426`, 세 CLI parser
- 누락 예: 음수 `noise_stddev`, 0/음수 prior sigma, 음수 residual cap, 역전된 Canny
  threshold, 음수 iteration/coarse rounds, 0/음수 허용 update, 비정상 intrinsic,
  과도한 rows/columns/stride.
- 영향: 잘못된 metadata, division by zero/NaN cost, 과도한 메모리 할당 또는 비정상
  solver 결과가 가능하다.
- 권고: `validateScanConfig()`, `validateCalibrationConfig()`,
  `validateCameraModel()`을 공개 또는 내부 공통 함수로 만들고 Core 진입 직후 한 번에
  검증한다. CLI에는 shape와 파일 출력 크기 상한도 둔다.

### 우선순위 Medium

#### M1. 다중 장면에서 `coarse_rounds` 무시

- 위치: 단일 호출 `src/calibration_core.cpp:233`; 다중 함수 `296-426`
- 현상: 동일 config 필드가 API에 따라 다르게 작동한다.
- 영향: 사용자가 다중 장면에도 coarse initialization이 적용된다고 오해할 수 있다.
- 권고: `coarseSearch`를 aggregate evaluator를 받을 수 있게 일반화해 다중 장면에도
  적용하거나, 다중 API에서 미지원임을 타입/문서로 명확히 한다.

#### M2. 일부 출력 stream 실패 미검사

- 위치: `src/synthetic_lidar.cpp:215-247`, 단일/다중 CLI 결과 JSON
- 현상: PCD와 OpenCV 출력은 검사하지만 JSON/YAML 결과 stream은 검사하지 않는다.
- 영향: 디스크 부족/권한 문제에서 불완전 패키지를 정상으로 보고할 수 있다.
- 권고: 모든 stream의 open/write/close 상태를 확인하고 가능하면 임시 파일 작성 후
  rename하는 atomic package commit을 사용한다.

#### M3. 재현 불가능한 Docker base

- 위치: `Dockerfile:1`
- 현상: `FROM ubuntu:latest`.
- 영향: 동일 commit도 날짜에 따라 compiler와 dependency가 바뀌고 빌드/수치 결과가
  달라질 수 있다.
- 권고: Ubuntu release와 가능하면 digest를 고정하고 CMake/Ceres/OpenCV 버전을
  CI metadata에 기록한다.

#### M4. Core와 I/O가 한 라이브러리 타깃에 결합

- 위치: `CMakeLists.txt`
- 현상: `synthetic_lidar`가 synthetic I/O와 calibration core를 함께 포함하고
  yaml/json/imgcodecs까지 PUBLIC link한다.
- 영향: 실제 actuator adapter나 embedded 이식 시 불필요한 의존성이 전파된다.
- 권고: `auto_calib_types`, `synthetic_stanford_adapter`,
  `auto_calib_calibration_core`, `package_io` 정도로 타깃을 분리하고 PUBLIC/PRIVATE
  링크 범위를 줄인다.

### 우선순위 Low

#### L1. runtime을 중복 기록

- 위치: `src/calibration_core.cpp:290-294`, `422-426`
- 현상: 함수 끝에서 runtime을 직접 기록하고 즉시 `finish()`에서 다시 기록한다.
- 영향: 기능 오류는 없지만 코드가 중복되고 실제 반환값은 두 번째 기록이다.
- 권고: 모든 return을 `finish()`로 통일한다.

#### L2. RGB 후보 suffix 검사

- 위치: `src/synthetic_lidar.cpp:51-54`
- 현상: suffix로 끝나는지가 아니라 문자열 포함 여부를 검사한다.
- 권고: C++20이면 `ends_with`, C++17이면 길이와 `compare()`로 정확히 검사한다.

#### L3. CLI 구현 중복

세 앱에 argument parsing, degree/radian, transform JSON 코드가 반복된다.
공통 CLI utility로 옮기면 검증과 help 문구 불일치를 줄일 수 있다.

## 7. 성능 시험

### 7.1 측정 방법

- `RelWithDebInfo`, Ninja build
- 동일 Docker 컨테이너와 Stanford `area_1`
- 기본값: 321×121 scan, depth stride 2, noise 0.005m, dropout 0.01
- 생성/단일/5장면 실행을 각각 3회
- wall time: `time.perf_counter()`
- user/system CPU와 최대 RSS: Python `resource.getrusage(RUSAGE_CHILDREN)`
- stdout은 측정 중 `/dev/null`로 보내 콘솔 출력 영향을 줄임
- 출력은 컨테이너 `/tmp/auto_calib_perf_*` 사용

`ru_maxrss`는 Linux에서 KiB 단위 high-water mark다. CPU 사용률은
`(user + system) / wall × 100`으로 계산했다. OpenCV 내부 병렬 처리 때문에 Ceres의
`num_threads=1` 설정과 별개로 100%를 넘을 수 있다.

### 7.2 환경

| 항목 | 값 |
|---|---|
| Kernel | Linux 6.18.33.2-microsoft-standard-WSL2 |
| CPU 할당 | 8 logical CPUs |
| 메모리 | 16,273,012 KiB (약 15.52 GiB) |
| Compiler | GCC 15.2.0 |
| CMake | 4.2.3 |
| Ceres | 2.2.0 |
| Stanford RGB 파일 수 | 10,327 |

### 7.3 단위 테스트

| 항목 | 결과 |
|---|---:|
| 테스트 | 2/2 PASS |
| CTest 보고 시간 | 1.04s |
| wrapper wall time | 1.088s |
| user CPU | 1.395s |
| system CPU | 0.272s |
| 평균 CPU 환산 | 약 153% |
| 최대 RSS | 70,312 KiB (약 68.7 MiB) |

개별 CTest 결과는 synthetic LiDAR 0.84s, Calibration Core 0.20s다.

### 7.4 end-to-end 반복 측정

| 시나리오 | 1회 | 2회 | 3회 | 평균 wall | CPU 범위 | 최대 RSS |
|---|---:|---:|---:|---:|---:|---:|
| 합성 package 생성 | 0.460s | 0.448s | 0.427s | 0.445s | 약 225~240% | 85,592 KiB (83.6 MiB) |
| 단일 장면 calibration | 0.550s | 0.490s | 0.440s | 0.493s | 약 211~258% | 101,940 KiB (99.6 MiB) |
| 5장면 calibration | 2.180s | 1.891s | 1.841s | 1.971s | 약 124~130% | 140,196 KiB (136.9 MiB) |

첫 실행이 가장 느린 것은 page cache와 라이브러리/파일 캐시 warm-up 영향으로 해석된다.
따라서 운영 예산에는 평균뿐 아니라 cold-run 상단값도 사용해야 한다.

### 7.5 Core 내부 지표와 정확도

마지막 반복 결과:

| 항목 | 단일 장면 | 5장면 |
|---|---:|---:|
| Core runtime | 42.26ms | 499.97ms |
| camera edge pixels | 45,398 | 109,098 |
| LiDAR edge points | 1,501 | 5,871 |
| solver iterations | 16 | 63 |
| projected ratio | 0.8488 | 0.6866 |
| 초기 평균 edge 거리 | 19.95px | 34.46px |
| 최종 평균 edge 거리 | 14.22px | 32.13px |
| 최종 회전 오차 | 2.306° | 0.789° |
| 최종 translation 오차 | 0.06166m | 0.03930m |
| Core status | PASS | PASS |
| 최종 conformance | FAIL | PASS |

단일 장면 exit code 3은 실행 오류가 아니다. Core 품질 gate는 통과했지만 translation
ground-truth tolerance 0.05m를 초과한 의도된 conformance 실패다.

### 7.6 Core와 end-to-end 시간 차이

- 단일: Core 약 42ms, end-to-end 평균 약 493ms
- 5장면: Core 약 500ms, end-to-end 평균 약 1,971ms

Core 밖의 비용은 데이터 파일 로딩, depth 역투영, 가상 scan 생성, ASCII PCD와
EXR/PNG/JSON/YAML 출력이다. 실장 장비에서 이미 scan과 RGB가 메모리에 있다면 Core API
지연은 end-to-end CLI보다 훨씬 작다. 반대로 현재 CLI를 그대로 batch 생성에 쓰면 I/O와
합성 producer 비용을 포함해 예산을 잡아야 한다.

### 7.7 디스크 사용량

| 출력 | 크기 |
|---|---:|
| 합성 package 1개 | 4.9 MiB |
| 단일 calibration 출력 | 4.9 MiB |
| 5장면 calibration 출력 | 22 MiB |

ASCII PCD가 주된 확장 요인이다. 장기 conformance 결과를 보존할 때는 원본 package와
요약 JSON의 retention 정책을 분리하는 것이 좋다.

## 8. 성능 복잡도와 병목 예상

| 단계 | 시간 복잡도 개요 | 주요 메모리 |
|---|---|---|
| depth projection | `O(W×H/stride²)` | camera point vector |
| virtual scan | `O(source + rows×cols)` | organized point vector |
| package write | `O(rows×cols + image pixels)` | images + ASCII serialization |
| camera edge DT | `O(image pixels)` | gray/blur/edge/inverse/distance Mats |
| LiDAR edge | `O(rows×cols)` | selected bitset + edge vector |
| numeric-diff Ceres | 대략 `O(iterations × variables × edges)` | residual/Jacobian/problem |

가장 유력한 Core 병목은 `DynamicNumericDiffCostFunction<CENTRAL>`이다. 각 iteration에서
6변수 중앙 차분을 위해 전체 edge residual을 여러 번 다시 계산한다. 5장면 결과가
63 iterations/5,871 edges에서 약 500ms였으므로, 장면 수와 scan 해상도를 크게 늘릴 때
선형적인 메모리 증가와 더 큰 solver 시간 증가를 예상해야 한다.

## 9. 권장 후속 작업

### 즉시

1. projected 여부와 edge residual을 분리하고 경계 테스트를 추가한다.
2. 모든 config를 Core 진입점에서 검증한다.
3. 음수/과대 CLI 값을 거절하고 unknown option을 오류로 처리한다.
4. 다중 장면의 coarse search 지원 여부를 구현 또는 명문화한다.
5. JSON/YAML writer 오류를 확인한다.

### 다음 단계

1. Docker base와 주요 의존성 버전을 고정한다.
2. 테스트를 GTest 케이스로 분리하고 sanitizer 빌드를 CI에 추가한다.
3. Core/Stanford adapter/package I/O를 별도 CMake target으로 분리한다.
4. 1/2/5/10장면, scan 해상도, stride, edge 수에 따른 benchmark matrix를 CI에서
   수집한다.
5. numeric differentiation을 영상 gradient 기반 analytic/autodiff cost와 비교한다.
6. 실제 actuator 데이터에서는 동기화, distortion, encoder backlash, range bias,
   rolling shutter, scene observability를 별도 검증한다.

## 10. 재현 명령

`develop/`에서 실행한다.

```bash
docker compose exec -T dev cmake \
  -S /workspace -B /workspace-build -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
docker compose exec -T dev cmake --build /workspace-build --parallel 2
docker compose exec -T dev ctest \
  --test-dir /workspace-build --output-on-failure
```

단일 장면:

```bash
docker compose exec -T dev /workspace-build/run_synthetic_calibration \
  --dataset-root /datasets/stanford2d3ds/area_1 \
  --output /tmp/auto_calib_single
```

5장면:

```bash
docker compose exec -T dev /workspace-build/run_multi_synthetic_calibration \
  --dataset-root /datasets/stanford2d3ds/area_1 \
  --output /tmp/auto_calib_multi \
  --frame-ids \
camera_0004591bfdc749a88db196a5d8b345cb_office_6_frame_0,camera_00d10d86db1e435081a837ced388375f_office_24_frame_0,camera_03eb3fa2e1524ee887ba22d1a4896f3c_WC_1_frame_0,camera_042a479869b44a7c9159922f19a285ea_conferenceRoom_1_frame_0,camera_042fab82b3a94af9bea3c80984bc2583_hallway_2_frame_0
```

## 11. 최종 판정

현재 코드는 합성 입력 기반 Calibration Core MVP와 회귀/conformance 개발 용도로는
사용 가능하다. 기본 테스트와 5장면 합성 검증은 통과하며, 현 개발 환경에서 5장면
end-to-end 실행은 약 2초, Core 계산은 약 0.5초, 최대 RSS는 약 137MiB다.

그러나 projected ratio 정의 오류와 설정 검증 부족을 해결하기 전에는 metric 기반 자동
판정과 외부 입력에 대한 production 안전성을 보장하기 어렵다. 실제 장비 보정값을 저장
또는 actuator/후단 perception에 적용하려면 본 문서의 High 항목, 실제 센서 시간 동기화,
관측성/반복성 시험을 완료해야 한다.

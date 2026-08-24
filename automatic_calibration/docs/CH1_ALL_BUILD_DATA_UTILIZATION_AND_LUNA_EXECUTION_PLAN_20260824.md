# CH1 전체 수집 데이터 활용 및 Luna 실행 계획

- 작성일: 2026-08-24 (KST)
- 문서 상태: 실행 전 계획 확정, CLI preflight 검증 완료
- 데이터 루트: `data/jenkins-capture/scene0`
- 대상: CH1과 대응 LiDAR scan, build5~24의 12개 package
- 제외: CH2~CH4 자동 캘리브레이션, Qt GUI, 제품 RT 활성화

## 1. 결론

현재 수집한 데이터를 최대한 활용하는 방향은 맞다. 다만 **12개 package를 하나의
최적화 입력으로 모두 넣는 방식은 맞지 않다.** 데이터마다 독립성·영상 신뢰도·동적 객체
조건이 다르기 때문이다.

현재 권장 방식은 다음과 같다.

```text
12개 package 전체
  ├─ 입력/scan 무결성 검사                         12/12 사용
  ├─ CH1 chair/monitor ChArUco camera-side 검사     24 target 사용
  ├─ Case A: 과거 baseline 회귀                    build5/8/9 → build10
  ├─ Case B: 편집·근거리·사람 포함 stress 진단     build17/18/19 → build20/21
  ├─ Case C: 깨끗한 primary 후보 추정              build22/23 → build24
  └─ Case C RT 고정 적용 검증                      Case A/B 전체에 재추정 없이 적용
```

이 구조는 모든 CH1 image/scan package에 역할을 부여하면서도 다음 오염을 막는다.

- build10의 CH1 영상이 build9과 byte-identical인 문제
- build17의 편집 영상
- build19~21의 사람 포함 장면
- build24 평가 데이터를 build22·23 추정에 섞는 문제
- ChArUco 결과를 targetless 후보 선택에 넣어 검증 정답이 학습에 누출되는 문제

따라서 이 계획은 **현재 데이터에서 internal candidate RT를 추정하고 견고성을 검증하는
방법으로 타당하다.** 그러나 독립 `T_lidar_marker_board`와 독립 설치 hold-out이 없으므로
제품 정확도 증명이나 `PRODUCT_APPROVED_RT` 승격까지 완료하는 계획은 아니다.

## 2. 평가: 맞는 점과 수정한 점

### 2.1 유지하는 방향

- 먼저 CH1 하나만 안정화한다.
- Manual ChArUco로 구한 고정 `K+D` profile을 사용하고 joint intrinsic 추정은 하지 않는다.
- 원본 영상은 `raw + radtan`으로 왜곡 보정한 뒤 자동 RT를 계산한다.
- targetless automatic calibration의 후보 생성과 점수 계산에는 marker 정보를 넣지 않는다.
- 추정에 쓰지 않은 pair로 fixed-RT 검증한다.
- 모든 결과에서 `activation_allowed=false`를 유지한다.

### 2.2 수정한 방향

기존의 “모든 데이터를 활용”을 다음처럼 해석한다.

```text
잘못된 해석: 모든 pair를 같은 loss에 넣으면 데이터 활용률이 최대다.
수정된 해석: 모든 pair에 독립적인 역할을 부여하고, 평가 데이터는 재추정에 넣지 않는다.
```

특히 build5~24 전체를 `--pair-start 0 --pair-count 12`로 한 번에 실행하지 않는다. 그렇게
하면 편집·동적·중복 입력이 깨끗한 primary 추정을 지배하고 마지막 한 pair만 hold-out이
되어 결과 의미가 불명확해진다.

### 2.3 현재 검증이 증명하는 범위

| 결과 | 증명 가능한 내용 | 증명하지 못하는 내용 |
|---|---|---|
| ChArUco ROI PASS | 해당 영상에서 고정 K+D로 camera-to-board pose 계산 가능 | camera-to-LiDAR RT 정답 |
| training gate PASS | 선택 RT가 학습 장면의 내부 품질 기준 충족 | 새로운 장면의 일반화 |
| build24 hold-out PASS | 같은 설치·장면의 시간 반복에서 고정 RT가 기준 충족 | 독립 설치/독립 공간 정확성 |
| 과거 case fixed validation PASS | 같은 기계 epoch라는 전제에서 장면 조건 변화에도 내부 기준 충족 | survey/CAD 기준 절대 오차 |
| `CANDIDATE_RT` | 후속 검토 가능한 후보 | 제품 자동 활성화 허용 |

build24는 최적화에 들어가지 않으므로 development hold-out으로 사용할 수 있다. 다만 이미
사람이 영상을 보고 marker 검출을 확인했으므로 완전히 봉인된 blind product hold-out은
아니다. 앞으로 threshold나 알고리즘을 build24 결과에 맞춰 수정하면 build24도 회귀 fixture로
내리고 새 sealed hold-out을 수집해야 한다.

## 3. 변경 금지 계약

Luna 또는 다른 실행자는 다음을 변경하지 않는다.

1. `--camera-channel 1`을 유지한다. CH2~CH4 결과를 섞지 않는다.
2. 원본 image, JSON, PCD, manifest를 편집·이동·이름 변경하지 않는다.
3. build24를 training으로 이동하지 않는다.
4. build10을 독립 product hold-out으로 부르지 않는다.
5. build17을 제품 추정 근거로 사용하지 않는다.
6. build17 monitor와 build18 chair를 통과시키기 위해 최소 corner 수나 RMSE 기준을 낮추지 않는다.
7. ChArUco pose/RMSE를 automatic RT 후보의 loss, 가중치 또는 finalist 선택에 넣지 않는다.
8. hold-out 결과를 본 뒤 threshold, ROI 또는 후보를 바꾸고 같은 hold-out을 독립 검증이라 부르지 않는다.
9. `INTERNAL_GATE_PASS`, `PASS`, `CANDIDATE_RT`를 `PRODUCT_APPROVED_RT`와 동일시하지 않는다.
10. `activation_allowed`를 수동으로 `true`로 바꾸지 않는다.

## 4. 고정 입력 계약

### 4.1 실행 위치

일반 파일 확인과 문서 작업은 WSL workspace에서 수행하고, C++ build/test/실행만 Docker
container에서 수행한다.

```text
WSL repository:
/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop

Docker source mount:
/workspace

Docker build directory:
/workspace-build
```

모든 명령은 먼저 다음 경로로 이동한 뒤 실행한다.

```bash
cd /mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop
```

### 4.2 공통 profile

| 항목 | 고정값 | 의미 |
|---|---|---|
| camera channel | `1` | 이번 계획은 CH1 전용 |
| K+D | `/workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json` | 현재 CCTV profile의 고정 intrinsic |
| image distortion state | `raw` | 입력 원본에 radtan 보정 적용 |
| LDC | `unknown` | 장치에서 확인되지 않은 상태를 임의로 on/off 선언하지 않음 |
| zoom/focus | `true` | K+D 취득 뒤 zoom/focus가 고정됐다는 운용 계약 |
| camera center prior | `(0.05928, -0.08105, 0) m` | LiDAR 축 기준 물리 설치 중심 prior; RT 정답은 아님 |
| search | `staged` | full yaw coarse → distinct basin → 5° → 1° → Ceres |
| scene pass ratio | `1.0` | 선택된 evaluation pair 전부 통과 요구 |

K+D 파일, zoom/focus 또는 영상 profile이 바뀌었다면 실행을 중단한다. 같은 rigid
camera-LiDAR module을 세계 좌표에서 이동·회전한 것은 RT를 바꾸지 않지만, camera와 LiDAR
사이의 체결이 움직였다면 별도 installation epoch이므로 서로 fixed-RT 비교하지 않는다.

현재 binary에서 별도 boundary option을 주지 않았을 때의 초기 방향 탐색은 다음과 같다.

```text
yaw:             -180° 이상 +180° 미만, 15° 간격 = 360° 전체 방향
camera down:       0°~90°, 15° 간격
initial optical roll layer: 0° 한 개
basin selection:  인접 후보 보정 점수로 최대 3개 distinct yaw basin
local stage 1:    각 basin 주변 10° 반경, 5° 간격
local stage 2:    각 winner 주변 5° 반경, 1° 간격
final:            서로 30° 이상 떨어진 최대 3개 seed에 Ceres 적용
```

여기서 `camera down`은 LiDAR scan의 tilt measurement 범위를 바꾸는 값이 아니라 카메라
광축 방향 prior를 탐색하는 값이다. Luna는 `--yaw-min/max`, `--prior-roll-deg`,
`--optical-roll-*`을 임의로 추가해 search boundary를 축소하지 않는다. 실행마다 commit hash와
`orientation_full_search.csv`, `orientation_corrected_layer_*.csv`, `search_5deg_scores.csv`,
`search_1deg_scores.csv`를 보존해 실제 탐색 범위를 확인한다.

## 5. 12-package 인덱스와 역할

`run_real_calibration`이 현재 scan 이름을 정렬한 결과를 다음과 같이 고정한다.

| pair index | package | CH1 image | 역할 |
|---:|---|---|---|
| 0 | `calib_dataset_build5_20260820_223238` | `20260820_120107_CH1.jpg` | Case A training |
| 1 | `calib_dataset_build8_20260820_232413` | `20260820_231445_CH1.jpg` | Case A training |
| 2 | `calib_dataset_build9_20260820_233643` | `20260820_232737_CH1.jpg` | Case A training |
| 3 | `calib_dataset_build10_20260821_000311` | `20260820_233037_CH1.jpg` | Case A limited hold-out; build9과 동일 image |
| 4 | `calib_dataset_build17_20260821_042721` | `20260821_041806_CH1.jpg` | Case B diagnostic training; 편집 영상 |
| 5 | `calib_dataset_build18_20260821_120114` | `20260821_115114_CH1.jpg` | Case B diagnostic training; 근거리 board |
| 6 | `calib_dataset_build19_20260821_122309` | `20260821_120749_CH1.jpg` | Case B diagnostic training; 사람 포함 |
| 7 | `calib_dataset_build20_20260821_151000` | `20260822_000015_CH1.jpg` | Case B dynamic hold-out |
| 8 | `calib_dataset_build21_20260822_141545` | `20260822_225748_CH1.jpg` | Case B dynamic hold-out |
| 9 | `calib_dataset_build22_20260823_231014` | `20260823_230009_CH1.jpg` | Case C primary training |
| 10 | `calib_dataset_build23_20260823_232209` | `20260823_231209_CH1.jpg` | Case C primary training |
| 11 | `calib_dataset_build24_20260823_233514` | `20260823_232509_CH1.jpg` | Case C development hold-out |

모든 JSON은 40,400 measurement를 가지며 checksum/out-of-range error가 0이다. valid point는
40,038~40,190 범위다. PCD는 `x y z`만 포함하고 JSON에는 `signal_strength`가 있지만,
둘 다 chair/monitor target label은 없다.

PCD와 JSON은 같은 scan의 중복 표현이므로 둘을 서로 독립 표본처럼 세지 않는다. 현재
automatic RT 계산은 JSON 계약을 사용하고 PCD는 point 수·좌표·시각화 교차검사에 쓴다.

## 6. 전체 ChArUco camera-side audit

CH1마다 파란 의자와 우측 모니터에 동일 ID의 보드가 있으므로 full-frame 결과로 물리적
target identity를 판단하지 않는다. 다음 ROI registry를 그대로 사용한다.

| build | chair ROI `x,y,w,h` | chair 결과 | monitor ROI `x,y,w,h` | monitor 결과 |
|---:|---|---|---|---|
| 5 | `850,1200,700,320` | PASS, 16 marker/22 corner, 0.545437 px | `2090,700,500,650` | PASS, 17/24, 0.448867 px |
| 8 | `850,1200,700,320` | PASS, 17/24, 0.597705 px | `2090,700,500,650` | PASS, 17/24, 0.427217 px |
| 9 | `850,1200,700,320` | PASS, 16/22, 0.666473 px | `2090,700,500,650` | PASS, 17/24, 0.363043 px |
| 10 | `850,1200,700,320` | PASS, 16/22, 0.666473 px | `2090,700,500,650` | PASS, 17/24, 0.363043 px |
| 17 | `1650,1200,650,320` | PASS, 15/19, 0.668599 px | `2090,700,500,650` | **EXPECTED FAIL**, 0/0 |
| 18 | `1500,1000,900,500` | **EXPECTED FAIL**, 6 marker/5 corner | `2090,700,500,650` | PASS, 17/24, 0.442788 px |
| 19 | `1200,1200,800,320` | PASS, 16/22, 0.579475 px | `2090,700,500,650` | PASS, 17/24, 0.477407 px |
| 20 | `1200,1200,800,320` | PASS, 16/22, 0.461731 px | `2090,700,500,650` | PASS, 17/24, 0.421290 px |
| 21 | `1200,1200,800,320` | PASS, 12/15, 0.556624 px | `2090,700,500,650` | PASS, 17/24, 0.476067 px |
| 22 | `1200,1200,800,320` | PASS, 16/22, 0.639542 px | `2090,700,500,650` | PASS, 16/22, 0.405639 px |
| 23 | `1200,1200,800,320` | PASS, 16/22, 0.598607 px | `2090,700,500,650` | PASS, 15/19, 0.430960 px |
| 24 | `1200,1200,800,320` | PASS, 16/22, 0.553490 px | `2090,700,500,650` | PASS, 17/24, 0.396608 px |

현재 회귀 기대값은 정확히 `22 PASS / 2 EXPECTED FAIL`이다.

- build17 monitor: 편집된 영상에서 보드 패턴이 손상되어 corner 0
- build18 chair: 강한 원근/작은 유효 문양 때문에 ChArUco corner 5로 최소 6 미달

두 실패를 억지로 PASS로 만들지 않는다. 다른 target이 PASS이므로 camera-side 진단 정보는
남아 있지만 full `T_camera_lidar` 정답을 만들지는 않는다.

보드가 수집 시점 사이에 이동했으므로 모든 build의 `T_camera_marker_board`를 하나의 고정
world target pose처럼 평균내지 않는다. pose 반복성은 동일 배치가 확인된 묶음 안에서만
비교한다.

위 수치는 2026-08-24 동일 container binary로 임시 audit하여 확인했다. build22~24의 6개
결과는 기존 workspace output에 있고, build5~21을 포함한 통합 24-target 결과는 Step 2에서
지정한 새 output 경로에 영구 기록한다.

### 6.1 marker 단일 실행 명령

아래 template에서 `IMAGE`, `OUT`, `ROI`를 위 registry의 한 행으로 치환한다. 한 번에 한
target만 실행하고 결과를 확인한 뒤 다음 target으로 이동한다.

```bash
docker compose exec -T dev /workspace-build/bin/estimate_marker_pose \
  --board /workspace/output/pdf/charuco_a4_board_config.json \
  --camera /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image IMAGE \
  --output-dir OUT \
  --maximum-rms-px 3 \
  --roi ROI
```

출력 경로 계약:

```text
/workspace/manual_calibration/output/jenkins-scene0/ch1-all-build-charuco-audit/
  build5/{chair,monitor}/marker_pose_result.json
  ...
  build24/{chair,monitor}/marker_pose_result.json
```

각 JSON에서 확인할 필드는 다음과 같다.

```text
status
reason_code
solved
quality.marker_count
quality.charuco_corner_count
quality.reprojection_rmse_px
provenance.roi_xywh
extrinsic.parent_frame
extrinsic.child_frame
extrinsic.translation_m
```

## 7. 실행 순서

### Step 0. 작업 트리와 입력 보호

다음 항목을 먼저 기록한다.

```bash
git status --short
find data/jenkins-capture/scene0 -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort -V
```

현재 작업 트리는 이미 다른 개발 변경을 포함한다. Luna는 이를 되돌리거나 정리하지 않는다.
출력 디렉터리가 이미 있으면 기존 결과를 삭제·덮어쓰지 말고 실행을 중단한 뒤 `_r2`처럼 새
suffix를 정한다.

### Step 1. Docker/build/test preflight

```bash
docker compose ps
docker compose exec -T dev cmake --build /workspace-build \
  --target estimate_marker_pose run_real_calibration manual_marker_tests -j2
docker compose exec -T dev ctest --test-dir /workspace-build \
  -R '^manual_marker_tests$' --output-on-failure
```

기대 결과:

- container `dev`가 Up
- build exit code 0
- `manual_marker_tests` 1/1 PASS

실패하면 calibration을 실행하지 않는다. dependency 설치나 소스 수정으로 우회하지 말고
실패 로그와 명령을 기록한다.

### Step 2. 24-target marker audit

Section 6 registry 순서대로 build5 chair부터 build24 monitor까지 실행한다. 예상과 다른
PASS/FAIL이 하나라도 나오면 다음 automatic run으로 넘어가지 않는다. 다음부터 먼저
확인한다.

1. 사용한 CH1 파일이 registry와 같은가
2. ROI 좌표가 같은가
3. board가 `20 mm marker / 27 mm square` 설정인가
4. K+D 파일이 같은가
5. 원본 영상 hash가 변경됐는가

### Step 3. Case C primary 후보 추정

가장 깨끗한 build22·23만 추정에 쓰고 build24는 내부 hold-out으로 남긴다.

```bash
docker compose exec -T dev /workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_primary_build22_24 \
  --debug-output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_primary_build22_24/debug \
  --pair-start 9 --pair-count 3 --holdout-count 1 \
  --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --search-strategy staged --minimum-scene-pass-ratio 1.0
```

이 명령은 full staged search 때문에 약 20~40분 이상 걸릴 수 있다. 출력이 잠시 없다는 이유로
같은 명령을 중복 시작하지 않는다. 60초 이상 무출력일 때는 새 실행 대신 container process와
기존 터미널 상태를 확인한다.

### Step 4. Case A baseline 회귀

```bash
docker compose exec -T dev /workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_baseline_build5_10 \
  --debug-output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_baseline_build5_10/debug \
  --pair-start 0 --pair-count 4 --holdout-count 1 \
  --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --search-strategy staged --minimum-scene-pass-ratio 1.0
```

build10은 build9과 동일 CH1 image이므로 이 결과는 이전 로직과 비교하는 회귀 진단일 뿐
독립 hold-out 증거가 아니다.

### Step 5. Case B stress 진단

```bash
docker compose exec -T dev /workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_stress_build17_21 \
  --debug-output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_stress_build17_21/debug \
  --pair-start 4 --pair-count 5 --holdout-count 2 \
  --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --search-strategy staged --minimum-scene-pass-ratio 1.0
```

이 case는 편집 영상과 동적 객체가 있어 FAIL도 유효한 결과다. Case C 후보를 대신 선택하는
용도로 쓰지 않는다.

### Step 6. primary RT를 과거 baseline에 고정 적용

먼저 Case C의 `calibration_result.json`이 존재하고 `estimated_t_camera_lidar`가 있는지
확인한다. Case C가 `FAIL`이면 임의 prior를 대신 넣지 말고 이 단계는
`BLOCKED_BY_PRIMARY_CANDIDATE`로 기록한다.

```bash
docker compose exec -T dev /workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output /workspace/automatic_calibration/generated/fixed_primary_on_build5_10 \
  --debug-output /workspace/automatic_calibration/generated/fixed_primary_on_build5_10/debug \
  --pair-start 0 --pair-count 4 --holdout-count 0 \
  --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --validation-pose-json /workspace/automatic_calibration/generated/jenkins_scene0_ch1_primary_build22_24/calibration_result.json \
  --validation-label primary_build22_23_on_baseline_build5_10 \
  --minimum-scene-pass-ratio 1.0
```

이 모드는 RT를 다시 찾거나 refine하지 않는다. primary RT를 네 pair에 그대로 적용한다.

### Step 7. primary RT를 stress 데이터에 고정 적용

```bash
docker compose exec -T dev /workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output /workspace/automatic_calibration/generated/fixed_primary_on_build17_21 \
  --debug-output /workspace/automatic_calibration/generated/fixed_primary_on_build17_21/debug \
  --pair-start 4 --pair-count 5 --holdout-count 0 \
  --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --validation-pose-json /workspace/automatic_calibration/generated/jenkins_scene0_ch1_primary_build22_24/calibration_result.json \
  --validation-label primary_build22_23_on_stress_build17_21 \
  --minimum-scene-pass-ratio 1.0
```

build5~21이 Case C와 같은 rigid camera-LiDAR 체결 및 같은 K+D/zoom/focus epoch이라는 확인이
없다면 Step 6~7 결과는 `CONDITION_METADATA_UNKNOWN / DIAGNOSTIC_ONLY`로 기록한다.

## 8. exit code 처리 계약

`run_real_calibration`의 exit code를 다음처럼 해석한다.

| exit code | 의미 | Luna 행동 |
|---:|---|---|
| 0 | 해당 내부 gate PASS | JSON을 검사하고 다음 단계 진행 |
| 3 | 정상 실행됐지만 품질 gate FAIL | 실패 원인을 보존하고 다음 독립 진단 단계는 진행 가능 |
| 그 외 | 입력/파싱/build/runtime 오류 | 즉시 중단하고 명령·stderr·경로 기록 |

`set -e` 상태에서 exit 3을 도구 오류로 오인하지 않는다. 각 장시간 실행은 한 번만 시작하고
완료 뒤 exit code와 JSON을 함께 판단한다.

fixed validation의 정상 출력은 다음 두 파일이다.

```text
fixed_pose_validation_result.json
fixed_pose_scene_validation.csv
```

fixed validation은 성공해도 다음 상태가 정상이다.

```text
candidate_rt_status          = NOT_CANDIDATE_RT
product_approved_rt_status   = NOT_PRODUCT_APPROVED_RT
activation_allowed           = false
```

그 이유는 입력 RT를 검증만 했고 새 후보를 생성하지 않았기 때문이다.

## 9. 결과 확인 순서

각 automatic estimation output의 `calibration_result.json`에서 최소 다음 필드를 추출한다.

```bash
jq '{
  status,
  reason_code,
  internal_gate_status,
  candidate_rt_status,
  product_approved_rt_status,
  activation_allowed,
  selected_candidate,
  estimated_t_camera_lidar,
  training_validation_pass,
  holdout_validation_pass,
  finalist_holdout_distinctive,
  minimum_separated_holdout_objective_margin
}' automatic_calibration/generated/OUTPUT_NAME/calibration_result.json
```

fixed validation output은 다음처럼 확인한다.

```bash
jq '{
  status,
  reason_code,
  mode,
  validation_label,
  candidate_rt_status,
  product_approved_rt_status,
  activation_allowed,
  minimum_scene_pass_ratio,
  scene_validation
}' automatic_calibration/generated/OUTPUT_NAME/fixed_pose_validation_result.json
```

PNG/PLY/OBJ는 수치 판정 이후 육안 진단에 사용한다. 보기 좋아 보이는 projection만으로
PASS를 선언하지 않고, 반대로 내부 gate PASS만으로 방향이 맞다고 선언하지 않는다. 최소한
다음을 함께 본다.

- `matching_scene_*.png`
- `debug/scene_*/06_projection_final.png`
- `debug/scene_*/07_projection_final_edges.png`
- colorized LiDAR 3D preview/PLY
- scene validation CSV의 per-scene reason과 coverage

## 10. 판정표

| 단계 | 통과 조건 | 실패 시 해석 |
|---|---|---|
| 입력 preflight | 12 package, 각 CH1+JSON+PCD+manifest 존재 | dataset/package 오류 |
| marker audit | 22 PASS + 지정된 2 EXPECTED FAIL | ROI/profile/data 변경 또는 detector 회귀 |
| Case C estimation | exit 0, `CANDIDATE_RT`, hold-out PASS, activation false | 현재 clean data에서도 후보 식별 실패 |
| Case A estimation | 결과 기록; build10 제한 명시 | baseline 회귀 또는 중복 hold-out 한계 |
| Case B estimation | PASS/FAIL 모두 reason 보존 | 동적·편집 입력 fail-safe 진단 |
| primary→Case A fixed | 같은 기계 epoch일 때 scene 전부 PASS가 목표 | 장면 일반화 부족 또는 epoch 불일치 |
| primary→Case B fixed | 동적 조건에서 결과와 reason 기록 | 동적 객체 민감도 정량화 |

Case A/B에서 따로 추정한 RT와 Case C RT의 회전 geodesic/translation 차이는 보고하되 현재
데이터를 보고 새 hard threshold를 만들지 않는다. 큰 차이가 나면 어느 RT가 정답인지 score만으로
선택하지 말고 projection, scene support, installation epoch와 독립 reference 부재를 함께 기록한다.

## 11. 최종 상태 정의

현재 계획이 성공해도 가능한 최고 상태는 다음과 같다.

```text
status                       = PASS 또는 내부 gate 결과
candidate_rt_status          = CANDIDATE_RT
product_approved_rt_status   = NOT_PRODUCT_APPROVED_RT
activation_allowed           = false
```

제품 승격에는 최소 다음이 추가로 필요하다.

- 독립 `T_lidar_marker_board` 또는 survey/CAD/LiDAR-visible fixture로 만든 RT truth
- 현재 개발에 반복 사용하지 않은 sealed hold-out
- 기계 재설치 또는 독립 공간을 포함한 재현성 검증
- positive/negative fixture에서 false acceptance 측정 후 승인 threshold 고정

이 항목은 현재 12개 데이터를 덜 활용한다는 뜻이 아니다. 현재 데이터는 개발·회귀·stress
evidence로 모두 보존하고, 추가 데이터는 제품 정확성의 빈칸만 채운다.

## 12. Luna 실행 체크리스트

Luna는 각 단계가 끝날 때 아래 체크리스트를 그대로 출력하고 채운다.

```text
[ ] 작업 경로가 develop인가
[ ] CH1만 선택했는가
[ ] 원본 12 package를 수정하지 않았는가
[ ] 기존 dirty worktree를 되돌리지 않았는가
[ ] K+D/raw/LDC unknown/zoom-focus true가 동일한가
[ ] pair index와 build mapping을 다시 확인했는가
[ ] marker 결과가 정확히 22 PASS + 2 EXPECTED FAIL인가
[ ] build24가 training에서 제외됐는가
[ ] 장시간 명령을 중복 시작하지 않았는가
[ ] exit 3을 품질 거절로 분리했는가
[ ] calibration_result.json 또는 fixed_pose_validation_result.json을 읽었는가
[ ] projection PNG와 scene CSV를 함께 확인했는가
[ ] ChArUco를 targetless 후보 선택에 사용하지 않았는가
[ ] PRODUCT_APPROVED_RT/activation true로 승격하지 않았는가
[ ] 실행 명령, commit hash, 시작/종료 시각, runtime, output 경로를 기록했는가
```

### Luna 보고 형식

```text
Stage:
Command:
Input pair indices/builds:
Start/finish time:
Exit code:
Output directory:
status / reason_code:
candidate/product/activation state:
training pass:
hold-out pass:
fixed validation pass ratio:
projection inspection summary:
unexpected difference from this plan:
next allowed step:
```

Luna는 계획과 다른 파일·인덱스·profile을 발견하면 추정하지 말고
`PLAN_INPUT_MISMATCH`로 중단한다. 품질 FAIL은 소스나 threshold를 즉시 수정하라는 뜻이
아니며, 먼저 failure artifact를 보존하고 원인을 보고한다.

## 13. 실행 완료 정의

이번 계획의 완료 조건은 다음이다.

1. 12 package 입력 inventory와 marker 24-target 결과가 기록됨
2. Case A/B/C가 서로 다른 output에 한 번씩 실행됨
3. Case C RT가 Case A/B에 fixed validation으로 적용됨
4. 모든 exit code, JSON state, scene CSV, projection artifact가 연결됨
5. 데이터 제한과 제품 비승격 상태가 보고서에 명시됨

“Case C가 PASS해야만 완료”는 아니다. Case C가 정당한 reason code로 FAIL해도 계획 실행과
증거 수집은 완료될 수 있다. 다만 이 경우 fixed validation은
`BLOCKED_BY_PRIMARY_CANDIDATE`이며 후보 RT를 임의로 만들어 진행하지 않는다.

### 13.1 2026-08-24 계획 검증 기록 (실행 전 preflight)

이 절은 계획을 확정하던 실행 전 시점의 preflight 기록이다. 당시에는 장시간 calibration
자체를 시작하지 않고 Luna가 사용할 실행 계약만 다음처럼 검증했다. 실제 계획 실행 결과는
아래 §15와 `CH1_ALL_BUILD_EXECUTION_REPORT_20260824.md`에 기록되어 이 절의 상태를
대체한다.

| 검사 | 결과 |
|---|---|
| Docker `run_real_calibration --help` | 문서의 모든 automatic/fixed-validation option 존재 |
| Docker `estimate_marker_pose --help` | `--roi`, `--maximum-rms-px`, board/camera/image/output option 존재 |
| board config | Docker `/workspace/output/pdf/charuco_a4_board_config.json` 존재 |
| K+D profile | Docker `/workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json` 존재 |
| dataset package count | 12 |
| 계획 output 5개 경로 | 작성 시점 모두 미사용 |
| incremental build | `ninja: no work to do`, exit 0 |
| marker unit test | `manual_marker_tests` 1/1 PASS, 0.55 s |

따라서 다음 실행자는 Step 2부터 시작할 수 있다. 소스 commit이나 데이터가 바뀌었다면 Step
0~1을 다시 수행한다.

## 14. 변경 로그

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-24 | CH1 build5~24의 12-package를 baseline/stress/primary/fixed validation으로 분리 |
| 2026-08-24 | 전체 24개 chair/monitor ROI audit의 22 PASS/2 expected FAIL 결과 고정 |
| 2026-08-24 | build24를 development hold-out으로 정의하고 blind product hold-out과 구분 |
| 2026-08-24 | Luna 실행 명령, exit code, 중단 조건, 결과 검사 및 보고 계약 추가 |
| 2026-08-24 | Docker CLI/path/build와 `manual_marker_tests` 실행 명령 검증 |

## 15. 실행 결과 링크

실제 계획 실행 결과는
[`CH1_ALL_BUILD_EXECUTION_REPORT_20260824.md`](CH1_ALL_BUILD_EXECUTION_REPORT_20260824.md)에
기록했다. 결과 요약은 다음과 같다.

```text
ChArUco audit: 22 PASS + 2 EXPECTED FAIL
Case C primary: exit 0, CANDIDATE_RT/PASS, build24 1/1 hold-out
Case A baseline: exit 3, FAIL/FINALIST_AMBIGUOUS, margin 1.8209%
Case B stress: exit 0, CANDIDATE_RT/PASS, build20/21 2/2 hold-out
Case C fixed on A: 4/4 INTERNAL_GATE_PASS
Case C fixed on B: 5/5 INTERNAL_GATE_PASS
```

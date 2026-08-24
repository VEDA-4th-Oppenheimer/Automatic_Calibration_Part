# Jenkins 데이터셋 수집 및 Calibration Test 운영 계획

문서 상태: 운영 기준 개정

최종 수정: 2026-08-24

변경 이력: `data_storage`의 실제 Freestyle 수집 Job(`cctv_capture`, `3d_scan`, `dataset_pack`) 구성을 반영하고, Jenkins의 자동화 목적을 **실환경 데이터셋 수집 및 수집 데이터를 활용한 캘리브레이션 테스트 진행**으로 정립

실데이터 calibration의 K/D 입력과 승인 상태는 [`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)를
따른다. Jenkins의 `PASS`는 job/test gate 결과이며 `PRODUCT_APPROVED_RT` 승격과 동일하지
않다. Manual ChArUco `K+D` profile이 없는 실데이터 job은 승인 대상이 아니라 진단 job으로
분류한다.

## 1. 목적

Jenkins 자동화의 1차 목적은 사람이 개입하지 않아도 실제 센서(CCTV 카메라 및 3D LiDAR) 데이터를 주기적/자동으로 수집하여 재현 가능한 calibration dataset을 만드는 것이다.
2차 목적은 Jenkins가 수집·패키징한 카메라 이미지·LiDAR PCD·LiDAR JSON 데이터셋을 입력으로 삼아, Calibration Core 알고리즘 및 conformance test를 자동 실행하여 정확도와 성능을 검증하는 것이다.

운영 흐름은 다음과 같다.

```text
[1단계: 데이터셋 수집 & 패키징 (data_storage)]
cctv_capture (CH1~4) ──► 3d_scan (PCD/JSON) ──► dataset_pack (*.tar.gz 아티팩트 보관)
                                                        │
                                                        ▼
                                       [2단계: 수집 데이터셋 기반 테스트]
                                       calibration_dataset_test / conformance test
```

현재 Jenkins(`http://172.20.33.193:8080/job/data_storage/`)에는 다음 Freestyle Job이 구성되어 운영 중이다.

| Job | 역할 | 주요 산출물 |
|---|---|---|
| `cctv_capture` | CCTV CH1~CH4 snapshot 수집 | `*.jpg` |
| `3d_scan` | LiDAR HTTP 상태 확인, REARM/HOME, scan, 결과 회수 | `*.pcd`, `*_pan_tilt_lidar.json` |
| `dataset_pack` | 이미지와 LiDAR 결과를 한 세션으로 압축 | `calib_dataset_build*.tar.gz`, `manifest.json` |

권장 실행 순서는 `cctv_capture → 3d_scan → dataset_pack`이며, 이후 별도의
Calibration Test Job이 생성된 아티팩트 패키지를 입력으로 전달받아 테스트를 수행한다.

검증 범위:

- C++17 빌드 및 단위 테스트
- Stanford RGB/depth/pose 입력 conformance
- Synthetic `PointCloudPackage` schema 및 재현성
- Calibration ground-truth 복원 정확도
- 노이즈, dropout, overlap 부족에 대한 강건성
- 처리 시간과 메모리 회귀
- staged 탐색 순서(coarse → top-3 basin → 5° → 1° → 단일 Ceres)와 no-fallback 회귀
- `INTERNAL_GATE_PASS`/`CANDIDATE_RT`/`PRODUCT_APPROVED_RT` 상태 분리 및 false activation 0

OpenSDK/CV5 빌드 및 실기기 호환성은 이 Jenkins job의 범위에서 제외한다.
실제 센서 수집은 데이터 확보 단계이며, 수집 성공 자체를 calibration 정확도
`PASS`로 해석하지 않는다. 수집 패키지는 별도 테스트 단계의 입력으로 검증한다.

Yaw/down coarse 간격, 인접 후보 보정 및 1° fine search의 주기별 성능 시험은
[Orientation Search Step Size Test Plan](ORIENTATION_SEARCH_STEP_SIZE_TEST_PLAN.md)을
기준으로 구성한다. 해당 시험은 1°×1° full-search baseline이
`FULL_SEARCH_BASELINE_PASS`인 경우에만 실행하며, 실패 시 step-size stage를
`BLOCKED_BY_FULL_SEARCH_BASELINE`으로 종료한다.

## 2. 현재 `data_storage` Job 구성

Jenkins URL: `http://172.20.33.193:8080/`

`data_storage`는 Job이 아니라 Folder이며, 세 개의 Freestyle Job을 포함한다.
세 Job 모두 동시 실행을 허용하지 않도록 구성되어 있다.

### 2.1 `cctv_capture`

주요 파라미터:

| 파라미터 | 기본값 |
|---|---|
| `CCTV_IP` | `172.20.32.43` |
| `CCTV_USER` | `admin` |
| `CCTV_PASSWORD` | Jenkins Password parameter |
| `CCTV_PROFILE` | `1` |
| `CCTV_CHANNELS` | `1 2 3 4` |
| `OUTPUT_DIR` | `capture` |

SUNAPI snapshot API로 채널별 JPEG를 저장한다. 카메라 비밀번호는 Job 설정이나
shell에 평문으로 기록하지 않고 Jenkins Credential로 관리한다.

### 2.2 `3d_scan`

주요 파라미터:

| 파라미터 | 기본값 |
|---|---|
| `SCANNER_IP` | `172.20.26.191:8080` |
| `SCAN_TIMEOUT_SEC` | `600` |
| `OUTPUT_DIR` | `lidar_scan` |

`/api/state`로 STM32 링크와 현재 상태를 확인하고, `DISARM`이면 `REARM`,
`HOME` 완료 후 `/api/cmd/scan`을 호출한다. 새 PCD를 감지하면 PCD와 대응 JSON을
`OUTPUT_DIR`에 내려받는다.

### 2.3 `dataset_pack`

앞 단계의 `capture` 및 `lidar_scan` 산출물을 찾아 한 세션 디렉터리에 복사하고,
`manifest.json`을 생성한 뒤 압축 파일을 만든다. 현재 artifact 보관 패턴은
`*.tar.gz`이며, 실행 환경에 `zip`이 있으면 ZIP을 생성할 수 있으므로 artifact
패턴을 `*.zip,*.tar.gz`로 관리하는 것을 권장한다.

### 2.4 권장 연쇄 실행

현재는 세 Job을 순서대로 실행할 수 있다. 장기적으로는 하나의 상위 Job 또는
Parameterized Trigger로 다음 순서를 보장한다.

```text
cctv_capture (capture success)
        ↓
3d_scan (PCD + JSON success)
        ↓
dataset_pack (manifest + archive success)
        ↓
calibration_dataset_test (수집 패키지 기반 테스트)
```

수집 Job과 테스트 Job은 목적과 실패 원인을 분리한다. 카메라/actuator 통신
실패는 `collection failure`, Calibration Core 결과 실패는 `test failure`로
기록한다.

## 3. 권장 Jenkins 구성

### 3.1 Jenkins node

전용 Linux/WSL 또는 Linux VM agent에 다음 label을 부여한다.

```text
auto-calib && docker && stanford2d3ds
```

권장 최소 자원:

| 항목 | 권장값 |
|---|---:|
| CPU | 8 core 이상 |
| RAM | 16 GB 이상 |
| Docker storage | 50 GB 이상 |
| Dataset storage | 100 GB 이상 |
| Workspace storage | 30 GB 이상 |

동일 데이터 cache를 사용하는 job은 동시 실행으로 인한 I/O 포화를 막기 위해
concurrency를 제한한다.

```groovy
options {
    disableConcurrentBuilds()
    timestamps()
    timeout(time: 2, unit: 'HOURS')
}
```

## 4. Dataset과 test asset 배치

### 4.1 권장 방식: workspace 외부 read-only cache

Jenkins workspace는 SCM checkout, `cleanWs()`, node 재할당 또는 관리자의
cleanup 정책에 의해 삭제될 수 있다. 31 GB 이상의 원본 데이터셋을 workspace에
영구 보관하지 않는다.

전용 agent에 다음 구조로 한 번만 배치한다.

```text
/srv/jenkins-datasets/
  stanford-2d3ds/
    area_1/
      3d/
      data/
      pano/
      raw/
    checksums/
      area_1.sha256
    VERSION
```

권한 예시:

```bash
sudo chown -R root:jenkins /srv/jenkins-datasets/stanford-2d3ds
sudo find /srv/jenkins-datasets/stanford-2d3ds -type d -exec chmod 0750 {} \;
sudo find /srv/jenkins-datasets/stanford-2d3ds -type f -exec chmod 0640 {} \;
```

Jenkins job은 원본을 read-only bind mount한다.

```bash
export STANFORD_AREA1_PATH=/srv/jenkins-datasets/stanford-2d3ds/area_1
docker compose up -d
```

`compose.yaml`의 mount:

```yaml
volumes:
  - ${STANFORD_AREA1_PATH}:/datasets/stanford2d3ds/area_1:ro
```

장점:

- workspace cleanup과 무관하게 데이터 유지
- 원본 변조 방지
- 여러 build가 같은 데이터를 재사용
- 매 build마다 31 GB를 복사하지 않음

### 4.2 workspace에 test file을 미리 둘 경우

반드시 Jenkins가 관리하는 별도 persistent workspace 또는 custom workspace를
사용한다.

```text
/var/lib/jenkins/persistent/auto-calib/
  datasets/stanford-2d3ds/area_1/
  cases/
  expected/
```

Pipeline에서 custom workspace를 지정할 수 있다.

```groovy
agent {
    node {
        label 'auto-calib && docker && stanford2d3ds'
        customWorkspace '/var/lib/jenkins/persistent/auto-calib/workspace'
    }
}
```

이 경우 다음 규칙을 지킨다.

1. `cleanWs()`로 persistent dataset 경로를 삭제하지 않는다.
2. SCM checkout 경로와 dataset 경로를 분리한다.
3. dataset은 read-only로 Docker에 mount한다.
4. 생성 결과는 `build-output/${BUILD_NUMBER}`에 기록한다.
5. 오래된 생성 결과만 별도 cleanup job으로 삭제한다.
6. 동일 workspace를 공유하면 `disableConcurrentBuilds()`를 사용한다.

가능하지만 workspace 외부 cache 방식이 더 안전하다.

### 4.3 Artifact repository를 사용하는 방식

새 Jenkins node가 자주 생성되는 환경에서는 dataset 전체 또는 선정된 case bundle을
다음 저장소에 보관한다.

- S3/MinIO
- Nexus raw repository
- Artifactory generic repository
- 사내 NAS

예시 bundle:

```text
stanford-conformance-v1/
  manifest.json
  cases/
    C001/
      rgb.png
      depth.png
      pose.json
    C002/
      ...
  SHA256SUMS
```

Daily job은 작은 curated bundle만 내려받고, weekly/monthly job은 node-local
전체 dataset cache를 사용한다.

## 5. Test case 저장 구조

SCM에서 test 정의와 기대값을 관리한다. 대용량 원본 RGB/depth/point cloud는
SCM에 commit하지 않는다.

```text
develop/
  conformance/
    suites/
      commit.yaml
      daily.yaml
      weekly.yaml
      monthly.yaml
      release.yaml
    cases/
      C001_office_nominal.yaml
      C002_hallway_degenerate.yaml
      C003_multiplane.yaml
      C004_low_overlap.yaml
      C005_depth_dropout.yaml
      C006_range_noise.yaml
      C007_bad_initial_pose.yaml
      C008_invalid_schema.yaml
      C009_determinism.yaml
      C010_multi_scene.yaml
    expected/
      C001.json
      C002.json
      ...
    schema/
      conformance_case.schema.json
```

Case 파일은 원본 파일을 복사하지 않고 globally unique한 `frame_id`로 참조한다.

```yaml
case_id: C001_office_nominal
enabled: true
tags: [smoke, nominal, office]

source:
  dataset: stanford_2d3ds
  area: area_1
  frame_id: camera_0004591bfdc749a88db196a5d8b345cb_office_6_frame_0

synthetic_lidar:
  shape: [121, 321]
  pan_range_deg: [-40, 40]
  tilt_range_deg: [-25, 25]
  pixel_stride: 2
  noise_stddev_m: 0.0
  dropout_probability: 0.0
  seed: 7

ground_truth:
  translation_m: [0.15, -0.02, 0.08]
  rotation_rpy_deg: [2.0, -4.0, 6.0]

expected:
  generation_status: PASS
  minimum_valid_ratio: 0.40
  deterministic: true
```

## 6. Test schedule과 목록

### 6.1 Commit/PR test

트리거:

- Pull request
- main branch push
- 수동 실행

목표 실행 시간: 5분 이내

| ID | 시험 | 판정 |
|---|---|---|
| PR-001 | CMake configure/build | exit code 0 |
| PR-002 | C++ unit tests | 100% pass |
| PR-003 | CLI `--help` 및 argument validation | 기대 exit code |
| PR-004 | C001 단일 frame package 생성 | status PASS |
| PR-005 | PCD organized shape | `121 × 321` |
| PR-006 | Manifest/JSON parse | schema valid |
| PR-007 | C009 동일 seed 재현성 | golden hash 또는 두 실행 hash 일치 |

권장 case 수: 1~3개.

### 6.2 Daily test

권장 시간: 매일 02:00 KST. Jenkins hash cron을 사용하면 부하를 분산할 수 있다.

```groovy
triggers {
    cron('TZ=Asia/Seoul\nH 2 * * *')
}
```

Jenkins에 설치된 cron parser가 `TZ=`를 지원하는지 확인한다. 지원하지 않으면
controller timezone을 기준으로 환산하고 실제 첫 실행 시각을 확인한다.

목표 실행 시간: 15~30분.

| ID | 시험 | Case/조건 |
|---|---|---|
| D-001 | 빠른 unit/regression | `ctest -E '^verify_'`; 장시간 real-data 제외 |
| D-002 | Nominal office | C001 |
| D-003 | Hallway degeneracy | C002 |
| D-004 | Multi-plane room | C003 |
| D-005 | Low overlap rejection | C004 |
| D-006 | Depth dropout | 10%, 20%, 40% |
| D-007 | Range noise | 5, 10, 20 mm |
| D-008 | Initial pose perturbation | roll/pitch/yaw ±2°, ±5° |
| D-009 | Schema negative tests | 필드 누락, 잘못된 shape/unit |
| D-010 | Determinism | 고정 seed 2회 |
| D-011 | Output integrity | PCD/EXR/PNG/JSON/YAML 존재 및 parse |
| D-012 | Runtime regression | 최근 baseline 대비 허용 범위 |
| D-013 | Geometry NID path | geometry point/NID projection/finite score |
| D-014 | 180° heading recovery | 360° yaw multi-start synthetic positive |
| D-015 | NID fail-safe reason | overlap/improvement/ambiguity 기대 FAIL 코드 |
| D-016 | Capture manifest preflight | camera/scan UTC 시각·해시·profile·installation epoch |

권장 dataset: 10~20개 curated frame.

### 6.3 Weekly test

권장 시간: 일요일 03:00 KST.

```groovy
triggers {
    cron('TZ=Asia/Seoul\nH 3 * * 0')
}
```

목표 실행 시간: 1~3시간.

| ID | 시험 | Case/조건 |
|---|---|---|
| W-001 | Room category coverage | office/hallway/conference/WC/pantry |
| W-002 | Frame coverage | 100~500 frame |
| W-003 | Noise × dropout matrix | 4 × 4 조합 |
| W-004 | 6-DoF perturbation sweep | 축별 positive/negative |
| W-005 | Scan resolution sweep | 61×161, 121×321, 241×641 |
| W-006 | FOV sweep | narrow/nominal/wide |
| W-007 | Range gate sweep | near/mid/far |
| W-008 | Multi-scene | 3/5/10 scene |
| W-009 | Repeated seed test | 10 seeds |
| W-010 | Memory and runtime | peak RSS, p50/p95 |
| W-011 | Dataset checksum | selected file 및 manifest checksum |
| W-012 | Docker rebuild | no-cache 또는 base image validation |
| W-013 | Yaw multi-start sweep | 0~345° 초기 heading, 15° 간격 |
| W-014 | NID observability | texture/geometry entropy 및 후보 margin 분포 |
| W-015 | Independent hold-out | 최적화 미사용 frame 재투영 검증 |
| W-016 | Staged search policy | top-3 distinct basin, 5°/1° local, 최대 3 Ceres, dual-margin, no fallback |
| W-017 | Reference RT perturbation | ±1/3/5/10° signal NMI diagnostic |
| W-018 | Jenkins scene0 CH1 batch | 원본·동시 3-training + 2-hold-out full staged search |
| W-019 | 새 pair 단독 fail-safe | ambiguity/overlap/Manhattan reason 및 false activation 0 |
| W-020 | Finalist별 binary hold-out 진단 | **구현 완료:** 모든 separated finalist fixed RT 평가; broad gate 동률 탐지 |
| W-021 | Finalist 결정론 | 후보 입력 순열에 무관한 동일 selected index 및 reason code |
| W-022 | Finalist 연속 hold-out margin | **구현 완료:** pass-ratio tier → 학습 동일 objective → 공통 coverage → 기존 2% margin |
| W-023 | Real-data 상태 계약 | **구현·검증 완료:** 공통 래퍼로 20260818 exit 3 + `FAIL / FINALIST_AMBIGUOUS`, 20260819 exit 0 + `CANDIDATE_RT / PASS`와 제품 비승격을 검사 |
| W-024 | Manhattan feature-prior 일관성 | **구현·검증 완료:** finalist별 training seed prior를 hold-out에도 고정하고 직교 소실점 회귀 및 실데이터 JSON policy로 검사 |
| W-025 | CH1 dual-ChArUco ROI preflight | **ROI 도구·수동 검증 완료, Jenkins 등록 대기:** build5~24의 24 target을 분리해 22 PASS + 2 expected rejection 및 camera-side pose 기록 |
| W-026 | Marker RT reference 완결성 | `T_camera_marker_board`와 독립 `T_lidar_marker_board`가 같은 board frame인지 검사; 후자가 없으면 `RT_REFERENCE_INCOMPLETE` 기대 거절 |
| W-027 | CH1 전체 데이터 계층형 활용 | **구현·실행 완료:** Case A baseline, Case B stress, Case C primary와 primary RT의 fixed cross-condition validation을 분리; 12-pair 일괄 학습 금지 |

2026-08-22~23 실측에서 CTest 9종은 `111.59 s`, CH1 단독 staged search는 pair당
약 8~10분, 기존 3-training + 2-hold-out batch는 약 22분 37초~30분 24초였고
최대 3 Ceres finalist와 1° 확장 탐색을 포함한 최신 batch는 약 40분이 관측됐다. 따라서
CTest와 manifest preflight는 daily, 실데이터 full search는
weekly/수동 목록으로 유지한다. 수정된 build20/21 영상은 파일명·EXIF가 scan 시작명과
일치하지만, build17 편집 영상·동적 객체·manifest 자동 provenance gate 부재 때문에
현재 build17~21은 W-018 제품 입력이 아닌 diagnostic/stress fixture다. 상세 목록과 결과는
[JENKINS_SCENE0_CH1_EXPANDED_TEST_20260822.md](JENKINS_SCENE0_CH1_EXPANDED_TEST_20260822.md)를
따른다.

2026-08-24 W-020 binary 실행에서 선택 167°, 경쟁 87°/−106° finalist가 모두
build20/21 hold-out `2/2 PASS`했다. W-022는 새 가중치를 만들지 않고 학습과 같은
목적함수와 모든 후보의 공통 coverage 분모를 적용했다. 목적함수는 각각
`0.763763 / 0.816782 / 0.871447`, 선택 후보의 최소 margin은 `6.491%`로 기존 2%
기준을 통과했다. 최신 기대 결과는 `CANDIDATE_RT / PASS`지만 제품 승인은 아니므로
`activation_allowed=false`다. 2% 기준의 물리 정확성은 독립 fixture에서 추가 검증한다.

W-024 적용 직후 build17~21, 20260818, 20260819를 각각 1회 실행했고, 이후 1회 제한을
해제한 뒤 20260818/19를 정식 weekly CTest로 다시 실행했다. 수정 전후 선택 `R,t`, 상태,
목적함수와 full/5°/1° score map은 동일했다. 기대 상태는 각각
`CANDIDATE_RT/PASS`, `FAIL/FINALIST_AMBIGUOUS`, `CANDIDATE_RT/PASS`로 유지한다.
공통 `verify_real_calibration_result.cmake`는 성공과 예상 거절 양쪽에서 exit code,
`status`, `reason_code`, candidate/product 상태, `activation_allowed=false`와 Manhattan
prior policy를 검사한다.
2026-08-24 최신 weekly 결과는 `2/2 PASS`, 병렬 real time `1240.91 s`다
(`20260819=1027.19 s`, `20260818=1240.90 s`). 이 검사는 현재 수치를 golden truth로
승인하는 시험이 아니라, hold-out에서 candidate RT로 수직 소실점 특징을 다시 고르는
평가 누수를 막는 계약 시험이다.

#### build22~24 training/hold-out 및 ChArUco reference

2026-08-24 추가된 build22~24는 기존 build17~21 진단 batch와 섞지 않고 다음 새 case로
고정한다.

```text
training = build22, build23
hold-out = build24
camera channel = CH1
primary marker = monitor ROI 2090,700,500,650
secondary marker = chair ROI 1200,1200,800,320
```

시간상 앞선 두 package를 training으로 사용하고 마지막 build24를 최적화에 넣지 않는다.
세 scan은 모두 40,183개 이상의 valid point와 checksum error 0을 가지며, 세 image/scan
hash는 서로 다르다. 동일 설치·동일 시야이므로 이는 temporal hold-out이지 독립 설치
hold-out은 아니다.

W-025는 세 build의 두 ROI에서 모두 PASS했다. 모니터 보드의 최대 camera pose 차이는
`1.094355° / 10.795 mm`, 파란 의자 보드는 `0.922760° / 4.229 mm`다. 이 값은
camera-side marker 반복성 관측값이며 사후 제품 threshold로 확정하지 않는다.

W-026의 full RT truth는 다음 식으로만 만든다.

```text
T_camera_lidar_reference
  = T_camera_marker_board * inverse(T_lidar_marker_board)
```

현재 입력에는 독립 `T_lidar_marker_board`가 없으므로 W-026 기대 상태는
`RT_REFERENCE_INCOMPLETE`다. Marker image만으로 만든 pose를 전체 RT truth로 취급하거나
targetless 학습 후보 선택에 넣지 않는다. 상세 입력, 검출 결과와 실행 명령은
[`JENKINS_SCENE0_CH1_BUILD22_24_CHARUCO_REFERENCE_PLAN_20260824.md`](JENKINS_SCENE0_CH1_BUILD22_24_CHARUCO_REFERENCE_PLAN_20260824.md)를
따른다.

W-025를 build5~24 전체로 확장한 수동 audit 결과는 24 target 중 22 PASS다. 예상 거절은
편집된 build17 monitor의 0 corner와 강한 원근의 build18 chair 5 corner다. 두 case의
검출 기준을 낮추지 않는다.

W-027은 전체 12 package를 하나의 optimizer에 넣지 않는다. build5/8/9→10은 baseline,
build17/18/19→20/21은 stress, build22/23→24는 primary로 분리하고, primary RT를 앞의
두 범위에 재추정 없이 고정 적용한다. 정확한 pair index, 명령, exit-code 의미와 Luna 실행
계약은
[`CH1_ALL_BUILD_DATA_UTILIZATION_AND_LUNA_EXECUTION_PLAN_20260824.md`](CH1_ALL_BUILD_DATA_UTILIZATION_AND_LUNA_EXECUTION_PLAN_20260824.md)를
따른다.

2026-08-24 실제 실행 결과는 다음과 같다.

- Case C build22·23→24: exit 0, `CANDIDATE_RT/PASS`, train 2/2, hold-out 1/1, margin 2.9436%
- Case A build5/8/9→10: exit 3, `FAIL/FINALIST_AMBIGUOUS`, train 3/3, hold-out 1/1, margin 1.8209%
- Case B build17/18/19→20/21: exit 0, `CANDIDATE_RT/PASS`, train 3/3, hold-out 2/2, margin 6.4912%
- Case C RT fixed→Case A: 4/4 `INTERNAL_GATE_PASS`
- Case C RT fixed→Case B: 5/5 `INTERNAL_GATE_PASS`

상세 JSON/PNG/CSV 경로는
[`CH1_ALL_BUILD_EXECUTION_REPORT_20260824.md`](CH1_ALL_BUILD_EXECUTION_REPORT_20260824.md)를
기준으로 한다.

권장 CTest 분리는 다음과 같다.

```bash
# PR/daily: 빠른 deterministic 회귀
ctest --test-dir /workspace-build -E '^verify_' --output-on-failure

# weekly: 실데이터 expected-rejection + candidate regression
ctest --test-dir /workspace-build -L weekly -j2 --output-on-failure
```

### 6.4 Monthly test

권장 시간: 매월 첫째 일요일.

목표 실행 시간: 야간 장시간 허용.

| ID | 시험 | 내용 |
|---|---|---|
| M-001 | area_1 broad regression | 가능한 전체 또는 stratified 전체 |
| M-002 | Full parameter matrix | noise/dropout/FOV/resolution/pose |
| M-003 | Long-run stability | 반복 생성 및 resource leak |
| M-004 | Golden baseline refresh candidate | 차이 보고만 생성 |
| M-005 | Dependency inventory | compiler/CMake/OpenCV/PCL/Ceres 버전 |
| M-006 | Docker image rebuild | 최신 `ubuntu:latest` 영향 확인 |
| M-007 | Dataset integrity | 전체 SHA-256 검증 |
| M-008 | Trend report | 정확도, 실패율, runtime, memory 추세 |

Golden 결과는 monthly job이 자동 교체하지 않는다. 사람이 차이를 검토하고 별도
승인된 commit으로 갱신한다.

### 6.5 Release candidate test

Release tag 또는 수동 승인 후 실행한다.

| ID | 시험 | 필수 조건 |
|---|---|---|
| R-001 | Commit + daily + weekly suite | 전부 PASS |
| R-002 | 고정 Docker image digest | 기록 필수 |
| R-003 | 고정 dataset version/checksum | 기록 필수 |
| R-004 | Calibration accuracy gate | 승인된 threshold |
| R-005 | Known failure behavior | 기대 FAIL reason 일치 |
| R-006 | Report completeness | JUnit/JSON/plots/artifacts |
| R-008 | False-PASS regression | session-003/130333 historical 후보가 PASS되지 않음 |
| R-007 | Reproducibility rerun | 동일 결과 |

## 7. Jenkins job 구성

권장 job:

```text
auto-calib-pr
auto-calib-daily
auto-calib-weekly
auto-calib-monthly
auto-calib-release
auto-calib-dataset-verify
auto-calib-cleanup
```

Parameter 예시:

```groovy
parameters {
    choice(
        name: 'SUITE',
        choices: ['commit', 'daily', 'weekly', 'monthly', 'release'],
        description: '실행할 conformance suite'
    )
    string(
        name: 'CASE_FILTER',
        defaultValue: '',
        description: '특정 case/tag만 실행'
    )
    booleanParam(
        name: 'REBUILD_IMAGE',
        defaultValue: false,
        description: 'Docker image를 다시 빌드'
    )
}
```

## 8. Jenkinsfile 예시

```groovy
pipeline {
    agent {
        label 'auto-calib && docker && stanford2d3ds'
    }

    options {
        disableConcurrentBuilds()
        timestamps()
        timeout(time: 3, unit: 'HOURS')
        buildDiscarder(logRotator(
            numToKeepStr: '30',
            artifactNumToKeepStr: '10'
        ))
    }

    parameters {
        choice(
            name: 'SUITE',
            choices: ['commit', 'daily', 'weekly', 'monthly', 'release'],
            description: 'Conformance suite'
        )
        string(name: 'CASE_FILTER', defaultValue: '')
        booleanParam(name: 'REBUILD_IMAGE', defaultValue: false)
    }

    environment {
        STANFORD_AREA1_PATH =
            '/srv/jenkins-datasets/stanford-2d3ds/area_1'
        COMPOSE_PROJECT_NAME = "auto-calib-${BUILD_NUMBER}"
    }

    stages {
        stage('Checkout') {
            steps {
                checkout scm
            }
        }

        stage('Dataset preflight') {
            steps {
                sh '''
                    test -d "$STANFORD_AREA1_PATH/data/rgb"
                    test -d "$STANFORD_AREA1_PATH/data/depth"
                    test -d "$STANFORD_AREA1_PATH/data/pose"
                    test -r "$STANFORD_AREA1_PATH/3d/pointcloud.mat"
                '''
            }
        }

        stage('Build environment') {
            steps {
                dir('develop') {
                    sh '''
                        if [ "$REBUILD_IMAGE" = "true" ]; then
                            docker compose build --pull
                        fi
                        docker compose up -d
                    '''
                }
            }
        }

        stage('Build') {
            steps {
                dir('develop') {
                    sh '''
                        docker compose exec -T dev \
                          cmake -S /workspace -B /workspace-build -G Ninja
                        docker compose exec -T dev \
                          cmake --build /workspace-build
                    '''
                }
            }
        }

        stage('Unit tests') {
            steps {
                dir('develop') {
                    sh '''
                        mkdir -p "build-output/${BUILD_NUMBER}"
                        docker compose exec -T dev \
                          ctest --test-dir /workspace-build \
                          -E '^verify_' \
                          --output-on-failure \
                          --output-junit \
                          /workspace/build-output/${BUILD_NUMBER}/ctest.xml
                    '''
                }
            }
        }

        stage('Conformance') {
            steps {
                dir('develop') {
                    sh '''
                        # conformance runner 구현 후 사용:
                        # docker compose exec -T dev \
                        #   /workspace-build/bin/run_conformance \
                        #   --suite /workspace/conformance/suites/${SUITE}.yaml \
                        #   --dataset /datasets/stanford2d3ds/area_1 \
                        #   --output /workspace/build-output/${BUILD_NUMBER}

                        # 현재 단일 smoke case:
                        OPENCV_IO_ENABLE_OPENEXR=1 \
                        docker compose exec -T \
                          -e OPENCV_IO_ENABLE_OPENEXR=1 dev \
                          /workspace-build/bin/generate_synthetic_scan \
                          --dataset-root \
                            /datasets/stanford2d3ds/area_1 \
                          --output \
                            /workspace/build-output/${BUILD_NUMBER}/C001 \
                          --columns 321 --rows 121 --pixel-stride 2 \
                          --noise-stddev-m 0.005 \
                          --dropout 0.01 --seed 7
                    '''
                }
            }
        }
    }

    post {
        always {
            junit(
                testResults: 'develop/build-output/*/ctest.xml',
                allowEmptyResults: true
            )
            archiveArtifacts(
                artifacts:
                    'develop/build-output/**/*',
                fingerprint: true,
                allowEmptyArchive: true
            )
            dir('develop') {
                sh 'docker compose down --remove-orphans || true'
            }
        }
    }
}
```

## 9. Test runner가 생성해야 할 결과

각 case:

```text
build-output/<build-number>/
  summary.json
  junit.xml
  C001/
    result.json
    manifest.yaml
    calibration/
    cloud/
    qa/
  metrics/
    runtime.csv
    memory.csv
    accuracy.csv
```

`result.json` 권장 필드:

```json
{
  "case_id": "C001_office_nominal",
  "status": "INTERNAL_GATE_PASS",
  "internal_gate_status": "INTERNAL_GATE_PASS",
  "candidate_rt_status": "NOT_CANDIDATE_RT",
  "product_approved_rt_status": "NOT_PRODUCT_APPROVED_RT",
  "activation_allowed": false,
  "reason_codes": [],
  "translation_error_m": null,
  "rotation_error_deg": null,
  "valid_ratio": 0.64,
  "lidar_geometry_points": 11063,
  "nid_projected_points": 10546,
  "initial_nid": 0.98985,
  "final_nid": 0.98938,
  "composite_objective_improvement_ratio": 0.002945,
  "multistart_candidates": 8,
  "multistart_objective_margin": 0.02,
  "search_strategy": "staged",
  "ceres_execution_policy": "up_to_3_distinct_yaw_finalists",
  "search_stages": [],
  "runtime_ms": 120,
  "peak_rss_mb": 180,
  "input_hash": "",
  "output_hash": "",
  "software_version": "",
  "docker_image_digest": ""
}
```

Synthetic ground truth가 없는 실제 데이터는 accuracy 필드를 `null`로 두되 NID,
복합 목적함수, projected ratio, multi-start margin과 reason code를 반드시 기록한다.
`PASS`는 독립 reference 또는 hold-out 검증 전까지 배포 승인과 동일하게 취급하지 않는다.

## 10. 보존과 cleanup 정책

| 산출물 | 보존 |
|---|---|
| PR 결과 | 최근 10~30 build |
| Daily summary/JUnit | 30~90일 |
| Weekly summary | 6~12개월 |
| Monthly trend | 장기 보존 |
| Release 결과 | release 수명 동안 |
| 원본 dataset | 별도 cache, 자동 삭제 금지 |
| 생성 PCD/이미지 | 실패 case 우선 보존 |

성공한 모든 PCD를 장기간 보관하면 저장공간이 빠르게 증가한다. 성공 build는
summary와 hash 위주로 보존하고, 실패한 case의 입력 설정과 결과 package를
archive한다.

Cleanup job은 다음만 대상으로 한다.

```text
workspace/build-output/*
Docker build cache
오래된 Jenkins artifact
```

다음은 삭제 대상에서 제외한다.

```text
/srv/jenkins-datasets/**
conformance/cases/**
conformance/expected/**
release baseline/**
```

## 11. Dataset 초기 배치 절차

1. Jenkins agent를 중지하거나 관련 job 실행을 잠시 막는다.
2. dataset을 임시 경로에 복사한다.
3. 디렉터리 구조와 RGB/depth/pose 개수를 확인한다.
4. 전체 또는 주요 파일의 SHA-256을 생성한다.
5. 최종 cache 경로로 atomic rename한다.
6. 소유자와 read-only 권한을 설정한다.
7. `auto-calib-dataset-verify` job을 실행한다.
8. Daily job에서 작은 smoke case를 실행한다.

예시:

```bash
rsync -a --info=progress2 \
  /source/area_1/ \
  /srv/jenkins-datasets/stanford-2d3ds/area_1.tmp/

find /srv/jenkins-datasets/stanford-2d3ds/area_1.tmp \
  -type f -print0 |
  sort -z |
  xargs -0 sha256sum \
  > /srv/jenkins-datasets/stanford-2d3ds/checksums/area_1.sha256

mv \
  /srv/jenkins-datasets/stanford-2d3ds/area_1.tmp \
  /srv/jenkins-datasets/stanford-2d3ds/area_1
```

기존 dataset을 교체할 때는 새 버전을 별도 디렉터리에 검증한 뒤 symlink를
전환한다.

```text
/srv/jenkins-datasets/stanford-2d3ds/
  area_1-v1/
  area_1-v2/
  area_1-current -> area_1-v2
```

## 12. 운영 체크리스트

- [ ] Jenkins agent label과 Docker 권한 확인
- [ ] dataset cache가 workspace 외부인지 확인
- [ ] Docker mount가 read-only인지 확인
- [ ] dataset version 및 SHA-256 기록
- [ ] case YAML과 expected 결과를 SCM에서 관리
- [ ] PR/daily/weekly/monthly job 분리
- [ ] Jenkins controller timezone 확인
- [ ] JUnit과 JSON summary archive
- [ ] 실패 case 결과 package 보존
- [ ] Golden baseline 자동 갱신 금지
- [ ] Docker image digest 기록
- [ ] dataset cleanup 제외 규칙 확인
- [x] build22~24 CH1 보드별 ROI 및 camera-side ChArUco pose 검증
- [x] build5~24 CH1 전체 24-target ROI audit: 22 PASS + 2 expected rejection
- [ ] 선택한 동일 보드의 독립 `T_lidar_marker_board` 확보
- [ ] build22·23 training RT를 build24 fixed hold-out 및 marker RT reference와 비교
- [x] Case A/B/C 분리 실행 및 Case C RT의 Case A/B fixed validation

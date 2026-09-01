# Jenkins scene0 CH1 파일별 target 분류 보고서

- 작성일: 2026-08-24 (KST)
- 대상: `data/jenkins-capture/scene0`의 build5~24, 총 12개 package
- 목적: build22~24의 image/JSON/PCD를 상세 분류하고, build5~24 전체의 파란 의자/우측 모니터 ChArUco 및 추정·검증 역할을 고정한다.
- 판정 범위: CH1을 주 대상 채널로 한다. CH2~CH4는 파일 존재와 기존 detector 결과를 기록하되 CH1 RT reference 계산에는 사용하지 않는다.

## 1. 최종 분류

현재 파일 세트에서 확정할 수 있는 것은 `T_camera_marker_board`이다.

```text
CH1 image + ROI
  ├─ chair ROI   → T_camera_marker_board(chair)
  └─ monitor ROI → T_camera_marker_board(monitor)

JSON/PCD
  └─ scan 전체는 읽을 수 있으나 target_id/marker_id/board point association 없음
```

따라서 LiDAR 쪽은 다음 상태로 분류한다.

```text
T_lidar_marker_board = UNLABELED
전체 T_camera_lidar reference = RT_REFERENCE_INCOMPLETE
```

이는 LiDAR에 보드가 없다는 뜻이 아니라, 현재 파일 포맷만으로 어느 LiDAR point가 의자 보드 또는 모니터 보드인지 증명할 수 없다는 뜻이다.

## 2. 보드 분류 계약

두 보드는 같은 ChArUco 설정과 같은 marker ID를 사용하므로 full-frame detector에 함께 넣으면 물리적 보드 identity가 보장되지 않는다. 현재는 ROI로 분리한다.

| target_id | 물리적 위치 | CH1 원본 ROI `x,y,w,h` | camera-side 판정 |
|---|---|---|---|
| `charuco_chair` | 파란 의자 위 | `1200,1200,800,320` | build22~24 모두 PASS |
| `charuco_monitor` | 우측 책상 모니터 앞 | `2090,700,500,650` | build22~24 모두 PASS |

ROI 결과 파일은 다음 위치에 있다.

```text
manual_calibration/output/jenkins-scene0/ch1-build22-24-charuco/
  build22/{chair,monitor}/marker_pose_result.json
  build23/{chair,monitor}/marker_pose_result.json
  build24/{chair,monitor}/marker_pose_result.json
```

## 3. 파일별 분류

### 3.1 build22

패키지: `calib_dataset_build22_20260823_231014`

| 파일 | 분류 | 근거 및 사용 범위 |
|---|---|---|
| `20260823_230009_CH1.jpg` | `PRIMARY_CAMERA / TWO_TARGETS` | chair와 monitor가 모두 보인다. full-frame은 두 동일-ID 보드가 섞이므로 ROI 검출만 사용한다. |
| `20260823_230009_CH2.jpg` | `SECONDARY_CHANNEL / NOT_ASSIGNED` | 기존 전체 데이터 audit에서 ChArUco 검출은 가능하지만 chair/monitor 의미를 CH1과 연결한 pose 결과가 없다. CH1 RT 계산에서 제외한다. |
| `20260823_230009_CH3.jpg` | `SECONDARY_CHANNEL / NOT_ASSIGNED` | 동일. 별도 채널 검증용으로만 보존한다. |
| `20260823_230009_CH4.jpg` | `SECONDARY_CHANNEL / INSUFFICIENT` | 기존 audit에서 검출 corner가 부족한 채널이다. reference 입력으로 사용하지 않는다. |
| `calib-20260824-080033_sweep-000001_pan_tilt_lidar.json` | `PAIR_SCAN / VALID_UNLABELED` | 40,400 samples, valid 40,188, checksum/out-of-range 오류 0. pan/tilt/distance/signal_strength는 있으나 target association은 없다. |
| `calib-20260824-080033_sweep-000001.pcd` | `PAIR_POINTCLOUD / XYZ_ONLY` | valid point 40,188, PCD field가 `x y z`뿐이다. chair/monitor label 또는 signal_strength가 없다. |
| `manifest.json` | `PAIR_METADATA` | CCTV 4, PCD 1, LiDAR JSON 1을 기록한다. 파일 hash와 board identity는 기록하지 않는다. |

camera-side pose:

| target | marker/corner | RMSE | camera translation (m) |
|---|---:|---:|---|
| chair | 16 / 22 | 0.639540 px | `[0.244631, 0.668992, 1.948859]` |
| monitor | 16 / 22 | 0.405639 px | `[1.058917, 0.157637, 1.703032]` |

판정: `camera-side PASS`, `LiDAR target association MISSING`.

### 3.2 build23

패키지: `calib_dataset_build23_20260823_232209`

| 파일 | 분류 | 근거 및 사용 범위 |
|---|---|---|
| `20260823_231209_CH1.jpg` | `PRIMARY_CAMERA / TWO_TARGETS` | build22와 동일 ROI로 chair/monitor를 분리한다. |
| `20260823_231209_CH2.jpg` | `SECONDARY_CHANNEL / NOT_ASSIGNED` | 별도 채널 검증용. CH1 target identity와 연결하지 않는다. |
| `20260823_231209_CH3.jpg` | `SECONDARY_CHANNEL / NOT_ASSIGNED` | 별도 채널 검증용. |
| `20260823_231209_CH4.jpg` | `SECONDARY_CHANNEL / INSUFFICIENT` | reference 입력으로 사용하지 않는다. |
| `calib-20260824-081228_sweep-000001_pan_tilt_lidar.json` | `PAIR_SCAN / VALID_UNLABELED` | 40,400 samples, valid 40,183, checksum/out-of-range 오류 0. target ID가 없다. |
| `calib-20260824-081228_sweep-000001.pcd` | `PAIR_POINTCLOUD / XYZ_ONLY` | valid point 40,183, `x y z`만 기록한다. target label이 없다. |
| `manifest.json` | `PAIR_METADATA` | package 단위 image/scan 개수만 기록한다. |

camera-side pose:

| target | marker/corner | RMSE | camera translation (m) |
|---|---:|---:|---|
| chair | 16 / 22 | 0.598605 px | `[0.245345, 0.669271, 1.950799]` |
| monitor | 15 / 19 | 0.430960 px | `[1.053292, 0.156918, 1.693846]` |

판정: `camera-side PASS`, `LiDAR target association MISSING`.

### 3.3 build24

패키지: `calib_dataset_build24_20260823_233514`

| 파일 | 분류 | 근거 및 사용 범위 |
|---|---|---|
| `20260823_232509_CH1.jpg` | `PRIMARY_CAMERA / TWO_TARGETS / HOLDOUT` | build22·23으로 역할을 분리한 뒤 최적화에 넣지 않고 검증에만 사용한다. |
| `20260823_232509_CH2.jpg` | `SECONDARY_CHANNEL / NOT_ASSIGNED` | CH1 hold-out 판정에 사용하지 않는다. |
| `20260823_232509_CH3.jpg` | `SECONDARY_CHANNEL / NOT_ASSIGNED` | CH1 hold-out 판정에 사용하지 않는다. |
| `20260823_232509_CH4.jpg` | `SECONDARY_CHANNEL / INSUFFICIENT` | reference 입력으로 사용하지 않는다. |
| `calib-20260824-082527_sweep-000001_pan_tilt_lidar.json` | `PAIR_SCAN / VALID_UNLABELED / HOLDOUT` | 40,400 samples, valid 40,189, checksum/out-of-range 오류 0. Automatic 최적화에는 넣지 않고 fixed-RT 평가에 사용한다. |
| `calib-20260824-082527_sweep-000001.pcd` | `PAIR_POINTCLOUD / XYZ_ONLY / HOLDOUT` | valid point 40,189. target label이 없어 LiDAR-side ground truth로는 사용할 수 없다. |
| `manifest.json` | `PAIR_METADATA / HOLDOUT` | build24 package 경계를 보존한다. |

camera-side pose:

| target | marker/corner | RMSE | camera translation (m) |
|---|---:|---:|---|
| chair | 16 / 22 | 0.553488 px | `[0.244406, 0.668355, 1.946779]` |
| monitor | 17 / 24 | 0.396608 px | `[1.058340, 0.157684, 1.701613]` |

판정: `camera-side PASS`, `LiDAR target association MISSING`, `hold-out 사용 가능`.

## 4. 역할 분류

| 데이터 | training | hold-out | 전체 RT reference |
|---|---:|---:|---:|
| build22 | 사용 | 아니오 | 불완전 |
| build23 | 사용 | 아니오 | 불완전 |
| build24 | 아니오 | 사용 | 불완전 |

build22·23은 동일 고정 환경의 training pair, build24는 untouched temporal hold-out이다. 세 패키지는 모두 같은 설치 상태 반복성만 검증하며, `T_lidar_marker_board`가 없으므로 제품 승인용 RT truth로 승격하지 않는다.

## 5. 현재 파일로 가능한 실행

가능한 작업:

- CH1 이미지에서 chair/monitor ROI별 ChArUco 검출
- `T_camera_marker_board(chair)`와 `T_camera_marker_board(monitor)` 반복성 비교
- build22·23 automatic training 및 build24 fixed-RT hold-out 평가
- JSON/PCD 전체 scan의 품질·유효점·시간 pair 검사

아직 불가능한 작업:

- PCD만 읽고 chair/monitor point를 자동으로 확정
- `T_lidar_marker_board` 독립 산출
- ChArUco pose만으로 전체 `T_camera_lidar` 정답 선언

## 6. 다음 입력 파일

현재 데이터를 그대로 이용해 진단을 진행하려면 보드별 LiDAR ray 또는 point index를 다음처럼 추가한다.

```json
{
  "target_id": "charuco_chair",
  "scan_file": "calib-20260824-080033_sweep-000001_pan_tilt_lidar.json",
  "association": {
    "method": "manual_ray_indices",
    "rows": [],
    "columns": []
  },
  "status": "diagnostic_only"
}
```

이 annotation은 파이프라인을 실행하기 위한 분류 파일이며 독립 reference가 아니다. 제품용 reference가 필요하면 ChArUco를 LiDAR-visible rigid panel에 부착하고 panel geometry 또는 독립 `T_lidar_marker_board`를 제공해야 한다.

## 7. 최종 판정

```text
파일 pair 무결성                  PASS
CH1 chair/monitor image 분리       PASS (ROI 기반)
CH1 camera-side pose               PASS (6/6)
JSON/PCD target association         MISSING
독립 T_lidar_marker_board          MISSING
제품 승인 RT                       NOT_ALLOWED
```

## 8. build5~21의 처리 상태

앞의 파일별 표는 이번 요청의 핵심인 CH1 chair/monitor ChArUco 분류를 위해
build22~24를 상세 기록한 것이다. `build5`부터의 기존 package를 삭제하거나 사용 금지한
것은 아니다. 다만 수집 조건과 독립성의 차이가 있으므로 build22~24와 같은 reference
case로 섞지 않는다.

`run_real_calibration`의 현재 scan 정렬은 다음과 같다.

```text
index 0  build5
index 1  build8
index 2  build9
index 3  build10
index 4  build17
index 5  build18
index 6  build19
index 7  build20
index 8  build21
index 9  build22
index 10 build23
index 11 build24
```

| package | 분류 | 사용 범위 | 주의점 |
|---|---|---|---|
| build5 | `BASELINE_TRAINING` | 과거 3-training baseline | 최신 binary로 재실행한 독립 golden은 아님 |
| build8 | `BASELINE_TRAINING` | 과거 baseline 반복성 | 동일 baseline 묶음으로만 비교 |
| build9 | `BASELINE_TRAINING` | 과거 baseline | build10과 CH1 image가 중복되는지 함께 확인 필요 |
| build10 | `LIMITED_HOLDOUT` | 과거 hold-out 진단 | CH1이 build9과 byte-identical하여 독립 hold-out으로 승격하지 않음 |
| build17 | `DIAGNOSTIC_TRAINING` | 확장 search/ambiguity 진단 | 편집된 영상 입력이므로 제품 evidence 제외 |
| build18 | `DIAGNOSTIC_TRAINING` | 확장 진단 | 근거리 board/촬영 조건 차이 포함 |
| build19 | `DIAGNOSTIC_TRAINING` | 확장 진단 | 사람 포함 장면 |
| build20 | `DYNAMIC_HOLDOUT` | finalist hold-out 및 stress 진단 | 동적 객체, manifest provenance 보강 전 제품 입력 제외 |
| build21 | `DYNAMIC_HOLDOUT` | finalist hold-out 및 stress 진단 | build20과 같은 제한을 적용 |
| build22 | `CHARUCO_TRAINING` | chair/monitor ROI camera-side reference + automatic training | LiDAR target association은 아직 없음 |
| build23 | `CHARUCO_TRAINING` | chair/monitor ROI camera-side reference + automatic training | LiDAR target association은 아직 없음 |
| build24 | `CHARUCO_HOLDOUT` | build22·23으로 추정한 RT의 untouched temporal hold-out | 독립 설치 hold-out이 아니며 LiDAR target association은 없음 |

정리하면 build5~21은 다음 용도로 계속 사용한다.

- 기존 로직과 최신 로직의 baseline/regression 비교
- yaw ambiguity, 구조선 부족, 동적 객체에 대한 stress test
- build17~21 finalist hold-out 정책 검증
- 전체 12개 scan에서 시간 순서·입력 유효성·성능 측정

반대로 다음 용도로는 사용하지 않는다.

- chair/monitor의 독립 `T_lidar_marker_board` 정답으로 사용
- build22~24 ChArUco reference와 근거 없이 혼합
- 사람 포함/편집/중복 package를 제품 승인용 독립 hold-out으로 선언

따라서 현재 운영 목록은 `기존 build5~21 = baseline/diagnostic`,
`build22~24 = 신규 CH1 ChArUco reference case`로 분리하는 것이 맞다.

## 9. build5~24 전체 CH1 ChArUco audit

12개 CH1 영상 모두에서 파란 의자와 우측 모니터의 보드 ROI를 별도로 검사했다. 같은 ID의
두 보드를 full-frame 결과로 합치지 않았으며, 전체 24개 target 중 22개가 PASS했다.

| build | chair | monitor | package 역할 |
|---:|---|---|---|
| 5 | PASS | PASS | baseline training |
| 8 | PASS | PASS | baseline training |
| 9 | PASS | PASS | baseline training |
| 10 | PASS | PASS | limited hold-out; build9 CH1과 byte-identical |
| 17 | PASS | `EXPECTED FAIL: 0 corner` | edited-image diagnostic |
| 18 | `EXPECTED FAIL: 5 corner` | PASS | near-board diagnostic |
| 19 | PASS | PASS | dynamic-scene training |
| 20 | PASS | PASS | dynamic hold-out |
| 21 | PASS | PASS | dynamic hold-out |
| 22 | PASS | PASS | primary training |
| 23 | PASS | PASS | primary training |
| 24 | PASS | PASS | development hold-out |

build17 monitor와 build18 chair의 검출 기준을 낮춰 통과시키지 않는다. 나머지 target의
camera-side 정보를 사용할 수는 있지만, 어느 결과도 독립 `T_lidar_marker_board` 없이
전체 camera-LiDAR RT truth가 되지는 않는다.

전체 데이터 활용 원칙, ROI registry의 marker/corner/RMSE 값, Case A/B/C 명령과 Luna
실행 중단 조건은
[`CH1_ALL_BUILD_DATA_UTILIZATION_AND_LUNA_EXECUTION_PLAN_20260824.md`](CH1_ALL_BUILD_DATA_UTILIZATION_AND_LUNA_EXECUTION_PLAN_20260824.md)를
따른다.

### 변경 로그

| 날짜 | 내용 |
|---|---|
| 2026-08-24 | build22~24의 image/JSON/PCD/manifest를 파일별로 분류 |
| 2026-08-24 | chair/monitor ROI pose 결과와 training/hold-out 역할을 연결 |
| 2026-08-24 | LiDAR target association 부재를 `RT_REFERENCE_INCOMPLETE`로 명시 |
| 2026-08-24 | build5~21을 삭제 대상이 아닌 baseline/diagnostic/stress fixture로 역할 분리 |
| 2026-08-24 | build5~24 CH1의 두 보드 24-target audit를 22 PASS/2 expected FAIL로 기록 |
| 2026-08-24 | 전체 12-package 계층형 추정·검증 및 Luna 실행 계획 연결 |

# Jenkins `scene0` CH1 build22~24 ChArUco reference 계획

- 작성일: 2026-08-24 (KST)
- 입력 범위: `data/jenkins-capture/scene0/calib_dataset_build22_*` ~ `build24_*`
- 카메라 채널: CH1
- 설치 상태: 동일 설치·동일 고정 장면·조명 조건
- 데이터 분할: **build22·23 training / build24 hold-out**
- 카메라 marker 판정: **PASS**
- 전체 LiDAR–camera RT 참값 판정: **`RT_REFERENCE_INCOMPLETE`**
- 제품 적용: **금지**, `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`

이 문서는 다음 운영 계약을 읽고 build22~24를 현재 세션의 후속 시험 입력으로
확정한 기록이다.

- [`JENKINS_CONFORMANCE_TEST_PLAN.md`](JENKINS_CONFORMANCE_TEST_PLAN.md)
- [`JENKINS_FREESTYLE_CAPTURE_SCAN.md`](JENKINS_FREESTYLE_CAPTURE_SCAN.md)

## 1. 결론

1. CH1에는 같은 `DICT_5X5_100`, `7×5`, square `27 mm`, marker `20 mm`인 A4
   ChArUco 보드가 **파란 의자 위**와 **우측 책상 모니터 앞** 두 곳에 있다.
2. 전체 2592×1520 frame에서는 보드가 작고 같은 ID 보드가 두 장 존재하여 기본
   detector가 두 보드를 안전하게 분리하지 못한다.
3. 보드별 ROI를 분리하고 crop만큼 camera principal point `cx,cy`를 이동하면 세 build의
   두 보드가 모두 검출되고 `T_camera_marker_board` pose가 계산된다.
4. 순서와 데이터 누수를 고려해 build22·23을 training, 가장 늦게 수집된 build24를
   untouched hold-out으로 사용한다.
5. 검출된 ChArUco pose는 **카메라 측 reference truth**로 사용한다. 그러나 현재 scan에는
   같은 보드의 독립 `T_lidar_marker_board`가 없으므로 이를 곧바로 전체
   `T_camera_lidar` RT 정답이라고 기록하지 않는다.

## 2. 입력 감사와 역할 선정

패키지 내부 image와 LiDAR JSON/PCD를 authoritative pair로 사용한다. image 파일명은 UTC,
scan 파일명은 KST로 해석하면 image가 scan 시작보다 약 18~24초 먼저 수집됐다. 현재
manifest에는 camera/scan UTC가 별도 필드로 없으므로 이 시간 차이는 filename 기반
진단값이며, package 경계가 최종 pairing 근거다.

| build | CH1 image | LiDAR scan | filename 기준 차이 | valid point | 역할 |
|---:|---|---|---:|---:|---|
| 22 | `20260823_230009_CH1.jpg` | `calib-20260824-080033_*` | 24 s | 40,188 | training 1 |
| 23 | `20260823_231209_CH1.jpg` | `calib-20260824-081228_*` | 19 s | 40,183 | training 2 |
| 24 | `20260823_232509_CH1.jpg` | `calib-20260824-082527_*` | 18 s | 40,189 | hold-out |

선정 근거:

- 세 CH1 image와 세 scan은 SHA-256이 각각 달라 byte duplicate가 아니다.
- 세 scan 모두 `checksum_error_count=0`, `out_of_range_angle_count=0`이다.
- build22↔23 scan의 동일 grid point 거리 중앙값은 `13.022 mm`, build22↔24는
  `18.057 mm`, build23↔24는 `16.301 mm`다.
- build24는 시간상 마지막이며 build22·23보다 point distance p90이 큰 쪽이므로, 같은
  환경 안에서 상대적으로 더 어려운 temporal hold-out 역할이 적절하다.
- 이 분할은 **동일 시야·동일 설치 반복성**만 검증한다. 다른 장소·다른 installation
  epoch의 일반화 hold-out을 대체하지 않는다.

## 3. ChArUco 보드 선택과 ROI 계약

두 보드는 같은 marker ID를 사용하므로 한 detector 입력에 함께 넣어 pose를 계산하면
어느 물리 보드의 pose인지 모호해질 수 있다. 다음 ROI를 CH1 2592×1520 원본 좌표로
고정한다.

| 보드 | 용도 | ROI `x,y,w,h` | 판정 |
|---|---|---|---|
| 모니터 앞 보드 | primary camera-side reference | `2090,700,500,650` | 세 build 모두 PASS |
| 파란 의자 위 보드 | secondary consistency reference | `1200,1200,800,320` | 세 build 모두 PASS; frame 하단 일부 잘림 |

`estimate_marker_pose`에 `--roi`를 추가했다. 구현은 원본을 resize하지 않고 ROI만 자른 뒤
camera matrix를 다음처럼 보정한다.

```text
cx_roi = cx_full - roi_x
cy_roi = cy_full - roi_y
fx_roi = fx_full
fy_roi = fy_full
distortion_roi = distortion_full
```

따라서 출력 pose frame은 crop frame이 아니라 계속 `camera_optical`이며, overlay에는 사용한
ROI를 노란 사각형으로 표시한다.

## 4. 실제 검출 및 camera-side pose 결과

사용 profile:

```text
manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json
profile_id = charuco-pass-clean18-20260814
resolution = 2592×1520
image state = raw + OpenCV radtan D
```

### 4.1 모니터 앞 보드

| build | marker | corner | reprojection RMSE | `t_camera_marker_board` m |
|---:|---:|---:|---:|---|
| 22 | 16 | 22 | 0.405639 px | `[1.058917, 0.157637, 1.703032]` |
| 23 | 15 | 19 | 0.430960 px | `[1.053292, 0.156918, 1.693846]` |
| 24 | 17 | 24 | 0.396608 px | `[1.058340, 0.157684, 1.701613]` |

최대 pair 차이:

- rotation geodesic: `1.094355°`
- translation: `10.795 mm`

### 4.2 파란 의자 위 보드

| build | marker | corner | reprojection RMSE | `t_camera_marker_board` m |
|---:|---:|---:|---:|---|
| 22 | 16 | 22 | 0.639540 px | `[0.244631, 0.668992, 1.948859]` |
| 23 | 16 | 22 | 0.598605 px | `[0.245345, 0.669271, 1.950799]` |
| 24 | 16 | 22 | 0.553488 px | `[0.244406, 0.668355, 1.946779]` |

최대 pair 차이:

- rotation geodesic: `0.922760°`
- translation: `4.229 mm`

두 보드 사이의 상대 pose 반복성도 최대 rotation `0.707956°`, translation
`3.545 mm`다. 이는 세 image가 같은 정적 장면을 관측하고 ROI pose 계산이 반복된다는
카메라 측 근거다. 제품 정확도 threshold를 이 세 샘플에 맞춰 사후 확정하지 않고 관측값으로
기록한다.

## 5. RT 정답으로 사용하는 정확한 계약

ChArUco 검출로 직접 얻은 값은 다음이다.

```text
T_camera_marker_board
```

전체 LiDAR–camera RT 정답은 다음 두 값이 모두 있어야 계산된다.

```text
T_camera_lidar_reference
  = T_camera_marker_board
  × inverse(T_lidar_marker_board)
```

현재 A4 종이는 의자·모니터 표면에 놓인 평면 인쇄물이다. LiDAR JSON/PCD에는 marker ID가
없고 종이 문양 자체가 depth 구조를 만들지 않으므로, 현 데이터만으로 어느 LiDAR point가
정확히 marker-board frame인지를 증명할 수 없다.

따라서 현재 적용 범위는 다음과 같이 고정한다.

| 사용 | 허용 여부 |
|---|---|
| build22·23 marker 검출 preflight | 허용 |
| build22·23 `T_camera_marker_board` 반복성 검사 | 허용 |
| build24 marker pose를 untouched camera-side hold-out으로 검사 | 허용 |
| marker edge를 targetless RT 학습 목적함수의 정답으로 직접 주입 | 금지; 평가 누수 및 제품 목적 변경 |
| `T_lidar_marker_board` 없이 marker pose를 전체 RT truth로 선언 | 금지 |
| 독립 `T_lidar_marker_board` 추가 후 Automatic RT 6-DoF 비교 | 허용 및 다음 단계 |

즉 사용자가 요청한 “ChArUco를 RT 정답으로 사용”은 **동일한 물리 보드의 LiDAR pose를
추가한 뒤** 완성한다. 현 단계에서 ChArUco는 camera-side truth로 이미 사용 가능하며,
전체 RT truth 상태만 `RT_REFERENCE_INCOMPLETE`로 fail-closed한다.

## 6. 실행 명령

모니터 앞 보드 예시:

```bash
/workspace-build/bin/estimate_marker_pose \
  --board /workspace/output/pdf/charuco_a4_board_config.json \
  --camera /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image /workspace/data/jenkins-capture/scene0/calib_dataset_build22_20260823_231014/20260823_230009_CH1.jpg \
  --roi 2090,700,500,650 \
  --maximum-rms-px 3 \
  --output-dir /workspace/manual_calibration/output/jenkins-scene0/ch1-build22-24-charuco/build22/monitor
```

파란 의자 보드는 `--roi 1200,1200,800,320`을 사용한다.

생성 위치:

```text
manual_calibration/output/jenkins-scene0/ch1-build22-24-charuco/
  build22/{monitor,chair}/
  build23/{monitor,chair}/
  build24/{monitor,chair}/
    marker_pose_result.json
    marker_pose_report.md
    marker_pose_overlay.png
```

## 7. 다음 실행 순서

1. build22·23 두 pair로 targetless Automatic RT를 추정한다.
2. build24는 최적화에 넣지 않고 fixed-RT hold-out으로만 평가한다.
3. 동시에 build24의 두 ChArUco ROI가 검출 PASS인지 확인한다.
4. primary인 모니터 앞 보드 또는 새 LiDAR-visible rigid board의
   `T_lidar_marker_board`를 독립 생성한다.
5. `compare_marker_to_automatic`으로 Automatic RT와 marker reference RT의 rotation/
   translation 오차를 계산한다.
6. 4번이 완료되기 전 결과는 최대 `CANDIDATE_RT`; 완료 후에도 합의된 accuracy threshold,
   독립 epoch 및 반복성 조건을 모두 통과해야 `PRODUCT_APPROVED_RT`를 검토한다.

현재 12개 package의 scan 정렬에서 build22~24는 index 9~11이므로 Automatic 실행은
다음과 같다.

```bash
/workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_build22_24_2train_1holdout \
  --pair-start 9 --pair-count 3 --holdout-count 1 \
  --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --search-strategy staged --minimum-scene-pass-ratio 1.0
```

이 명령은 build24 marker pose를 최적화에 사용하지 않는다. 향후 독립
`T_lidar_marker_board`가 생기면 다음 비교를 별도 수행한다.

```bash
/workspace-build/bin/compare_marker_to_automatic \
  --manual-pose manual_calibration/output/jenkins-scene0/ch1-build22-24-charuco/build24/monitor/marker_pose_result.json \
  --board-in-lidar manual_calibration/output/jenkins-scene0/ch1-build22-24-charuco/build24/monitor/T_lidar_marker_board.json \
  --automatic automatic_calibration/generated/jenkins_scene0_ch1_build22_24_2train_1holdout/calibration_result.json \
  --output-dir manual_calibration/output/jenkins-scene0/ch1-build22-24-charuco/build24/comparison
```

## 8. 다음 수집 시 보강 항목

- 같은 ID 보드는 한 camera frame에 한 장만 둔다.
- 보드는 frame 경계에서 잘리지 않고 영상 면적의 약 15~30%를 차지하게 한다.
- 종이만 붙이지 말고 LiDAR에서 외곽·normal·원점을 식별할 수 있는 두께 있는 rigid panel에
  부착한다.
- panel의 board origin/축과 LiDAR-visible corner 간 CAD 치수를 기록한다.
- manifest에 camera capture UTC, scan start/end UTC, image/JSON/PCD SHA-256, channel,
  resolution, zoom/focus/LDC, installation epoch를 기록한다.

## 9. 변경 로그

| 날짜 | 내용 |
|---|---|
| 2026-08-24 | build22~24 입력 감사, build22·23 training/build24 hold-out 역할 확정 |
| 2026-08-24 | CH1 모니터/파란 의자 보드 ROI 분리, 세 build 총 6개 camera-side pose PASS |
| 2026-08-24 | `estimate_marker_pose --roi`와 crop principal-point 보정 추가 |
| 2026-08-24 | camera-side marker truth와 전체 RT truth의 완료 조건을 분리하고 다음 실행 순서 확정 |

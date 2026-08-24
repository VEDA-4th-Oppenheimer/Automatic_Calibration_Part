# 2D ChArUco Manual Calibration 사용 방법

- 작성일: 2026-07-30
- 수정 이력:
  - 2026-07-31 — Galaxy Tab S7 전체 화면 ChArUco를 기본 현장 매체로 변경
  - 2026-07-30 — LiDAR 없는 2D ChArUco Manual Calibration으로 전면 재설계
  - 2026-08-14 — session-const-env 태블릿 display geometry 보정 및 예비 RT 결과 문서화

## 1. 빌드

`develop`에서 실행한다.

```bash
./scripts/install-ubuntu-deps.sh
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

실행 파일은 `build/bin`에 생성된다. Docker 구성(`Dockerfile`, `compose.yaml`)은
선택적인 로컬 개발 환경이며 이 문서의 기본 경로가 아니다.

## 2. Galaxy Tab S7용 ChArUco 생성

기본 방식은 인쇄가 아니라 Tab S7 화면 표시다. 공식 사양인 278.1 mm, 2560×1600을 사용하는 내장 프로필로 생성한다.

```bash
build/bin/generate_charuco_board --display-profile galaxy-tab-s7 --output-dir manual_calibration/output/tablet-board
```

출력:

```text
tablet-board/
  charuco_board.png      # 2560×1600 네이티브 화면
  board_config.json      # 실제 meter 단위 square/marker 크기
  display_spec.json      # 화면·픽셀·100 mm 검증선 명세
```

`charuco_board.png`를 Tab S7으로 복사하고 가로 방향 immersive fullscreen에서 1:1로 표시한다. 상태 표시줄, 내비게이션 바, 화면 맞춤 확대, crop은 모두 없어야 한다. 자동 회전과 화면 꺼짐을 해제하고 밝기를 고정한다.

화면 위 검증선의 실제 길이를 자로 측정하고 `display_spec.json`의 `verification_ruler_expected_m`와 비교한다. 차이가 있으면 활성 화면 폭을 실측해 다음처럼 재생성한다.

```bash
build/bin/generate_charuco_board --display-profile galaxy-tab-s7 --display-width-mm 235.828 --output-dir manual_calibration/output/tablet-board-measured
```

이후 모든 calibration 명령에는 같은 출력 디렉터리의 `board_config.json`을 사용한다. 화면 표시 크기와 JSON의 물리 크기가 다르면 translation scale이 틀어진다.

QR code보다 ChArUco를 사용한다. ChArUco는 sub-pixel corner와 부분 가림 복구를 제공해 camera calibration에 적합하다. 기존 인쇄 보드는 `--board` 옵션을 쓰는 호환 모드로만 유지한다.

## 3. CCTV image 수집

Zoom/focus/resolution/WDR/IR 상태를 고정한 뒤 같은 profile에서 촬영한다.

권장:

- 15–30장
- 태블릿을 CCTV 화면 중앙·네 모서리·좌우·상하에 배치
- 정면뿐 아니라 pitch/yaw가 다른 자세 포함
- 태블릿의 ChArUco가 CCTV 화면의 20–70%를 차지하도록 거리 변경
- motion blur, 과노출, 심한 반사 제외
- CCTV 설치·zoom·focus는 고정하고 태블릿만 이동

다음 구조로 저장한다.

```text
manual_calibration/data/session-001/
  intrinsic_images/
    001.png
    002.png
    ...
```

## 4. Camera intrinsic 계산

```bash
build/bin/calibrate_camera_markers \
  --board manual_calibration/output/tablet-board/board_config.json \
  --images-dir manual_calibration/data/session-001/intrinsic_images \
  --output-dir manual_calibration/output/session-001/intrinsic \
  --minimum-frames 10 \
  --maximum-rms-px 2.0 \
  --camera-model PNM-C16083RVQ \
  --profile-id channel1-fixed-zoom-focus-v1
```

출력:

```text
intrinsic/
  camera_intrinsic.json
  camera_intrinsic_report.md
  detections/
```

`detections` 이미지를 확인해 오검출·blur frame을 제거하고 재실행한다.

## 5. 2D image에서 Marker pose 계산

기준 위치에 전체 화면 ChArUco를 표시한 태블릿을 고정하고 image 한 장을 촬영한다.

```bash
build/bin/estimate_marker_pose \
  --board manual_calibration/output/tablet-board/board_config.json \
  --camera manual_calibration/output/session-001/intrinsic/camera_intrinsic.json \
  --image manual_calibration/data/session-001/reference_pose.png \
  --output-dir manual_calibration/output/session-001/pose
```

같은 ID 보드가 여러 장 있거나 전체 frame에서 marker가 너무 작으면 보드 하나만 포함하는
원본 pixel ROI를 지정한다.

```bash
build/bin/estimate_marker_pose \
  --board output/pdf/charuco_a4_board_config.json \
  --camera manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image data/jenkins-capture/scene0/calib_dataset_build22_20260823_231014/20260823_230009_CH1.jpg \
  --roi 2090,700,500,650 \
  --output-dir manual_calibration/output/jenkins-scene0/ch1-build22-24-charuco/build22/monitor
```

ROI 경로는 영상을 resize하지 않으며 `cx_roi=cx-roi_x`, `cy_roi=cy-roi_y`로 camera
matrix를 이동한다. 따라서 출력 transform은 계속 원래 `camera_optical` 기준이다. ROI를
사용하지 않고 crop image에 원본 K를 그대로 넣으면 principal point가 틀려 pose가 왜곡될
수 있다.

출력:

```text
pose/
  marker_pose_result.json
  marker_pose_report.md
  marker_pose_overlay.png
```

출력 transform은 다음뿐이다.

```text
T_camera_marker_board
```

이 단계는 LiDAR 또는 point cloud를 사용하지 않는다.

## 6. 태블릿 display geometry를 이용한 RT reference 추정

카메라 image와 LiDAR scan이 같은 태블릿 pose에서 수집되고, ChArUco board가 활성 display 중앙에 표시되었다면 다음 보정 경로를 사용할 수 있다.

```text
LiDAR points → T_lidar_display_plane
display_spec + board_config + portrait/CW90
             → T_display_plane_marker_board

T_lidar_marker_board
  = T_lidar_display_plane × T_display_plane_marker_board

T_camera_lidar
  = T_camera_marker_board × inverse(T_lidar_marker_board)
```

이번 session-const-env에서 확인한 설정은 다음과 같다.

```text
tablet: Samsung Galaxy Tab S7
display: portrait
board image rotation: 90° clockwise
board placement: active display center
```

ChArUco marker ID와 코너 순서는 카메라 기준 보드 방향을 결정하므로 `T_camera_marker_board` 계산 때 회전값을 매번 입력하지 않는다. 회전값은 LiDAR display plane frame과 board frame을 연결할 때만 사용한다.

현재 계산 결과는 다음에 저장되어 있다.

```text
manual_calibration/output/session-const-env/lidar-tablet-reference/
  T_lidar_marker_board_110828.json
  T_camera_lidar_110828.json
  tablet_marker_rt_report_110828.md
```

이 결과의 status는 `ESTIMATED_GEOMETRY_CORRECTED`다. LiDAR plane 후보의 평균 잔차가 약 9.08 mm, 최대 잔차가 약 74.65 mm이므로 screen-edge fit, 반복 스캔, camera/LiDAR 장착 sanity check 전에는 최종 conformance PASS로 사용하지 않는다. 상세 기록은 [`SESSION_CONST_ENV_CALIBRATION_RECORD.md`](SESSION_CONST_ENV_CALIBRATION_RECORD.md)를 참조한다.

현재 이 보정 경로를 위한 전용 `estimate_tablet_marker_pose` CLI는 아직 없으며, 위 결과는 기록된 입력 보고서와 geometry contract를 이용해 생성된 세션 산출물이다. 반복 운영에 투입하기 전 LiDAR plane/edge fit CLI를 추가해야 한다.

## 7. Automatic 결과와 6-DoF 비교

Marker image만으로는 `T_camera_lidar`를 만들 수 없다. 독립적으로 측정한
`T_lidar_marker_board`가 있을 때만 다음 명령을 사용한다.

```bash
build/bin/compare_marker_to_automatic \
  --manual-pose manual_calibration/output/session-001/pose/marker_pose_result.json \
  --board-in-lidar manual_calibration/data/session-001/T_lidar_marker_board.json \
  --automatic automatic_calibration/generated/calibration_core_multi_validation/calibration_result.json \
  --output-dir manual_calibration/output/session-001/comparison
```

Threshold는 프로젝트에서 합의된 경우에만 추가한다.

```text
--max-rotation-deg 3.0 --max-translation-m 0.05
```

Reference가 없다면 pose 직접 비교를 생략하고
[`COMPARISON_PROTOCOL.md`](COMPARISON_PROTOCOL.md)의 작업 시간·재시도·반복성
비교만 수행한다.

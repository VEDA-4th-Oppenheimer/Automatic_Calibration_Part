# 2D ChArUco Manual Calibration 사용 방법

- 작성일: 2026-07-30
- 수정 이력:
  - 2026-07-31 — Galaxy Tab S7 전체 화면 ChArUco를 기본 현장 매체로 변경
  - 2026-07-30 — LiDAR 없는 2D ChArUco Manual Calibration으로 전면 재설계

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

## 6. Automatic 결과와 6-DoF 비교

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

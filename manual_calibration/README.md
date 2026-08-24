# Manual Marker Calibration

- 작성일: 2026-07-30
- 수정 이력:
  - 2026-07-31 — Galaxy Tab S7 전체 화면 ChArUco를 기본 현장 매체로 변경
  - 2026-07-30 — LiDAR 없는 2D ChArUco Manual Calibration으로 전면 재설계
  - 2026-08-14 — session-const-env camera/LiDAR 태블릿 geometry 보정 결과 기록

LiDAR 없이 CCTV의 2D image만 사용해 ChArUco board를 검출하는 전통적 calibration
기준 프로젝트다. 기본 현장 매체는 Galaxy Tab S7이며 보드를 인쇄하지 않는다. 제품 실행 경로가 아니라 Automatic 방식이 줄여야 할 현장 작업량과
기준 재투영 성능을 측정하기 위한 독립 baseline이다.

기능:

- Galaxy Tab S7용 2560×1600 ChArUco PNG, 실제 크기 board config와 100 mm 검증선 생성
- 실측 display 폭을 반영한 marker 물리 크기 보정
- 인쇄 보드는 호환용 선택 기능으로 유지
- 여러 2D image로 camera intrinsic/distortion calibration
- 한 장의 image로 `T_camera_marker_board` pose 계산
- 검출 corner와 reprojection overlay 생성
- 독립적인 `T_lidar_marker_board` reference가 있을 때 Automatic
  `T_camera_lidar`와 같은 frame에서 비교
- 태블릿 display plane과 문서화된 board geometry로 예비 `T_camera_lidar`를 계산하는
  별도 진단 경로

기본 2D ChArUco intrinsic/marker pose 계산에는 PCD, LiDAR JSON, point cloud를 사용하지
않는다. 태블릿-assisted RT reference 확장 경로에서만 LiDAR JSON/PCD를 display plane
검출 입력으로 사용한다.

- [Manual RT 기준값 취득 및 인수인계](docs/MANUAL_RT_REFERENCE_WORKFLOW.md)
- [session-const-env 작업 기록 및 계산 결과](docs/SESSION_CONST_ENV_CALIBRATION_RECORD.md)

- 상세 사용법: [`docs/USAGE.md`](docs/USAGE.md)
- 설계: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)
- 수치 비교 원칙: [`docs/COMPARISON_PROTOCOL.md`](docs/COMPARISON_PROTOCOL.md)

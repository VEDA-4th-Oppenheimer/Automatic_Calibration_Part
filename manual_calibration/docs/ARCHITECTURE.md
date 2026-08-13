# 2D Marker Manual Calibration Architecture

- 작성일: 2026-07-30
- 수정 이력:
  - 2026-07-31 — Galaxy Tab S7 전체 화면 ChArUco를 기본 현장 매체로 변경
  - 2026-07-30 — LiDAR 없는 2D ChArUco Manual Calibration으로 전면 재설계

## 목적

기존 CCTV 현장에서 수행하는 marker board 기반 작업을 독립 baseline으로 재현한다.
Manual calibration 알고리즘은 LiDAR sensor, pan-tilt JSON, PCD 또는 Automatic 결과를 읽지 않는다. `compare_marker_to_automatic`은 calibration 이후 독립 기준 변환이 있을 때만 사용하는 별도 평가 utility이며 Manual pose 계산에는 관여하지 않는다.

```text
Galaxy Tab S7 (2560×1600 immersive fullscreen ChArUco)
        │
        ├─ 여러 위치·각도의 2D CCTV images ─> Camera intrinsic/distortion
        │
        └─ 기준 위치의 2D CCTV image ─> T_camera_marker_board
```

Tab S7 프로필은 공식 278.1 mm, 2560×1600 사양에서 16:10 활성 화면 폭을 계산한다. 생성된 100 mm 검증선을 실제 자로 확인하고, 차이가 있으면 실측 활성 화면 폭으로 board config를 다시 생성한다. 상태 표시줄·내비게이션 바·이미지 확대가 남으면 픽셀-물리 길이 대응이 깨지므로 immersive fullscreen 1:1 표시를 필수 조건으로 둔다.

Automatic 제품 경로:

```text
Natural camera image + 1D LiDAR pan-tilt sweep
        └─ targetless Calibration Core ─> T_camera_lidar
```

## 출력의 차이

- Manual pose: `T_camera_marker_board`
- Automatic pose: `T_camera_lidar`

두 transform은 child frame이 다르므로 직접 차감하면 안 된다. Marker image에는
LiDAR frame 정보가 전혀 없기 때문이다.

독립적으로 실측한 다음 reference가 있을 때만 변환 가능하다.

```text
T_lidar_marker_board
```

이때:

```text
T_camera_lidar
  = T_camera_marker_board × inverse(T_lidar_marker_board)
```

`T_lidar_marker_board`는 정밀 jig, CAD/측정기 또는 survey로 얻어야 한다. Automatic
결과에서 역산하면 독립 비교가 아니므로 금지한다.

## 제품 목적과 관계

Manual ChArUco 방식은 인쇄 보드 대신 보유한 Galaxy Tab S7을 사용해 제작·운반 비용을 줄인다. 다만 태블릿 전체 화면 설정, 물리 크기 검증, 다양한 자세의 반복 촬영과 작업자 판단은 필요하다. Automatic 방식은 이러한 표적 설치를 없애고 휴대 가능한
저가 scanner와 자연 장면만으로 calibration하는 것이 목표다.

따라서 개발 성공 평가는 pose 정확도뿐 아니라 다음 작업량 감소를 함께 본다.

- 현장 준비 시간
- 태블릿 배치와 이동 횟수
- 필요한 작업자 수
- 촬영 frame 수
- 재시도 횟수
- 전체 소요 시간
- 결과 재현성

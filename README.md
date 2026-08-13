# Camera Calibration Workspace

기존 CCTV 설치 후 발생하는 calibration 작업의 불편과 비용을 저가 1D LiDAR
pan-tilt 장치로 줄일 수 있는지 개발·검증하는 workspace다.

## 프로젝트 분리

| 디렉터리 | 역할 | 현장 입력 | 주 출력 |
|---|---|---|---|
| `automatic_calibration/` | 제품 개발 경로: 자연 장면 기반 targetless camera–LiDAR extrinsic | Camera image + 1D LiDAR pan-tilt sweep | `T_camera_lidar` |
| `manual_calibration/` | 저비용 현장 기준: Galaxy Tab S7 전체 화면 ChArUco를 촬영하는 LiDAR 없는 2D image-only 방식 | Tab S7 ChArUco camera images | Camera intrinsic, `T_camera_marker_board` |
| `top_view_gui/` | Automatic/Manual 공용 RT 소비 및 기준평면 Top-View Qt GUI | Camera image + intrinsic + RT + 선택적 plane RT | Top-View PNG + metadata |

Manual Marker 결과는 Automatic optimizer의 입력이나 성공 gate로 사용하지 않는다.
두 pose의 child frame이 다르므로 marker image만으로 `T_camera_lidar`와 직접 비교할
수 없다. 독립적으로 실측한 `T_lidar_marker_board`가 있을 때만 같은 frame으로
변환해 rotation/translation 차이를 계산한다.

## Ubuntu native 시작

```bash
./scripts/install-ubuntu-deps.sh
./scripts/verify-dev-env.sh
```

## Ubuntu native 전체 빌드와 테스트

```bash
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

실행 파일은 `build/bin`에 생성된다. `Dockerfile`과 `compose.yaml`은 로컬 개발용
보조 환경이며 Git 저장소의 기본 실행 경로가 아니다.

- Automatic 사용법: [`automatic_calibration/README.md`](automatic_calibration/README.md)
- Manual Marker 사용법: [`manual_calibration/docs/USAGE.md`](manual_calibration/docs/USAGE.md)
- 비교 프로토콜: [`manual_calibration/docs/COMPARISON_PROTOCOL.md`](manual_calibration/docs/COMPARISON_PROTOCOL.md)
- 공용 Top-View GUI: [`top_view_gui/README.md`](top_view_gui/README.md)
- 개발환경 가이드: [`docs/DEVELOPMENT_ENVIRONMENT.md`](docs/DEVELOPMENT_ENVIRONMENT.md)

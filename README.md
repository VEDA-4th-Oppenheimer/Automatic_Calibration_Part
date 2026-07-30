# 개발환경

이 폴더가 Docker 컨테이너의 `/workspace`에 마운트된다. 소스 코드는
`develop/` 아래에서 작성하고 빌드 결과는 Docker 볼륨 `/workspace-build`에
저장한다.

## 시작

`develop` 폴더에서 실행한다.

```bash
docker compose up -d --build
docker compose exec dev verify-dev-env
docker compose exec dev bash
```

## 빌드 및 테스트

```bash
docker compose exec dev cmake \
  -S /workspace -B /workspace-build -G Ninja
docker compose exec dev cmake --build /workspace-build
docker compose exec dev ctest \
  --test-dir /workspace-build --output-on-failure
```

## 종료

```bash
docker compose stop
```

컨테이너만 제거하고 빌드 볼륨은 유지하려면 다음을 실행한다.

```bash
docker compose down
```

`docker compose down --volumes`는 빌드 결과와 셸 기록까지 삭제하므로 필요한
경우에만 사용한다.

OpenSDK 크로스 컴파일 시에는 공식 `opensdk:26.05.19` 이미지와
`SOC=cv5` 설정을 별도 구성해야 한다. 이 환경은 호스트 측 알고리즘 개발과
단위 테스트를 위한 Ubuntu native 환경이다.

## Stanford 합성 pan-tilt scan 생성

`area_1`은 기본적으로 컨테이너의 `/datasets/stanford2d3ds/area_1`에 읽기 전용 마운트된다. 경로가 다르면 `.env.example`을 `.env`로 복사해 수정한다.

```bash
docker compose exec dev /workspace-build/generate_synthetic_scan \
  --dataset-root /datasets/stanford2d3ds/area_1 \
  --output /workspace/generated/area1_smoke \
  --columns 321 --rows 121 --pixel-stride 2 \
  --tx-m 0.15 --ty-m -0.02 --tz-m 0.08 \
  --roll-deg 2 --pitch-deg -4 --yaw-deg 6 \
  --noise-stddev-m 0.005 --dropout 0.01 --seed 7
```

설계와 한계는 [`docs/SYNTHETIC_DATA_ARCHITECTURE.md`](docs/SYNTHETIC_DATA_ARCHITECTURE.md)를 참조한다.

## Calibration Core

단일 장면 smoke test와 다중 장면 공동 최적화를 제공한다. 최종 보정 검증에는 다중 장면 사용을 권장한다.

```bash
docker compose exec -T dev /workspace-build/run_synthetic_calibration --help
docker compose exec -T dev /workspace-build/run_multi_synthetic_calibration --help
```

API, 좌표계, 품질 게이트, Stanford 검증 결과와 실제 actuator 연결 경계는 [`docs/CALIBRATION_CORE_ARCHITECTURE.md`](docs/CALIBRATION_CORE_ARCHITECTURE.md)를 참조한다.

## Calibration 결과 시각화

다중 장면 calibration 결과를 원본 RGB, 초기 mechanical prior 투영, 보정된 extrinsic 투영으로 확인할 수 있다.

```bash
docker compose exec -T dev /workspace-build/render_calibration_visualization \
  --dataset-root /datasets/stanford2d3ds/area_1 \
  --result-json /workspace/generated/calibration_core_multi_validation/calibration_result.json \
  --output /workspace/generated/calibration_core_multi_validation/visualization
```

생성 파일:

- `original_rgb.png`
- `initial_pointcloud_overlay.png`
- `calibrated_pointcloud_overlay.png`
- `calibration_comparison.png`
- `visualization_summary.json`
- `pointcloud_lidar.ply` / `pointcloud_lidar.obj` (lidar_scan 좌표계)
- `pointcloud_calibrated_camera.ply` / `pointcloud_calibrated_camera.obj` (camera_optical 좌표계)


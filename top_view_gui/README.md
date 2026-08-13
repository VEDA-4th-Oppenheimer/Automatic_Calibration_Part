# Calibration Top-View Qt GUI

작성일: 2026-07-31
상태: Automatic / Manual 공용 구현

## 목적

Automatic Calibration의 `T_camera_lidar`와 Manual Calibration의
`T_camera_marker_board`를 동일한 transform 계약으로 읽어 camera image를 기준평면에
rectification한다. GUI는 calibration 계산에 관여하지 않으며 검증된 RT를 소비하는
후속 시각화 모듈이다.

## 중요한 좌표 조건

외부 파라미터 RT 하나만으로 실제 지면 Top-View를 정의할 수는 없다. 다음 변환이
필요하다.

```text
T_camera_plane = T_camera_child * T_child_plane
```

- Automatic: `T_camera_lidar * T_lidar_ground`
- Manual: `T_camera_marker_board * T_marker_board_ground`
- Marker board 자체 정면 보기가 목적이면 `T_marker_board_plane = Identity`

GUI에서 `Child→plane RT JSON`을 생략하면 RT child frame의 `Z=0`을 임시 평면으로
사용한다. 이 결과는 미리보기이며 측정된 지면 좌표가 아니다.

## 지원 RT JSON

다음 object key를 자동 감지하거나 GUI에서 직접 선택할 수 있다.

- `estimated`: Automatic calibration report
- `extrinsic`: Manual ChArUco pose 및 정식 extrinsic
- `manual_camera_lidar`, `automatic_camera_lidar`: 수치 비교 report
- `transform`, `manual_transform`, `automatic_transform`
- RT object 자체

Rotation은 `rotation_matrix` 또는 `quaternion_xyzw`, translation은
`translation_m`을 사용한다. Convention은 다음으로 고정한다.

```text
p_parent = R_parent_child * p_child + t_parent_child
```

## 기능

- Automatic / Manual RT 자동 판별
- Camera intrinsic JSON과 입력 영상 해상도 검증
- 선택적 intrinsic resize 보정
- 별도 child-to-ground/reference-plane transform 합성
- X/Y 범위, pixels-per-meter, grid 간격 조정
- Original camera / Top-View 동시 표시
- 마우스 위치의 기준평면 X/Y 좌표 표시
- Top-View PNG와 사용 RT, homography, 범위를 기록한 metadata JSON 저장

## Ubuntu native 빌드

```bash
./scripts/install-ubuntu-deps.sh
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

실행 파일:

```text
build/bin/calibration_top_view_gui
```

## GUI 실행

WSLg 또는 X11 display가 활성화된 터미널에서 실행한다.

```bash
build/bin/calibration_top_view_gui
```

자동 입력 경로를 지정할 수도 있다.

```bash
build/bin/calibration_top_view_gui \
  --image /path/camera.png \
  --camera /path/camera.json \
  --rt /path/automatic_result.json \
  --plane /path/T_lidar_ground.json
```

Manual marker board 평면을 그대로 펼칠 경우 `--plane`을 생략한다.

```bash
build/bin/calibration_top_view_gui \
  --image /path/marker_capture.png \
  --camera /path/intrinsic_result.json \
  --rt /path/manual_pose.json
```

## 결과 파일

GUI에서 `Save PNG + metadata`를 선택하면 다음 파일이 생성된다.

```text
top_view.png
top_view.top_view.json
```

Metadata에는 입력/평면/합성 transform, camera K, 출력 범위와
`image_from_top_view_homography`가 저장된다.

## 예제

- `examples/camera.example.json`
- `examples/automatic_rt.example.json`
- `examples/manual_rt.example.json`
- `examples/T_lidar_ground.example.json`

예제 intrinsic과 RT 값은 포맷 설명용이며 실제 카메라에 사용하면 안 된다.

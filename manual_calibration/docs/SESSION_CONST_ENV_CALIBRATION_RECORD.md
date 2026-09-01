# session-const-env 캘리브레이션 작업 기록

- 작성일: 2026-08-14
- 대상 세션: `session-const-env`
- 대상 장비: PNM-C16083RVQ CCTV + TOFSense F2P 1D pan-tilt LiDAR + Samsung Galaxy Tab S7
- 문서 목적: 현재까지 수행한 카메라/ChArUco/LiDAR 분석, 기하 보정, RT 계산 결과와 남은 검증 작업을 한 곳에서 추적
- 결과 상태: `T_camera_marker_board`는 카메라 기준 PASS, `T_camera_lidar`는 태블릿 디스플레이 기하 보정을 적용한 **예비 추정값**

## 1. 프로젝트 구조와 목표

작업은 다음 두 경로를 분리해 관리한다.

```text
develop/
├─ automatic_calibration/   # 자연 장면 + LiDAR 기반 제품 경로
└─ manual_calibration/      # ChArUco 기반 독립 기준/진단 경로
```

Manual calibration의 기본 목적은 LiDAR 없이 2D CCTV 이미지와 ChArUco만으로 다음을 산출하는 것이다.

```text
여러 장의 ChArUco image → camera intrinsic/distortion
기준 위치의 ChArUco image → T_camera_marker_board
```

다만 이번 세션에서는 같은 태블릿을 LiDAR가 평면으로 검출할 수 있으므로, 문서에 기록된 디스플레이 크기와 보드 배치를 이용해 LiDAR 쪽 보드 좌표를 추정하는 확장 경로도 수행했다.

```text
LiDAR scan → T_lidar_display_plane
display_spec + board_config + 표시 방향 → T_display_plane_marker_board
두 변환 조합 → T_lidar_marker_board
T_camera_marker_board × inverse(T_lidar_marker_board)
  → T_camera_lidar
```

이 확장 경로는 일반적인 2D marker pose 계산과 구분한다. LiDAR 평면 검출 오차가 포함되므로 독립적인 CAD/지그/측량 기준을 대체하는 최종 conformance truth로 아직 승격하지 않았다.

## 2. 사용 장비와 고정 조건

### 2.1 Camera

- 모델: `PNM-C16083RVQ`
- 이미지 해상도: `2592 × 1520`
- zoom/focus: 고정
- LDC: 해당 모델에서 사용할 수 없으며 적용하지 않음
- 카메라 내부 파라미터는 동일한 zoom/focus 프로파일의 intrinsic만 사용

### 2.2 LiDAR

- 모델: `TOFSense-F2P`
- JSON `schema_version`: `1.2`
- JSON `interface_version`: `1.0`
- `range_offset_m`: `0.084`
- scan: `101 × 400`, continuous tilt sweep
- LiDAR frame: 오른손 좌표계, `+x right`, `+y down`, `+z forward`

JSON에 기록된 점 변환식은 다음을 그대로 사용한다.

```text
r = distance_m + range_offset_m
x = r * cos(tilt) * sin(pan)
y = -r * sin(tilt)
z = r * cos(tilt) * cos(pan)
```

### 2.3 Tablet/Display

생성된 보드는 Galaxy Tab S7의 활성 화면에 네이티브 픽셀 크기로 표시한다.

- 공식 대각선: `278.1 mm`
- landscape 활성 화면: `235.828328 × 147.392705 mm`
- native resolution: `2560 × 1600`
- ChArUco board: `7 × 5` squares
- board size: `167.659202 × 119.756573 mm`
- board는 활성 디스플레이 중앙에 배치
- 사진의 태블릿은 portrait 방향
- 원본 landscape ChArUco 이미지가 디스플레이 기준 **시계방향 90°** 회전된 상태

이 회전은 카메라의 ChArUco 검출에 매번 입력하는 값이 아니다. marker ID, 코너 순서와 `board_config`가 카메라 기준 보드 방향을 결정한다. 회전값은 LiDAR 디스플레이 평면 좌표를 보드 좌표로 변환할 때 사용한다.

기준 명세:

- [display_spec.json](../output/tablet-board/display_spec.json)
- [board_config.json](../output/tablet-board/board_config.json)

## 3. 입력 데이터

### 3.1 Camera image

```text
data/real_calibration/session-const-env/auto_data/aruco_marker/20260814-110828-CH1.jpg
```

이미지에서 태블릿과 ChArUco board가 보이며, 보드가 portrait 화면에 세로로 표시되어 있다.

### 3.2 LiDAR JSON

```text
/mnt/c/Users/3-16/Downloads/calib-20260814-110829_sweep-000001_pan_tilt_lidar.json
```

이 파일은 LiDAR 점과 pan/tilt/range 메타데이터를 제공하지만, ChArUco ID, board origin, board-to-display 회전 필드는 제공하지 않는다. 따라서 LiDAR 쪽 보드 좌표는 점군과 별도 display geometry를 조합해 추정했다.

## 4. 수행 완료 내용

### 4.1 ChArUco board 생성

Galaxy Tab S7용 `2560 × 1600` display canvas와 실제 meter 단위 board 설정을 생성했다. 화면에는 검증용 100 mm ruler도 포함한다. 실제 촬영에서는 태블릿 자동 회전/확대/crop/status bar가 없어야 하며, 현재 계산은 board가 중앙에 1:1로 표시되었다는 전제를 사용한다.

### 4.2 Camera intrinsic calibration

고정 zoom/focus 프로파일에서 PASS 이미지 18장을 사용한 clean intrinsic 결과를 확인했다.

```text
profile_id: charuco-pass-clean18-20260814
resolution: 2592 × 1520
RMS: 0.647 px

K =
[[2033.901952,    0.000000, 1337.029701],
 [   0.000000, 2037.779638,  745.370056],
 [   0.000000,    0.000000,    1.000000]]

distortion =
[-0.565317439, 0.344593856, -0.003914537,
  0.000818275, -0.108094125]
```

원본 intrinsic 파일:

- [camera_intrinsic.json](../output/session-const-env/intrinsic/camera_intrinsic.json)

### 4.3 Camera–marker board pose

ChArUco image에서 다음을 검출했다.

- marker: `17`
- ChArUco corners: `24`
- reprojection RMSE: `0.335255 px`
- maximum reprojection error: `0.573315 px`
- status: `PASS`

사용한 변환 방향:

```text
p_camera = R_camera_marker_board * p_marker_board
         + t_camera_marker_board
```

현재 결과:

```text
R_camera_marker_board =
[[-0.059942918, -0.996896946, -0.051022791],
 [ 0.881443672, -0.028873465, -0.471405745],
 [ 0.468469743, -0.073231153,  0.880439264]]

t_camera_marker_board =
[0.163765177, -0.034758982, 0.962049769] m
```

결과 파일:

- [T_camera_marker_board_110828.json](../output/session-const-env/lidar-tablet-reference/T_camera_marker_board_110828.json)
- [marker_pose_overlay_110828.png](../output/session-const-env/lidar-tablet-reference/marker_pose_overlay_110828.png)

### 4.4 LiDAR 점군에서 태블릿 디스플레이 후보 검출

LiDAR JSON을 ICD 식으로 3D 점으로 복원하고, 다음 ROI/필터로 태블릿 평면 후보를 선택했다.

```text
grid row:    16–33
grid column: 216–228
range:       0.7–1.1 m
spread:      <= 50 mm
selected:    213 points
```

검출된 평면 후보:

```text
T_lidar_display_plane translation
[-0.274441083, 0.322395024, -0.763614783] m

observed robust extent: [0.151830738, 0.233563918] m
expected portrait display: [0.147392705, 0.235828328] m
mean absolute plane residual: 0.009081525 m
maximum absolute plane residual: 0.074652560 m
```

후보 점군과 전체 스캔 시각화:

- [tablet_candidate_points.ply](../output/session-const-env/lidar-tablet-reference/tablet_candidate_points.ply)
- [scan_with_tablet_candidate.ply](../output/session-const-env/lidar-tablet-reference/scan_with_tablet_candidate.ply)
- [tablet_candidate_report.json](../output/session-const-env/lidar-tablet-reference/tablet_candidate_report.json)
- [tablet_candidate_report.md](../output/session-const-env/lidar-tablet-reference/tablet_candidate_report.md)

현재 평면 좌표계는 다음과 같이 정의했다.

```text
x = display horizontal right (u)
y = display vertical down (v)
z = x cross y (sensor에서 멀어지는 후보 법선)
```

LiDAR 평면 자체에는 marker texture가 없으므로, 평면만 피팅하면 보드 `+X/+Y`와 원점을 직접 알 수 없다. 이 문제를 다음 display geometry 보정으로 해결했다.

### 4.5 Display plane → marker board 보정

원본 board 좌표는 `x=가로 긴 축`, `y=세로 짧은 축`이며, 디스플레이에서 시계방향 90° 회전되었다. 따라서 보드 좌표를 display plane 좌표로 바꾸는 회전은 다음과 같다.

```text
R_display_plane_marker_board =
[[ 0, -1, 0],
 [ 1,  0, 0],
 [ 0,  0, 1]]
```

보드가 디스플레이 중앙에 있고 board origin이 OpenCV ChArUco 좌상단 기준이라는 전제에서:

```text
t_display_plane_marker_board =
[+0.059878286, -0.083829601, 0] m
```

보드가 중앙 표시된 경우 portrait display 여백은 다음과 같다.

```text
좌/우 각: 13.818 mm
상/하 각: 34.085 mm
```

## 5. 계산된 RT

다음 식으로 두 pose를 조합했다.

```text
T_lidar_marker_board
    = T_lidar_display_plane * T_display_plane_marker_board

T_camera_lidar
    = T_camera_marker_board * inverse(T_lidar_marker_board)
```

### 5.1 `T_lidar_marker_board`

```text
R_lidar_marker_board =
[[-0.086063593,  0.994666943, -0.056839513],
 [ 0.996028662,  0.084595143, -0.027759086],
 [-0.022802699, -0.059002830, -0.997997346]]

t_lidar_marker_board =
[-0.326785359, 0.233832926, -0.758170253] m
```

파일:

- [T_lidar_marker_board_110828.json](../output/session-const-env/lidar-tablet-reference/T_lidar_marker_board_110828.json)
- [T_lidar_marker_board_110828.reference.json](../output/session-const-env/lidar-tablet-reference/T_lidar_marker_board_110828.reference.json) (strict `reference_transform.schema.json` shape)

### 5.2 `T_camera_lidar`

변환 방향은 다음과 같다.

```text
p_camera = R_camera_lidar * p_lidar + t_camera_lidar
```

```text
R_camera_lidar =
[[-0.983521425, -0.142621159,  0.111107212],
 [-0.077785218,  0.888586398,  0.452066005],
 [-0.163202535,  0.435974102, -0.885037578]]

t_camera_lidar =
[-0.040047519, 0.074784187, 0.135763305] m

quaternion_xyzw =
[-0.056854541, 0.969167865, 0.229072831, 0.070759091]
```

파일:

- [T_camera_lidar_110828.json](../output/session-const-env/lidar-tablet-reference/T_camera_lidar_110828.json)
- [tablet_marker_rt_report_110828.md](../output/session-const-env/lidar-tablet-reference/tablet_marker_rt_report_110828.md)

계산된 카메라 중심을 LiDAR frame으로 역변환하면:

```text
C_camera_in_lidar =
[-0.011413573, -0.131353120, 0.090797807] m
```

## 6. 현재 판정

| 항목 | 상태 | 근거 |
|---|---|---|
| ChArUco marker 검출 | PASS | 17 markers / 24 corners |
| Camera intrinsic | PASS | clean18, RMS 0.647 px |
| `T_camera_marker_board` | PASS | reprojection RMSE 0.335 px |
| LiDAR 태블릿 평면 위치 | 후보 검출 | 213 points, display extent와 근접 |
| `T_lidar_marker_board` | 추정 | display geometry 보정으로 생성 |
| `T_camera_lidar` | 예비 추정 | `ESTIMATED_GEOMETRY_CORRECTED` |
| 최종 conformance reference | 미완료 | LiDAR screen edge/독립 기준 미검증 |

이 결과는 수학적으로 RT를 산출할 수 있다는 것을 확인한 값이다. 그러나 LiDAR 후보 ROI의 최대 평면 잔차가 `74.65 mm`이고, 기존에 측정한 camera–LiDAR 장착 중심값과 계산된 camera center가 약 `125 mm` 차이 나므로 최종 정답으로 사용하면 안 된다.

## 7. 남은 작업

### 필수 검증

1. 현재 ROI에서 옷걸이/태블릿 외곽/배경 점을 분리하고 실제 활성 화면의 네 변을 다시 피팅한다.
2. 화면 실제 폭을 자로 측정해 `display_spec.json`의 `display_width_m`을 검증한다.
3. camera image와 LiDAR scan이 같은 태블릿 pose에서 수집되었는지 timestamp와 촬영 기록을 확인한다.
4. portrait + clockwise 90° 변환과 보드 origin convention을 board asset으로 재확인한다.
5. 동일 장면을 3~5회 반복 스캔해 `T_camera_lidar`의 평균/표준편차를 계산한다.
6. camera center의 rough mount measurement와 screen plane fit 결과가 일치하는지 확인한다.

### 구현 상태

- ChArUco intrinsic/marker pose CLI는 구현되어 있다.
- 이번 태블릿 display-plane 보정은 입력 보고서와 고정된 geometry contract를 조합해 결과 JSON/MD를 생성한 단계다.
- `estimate_lidar_display_plane` 또는 `estimate_tablet_marker_pose` 형태의 재사용 CLI와 자동 screen-edge fit은 아직 구현되지 않았다.

### 최종 conformance 기준으로 올리기 위한 조건

- screen edge 또는 LiDAR-visible rigid target을 이용한 독립 `T_lidar_marker_board`
- plane/edge fit RMS 및 inlier/outlier 기준 기록
- 반복 측정 표준편차와 translation/rotation uncertainty 기록
- `reference_transform.schema.json` 검증
- 자동 Calibration Core 결과와 회전 오차(deg), 이동 오차(mm), reprojection/overlay를 동일한 frame convention으로 비교

## 8. 재현·사용 시 주의사항

- `T_camera_lidar_110828.json`의 parent는 `camera_optical`, child는 `lidar_scan`이다.
- 반대 방향이 필요하면 회전행렬 전치와 역변환 translation을 사용한다.
- LiDAR `range_offset_m=0.084`를 중복 적용하지 않는다.
- zoom/focus가 바뀐 intrinsic을 섞지 않는다.
- LDC가 없는 PNM-C16083RVQ에 임의의 LDC 보정을 적용하지 않는다.
- board display가 회전/크롭/확대되면 `T_display_plane_marker_board`를 다시 계산한다.
- 현재 결과를 자동 calibration 결과에서 역산해 독립 기준으로 만들지 않는다.

## 9. 2026-08-20 다중 marker profile 재사용 기록

`auto_data/aruco_marker`는 단일 reference image가 아니라 다음 다중 관측 세트다.

| 회차/채널 | 이미지 수 |
|---|---:|
| 2026-08-14 / CH1 | 78 |
| 2026-08-19 / CH3 | 64 |
| 2026-08-19 / CH4 | 41 |

CH1은 이 중 18개 유효 프레임으로 `charuco-pass-clean18-20260814` K/D profile을
생성했다. 해상도는 `2592×1520`, calibration RMS는 `0.647 px`, 상태는 `PASS`다.

카메라를 모니터암 또는 강체 모듈 단위로 이동·회전했더라도 zoom/focus, 해상도,
ROI/crop, LDC 및 flip/mirror/rotation 설정이 변하지 않았다면 K/D는 재사용하고,
변경된 장면에 대한 외부 자세 `R,t`만 새로 추정한다. 카메라와 LiDAR가 같은 강체로
함께 이동했다면 장면 기준 pose는 변하지만 센서 간 `T_camera_lidar`는 원칙적으로
유지된다. 반대로 zoom/focus/ROI/LDC/영상 변환 중 하나라도 바뀌면 새 intrinsic
profile로 분리한다.

2026-08-19 CH1 reference image(`20260819-200910-CH1.jpg`)는 단일 이미지이므로 새
K/D를 추정하는 입력이 아니라 기존 CH1 profile을 적용한 RT/hold-out 검증 자료로
분류한다. CH3와 CH4는 서로 다른 광학 채널이므로 CH1 profile을 재사용하지 않고
채널별 profile을 사용한다.

기존 profile의 board 설정은 태블릿 보드 기준(`marker_length=17.963 mm`,
`square_length=23.951 mm`)이다. 실제 출력 A4 보드가 `marker=20 mm`, `square=27 mm`인
경우에는 해당 치수를 반영한 board config로 별도 재검증한다. K/D의 재사용 여부와
보드 pose/절대 RT 계산의 유효성은 분리해서 판정한다.

## 10. 수정 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-14 | session-const-env camera intrinsic, ChArUco pose, LiDAR tablet plane candidate, display geometry correction, `T_lidar_marker_board`, `T_camera_lidar` 계산 결과를 기록 |
| 2026-08-14 | 태블릿 전체 크기와 active display 크기 차이, portrait/clockwise 90° 보정, 예비 추정값과 최종 검증 한계를 명시 |
| 2026-08-20 | `aruco_marker` 다중 이미지 세트, CH1 K/D profile 재사용 조건, 카메라 이동 시 K/D·RT 구분, A4 보드 치수 검증 주의사항을 추가 |

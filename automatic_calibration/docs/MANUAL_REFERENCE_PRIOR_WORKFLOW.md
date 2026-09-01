# 다른 Manual 세션 결과를 Automatic Calibration의 Reference/Prior로 사용하는 방법

- 작성일: 2026-08-14
- 최종 수정일: 2026-08-20
- 적용 범위: `automatic_calibration` Calibration Core와 `manual_calibration` 결과의 연계
- 대상 장비: PNM-C16083RVQ + TOFSense F2P pan-tilt LiDAR
- 목적: 다른 세션에서 취득한 Manual 결과를 Automatic Calibration의 초기값·탐색범위·독립 검증값으로 안전하게 활용
- 현재 상태: Manual intrinsic 입력과 진단용 Manual RT reference 입력 adapter를 구현하고,
  `repeat_test_sample` 검증 실행을 완료했다. Manual ChArUco `K+D`는 제품 projection의
  고정 profile이고, Manual RT는 prior/reference/hold-out 중 하나로만 사용한다.

현재 MVP 정책은 [`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)와 동일하다.
K+RT 공동 추정은 연구·진단용 flag로만 보존하며 제품 승인 경로에서는 비활성화한다.

## 1. 핵심 원칙

Manual 결과는 Automatic Calibration에 다음 세 가지 역할 중 하나로만 등록한다.

| 역할 | 의미 | Automatic 최적화에 사용 | 최종 PASS 근거 |
|---|---|---:|---:|
| `prior` | 초기 자세·intrinsic의 참고값 | 가능 | 불가 |
| `reference` | 독립 측정된 비교 기준 | 직접 최적화에는 사용하지 않음 | 가능(불확실도 포함) |
| `holdout` | 최적화에 사용하지 않은 독립 검증 세션 | 사용하지 않음 | 평가용 |

Manual 값으로 Automatic 결과를 강제로 덮어쓰거나, Automatic 결과에서 역산한 값을 Manual reference로 등록하지 않는다. 두 결과가 같은 데이터를 공유하면 “학습/최적화에 사용한 값으로 다시 정답을 만드는” 순환 검증이 된다.

현재 권장 흐름은 다음과 같다.

Manual session A/B/C
  ├─ 같은 카메라 profile인지 검증
  ├─ T_camera_marker_board + T_lidar_marker_board가 모두 있는지 확인
  ├─ T_camera_lidar_manual을 계산
  ├─ quality/uncertainty에 따라 prior 또는 reference로 등록
  └─ Automatic session D의 초기값/탐색범위 또는 hold-out 평가에 사용

## 2. Manual 결과의 종류와 사용 가능 범위

### 2.1 `T_camera_marker_board`만 있는 세션

ChArUco 이미지에서 얻는 다음 값은 카메라 기준 보드 pose다.

```text
p_camera = T_camera_marker_board * p_marker_board
```

이 값만으로는 `T_camera_lidar`를 만들 수 없다. 같은 board 좌표계에 대한 LiDAR pose가 별도로 필요하다.

필요한 두 값:

```text
T_camera_marker_board
T_lidar_marker_board
```

### 2.2 독립 측정 `T_lidar_marker_board`가 있는 세션

CAD, rigid jig, survey, LiDAR-visible rigid target 등으로 `T_lidar_marker_board`를 독립 취득한 경우 다음 RT를 계산할 수 있다.

```text
T_camera_lidar_manual
    = T_camera_marker_board
    * inverse(T_lidar_marker_board)
```

이 값은 Automatic 결과와 같은 convention을 사용한다.

```text
p_camera = R_camera_lidar * p_lidar + t_camera_lidar
```

독립 측정값은 `reference` 또는 `holdout`으로 사용할 수 있다.

### 2.3 태블릿 display geometry 보정값

LiDAR가 태블릿 활성 화면 평면을 검출하고, `display_spec`·`board_config`·보드 중앙 배치·표시 회전으로 `T_lidar_marker_board`를 추정한 경우다.

현재 `session-const-env` 결과가 여기에 해당한다.

- status: `ESTIMATED_GEOMETRY_CORRECTED`
- 용도: Automatic 초기값/진단용 `prior`
- 용도 제한: 독립 conformance `reference`로 승격하지 않음

화면 edge fit, 반복 스캔, camera/LiDAR 장착 측정이 검증되기 전에는 이 값을 최종 정답으로 사용하지 않는다.

## 3. 세션 간 전달 가능성 확인

다른 Manual 세션의 값을 사용하기 전에 다음 조건을 확인한다.

### 3.1 반드시 일치해야 하는 항목

- 카메라 모델과 channel
- image resolution
- zoom/focus 상태
- LDC/WDR/IR 및 image flip/mirror/rotate 상태
- 카메라 optical frame과 LiDAR frame convention
- LiDAR JSON `schema_version`, `range_offset_m`, `frame.convention`
- camera/LiDAR가 강체로 고정된 설치 상태

PNM-C16083RVQ에서 zoom/focus가 바뀌면 새로운 camera profile로 취급한다. LDC 상태가 바뀌거나 모르면 기존 intrinsic을 그대로 재사용하지 않는다.

### 3.2 세션별로 달라도 되는 항목

센서가 움직이지 않고 target만 이동한 경우에는 Manual target의 위치·거리·각도가 달라도 된다. `T_camera_lidar`는 센서 간 강체 관계이므로 target pose에 대해 불변이어야 한다.

반대로 camera 또는 LiDAR를 재설치했거나 pan-tilt 기구 기준축이 바뀌었다면 이전 RT를 그대로 prior로 사용하면 안 된다. 이 경우 이전 결과는 참고 방향 또는 초기 global search 후보로만 사용하고, translation은 기계 측정값으로 다시 구성한다.

## 4. 좌표계와 파일 계약

모든 Manual/Automatic transform은 다음 방향으로 정규화한다.

```text
p_parent = R_parent_child * p_child + t_parent_child
```

Automatic 최종 RT:

```text
parent_frame: camera_optical
child_frame:  lidar_scan
```

LiDAR 입력 좌표는 다음 계약을 그대로 사용한다.

```text
+x right, +y down, +z forward
r = distance_m + range_offset_m
x = r*cos(tilt)*sin(pan)
y = -r*sin(tilt)
z = r*cos(tilt)*cos(pan)
```

반대 방향 변환이 필요한 경우:

```text
R_lidar_camera = R_camera_lidar^T
t_lidar_camera = -R_camera_lidar^T * t_camera_lidar
```

단위는 meter, 내부 각도는 radian, 보고서 회전 차이는 degree로 고정한다. m/mm, parent/child, camera optical frame을 혼동하면 수치가 그럴듯해도 잘못된 prior가 된다.

## 5. 다른 Manual 세션을 prior로 등록하는 절차

### 단계 A. Manual 세션 manifest 작성

각 세션은 수치 파일만 복사하지 말고 촬영 조건과 품질을 함께 등록한다.

```json
{
  "schema_version": "1.0",
  "session_id": "manual-session-003",
  "role": "prior",
  "source_type": "independent_measured|geometry_corrected|camera_only",
  "camera": {
    "model": "PNM-C16083RVQ",
    "channel": 1,
    "resolution": [2592, 1520],
    "profile_id": "channel1-fixed-zoom-focus-v1",
    "zoom_focus_locked": true,
    "ldc_enabled": false,
    "image_transform": {"flip": false, "mirror": false, "rotate_deg": 0}
  },
  "lidar": {
    "model": "TOFSense-F2P",
    "frame": "lidar_scan",
    "range_offset_m": 0.084,
    "schema_version": "1.2"
  },
  "manual_outputs": {
    "camera_intrinsic": "manual_calibration/output/session-003/intrinsic/camera_intrinsic.json",
    "t_camera_marker_board": "manual_calibration/output/session-003/pose/T_camera_marker_board.json",
    "t_lidar_marker_board": "manual_calibration/output/session-003/reference/T_lidar_marker_board.reference.json",
    "t_camera_lidar": "manual_calibration/output/session-003/reference/T_camera_lidar.json"
  },
  "quality": {
    "manual_status": "PASS",
    "reprojection_rmse_px": 0.5,
    "lidar_reference_rms_m": 0.01,
    "uncertainty_rotation_deg": 3.0,
    "uncertainty_translation_m": 0.02
  }
}
```

`source_type=geometry_corrected`인 현재 session-const-env 결과는 `prior`로만 등록한다. `camera_only`는 RT prior로 등록할 수 없고 camera intrinsic/보드 검출 품질 참고로만 사용한다.

### 단계 B. Manual RT sanity check

다음 항목을 자동 입력 전에 확인한다.

- rotation matrix가 정규직교이고 determinant가 `+1`인지
- translation이 meter인지
- `T_camera_marker_board`와 `T_lidar_marker_board`의 child frame이 동일한 `marker_board`인지
- board origin, 표시 회전, display center 보정이 기록되어 있는지
- camera center `C_lidar = -R_camera_lidar^T t_camera_lidar`가 설치 측정값과 크게 모순되지 않는지
- 같은 세션의 image와 LiDAR scan이 동일 pose/time인지

수치가 위 조건을 통과하지 못하면 prior 후보에서 제외하고 원인을 기록한다.

### 단계 C. prior 초기화

Core API는 이미 `mechanical_prior`를 입력으로 받는다.

```cpp
auto result = auto_calib::calibrateExtrinsicMultiScene(
    observations,
    manual_t_camera_lidar_as_prior,
    config);
```

Manual RT를 하나의 정답으로 고정하지 않고 다음 후보를 함께 평가한다.

1. Manual RT prior
2. 독립 mechanical installation prior
3. 이전 세션의 robust mean prior
4. 축 부호/180° ambiguity를 고려한 보조 후보

Manual prior 하나만 사용하면 Manual 오차가 Automatic 최적화 결과에 그대로 잠길 수 있다. multi-start ambiguity, NID/edge 개선률, projected ratio와 prior update gate를 계속 적용한다.

### 단계 D. 탐색 범위 설정

아래 값은 초기 운영 예시이며, 반복 Manual 세션의 분산으로 재산정한다.

| Manual source | 권장 초기 회전 범위 | 권장 초기 이동 범위 | 용도 |
|---|---:|---:|---|
| 독립 jig/CAD/survey | ±2–5° | ±10–30 mm | 강한 prior 또는 reference |
| tablet geometry corrected | ±10–20° | ±100–150 mm | 약한 prior/진단 |
| camera-only | 사용하지 않음 | 사용하지 않음 | intrinsic/검출 품질만 |

범위를 좁히더라도 최소 한 번은 global yaw multi-start 또는 대체 방향 후보를 남긴다. 그래야 잘못된 Manual orientation을 Automatic이 조용히 답습하지 않는다.

## 6. Manual intrinsic을 Automatic에 사용하는 방법

Manual intrinsic은 같은 camera profile의 Automatic 경로에서 제품용으로 **K+D를 고정**하는
용도로 사용한다. K를 다시 최적화하는 방식은 현재 MVP에서 보류한다.

### 6.1 제품 경로: K+D 고정

동일한 resolution/zoom/focus/LDC profile이면 Manual `K`와 distortion을 Automatic의
camera model로 전달하고, raw 영상이면 동일 `K+D`로 undistort한다. 이후 구조가 다른
scene에서 `R,t`만 최적화한다.

```text
manual K+D → fixed camera profile
manual distortion/LDC 상태 → projection contract
multi-scene optimization → candidate R,t
quality gate → candidate/active R,t
```

### 6.2 연구·진단 경로: K 공동 추정 보류

RT만 비교할 때는 Manual K+D를 고정한다. 제조사 FOV K와의 비교는 profile 민감도를 보는
진단 실험으로만 수행하며, 제품 승인 결과와 섞지 않는다. K+RT 공동 추정은 현재 테스트와
승인 범위에서 보류한다.

현재 `run_real_calibration`은 다음 입력을 지원한다.

~~~
--manual-intrinsic-json PATH
--allow-intrinsic-refinement true|false
--image-distortion-state raw|rectified|unknown
--manual-reference-json PATH
~~~

Manual intrinsic을 지정하면 K+D를 고정하고, raw 영상이며 distortion 계수가
있으면 동일 K+D로 undistort한다. `--allow-intrinsic-refinement true`는 연구·진단용으로만
남겨 둔다. `--manual-reference-json`은 자동 결과와의 회전/이동 차이를
진단 JSON에 기록하지만, 해당 값을 정답으로 사용하거나 최적화에 강제하지 않는다.

## 7. 여러 Manual 세션을 하나의 prior로 통합

같은 설치 상태의 Manual 세션이 여러 개면 각 `T_camera_lidar_manual`을 바로 평균내지 말고 다음 순서를 사용한다.

1. frame/profile/quality가 다른 세션 제거
2. 회전 geodesic distance와 translation norm으로 outlier 제거
3. rotation은 quaternion/SE(3) robust mean, translation은 median 또는 weighted mean 계산
4. 반복 표준편차를 `uncertainty_rotation_deg`, `uncertainty_translation_m`으로 저장
5. robust mean은 `prior`, 최상 품질 독립 세션은 `reference` 또는 `holdout`으로 별도 보관

Manual 세션이 서로 다른 설치 상태에서 취득되었다면 하나의 평균 RT로 합치지 않는다. 설치 상태별 profile을 분리한다.

## 8. Automatic 실행과 평가 분리

### 8.1 최적화에 사용할 데이터

Manual target image를 Automatic NID/edge observation에 포함할 수는 있지만, 그 image의 Manual RT를 같은 실행의 정답으로 다시 사용하지 않는다. 가장 안전한 구성은 다음이다.

```text
Manual session A/B  → prior 생성
Automatic scenes C/D/E/F/G → multi-scene 최적화
Manual session H 또는 독립 jig → holdout 평가
```

### 8.2 비교 지표

Automatic 결과와 Manual reference를 다음으로 비교한다.

- rotation geodesic difference [deg]
- translation norm difference [m]
- camera center difference [m]
- image reprojection/edge overlay
- projected ratio
- final NID와 NID improvement
- multi-start objective margin
- 세션 간 RT 표준편차

`PASS` threshold는 프로젝트가 승인한 값을 사용한다. 기존 문서의 예시 threshold는 회전 `3°`, 이동 `0.05 m`이며, geometry-corrected tablet 값에는 이 threshold를 곧바로 적용하지 않는다.

## 9. 현재 구현과 필요한 추가 작업

### 현재 가능한 것

- `--manual-intrinsic-json`으로 Manual ChArUco `K+D` profile을 읽고 제품 projection에서 고정
- `calibrateExtrinsic`/`calibrateExtrinsicMultiScene` API에 Manual `T_camera_lidar`를 `mechanical_prior`로 전달
- 기존 mechanical center/direction prior와 Manual prior를 별도 후보로 비교
- Automatic 결과에서 `estimated_t_camera_lidar`, `candidate_t_camera_lidar`, mechanical prior 반환을 분리 기록
- staged 제품 경로는 basin 실패 시 다른 PASS 후보를 fallback으로 승격하지 않으며, prior는
  fail-safe 시각화/반환값으로만 보존한다.
- Manual reference와 Automatic 결과의 pose error 계산 API 사용

### 현재 불가능하거나 별도 구현이 필요한 것

- 여러 Manual 세션의 `T_camera_lidar`를 자동으로 통합해 제품 reference로 승인
- 여러 Manual 세션의 SE(3) robust mean/covariance 자동 계산
- session manifest/schema 자동 검증
- Manual source quality에 따른 prior sigma/weight 자동 설정

### 권장 구현 순서

1. `manual_reference_prior.schema.json`과 session manifest 자동 검증
2. 여러 Manual RT의 SE(3) robust mean/covariance와 source uncertainty 계산
3. Manual RT를 초기 seed로 사용하되 mechanical/global seed를 함께 유지
4. 세션별 prior source/uncertainty를 `calibration_result.json`에 저장
5. holdout reference 평가와 Jenkins conformance job 연결

## 10. 현재 session-const-env 예시

현재 태블릿 세션의 예비 결과는 다음 파일에 있다.

- [T_camera_lidar_110828.json](../../manual_calibration/output/session-const-env/lidar-tablet-reference/T_camera_lidar_110828.json)
- [T_lidar_marker_board_110828.reference.json](../../manual_calibration/output/session-const-env/lidar-tablet-reference/T_lidar_marker_board_110828.reference.json)
- [세션 작업 기록](../../manual_calibration/docs/SESSION_CONST_ENV_CALIBRATION_RECORD.md)

이 결과의 status는 `ESTIMATED_GEOMETRY_CORRECTED`다. 따라서 다른 Automatic 세션에서 사용할 때:

```text
허용: 약한 initial prior, 방향 후보 sanity check, 진단 overlay
금지: 최종 RT 강제 주입, Automatic 결과의 ground truth 대체, holdout과 동일 데이터 재사용
```

## 11. 수정 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-14 | 다른 Manual 세션의 intrinsic/marker pose/RT를 Automatic prior·reference·holdout으로 분리해 사용하는 절차, frame 계약, 품질 등급, 현재 CLI 제한과 구현 계획을 최초 작성 |
| 2026-08-20 | Manual ChArUco K+D를 제품 고정 profile로 확정하고 K+RT 공동 추정을 연구·진단으로 보류 |

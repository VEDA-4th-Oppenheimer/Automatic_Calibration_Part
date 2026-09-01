# Hanwha Vision PNM-C16083RVQ 카메라 사양 및 캘리브레이션 적용 문서

- 작성일: 2026-08-11
- 대상: PNM-C16083RVQ 4MP × 4CH
- 용도: LiDAR–camera 자동 외부 파라미터(RT) 캘리브레이션
- 상태: 제품 K/D profile 정책 반영 (2026-08-20)

현재 MVP 운용 기준은 [`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)다.
이 문서는 카메라 사양과 profile 관리 규칙을 설명하며, 제품 실행 중 ChArUco를 요구하거나
K를 자동으로 다시 추정한다는 의미가 아니다.

## 1. 공식 자료

- [Hanwha Vision 제품 사양 페이지](https://www.hanwhavision.com/global/products/product-details/PNM-C16083RVQ)
- [PNM-C16083RVQ 공식 데이터시트(PDF)](https://device.report/m/5983807791b63d45f669731dd3cdb40a477677561d2da25cfed3487381802cf9)

제품 페이지와 데이터시트의 표기 버전에 따라 일부 프레임레이트·세부 수치가 달라질 수 있으므로, 실제 운용값은 카메라에서 저장한 이미지와 현재 채널 설정을 우선한다.

## 2. 캘리브레이션에 직접 영향을 주는 사양

| 항목 | 공식 사양 | 프로젝트 적용 |
|---|---|---|
| 센서 | 1/2.8″ CMOS × 4채널 | CH1~CH4를 서로 다른 카메라로 취급 |
| 최대 영상 해상도 | 2592×1520 | 내부 파라미터의 기준 해상도 |
| 초점거리 | 3.3~5.7 mm, 1.7× motorized varifocal | 줌 위치가 바뀌면 채널별 K가 바뀜 |
| 최대 조리개 | F1.5(Wide)~F1.9(Tele) | 노출/초점 변화의 참고값 |
| 화각 | H 100°~53°, V 54°~30°, D 125°~61° | 줌 위치별 초기 fx/fy 추정에 사용 |
| 최소 물체 거리 | 1.3 m | 현재 천장–센서 거리(약 0.6 m)에서는 공식 최소거리보다 가까움. 초점/왜곡 검증 필요 |
| 초점 제어 | Simple focus | 실제 촬영 시 focus 상태를 고정해야 함 |
| 렌즈 | Fixed iris | 조리개 고정, 노출 변화는 다른 설정의 영향 |
| 왜곡 보정 | LDC 지원 | 카메라 내부 보정 활성 여부를 촬영 세션에 기록 |
| 영상 회전 | Flip, Mirror, Hallway view 90°/270°, 채널별 | 촬영 후 이미지의 방향을 데이터셋 메타데이터에 기록 |

## 3. 내부 파라미터(K) 관련 판단

카메라 제조사가 공개한 제품 사양에는 다음 실제 내부 파라미터가 제공되지 않는다.

- `fx`, `fy`
- `cx`, `cy`
- 방사 왜곡 `k1, k2, k3`
- 접선 왜곡 `p1, p2`
- 각 채널의 실제 줌 위치 및 focus 위치
- 카메라 내부 LDC가 적용된 영상인지 여부

따라서 제품 사양의 초점거리와 화각만으로 최종 K를 확정하면 안 된다. 특히 이 모델은 motorized varifocal이므로 CH1~CH4의 줌 위치가 다르면 각 채널의 K도 달라진다.

### 3.1 초기값 추정식

이미지 크기를 `W=2592`, `H=1520`, 수평·수직 화각을 각각 `FOVx`, `FOVy`라고 하면 초기값은 다음과 같이 계산할 수 있다.

```text
fx0 = W / (2 * tan(FOVx / 2))
fy0 = H / (2 * tan(FOVy / 2))
cx0 = W / 2
cy0 = H / 2
```

이 값은 자동 캘리브레이션의 초기화 또는 sanity check용이며, 최종 결과로 사용하지 않는다. Wide와 Tele 화각 범위가 넓으므로 줌 상태를 모르면 `fx0`, `fy0`의 범위가 매우 커진다.

### 3.2 자동 캘리브레이션에서의 내부 파라미터 처리

본 프로젝트의 기본 자동 캘리브레이션 입력에는 체커보드·Charuco 촬영을 요구하지 않는다. 외부 파라미터 `R,t`를 자동으로 찾는 것이 목표이므로, 별도의 수동 카메라 캘리브레이션을 필수 단계로 두면 프로젝트 목적과 맞지 않는다.

자동 파이프라인에서는 다음 우선순위를 사용한다.

1. 동일 채널·해상도·zoom·focus profile의 Manual ChArUco `K + distortion`을 등록한다.
2. 고정된 설치 환경과 동일한 촬영 세션에서는 해당 `K,D`를 그대로 유지한다.
3. Automatic은 구조가 다른 관측에서 `R,t`만 추정한다.
4. 제조사 화각 기반 K는 profile이 없을 때의 초기화·민감도 진단으로만 기록한다.
5. 줌·focus·LDC·영상 방향이 변경되면 새 profile/session으로 취급하며 촬영 중에는 값을 고정한다.

체커보드/ChArUco 기반 내부 캘리브레이션은 **제품 실행 중 작업자에게 요구하는 단계가
아니라 profile을 사전에 만들거나 갱신하는 절차**다. 동일한 zoom/focus 프로파일의
기준 K를 확보하면 automatic 실행기의 `--manual-intrinsic-json`으로 입력하고 K와 D를
고정한다. `--allow-intrinsic-refinement true`는 연구·진단용으로만 남겨 두며 제품 승인
경로에서는 사용하지 않는다.

### 3.3 다중 ChArUco 이미지와 카메라 이동의 해석

내부 파라미터는 단일 이미지가 아니라 같은 광학 프로파일에서 촬영한 여러 ChArUco
관측으로 산출한다. 카메라가 설치 위치에서 이동하거나 회전해도 렌즈의 zoom/focus,
해상도, ROI/crop, LDC/영상 변환이 바뀌지 않았다면 K와 D는 재사용할 수 있다. 이때
바뀌는 값은 카메라의 장면·LiDAR에 대한 외부 자세 `R,t`이며, 카메라와 LiDAR가 하나의
강체 모듈로 함께 이동했다면 두 센서 사이의 `T_camera_lidar` 자체는 원칙적으로
변하지 않는다.

2026-08-20에 확인한 `auto_data/aruco_marker`의 다중 이미지 수는 다음과 같다.

| 회차/채널 | 이미지 수 | 용도 |
|---|---:|---|
| 2026-08-14 / CH1 | 78 | CH1 K,D profile 후보 및 검증 |
| 2026-08-19 / CH3 | 64 | CH3 별도 K,D profile 후보 |
| 2026-08-19 / CH4 | 41 | CH4 별도 K,D profile 후보 |

CH1은 이 중 18개 유효 프레임으로 `charuco-pass-clean18-20260814` profile을
생성했다. 해상도는 `2592×1520`, calibration RMS는 `0.647 px`, 상태는 `PASS`다.
따라서 8월 19일 CH1 이미지에 줌/초점/ROI/LDC/해상도 및 영상 변환이 동일하다는 확인이
있으면 이 profile을 K,D 입력으로 고정하고, 8월 19일 이미지에서는 RT 재현성만 검증한다.

다음 조건이 하나라도 달라지면 기존 profile을 자동으로 재사용하지 않고 새 profile로
분리한다.

- zoom 또는 focus 위치
- 출력 해상도, ROI, crop, digital zoom
- LDC/rectified 상태
- flip/mirror/rotation 등 픽셀 변환
- 렌즈·카메라 채널 자체

조명·노출 변화는 K,D를 직접 바꾸지는 않지만 marker 검출 품질에 영향을 주므로,
profile 적용 후 hold-out 이미지의 코너 수와 재투영 오차를 확인한다. 카메라 위치가
바뀐 이미지 한 장은 K,D를 새로 추정하는 자료가 아니라 해당 profile을 사용한
`T_camera_board` 또는 RT hold-out 검증 자료로 분류한다.

> 보드의 실제 치수는 pose 계산에 직접 영향을 준다. 기존 CH1 profile의 board 설정은
> `marker_length=17.963 mm`, `square_length=23.951 mm`인 태블릿 보드 기준이다.
> 이후 출력한 A4 보드가 `marker=20 mm`, `square=27 mm`라면 실제 보드 설정으로
> 별도 profile을 생성하거나, 최소한 board config를 일치시킨 재검증을 수행해야 한다.
> 보드 치수 불일치를 숨긴 채 `T_camera_board`를 절대 기준으로 사용하지 않는다.

이 카메라에서 LDC(Lens Distortion Correction)를 사용할 수 없거나 상태가
`false`라면, raw 영상에는 manual intrinsic의 왜곡 계수를 적용해 undistort한 뒤
LiDAR 투영을 수행한다. LDC가 `true`인 rectified 출력이라면 중복 undistort하지
않지만, rectified 픽셀을 투영할 K는 여전히 필요하다. 즉 LDC는 내부 파라미터를
대체하지 않는다. 중요한 계약은 LDC UI 표시보다 실제 입력 픽셀이 raw인지 rectified인지다.
OpenSDK/capture 경로가 raw stream임을 보장하면 LDC 지원 여부가 `unknown`이어도
`--image-distortion-state raw`와 해당 Manual K+D를 사용하고 warning을 남긴다. 실제
raw/rectified 상태를 알 수 없을 때만 `--image-distortion-state unknown`으로 실행하고
결과를 진단용으로 취급한다.

자동 파이프라인 입력 메타데이터 예시는 다음과 같다.

```json
{
  "camera_model": "PNM-C16083RVQ",
  "channel": 1,
  "image_size": [2592, 1520],
  "intrinsics_source": "manual_charuco_profile",
  "intrinsics": {"fx": 2033.90, "fy": 2037.78, "cx": 1337.03, "cy": 745.37},
  "distortion_model": "opencv_radtan|none",
  "distortion": "manual camera_intrinsic.json when available",
  "image_transform": {"flip": false, "mirror": false, "rotate_deg": 0},
  "ldc_enabled": null,
  "extrinsics_solver": "automatic_lidar_camera_alignment",
  "calibration_status": "extrinsics_only"
}
```

## 4. 현재 프로젝트에 대한 결론

- 기존 실측 데이터의 `2592×1520` 프로파일은 카메라 최대 해상도와 일치한다.
- 자동 캘리브레이션은 제품 실행 중 별도 체커보드 촬영 없이 외부 파라미터 `R,t`를 찾는다. 단, 실행 전에 같은 카메라 profile에서 만든 Manual ChArUco `K+D`를 입력으로 고정한다. Manual 측정 자체는 profile 생성·검증 경로로 관리한다.
- 3.3~5.7 mm varifocal 특성 때문에 제품 사양만으로는 정확한 투영이 불가능하다.
- 현재 천장 측정 환경의 약 0.6 m 거리는 데이터시트의 최소 물체 거리 1.3 m보다 짧다. 초점 흐림과 왜곡 증가 가능성을 별도로 확인해야 한다.
- 카메라에서 LDC가 켜진 영상과 꺼진 영상은 서로 다른 카메라 모델로 취급해야 한다. LiDAR 투영 전에 보정 영상을 사용할지 원본 영상을 사용할지 결정하고 전 세션에서 일관되게 유지한다. LDC가 켜져도 K는 필요하며, LDC가 없으면 manual K와 왜곡 계수 또는 동일 K 기반 undistort 경로를 사용한다.
- 이전에 확인한 카메라 API는 회전·반전·LDC 기능의 지원 여부는 노출하지만, 현재 활성값과 실제 zoom/focus 값을 모두 제공하지 않았다. 따라서 촬영 시점의 profile metadata를 기록하고, 실행 중 내부 캘리브레이션을 강제하지 않는다.

### 현재 확인된 채널 영상 변환 설정

2026-08-11 카메라 웹 UI에서 확인한 설정:

| 항목 | 현재 값 |
|---|---|
| 상하 반전(Flip) | 사용 안 함 |
| 좌우 반전(Mirror) | 사용 안 함 |
| 복도뷰(Hallway view) | 0° |

현재 캘리브레이션 입력 영상의 방향 변환은 `rotate_deg=0`, `flip_vertical=false`, `mirror_horizontal=false`로 기록한다. 기능 지원 여부와 현재 활성값은 서로 다른 정보이므로, 설정 변경 시 촬영 세션 메타데이터도 함께 갱신한다.

## 5. 담당자 간 전달 사항

센서/액추에이터 담당자는 다음 값을 데이터 수집 JSON에 포함해야 한다.

- 카메라 채널 번호
- 이미지 해상도와 프레임레이트
- 촬영 시각 및 LiDAR sweep 시각
- 채널별 image rotation/flip/mirror 상태
- LDC 활성 상태
- zoom/focus를 변경했는지 여부

이 값들이 누락되면 동일한 공간에서 촬영했어도 영상 좌표계가 달라져 자동 캘리브레이션 결과를 비교할 수 없다.

## 6. 수정 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-11 | 카메라 사양 및 K/D profile 운용 규칙 작성 |
| 2026-08-20 | 다중 ChArUco 이미지 수, CH1 `PASS` profile 재사용 조건, 카메라 이동과 K/D·RT의 구분, A4 보드 치수 검증 주의사항 추가 |

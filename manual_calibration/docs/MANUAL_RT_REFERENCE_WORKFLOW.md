# Manual RT 기준값 취득 및 Automatic Calibration 비교 인수인계

- 작성일: 2026-08-14
- 대상: `automatic_calibration`과 `manual_calibration`을 이어서 작업하는 개발 세션
- 목적: 카메라–LiDAR 외부 파라미터(`RT`)를 독립적인 기준 측정으로 구하고, 자동 캘리브레이션 결과를 정량·시각적으로 비교
- 상태: 카메라–보드 포즈는 PASS, 태블릿 디스플레이 기하 보정 기반 LiDAR–보드 포즈와 예비 RT까지 산출. 최종 conformance 기준 RT는 screen-edge/독립 기준 검증 전

> Automatic MVP는 이 문서에서 생성한 동일 profile의 Manual ChArUco `K+D`를 고정하고
> `R,t`만 추정한다. K+RT 공동 추정은 현재 제품 경로에서 보류한다. 상세 정책은
> [`automatic_calibration/docs/PRODUCT_CALIBRATION_POLICY.md`](../../automatic_calibration/docs/PRODUCT_CALIBRATION_POLICY.md)를 따른다.

> 최신 session-const-env 작업 결과는 [`SESSION_CONST_ENV_CALIBRATION_RECORD.md`](SESSION_CONST_ENV_CALIBRATION_RECORD.md)에 기록되어 있다. 이 문서는 일반적인 독립 기준 절차를 설명하고, 최신 세션 문서는 태블릿 디스플레이 평면을 이용한 보정 경로와 실제 수치를 기록한다.

## 1. 먼저 알아야 할 결론

일반적인 2D 수동 캘리브레이션에서 이미지의 ChArUco/마커 검출만으로 얻는 값은 **카메라 내부 파라미터와 카메라–마커 보드 포즈**까지다. 이미지 자체만으로는 LiDAR와 보드 사이의 포즈를 알 수 없으므로, 이 값만으로 `T_camera_lidar`를 만들 수 없다.

다만 이번 session-const-env처럼 ChArUco가 표시된 태블릿의 활성 디스플레이 평면을 LiDAR에서 검출하고, 디스플레이 크기·보드 크기·중앙 배치·표시 회전(세로/시계방향 90°)을 알고 있으면 다음 **geometry-corrected 추정 경로**를 사용할 수 있다.

~~~
T_lidar_display_plane = LiDAR 점군에서 검출한 화면 평면 포즈
T_display_plane_marker_board = display_spec와 board_config로 계산한 강체 보정
T_lidar_marker_board = T_lidar_display_plane * T_display_plane_marker_board
T_camera_lidar = T_camera_marker_board * inverse(T_lidar_marker_board)
~~~

이 경로는 수학적으로 RT를 만들 수 있지만, 평면 ROI에 옷걸이/외곽/배경 점이 섞이거나 화면 테두리가 독립적으로 확인되지 않으면 최종 conformance 기준값으로 승격하지 않는다. 최종 기준값은 여전히 CAD/강체 지그/측량 또는 LiDAR-visible rigid target으로 독립 취득하는 방식을 우선한다.

완전한 수동 RT를 만들려면 같은 보드 좌표계에 대해 다음 두 포즈가 모두 필요하다.

~~~
T_camera_board  = 카메라에서 본 보드 포즈
T_lidar_board   = LiDAR에서 본 보드 포즈
~~~

그 뒤 다음 식으로 외부 파라미터를 계산한다.

~~~
p_camera = R_camera_lidar * p_lidar + t_camera_lidar

R_camera_lidar = R_camera_board * R_lidar_board^T
t_camera_lidar = t_camera_board - R_camera_lidar * t_lidar_board
~~~

즉, `T_lidar_board`를 자동 캘리브레이션 결과로 대신 넣으면 기준값과 평가값이 서로 원인이 되어 **순환 검증**이 된다. `T_lidar_board`는 CAD, 지그 측정, 측량 또는 LiDAR에서 관측 가능한 기준 형상으로 독립적으로 취득해야 한다.

## 2. 좌표계와 변환 계약

### 2.1 변환 방향

모든 파일과 코드에서 아래 방향을 사용한다.

~~~
p_parent = R_parent_child * p_child + t_parent_child
~~~

따라서 `T_camera_lidar`는 `p_camera = R_camera_lidar p_lidar + t_camera_lidar`이며, 역변환은 다음과 같다.

~~~
R_lidar_camera = R_camera_lidar^T
t_lidar_camera = -R_camera_lidar^T * t_camera_lidar
~~~

### 2.2 LiDAR JSON 계약

현재 LiDAR JSON의 프레임과 좌표 생성식은 다음과 같다.

~~~
+x: right, +y: down, +z: forward
pan 증가: 장치 기준 시계 방향(Top-view)
tilt_rad: 0 = 수평, 음수 = 아래 방향, 범위 -pi/2 .. 0

x = r * cos(tilt) * sin(pan)
y = -r * sin(tilt)
z = r * cos(tilt) * cos(pan)
~~~

`mechanism.tilt_zero = nadir`는 모터의 기구 홈/영점 메타데이터다. 좌표식에 사용하는 `measurements[].tilt_rad`의 0점(수평)과 혼동하지 않는다. `range_offset_m`은 JSON의 ICD에 명시된 값만 사용한다.

### 2.3 현재 설치의 물리적 사전 점검값

고정 환경에서 확인한 값은 다음과 같다.

~~~
지면 -> LiDAR -> 카메라 순서
LiDAR 회전축 교점 -> 카메라 optical center 수평 오프셋: +59.28 mm
LiDAR 회전축 교점 -> 카메라 optical center 수직 오프셋: -81.05 mm
전후 깊이 오프셋: 0 mm 가정(최종 조립 실측 시 갱신)
~~~

과거 천장 기준 높이 `LiDAR 635.00 mm`, `카메라 551.95 mm`를 단순 차감한
83.05 mm는 서로 다른 외형 기준점을 센서 중심으로 간주한 계산이므로 현재 center
prior에 사용하지 않는다. 모델링에서 동일 기준점인 LiDAR 회전축 교점과 카메라 optical
center 사이를 측정한 81.05 mm를 사용한다.

LiDAR 프레임(+Y가 아래)에서 카메라 중심의 측정값은 대략 다음과 같이 sanity check에 사용한다.

~~~
C_L = (+0.05928, -0.08105, 0.0) m
C_L = -R_camera_lidar^T * t_camera_lidar
~~~

이 값은 완전한 RT를 결정하는 입력이 아니라, 계산된 RT가 설치 측정과 크게 모순되는지 확인하는 용도다. 실제 중심축 오프셋, 발광점과 기구축 차이, 카메라 optical center 위치가 다르면 측정값을 갱신한다.

## 3. 현재 파일과 확인된 상태

### 3.1 이미 산출된 값

~~~
manual_calibration/output/session-002/intrinsic/camera_intrinsic.json
manual_calibration/output/session-002/pose/marker_pose_result.json
~~~

현재 session-002의 주요 지표:

~~~
카메라 내부 캘리브레이션: 18개 채택 프레임 / 37개 입력
intrinsic RMS: 약 1.176 px
카메라-보드 pose reprojection RMSE: 약 0.284 px
~~~

`marker_pose_result.json`의 pose는 `T_camera_board`(`T_camera_marker_board`)다. 해당 translation은 보드 좌표계 원점의 카메라 좌표이며, LiDAR 좌표계의 원점/축과 직접 비교할 수 없다.

### 3.2 현재 누락된 값

일반적인 독립 기준 경로에서는 다음 파일이 필요하다.

~~~
manual_calibration/data/<session>/T_lidar_marker_board.json
~~~

스키마 예시는 다음 파일을 참고한다.

~~~
manual_calibration/examples/T_lidar_marker_board.example.json
manual_calibration/schemas/reference_transform.schema.json
~~~

예시 파일을 복사해 숫자만 임의로 채우면 안 된다. 최종 기준 경로에서는 실제 보드 지그/CAD/측량 결과로 `R_lidar_board`, `t_lidar_board`를 채워야 한다.

이번 session-const-env에는 디스플레이 geometry 보정으로 생성한 예비 결과가 있다.

~~~
manual_calibration/output/session-const-env/lidar-tablet-reference/T_lidar_marker_board_110828.json
manual_calibration/output/session-const-env/lidar-tablet-reference/T_camera_lidar_110828.json
~~~

두 파일의 status는 `ESTIMATED_GEOMETRY_CORRECTED`이며, 독립 conformance reference인 `PASS`와 구분한다. 계산 근거와 남은 검증은 [`SESSION_CONST_ENV_CALIBRATION_RECORD.md`](SESSION_CONST_ENV_CALIBRATION_RECORD.md)를 따른다.

## 4. 권장 기준 타깃 설계

### 4.1 1순위: 카메라와 LiDAR가 동시에 관측하는 강체 타깃

현재의 평면 ChArUco 보드는 카메라에는 좋지만, 1D LiDAR 점군에는 검은색/흰색 패턴이 보이지 않는다. 따라서 다음과 같이 구성한다.

1. 큰 ChArUco 보드(카메라용)를 rigid plate에 고정한다.
2. 보드 좌표계에 대해 위치가 측정된 3D 형상 4개 이상을 추가한다.
   - 반사 구(sphere), 원통 봉, 작은 기둥 또는 L자/삼면체 코너를 권장
   - LiDAR 거리 점에서 중심/평면/교차선을 안정적으로 피팅할 수 있어야 함
3. 3D 형상의 중심 좌표를 `board_config` 또는 별도 `target_geometry.json`에 mm/m 단위로 명시한다.
4. 카메라에서 ChArUco로 `T_camera_board`를 구하고, LiDAR에서 3D 형상 대응점으로 `T_lidar_board`를 구한다.

현재 tablet board 설정은 7 x 5 squares, square length 약 0.02395 m로 약 168 x 120 mm 규모다. 1D LiDAR가 충분한 점을 얻기에는 작을 수 있으므로 실제 기준 측정용으로는 약 400~600 mm급 판과 돌출 형상을 권장한다. 설치 거리와 LiDAR 측정 밀도에 따라 크기를 조정한다.

### 4.2 2순위: CAD/강체 지그/측량으로 `T_lidar_board` 제공

가장 재현성이 높은 방법은 카메라–보드를 같은 강체 지그에 고정하고, LiDAR 원점/축과 보드 좌표계의 관계를 CAD 또는 3점 이상 실측으로 정의하는 것이다. 이 경우 LiDAR가 보드 패턴을 직접 보지 못해도 된다. 측정값에는 다음을 함께 기록한다.

~~~
측정 방법, 기준점 정의, 축 방향, 단위, 각 축의 불확실도, 지그 재조립 오차
~~~

### 4.3 3순위: 대응점 PnP를 이용한 빠른 진단

LiDAR 점과 이미지 픽셀의 대응을 사람이 4~6개 이상 선택해 `solvePnPRansac`으로 RT를 구할 수 있다. 단, 모든 점이 한 평면에만 있으면 깊이/방향이 불안정하므로 서로 다른 높이의 돌출점 또는 비평면 형상을 사용한다. 이 방법은 원인 진단용으로 유용하지만, 최종 기준값은 반복 측정 가능한 강체 타깃/CAD 방식을 우선한다.

## 5. 데이터 수집 절차

1. 카메라 채널 해상도, 줌, 포커스, 회전/상하·좌우 반전, LDC/WDR/IR 상태를 고정하고 기록한다. LDC 상태가 모르면 `unknown`으로 기록하고, 원본 이미지와 왜곡 보정 이미지를 구분한다.
2. 카메라와 LiDAR를 실제 자동 캘리브레이션 설치 상태로 강체 고정한다. LiDAR 회전축 교점과 카메라 optical center의 81.05 mm 수직/59.28 mm 수평 오프셋은 측정 기록과 비교한다.
3. 보드와 LiDAR가 동시에 같은 장면을 보도록 정지시킨다. 이미지와 JSON/PCD가 같은 수집 시점인지 파일명만 믿지 말고 timestamp/메타데이터로 확인한다.
4. 한 자세만 찍지 말고 가능하면 보드 자세 3~5개를 수집한다. 보드가 카메라 화면의 중앙/좌우/상하에 분산되면 카메라 pose 반복성을 확인하기 좋다.
5. 각 보드 자세에 대해 카메라 이미지, LiDAR JSON/PCD, 타깃 형상 파일, 설정 스냅샷을 한 디렉터리에 둔다.

권장 디렉터리 예:

~~~
manual_calibration/data/session-manual-rt/
  images/
  lidar/
  target_geometry.json
  T_lidar_marker_board.json
  capture_manifest.json
~~~

## 6. 실제 계산 순서

### 단계 A. 카메라 내부 파라미터

체커보드/ChArUco로 카메라 내부 파라미터를 구한다. 이것은 외부 RT를 수동으로 측정하는 것과 별개의 단계다. 카메라가 줌/포커스/LDC에 따라 광학 상태가 바뀌면 프로파일을 새로 만들어야 한다.

기존 실행 예:

~~~
cd develop
build/bin/calibrate_camera_markers \
  --board manual_calibration/output/tablet-board/board_config.json \
  --images-dir manual_calibration/data/session-002/intrinsic_images \
  --output-dir manual_calibration/output/session-002/intrinsic \
  --minimum-frames 10 \
  --maximum-rms-px 2.0 \
  --camera-model PNM-C16083RVQ \
  --profile-id channel1-fixed-zoom-focus-v1
~~~

### 단계 B. 카메라–보드 포즈

~~~
build/bin/estimate_marker_pose \
  --board manual_calibration/output/tablet-board/board_config.json \
  --camera manual_calibration/output/session-002/intrinsic/camera_intrinsic.json \
  --image "manual_calibration/data/session-002/intrinsic_images/<image>.png" \
  --output-dir manual_calibration/output/session-002/pose
~~~

검사할 값:

~~~
reprojection RMSE, inlier corner 수, 보드가 화면 밖으로 잘리지 않았는지,
rotation/translation의 단위와 frame 이름
~~~

### 단계 C. LiDAR–보드 포즈

다음 중 하나로 독립 취득한다.

~~~
선호: CAD/강체 지그/측량 -> T_lidar_marker_board.json
대안: LiDAR 점에서 돌출 타깃을 피팅 -> known board 3D 좌표와 SVD/Umeyama
진단: 대응점 수동 선택 -> PnP/RANSAC
~~~

피팅 결과에는 대응점 수, RMS 거리 오차, 사용한 LiDAR 원본 파일, outlier 제거 기준을 함께 저장한다. `range_offset_m` 적용 여부도 반드시 기록한다.

### 단계 D. `T_camera_lidar` 계산

`T_lidar_marker_board.json`이 준비되면 기존 비교 도구를 실행한다.

~~~
build/bin/compare_marker_to_automatic \
  --manual-pose manual_calibration/output/session-002/pose/marker_pose_result.json \
  --board-in-lidar manual_calibration/data/session-manual-rt/T_lidar_marker_board.json \
  --automatic automatic_calibration/generated/<run>/calibration_result.json \
  --output-dir manual_calibration/output/session-manual-rt/comparison
~~~

이 도구는 `T_camera_board * inverse(T_lidar_board)`를 계산한다. 구현은 다음 파일을 참고한다.

~~~
manual_calibration/apps/compare_marker_to_automatic.cpp
~~~

## 7. 투영과 검증 기준

수동 RT가 생성되면 숫자 비교만 하지 말고 LiDAR 점을 카메라 이미지에 투영한다.

### 7.1 왜곡 모델을 반드시 통일

현재 manual session-002의 내부 파라미터는 대략 `fx=2784`, `fy=2797`이고 왜곡 계수가 큰 반면, 과거 automatic 진단은 제조사 FOV 기반 K(`fx` 약 1843, `fy` 약 2164)와 무왜곡 투영을 사용했다. 현재 제품 경로는 manual session과 동일 profile의 `K+D`를 고정하고 raw image를 undistort한 뒤 비교한다. 서로 다른 profile을 raw image 위에 직접 비교하면 RT가 틀린 것처럼 보일 수 있다.

다음 중 하나를 선택해 양쪽을 동일하게 만든다.

~~~
A. 원본 이미지에 distortion-aware projection을 적용
B. 카메라 내부 파라미터로 이미지를 undistort한 뒤 양쪽 모두 pinhole projection
~~~

줌/포커스/LDC 상태가 다른 이미지와 K를 섞지 않는다. FOV 스펙값은 초기값/대략값이지, 같은 광학 프로파일의 실측 K를 자동으로 대체하지 않는다.

### 7.2 반드시 확인할 항목

~~~
벽–바닥, 벽–책상 경계가 실제 엣지와 겹치는가
점들이 한쪽 벽/바닥으로 몰리는가
카메라 중심 sanity check가 수직 81.05 mm/수평 59.28 mm 측정과 맞는가
평면 하나만 맞고 yaw/roll이 자유롭게 남아 있지 않은가
타깃을 다른 위치에서 찍어도 RT가 반복되는가
~~~

평면 하나만으로 맞춘 결과는 in-plane yaw와 원점이 모호할 수 있다. 따라서 “점수가 통과했다”보다 투영 overlay와 반복성, 타깃 피팅 잔차를 우선 판단한다.

## 8. 자동 결과와 비교할 때의 원칙

1. 수동 RT는 독립 기준값이며, 자동 최적화의 후보 선택을 위한 prior로 사용할 수는 있어도 정답을 자동 결과에서 만들지 않는다.
2. 자동 결과의 `R,t`를 수동 결과와 비교하기 전에 parent/child frame, 행렬 방향, 축 부호, m/mm 단위를 정규화한다.
3. LiDAR JSON 좌표를 다른 식(`x=cos* sin`, `y=-sin`, `z=cos*cos`)으로 다시 변환하지 않는다. ICD 식을 한 곳에서만 적용한다.
4. 카메라 image flip/rotation, LDC, distortion을 변환 행렬의 일부로 임의 흡수하지 않는다. 이미지 전처리 상태와 3D frame 변환을 별도로 기록한다.
5. 사람이 없는 고정 환경의 score만으로 성공을 선언하지 않는다. 벽·바닥처럼 대칭적인 장면은 mirror/yaw 오류도 높은 점수를 낼 수 있으므로 비대칭 타깃/구조물을 포함한다.

## 9. 다음 세션에서 구현할 작업 목록

### 필수

- [ ] 실제 LiDAR–보드 기준 측정 또는 LiDAR-visible 3D 타깃 설계
- [ ] `T_lidar_marker_board.json` 생성 및 `reference_transform.schema.json` 검증
- [ ] `T_camera_board`와 조합해 `manual_camera_lidar.json` 생성
- [ ] 수동 RT 투영 overlay(PNG)와 residual/통계(JSON) 생성
- [ ] distortion-aware 또는 undistorted 공통 투영 경로 선택
- [ ] 변환 방향/역변환/단위에 대한 합성 단위 테스트 추가

### 권장

- [ ] `estimate_lidar_board_pose` CLI 추가: LiDAR JSON/PCD + target geometry -> `T_lidar_marker_board.json`
- [ ] 3D 타깃 피팅 RMS, inlier 수, 불확실도 저장
- [ ] 자동 RT와 수동 RT의 회전 오차(deg), 이동 오차(mm), 투영 RMSE를 한 파일로 요약
- [ ] target pose 3~5개에 대한 RT 반복성 리포트 작성
- [ ] LDC 상태가 `unknown`인 경우 raw/undistorted 두 경로를 모두 출력해 민감도 확인

## 10. 실패 시 판단 순서

~~~
1. 파일 timestamp와 좌표 단위 확인
2. JSON ICD 식과 tilt/pan 부호 확인
3. T_camera_board reprojection RMSE 확인
4. T_lidar_board의 독립성·타깃 피팅 잔차 확인
5. R,t 행렬 방향과 frame 이름 확인
6. K/distortion/LDC/flip 상태 통일
7. 카메라 중심 수직 81.05 mm/수평 59.28 mm sanity check
8. manual RT overlay에서 벽·책상·바닥 경계 확인
9. 그 뒤에만 automatic coarse/fine search와 score를 비교
~~~

## 관련 문서

- [manual_calibration/README.md](../README.md)
- [manual_calibration/docs/ARCHITECTURE.md](ARCHITECTURE.md)
- [manual_calibration/docs/USAGE.md](USAGE.md)
- [manual_calibration/docs/COMPARISON_PROTOCOL.md](COMPARISON_PROTOCOL.md)
- [manual_calibration/schemas/reference_transform.schema.json](../schemas/reference_transform.schema.json)
- [automatic_calibration/docs/CURRENT_PROGRESS_AND_STATUS.md](../../automatic_calibration/docs/CURRENT_PROGRESS_AND_STATUS.md)

## 수정 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-14 | 수동 RT 산출에 필요한 `T_lidar_board` 누락 상태, 좌표계 계약, 타깃 설계, 데이터 수집·계산·검증 절차, 다음 세션 구현 목록을 최초 작성 |
| 2026-08-14 | session-const-env의 태블릿 display plane + portrait/CW90 geometry 보정 경로와 예비 `T_camera_lidar` 결과를 추가. 최종 독립 reference와 구분 |

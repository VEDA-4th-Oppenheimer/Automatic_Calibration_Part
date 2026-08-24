# repeat_test_sample Manual Intrinsic Automatic Calibration 검증 기록

- 작성일: 2026-08-14
- 대상 카메라: Hanwha Vision PNM-C16083RVQ, CH1
- 대상 LiDAR: TOFSense F2P 1D pan-tilt LiDAR
- 목적: Manual ChArUco intrinsic/distortion을 Automatic Calibration 입력으로 사용하고, 고정 환경 반복 데이터에서 자동 RT 탐색·투영·품질 게이트를 검증
- 실행 환경: Ubuntu 기반 Docker 개발 컨테이너
- 최신 결과: 내부 gate PASS (v11, 제품 RT/conformance 승인은 보류)

> 2026-08-20 정책 정정: 제품 경로는 Manual ChArUco `K+D`를 고정하고 `R,t`만 추정한다.
> 아래의 intrinsic refinement 옵션 설명은 연구·진단 경로를 위한 historical 기록이며
> 제품 승인 실행에는 사용하지 않는다. 상세 기준은 [`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)다.

2026-08-18에 추가된 `repeat_test_sample/20260818` CH1 고정환경 회차는 이 문서의
2026-08-14 historical 10-set 실행과 분리한다. 해당 회차는 동일 installation epoch의
3개 RT 추정용 입력과 1개 고정 RT hold-out으로 구성되며, 파일 목록·운영자 확인·pairing
해석은 [`CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md`](CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md)를
기준으로 한다.

## 1. 입력 데이터

입력 디렉터리:

~~~
data/real_calibration/session-const-env/repeat_test_sample
~~~

확인된 파일:

~~~
camera images: 10개, 2592 x 1520
LiDAR JSON: 10개, schema_version 1.2, 101 x 400
range_offset_m: JSON header의 0.084 m 사용
frame: +x right, +y down, +z forward
tilt: 0=horizon, negative=down
~~~

현재 실행기는 입력 디렉터리의 image와 JSON을 각각 파일명 사전순으로 정렬한 뒤
같은 index끼리 pair한다. 이번 실행의 pairing은 자동으로 처리되었지만, 이미지 파일
시각과 JSON sweep 시각이 정확히 대응한다는 manifest는 제공되지 않았다.

확인된 이름 범위는 다음과 같다.

~~~
image: 20260813 205942~205952, 20260813 233952~234013
scan : 20260813 205921~222748
~~~

따라서 고정 환경이라도 조명/야간 그룹과 LiDAR sweep의 실제 대응 관계는 별도 확인이
필요하다. 이 실행 결과는 pairing_basis=lexicographic filename order supplied by
operator인 진단 결과로 취급한다.

## 2. Manual intrinsic 및 LDC 정책

사용한 파일:

~~~
manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json
~~~

Manual calibration 결과:

~~~
resolution: 2592 x 1520
fx: 2033.901952
fy: 2037.779638
cx: 1337.029701
cy: 745.370056
distortion_model: opencv_radtan
distortion: [-0.565317439, 0.344593856, -0.003914537,
              0.000818275, -0.108094125]
calibration RMS: 0.647 px
~~~

이번 실행 옵션:

~~~
--manual-intrinsic-json manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json
--image-distortion-state raw
--ldc-enabled false
--allow-intrinsic-refinement false
~~~

즉, Manual K를 고정하고 raw image에 동일 K와 distortion으로 cv::undistort를
적용한 뒤 pinhole projection을 수행했다. Automatic 최적화가 K를 다시 바꾸지
않았으므로 이번 결과에서 K와 RT의 결합으로 인한 모호성은 줄였다.

LDC에 대한 결론:

- LDC가 없거나 false인 경우: raw image에 manual distortion 보정을 적용해야 한다.
- LDC가 true인 경우: 카메라/서버가 이미 rectified image를 출력한다고 보고 중복 보정하지 않는다.
- LDC가 true여도 rectified image의 픽셀 좌표를 LiDAR에 투영할 K는 필요하다.
- LDC는 내부 파라미터를 없애는 기능이 아니며, 출력 profile의 K가 필요하다.
- 상태가 unknown이면 보정하지 않고 진단용으로만 결과를 사용한다.

## 3. 실행 명령

컨테이너 내부에서 다음 명령을 실행했다.

~~~
/workspace-build/bin/run_real_calibration \
  --input-dir data/real_calibration/session-const-env/repeat_test_sample \
  --output automatic_calibration/generated/repeat_test_sample_20260814_manual_intrinsic \
  --camera-channel 1 \
  --ldc-enabled false \
  --zoom-focus-locked true \
  --manual-intrinsic-json manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --manual-reference-json manual_calibration/output/session-const-env/lidar-tablet-reference/T_camera_lidar_110828.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 \
  --camera-center-y-m -0.08305 \
  --camera-center-z-m 0 \
  --expected-forward-x 0 --expected-forward-y 0 --expected-forward-z -1 \
  --expected-down-x 0 --expected-down-y 1 --expected-down-z 0 \
  --direction-prior-weight 0 \
  --down-min-deg 0 --down-max-deg 90 --down-step-deg 15 \
  --yaw-step-deg 15 \
  --legacy-range-offset-m 0.084
~~~

탐색 후보는 yaw 24개(15° 간격)와 downward direction 7개(0~90°, 15° 간격)이며,
각 후보에 대해 Ceres refinement와 인접 후보 basin 보정을 수행했다.

## 4. 실행 결과

결과 파일:

~~~
automatic_calibration/generated/repeat_test_sample_20260814_manual_intrinsic/calibration_result.json
~~~

핵심 결과:

| 항목 | 값 |
|---|---:|
| status | FAIL |
| reason_code | NID_IMPROVEMENT_INSUFFICIENT |
| mode | real_geometry_nid_edge_multistart_manual_intrinsics_fixed |
| intrinsics_source | manual_intrinsic_fixed |
| distortion_application | manual_intrinsic_undistort |
| scene count | 10 |
| selected downward direction | 90° |
| selected optical roll | 0° |
| final composite objective | 0.753809 |
| initial composite objective | 0.857293 |
| objective improvement | 12.07% |
| initial NID | 0.981570 |
| final NID | 0.977061 |
| NID improvement | 0.459% |
| final mean edge distance | 38.35 px |
| projected ratio | 53.57% |
| multistart margin | 0.05465 |

목적함수 전체는 감소했지만 NID 개선이 설정된 최소 1%보다 작아
NID_IMPROVEMENT_INSUFFICIENT으로 후보를 활성화하지 않았다. 따라서
matching_scene_*.png와 colorized PLY/OBJ는 검토용 거절 후보이며 최종 RT로
사용하면 안 된다.

## 5. Manual RT reference와의 비교

비교에 사용한 파일:

~~~
manual_calibration/output/session-const-env/lidar-tablet-reference/T_camera_lidar_110828.json
~~~

이 파일의 상태는 ESTIMATED_GEOMETRY_CORRECTED이며 태블릿 디스플레이 평면과
보드 geometry를 조합한 예비값이다. 독립 CAD/강체 지그/측량 reference가 아니므로
conformance ground truth로 승격하지 않았다.

이번 자동 실행의 거절 후보와 비교한 값:

~~~
rotation geodesic difference: 65.4376 deg
translation difference: 0.142727 m
~~~

따라서 현재 결과만으로 “자동 알고리즘이 65° 틀렸다”고 단정할 수 없다. 다음 세
가지가 동시에 가능하다.

1. 태블릿 geometry-corrected manual RT 자체의 LiDAR 평면/화면 edge 오차
2. image–scan pairing 또는 조명/야간 상태 불일치
3. 현재 targetless 목적함수가 벽·바닥의 반복 구조에서 다른 basin으로 수렴

## 6. 생성된 시각화

최종/거절 후보 투영:

~~~
automatic_calibration/generated/repeat_test_sample_20260814_manual_intrinsic/matching_scene_0.png
automatic_calibration/generated/repeat_test_sample_20260814_manual_intrinsic/matching_scene_1.png
...
automatic_calibration/generated/repeat_test_sample_20260814_manual_intrinsic/matching_scene_9.png
~~~

3D colorized 결과:

~~~
automatic_calibration/generated/repeat_test_sample_20260814_manual_intrinsic/scene_0_colorized_lidar_3d_preview.png
automatic_calibration/generated/repeat_test_sample_20260814_manual_intrinsic/scene_0_colorized_lidar_z_up_viewer_mesh.ply
automatic_calibration/generated/repeat_test_sample_20260814_manual_intrinsic/scene_0_colorized_lidar_z_up_viewer_mesh.obj
~~~

각 scene의 calibration_result.json pairs[]에 전체 PNG/PLY/OBJ 경로가 기록되어
있다. mechanical_prior_* 파일은 측정된 camera center prior를 투영한 별도 진단
결과이며, 자동 후보와 혼동하지 않는다.

## 7. 구현 변경 사항

run_real_calibration에 다음 옵션을 추가했다.

| 옵션 | 동작 |
|---|---|
| --manual-intrinsic-json PATH | Manual camera_intrinsic.json의 K/왜곡 계수 로드 |
| --image-distortion-state raw\|rectified\|unknown | undistort 적용 여부 명시 |
| --allow-intrinsic-refinement true\|false | Manual K를 고정하거나 공동 미세 조정 |
| --manual-reference-json PATH | 자동 후보와 Manual RT의 차이를 진단 JSON에 기록 |
| --minimum-structural-direction-groups 0\|1\|2 | 수평·수직 구조 방향군 최소 개수 |
| --maximum-camera-downward-deg DEG | 바닥 직하방 후보를 막는 광축 하향 한계 |

결과 JSON에는 intrinsics_source, manual_distortion_model,
distortion_application, manual_reference_comparison을 남긴다. Manual reference는
최적화에 사용하지 않고 비교용으로만 읽는다.

## 8. 다음 검증 순서

1. 실제 capture manifest를 작성해 각 image와 JSON의 대응을 명시한다. 현재
   lexicographic pairing은 smoke/diagnostic 용도다.
2. 조명 켜짐 5개와 night vision 5개를 별도 입력으로 나눠 각각 실행한다.
3. 같은 image profile에서 raw/rectified 두 경로를 별도 실행해 LDC/왜곡 민감도를
   비교한다.
4. 화면 edge를 독립 피팅한 T_lidar_marker_board 또는 CAD/지그 reference를
   확보한다.
5. manual RT reference와 자동 결과를 비교할 때 rotation 3°, translation 50 mm
   등의 승인 threshold는 독립 reference 품질이 PASS일 때만 적용한다.
6. targetless PASS gate를 완화하지 말고, 실제 overlay가 벽–책상–바닥 경계에
   겹치는지 먼저 확인한다.

## 9. 바닥 방향 고착 수정 및 조명 ON 재검증

### 9.1 원인과 코드 수정

기존 결과의 카메라 광축은 LiDAR `+Y down`과 `0.0098°` 차이로 사실상 바닥을
직하방으로 향했다. 원시 coarse 최저점은 `down=60°`였지만 인접 후보를 50% 반영한
보정 점수가 `down=90°`를 `0.00176` 차이로 선택했고, 이 coarse basin 선택이 이미
refinement를 마친 후보의 최종 순위와 gate를 덮어썼다.

다음과 같이 수정했다.

1. 수평·수직 구조 방향군이 설정 개수보다 적은 coarse 후보는 basin 계산에서 제외한다.
2. 인접 후보 보정은 유지하되 raw/neighbor 비율을 `0.8/0.2`로 변경한다.
3. basin 후보가 최종 gate를 통과하지 못하면 검증을 통과한 refined 후보를 우선한다.
4. 최종 광축이 `maximum-camera-downward-deg`를 넘는 후보는 활성화하지 않는다.
5. Manual RT를 최적화 입력으로 사용하지 않고 같은 장면의 별도 2D·3D 투영으로 저장한다.
6. 수동 reference와 mechanical prior의 3D preview가 `REJECTED RT`로 표시되던 라벨을
   각각 `MANUAL RT REF`, `INSTALLED CAMERA`로 분리한다.

모든 down 후보는 기존에도 각각 Ceres refinement를 수행하므로 별도의 top-K 최적화
프레임워크는 추가하지 않았다. 인접 basin은 제안·진단 역할을 유지하고 최종 PASS gate가
우선한다.

### 9.2 조명 ON 5세트 실행

조명 ON 이미지 5개와 정렬 순서상 앞의 JSON 5개만 임시 입력으로 분리했다. 실제 capture
ID가 없는 lexicographic pair이므로 conformance 입력이 아니라 고정장면 진단 입력이다.

공통 조건:

```text
manual intrinsic fixed + raw image undistort
camera center = (0.05928, -0.08305, 0) m
expected forward/down = (0,0,-1) / (0,1,0)
direction prior weight = 0.15
minimum structural direction groups = 2
maximum camera downward = 75°
```

| 실행 | 탐색 | 선택 자세 | 구조 방향군 | NID 개선 | edge mean | Manual RT 회전 차이 | 상태 |
|---|---|---|---:|---:|---:|---:|---|
| 기존 10세트 | yaw/down 15° | down 90°, yaw 165° | 1 | +0.459% | 38.35 px | 65.44° | FAIL |
| 조명 ON gate v1 | yaw/down 15° | down 12.75°, yaw 165° | 2 | -0.572% | 25.73 px | 15.30° | FAIL |
| 조명 ON 5° v3 | yaw/down 5° | down 20.37°, yaw 165° | 2 | -0.642% | 25.00 px | 12.31° | FAIL |
| geometry-first v4 | yaw/down 5° | down 20.49°, yaw 165° | 2 | -0.610% | 25.36 px | 11.17° | FAIL |

geometry-first v4는 `NID/edge/line=0.10/0.35/0.55`로 실행했다. 바닥 고착은 제거됐고
yaw는 Manual reference heading과 약 3° 범위까지 접근했지만, geometry NID는 여전히
악화되므로 활성 RT로 승인하지 않았다. 구조선 가중치를 높여도 overlay 개선은 제한적이었다.

최신 산출물:

```text
automatic_calibration/generated/repeat_test_sample_20260814_light_on_geometry_first_5deg_v4/
  calibration_result.json
  matching_scene_0.png
  scene_0_colorized_lidar_3d_preview.png
  manual_reference_matching_scene_0.png
  manual_reference_scene_0_colorized_lidar_3d_preview.png
  orientation_full_search.csv
  orientation_corrected_scores.csv
```

### 9.3 현재 결론

- `down=90°` 바닥 후보가 최종 표시 후보가 되는 선택 결함은 재현되지 않는다.
- Automatic과 Manual reference 모두 실제 영상 경계에 완전히 일치하지 않는다.
- Manual reference의 camera center는 실측 mechanical center와 약 0.13~0.14 m 차이가 있어
  translation ground truth로 사용할 수 없다.
- 현재 NID는 LiDAR reflectivity가 아니라 range/normal 변화량과 영상 gradient를 비교한다.
  이번 장면에서는 값이 약 0.98로 평탄해 최종 방향을 판별할 정보가 부족하다.
- 다음 구현 우선순위는 threshold 완화가 아니라 구조선 correspondence의 공간 분포와
  hold-out 재투영 검증이다. F2P `signal_strength` NMI는 거리·입사각 보정과 반복성 검증
  이후 별도 보조 채널로 추가한다.

## 10. 카메라 중심 오프셋 정정 및 재검증

### 10.1 모델링 도면과 좌표 계약

기존 v1~v4 실행에는 수평 오프셋 59.28 mm가 이미 다음 경로로 반영되어 있었다.

```text
--camera-center-x-m 0.05928
candidate t = -R * C_L
Ceres camera-center prior sigma = 5 mm
```

따라서 누락된 값은 59.28 mm가 아니다. 이전 높이 계산 `83.05 mm`를 운영자가
`81.05 mm`로 정정했으므로 현재 설치 prior를 다음과 같이 확정했다.

```text
LiDAR frame: +x right, +y down, +z forward
camera center C_L = (+0.05928, -0.08105, 0) m
center distance |C_L| = 0.100415 m
```

모델링 도면에서 카메라가 LiDAR 중심선의 오른쪽에 있으므로 X는 양수이고, 설치 설명상
카메라가 LiDAR보다 위에 있으므로 Y는 음수다. Z는 두 중심의 전후 깊이가 같다는 현재
가정이며, 최종 조립 실측에서 깊이 차이가 확인되면 `--camera-center-z-m`만 갱신한다.

### 10.2 동일 조건 A/B 실행

조명 ON 5세트와 geometry-first 가중치, 5° yaw/down grid를 그대로 유지하고 Y만
`-0.08305 m`에서 `-0.08105 m`로 변경했다.

| 항목 | v4: 83.05 mm | v5: 81.05 mm |
|---|---:|---:|
| 선택 grid | yaw 165°, down 20° | yaw 165°, down 20° |
| refinement down | 20.4896° | 20.4017° |
| final composite | 0.392894 | 0.394135 |
| NID 개선 | -0.6099% | -0.5059% |
| edge mean | 25.3615 px | 26.1217 px |
| projected ratio | 66.03% | 63.02% |
| Manual RT 회전 차이 | 11.1689° | 11.4886° |
| 상태 | FAIL | FAIL |

v5의 최적화 후 진단 camera center는
`(0.0592904, -0.0810606, 0.0000064) m`로 입력 prior를 유지했다. 중심 오프셋은
정상 반영됐지만 선택 방향과 FAIL 판정은 바뀌지 않았다. 2 mm 정정은 translation과
시차의 정밀도에는 필요하지만, 현재 남은 구조 대응·NID 관측성 문제의 원인은 아니다.

산출물:

```text
automatic_calibration/generated/
  repeat_test_sample_20260814_light_on_geometry_first_5deg_v5_offset_81p05mm/
    calibration_result.json
    matching_scene_0.png
    scene_0_colorized_lidar_3d_preview.png
```

## 11. Geometry NID·수직 구조 보강과 hold-out 재검증

### 11.1 구현 변경

2026-08-14에 반복·대칭 장면에서 geometry NID와 수직 방향의 식별력이 부족했던 경로를
다음과 같이 변경했다.

1. range discontinuity와 surface-normal change를 한 값으로 합치지 않고 별도 NID 채널로
   평가한다.
2. 영상 전체 histogram 대신 2×2 spatial NID를 사용하고, cell별 최소 투영점·entropy를
   통과한 구역만 동일 가중치로 평균한다.
3. 평면 교차선뿐 아니라 평면–미분류 geometry 경계선과 여러 scan에서 반복되는 폐색선을
   3D 구조선에 추가한다.
4. 투영 3D 선분과 2D LSD 선분은 방향·끝점·겹침 비용으로 1:1 대응시킨다.
5. 영상 소실점과 LiDAR 중력축/벽축을 맞추는 Manhattan 잔차를 별도로 추가한다. 상위
   3개만 남기던 소실점 후보는 최대 12개로 확대했다.
6. 각 장면에 `03a_manhattan_vanishing_directions.csv`를 남겨 후보 방향, 지지선 수,
   초기/최종 중력축 오차를 확인할 수 있게 했다.
7. 최종 후보를 고른 뒤 학습/hold-out 장면에 고정 자세를 다시 평가한다. 후보 재선택 뒤
   이 판정을 덮어쓰던 실행기 결함도 수정했다.
8. `signal_strength`는 거리 제곱·입사각·range-bin median/MAD 보정을 적용한 NMI와 Manual
   RT ±1/3/5/10° perturbation CSV를 만들지만 conformance 전에는 가중치 0을 유지한다.

### 11.2 CH1 조명 ON 비교 결과

입력은 `repeat_test_sample`의 앞 5쌍이며 앞 4쌍을 학습, 마지막 1쌍을 hold-out으로
사용했다. 고정 intrinsic/왜곡 보정과 camera center
`(+0.05928,-0.08105,0) m`는 동일하다.

| 실행 | down/yaw 범위 | 선택 grid / refinement down | 수직 오차 | 학습 통과 | hold-out | 최종 상태 |
|---|---|---|---:|---:|---:|---|
| v6 | down 0~75°, yaw 360°, 5° | 10° / 약 11.0° | 약 67.8° | 0/4 | 실패 | FAIL |
| v8 | down 5~20°, yaw 155~185°, 5° | 15° / 14.37° | 10.23~20.40° | 1/4 | 실패 | `PER_SCENE_VALIDATION_FAILED` |
| v9 | down 25~40°, yaw 155~185°, 5° | 30° / 27.81° | **3.86~7.88°** | **4/4** | 0/1 | `HOLDOUT_VALIDATION_FAILED` |

v8 진단에서 장면 0/2/3/4의 공통 수직 소실점은 camera direction의 `z≈0.553`이었다.
이는 `asin(0.553)≈33.6°` 하향 광축을 지지한다. 5~20° 제한 탐색은 이 증거에 도달할 수
없었고, 25~40° 구간을 연 v9에서 수직 오차가 한 자리 각도로 감소했다. 따라서 이전
실패를 “수직선 자체가 없음”으로만 해석하면 안 되며, 실제 원인은 **소실점 후보 3개
절단 + 잘못 좁힌 down 경계**의 결합이었다.

v9 선택 후보의 aggregate 값은 다음과 같다.

| 항목 | 값 |
|---|---:|
| coarse yaw | 170° |
| refinement down | 27.8065° |
| mean edge distance | 35.9616 px |
| geometry NID 개선률 | +1.7475% |
| Manhattan vertical error | 6.8644° |
| Manual 예비 RT 회전 차이 | 9.5687° |

학습 네 장면은 모두 통과했지만 hold-out mean edge가 `40.8389 px`로 gate `40 px`를
`0.8389 px` 초과했다. 수직 구조/NID 문제는 개선됐으나 별도 장면 일반화가 아직 기준을
넘지 못했으므로 threshold를 완화하지 않고 최종 FAIL을 유지한다. 다음 단계는
`down 27~33°`, `yaw 165~171°`의 1° fine grid와 더 많은 동기 hold-out 수집이다.

`signal_strength` perturbation은 reference보다 나쁜 후보 비율 `0.5833`, median margin
`0.000633`으로 conformance에 실패했다. 따라서 이번 v9에도 signal NMI는 점수에 넣지
않았다.

최신 산출물:

```text
automatic_calibration/generated/
  repeat_test_sample_20260814_light_on_structural_nid_manhattan_v9/
    calibration_result.json
    training_scene_validation.csv
    holdout_scene_validation.csv
    matching_scene_0.png
    matching_scene_4.png
    scene_0_colorized_lidar_3d_preview.png
    signal_nmi_manual_rt_perturbation.csv
    debug/scene_*/03a_manhattan_vanishing_directions.csv
```

### 11.3 1° fine search와 평면 내부 가짜 edge 제거

v9 주변 `down=27~33°`, `yaw=165~171°`를 각각 1°로 탐색했다. 이 bounded yaw 구간은
첫 열 165°와 마지막 열 171°를 이웃으로 연결하지 않는다. 360° 전체 탐색에서만 yaw를
원형으로 wrap하며, `calibration_result.json`의 `yaw_neighbor_topology`로 구분한다.

첫 fine 실행 v10은 `yaw=167°`, grid down `28°`, refinement down `28.6895°`를 선택했다.
학습 4장면은 모두 통과했지만 hold-out edge mean이 `40.4238 px`여서 다음과 같이
최종 FAIL이었다.

```text
FAIL / HOLDOUT_VALIDATION_FAILED
```

`07a_projection_final_edge_residual.png`를 추가해 보이는 LiDAR edge를 Canny 거리별로
초록(≤10 px), 노랑(≤30 px), 빨강(>30 px)으로 표시했다. v10 hold-out은 mean
`40.35 px`, p50 `15.30 px`, p90 `110.01 px`, 30 px 초과 비율 `30.24%`였다. 빨간 점이
실제 깊이 단절뿐 아니라 책상 상판과 벽 내부에 연속해서 나타났다. 원인은 비스듬한 한
평면에서 인접 ray의 range 차이가 기존 절대 임계값을 넘으면 depth edge로 오인한 것이다.

다음 필터를 공통 LiDAR edge 추출 경로에 적용했다.

1. 같은 fitted plane label의 인접점 제외
2. normal이 유사하고 서로의 접평면 거리가 40 mm 이내인 공면점 제외
3. 현재 range gap이 같은 축의 앞·뒤 gap 최댓값보다 기본 2배 이상일 때만 보존
4. 실제 단절 양쪽의 점은 모두 유지

합성 grazing-plane 회귀 데이터는 기존 절대 임계값을 넘는 인접 쌍이 100개 이상이지만,
새 필터에서는 edge 0개가 되어야 한다. 비율은
`--lidar-edge-local-contrast-ratio`로 기록·조정하며 1 미만은 입력 오류로 거절한다.

동일 자세 범위를 재실행한 v11 결과는 다음과 같다.

| 항목 | v10: 기존 range edge | v11: 공면/local-contrast 필터 |
|---|---:|---:|
| 선택 yaw / grid down | 167° / 28° | 167° / 28° |
| refinement down | 28.6895° | 27.3755° |
| aggregate edge mean | 35.1294 px | 32.8701 px |
| aggregate NID 개선 | +1.2920% | +1.2360% |
| aggregate 수직 오차 | 6.0181° | 5.9567° |
| 장면별 LiDAR edge | 6,307~6,457 | 2,899~2,966 |
| 학습 장면 | 4/4 PASS | 4/4 PASS |
| hold-out edge mean | 40.4238 px | **37.1998 px** |
| hold-out | FAIL | **1/1 PASS** |
| 최종 내부 gate | FAIL | **PASS** |

v11 hold-out은 NID cell 7개, geometry NID `0.951842`, 구조선 18/18 대응
(수평 4, 수직 13), Manhattan 수직 오차 `6.8348°`를 얻었다. Top-view 광축은 이전처럼
반대 벽이나 바닥으로 뒤집히지 않았고, 2D 투영에서도 책상 전면·좌측 구조 주변의 실제
깊이 단절이 남았다.

단, 이 PASS는 **현재 4 training + 1 hold-out 분할의 내부 품질 gate 통과**다. Manual
예비 RT와는 회전 `8.5588°`, translation norm `0.12646 m` 차이가 있고 Manual 값도 독립
ground truth가 아니다. hold-out 한 장면만으로 제품 RT나 conformance PASS로 승인하지
않는다. `signal_strength` NMI도 기존 conformance FAIL이므로 가중치 0을 유지한다.

설정 검증과 리포트 metadata를 추가한 최종 코드로 v12를 다시 실행했고 v11과 같은
RT·수치·PASS가 재현됐다. v11/v12의 `matching_scene_0.png`와
`scene_0_colorized_lidar_3d_preview.png` SHA-256도 각각 동일하다. v12 리포트에는
`lidar_edge_policy`, local contrast `2.0`, bounded yaw topology가 명시된다.

산출물:

```text
automatic_calibration/generated/
  repeat_test_sample_20260814_light_on_structural_nid_manhattan_v10_fine1deg/
  repeat_test_sample_20260814_light_on_structural_nid_manhattan_v11_fine1deg_coplanar_edge/
  repeat_test_sample_20260814_light_on_structural_nid_manhattan_v12_fine1deg_coplanar_edge/
```

### 11.4 나이트비전 5쌍 고정 RT 교차검증 — 참고 진단

나이트비전은 현재 제품 요구사항이나 조명 ON calibration의 승인 조건이 아니다. 이 절은
고정 RT 검증 기능을 확인하기 위해 수행한 참고 실행을 보존한다. 아래 FAIL은 조명 ON
v12의 내부 PASS를 취소하지 않는다.

운영자 확인에 따라 앞 5쌍은 조명 ON, 뒤 5쌍은 조명 OFF/나이트비전으로 분류했다.
먼저 나이트비전 5쌍만으로 v12와 같은 4 training + 1 holdout 자동 추정을 수행했다.
v13은 yaw `168°`, refinement down `27.7697°` 부근을 찾았으나 training 1/4와
holdout 0/1로 `EDGE_ALIGNMENT_POOR / FAIL`이었다.

다음으로 v12의 `estimated_t_camera_lidar`를 고정해 나이트비전 5쌍 전체에 적용했다.
이 v14 경로는 pose를 최적화하지 않으며, 장면별 품질만 계산한다.

| scene | mean edge | visible/aligned | 판정 |
|---:|---:|---:|---|
| 0 | 44.4074 px | 100/47 | FAIL |
| 1 | 54.3862 px | 121/60 | FAIL |
| 2 | 48.8310 px | 106/53 | FAIL |
| 3 | 35.9032 px | 134/73 | PASS |
| 4 | 46.9635 px | 144/64 | FAIL |

v13의 거절 후보와 v12 RT 차이는 회전 `3.148°`, 이동 `3.566 mm`였다. 조명 ON과
나이트비전의 장면당 Canny edge 평균은 각각 약 `45,425`, `20,974 px`로 나이트비전에서
`53.8%` 감소했다. 따라서 현 FAIL은 전혀 다른 방향을 선택한 증상보다는 IR 영상의
edge sparsity/contrast와 day profile에서 구한 K·왜곡 보정의 재사용 가정에 민감한
증상이다. IR-cut 전환에 따른 유효 K 변화 가능성도 후속 분리시험 대상이며, 현
holdout을 보고 gate를 조정하지 않는다.

뒤 5쌍은 참고용 조명조건 반복 데이터다. 당시에는 구조가 다른 holdout을 권고했으나,
2026-08-15에 현재 시험 범위를 고정환경 시간 반복성으로 확정했다. 최신 데이터 조건은
12절을 따르며 나이트비전 일반화는 후속 선택 시험으로 분리한다.

산출물:

```text
automatic_calibration/generated/
  repeat_test_sample_20260814_night_vision_structural_nid_manhattan_v13_fine1deg/
  repeat_test_sample_20260814_night_vision_fixed_light_v12_rt_v14/
    fixed_pose_validation_result.json
    fixed_pose_scene_validation.csv
    debug/scene_*/06_projection_final.png
    debug/scene_*/07a_projection_final_edge_residual.png
```

## 12. 20260815 고정환경·고정조명 수집과 설치 교란 정정

2026-08-15 운영자 확인으로 현재 검증 범위를 다시 고정했다.

- 장소·공간 구조와 camera–LiDAR rigid 설치를 변경하지 않는 것을 당초 시험 조건으로 했다.
- 공식 automatic calibration 시험은 조명 ON 상태로 고정한다.
- `20260815`의 조명 OFF 사진은 야간/나이트비전이 아니라 같은 시간대의 환경 참고
  이미지이며 calibration 입력에서 제외한다.
- 현재 목적은 구조 일반화가 아니라 동일 installation epoch 안에서 같은 RT가 반복되는지
  확인하는 것이다.

후속 운영자 확인에서 `20260813`과 `20260815` 사이 actuator 문제로 장치를 건드렸고,
camera–LiDAR 상대 자세가 조금 변했을 가능성이 보고됐다. 따라서 두 날짜는 동일 rigid
설치라는 전제를 만족한다고 볼 수 없으며 서로 다른 installation epoch로 관리한다.

확인된 `20260815` 구성은 JPG 6개와 LiDAR JSON 3개다. JSON 3개는 같은 정적 환경에서
연속 측정한 반복 scan이고, JPG는 환경 변화가 없는 주말에 시간별로 획득한 snapshot이다.
공식 시험에는 조명 ON JPG 3개만 사용하고 조명 OFF JPG 3개는 별도 보관한다. 실행기는
입력 디렉터리의 image와 JSON 개수가 같아야 하므로 다음과 같이 분리하는 것을 기준으로
한다.

```text
repeat_test_sample/20260815/
  light_on/
    pair-001-CH1.jpg
    pair-001-lidar.json
    pair-002-CH1.jpg
    pair-002-lidar.json
    pair-003-CH1.jpg
    pair-003-lidar.json
  light_off_reference/
    20260815-181310-CH1.jpg
    20260815-181315-CH1.jpg
    20260815-181318-CH1.jpg
```

장면과 센서 설치가 고정되어 있으므로 image와 JSON의 촬영 시각 동기화 또는 물리적
pairing은 요구하지 않는다. 실행기에서는 조명 ON image와 JSON을 사전식 순서로 한 번씩
연결해 3개 observation을 구성한다. 이 association은 재현 가능한 파일 입력 형식일 뿐
동시 획득 관계를 뜻하지 않는다. 3×3 전체 조합을 9개 독립 표본으로 만들면 같은 증거가
중복되므로 사용하지 않는다. 실행 전 camera angle, zoom, focus, 해상도, 상하/좌우 반전,
복도뷰 상태가 `20260813` intrinsic profile과 동일한지는 확인한다.

검증은 다음 두 경로를 분리한다.

1. **고정 RT hold-out:** v12 `estimated_t_camera_lidar`를 새 3쌍에 그대로 적용하며
   최적화하지 않는다.
2. **RT 반복 추정:** 새 수집 회차에서 RT를 다시 계산하고 v12 대비 회전·이동 차이를
   기록한다.

위 두 실행은 이미 수행했지만, 설치 교란 정정 이후에는 고정환경 시간 반복성 판정이
아니라 교차 installation-epoch 진단으로 해석한다. `20260815` 회차 내부의 품질은 평가할
수 있으나 `20260813` 대비 차이를 순수 반복 오차로 사용할 수 없다.

## 13. 20260815 조명 ON 교차 installation-epoch 진단

### 13.1 실행 입력

원본 파일은 그대로 보존하고 Docker 내부 임시 디렉터리에 심볼릭 링크로만 3개
observation을 구성했다.

| scene | image | scan |
|---:|---|---|
| 0 | `20260814-230655-CH1.jpg` | `calib-20260814-232414_sweep-000001_pan_tilt_lidar.json` |
| 1 | `20260814-230728-CH1.jpg` | `calib-20260814-233403_sweep-000001_pan_tilt_lidar.json` |
| 2 | `20260814-230731-CH1.jpg` | `calib-20260814-234352_sweep-000001_pan_tilt_lidar.json` |

위 연결은 정적 환경에서 실행기의 동일 개수 제약을 충족하기 위한 사전식 association이다.
실제 동시 획득 pair나 3개의 완전 독립 장면을 뜻하지 않는다. 조명 OFF 참고 image 3개는
이번 입력에서 제외했다.

Manual intrinsic은 고정했고 raw distortion correction을 적용했다. LDC는 `false`,
camera center prior는 `(0.05928, -0.08105, 0) m`다. v12와 같은 geometry NID/edge/
structural line/Manhattan 목적함수 및 local contrast `2.0` 필터를 사용했다.

### 13.2 v15 기존 RT 고정 검증

`--validation-pose-json`으로 v12 RT를 입력해 pose 최적화를 비활성화했다.

| scene | aligned ratio | mean edge | geometry NID | structural matched | vertical error | 판정 |
|---:|---:|---:|---:|---:|---:|---|
| 0 | 0.6632 | 25.9134 px | 0.954940 | 25/25 | 4.0758° | PASS |
| 1 | 0.6643 | 23.0789 px | 0.954377 | 22/22 | 3.3905° | PASS |
| 2 | 0.7920 | 19.1769 px | 0.962132 | 25/25 | 3.0148° | PASS |

전체 수치는 `3/3 PASS`다. 그러나 `20260813`과 `20260815` 사이 장치 접촉으로 상대
자세가 변했을 가능성이 있으므로 외부 반복성 검증으로 분류하지 않는다. 작은 설치 변화가
있을 수 있는 회차에 v12 RT를 적용했을 때 현재 gate가 통과했다는 교차 회차 진단이다.

### 13.3 v16 독립 RT 재추정

동일한 bounded yaw/down 1° 탐색을 새 3개 observation에 수행했다. 별도 hold-out 없이
3개 모두 최적화에 사용했다.

| scene | aligned ratio | mean / p50 / p90 edge | 30 px 초과 | geometry NID | 판정 |
|---:|---:|---:|---:|---:|---|
| 0 | 0.7930 | 18.6291 / 13.0 / 46.1 px | 20.91% | 0.936950 | PASS |
| 1 | 0.7817 | 16.1479 / 11.0 / 38.3 px | 21.83% | 0.938573 | PASS |
| 2 | 0.7934 | 17.9024 / 12.0 / 43.3 px | 20.66% | 0.942369 | PASS |

선택 결과는 yaw `166°`, grid down `29°`, refined down `27.0676°`, roll `0°`다.
v12의 yaw `167°`, refined down `27.3755°`와 비교하면 최종 rotation geodesic은
`1.6873°`, translation norm 차이는 `2.8198 mm`다. translation 성분은 다음과 같다.

```text
v12 = (0.0532654, 0.0684590, 0.0506426) m
v16 = (0.0554520, 0.0679997, 0.0489224) m
delta = (+2.1866, -0.4593, -1.7202) mm
```

이 차이는 실제 actuator/camera–LiDAR 상대 자세 변화와 알고리즘 추정 변동을 함께 포함할
수 있다. camera와 LiDAR가 서로 상대적으로 움직였다면 extrinsic RT가 실제로 바뀐 것이고,
전체 assembly만 방 안에서 함께 움직였다면 센서 간 RT는 원칙적으로 바뀌지 않는다. 현재
기록만으로 어느 경우인지 분리할 수 없으므로 이 값을 실제 이동량이나 반복 오차로 단정하지
않는다.

v16의 장면별 투영에서는 캐비닛–벽, 책상 전면, 바닥/벽의 큰 구조 방향이 일관되게
정렬됐다. `07a`의 빨간 잔차는 캐비닛 내부 반복선, 케이블, 열린 책상 내부와 가림 경계에
집중돼 있으며 전체 자세가 반대 벽이나 바닥을 향하는 과거 실패 형태는 재발하지 않았다.

signal NMI manual-RT perturbation 진단은 `FAIL`, `activation_recommended=false`이며
가중치는 계속 `0`이다. signal 값은 본 PASS의 근거로 사용하지 않았다.
동일 코드 기준 Docker CTest는 `5/5 PASS`다.

### 13.4 정정 판정과 후속 조건

- v15 고정 RT와 v16 재추정은 각각 `3/3 PASS`지만 **`20260813→20260815` 동일 RT
  시간 반복성은 관측됐다고 판정할 수 없다.**
- v16은 3개 observation을 모두 최적화에 사용했으므로 그 자체는 외부 hold-out이 아니다.
  v15도 설치 동일성이 깨졌을 가능성 때문에 외부 반복성 근거가 아닌 교차 회차 진단이다.
- 회전 `1.6873°`, 이동 `2.8198 mm`는 실제 설치 변화와 추정 변동을 분리하지 못한
  관측값이며 최종 제품 conformance 판정이 아니다.
- 같은 정적 장면의 가까운 시간 snapshot과 연속 scan이라 표본 간 상관이 크다. 동일
  installation epoch에서 장치를 건드리지 않은 날짜 분리 수집을 누적해 RT 평균·표준편차·
  최댓값을 산출한 뒤 허용치를 정한다.
- actuator를 건드리거나 정비한 시점마다 installation epoch를 새로 만들고, 가능하면 전후
  기구 측정값 또는 해당 epoch의 Manual RT를 진단 reference로 남긴다.
- 다른 설치 장소, 공간 구조, 조명 OFF/나이트비전 일반화는 현재 결과에 포함하지 않는다.

산출물:

```text
automatic_calibration/generated/
  repeat_test_sample_20260815_light_on_fixed_v12_rt_v15/
  repeat_test_sample_20260815_light_on_repeat_estimation_v16/
```

## 14. 20260815 추가 조명 OFF 저조도 사진 진단

### 14.1 입력과 목적

추가된 `20260815-182818-CH1.jpg`, `20260815-182821-CH1.jpg`,
`20260815-182823-CH1.jpg`는 `2592×1520`이며 기존 18:13 사진군과 같은 조명 OFF
저조도 영상이다. 현재 나이트비전 또는 제품 조명 OFF 요구사항은 아니므로 조명 ON
conformance와 분리한 참고 시험으로만 사용했다.

18:28 사진 3개와 기존 연속 scan 3개를 결정론적으로 연결했다. 같은 실패가 다른 snapshot
시점에도 재현되는지 보기 위해 18:13 사진 3개도 동일 scan으로 별도 실행했다. scan을
재사용했으므로 두 영상군 결과를 6개 독립 observation으로 세지 않는다.

### 14.2 v16 RT 고정 결과

| 실행 | scene | aligned ratio | mean edge | vertical error | 결과 |
|---|---:|---:|---:|---:|---|
| v17 18:28 | 0 | 0.1129 | 226.8820 px | 50.9878° | FAIL |
| v17 18:28 | 1 | 0.1056 | 196.3935 px | 50.8897° | FAIL |
| v17 18:28 | 2 | 0.1157 | 179.8525 px | 51.1303° | FAIL |
| v19 18:13 | 0 | 0.1344 | 190.0897 px | 51.1928° | FAIL |
| v19 18:13 | 1 | 0.1549 | 191.8950 px | 51.2378° | FAIL |
| v19 18:13 | 2 | 0.2479 | 180.9749 px | 51.1223° | FAIL |

두 영상군 모두 `0/3 FAIL`이며 공통 실패 코드는 `EDGE_ALIGNMENT_POOR`와
`MANHATTAN_VERTICAL_ALIGNMENT_POOR`다. 다섯 장면에서는
`EDGE_OVERLAP_INSUFFICIENT`도 발생했다.

### 14.3 v18 저조도 RT 재추정

18:28 사진만으로 v16과 같은 yaw `165~171°`, down `27~33°`, roll `0°`, 1° 탐색을
수행했다. 선택된 진단 후보는 yaw `169°`, grid down `27°`, refined down `27.0031°`다.

| scene | aligned ratio | mean edge | vertical error | 결과 |
|---:|---:|---:|---:|---|
| 0 | 0.1588 | 184.5705 px | 51.6167° | FAIL |
| 1 | 0.1739 | 168.7591 px | 51.5204° | FAIL |
| 2 | 0.2034 | 161.0420 px | 51.7586° | FAIL |

최종 종료 코드는 `3`, 상태는 `FAIL / MANHATTAN_VERTICAL_ALIGNMENT_POOR`다. v16과
거절 후보의 회전 차이는 `2.1646°`, 이동 차이는 `3.6400 mm`지만, v18의
`visualization_pose_source`는 `rejected_optimization_candidate`이므로 이 RT를 활성
calibration 값으로 사용하지 않는다.

### 14.4 원인과 범위 판정

조명 ON v16 대비 18:28 저조도 v18의 camera edge pixel은 `162,746→10,417`
(`-93.60%`), camera structural line은 `316→72`(`-77.22%`)로 감소했다. 밝은 영상의
선택 수직 오차는 약 `4.66°`였으나 저조도는 `51.63°`다. scene 0의 밝은 영상은
56-inlier 수직 후보를 찾았지만 저조도는 14-inlier의 잘못된 방향을 선택했다.

geometry NID는 약 `0.95~0.97`을 유지했으나 edge와 구조 방향이 무너졌다. 따라서 이번
FAIL은 RT가 전혀 투영되지 않아서라기보다 저조도에서 camera feature가 부족해 목적함수의
식별력이 사라진 결과다. 전체 점 투영 PNG는 표면을 채워 맞아 보일 수 있지만, edge 잔차의
30 px 초과 비율은 v18에서 `79.7~84.1%`다.

현재 공식 조건은 조명 ON이므로 gate를 완화하거나 v18 RT를 채택하지 않는다. 저조도
지원이 제품 요구사항이 될 때 별도 profile/K 검증, photometric normalization/CLAHE,
저조도 edge/line 검출과 독립 데이터셋을 별도 설계한다. 활성 `20260815` RT 후보는
조명 ON v16으로 유지한다.

## 수정 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-15 | 추가 18:28 저조도 사진의 v16 고정 검증(v17), RT 재추정(v18), 18:13 대조군(v19)을 실행하고 0/3 반복 FAIL·feature 감소·거절 RT 비활성 판정을 기록 |
| 2026-08-15 | `20260813`과 `20260815` 사이 actuator 접촉에 따른 상대 자세 변화 가능성을 반영해 동일 RT 반복성 PASS 해석을 철회하고, v15/v16을 교차 installation-epoch 진단으로 재분류 |
| 2026-08-15 | `20260815` 조명 ON 데이터에 v12 RT 고정 검증(v15)과 독립 RT 재추정(v16)을 실행하고, 3/3 PASS·회전 1.6873°·이동 2.8198 mm 및 승인 한계를 기록 |
| 2026-08-15 | `20260815`가 연속 LiDAR scan과 시간별 snapshot으로 구성됨을 반영해 물리적 pairing 요구를 제거하고, 결정론적 계산 association·중복 조합 금지·디렉터리 규칙을 기록 |
| 2026-08-15 | 시험 범위를 고정 환경·조명 ON의 시간 반복성으로 확정하고, `20260815` 조명 ON 입력과 조명 OFF 참고 사진의 역할을 기록 |
| 2026-08-14 | 나이트비전 5쌍의 별도 RT 추정 v13과 조명 ON v12 RT 고정 교차검증 v14를 수행하고, 1/5 통과·edge 감소·구조 독립성 한계를 기록 |
| 2026-08-14 | 최종 코드 v12 재실행에서 v11과 동일한 RT·시각화 checksum·학습/hold-out PASS를 확인하고 edge 정책·비율을 결과 JSON에 기록 |
| 2026-08-14 | bounded yaw 이웃 topology, 1° v10 진단, Canny edge 잔차 heatmap, 공면/local-contrast LiDAR edge 필터와 grazing-plane 회귀 테스트를 추가하고 v11 내부 gate PASS 결과 및 승인 한계를 기록 |
| 2026-08-14 | range/normal spatial NID, 평면 경계·반복 폐색 구조선, 1:1 선분 대응, Manhattan 방향 잔차, signal NMI 진단, 장면별/hold-out gate를 구현하고 v6~v9 결과를 기록 |
| 2026-08-14 | 소실점 후보 3개 절단을 최대 12개로 수정하고 후보 CSV를 추가했으며, 최종 후보 재선택 후 품질 gate가 덮어써지던 PASS 오판을 수정 |
| 2026-08-14 | 모델링 도면의 X=59.28 mm가 기존 실행에 반영됐음을 확인하고, 높이 성분을 83.05→81.05 mm로 정정한 동일 조건 v5 A/B 결과 기록 |
| 2026-08-14 | 바닥 방향 후보 gate, 수평·수직 구조 방향 gate, refined 후보 우선 선택, Manual RT 2D·3D 비교 출력을 구현하고 조명 ON 15°/5°/geometry-first A/B 결과 기록 |
| 2026-08-14 | Manual intrinsic 고정·raw distortion 보정·Manual RT 진단 비교를 구현하고 repeat_test_sample 10세트 실행 결과를 기록 |

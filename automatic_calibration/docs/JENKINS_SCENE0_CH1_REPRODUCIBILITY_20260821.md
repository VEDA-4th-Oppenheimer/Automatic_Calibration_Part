# Jenkins `scene0` CH1 시간 반복성·hold-out 검증 기록

작성일: 2026-08-21  
데이터 수집일: 2026-08-20~2026-08-21  
대상 채널: Hanwha Vision PNM-C16083RVQ CH1  
installation epoch: `jenkins-scene0-fixed-20260820-20260821-ch1`  
최종 판정: **다중 장면 내부 PASS / 단독 RT 반복성 FAIL / 제품 RT 승격 금지**

> 2026-08-22~23에 추가된 `build17`, `build20`, `build21`의 테스트 목록, 입력
> 적격성 감사와 실행 결과는
> [JENKINS_SCENE0_CH1_EXPANDED_TEST_20260822.md](JENKINS_SCENE0_CH1_EXPANDED_TEST_20260822.md)에
> 이어서 기록했다. 본 문서의 4-pair 결과는 당시 코드·데이터 기준의 이력으로 유지한다.

## 1. 목적과 실행 전 검토

사용자가 확인한 조건은 다음과 같다.

- 네 데이터 묶음은 같은 장소와 같은 카메라–LiDAR 설치 상태에서 서로 다른 시각에 수집됐다.
- 사람이 없는 시간에 수집했고 CH1을 우선 검증한다.
- 상대 설치가 같으므로 네 측정에서 얻는 extrinsic RT는 오차 범위 안에서 같아야 한다.
- 세 묶음은 RT 추정·반복성 시험에 사용하고, 마지막 한 묶음은 최적화에서 제외한
  hold-out으로 사용한다.

실행 전에 아래 절차를 검토하고 그대로 적용했다.

1. 파일·manifest·해상도·JSON 좌표계·해시를 검사한다.
2. 시간순 앞의 세 묶음으로 하나의 RT를 추정하고 마지막 묶음에 고정 적용한다.
3. 앞의 세 묶음을 각각 단독 실행해 RT가 실제로 반복되는지 비교한다.
4. 숫자상의 gate뿐 아니라 2D reprojection과 3D top/front/side preview를 직접 확인한다.
5. 데이터 한계, 실패 원인, 코드 변경, 명령과 산출물을 모두 보존한다.

## 2. 원본 데이터와 결정론적 pairing

원본 루트:

```text
data/jenkins-capture/scene0/
```

각 Jenkins 패키지는 CH1~CH4 이미지, PCD, `_pan_tilt_lidar.json`, `manifest.json`으로
구성된다. 파일명 시각을 image–scan 동기화 근거로 사용하지 않고 **같은 패키지
디렉터리**를 pairing의 authoritative boundary로 사용했다.

| 순서 | Jenkins 패키지 | CH1 이미지 | LiDAR JSON | 용도 |
|---:|---|---|---|---|
| 0 | `calib_dataset_build5_20260820_223238` | `20260820_120107_CH1.jpg` | `calib-20260820-210106_sweep-000001_pan_tilt_lidar.json` | 단독 시험 + training |
| 1 | `calib_dataset_build8_20260820_232413` | `20260820_231445_CH1.jpg` | `calib-20260821-081040_sweep-000001_pan_tilt_lidar.json` | 단독 시험 + training |
| 2 | `calib_dataset_build9_20260820_233643` | `20260820_232737_CH1.jpg` | `calib-20260821-082657_sweep-000001_pan_tilt_lidar.json` | 단독 시험 + training |
| 3 | `calib_dataset_build10_20260821_000311` | `20260820_232737_CH1.jpg` | `calib-20260821-084019_sweep-000001_pan_tilt_lidar.json` | 고정 RT hold-out |

네 manifest 모두 `cctv_count=4`, `pcd_count=1`, `lidar_json_count=1`이다. CH1 영상은
모두 `2592×1520`이고 육안상 사람이 없으며 같은 고정 장면을 본다.

LiDAR JSON 공통 계약:

- schema `1.2`, `101×400 = 40,400` cells
- right-handed `lidar_scan`, 원점은 pan/tilt 축 교점
- `+x right, +y down, +z forward`
- `x=r*cos(tilt)*sin(pan)`, `y=-r*sin(tilt)`, `z=r*cos(tilt)*cos(pan)`
- `range_offset_m=0.084`는 producer 좌표에 이미 반영
- 유효 point 수는 `40,042~40,102`; 실행 필터 후 `39,055~39,296`

### 2.1 hold-out 독립성 제한

`build9`과 `build10`의 CH1 이미지는 파일명뿐 아니라 SHA-256도 같다.

```text
5570ed341738df4cdecdca1319c12584b4219d9f74c5a83cc802ae5d030d5101
```

두 LiDAR JSON의 SHA-256은 서로 다르다. 따라서 scene 3은 최적화에서 제외된 독립
LiDAR sweep 검증에는 사용할 수 있지만, **독립 camera–LiDAR pair hold-out은 아니다.**
이 문서에서는 이를 `limited hold-out`으로 분류한다.

## 3. 입력 처리 수정

기존 `run_real_calibration`은 입력 디렉터리 바로 아래의 파일만 읽고, 모든 JPG와 모든
JSON의 개수가 같아야 했다. Jenkins 패키지는 하위 디렉터리마다 CH1~CH4 네 장과
`manifest.json`을 포함하므로 원본 루트를 직접 입력할 수 없었다.

`automatic_calibration/apps/run_real_calibration.cpp`를 최소 수정했다.

- 루트에 직접 입력 파일이 없으면 하위 Jenkins 패키지를 재귀 탐색한다.
- `--camera-channel 1`이면 파일 stem이 `_CH1`로 끝나는 영상만 선택한다.
- JSON은 `_pan_tilt_lidar.json`만 선택하고 `manifest.json`은 제외한다.
- 같은 부모 패키지 안의 CH1 이미지와 LiDAR JSON만 pair로 묶는다.
- pair는 LiDAR scan 파일명 기준 시간순으로 정렬한다.
- 기존 flat input 디렉터리 동작은 유지한다.

Docker 빌드:

```bash
cmake --build /workspace-build --target run_real_calibration -j2
```

결과: compile 및 link PASS. 원본 `scene0` 루트를 직접 사용한 아래 실데이터 실행까지
완료해 입력 탐색 경로를 검증했다.

## 4. 공통 실행 조건

| 항목 | 값 |
|---|---|
| K+D | `manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json` |
| intrinsic profile | `charuco-pass-clean18-20260814` |
| resolution | `2592×1520` |
| image distortion | `raw`, OpenCV radtan D로 undistort 적용 |
| LDC metadata | `unknown` |
| zoom/focus | 수집 중 고정으로 기록 |
| camera center prior | `(0.05928, -0.08105, 0) m` in LiDAR frame |
| Manual RT prior | 사용하지 않음 |
| search | staged: 15° coarse → 5° local → 1° fine → Ceres |
| yaw topology | 360° circular |
| signal NMI weight | `0`(진단 전용) |
| product activation | 항상 차단, `activation_allowed=false` |

결합 실행 명령:

```bash
/workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_20260821_3train_1holdout \
  --pair-start 0 --pair-count 4 --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --search-strategy staged --holdout-count 1 --minimum-scene-pass-ratio 1.0
```

단독 반복성 실행은 위 명령에서 `--pair-count 1 --holdout-count 0`으로 바꾸고
`--pair-start 0`, `1`, `2`를 각각 사용했다.

## 5. 3 training + 1 limited hold-out 결과

| 항목 | 결과 |
|---|---|
| 상태 | `CANDIDATE_RT` |
| reason | `PASS` |
| activation | `false`, `NOT_PRODUCT_APPROVED_RT` |
| 선택 방향 | yaw `-190°` = 정규화 `170°`, down `29°`, optical roll `-1°` |
| confidence | `0.585022` |
| training | `3/3 PASS` |
| limited hold-out | `1/1 PASS` |
| 선택 후보 mean edge | `18.506 px` |
| 선택 후보 geometry NID | `0.931760` |
| 선택 후보 visible edge | `1,674` |
| 선택 후보 Manhattan vertical error | `9.069°` |

최종 `T_camera_lidar`:

```text
R = [
  [-0.9832071874,  0.0165296095,  0.1817426716],
  [ 0.1023551369,  0.8744481932,  0.4741980424],
  [-0.1510862423,  0.4848372196, -0.8614556390]
]

t = [0.0596225771, 0.0648094973, 0.0482505019] m
```

`t`는 camera center 좌표 자체가 아니라 `p_camera = R*p_lidar + t`의 translation이다.
camera center prior와 직접 성분 비교하면 안 된다.

Training scene별 mean edge는 `18.607`, `17.243`, `19.704 px`이고 limited hold-out은
`21.804 px`다. 네 2D reprojection과 3D preview는 같은 방 방향을 보고 있으며 결합
후보의 방향은 육안상 일관된다. 다만 초록색 LiDAR edge가 의자·책상 장애물에도 많이
반응하므로 이 내부 PASS만으로 정확한 RT라고 확정하지 않는다.

## 6. 세 training pair 단독 RT 반복성 결과

| 실행 | 선택 yaw | down | roll | confidence | 내부 상태 | 시각 판정 |
|---|---:|---:|---:|---:|---|---|
| pair 0 / build5 | `-128°` | `35°` | `10°` | `0.581537` | `INTERNAL_GATE_PASS` | **잘못된 방향** |
| pair 1 / build8 | `-190°` | `34°` | `6°` | `0.579188` | `INTERNAL_GATE_PASS` | 대체로 같은 yaw basin |
| pair 2 / build9 | `-190°` | `29°` | `-1°` | `0.585214` | `INTERNAL_GATE_PASS` | 결합 RT와 유사 |

회전은 전체 rotation matrix의 geodesic 차이로, 이동은 translation vector 차이 norm으로
계산했다.

| 비교 | rotation geodesic | translation 차이 |
|---|---:|---:|
| pair 0 ↔ pair 1 | `64.548°` | `65.055 mm` |
| pair 0 ↔ pair 2 | `68.564°` | `65.001 mm` |
| pair 1 ↔ pair 2 | `8.190°` | `8.429 mm` |
| pair 0 ↔ 결합 RT | `68.591°` | `65.383 mm` |
| pair 1 ↔ 결합 RT | `8.391°` | `8.991 mm` |
| pair 2 ↔ 결합 RT | `0.393°` | `0.669 mm` |

같은 설치 상태에서 같은 RT가 나와야 한다는 시험 불변식을 만족하지 못했다. 세 실행이
모두 내부 `PASS`를 반환했으므로 현재 gate는 **단일 장면 false positive를 차단하지
못한다.**

## 7. 실패 원인 분석

### 7.1 finalist 사이의 ambiguity가 거절되지 않음

pair 0에는 yaw `-185°`의 다른 PASS 후보가 있었고 confidence는 `0.575316`이었다.
최종 선택된 잘못된 `-128°` 후보의 confidence `0.581537`과 차이는 약 `0.00622`뿐이다.
결과 JSON에는 ambiguity margin `0.02`가 기록되지만, 현재 이 값은 Core 내부 multistart
objective에만 적용된다. staged finalist 선택 코드는 confidence가 `1e-4`보다 다르면 더
큰 값을 즉시 선택하며 **finalist 간 ambiguity FAIL을 만들지 않는다.**

### 7.2 TESL/asymmetric 집계 누락

결합 실행의 scene CSV에는 `total_explained_structural_length`와
`asymmetric_structural_weight`가 유효한 값으로 존재한다. 그러나 선택 finalist의 집계
metrics에는 두 값이 모두 `0`이다. 따라서 새 구조선 가중치가 최종 confidence의 TESL
항에 제대로 반영되지 않는다.

### 7.3 단일 장면의 구조 모호성과 장애물 edge

고정 장면에는 평행한 책상, 모니터, 의자, 벽·바닥 선이 반복된다. 현재 depth/normal
edge가 이러한 장애물 경계를 많이 사용하므로 서로 다른 3D 방향도 영상의 여러 선과
부분적으로 맞을 수 있다. pair 0의 잘못된 top-view가 내부 edge·Manhattan gate를 모두
통과한 것이 직접 증거다.

### 7.4 down/roll 식별력 부족

pair 1과 pair 2는 yaw basin은 같지만 전체 회전 차이가 `8.190°`다. 이는 단일 영상의
수평·수직 구조만으로 down과 optical roll을 안정적으로 분리하지 못하고 있음을 뜻한다.

### 7.5 hold-out의 camera frame 중복

limited hold-out PASS는 새 LiDAR sweep에 대한 고정 RT 재투영 일관성을 보여준다. 하지만
training scene 2와 동일한 CH1 픽셀을 재사용했으므로 독립 영상 일반화 증거가 아니다.

## 8. 판정과 다음 수정 우선순위

현재 판정:

- 결합 실행: `CANDIDATE_RT / limited hold-out PASS`
- 독립 단독 실행 반복성: **FAIL**
- 제품 RT: `NOT_PRODUCT_APPROVED_RT`, activation 금지 유지

수정 우선순위:

1. finalist confidence 1·2위 차이가 기준보다 작으면 선택하지 않고
   `FINALIST_AMBIGUOUS`로 FAIL 처리한다.
2. scene별 TESL/asymmetric support를 finalist aggregate에 올바르게 누적한다.
3. finalist 공통 기준으로 absolute visible edge, explained structural length, 영상 공간
   coverage를 비교해 sparse/obstacle basin을 거절한다.
4. 세 독립 실행의 SO(3) rotation cluster와 translation cluster가 합의하지 않으면 batch
   RT를 승격하지 않는 반복성 gate를 추가한다.
5. `build10`에는 실제로 새로 캡처된 CH1 이미지를 넣어 camera와 LiDAR 모두 독립인
   hold-out을 다시 만든다.
6. 같은 설정 반복을 누적해 제품 정책 초안인 rotation 표준편차 `≤0.2°`, translation
   표준편차 `≤10 mm`, 약 10회 반복을 평가한다.

## 9. 산출물과 영구 로그

결합 실행:

```text
automatic_calibration/generated/jenkins_scene0_ch1_20260821_3train_1holdout/
├── calibration_result.json
├── console.log
├── training_scene_validation.csv
├── holdout_scene_validation.csv
├── matching_scene_0.png ... matching_scene_3.png
├── scene_0_colorized_lidar_3d_preview.png ... scene_3_*.png
├── orientation_full_search.csv
├── search_5deg_scores.csv
├── search_1deg_scores.csv
└── top_candidates/rank_1.png ... rank_5.png
```

단독 실행:

```text
automatic_calibration/generated/jenkins_scene0_ch1_20260821_pair0/
automatic_calibration/generated/jenkins_scene0_ch1_20260821_pair1/
automatic_calibration/generated/jenkins_scene0_ch1_20260821_pair2/
```

각 디렉터리에 전체 stdout JSON을 담은 `console.log`, 최종
`calibration_result.json`, score map, 2D/3D 시각화가 보존돼 있다.

## 10. 변경 이력

| 날짜 | 변경 |
|---|---|
| 2026-08-21 | Jenkins scene0 4개 패키지 무결성 검사, CH1 3 training + 1 limited hold-out 분할 확정 |
| 2026-08-21 | Jenkins 중첩 패키지 직접 입력·채널 필터·manifest 제외·부모 디렉터리 pairing 구현 및 빌드 |
| 2026-08-21 | 결합 실행과 training 3개 단독 실행 완료, 2D/3D 시각 검토 및 RT geodesic/translation 비교 기록 |
| 2026-08-21 | 단독 RT 반복성 FAIL, finalist ambiguity/TESL/hold-out 중복 문제와 후속 우선순위 확정 |
| 2026-08-23 | build17/20/21 확장 시험 문서 링크 추가; 기존 4-pair 결과는 과거 baseline으로 고정 |

# Jenkins `scene0` CH1 확장 데이터 테스트 목록 및 실행 기록

- 작성일: 2026-08-22
- 대상: `build17`, `build20`, `build21` 추가분과 기존 `scene0` CH1 데이터
- 실행 환경: Ubuntu `latest` 기반 `auto-calib-dev` Docker 컨테이너
- 실행 시작일: 2026-08-22
- 완료일: 2026-08-23
- 상태: 최초 실행과 build20/build21 영상 수정본 재실행·시각 검토·문서화 완료
- 제품 RT 승격: 금지 (`activation_allowed=false` 유지)
- 최신 입력/결과: build20/build21 수정본은 §2.2와 §6.8 참조

## 1. 목적

`build17`의 수정 영상과 새로 추가된 `build20`, `build21`을 기존 Jenkins
`scene0` 반복성 시험에 포함한다. 이번 시험은 다음을 분리해 확인한다.

1. 입력 파일과 고정 Manual ChArUco K+D의 계약이 맞아 프로그램이 실행되는가.
2. 각 추가 pair를 단독 추정할 때 동일한 물리 RT basin이 반복되는가.
3. `build17~19`에서 추정한 RT가 `build20~21` hold-out에서 유지되는가.
4. 편집 영상, 동적 객체, camera–scan 시간 불일치가 있을 때 품질 gate가 안전하게
   거절하는가.

내부 `PASS`는 제품 승인과 다르다. 입력 pair의 독립성과 동시성이 확인되지 않은 실행은
수치가 좋아도 `DIAGNOSTIC_ONLY`로 해석한다.

## 2. 입력 목록과 사전 감사

`run_real_calibration`의 scan 파일명 정렬 기준 전체 CH1 pair index는 다음과 같다.

| pair index | 패키지 | 역할/제약 |
|---:|---|---|
| 0 | `build5` | 기존 training 기준 |
| 1 | `build8` | 기존 training 기준 |
| 2 | `build9` | 기존 training 기준 |
| 3 | `build10` | CH1이 `build9`과 byte-identical인 limited hold-out |
| 4 | `build17` | 이번 추가 단독 시험 및 batch training 진단 |
| 5 | `build18` | batch training 진단; 영상 내 큰 근거리 보드 존재 |
| 6 | `build19` | batch training 진단; 사람 존재 |
| 7 | `build20` | 수정본 단독 시험 및 start-synchronized dynamic-object hold-out |
| 8 | `build21` | 수정본 단독 시험 및 start-synchronized dynamic-object hold-out |

### 2.1 추가 요청 세트의 최초 파일 계약(영상 교체 전)

| build | CH1 영상 | LiDAR JSON | 해상도 | CH1 SHA-256 | 입력 판정 |
|---|---|---|---:|---|---|
| 17 | `20260821_041806_CH1.jpg` | `calib-20260821-131830_sweep-000001_pan_tilt_lidar.json` | `2592×1520` | `b338574bbd987fe74e136f2f9536410fc7de201b20b724491d32a2b4f7377566` | 실행 가능, 편집 영상 진단용 |
| 20 | `20260821_120808_CH1.jpg` | `calib-20260822-000015_sweep-000001_pan_tilt_lidar.json` | `2592×1520` | `f3906c1097a3b498db5de5d05834758c59d78a72f206f26c6c6e8e83c280dd12` | 실행 가능, 시간 불일치 stress용 |
| 21 | `20260821_120829_CH1.jpg` | `calib-20260822-225748_sweep-000001_pan_tilt_lidar.json` | `2592×1520` | `33b3434a1ed2932a010b18b34f2247b34a2546f5e2d932c55b87703637f19843` | 실행 가능, 시간 불일치 stress용 |

`build17`은 수정 후 Manual intrinsic profile의 `2592×1520`과 일치해 이전 해상도
거절 원인은 해소됐다. 다만 영상 좌하단에 `AI로 생성한 콘텐츠` 표기가 있고 EXIF 수정
시각이 2026-08-22로 기록되어 있다. 생성형 보정/업스케일은 원래의 pixel geometry와
edge를 바꿀 수 있으므로 원본 카메라 프레임과 동등한 제품 검증 입력으로 인정하지 않는다.

`build20`과 `build21`은 사람이 보이는 동적 장면이다. 또한 camera 파일명을 UTC,
LiDAR 파일명을 KST로 해석하는 현재 수집 규칙을 적용하면 각각 약 `2 h 52 min`,
`25 h 49 min` 차이가 난다. `manifest.json`에는 pair별 촬영 시각·해시가 없어 이 해석을
자동 검증할 수 없으므로, 두 세트는 요청대로 실행하되 독립 hold-out이 아니라
**시간 불일치 stress hold-out**으로 분류한다.

LiDAR JSON/PCD는 세 세트 모두 schema `1.2`, `101×400=40,400` grid, 좌표계
`+x right, +y down, +z forward` 계약을 만족하며 JSON 좌표와 PCD가 일치한다.

### 2.2 2026-08-23 build20/build21 영상 수정본

기존 결과를 덮어쓰지 않고 입력 영상 변경의 영향만 비교하기 위해 수정본 재시험을
별도 실행으로 추가한다. CH1뿐 아니라 CH1~CH4 영상이 모두 교체됐으나, 이번 재시험은
기존 시험과 동일하게 CH1만 사용한다.

| build | 수정 CH1 영상 | 해상도 | 수정 CH1 SHA-256 | LiDAR session과의 관계 |
|---|---|---:|---|---|
| 20 | `20260822_000015_CH1.jpg` | `2592×1520` | `1fe7f93e787c2de8e8a38358210aca963e9ea07dd200849af0091e47a6b256d8` | 영상 파일명·EXIF와 scan session 시작명이 `2026-08-22 00:00:15`로 일치 |
| 21 | `20260822_225748_CH1.jpg` | `2592×1520` | `ac22e5d454c682a846373445c012d9e7550b42bc09cb631b320064335762cb72` | 영상 파일명·EXIF와 scan session 시작명이 `2026-08-22 22:57:48`로 일치 |

두 LiDAR scan의 monotonic 시작/종료 차이는 각각 약 `570.5 s`, `570.9 s`다. 따라서
수정 영상은 **스캔 시작 프레임과 일치한다는 강한 파일 증거**가 있으며, 기존의 장시간
camera–scan 불일치 분류는 철회한다. 단, manifest에 카메라 캡처 시각과 scan 시작/종료
시각이 독립 필드로 서명돼 있지는 않고 두 영상 모두 원거리 사람이 보인다. 약 9분 31초
동안 움직인 물체는 snapshot 영상과 누적 LiDAR 사이의 국소 불일치를 만들 수 있으므로
`동적 객체 포함 동시 pair`로 판정한다.

수정본 재시험은 기존 산출물을 보존하고 다음 ID와 출력 경로를 사용한다.

| ID | 구성 | 출력 디렉터리 |
|---|---|---|
| `E20-R1` | build20 수정 CH1 단독 재추정 | `jenkins_scene0_ch1_20260823_build20_image_fixed` |
| `E21-R1` | build21 수정 CH1 단독 재추정 | `jenkins_scene0_ch1_20260823_build21_image_fixed` |
| `E17-21-R1` | build17~19 training + 수정 build20~21 hold-out | `jenkins_scene0_ch1_20260823_build17_21_3train_2holdout_image_fixed` |

## 3. 고정 실행 조건

| 항목 | 값 |
|---|---|
| 카메라 | PNM-C16083RVQ CH1 |
| K+D | `manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json` |
| intrinsic profile | `charuco-pass-clean18-20260814`, `2592×1520` |
| distortion | `raw`, Manual D로 undistort |
| LDC | `unknown` metadata로 입력하되 `image-distortion-state=raw`를 명시 |
| zoom/focus | 고정으로 기록 |
| camera center prior | `(0.05928, -0.08105, 0) m` in LiDAR frame |
| Manual RT prior | 사용하지 않음 |
| 탐색 | staged: coarse → local 5° → fine 1° → Ceres |
| hold-out 정책 | 최적화에 미사용, 고정 RT로만 평가 |

공통 옵션:

```bash
--camera-channel 1 \
--ldc-enabled unknown --zoom-focus-locked true \
--manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
--image-distortion-state raw \
--camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
--search-strategy staged --minimum-scene-pass-ratio 1.0
```

## 4. 테스트 목록

아래 표는 영상 교체 전 최초 실행이다. 수정본 재시험 `E20-R1`, `E21-R1`,
`E17-21-R1`은 §2.2와 §6.8에 기록한다.

| ID | 입력/명령 핵심 | 검증 목적 | 입력 계약상 기대 판정 | 실행 결과 |
|---|---|---|---|---|
| `E00` | Docker build + CTest | 코드 회귀 확인 | 전체 PASS | **PASS, 9/9, 111.59 s** |
| `E17` | `pair-start=4`, `pair-count=1`, `holdout=0` | 수정된 build17 실행 및 단독 RT | 실행 가능, 제품 근거 불가 | **내부 PASS / 데이터 적격성 FAIL** |
| `E20` | `pair-start=7`, `pair-count=1`, `holdout=0` | 시간 불일치/사람 장면 fail-safe | FAIL 또는 diagnostic이 바람직 | **FAIL / `FINALIST_AMBIGUOUS`** |
| `E21` | `pair-start=8`, `pair-count=1`, `holdout=0` | 장시간 불일치 fail-safe | FAIL 또는 diagnostic이 바람직 | **FAIL / `COARSE_OVERLAP_INSUFFICIENT`** |
| `E17-21` | `pair-start=4`, `pair-count=5`, `holdout=2` | build17~19 training + build20~21 stress hold-out | 제품 승격 불가; hold-out 거절 여부 확인 | **코드상 `CANDIDATE_RT`, 데이터 적격성 FAIL** |

`E20/E21`에서 내부 gate가 `PASS`를 반환해도 알고리즘 정확도 PASS로 해석하지 않는다.
그 경우 입력 metadata gate가 시간 불일치를 차단하지 못한 **false-accept 안전성 결함**으로
기록한다.

## 5. 실행 명령

단독 실행은 아래의 `PAIR`와 `NAME`을 각각 `(4, build17)`, `(7, build20)`,
`(8, build21)`로 바꿔 수행한다.

```bash
/workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_20260822_NAME \
  --pair-start PAIR --pair-count 1 --holdout-count 0 \
  --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --search-strategy staged --minimum-scene-pass-ratio 1.0
```

확장 batch:

```bash
/workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_20260822_build17_21_3train_2stress_holdout \
  --pair-start 4 --pair-count 5 --holdout-count 2 \
  --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --search-strategy staged --minimum-scene-pass-ratio 1.0
```

## 6. 실행 결과

### 6.1 회귀 테스트

Docker의 현재 build tree에서 아래 회귀 테스트를 수행했다.

```text
9/9 PASS, 0 FAIL, total 111.59 s
```

실행 대상은 Calibration Core, challenger stress/ambiguity, manual marker, Top-view와
GUI smoke를 포함한다. 실데이터 실행 전 코드 회귀는 확인됐다.

### 6.2 추가 세트 단독 실행

| 실행 | runtime | 상태 / reason | yaw seed | down | roll | confidence | finalist margin | 해석 |
|---|---:|---|---:|---:|---:|---:|---:|---|
| `E17` | 약 8분 19초 | `INTERNAL_GATE_PASS / PASS` | `169°` | `22°` | `4°` | `0.789297` | `0.030651` | 기존 정상 basin과 일치, 그러나 편집 영상이라 제품 근거 불가 |
| `E20` | 약 8분 39초 | `FAIL / FINALIST_AMBIGUOUS` | `51°` | `26°` | `7°` | `0.789629` | `0.013909` | 51°와 정상 방향 166° 후보가 근소하게 경쟁해 안전 차단 |
| `E21` | 약 7분 53초 | `FAIL / COARSE_OVERLAP_INSUFFICIENT` | `0°` | `15°` | `-15°` | `0.050000` | `1.0` | 최종 seed가 support를 확보하지 못했고 scene Manhattan gate도 실패 |

`E20`의 두 유효 finalist는 다음과 같다.

| 후보 | yaw seed | down | roll | confidence |
|---|---:|---:|---:|---:|
| 잘못된 고득점 basin | `51°` | `26°` | `7°` | `0.789629` |
| 물리적으로 일관된 basin | `166°` | `26°` | `-3°` | `0.775721` |

두 confidence 차이는 `0.013909`로 기준 `0.02`보다 작다. 이전 로직이라면 51°를
선택할 수 있었지만, 현재 finalist ambiguity gate는 이를 `FAIL`로 차단했다.

`E21`은 coarse score map의 raw best가 여전히 yaw `165°`, down `30°`였지만 fine/Ceres
후보가 충분한 absolute support를 확보하지 못했다. 장면 검증 실패에는
`MANHATTAN_VERTICAL_ALIGNMENT_POOR`가 기록됐다. 실패 결과의 PNG와 3D preview는
`REJECTED CANDIDATE: RT not active`로 명시돼 active RT와 혼동되지 않는다.

### 6.3 3-training + 2-stress-hold-out batch

| 항목 | 결과 |
|---|---|
| runtime | 약 22분 37초 |
| 프로그램 상태 | `CANDIDATE_RT / PASS` |
| activation | `false`, `NOT_PRODUCT_APPROVED_RT` |
| 선택 방향 | yaw seed `169°`, down `23°`, roll `5°` |
| confidence | `0.785223` |
| finalist margin | `0.033943 > 0.02` |
| training | build17/18/19, `3/3 PASS` |
| stress hold-out | build20/21, `2/2 PASS` |

최종 `T_camera_lidar`:

```text
R = [
  [-0.9843840383, -0.0797655733,  0.1569251999],
  [-0.0107793188,  0.9170874686,  0.3985403132],
  [-0.1757039309,  0.3906251762, -0.9036260844]
]

t = [0.0518908643, 0.0749690858, 0.0420737964] m
```

Hold-out scene별 지표:

| hold-out | visible edge | mean edge | geometry NID | 구조선 matched/visible | vertical error | 판정 |
|---|---:|---:|---:|---:|---:|---|
| build20 | 686 | `11.671 px` | `0.925647` | `23/24` | `13.347°` | PASS |
| build21 | 687 | `11.832 px` | `0.931273` | `17/17` | `13.361°` | PASS |

단독 실행 실패와 batch hold-out PASS는 모순이 아니다. 단독 실행은 360° 공간에서 RT를
**새로 식별할 수 있는지** 검사하고, hold-out은 training에서 이미 정한 하나의 RT가 해당
장면을 **설명할 수 있는지**만 검사한다. build20의 정상 166° basin과 build21의 coarse
165° basin이 이미 존재했기 때문에, 다중 장면이 169°를 고정하면 두 hold-out은 통과할
수 있다.

그러나 이 PASS는 camera–scan 동시성을 복원하지 않는다. 정적 벽·바닥·가구가 오랜 시간
유지된 방에서는 오래된 camera frame도 최신 LiDAR와 유사한 구조 score를 낼 수 있다.
따라서 실행 결과의 `CANDIDATE_RT`와 별도로 이 시험의 **데이터 conformance 판정은 FAIL**이며
제품 승인 evidence로 사용하지 않는다.

### 6.4 RT 반복성과 시각 검토

| 비교 | rotation geodesic | translation 차이 | 해석 |
|---|---:|---:|---|
| E17 단독 ↔ 새 batch | `1.373°` | `1.740 mm` | 같은 yaw basin이나 제품 반복성 목표를 입증하지 못함 |
| 2026-08-21 과거 batch ↔ 새 batch | `8.260°` | `14.183 mm` | 실행 코드 버전이 달라 순수 데이터 반복성 비교로 사용 불가 |

새 batch의 2D/3D 시각화는 다섯 장 모두 같은 방 방향을 보고 있어 90°/180° gross 방향
반전은 보이지 않는다. 다만 green LiDAR edge가 의자·책상·바닥의 반복 구조에 넓게 분포하고,
pixel correspondence의 독립 참값도 없으므로 육안 PASS만으로 정밀 RT라고 확정하지 않는다.

제품 정책의 동일 설치 10회 회전 표준편차 `≤0.2°`, 이동 표준편차 `≤10 mm`는 아직
검증되지 않았다. E20/E21은 독립 RT를 산출하지 못했고 E17은 편집 영상이므로, 이번 세트로
제품 반복성 표준편차를 계산하면 안 된다.

### 6.5 최종 판정과 다음 입력 조건

이번 실행으로 확인된 사항:

- build17 해상도 오류는 해소됐고 정상 yaw basin을 다시 찾았다.
- finalist ambiguity gate는 build20의 51° false basin 승격을 실제로 차단했다.
- overlap/Manhattan gate는 build21의 support 없는 후보를 차단했다.
- 다중 장면 결합은 169° basin에 수렴했고 고정 RT hold-out `2/2`를 통과했다.
- 모든 실행에서 `activation_allowed=false`가 유지됐다.

제품 검증에 다시 사용할 입력 조건:

1. `build17`은 resize/AI 보정하지 않은 **카메라 원본 2592×1520** 프레임으로 교체한다.
2. build20/21 수정본처럼 scan 시작 프레임을 사용하되, 같은 수집 job에서 자동 캡처하고
   사람·이동 물체가 없는 장면을 확보한다.
3. manifest에 `camera_captured_at_utc`, `scan_started_at_utc`, `scan_ended_at_utc`, 파일
   SHA-256, channel profile, installation epoch를 저장하고 허용 시간 차이를 자동 gate한다.
4. 사람이나 이동 중인 대형 보드가 없는 장면을 제품 training/hold-out으로 사용한다.
5. 원본·동시 pair로 training 외 독립 hold-out 3쌍 이상과 동일 설치 반복 10회를 채운다.

### 6.6 Jenkins 운용 분류

| 주기 | 테스트 | 실측 시간/근거 |
|---|---|---|
| daily | `E00` 9종 회귀 | 약 1분 52초 |
| weekly 또는 수동 | 새 pair 단독 staged search | pair당 약 8~10분 |
| weekly 또는 수동 | 3-training + 2-hold-out batch | 약 22분 37초~30분 24초 |

실데이터 staged search를 daily 전체 목록에 넣으면 데이터 수가 늘수록 Jenkins 시간이 크게
증가하므로, daily에는 빠른 회귀와 입력 manifest audit를 두고 실데이터 full search는
weekly/수동 conformance로 유지한다.

### 6.7 산출물

```text
automatic_calibration/generated/jenkins_scene0_ch1_20260822_build17/
automatic_calibration/generated/jenkins_scene0_ch1_20260822_build20/
automatic_calibration/generated/jenkins_scene0_ch1_20260822_build21/
automatic_calibration/generated/jenkins_scene0_ch1_20260822_build17_21_3train_2stress_holdout/
```

각 경로에는 `console.log`, `calibration_result.json`, score CSV, scene validation CSV,
2D matching PNG, 3D preview와 PLY/OBJ가 보존돼 있다.

### 6.8 2026-08-23 build20/build21 영상 수정본 재시험

#### 입력 변경의 실제 범위

수정 JPEG 파일의 SHA-256은 바뀌었지만 캘리브레이션 입력 픽셀은 바뀌지 않았다.
수정 전·후의 undistort 결과 `prepared/scene_0.png`, 5° score map, 1° score map을
비교한 결과 build20과 build21 모두 각각 byte-identical이었다. JPEG 파일명과 EXIF
`DateTime`이 scan session 시작명과 일치하도록 수정됐고, 영상 내용은 유지된 것이다.

| 검사 | build20 | build21 |
|---|---|---|
| 수정 전·후 prepared PNG | 동일 | 동일 |
| 수정 전·후 5° score CSV | 동일 | 동일 |
| 수정 전·후 1° score CSV | 동일 | 동일 |
| camera frame 이름 ↔ scan session 시작명 | `20260822_000015` 일치 | `20260822_225748` 일치 |
| scan 누적 시간 | 약 `570.5 s` | 약 `570.9 s` |

따라서 §2.1의 장시간 camera–scan 불일치 판정은 현재 수정본에는 적용하지 않는다.
대신 두 세트는 **스캔 시작 동기 pair(파일명·EXIF 근거), 약 9분 31초 누적 scan,
동적 객체 포함**으로 분류한다. manifest가 camera/scan 시각과 SHA를 독립 필드로
보관하지 않아 자동 provenance gate가 통과한 것은 아니므로 최종 판정은 조건부다.

#### 단독 및 batch 재실행 결과

| 실행 | runtime | 상태 / reason | yaw/down/roll | confidence / margin | 수정 전 대비 |
|---|---:|---|---|---|---|
| `E20-R1` | 약 `9.93 min` | `FAIL / FINALIST_AMBIGUOUS` | `51° / 26° / 7°` rejected | `0.789629 / 0.013909` | 수치 판정 동일 |
| `E21-R1` | 약 `10.21 min` | `FAIL / COARSE_OVERLAP_INSUFFICIENT` | `0° / 15° / -15°` rejected | `0.05 / 1.0` | 수치 판정 동일 |
| `E17-21-R1` | 약 `30.40 min` | `CANDIDATE_RT / PASS` | `169° / 23° / 5°` | `0.785223 / 0.033943` | RT·판정·hold-out 지표 동일 |

재실행 batch의 최종 `T_camera_lidar`는 최초 batch와 bit-for-bit 같은 수치다.

```text
R = [
  [-0.9843840383, -0.0797655733,  0.1569251999],
  [-0.0107793188,  0.9170874686,  0.3985403132],
  [-0.1757039309,  0.3906251762, -0.9036260844]
]

t = [0.0518908643, 0.0749690858, 0.0420737964] m
```

| 수정 hold-out | visible edge | mean edge | geometry NID | 구조선 matched/visible | vertical error | 판정 |
|---|---:|---:|---:|---:|---:|---|
| build20 | 686 | `11.671 px` | `0.925647` | `23/24` | `13.347°` | PASS |
| build21 | 687 | `11.832 px` | `0.931273` | `17/17` | `13.361°` | PASS |

수정 전·후 batch의 `matching_scene_3.png`, `matching_scene_4.png`와 두 3D preview도
각각 byte-identical이다. 시각적으로 gross 90°/180° 방향 반전은 없지만 green LiDAR
edge가 반복적인 바닥·의자·책상 구조에 넓게 걸쳐 있다. 현재 gate PASS는 이 고정 RT가
장면 구조를 설명한다는 뜻이지, pixel correspondence나 독립 물리 참값으로 RT 정확도가
입증됐다는 뜻은 아니다.

#### 최신 판정

- **해소:** build20/build21의 잘못된 camera 파일명과 장시간 불일치 해석.
- **유지:** build20 단독의 51°/166° ambiguity와 build21 단독의 absolute support 부족.
- **유지:** training에 편집 build17이 포함되고, 사람/보드 등 동적 객체가 있으며,
  independent physical reference와 동일 설치 반복 10회가 없다.
- **결론:** corrected batch는 유효한 결정론적 재현성 증거이지만 여전히
  `CANDIDATE_RT`; `activation_allowed=false`, 제품 RT 승격 금지.

수정본 산출물:

```text
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build20_image_fixed/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build21_image_fixed/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build17_21_3train_2holdout_image_fixed/
```

### 6.9 2026-08-23 staged gate·finalist 선택 개선 후 재시험

#### 원인

build21의 정상 방향은 full coarse score map에 존재했지만, 서로 다른 카메라 FOV에서
계산한 global maximum NID point count가 5°/1° local hard gate의 공통 분모로 사용됐다.
정상 `yaw≈168°` basin의 상대 NID 수가 0.5 미만이 되어 basin 전체가
`COARSE_OVERLAP_INSUFFICIENT`로 제거됐다.

`minimum-relative-nid-coverage=0`만 적용한 A/B는 정상 후보를 복구했지만 confidence-first
선택이 `yaw≈85°` false basin을 내부 PASS했다. 임계값 완화는 폐기하고 다음 구조로
수정했다.

- local 5°/1° 단계: NID relative hard gate를 local yaw window 안에서 계산
- final Ceres: global NID coverage는 soft objective/confidence로만 사용
- absolute NID/visible/spatial/구조/Manhattan gate 유지
- 최대 3개 finalist: objective 2% → near-tie TESL 10% → confidence 계층 선택
- objective와 confidence margin이 동시에 2% 미만이거나 support가 60% 미만이면 fail
- pairwise threshold sort를 명시적 단계 선택으로 교체해 입력 순서 결정론 보장

#### 단독 A/B

| 실행 | 선택/상태 | 핵심 해석 |
|---|---|---|
| build21 NID soft 단순 A/B | `85°`, 잘못된 내부 PASS | confidence-first false pass, 폐기 |
| build21 basin-local/objective | `168°`, 단독 진단 | 정상 방향 복구; objective `0.736508`, TESL `3966 px` |
| build20 TESL near-tie | `166°`, `FINALIST_AMBIGUOUS` | `65°`와 objective 차이 약 0.14%, 승격 금지 |

#### 3-training + 2-hold-out

최종 로직은 build17~19를 training, 수정 build20~21을 hold-out으로 사용했다.

| 항목 | 값 |
|---|---:|
| status / reason | `CANDIDATE_RT / PASS` |
| yaw / down / roll | `167° / 37.16° / 7°` |
| training | `3/3 PASS` |
| hold-out | `2/2 PASS` |
| objective margin | `0.079542` |
| confidence margin | `0.013440` |
| absolute support | PASS |
| product status | `NOT_PRODUCT_APPROVED_RT` |
| activation | `false` |

선택 후보 objective `0.752967`은 2위 `0.818036`보다 7.954% 우세하며, TESL
`13,219 px`, Manhattan vertical error `1.514°`다. 2위는 `yaw=87°`, 3위는
`yaw=-106°`다. hold-out 2D/3D 검토에서 gross 방향 반전은 없지만 LiDAR 구조점이 모든
실제 영상 edge에 정밀하게 겹치는 수준은 아니므로 제품 정답으로 해석하지 않는다.

이 결과는 이전 `image_fixed` batch의 `169°/23°/5°`와 회전 약 `14.299°`, 이동 약
`20.91 mm` 다르다. 따라서 Jenkins artifact에는 algorithm version, 입력 SHA,
intrinsic profile, lifecycle status를 함께 저장해야 한다.

개선 산출물:

```text
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build21_nid_soft_ab/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build21_basin_local_objective_first/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build20_tesl_tiebreak_final/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build17_21_dual_margin_final/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build17_21_deterministic_final/
```

full batch 관측 runtime은 약 40분이다. 정확도를 보존한 성능 개선은 고정 ±3° 축소가
아니라 ±3° 우선 탐색 후 경계 winner 축만 ±5°로 확장하는 방식으로 별도 golden A/B한다.

### 6.10 2026-08-24 build20/build21 재확인 및 finalist별 hold-out

build20/build21 CH1의 raw SHA-256과 Manual K+D 적용 prepared SHA-256은 2026-08-23
수정본과 동일했다. 따라서 테스트 목록의 파일명·pair index·해시는 변경하지 않는다.

기존 E17-21은 선택 RT만 hold-out에 적용해 `CANDIDATE_RT`를 반환했다. 다음 검증을
추가해 같은 E17-21을 재실행했다.

| 추가 시험 ID | 입력/분할 | 목적 | 기대 판정 |
|---|---|---|---|
| `E17-21-FH1` | build17~19 training, build20~21 hold-out | 최대 3개 separated finalist를 모두 hold-out에 고정 적용 | 경쟁 후보 동률이면 fail-closed |
| `E17-21-FH2` | FH1과 동일 | 같은 pass tier를 학습 동일 objective·공통 coverage·2% margin으로 비교 | 충분한 우위면 candidate, 미달이면 fail-closed |
| `E17-21-FH3` | FH2와 동일 | finalist별 Manhattan training seed prior를 hold-out에도 고정 | FH2 수치·판정 유지 및 특징 계약 일관성 |

산출물은
`generated/jenkins_scene0_ch1_20260824_build17_21_finalist_holdout`이다. 선택 167°, 경쟁
87°와 −106° 후보가 모두 training `3/3`, hold-out `2/2`를 통과했다. 최종 판정은
`INTERNAL_GATE_PASS / FINALIST_HOLDOUT_AMBIGUOUS`, `NOT_CANDIDATE_RT`다. 이전
`CANDIDATE_RT`는 historical result로만 유지한다.

후속 FH2 산출물은
`generated/jenkins_scene0_ch1_20260824_build17_21_objective_holdout`이다. 탐색 RT와
full/5°/1° score map은 FH1과 동일하며, hold-out 목적함수는 167° `0.763763`, 87°
`0.816782`, −106° `0.871447`이었다. 최소 margin `6.491% > 2%`로 최신 판정은
`CANDIDATE_RT / PASS`다. 제품 승인은 아니므로 `activation_allowed=false`다.

FH3 최신 산출물은
`generated/jenkins_scene0_ch1_20260824_build17_21_objective_holdout_prior_locked`다.
FH2와 선택 `R,t`, 목적함수, 판정과 full/5°/1° score map이 모두 동일했다. 차이는
training과 hold-out이 같은 Manhattan 소실점 축을 평가하도록 코드 계약을 고정한 것이다.

후보별 상세 수치와 출력 계약은
[`FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md`](FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md)에
기록했다.

## 7. 변경 이력

| 날짜 | 변경 |
|---|---|
| 2026-08-22 | build17 수정 영상과 build20/build21 추가분 재감사, 확장 테스트 목록 확정 |
| 2026-08-23 | Docker 회귀 9종, 추가 세트 단독 3회, 3-training/2-stress-hold-out batch 실행 완료 |
| 2026-08-23 | 단독/결합 RT, finalist margin, scene CSV, 2D/3D 시각 검토와 데이터 적격성 판정 기록 |
| 2026-08-23 | build20/build21 파일명·EXIF 수정본 감사, 단독 2회와 3-training/2-hold-out batch 재실행 |
| 2026-08-23 | 수정 전·후 pixel/score/RT/hold-out/시각화 동일성 및 최신 제품 판정 기록 |
| 2026-08-23 | basin-local NID hard gate, objective/TESL/confidence 계층 선택, dual-margin ambiguity 및 결정론적 finalist 개선 결과 추가 |
| 2026-08-24 | build20/build21 해시 재확인, E17-21-FH1 추가, separated finalist 2개 hold-out 동률 및 `NOT_CANDIDATE_RT` 판정 기록 |
| 2026-08-24 | E17-21-FH2 학습 동일 hold-out objective 적용, 최소 margin 6.491%와 `CANDIDATE_RT` 기록 |
| 2026-08-24 | E17-21-FH3 Manhattan feature prior 일관성 보강 후 1회 결과 불변 검증 |

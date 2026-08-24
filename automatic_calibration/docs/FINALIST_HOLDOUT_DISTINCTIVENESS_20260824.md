# Finalist별 hold-out 식별성 검증 보고서

- 작성일: 2026-08-24 (KST)
- 대상: Jenkins `scene0` CH1 build17~21
- 최초 binary-gate 결과: `generated/jenkins_scene0_ch1_20260824_build17_21_finalist_holdout`
- 최신 연속-score 결과: `generated/jenkins_scene0_ch1_20260824_build17_21_objective_holdout_prior_locked`
- 현재 판정: `CANDIDATE_RT / PASS`
- 제품 판정: `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`

> §1~§4의 binary pass-ratio 실험은 문제 발견 당시의 이력이다. 학습과 같은 목적함수로
> hold-out 후보를 비교한 최신 판정과 수치는 §7과 §9를 우선한다. `CANDIDATE_RT`는 제품
> 활성 승인이 아니다.

## 1. 목적

이전 로직은 training에서 선택한 RT 하나만 build20·21 hold-out에 고정 적용했다. 이
방식으로는 선택 RT가 `2/2 PASS`하더라도 80~90° 떨어진 false basin도 같은 hold-out을
통과하는지 알 수 없었다. 본 변경은 최대 3개 Ceres finalist를 모두 동일 hold-out에
고정 적용하고, 분리된 viable 후보가 선택 후보와 같은 pass ratio를 보이면 승격을
fail-closed하는 것이 목적이다.

## 2. 입력 재감사

사용자가 수정했다고 알린 두 CH1 파일은 2026-08-23에 검증한 최신 수정본과
byte-identical하다.

| build | CH1 파일 | 해상도 | SHA-256 |
|---|---|---:|---|
| 20 | `20260822_000015_CH1.jpg` | 2592×1520 | `1fe7f93e787c2de8e8a38358210aca963e9ea07dd200849af0091e47a6b256d8` |
| 21 | `20260822_225748_CH1.jpg` | 2592×1520 | `ac22e5d454c682a846373445c012d9e7550b42bc09cb631b320064335762cb72` |

Manual K+D 적용 후 prepared image도 이전 golden 실행과 같다.

| scene | 역할 | prepared SHA-256 |
|---|---|---|
| 3 / build20 | hold-out | `bc52ca8d634fbc963d026319f5b09569f317149d1e617add9b7e4bd47c38f2e9` |
| 4 / build21 | hold-out | `4b58021272d928fe47313736906b71f32a08d6f7d5d0847be3e6073b2de1af71` |

따라서 이번 판정 차이는 입력 픽셀 변경이 아니라 finalist별 hold-out 검증 추가에서만
발생한다. pair 순서는 build17/18/19 training, build20/21 hold-out으로 재확인했다.

## 3. 구현 변경

`run_real_calibration`은 staged search가 만든 finalist마다 다음을 수행한다.

1. finalist 자신의 camera profile과 refined RT를 hold-out 전체에 고정 적용한다.
2. 기존 `sceneValidationFailures`와 같은 절대 gate로 scene별 PASS/FAIL을 계산한다.
3. training/core/absolute-support를 통과한 후보만 viable competitor로 취급한다.
4. 선택 yaw와 15°보다 멀리 떨어진 viable 후보가 선택 후보 이상의 hold-out pass ratio를
   얻으면 `FINALIST_HOLDOUT_AMBIGUOUS`로 승격을 막는다.
5. 후보별 CSV와 JSON의 `finalist_holdout_validation` 배열을 저장한다.

새 주요 출력은 다음과 같다.

- `finalist_holdout_candidate_<index>.csv`
- `finalist_holdout_distinctive`
- `passing_separated_holdout_competitors`
- `search_stages[].stage=finalist_holdout_validation`

이 정책은 임계값을 낮춰 후보를 억지로 통과시키지 않는다. 아직 연속적인 hold-out
objective margin을 새로 정의하지 않았으며, 현재 단계에서는 기존 승인 gate의
비식별성을 검출하는 보수적 정책이다.

## 4. 검증 결과

### 4.1 코드·회귀

- Docker 빌드: PASS
- `challenger_m2_1_ambiguity_tests`: Test 11 추가 및 PASS
  - 분리 viable 후보가 같은 hold-out ratio이면 거절
  - 낮은 ratio, 15° 이내 동일 basin, training/core 비통과 후보는 경쟁자로 취급하지 않음
- 전체 CTest: `9/11 PASS`, 총 `2311.13 s`
- 실데이터 expected-rejection을 제외한 기본 회귀: `9/9 PASS`, 총 `85.96 s`

두 CTest 실패는 build/예외가 아니라 의도된 실데이터 fail-closed 판정이다.

| 실데이터 CTest | 결과 | 원인 |
|---|---|---|
| `verify_20260818_staged` | FAIL | 기존 `FINALIST_AMBIGUOUS`; finalist hold-out competitor 0 |
| `verify_20260819_staged` | FAIL | 선택/분리 viable 후보가 모두 hold-out 1/1 PASS → `FINALIST_HOLDOUT_AMBIGUOUS` |

현재 CTest는 실행 파일의 non-zero 알고리즘 판정을 일반 테스트 실패로 표시한다. 이후
weekly expected-rejection wrapper에서 `reason_code`까지 검사하도록 분리해야 한다.

### 4.2 build17~21 batch

이전 실행과 full/5°/1° score CSV SHA-256이 모두 같고, 선택 diagnostic RT도 수치상
완전히 같다.

```text
R = [
  [-0.9842833725713, -0.0984310271301,  0.1466205148599],
  [ 0.0120562446983,  0.7908669266090,  0.6118693907689],
  [-0.1761842485679,  0.6040205603241, -0.7772504572288]
]
t = [0.0503757073740, 0.0633742645628, 0.0594082690241] m
```

| 항목 | 결과 |
|---|---:|
| 선택 seed | yaw `167°`, down `37°`, roll `7°` |
| refined down | `37.1584°` |
| training | `3/3 PASS` |
| 선택 후보 hold-out | `2/2 PASS` |
| objective margin | `0.079542` |
| confidence margin | `0.013440` |
| separated hold-out 동률 후보 | **2개** |
| 최종 status | `INTERNAL_GATE_PASS` |
| reason | `FINALIST_HOLDOUT_AMBIGUOUS` |

후보별 결과는 다음과 같다.

| candidate | yaw | 선택 yaw와 거리 | training | hold-out | viable | 판정 |
|---|---:|---:|---:|---:|---|---|
| 7, 선택 | 167° | 0° | 3/3 | 2/2 | yes | 기준 후보 |
| 8 | 87° | 80° | 3/3 | 2/2 | yes | 동률 false-basin 가능성 |
| 9 | −106° | 87° | 3/3 | 2/2 | yes | 동률 희소 false-basin 가능성 |

hold-out 세부 수치는 broad pass gate가 왜 후보를 구별하지 못했는지 보여준다.

| yaw | build | mean edge | geometry NID | visible edge | H/V match | vertical error |
|---:|---|---:|---:|---:|---:|---:|
| 167° | 20 | **13.739 px** | 0.921414 | 674 | 12/10 | **1.846°** |
| 167° | 21 | **13.541 px** | 0.921871 | 704 | 13/6 | **1.752°** |
| 87° | 20 | 18.588 px | **0.919854** | 951 | 8/5 | 3.592° |
| 87° | 21 | 19.111 px | **0.916595** | 1039 | 3/4 | 3.632° |
| −106° | 20 | 15.781 px | 0.959790 | 117 | 1/8 | 6.914° |
| −106° | 21 | 14.188 px | 0.941755 | 102 | 0/7 | 6.924° |

167° 후보는 edge, vertical alignment, 구조 방향 수에서 더 좋아 보이지만 현 hold-out
gate는 모두 허용 범위로만 판정한다. 이 결과를 본 뒤 사후 가중치를 만들어 167°를
승격하면 검증 데이터에 과적합된다. 사전에 정의하고 독립 fixture로 검증한 연속
hold-out score/margin이 준비되기 전에는 `NOT_CANDIDATE_RT`가 맞다.

## 5. 출력 계약 주의

최초 binary-gate 실행은 실패했으므로 그 산출물의 `estimated_t_camera_lidar`는 안전상
mechanical prior를 담고 167° 결과는 아래 진단 필드에만 남는다.

- `diagnostic_candidate_t_camera_lidar`
- `visualization_t_camera_lidar`

최신 연속-score 실행은 `candidate_rt_status=CANDIDATE_RT`라
`estimated_t_camera_lidar`에 167° 후보를 기록한다. 그래도
`product_approved_rt_status=NOT_PRODUCT_APPROVED_RT`와 `activation_allowed=false`이므로
OpenSDK 활성값으로 자동 적용하면 안 된다. 소비자는 후보 보관과 제품 활성화를 별도
상태로 처리해야 한다.

## 6. 다음 고도화 순서

1. 현재 결과는 algorithm-versioned candidate regression fixture로 고정한다.
2. 구현된 연속 score와 기존 2% margin을 독립 physical/perturbation fixture에서
   재검증한다. build20/21을 보고 가중치나 임계값을 바꾸지 않는다.
3. 물리 기준 또는 확실한 positive/negative RT perturbation fixture로 false acceptance를
   측정한다.
4. 원본·동시 수집, 정적 장면, 동일 profile의 독립 hold-out을 최소 3쌍 확보한다.
5. 10회 반복성 및 독립 RT 오차를 통과한 뒤에만 `CANDIDATE_RT`, 그 이후에만
   `PRODUCT_APPROVED_RT`를 검토한다.
6. Jenkins daily에서는 빠른 Core/Challenger를 실행한다. weekly에서는 20260818의
   exit 3 + `FINALIST_AMBIGUOUS`와 20260819의 candidate 회귀를 분리 확인한다.

## 7. 학습 동일 목적함수 기반 hold-out 고도화

### 7.1 변경 이유와 계산식

Binary scene gate는 각 후보가 허용 범위 안에 있는지만 판단한다. 이를 후보 간 식별
점수로 사용하지 않고, Core가 학습 후보를 평가할 때 이미 사용하는 목적함수를 그대로
hold-out에 적용했다.

```text
J = 0.25·J_edge + 0.55·J_geometry_NID
  + 0.20·J_structural + 0.15·J_Manhattan
  + 0.25·J_coverage
```

현재 signal NMI와 direction prior의 가중치는 0이다. Edge는 visible point 수,
구조선은 기존 score weight로 장면 간 집계하고 NID는 장면 평균을 사용한다. Coverage는
후보마다 자기 자신을 분모로 쓰지 않고, 같은 hold-out의 모든 finalist가 공유하는 최대
visible edge/NID point/spatial cell을 분모로 사용한다.

판정 순서는 다음과 같다.

1. training/core/absolute-support를 통과한 15° 초과 분리 후보만 경쟁자로 본다.
2. hold-out pass ratio가 더 낮은 후보는 하위 tier로 둔다.
3. 같은 pass-ratio tier에서는 낮은 목적함수가 우수하다.
4. `(J_competitor - J_selected) / |J_competitor|`가 기존
   `minimum_multistart_objective_margin=0.02` 미만이면 fail-closed한다.

새 가중치나 build20/21 전용 임계값은 추가하지 않았다.

### 7.2 build17~21 최신 결과

전역/5°/1° score map SHA-256과 RT는 최초 실행과 동일했다. 즉 탐색 결과는 바뀌지 않고
hold-out 판정만 연속 점수로 정밀해졌다.

| yaw | hold-out | 목적함수 | 선택 대비 margin | 판정 |
|---:|---:|---:|---:|---|
| 167° | 2/2 | **0.763763** | 기준 | 선택 |
| 87° | 2/2 | 0.816782 | **6.491%** | 충분히 열세 |
| −106° | 2/2 | 0.871447 | **12.357%** | 충분히 열세 |

최소 margin `6.491% > 2%`이므로 최신 상태는 다음과 같다.

- `status=CANDIDATE_RT`
- `reason_code=PASS`
- `finalist_holdout_distinctive=true`
- `ambiguous_separated_holdout_competitors=0`
- `product_approved_rt_status=NOT_PRODUCT_APPROVED_RT`
- `activation_allowed=false`

167°의 우위는 주로 edge와 구조선/Manhattan에서 발생했다. 87°는 coverage와 geometry
NID가 조금 좋았지만 edge·구조선 손실이 더 컸다. −106°는 구조선 항목 일부가 좋았으나
spatial coverage와 Manhattan/NID가 크게 나빴다.

### 7.3 독립 회귀 결과

| 데이터 | 최신 결과 | 핵심 근거 |
|---|---|---|
| 20260818 | `FAIL / FINALIST_AMBIGUOUS` | training finalist 모호성 유지; hold-out 로직 이전 단계에서 안전 거절 |
| 20260819 | `CANDIDATE_RT / PASS` | yaw 80°, 분리 165° 경쟁 후보 대비 hold-out margin 9.882% |
| build17~21 | `CANDIDATE_RT / PASS` | yaw 167°, 두 분리 경쟁 후보 대비 최소 margin 6.491% |

20260818/19의 full·1° score map 해시는 수정 전과 동일했다. 20260819의 상태 변화는
탐색 변경이 아니라 기존 binary 동률을 연속 목적함수로 해소한 결과다. 물리 참값이 없는
두 `CANDIDATE_RT`는 제품 정확도 증명이 아니며 독립 reference 검증이 여전히 필요하다.

### 7.4 코드·테스트 결과

- `PoseSceneMetrics`가 학습 목적함수의 raw component를 보존한다.
- `summarizeCalibrationPoseScenes()`가 학습과 같은 장면 집계와 공통 coverage 목적함수를
  계산한다.
- 후보별 JSON에 objective component, coverage, margin, ambiguity 판정을 저장한다.
- Core 수치 경계와 Challenger pass-ratio/margin 경계 테스트: PASS.
- 장시간 실데이터 제외 회귀: `9/9 PASS`, `88.52 s`.
- 20260818은 weekly expected-rejection, 20260819는 weekly candidate regression으로
  CTest label을 분리했다.

## 8. 수정 이력

| 날짜 | 변경 |
|---|---|
| 2026-08-24 | build20/build21 raw·prepared 해시 및 pair 순서 재감사 |
| 2026-08-24 | 최대 3개 finalist의 candidate-specific hold-out 검증 구현 |
| 2026-08-24 | 20260818/19 회귀와 build17~21 3-training/2-hold-out 재실행 |
| 2026-08-24 | 기존 `CANDIDATE_RT`를 `FINALIST_HOLDOUT_AMBIGUOUS`로 정정 |
| 2026-08-24 | 학습 동일 연속 목적함수·공통 coverage·2% margin 구현 |
| 2026-08-24 | build17~21 및 20260819 `CANDIDATE_RT`, 20260818 expected rejection 재검증 |
| 2026-08-24 | Manhattan 특징 prior의 training/hold-out 불일치 수정, 고정 prior 회귀 추가 및 1회 실데이터 재검증 |
| 2026-08-24 | 1회 실행 제한 해제 후 성공·예상 거절 공통 weekly JSON 계약 2/2 재검증 |

## 9. Manhattan 특징 prior 일관성 감사 및 1회 재검증

### 9.1 발견한 문제

Training의 `calibrateExtrinsicMultiScene()`은 각 finalist를 만든 `seed.prior`를 기준으로
영상 소실점 후보 중 수직축을 한 번 선택하고, 그 특징을 Ceres 동안 고정한다. 반면 기존
fixed-pose/hold-out 평가 함수는 평가 대상인 **refined candidate RT 자체**를 수직축 선택
prior로 다시 사용했다.

이 차이는 다음 위험을 만든다.

1. 학습과 hold-out이 서로 다른 영상 특징을 점수화할 수 있다.
2. 잘못된 후보도 자신의 방향과 가까운 다른 소실점 축을 다시 선택해 Manhattan 점수를
   유리하게 만들 수 있다.
3. 따라서 동일 목적함수라고 표시해도 실제 비교 대상이 동일하지 않을 수 있다.

이는 현재 데이터의 RT가 틀렸다는 증거가 아니라, 후보 비교 계약의 일관성 결함이다.

### 9.2 수정 내용

- `evaluateCalibrationPoseScenes()`에 명시적 `manhattan_feature_prior`를 전달할 수 있게 했다.
- 각 finalist의 training scene, hold-out scene, scene pass-ratio 계산은 모두 해당
  finalist를 생성한 동일 `seed.prior`를 사용한다.
- refined candidate RT는 투영, z-buffer, Edge/NID/구조선/Manhattan 잔차 계산에만 쓰고,
  어떤 소실점 축을 수직으로 선택할지 다시 바꾸지 않는다.
- 결과 JSON의 알고리즘 계약에
  `manhattan_image_feature_prior_policy=fixed_per_finalist_training_seed_prior_reused_for_training_and_holdout`
  를 기록하도록 했다.
- 직교하는 두 소실점 축을 가진 합성 영상에서 명시적 prior가 실제 선택 축을 고정하는
  회귀 테스트를 `calibration_core_tests.cpp`에 추가했다.

### 9.3 1회 실행 결과

사용자 요청에 따라 아래 세 묶음을 수정본으로 **각각 1회만** 실행했으며 결과에 맞춘
threshold 조정이나 재시도는 하지 않았다.

이번 세 실데이터 JSON은 prior 고정 로직이 포함된 binary로 생성됐다. 다만 위
machine-readable policy 필드는 1회 실행 완료 뒤 추가하고 runner만 재빌드했으므로 이번
세 `calibration_result.json`에는 아직 존재하지 않는다. 사용자 요청대로 실데이터를 두
번째로 실행하지 않았으며, 다음 실행부터 이 필드가 기록된다.

| 데이터 | training / hold-out | 결과 | 선택 방향 | 선택 hold-out J | 최소 분리 후보 margin |
|---|---:|---|---|---:|---:|
| build17~21 | 3 / 2 | `CANDIDATE_RT / PASS` | yaw `167°`, down `37.1584°`, roll `7°` | `0.763763` | `6.491%` |
| 20260818 | 3 / 1 | `FAIL / FINALIST_AMBIGUOUS` | 진단 yaw `175°`, down `20.0067°`, roll `3°` | `0.876239` | 해당 없음 |
| 20260819 | 2 / 1 | `CANDIDATE_RT / PASS` | yaw `80°`, down `30.0006°`, roll `4°` | `0.733585` | `9.882%` |

build17~21의 finalist별 결과도 유지됐다.

| yaw | hold-out | J | 선택 대비 margin | viable / ambiguous |
|---:|---:|---:|---:|---|
| `167°` | `2/2` | `0.763763` | 기준 | yes / no |
| `87°` | `2/2` | `0.816782` | `6.491%` | yes / no |
| `−106°` | `2/2` | `0.871447` | `12.357%` | yes / no |

최신 build20/build21 2D reprojection과 3D preview도 직접 확인했다.

| hold-out | projected point | aligned / visible edge | mean edge | 구조선 matched / visible | Manhattan vertical error |
|---|---:|---:|---:|---:|---:|
| build20 | `3955` | `494 / 674` (`73.29%`) | `13.739 px` | `25 / 27` | `1.846°` |
| build21 | `3958` | `519 / 704` (`73.72%`) | `13.541 px` | `22 / 23` | `1.752°` |

두 장면 모두 gross 90°/180° 반전이나 바닥만 향하는 투영은 보이지 않았다. 캐비닛 상단,
벽 코너와 일부 가구 경계는 같은 방향으로 겹쳤다. 그러나 바닥·의자·책상 주변의 LiDAR
depth edge 상당수는 영상 edge와 일대일로 붙지 않는다. 따라서 육안 검토도 “방향 후보는
합리적이지만 정밀 물리 RT 참값은 미입증”이라는 lifecycle 판정을 지지한다.

수정 전후 비교 결과는 다음과 같다.

- 세 실행 모두 선택 `R`, `t`의 최대 원소 차이: `0`
- 세 실행 모두 status, reason, selected index, hold-out objective: 동일
- 각 데이터의 full/5°/1° score CSV SHA-256: 동일
- 수정 직후 빠른 회귀: `9/9 PASS`, `77.19 s`
- 새 prior 선택 회귀를 포함한 Core 단위 테스트: `1/1 PASS`, `5.51 s`

따라서 이번 변경은 현재 세 fixture의 답을 바꾼 score tuning이 아니라, 이후 다른 소실점
구조를 만났을 때 후보가 hold-out 특징을 자기에게 유리하게 재선택하지 못하도록 만든
평가 안전성 보강이다. 최신 산출물은 다음과 같다.

```text
automatic_calibration/generated/jenkins_scene0_ch1_20260824_build17_21_objective_holdout_prior_locked/
automatic_calibration/generated/verify_20260818_objective_holdout_prior_locked_20260824/
automatic_calibration/generated/verify_20260819_objective_holdout_prior_locked_20260824/
```

### 9.4 1회 제한 해제 후 정식 weekly 회귀

초기 1회 비교는 threshold 사후 조정 없이 변경 전후 불변성을 확인하기 위한 이력이었다.
이후 사용자가 1회 제한을 해제함에 따라 20260818/19를 Jenkins와 같은 weekly CTest
경로로 다시 실행했다. 성공 케이스와 예상 거절 케이스가 서로 다른 방식으로 검사되던
허점을 제거하고, 두 테스트 모두 `tests/verify_real_calibration_result.cmake`를 사용한다.

공통 래퍼가 검사하는 계약은 다음과 같다.

- 프로세스 exit code
- `status`, `reason_code`, `candidate_rt_status`
- `product_approved_rt_status=NOT_PRODUCT_APPROVED_RT`
- `activation_allowed=false`
- `algorithm.manhattan_image_feature_prior_policy`

| CTest | 기대/실제 계약 | 결과 | 소요 시간 |
|---|---|---|---:|
| `verify_20260818_staged` | exit 3, `FAIL / FINALIST_AMBIGUOUS`, `NOT_CANDIDATE_RT` | PASS | `1240.90 s` |
| `verify_20260819_staged` | exit 0, `CANDIDATE_RT / PASS`, `CANDIDATE_RT` | PASS | `1027.19 s` |

병렬 실행 결과는 `2/2 PASS`, real time `1240.91 s`였다. 빠른 회귀도 직전에
`9/9 PASS (52.26 s)`했다. 새 weekly 산출물에는 위 prior policy가 실제 기록됐고,
prior-locked 1회 산출물과 각 데이터의 full/5°/1° score CSV SHA-256이 모두 같다.

```text
automatic_calibration/generated/verify_20260818/
automatic_calibration/generated/verify_20260819/
```

### 9.5 최종 평가와 남은 문제

- Calibration Core의 staged 후보 추정과 동일 목적함수 hold-out 비교는 코드·회귀 수준에서
  일관성을 확보했다.
- build17~21과 20260819는 `CANDIDATE_RT`일 뿐 물리 정확도 승인이 아니다.
- 20260818의 모호성 거절은 정상적인 fail-closed 결과다.
- 기존 2% margin은 독립 jig/CAD/survey 또는 LiDAR-visible target 기반 positive/negative
  fixture에서 false acceptance를 측정하기 전까지 제품 승인 기준으로 확정할 수 없다.
- 원본 동시 hold-out 3쌍 이상, 동일 설치 10회 반복성, manifest 시간·해시·profile gate,
  알고리즘/binary provenance가 여전히 필요하다.
- 현재 `data/jenkins-capture/scene0`의 manifest는 build/session/count/created-at 수준이며,
  camera capture UTC, scan start/end UTC, 파일 SHA-256, channel profile, installation epoch가
  없다. 따라서 이 gate는 기존 fixture만으로 완료할 수 없고 수집 producer 계약 보강이
  선행돼야 한다.
- 모든 결과는 `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`를 유지한다.

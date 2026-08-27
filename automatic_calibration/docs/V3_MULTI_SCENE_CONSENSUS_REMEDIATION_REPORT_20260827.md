# V3 Hybrid Analyzer 다중 장면 합의 보정 보고서

- 작성일: 2026-08-27
- 브랜치: `codex/exp-hybrid-analyzer-v3-20260827`
- 선행 V3 커밋: `21ed4b7`
- 기준 B0: `develop@f684cd66`
- 대상: CH1 camera–pan/tilt 1D LiDAR markerless extrinsic calibration
- 제품 상태: **NOT_PRODUCT_APPROVED_RT**

## 1. 결론

초기 V3는 Jenkins 구성에서 계산량을 줄이면서 B0와 가까운 RT를 찾았지만, 2026-08-18
구성에서는 장면별 analyzer 후보가 불안정한데도 첫 training scene만 사용해 잘못된
`yaw≈-49°` RT를 `CANDIDATE_RT/PASS`로 통과시켰다.

후속 수정에서는 모든 training image–scan pair를 분석하고, 첫 장면 proposal이 모든 다른
장면에서 circular yaw±15°, down±15° 안에 지지를 받을 때만 bounded search로 전달한다.
공통 proposal이 없으면 `MULTI_SCENE_PROPOSAL_INCONSISTENT`로 full B0 search에 fallback한다.

결과는 다음과 같다.

- Jenkins: 공통 basin 2개 유지, 최종 RT 유지, projection-scene 평가 336→64
- 2026-08-18: 공통 basin 0개, 잘못된 bounded RT 차단, B0 fallback 후 ambiguity FAIL
- E2E 두 구성에서 unsafe non-fallback은 0건이 됐지만 fallback 비율은 50%

따라서 안전성은 의미 있게 개선됐지만 정확도·fallback·설치 다양성 기준은 여전히 미달이다.

## 2. 발견된 결함

### 2.1 보정 전 동작

보정 전 E2E는 다음 코드 경로를 사용했다.

1. 첫 training image–scan 한 쌍만 analyzer에 입력
2. Top-3 proposal 각각에서 down covariance seed 생성
3. bounded 5°/1° search와 Ceres 수행
4. training/hold-out gate로 최종 후보 선택

2026-08-18 첫 장면 proposal은 `(-44°,41°)`, `(-2°,75°)`, `(177°,25°)`였다. 첫 번째
proposal 주변의 calibration objective가 장면 일부의 반복 구조와 맞아 `yaw≈-49°` 후보가
통과했다. 그러나 다른 training 장면의 같은 yaw basin은 down이 각각 약 40°와 21°로
20° 벌어졌다. 단일 장면 분석으로는 이 불일치를 볼 수 없었다.

### 2.2 영향

- lifecycle: `CANDIDATE_RT`
- reason: `PASS`
- 잘못된 orientation: yaw `-49.062°`, down `32.673°`, roll `9.547°`
- 기존 B0 진단 결과와 차이: 회전 `145.522°`, 이동 `117.076 mm`

hold-out 한 장도 같은 반복 구조를 공유하므로 false acceptance를 막지 못했다. 이는 analyzer
Top-K를 줄이는 문제보다 먼저 해결해야 하는 제품 안전 결함이다.

## 3. 구현한 보정

수정 파일:

- `automatic_calibration/apps/run_real_calibration.cpp`

처리 계약:

1. hold-out을 제외한 모든 `calibration_observations`를 analyzer에 입력한다.
2. 장면별 analyzer JSON/CSV/PNG를 `orientation_analyzer/scene_N/`에 보존한다.
3. yaw 거리는 -180°/180° 경계를 고려한 circular distance로 계산한다.
4. 첫 장면 proposal마다 이후 모든 장면에서 yaw±15°, down±15° 지지 후보를 찾는다.
5. 모든 장면이 지지하는 proposal만 `consensus_proposals`로 남긴다.
6. 합의 후보가 없으면 bounded search를 실행하지 않고 B0 fallback한다.
7. 결과 JSON에 scene 수, 장면별 proposal, 합의 tolerance, 합의 후보 수, analyzer 합산 시간을 기록한다.

추가 결과 필드:

- `orientation_analyzer.scene_count`
- `orientation_analyzer.scenes[]`
- `orientation_analyzer.consensus_yaw_tolerance_deg`
- `orientation_analyzer.consensus_down_tolerance_deg`
- `orientation_analyzer.consensus_proposal_count`
- `orientation_analyzer.reference_scene_proposals`
- `orientation_analyzer.proposals`

현재 방식은 score curve를 합치는 estimator가 아니라, 불일치 후보를 제거하는 hard safety gate다.

## 4. Jenkins build22~24 재검증

입력:

- build22, build23: training
- build24: hold-out
- Docker CPU 제한: 2 core
- K·D: manual ChArUco fixed profile
- image: raw 입력 후 pipeline undistort

출력:

- `automatic_calibration/generated/v3_case_c_build22_24_multiscene_consensus/`

결과:

| 항목 | r4 단일 장면 | r5 다중 장면 |
|---|---:|---:|
| consensus proposal | 해당 없음 | 2 |
| analyzer fallback | false | false |
| analyzer runtime | 1,861.041 ms | 3,044.547 ms |
| pipeline runtime | 76,166.329 ms | 53,494.429 ms |
| internal yaw candidate | 48 | 32 |
| projection-scene evaluation | 96 | 64 |
| B0 336 대비 감소 | 71.429% | 80.952% |
| status/reason | CANDIDATE_RT/PASS | CANDIDATE_RT/PASS |

r5 analyzer 비중은 `3,044.547 / 53,494.429 = 5.691%`로 10% 이하 조건을 통과했다.
pipeline은 단일 실행 기준 r4보다 29.766% 짧았다. 반복 benchmark 전에는 이 값을 제품
wall-time 개선율로 확정하지 않는다.

최종 RT는 r4와 동일한 basin을 유지했다.

- 최종 분해 orientation: yaw `178.251°`, down `42.561°`, roll `0.023°`
- B0 대비 회전 차이: `2.394°`
- B0 대비 이동 차이: `4.155 mm`
- selected multistart yaw: `177°`

## 5. 2026-08-18 재검증

입력:

- 4 pairs
- 앞 3 pairs: training
- 마지막 1 pair: hold-out
- Docker CPU 제한: 2 core

출력:

- `automatic_calibration/generated/v3_repeat_20260818_multiscene_consensus/`

장면별 공통 후보가 없어 다음 상태가 기록됐다.

- `scene_count=3`
- `consensus_proposal_count=0`
- `fallback_reason=MULTI_SCENE_PROPOSAL_INCONSISTENT`
- `orientation_analyzer_fallback_triggered=true`
- `orientation_analyzer_engaged=false`

fallback 결과:

- pipeline runtime: `1,221,273.054 ms` = 약 20분 21초
- analyzer runtime: `4,460.510 ms` = 전체의 `0.365%`
- bounded candidate/projection evaluation: 0
- B0 finalist search metadata: selected multistart yaw `175°`, camera down `20.007°`, prior optical roll `3°`
- 최종 상태: `FAIL`
- 최종 reason: `FINALIST_AMBIGUOUS`

기존 B0 진단 결과와 비교하면 fallback 후보 차이는 회전 `6.380°`, 이동 `7.203 mm`다.
이전 잘못된 V3 후보의 `145.522°/117.076 mm`보다 B0 basin을 회복했지만, finalist가
모호하므로 activation 가능한 RT가 아니다. exit code 1은 실행 장애가 아니라 이 FAIL 판정이다.

## 6. 합격 기준 최신 판정

| 기준 | 최신 결과 | 판정 |
|---|---:|---|
| standalone B0 basin recall@3 ≥99% | 3/6 = 50% | FAIL |
| E2E unsafe non-fallback | 0/2 구성 | PASS, 표본 제한 |
| 일반 구성 fallback ≤20% | 1/2 = 50% | FAIL |
| analyzer ≤ pipeline 10% | Jenkins 5.691%, fallback 0.365% | PASS |
| projection 평가 ≥70% 감소 | Jenkins 80.952% | PASS |
| 최종 RT B0/manual 대비 1~2° | Jenkins 2.394° | FAIL(근접) |
| 서로 다른 설치 구성 3개 | 2/3 | PENDING |
| textureless fallback | 단위 테스트 PASS | PASS |

`unsafe_non_fallback=0`은 두 E2E 구성의 기능 검증 결과이지 99% 신뢰도 증명이 아니다.

## 7. B0·T1·T2·V3의 현재 역할

- B0: offline oracle와 fail-safe fallback. 계산량이 크고 절대 ground truth는 아님.
- T1: 구조선/vanishing/Manhattan 가설의 독립 실험. down/roll predictor 부품만 V3가 재사용.
- T2: panorama raster/signature 가설의 독립 실험. 전역 perspective remap 비용 때문에 동결.
- V3: T2 raster + T1 구조 분석 + remap 없는 1D signature + bounded calibration + B0 fallback.

T1/T2는 결과 재현을 위해 동결하고 V3만 후속 실험한다. B0 fallback은 제거하지 않는다.

## 8. 득과 실

득:

- 전역 perspective remap 216회 제거
- Jenkins projection-scene 평가 80.952% 감소
- 실제 내부 후보 수와 장면별 projection 수를 분리 계측
- 잘못된 단일 장면 RT의 false PASS를 다중 장면 합의로 차단
- B0 fallback과 ambiguity gate가 fail-closed로 동작함을 실데이터로 확인

실:

- hard consensus 때문에 현재 구성 fallback 비율이 50%
- fallback 1회가 약 20분으로 CV5 동기 경로에 부적합
- standalone Top-3 recall 자체는 여전히 50%
- roll predictor 잔여 오차 때문에 Jenkins가 B0 대비 2.394°
- 설치 구성은 2개뿐이고 같은 장소 계열이라 일반화 근거가 부족

## 9. 다음 수정 우선순위

1. hard Top-K 교집합을 circular score-curve median/trimmed mean으로 발전시킨다.
2. unsafe non-fallback 0을 유지하면서 fallback을 20% 이하로 줄이는 threshold를 학습과 무관한 hold-out에서 고정한다.
3. yaw/down tolerance 15°를 구성 C에서 그대로 검증한다. 새 데이터마다 threshold를 바꾸지 않는다.
4. roll은 VP 한 점이 아니라 line residual/support 기반 robust estimator로 개선한다.
5. bounded 일반 경로와 B0 fallback 경로의 median/p95 runtime을 각각 최소 5회 측정한다.
6. 다른 벽 방향·높이·구조 분포의 구성 C에서 최소 3 training + 1 sealed hold-out pair를 수집한다.
7. 정확도 조건 통과 후 1D signature는 ARM NEON, fallback 병목은 projection/NID loop를 우선 최적화한다.

## 10. 최종 판단

V3 r5는 “analyzer로 부하를 줄이되 불확실하면 안전하게 물러난다”는 제품 방향에 r4보다
가깝다. 특히 145° 이상 틀린 RT의 false activation 가능성을 막은 것이 가장 큰 개선이다.

그러나 fallback 50%, 회전 2.394°, 설치 2/3 때문에 상용 RT 자동화 로직 완성으로 판단할
수 없다. 현재 산출물은 `CANDIDATE_RT` 또는 `FAIL`이며, `PRODUCT_APPROVED_RT`로 승격하지 않는다.

## 11. 검증 실행

- Docker image: `auto-calib-dev:ubuntu-latest`
- compile: `run_real_calibration` PASS
- targeted: `hybrid_orientation_analyzer_tests` 1/1 PASS, 0.43 s
- regression: `ctest --test-dir build-v3 --output-on-failure -E '^verify_'`
- regression 결과: 7/7 PASS, 0 FAIL, 124.75 s
- Jenkins E2E: exit 0, `CANDIDATE_RT/PASS`
- 2026-08-18 E2E: exit 1, `FAIL/FINALIST_AMBIGUOUS` — 의도한 fail-closed 결과

실데이터 E2E의 exit code는 프로세스 장애가 아니라 calibration lifecycle 판정값으로 해석한다.

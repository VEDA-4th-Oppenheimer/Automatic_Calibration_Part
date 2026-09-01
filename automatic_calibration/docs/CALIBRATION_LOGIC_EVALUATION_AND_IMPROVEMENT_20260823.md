# Automatic Calibration 로직 재평가·개선 및 검증 보고서

> **2026-08-24 binary 중간 판정:** 이 문서의 `CANDIDATE_RT`는 선택 RT만 hold-out에 적용한
> 당시 결과다. finalist 3개를 모두 같은 build20·21에 적용하자 87°와 −106° 분리
> 후보도 2/2 PASS했다. 현재 판정은
> `INTERNAL_GATE_PASS / FINALIST_HOLDOUT_AMBIGUOUS`, `NOT_CANDIDATE_RT`다. 최신 근거는
> [`FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md`](FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md)를
> 우선한다.

> **2026-08-24 후속 고도화:** 위 판정은 binary pass-ratio 단계의 중간 결과다. 학습과
> 같은 연속 목적함수와 공통 coverage를 적용한 최소 경쟁 margin은 6.491%로, 최신
> build17~21 상태는 `CANDIDATE_RT / PASS`다. 제품 상태는 계속
> `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`다.

- 작성일: 2026-08-23
- 적용 범위: `automatic_calibration` CH1 staged targetless extrinsic calibration
- 평가 데이터: Jenkins `scene0`의 build17~build21
- 기준 상태: **후보 RT 산출 가능, 제품 RT 자동화는 미완성**
- 최신 실행: `generated/jenkins_scene0_ch1_20260823_build17_21_deterministic_final`

## 1. 결론

현재 구현은 고정된 Manual ChArUco `K+D`와 자연 장면의 2D 영상·3D LiDAR 데이터를
사용해 `T_camera_lidar=(R,t)` 후보를 자동 추정하고, training/hold-out 내부 gate까지
검사할 수 있다. 그러나 독립 물리 기준과 설치 epoch 간 반복성 승인을 아직 통과하지
않았으므로 **제품에 자동 적용 가능한 Calibration Core 완성 상태는 아니다.**

2026-08-23 개선 전 build21의 올바른 방향 후보는 full coarse search에서 발견됐지만,
서로 다른 카메라 시야 방향의 최대 NID 점 수를 local search의 hard gate 분모로 사용해
정상 basin 전체가 탈락했다. 이 hard gate를 basin-local 기준으로 고치자 정상 방향
`yaw≈168°`가 회복됐다. 이어 confidence 우선 선택이 `yaw≈85°` false basin을 고르는
문제를 matching objective와 절대 구조 증거 중심의 계층 선택으로 수정했다.

최종 5-pair 검증은 다음 상태다.

| 항목 | 결과 |
|---|---|
| training | build17~19, `3/3 PASS` |
| hold-out | 수정 build20~21, `2/2 PASS` |
| 선택 방향 | `yaw=167°`, `down≈37.16°`, `optical roll=7°` |
| finalist objective margin | `0.079542` (`7.954%`) |
| finalist confidence margin | `0.013440` (`1.344%`) |
| 수명주기 | `CANDIDATE_RT` |
| 제품 상태 | `NOT_PRODUCT_APPROVED_RT` |
| 활성화 | `activation_allowed=false` |

따라서 이번 결과의 의미는 **정상 방향 basin을 다시 찾았고 제한된 동일 환경 hold-out을
설명하는 후보 RT를 만들었다**는 것이다. 픽셀 단위 ground truth나 설치 장소가 바뀐
독립 재현성을 증명한 것은 아니다.

## 2. 감사에서 확인한 문서·구현 불일치

### 2.1 과거 종합 보고서의 과장

`FINAL_CALIBRATION_EVALUATION_REPORT.md`의 다음 표현은 2026-08-23 증거와 양립하지 않는다.

- “5대 핵심 결함 완전 해결”
- “false basin 완전 제거”
- “최종 확인 및 승인”
- camera-center bracket residual을 독립 물리 정확도처럼 해석한 부분

camera center는 강한 translation prior로 최적화에 입력되므로
`t + R*C_camera ≈ 0`이 작다는 사실은 **prior가 지켜졌다는 검사**이지 RT 정답의 독립
증거가 아니다. 해당 보고서는 당시 R4 역사 기록으로 유지하고 본 문서를 최신 판정 기준으로
지정한다.

### 2.2 실제 Ceres 실행 횟수

과거 정책에는 “최종 seed 1개만 Ceres”라고 적혀 있었지만 실제 staged runner는 서로
분리된 최대 3개 basin을 각각 Ceres로 refinement한 뒤 finalist를 비교한다. false basin을
결과 단계에서 비교·거절하려면 현재 방식이 필요하므로 정책 문서를 실제 구현에 맞췄다.

### 2.3 LDC `unknown`의 의미

현재 입력은 `image-distortion-state=raw`이고 Manual ChArUco `K+D`로 undistort한다.
이 raw 계약이 수집·파일 provenance로 명시됐다면 카메라 UI의 LDC 항목을 찾지 못해
`ldc_enabled=unknown`인 것만으로 결과 전체를 diagnostic-only로 낮출 필요는 없다.
반대로 raw/rectified 상태 자체가 불명확하면 중복 또는 누락 보정 위험이 있으므로 반드시
실패 처리한다.

## 3. 실패 원인 분석

### 3.1 서로 다른 FOV의 NID 수를 hard gate로 비교

기존 local 5°/1° 탐색은 전체 360° coarse 후보 중 가장 많은 NID 투영 점 수를
`global_reference_nid_projected_points`로 사용했다. 카메라가 보는 방향이 달라지면 투영되는
점 수와 표면 면적도 크게 달라진다. 따라서 점 수가 적지만 구조가 잘 맞는 정상 방향도
`minimum_relative_nid_coverage=0.5`를 넘지 못하고
`COARSE_OVERLAP_INSUFFICIENT`로 탈락했다.

build21에서는 full coarse의 정상 후보가 `yaw≈165°`, `down≈30°` 부근에서 좋은 raw
score와 연속 지지를 가졌지만 local 단계가 이를 모두 제거했다. 이는 센서 데이터 부족이
아니라 **비교 분모의 범위가 잘못된 gate 설계**였다.

### 3.2 hard gate 해제만으로는 false pass

진단 A/B에서 `--minimum-relative-nid-coverage 0.0`만 적용하면 정상 `yaw≈168°` 후보가
살아났지만, coverage 기반 confidence가 높은 `yaw≈85°` 후보가 선택되어
`INTERNAL_GATE_PASS`가 됐다.

| build21 finalist | matching objective | confidence | mean edge | vertical error | TESL |
|---|---:|---:|---:|---:|---:|
| 정상 방향 `168°` | **0.736508** | 0.755533 | **14.300 px** | **2.817°** | **3,966 px** |
| false 방향 `85°` | 0.769363 | **0.782845** | 18.019 px | 7.407° | 828 px |

정상 후보가 matching objective, edge, Manhattan vertical, TESL에서 우세했지만 점 수 기반
confidence 하나 때문에 뒤집혔다. 따라서 NID hard gate만 없애는 접근은 안전하지 않다.

### 3.3 단일 장면의 본질적 모호성

build20은 정상 방향으로 보이는 `166°`와 false 방향 `65°`의 objective 차이가 약
`0.14%`뿐이었다. `166°` 후보의 TESL은 `4,631 px`, `65°`는 `1,723 px`로 구조 설명은
정상 후보가 강했지만 confidence는 반대였다. 이 경우 정상 방향을 진단 후보로 표시하되
`FINALIST_AMBIGUOUS`로 실패하는 현재 정책이 타당하다. 더 세밀한 yaw step만으로는
관측 정보가 늘지 않으므로 해결되지 않는다.

## 4. 적용한 코드 개선

### 4.1 basin-local overlap gate

- 5°/1° local 단계에서는 NID 상대 coverage의 기준을 해당 local yaw window 안에서 계산한다.
- 전체 360°의 NID 최대값은 final objective/confidence의 soft 진단값으로만 유지한다.
- final Ceres 단계에서는 global NID 상대값으로 hard reject하지 않는다.
- 절대 NID/visible edge, 공간 분포, 구조 방향, Manhattan, TESL gate는 유지했다.

즉 임계값을 낮춰 통과시킨 것이 아니라 **hard gate가 비교해야 할 모집단을 올바르게
제한**했다.

### 4.2 finalist 계층 선택

최대 3개 Ceres finalist는 다음 순서로 비교한다.

1. training scene validation
2. Calibration Core success
3. 선택 가능 자세 범위
4. absolute support gate
5. objective가 차순위보다 2% 이상 우세하면 낮은 objective
6. objective가 2% 미만으로 근접하면 TESL이 10% 이상 큰 후보
7. 위 둘도 동률이면 multi-criteria confidence
8. 마지막 결정론적 tie-break는 objective와 입력 index

선택 후 15°보다 떨어진 최강 경쟁 후보와 비교해 다음 두 조건을 검사한다.

- objective margin과 confidence margin이 **동시에** 각각 2% 미만이면
  `FINALIST_AMBIGUOUS`
- 선택 후보의 visible edge 또는 NID 절대 지지가 경쟁 후보의 60% 미만이면
  `FINALIST_AMBIGUOUS`

objective와 confidence 중 하나의 독립 신호가 충분히 분리되고 절대 지지가 유지되면 내부
후보를 허용한다. 어느 한 숫자만 높다고 승인하지 않는다.

### 4.3 결정론적 3-candidate 선택

“두 후보 간 2%” 조건을 `std::sort` comparator 안에서 사용하면 세 후보 A/B/C가
`A>B`, `B>C`, `C>A` 순환 관계를 만들 수 있다. 이를 다음 명시적 절차로 교체했다.

1. 동일 품질 tier 후보를 모은다.
2. objective 최솟값과 차순위를 비교한다.
3. objective 근접 집합에서 TESL을 비교한다.
4. 필요할 때만 confidence를 비교한다.
5. 선택 후보를 제거하고 같은 절차로 차순위를 만든다.

회귀 테스트는 순환 점수 fixture의 6가지 입력 순서가 모두 같은 후보를 선택하는지 확인한다.

### 4.4 결과 provenance

JSON에 다음 필드를 기록한다.

- `finalist_objective_margin`
- `finalist_confidence_margin`
- `absolute_support_pass`
- `global_max_visible_edges`, `global_max_nid_points`
- local-hard/global-soft NID 정책 문자열
- 실제 finalist selection policy

## 5. A/B 실행 결과

| 단계 | 데이터 | 결과 | 해석 |
|---|---|---|---|
| 기존 로직 | build21 | `COARSE_OVERLAP_INSUFFICIENT` | 정상 basin이 global NID hard gate로 제거 |
| NID hard gate 단순 해제 | build21 | 잘못된 `85°`, 내부 PASS | confidence-first false pass, 폐기 |
| basin-local + objective 우선 | build21 | `168°`, `FINALIST_AMBIGUOUS` | 방향 회복, 단독 scene은 보수적 거절 |
| TESL near-tie | build20 | `166°`, `FINALIST_AMBIGUOUS` | 정상 방향 진단, 근소한 objective로 승격 금지 |
| dual-margin batch | build17~21 | `167°`, `CANDIDATE_RT` | 3 training + 2 hold-out 내부 통과 |

`build21_nid_soft_ab`는 잘못된 방향을 PASS시킨 반례이므로 제품 후보로 사용하지 않는다.
`build20_tesl_tiebreak_final`과 `build21_basin_local_objective_first`의 RT도 단독 scene
진단 결과이며 활성화하면 안 된다.

batch의 dual-margin 수정 전 FAIL과 수정 후 PASS 사이에서 full/5°/1° score CSV의
SHA-256은 각각 아래처럼 동일하다.

```text
orientation_full_search.csv  0754ef654192c8a609c92d16eb654c6446c8ebd380d5cac68b0c6dc922552d52
search_5deg_scores.csv        015008ecdfbe3c7e6831b9a63087dd476f6a7a9303c0870ee9eb75910df765dc
search_1deg_scores.csv        e5b69228bcc063b14258cc7239424fb61142aecbee7838d0e5f1ea850bb1755b
```

두 실행의 selected candidate #7 내부 optimized `R,t`도 동일했다. FAIL 실행의 root
`estimated_t_camera_lidar`는 fail-safe prior를 반환하므로 candidate 내부 값과 구분해야
한다. 즉 `FAIL → CANDIDATE_RT` 변화는 탐색 결과나 optimized candidate RT를 바꾼 것이
아니라, 이미 7.954% 분리된 objective 증거를 confidence 1.344% 하나만으로 다시 거절하던
최종 ambiguity 정책을 dual-margin으로 정정한 결과다.

## 6. 최종 후보 RT와 증거

dual-margin batch에서 선택된 변환은 다음과 같다.

아래 `yaw/down/roll`은 staged search의 basin/seed 진단값이며 OpenSDK에 전달할 최종
외부 파라미터 자체는 아니다. 좌표 변환에는 반드시 아래 `R,t` 행렬을 사용한다.

```text
R = [
  [-0.9842833725713, -0.0984310271301,  0.1466205148599],
  [ 0.0120562446983,  0.7908669266090,  0.6118693907689],
  [-0.1761842485679,  0.6040205603241, -0.7772504572288]
]

t = [0.0503757073740, 0.0633742645628, 0.0594082690241] m
```

| finalist | objective | confidence | mean edge | vertical error | TESL | 구조 match H/V |
|---|---:|---:|---:|---:|---:|---:|
| 선택 `167°` | **0.752967** | **0.759869** | 13.200 px | **1.514°** | 13,219 px | 26/27 |
| 경쟁 `87°` | 0.818036 | 0.746430 | 17.810 px | 4.027° | 7,766 px | 16/41 |
| 경쟁 `-106°` | 0.847919 | 0.746307 | **13.044 px** | 6.638° | 20,161 px | 3/45 |

세 번째 후보는 TESL 총량만 크지만 horizontal match가 3개이고 visible edge가 428개로
매우 희소하다. 따라서 단일 지표 TESL 최댓값이 아니라 계층 기준과 support 비교가 필요하다.

| hold-out | visible edge | mean edge | geometry NID | 구조선 matched/visible | vertical error |
|---|---:|---:|---:|---:|---:|
| build20 | 674 | 13.739 px | 0.921414 | 25/27 | 1.846° |
| build21 | 704 | 13.541 px | 0.921871 | 22/23 | 1.752° |

최종 hold-out 2D 투영은 gross 90°/180° 반전 없이 동일한 방 영역을 바라본다. 다만
LiDAR 구조점이 영상의 모든 실제 경계에 정밀하게 일치하지는 않는다. 이 시각 결과도
`CANDIDATE_RT` 판정과 일치한다.

또한 build17 영상의 편집 provenance, build20/21 장면의 사람·동적 객체, camera/scan
UTC와 SHA-256을 강제하지 않는 manifest 문제가 남아 있다. 따라서 이 5-pair batch는
알고리즘 회귀용 stress fixture로는 유효하지만 제품 승인용 capture set으로는 부족하다.

### 이전 후보와의 호환성

이전 `image_fixed` batch 후보(`169°/23°/5°`)와 새 후보의 차이는 회전 약 `14.299°`,
이동 약 `20.91 mm`다. 이는 단순 출력 포맷 변경이 아니라 algorithm-version에 따른
실질적 RT 변경이다. OpenSDK 또는 다른 팀에 전달할 때 다음을 반드시 함께 고정한다.

- 실행 binary/commit 또는 algorithm version
- 입력 pair SHA-256과 installation epoch
- intrinsic profile ID와 raw/rectified 상태
- `calibration_result.json` 전체
- `activation_allowed`와 lifecycle status

이전 RT와 새 RT를 같은 제품 승인값처럼 섞어 사용하면 안 된다.

## 7. 테스트 증거

최종 코드의 Docker 빌드와 핵심 회귀는 통과했다.

- `automatic_synthetic_lidar_tests`
- `automatic_calibration_core_tests`
- `challenger_m1_2_stress_tests`
- `challenger_m2_1_ambiguity_tests`
- `challenger_m2_2_stress_tests`
- `challenger_m3_stress_tests`

총 `6/6 PASS`이며, finalist 테스트에는 build20형 near-tie, build21형
objective/confidence disagreement, 3-scene batch형 decisive objective, 3-candidate 순환
입력 순서 불변 검사가 포함된다.

## 8. 남은 문제와 고도화 계획

### P0 — 제품 승인 근거

1. 현재 algorithm version과 후보 RT provenance를 freeze한다.
2. 최적화에 사용하지 않은 원본·동시 CH1 hold-out을 총 3쌍 이상 확보한다.
3. 동일 설치 10회에서 회전/이동 분산을 계산한다.
4. jig, CAD, survey 또는 LiDAR-visible target으로 독립 RT 기준을 만든다.
5. 기준 오차와 false activation 0건을 통과해야만 `PRODUCT_APPROVED_RT`를 허용한다.

build20/21은 동일 환경의 제한된 hold-out이다. 이것만으로 다른 설치 장소 일반화를
주장할 수 없다.

### P1 — 알고리즘 판정 고도화

- 선택 RT만 hold-out에서 검사하지 말고 separated finalist들도 같은 hold-out에 고정
  투영해 **후보별 hold-out margin**을 계산한다.
- build5/8/9/10과 20260818/19의 정상·실패 사례를 versioned positive/negative fixture로
  고정한다.
- 단일 장면 build20/21은 계속 fail-closed 상태를 유지하고 threshold를 낮추지 않는다.
- camera/scan 시각, SHA-256, 채널 profile, installation epoch를 manifest preflight에서
  자동 검증한다.

### P2 — 실행 시간

5-pair full search는 관측상 약 40분이 걸린다. 현재 5° winner에서 1° 검색이 최대 ±5°라
비용이 크지만, build20의 roll winner가 5° 단계에서 `10°`였다가 1° 단계에서 `5°`까지
이동했으므로 고정 ±3°로 줄이면 정상 후보를 놓칠 수 있다.

권장 최적화는 다음과 같다.

1. 먼저 ±3°에서 1° 검색한다.
2. winner가 경계에 닿은 축만 ±5°까지 확장한다.
3. build20/build21/full batch의 후보·판정·RT golden equality를 확인한다.
4. 결과가 같을 때만 Jenkins weekly 기본값으로 승격한다.

### P2 — 연구 항목

- `signal_strength` NMI는 현재 `weight=0`, diagnostic-only다.
- K+RT 공동 추정은 MVP 이후 연구 항목으로 보류한다.
- Qt Top-view와 OpenSDK adapter는 Core 승인과 별도 수명주기로 검증한다.

## 9. 산출물

```text
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build21_nid_soft_ab/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build21_basin_local_objective_first/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build20_tesl_tiebreak_final/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build17_21_final_logic_3train_2holdout/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build17_21_dual_margin_final/
automatic_calibration/generated/jenkins_scene0_ch1_20260823_build17_21_deterministic_final/
```

## 10. 수정 이력

| 날짜 | 변경 |
|---|---|
| 2026-08-23 | 문서·코드·실데이터 결과 재감사 및 과거 과장 주장 정정 |
| 2026-08-23 | basin-local NID gate, finalist 계층 선택, dual-margin ambiguity 판정 반영 |
| 2026-08-23 | 3-candidate 결정론적 단계 선택과 입력 순서 회귀 추가 |
| 2026-08-23 | build20/build21 단독 A/B 및 build17~21 3-training/2-hold-out 검증 기록 |
| 2026-08-24 | finalist별 hold-out에서 분리 후보 2개 동률 확인; 기존 `CANDIDATE_RT` 판정 supersede |

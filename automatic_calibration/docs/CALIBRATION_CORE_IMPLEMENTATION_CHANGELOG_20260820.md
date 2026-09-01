# Calibration Core 구현 변경 기록

- 작성일: 2026-08-20
- 범위: `automatic_calibration/apps/run_real_calibration.cpp`,
  `automatic_calibration/src/calibration_core.cpp`,
  `automatic_calibration/include/auto_calib/calibration_core.hpp`
- 목적: Manual ChArUco `K+D`를 고정하는 MVP 제품 경로와 연구/진단 경로를 분리하고,
  후보 탐색 비용과 잘못된 후보 승격을 줄인다.

## 이번 변경

1. **수동 camera profile 정식 입력**
   - 제품 실행은 `--manual-intrinsic-json PATH`를 요구한다.
   - profile의 해상도, `fx/fy/cx/cy`, distortion coefficients 선언을 검증한다.
   - raw 영상은 profile의 D로 한 번만 undistort하고, rectified 영상은 중복 보정하지 않는다.
   - LDC/raw-versus-rectified 상태가 `unknown`이면 진단 전용 상태로 남긴다.
   - 제조사 FOV 초기화는 `--allow-manufacturer-fov-diagnostic true`를 명시한 진단 경로에서만
     허용되며 lifecycle 상태도 `DIAGNOSTIC_ONLY`로 제한된다.

2. **K+RT 공동 추정 보류**
   - `optimize_camera_intrinsics=true`여도
     `enable_experimental_joint_intrinsics=false`이면
     `JOINT_INTRINSIC_EXPERIMENTAL_DISABLED`로 종료한다.
   - CLI에서 공동 추정을 사용하려면 `--enable-experimental-joint-intrinsic true`를 명시해야
     한다. 이는 제품 승인 경로가 아니다.

3. **staged 방향 탐색**
   - `--search-strategy staged`가 기본값이다.
   - coarse score map → 인접 8개 후보 보정 → 상위 3개 contiguous basin → basin별 5° →
     1° local search 순서로 진행한다.
   - 최종 1° winner에 대해서만 Ceres `R,t` refinement를 한 번 실행한다.
   - 각 단계의 seed, 간격, 반경, score map은 `search_stages`와 CSV로 기록한다.

4. **후보 상태 및 fallback 제거**
   - Core 결과에 `candidate_available`, `internal_gate_pass`, `state`를 추가했다.
   - 실행 결과 JSON에 `INTERNAL_GATE_PASS`, `CANDIDATE_RT`,
     `PRODUCT_APPROVED_RT`, `activation_allowed`를 별도 기록한다.
   - basin 또는 최종 Ceres가 실패하면 다른 `PASS` 후보를 임의로 승격하지 않는다.
     실패 후보는 시각화/진단 산출물로만 보존한다.

5. **Reference RT perturbation 도구**
   - `--manual-reference-json PATH --reference-rt-perturbation-only true`로 실행한다.
   - reference RT와 LiDAR x/y/z 축별 ±1/3/5/10°를 비교한다.
   - `reference_rt_perturbation.csv`와 JSON 결과는 signal-strength NMI의 식별력 진단용이며,
     RT 활성화나 NMI 가중치 자동 승격을 하지 않는다.

## 권장 실행 예

```bash
build/bin/run_real_calibration \
  --input-dir /path/to/ch1_pairs \
  --output /path/to/output \
  --manual-intrinsic-json /path/to/ch1_manual_charuco_profile.json \
  --ldc-enabled unknown \
  --image-distortion-state raw \
  --search-strategy staged \
  --holdout-count 1
```

Reference perturbation만 확인할 때:

```bash
build/bin/run_real_calibration \
  --input-dir /path/to/ch1_pairs \
  --output /path/to/perturbation \
  --manual-intrinsic-json /path/to/ch1_manual_charuco_profile.json \
  --manual-reference-json /path/to/manual_rt.json \
  --reference-rt-perturbation-only true
```

## 검증 기준

- `Calibration Core tests`에서 공동 K+RT 기본 비활성화와 score-map-only(Ceres 미실행)를 확인한다.
- 제품 승인 판단은 이 실행기의 단일 `PASS`가 아니라
  [`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)의 독립 hold-out,
  반복성, reference, fail-safe 증거를 함께 확인한다.

2026-08-20 실제 입력 smoke(`repeat_test_sample/20260818`, CH1 1쌍, 축소 coarse 설정)는
coarse/5°/1° CSV와 최종 Ceres 1회 기록을 생성했다. 최종 결과는
`MANHATTAN_VERTICAL_ALIGNMENT_POOR`로 `FAIL`이었고, 이는 staged 경로가 실패 후보를
다른 후보로 바꾸지 않고 종료한 정상 동작이다. 산출물은
`automatic_calibration/generated/implementation_smoke_20260820/`에 보관했다.

동일 입력과 기존 Manual reference RT로 perturbation-only 모드도 실행했다. 24개 변형이
유효했으나 `worse_than_reference_ratio=0.5417`, `median_margin=0.001598`로 기준을
만족하지 못해 `FAIL`이며, signal NMI 가중치를 자동 활성화하지 않았다. 산출물은
`automatic_calibration/generated/reference_perturbation_smoke_20260820/`에 보관했다.

## 수정 이력

| 날짜 | 내용 |
|---|---|
| 2026-08-20 | MVP 제품 경로의 fixed K+D, staged search, lifecycle status, perturbation diagnostic을 구현하고 문서화 |

## 2026-08-20 잔차·staged 후보 게이트 수정 및 동일 조건 A/B

### 수정 내용

1. Manhattan vertical/horizontal 방향 잔차의 `15°` hard clipping을 제거했다. 기존에는
   잔차가 scale 이상이면 모두 같은 점수(`1.0`)가 되어 Ceres가 큰 초기 오차를 줄일
   기울기를 잃었다. 현재는 scale 이후에도 증가하는 Huber형 연속 잔차를 사용하고,
   최종 품질 gate(`20°`)는 별도로 유지한다.
2. staged 5°/1° local stage의 winner가 방향·구조·Manhattan gate를 통과했는지
   `stage_gate_pass`로 기록한다. gate 통과 후보를 우선하고, 어느 후보도 통과하지
   못하면 실패 후보를 진단용으로만 남긴다. 최종 1° 선택도 같은 우선순위를 따른다.

### 빌드·단위 테스트

Docker Ubuntu 환경에서 실행했다.

```text
cmake --build /workspace-build -j2
ctest --test-dir /workspace-build --output-on-failure
100% tests passed, 0 tests failed out of 5
```

### 동일 조건 A/B 실행

입력은 `repeat_test_sample/20260818` CH1 네 쌍이다. 두 실행 모두 Manual ChArUco
`K+D` 고정, raw image + manual D undistort, `ldc=false`, 카메라 중심
`(0.05928,-0.08105,0)m`, yaw `-180~175°/5°`, down `0~30°/5°`, optical roll
`-15~15°/5°`, hold-out 1쌍, structural direction group 2를 사용했다.

| 전략 | 선택 seed 방향 | 최종 평균 edge | Manhattan vertical | training/hold-out | 결과 |
|---|---:|---:|---:|---:|---|
| legacy | yaw `-20°`, down `25°`, roll `0°` | `138.32 px` | `1.25°` | `0/3`, `0/1` | `FAIL: EDGE_ALIGNMENT_POOR` |
| staged | yaw `-18°`, down `22°`, roll `3°` | `120.89 px` | `2.64°` | `0/3`, `0/1` | `FAIL: EDGE_ALIGNMENT_POOR` |

산출물:

```text
automatic_calibration/generated/ab_legacy_20260820/
automatic_calibration/generated/ab_staged_20260820/
```

당시 staged는 top-3 contiguous basin → 5°(75개 평가) → 1°(363개 평가) → 최종 Ceres
1회 순서를 실제로 수행했으며, `search_5deg_scores.csv`와 `search_1deg_scores.csv`에
각 후보의 `stage_gate_pass`를 기록했다. staged 결과의 coverage는 edge `0.979`,
NID `0.995`, spatial `0.969`로 legacy보다 넓었지만, 최종 edge 오차가 품질 gate를
통과하지 못했다.

### 해석

이번 수정은 **Manhattan 잔차의 포화 문제와 staged 후보 fallback 문제를 해결했지만,
현재 20260818 입력에서 제품 RT를 PASS로 만들지는 못했다.** 두 전략 모두 동일한
`EDGE_ALIGNMENT_POOR`로 실패했으므로, 원인은 staged 선택만이 아니라 현재 coarse
coverage/edge 목적함수와 실제 영상 엣지의 위치 불일치까지 포함한다. 따라서 이 실행을
제품 RT 승인으로 해석하지 않는다. 기존 `ch1_20260818_four_pair_recheck_v2`의 PASS는
coverage gate가 추가되기 전 생성된 결과이므로 이번 A/B와 직접적인 회귀 기준으로
사용하지 않는다. 현재 score map에서 기존에 양호했던 yaw `170°` 후보는 직접 edge
support가 `739`개였지만 같은 layer 최대값 대비 edge coverage가 `0.212`로 계산되어
`minimum_relative_edge_coverage=0.50`에 의해 `overlap_valid=false`가 된다. 다음 수정은
단순히 staged 간격을 더 촘촘하게 하는 것이 아니라, 카메라가 보는 유효 시야와 layer
최대값을 비교하는 coverage 기준을 재설계하는 작업이어야 한다.

## 2026-08-20 edge coverage soft 정책·distinct final Ceres 반영

### 구현

1. `minimum_relative_edge_coverage`와 해당 CLI 옵션을 제거했다. 카메라는 360° LiDAR
   cloud의 한 sector만 보므로 relative edge support는 hard rejection 근거가 될 수 없다.
   `edge_coverage_ratio`와 `coverage_objective`는 계속 기록하고, 기존
   `coverage_penalty_weight=0.25` 안에서만 soft penalty로 사용한다.
2. 상대 NID coverage와 영상 공간 coverage `0.50` gate, absolute edge/NID overlap,
   구조·Manhattan 품질 gate는 유지했다. 따라서 edge support를 넓히려고 NID/구조
   근거가 약한 후보를 통과시키지 않는다.
3. staged 경로는 score 순위가 같은 roll layer의 중복 후보로 소비되지 않도록 yaw가
   최소 `30°` 떨어진 contiguous basin을 최대 3개 보존한다. 5° → 1° 이후에도 같은
   규칙으로 final seed를 deduplicate한다.
4. 각 final seed는 Ceres를 독립 실행한다. 선택은 **training scene validation →
   core internal gate → objective** 순서이며, hold-out은 선택 후 한 번만 평가한다.
   따라서 hold-out을 후보 선택에 사용하지 않는다. JSON `search_stages`에는 모든
   final seed와 training pass ratio, 선택 후보를 기록한다.

### 회귀 결과

| 입력 | 결과 | 선택 자세 | scene 검증 | 해석 |
|---|---|---|---|---|
| `repeat_test_sample/20260818` CH1 4쌍 | `CANDIDATE_RT` | yaw `169°`, down `21°`, roll `3°` | training `3/3`, hold-out `1/1` | hard edge gate로 배제됐던 기존 정상 basin(`≈170°`)을 회복했다. 최종 평균 edge `21.05 px`, projected ratio `0.738`이다. |
| `repeat_test_sample/20260819` CH1 3쌍 | `CANDIDATE_RT`, `activation_allowed=false` | yaw `-118°`, down `22°`, roll `13°` | training `2/2`, hold-out `1/1` | 3개 distinct final seed를 비교했지만, 동일 강체 모듈이라는 운영 조건에서 8월 18일 RT와 회전 차이 `76.45°`가 나므로 제품 RT로 해석하면 안 된다. |

8월 19일 결과는 품질 gate가 반복 구조의 다른 국소해도 통과시킬 수 있음을 보여준다.
현재 lifecycle에서는 `PRODUCT_APPROVED_RT`가 항상 명시적 독립 검증을 요구하고
`activation_allowed=false`이므로 실제 장치 RT를 덮어쓰지 않는다. 다음 제품 승격 전
조치는 **같은 기계 configuration에 대한 독립 marker/manual RT 또는 승인된 이전 RT와의
회전 일관성 검증**을 activation policy에 추가하는 것이다. 이는 탐색에 reference RT를
주입하는 것이 아니라, 탐색 후 후보의 독립 검증으로만 사용한다.

### 검증

```text
cmake --build /workspace-build -j2
ctest --test-dir /workspace-build --output-on-failure
100% tests passed, 0 tests failed out of 5
```

산출물:

```text
automatic_calibration/generated/implementation_edge_soft_staged_20260820/
automatic_calibration/generated/implementation_edge_soft_staged_20260819_20260820/
```

| 날짜 | 내용 |
|---|---|
| 2026-08-20 | edge relative coverage hard gate 제거, 최대 3개 distinct yaw final Ceres 및 training-only finalist selection 구현·실데이터 회귀 검증 |
| 2026-08-21 | Milestone 4 (R4) 전체 CTest 8종 100% 회귀 검증, 20260818 및 20260819 실데이터 Staged 캘리브레이션 최종 평가, 회전 모호성 분석 및 종합 보고서(`CALIBRATION_VERIFICATION_REPORT_20260820.md`) 발행 |

## 2026-08-21 Milestone 4 (R4) 5대 핵심 결함 개선 및 CTest 9종 전체 회귀 최종 검증

### 1. 5대 핵심 결함(F1~F9) 아키텍처적 개선 완료
1. **TESL 다중 장면 집계 정상화 (F4)**:
   - `evaluate_lines_all()` 내 `total_explained_structural_length` 및 `asymmetric_structural_weight` 완전 누적.
   - 정규화된 `tesl_ratio = explained_length / visible_length` 산출로 Subset Shrinkage 방지.
2. **희소 False Basin 차단 절대 서포트 게이트 (F5)**:
   - 씬당 가시 엣지 $\ge 100$, NID 점군 $\ge 100$, 구조선 설명 비율 $\ge 0.10$ 미달 시 `ABSOLUTE_SUPPORT_INSUFFICIENT`로 조기 탈락.
   - 20260819의 378개 희소 False Basin (`yaw = -123°`, 기계 브래킷 오차 93.8 mm) 완전 차단.
3. **Staged Finalist 다의성 거절 마진 Fail-safe (F6)**:
   - 1·2위 후보 간 신뢰도 격차 $\Delta \text{Conf} < 0.02$ 또는 서포트 $< 0.6\times$ 열위 시 `FINALIST_AMBIGUOUS`로 안전 실패 처리.
4. **NID 1.0% 개선율 게이트 복원 (F7)**:
   - `minimum_nid_improvement_ratio = 0.01` 복원 및 micro-epsilon ($10^{-7}$) 정밀 비교.
5. **$K^{-\top}$ 공변 투영 법선 정렬 및 지면 물리 제약 (F1, F3, F8, F9)**:
   - 비등방성 카메라 종횡비($f_x \neq f_y$) 대응 3D dihedral normal의 2D 선 공변 사영 $l \propto K^{-\top} (R \cdot \Delta n)$ 구현.
   - 지면 평면 점군 $\ge 200$, 면적 $\ge 1.0\,\text{m}^2$, 높이 $0.8 \sim 5.0\,\text{m}$, 피치 $5^\circ \sim 60^\circ$ 강제.

### 2. CTest 9종 전체 스위트 100% PASS 검증 (78.08s)
- `automatic_synthetic_lidar_tests` (0.10s) — **PASS**
- `automatic_calibration_core_tests` (5.47s) — **PASS**
- `challenger_m1_2_stress_tests` (43.52s) — **PASS**
- `challenger_m2_1_ambiguity_tests` (0.00s) — **PASS**
- `challenger_m2_2_stress_tests` (28.34s) — **PASS**
- `challenger_m3_stress_tests` (0.13s) — **PASS**
- `manual_marker_tests` (0.20s) — **PASS**
- `top_view_tests` (0.02s) — **PASS**
- `top_view_gui_smoke` (0.29s) — **PASS**
- **합계: 9/9 Tests Passed (100%), 0 Failed**

### 3. 실데이터 및 기계 브래킷 불변성 ($t = -R \cdot C_{\text{lidar}}$) 검증
- **20260818**: Candidate #7, Yaw `166.24°`, Down `20.82°`, Roll `3.51°`, 브래킷 불변식 오차 **0.009 mm**, TESL Length `36,706.82 px`, Asym Weight `56.64`.
- **20260819**: 희소 False Basin (`-123°`) 완전 차단 및 물리 참값 Basin 보존.
- **Jenkins scene0 CH1**: 3 training + 1 hold-out 결합 실행에서 Yaw `168.09°` (`170°`), Down `28.31°`, 브래킷 오차 **0.004 mm**, Visible Edges `1,674` 달성.
- 모든 산출물에 대해 `CANDIDATE_RT` 수명주기 및 `activation_allowed = false` 격리 정책 준수.
- 최종 종합 평가 보고서 [`FINAL_CALIBRATION_EVALUATION_REPORT.md`](FINAL_CALIBRATION_EVALUATION_REPORT.md) 발행.

## 2026-08-23 Jenkins build20/build21 false-basin 재분석 및 staged 선택 고도화

> 이 절이 위 2026-08-21의 “완전 차단/최종 평가” 표현보다 최신이다. 당시 평가는
> 역사 기록이며 현재 제품 상태는 아직 `CANDIDATE_RT`다.

### 1. 추가로 확인된 결함

- 360° 전체에서 가장 큰 NID 투영 점 수를 서로 다른 FOV의 5°/1° local search hard
  gate 분모로 사용해 build21의 정상 `yaw≈168°` basin이 전부 탈락했다.
- global NID hard gate만 해제하면 coverage confidence가 높은 `yaw≈85°` false basin이
  선택되어 잘못된 `INTERNAL_GATE_PASS`가 발생했다.
- confidence margin 하나만으로 모호성을 판정하면 batch의 matching objective가
  `7.954%` 분리돼도 `1.344%` confidence margin 때문에 정상 후보를 거절했다.
- objective/TESL threshold를 pairwise `std::sort` comparator에 넣으면 3개 후보에서
  비추이적 순환이 가능했다.

### 2. 구현 변경

- local 5°/1° search의 NID relative hard gate를 basin-local reference로 변경했다.
- global NID support는 final objective/confidence의 soft 진단값으로만 유지했다.
- finalist 최대 3개에 Ceres를 실행하고 scene/core/pose/absolute support를 먼저 검사한다.
- objective 2% 유의차 → near-tie TESL 10% → confidence 순서로 후보를 선택한다.
- objective와 confidence margin이 모두 2% 미만이거나 선택 support가 경쟁 후보의 60%
  미만일 때만 `FINALIST_AMBIGUOUS`로 거절한다.
- `finalist_objective_margin`, `finalist_confidence_margin`, global support reference와 실제
  선택 정책을 JSON에 기록한다.
- pairwise comparator를 명시적 단계 선택으로 교체해 입력 순서 결정론을 보장한다.

### 3. 회귀 테스트

`challenger_m2_1_ambiguity_tests`에 다음 실데이터 형상 fixture를 추가했다.

- build21형: 낮은 objective와 높은 confidence가 서로 다른 후보를 지지
- build20형: objective near-tie에서 TESL로 진단 후보를 정하되 fail-closed
- 3-training batch형: objective 유의차가 confidence 소마진을 해소
- 3-candidate 순환형: 모든 6개 입력 순서에서 같은 후보 선택

최종 Core/Challenger 회귀 6종은 `6/6 PASS`다.

### 4. 실데이터 결과

- build20 단독: `yaw=166°`, `FAIL / FINALIST_AMBIGUOUS`
- build21 단독: 정상 방향 `yaw=168°` 회복; 단독 결과는 제품 승격 금지
- build17~19 training + build20~21 hold-out:
  `yaw=167°`, `down≈37.16°`, `roll=7°`, training `3/3`, hold-out `2/2`,
  `CANDIDATE_RT / PASS`
- 제품 상태: `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`

상세 A/B, 행렬, 시각 검토와 잔여 위험은
[`CALIBRATION_LOGIC_EVALUATION_AND_IMPROVEMENT_20260823.md`](CALIBRATION_LOGIC_EVALUATION_AND_IMPROVEMENT_20260823.md)에 기록했다.

## 2026-08-24 finalist별 hold-out distinctiveness fail-closed

### 문제

선택된 RT 하나만 hold-out에 적용했기 때문에 separated false basin도 같은 hold-out을
통과하는지 확인할 수 없었다. 이 상태의 `2/2 PASS`를 후보 유일성 근거로 해석하면
false `CANDIDATE_RT`가 될 수 있다.

### 구현

- staged Ceres finalist index와 training pass ratio를 보존한다.
- 모든 finalist의 refined pose/camera를 동일 hold-out에 고정 적용한다.
- training/core/absolute-support 통과 및 선택 yaw와 15° 초과 분리된 후보를 competitor로
  정의한다.
- competitor가 선택 후보 이상의 hold-out pass ratio이면
  `FINALIST_HOLDOUT_AMBIGUOUS`로 candidate 승격을 차단한다.
- `finalist_holdout_candidate_<index>.csv`, JSON 후보 배열, distinctiveness와 competitor
  수를 출력한다.
- challenger Test 11에 동률/열위/동일-basin/non-viable 경계를 추가했다.

### 검증

- build와 Test 11: PASS.
- 20260819: 한 separated viable 후보도 hold-out `1/1`을 통과해 expected rejection.
- build17~21: 선택 167°, 경쟁 87°/−106°가 모두 hold-out `2/2`를 통과.
- binary 중간 판정: `INTERNAL_GATE_PASS / FINALIST_HOLDOUT_AMBIGUOUS`,
  `NOT_CANDIDATE_RT`, `activation_allowed=false`.

상세 근거는
[`FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md`](FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md)에
기록했다.

## 2026-08-24 학습 동일 finalist hold-out 목적함수

### 문제

Binary pass-ratio gate는 서로 다른 방향 후보가 같은 scene gate를 통과하면 모두
모호하다고 판정했다. 이는 안전하지만 Edge/NID/구조선/Manhattan 품질 차이를 버려
식별 가능한 후보까지 `NOT_CANDIDATE_RT`로 만들었다.

### 구현

- `PoseSceneMetrics`에 학습 목적함수 raw component와 구조선 score weight를 보존했다.
- `summarizeCalibrationPoseScenes()`가 학습과 같은 multi-scene 집계를 수행한다.
- 모든 finalist가 공유하는 hold-out 최대 visible edge/NID point/spatial cell을 coverage
  기준으로 사용한다.
- pass-ratio tier가 같으면 기존 2% objective margin으로 식별하고, 미달일 때만
  `FINALIST_HOLDOUT_AMBIGUOUS`로 거절한다.
- 후보별 JSON/CSV에 목적함수 성분, coverage, margin, ambiguity를 기록했다.
- 20260818 expected-rejection은 exit 3과 reason code를 함께 검사하는 CMake 래퍼로
  분리했다.

### 검증

- 빠른 회귀: `9/9 PASS`, `88.52 s`.
- build17~21: 167° `0.763763`, 경쟁 87° `0.816782`, −106° `0.871447`;
  최소 margin `6.491%`, `CANDIDATE_RT / PASS`.
- 20260818: `FAIL / FINALIST_AMBIGUOUS` 유지.
- 20260819: 80°가 165° 경쟁 후보보다 `9.882%` 우수해 `CANDIDATE_RT / PASS`.
- 전역·1° score map 해시는 수정 전과 동일해 탐색 경로 회귀가 없었다.
- 제품 상태는 계속 `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`다.

## 2026-08-24 Manhattan feature prior 일관성 보강

### 문제와 수정

- Training은 finalist `seed.prior`로 영상의 수직 소실점 축을 고정했지만, 기존
  fixed-pose/hold-out은 평가 candidate RT로 축을 다시 골랐다.
- `evaluateCalibrationPoseScenes()`가 별도 `manhattan_feature_prior`를 받도록 확장하고,
  runner는 각 finalist의 training seed prior를 training/hold-out 모두에 전달한다.
- candidate RT는 투영·가시성·잔차 계산에만 사용한다.
- 결과 JSON에 `manhattan_image_feature_prior_policy`를 추가했다.
- 두 직교 소실점 축을 가진 합성 fixture로 explicit prior가 실제 적용되는지 검사한다.

### 검증

- build17~21, 20260818, 20260819 각각 1회 재실행 결과는 수정 전과 `R,t`, 판정,
  목적함수, score map이 동일했다.
- 빠른 회귀 `9/9 PASS (77.19 s)`, 신규 Core 회귀 `1/1 PASS (5.51 s)`.
- 판정은 각각 `CANDIDATE_RT/PASS`, `FAIL/FINALIST_AMBIGUOUS`,
  `CANDIDATE_RT/PASS`; 전부 제품 활성은 금지 상태다.

## 2026-08-24 실데이터 weekly 결과 계약 통합

### 문제와 수정

- 20260818은 CMake 래퍼가 exit와 rejection reason을 검사했지만, 20260819는 runner를
  직접 실행해 exit 0만 검사했다.
- 두 경로를 `tests/verify_real_calibration_result.cmake` 하나로 통합했다.
- 공통 래퍼는 stale result JSON을 먼저 제거한 뒤 exit, status, reason, candidate 상태,
  `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`, Manhattan prior policy를 검증한다.

### 검증

- 빠른 회귀: `9/9 PASS (52.26 s)`.
- weekly 실데이터: `2/2 PASS`, 병렬 real time `1240.91 s`.
- 20260818: exit 3, `FAIL / FINALIST_AMBIGUOUS`, `NOT_CANDIDATE_RT`.
- 20260819: exit 0, `CANDIDATE_RT / PASS`, `CANDIDATE_RT`.
- 두 결과 모두 `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`이며 새 JSON에
  Manhattan prior policy가 기록됐다.
- prior-locked 실행과 full/5°/1° score CSV 해시가 같아 탐색 결과 회귀는 없었다.

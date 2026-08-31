# V3-R6-R1 Directional Edge Orientation Contract Correction (2026-08-31)

> Correction status: **INVALIDATED_BY_ORIENTATION_CONVENTION_AND_SUPPORT_MISMATCH**
> R6 evidence is preserved. This report contains the corrected tangent contract and exact-support audit.

## 결론

- 판정: **DIRECTIONAL_EDGE_E2E_NOT_ACCEPTED**
- corrected variable-support Reference 우세: **3/4**
- corrected exact-common-support Reference 우세: **3/4**
- Stage B E2E 실행: **True**
- 제품 기본 경로, Ground Truth, 제품 RT는 선언하지 않는다.

## 정정 내용

- 카메라 Sobel 방향은 edge normal이다.
- R6-R1은 `camera_edge_tangent = fmod(atan2(gy,gx)+pi/2, pi)`로 변환한 뒤 기존 4개 unsigned bin을 그대로 사용한다.
- LiDAR projected tangent는 기존 `atan2(dv,du)`를 그대로 사용한다.
- 각 pose의 C++ directional scorer가 반환한 최종 visible `point_index`의 교집합을 강제 mask로 사용했다.
- Python에서 score/NID를 재구현하지 않았고, exact score도 C++ production scorer가 계산했다.

## Fixed-pose 결과

| build | pose | variable objective | variable samples | exact objective | exact samples |
| --- | --- | ---: | ---: | ---: | ---: |
| build45 | manual_reference | 0.4705923444206655 | 134 | 0.47914558332801105 | 129 |
| build45 | r5_d_selected | 0.49934639695151095 | 134 | 0.5091068954926194 | 129 |
| build46 | manual_reference | 0.3732852144496231 | 143 | 0.35399615593100103 | 100 |
| build46 | r5_d_selected | 0.3980764531420837 | 110 | 0.3912537192124266 | 100 |
| build48 | manual_reference | 0.41824185545810283 | 135 | 0.43072061677545836 | 104 |
| build48 | r5_d_selected | 0.37187612535288583 | 144 | 0.3665616145777864 | 104 |
| build49 | manual_reference | 0.41553857187899135 | 181 | 0.3789409718192863 | 67 |
| build49 | r5_d_selected | 0.5096805135142174 | 87 | 0.5573939952458989 | 67 |

## Exact support 판정

- `common_point_ids=0`인 build는 `NON_COMPARABLE`이며 Stage A 근거에 포함하지 않았다.
- Reference와 Selected의 exact sample count가 common ID 수와 다르면 `EXACT_SAMPLE_SUPPORT_NOT_ACHIEVED`이다.
- exact margin은 `selected_objective - reference_objective`이며 양수일 때 Reference가 우세하다.

## Stage B

| build | status | R5-D error | R6-R1 error | improvement |
| --- | --- | ---: | ---: | ---: |
| build45 | FAIL | 7.032299 | 168.386104 | -161.353805 |
| build46 | INTERNAL_GATE_PASS | 10.015828 | 9.620771 | 0.395057 |
| build48 | INTERNAL_GATE_PASS | 21.887468 | 21.887468 | 0.000000 |
| build49 | INTERNAL_GATE_PASS | 19.151480 | 19.151480 | 0.000000 |
| build50 | INTERNAL_GATE_PASS | 168.263640 | 28.771448 | 139.492192 |

- 유효 build에서 2° 이상 개선: `없음`.
- 유효 build에서 2° 초과 악화: `build45`.
- 유효 build의 90° 이상 operational branch: `build45`.
- build50 요구 상태 `FAIL`, 실제 상태 `INTERNAL_GATE_PASS`.
- 최대 runtime overhead: `0.518221` (요구 ≤ 0.10).
- 따라서 Stage B gate는 통과하지 못했으며, directional edge를 제품 기본 경로로 승격하지 않는다.

## 검증

- orientation synthetic/Core/Analyzer tests: `True`
- exact sample-count equality: `True`
- finite outputs: `True`
- 기존 R4/R5/R6 evidence는 덮어쓰지 않았다.
- 다음 score weight/threshold 변경은 Sol 리뷰 이후에만 가능하다.

## 산출물

- output root: `/workspace/automatic_calibration/generated/v3_r6_r1_directional_edge_correction_20260831_r1`
- directional_edge_variable_support.csv
- directional_edge_exact_common_support.csv
- directional_edge_sample_ids.csv
- directional_edge_e2e_comparison.csv
- directional_edge_runtime.csv
- validation_checks.json
- orientation contract synthetic test log and four orientation-bin images

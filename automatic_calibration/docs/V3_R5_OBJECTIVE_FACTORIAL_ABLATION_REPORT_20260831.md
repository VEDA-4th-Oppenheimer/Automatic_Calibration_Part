# V3-R5 Structural/Manhattan Objective Factorial Ablation (2026-08-31)

## 상태

- 판정: **CURRENT_CROSS_MODAL_FEATURES_INSUFFICIENT**
- 목적은 NID-off 이후 잔여 오차의 structural line/Manhattan 기여를 기존 production CLI로 분리하는 것이다.
- 제품 기본값, 제품 RT, Ground Truth를 선언하지 않는다.

## 조건 및 실행 수

- B는 R4 NID-off 결과를 그대로 재사용했다: `nid=0.0, line=0.20, manhattan=0.15`.
- C/D/E만 새로 실행했다. 각 build 1회씩 총 15회이며 runtime loop/repeat는 없다.
- 모든 조건의 edge weight=0.25, coverage penalty weight=0.25, analyzer/search/K/D/camera-center/gate/fallback은 R4와 동일하다.
- camera center=`[0,-0.08105,0] m`, baseline은 지정하지 않았다.
- build50은 fail-closed 유지 여부를 별도로 검증했다.

| condition | nid | line | manhattan |
| --- | --- | --- | --- |
| B | 0.000000 | 0.200000 | 0.150000 |
| C | 0.000000 | 0.200000 | 0.000000 |
| D | 0.000000 | 0.000000 | 0.150000 |
| E | 0.000000 | 0.000000 | 0.000000 |

## 결과

| build | condition | status | reason_code | operational_rotation_error_deg | selected_candidate_index | basin_proposal_candidate | fallback_triggered | composite_objective | geometry_nid_objective | manhattan_objective | coverage_objective | optimization_runtime_ms | total_runtime_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| build45 | B | INTERNAL_GATE_PASS | PASS | 15.749186 | 0 | 0 | False | 0.263955 | 0.825200 | 0.184146 | 0.000156 | 7475.352785 | 66310.921060 |
| build45 | C | INTERNAL_GATE_PASS | PASS | 37.056175 | 0 | 0 | False | 0.243916 | 0.781167 | 0.344545 | 0.001134 | 6673.516510 | 61079.155471 |
| build45 | D | INTERNAL_GATE_PASS | PASS | 7.032299 | 0 | 0 | False | 0.092800 | 0.837926 | 0.068277 | 0.000380 | 3843.013576 | 52248.729421 |
| build45 | E | INTERNAL_GATE_PASS | PASS | 10.515106 | 0 | 0 | False | 0.071185 | 0.850630 | 0.253557 | 0.000175 | 2901.704597 | 67160.215275 |
| build46 | B | INTERNAL_GATE_PASS | PASS | 11.615041 | 0 | 0 | False | 0.274357 | 0.838503 | 0.059574 | 0.000005 | 3038.678700 | 52635.605721 |
| build46 | C | INTERNAL_GATE_PASS | PASS | 48.036179 | 0 | 0 | False | 0.262418 | 0.870047 | 1.005018 | 0.000253 | 3272.112204 | 57503.559516 |
| build46 | D | INTERNAL_GATE_PASS | PASS | 10.015828 | 0 | 0 | False | 0.088067 | 0.849957 | 0.028215 | 0.001958 | 2964.721891 | 55912.175665 |
| build46 | E | INTERNAL_GATE_PASS | PASS | 49.267424 | 0 | 0 | False | 0.076544 | 0.866539 | 0.974433 | 0.000131 | 2432.853282 | 53162.480441 |
| build48 | B | INTERNAL_GATE_PASS | PASS | 26.660355 | 0 | 0 | False | 0.275856 | 0.800972 | 0.125639 | 0.000032 | 3150.191782 | 52066.403005 |
| build48 | C | INTERNAL_GATE_PASS | PASS | 31.791090 | 0 | 0 | False | 0.241112 | 0.826710 | 0.310214 | 0.002525 | 2987.239457 | 50529.585849 |
| build48 | D | INTERNAL_GATE_PASS | PASS | 21.887468 | 0 | 0 | False | 0.106297 | 0.849982 | 0.048580 | 0.000105 | 3592.108150 | 50687.035776 |
| build48 | E | INTERNAL_GATE_PASS | PASS | 65.187720 | 0 | 0 | False | 0.071657 | 0.855224 | 0.473234 | 0.000367 | 2397.612570 | 50882.345798 |
| build49 | B | INTERNAL_GATE_PASS | PASS | 20.558838 | 0 | 0 | False | 0.249394 | 0.820581 | 0.146470 | 0.001816 | 5051.470663 | 54077.871243 |
| build49 | C | INTERNAL_GATE_PASS | PASS | 32.924758 | 1 | 1 | False | 0.257997 | 0.812818 | 0.583305 | 0.000129 | 3328.378078 | 47765.414792 |
| build49 | D | INTERNAL_GATE_PASS | PASS | 19.151480 | 0 | 0 | False | 0.093967 | 0.800637 | 0.071230 | 0.000326 | 3230.682426 | 48340.936663 |
| build49 | E | INTERNAL_GATE_PASS | PASS | 34.441977 | 0 | 0 | False | 0.081234 | 0.802713 | 0.604033 | 0.000207 | 2515.684541 | 43476.955337 |
| build50 | B | FAIL | OBJECTIVE_IMPROVEMENT_INSUFFICIENT | 168.263640 | 0 | 0 | False | 0.448334 | 0.861214 | 0.945619 | 0.000257 | 2703.416277 | 47964.138211 |
| build50 | C | FAIL | NID_IMPROVEMENT_INSUFFICIENT | 168.185754 | 0 | 0 | False | 0.296851 | 0.893593 | 1.071184 | 0.000181 | 2492.582305 | 39660.756060 |
| build50 | D | FAIL | OBJECTIVE_IMPROVEMENT_INSUFFICIENT | 168.263640 | 0 | 0 | False | 0.232657 | 0.871380 | 0.526409 | 0.029790 | 2816.648104 | 40397.905140 |
| build50 | E | INTERNAL_GATE_PASS | PASS | 48.876008 | 0 | 0 | False | 0.117539 | 0.872870 | 1.224689 | 0.000024 | 2033.816213 | 39971.160877 |

### Reference 회전 오차 변화

| build | B_error_deg | C_error_deg | C_improvement_deg | D_error_deg | D_improvement_deg | E_error_deg | E_improvement_deg |
| --- | --- | --- | --- | --- | --- | --- | --- |
| build45 | 15.749186 | 37.056175 | -21.306989 | 7.032299 | 8.716887 | 10.515106 | 5.234080 |
| build46 | 11.615041 | 48.036179 | -36.421138 | 10.015828 | 1.599212 | 49.267424 | -37.652384 |
| build48 | 26.660355 | 31.791090 | -5.130735 | 21.887468 | 4.772887 | 65.187720 | -38.527365 |
| build49 | 20.558838 | 32.924758 | -12.365921 | 19.151480 | 1.407358 | 34.441977 | -13.883140 |
| build50 | 168.263640 | 168.185754 | 0.077886 | 168.263640 | 0.000000 | 48.876008 | 119.387632 |

- C qualifying builds (≥2° improvement): `0` / 4.
- D qualifying builds (≥2° improvement): `2` / 4.
- E qualifying builds (≥2° improvement and best): `0` / 4.
- wrong branch condition: `['build50/E']`.
- wrong product PASS: `[]`.

## build50 fail-closed

| condition | status | reason_code | condition_disqualified | operational_rotation_error_deg |
| --- | --- | --- | --- | --- |
| B | FAIL | OBJECTIVE_IMPROVEMENT_INSUFFICIENT | False | 168.263640 |
| C | FAIL | NID_IMPROVEMENT_INSUFFICIENT | False | 168.185754 |
| D | FAIL | OBJECTIVE_IMPROVEMENT_INSUFFICIENT | False | 168.263640 |
| E | INTERNAL_GATE_PASS | PASS | True | 48.876008 |

- 요구 상태 유지 여부: `False`.
- fail-closed 위반 조건: `['build50/E']`.
build50/E가 PASS 상태가 되어 fail-closed 요구를 위반했으므로 해당 조건은 안전한 ablation 개선으로 인정하지 않고 탈락 처리했다. product-approved RT는 생성되지 않았다.

## Legacy 및 설정 불변성

- R4 B source: `/workspace/automatic_calibration/generated/v3_r4_geometry_nid_ablation_20260831`
- 신규 runner 호출 수: `15` (expected 15)
- factor 외 command parity: `True`
- factor 외 result configuration parity: `True`
- R4 B evidence hash unchanged: `True`
- targeted Core/Analyzer tests: `True`

## 점수 필드 해석 제한

현재 unchanged runner는 scalar `edge_objective`와 scalar `structural_objective`를 top-level JSON에 직접 직렬화하지 않는다.
따라서 이를 Python에서 재구현하거나 추정하지 않고 null로 기록했으며, edge mean distance, structural horizontal/vertical diagnostic, Manhattan/NID/coverage/composite는 실제 runner JSON 값으로 보존했다.
기존 R4 errata와 같이 `final_nid`는 NID score이고 `geometry_nid_objective`는 production squared term이다.

## 산출물 및 다음 단계

- output root: `/workspace/automatic_calibration/generated/v3_r5_objective_ablation_20260831`
- `objective_ablation_comparison.csv`, `candidate_selection_comparison.csv`, `runtime_comparison.csv`, `validation_checks.json`
- 각 B/C/D/E run 디렉터리의 `matching_scene_0.png` 및 `scene_0_colorized_lidar_3d_preview.png`
- 결과가 어떤 조건에서 개선되더라도 weight 변경은 Sol 리뷰 이후 별도 작업으로만 진행한다.

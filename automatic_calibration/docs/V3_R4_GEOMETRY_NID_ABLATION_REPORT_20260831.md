# V3-R4 Geometry NID Production Ablation (2026-08-31)

## 상태

- 판정: **NID_HARMFUL_IN_CURRENT_SCENE_SET**
- 이 실험은 기존 production scorer의 `--nid-weight`만 바꾼 offline/engineering 비교다.
- Manual Reference는 실행 후 회전 오차 계산에만 사용했다. Ground Truth·제품 RT로 사용하지 않았다.
- build50의 FAIL/Fail-closed는 정상적인 안전 동작으로 별도 기록한다.

## 실행 계약

- branch: `exp-v3-r4-nid-ablation`
- base commit: `330eb457`
- input root: `/workspace/develop/data/jenkins-capture/scene0`
- K/D: `/workspace/develop/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json`; image distortion state=`raw`; intrinsic refinement=`false`
- camera center: `[0, -0.08105, 0] m`; `baseline_m`는 지정하지 않았다.
- common search: `staged + hybrid`, yaw 15°, down 15°, optical roll 5°.
- A Legacy: `--nid-weight 0.55`; B NID ablation: `--nid-weight 0.0`.
- build별 A/B 각각 1회, 총 10회 실행. 추가 runtime loop/repeat 없음.

## 입력 역할

- build45, 46, 48, 49, 50은 각각 단일 scene training input이다.
- 이 실험은 hold-out split을 수행하지 않으며 `holdout_count=0`이다.
- Reference는 자동 입력/초기값/analyzer/fallback에 주입하지 않았다.

## 핵심 결과

| build | condition | status | reason_code | selected_candidate_index | basin_proposal_candidate | operational_rotation_error_deg | camera_center_error_mm | projection_evaluation_count | analyzer_runtime_ms | optimization_runtime_ms | total_runtime_ms | composite_objective | geometry_nid_objective | manhattan_objective | coverage_objective |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| build45 | legacy | INTERNAL_GATE_PASS | PASS | 0 | 0 | 31.364358071223645 | 1.794444670422438e-14 | 48.0 | 8917.57709 | 3459.002043 | 53761.259148 | 0.7315257500894112 | 0.7547062139355009 | 0.35666860742589973 | 4.286549239458036e-05 |
| build45 | nid_off | INTERNAL_GATE_PASS | PASS | 0 | 0 | 15.74918587679016 | 2.2380897915594677e-14 | 48.0 | 9289.178209 | 7475.352785 | 66310.92106 | 0.26395548536832053 | 0.8252002078471813 | 0.18414610395667003 | 0.0001558868091329803 |
| build46 | legacy | INTERNAL_GATE_PASS | PASS | 0 | 0 | 12.452442784018816 | 2.248458606249289e-14 | 48.0 | 12745.19653 | 4648.592502 | 64934.090881 | 0.7249881477629934 | 0.7992351898085286 | 0.07220571868603026 | 7.77925583638676e-06 |
| build46 | nid_off | INTERNAL_GATE_PASS | PASS | 0 | 0 | 11.615040741641222 | 4.422694427346622e-14 | 48.0 | 9094.638947 | 3038.6787 | 52635.605721 | 0.27435693593969 | 0.8385034971824433 | 0.05957423011947975 | 4.969116938228888e-06 |
| build48 | legacy | INTERNAL_GATE_PASS | PASS | 0 | 0 | 26.189520855029862 | 6.13071990641655e-14 | 48.0 | 8814.497232 | 3786.573599 | 54220.119818 | 0.7066966986720901 | 0.7841635696964137 | 0.11871889522071134 | 5.782973922041365e-05 |
| build48 | nid_off | INTERNAL_GATE_PASS | PASS | 0 | 0 | 26.660354780925065 | 3.162006443755934e-14 | 48.0 | 8645.912329 | 3150.191782 | 52066.403005 | 0.27585613154605493 | 0.8009722758158039 | 0.12563852627553576 | 3.1842309711316425e-05 |
| build49 | legacy | INTERNAL_GATE_PASS | PASS | 0 | 0 | 20.776462031495747 | 1.4577177840657118e-14 | 38.0 | 9207.889475 | 4487.641275 | 48062.575368 | 0.6988428302369788 | 0.7834684918706014 | 0.18760955243855423 | 0.0004904833703743394 |
| build49 | nid_off | INTERNAL_GATE_PASS | PASS | 0 | 0 | 20.5588376381062 | 3.5619960450617347e-14 | 38.0 | 10835.246523 | 5051.470663 | 54077.871243 | 0.24939416609193304 | 0.8205807481127363 | 0.14646984239642039 | 0.0018156967022161942 |
| build50 | legacy | FAIL | OBJECTIVE_IMPROVEMENT_INSUFFICIENT | 0 | 0 | 168.26364003304207 | 0.0 | 38.0 | 9126.417346 | 4265.999692 | 45412.986975 | 0.9315290997528983 | 0.8599592090277364 | 0.8334025639060371 | 0.00025244779698212823 |
| build50 | nid_off | FAIL | OBJECTIVE_IMPROVEMENT_INSUFFICIENT | 0 | 0 | 168.26364003304207 | 0.0 | 38.0 | 11216.149227 | 2703.416277 | 47964.138211 | 0.4483336063135958 | 0.8612138375267954 | 0.9456191445391845 | 0.00025651555704025276 |

`edge_objective`와 scalar `structural_objective`는 현재 unchanged runner의 top-level JSON에 직접 직렬화되지 않는다.
따라서 표에는 null로 두고, `edge_mean_distance_px`, `structural_horizontal_objective`, `structural_vertical_objective`와 실제 production `final_composite_objective`를 함께 보존했다.
`geometry_nid_objective`는 production 코드의 `final_nid^2`로 기록했다.

## Legacy 무동작 비교

| build | legacy_match_selected_candidate | legacy_match_status | legacy_match_reason | legacy_match_operational_rt |
| --- | --- | --- | --- | --- |
| build45 | True | True | True | True |
| build46 | True | True | True | True |
| build48 | True | True | True | True |
| build49 | True | True | True | True |
| build50 | True | True | True | True |

A 조건의 selected candidate/status/reason/operational RT가 기존 V3 evidence와 일치해야 한다. `validation_checks.json`의 세부 결과를 기준으로 확인한다.

## 판정 근거

| build | legacy_error_deg | nid_off_error_deg | nid_off_improved | nid_off_wrong_branch |
| --- | --- | --- | --- | --- |
| build45 | 31.364358071223645 | 15.74918587679016 | True | False |
| build46 | 12.452442784018816 | 11.615040741641222 | True | False |
| build48 | 26.189520855029862 | 26.660354780925065 | False | False |
| build49 | 20.776462031495747 | 20.5588376381062 | True | False |
| build50 | 168.26364003304207 | 168.26364003304207 | False | False |

- eligible builds with lower NID-OFF error: `3` / 4.
- new NID-OFF 90°/180° branch: `False`.
- wrong product PASS caused by NID-OFF: `False`.
- build50은 비교 대상에는 포함하지만, NID-OFF가 fail-closed이면 그것 자체를 실패로 간주하지 않는다.

## 산출물

- `nid_ablation_comparison.csv`: production result/objective/RT summary
- `runtime_comparison.csv`: analyzer, selected-candidate optimization, pipeline/wall runtime
- `candidate_selection_comparison.csv`: selected basin/candidate/fallback and legacy no-op checks
- 각 run 디렉터리의 `matching_scene_0.png` 및 `scene_0_colorized_lidar_3d_preview.png`
- 실행 루트: `/workspace/automatic_calibration/generated/v3_r4_geometry_nid_ablation_20260831`

## 제한 및 다음 조치

이번 단계에서는 NID score/weight/threshold/fallback/optimizer를 수정하지 않았다.
NID OFF가 순위를 바꾸더라도 이는 production 변경 승인이 아니며, Sol 리뷰 후 별도 변경으로 다룬다.
특히 scalar edge/structural objective를 직접 필요로 하는 후속 감사는 runner diagnostic serialization 또는 별도 고정-pose 평가가 필요하다.
제품 PASS, Ground Truth, reference 승격은 선언하지 않는다.

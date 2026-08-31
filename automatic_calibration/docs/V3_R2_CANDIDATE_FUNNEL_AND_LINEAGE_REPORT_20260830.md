# V3-R2 Candidate Funnel and Finalist Lineage Audit

Date: 2026-08-30
Branch: exp-v3-r2-candidate-lineage
Base: 19a42d31f17c372019852ffad8e0617c741acb6f
Manual Reference: /mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/manual_calibration/output/manual_projection_refiner_build51_20260830/manual_reference_candidate/manual_reference_rt.json

## 정정 상태 — V3-R2-R1 (2026-08-30)

이 문서는 V3-R2 당시의 감사 기록을 보존한다. 다만 기존 evaluator가
`candidate_results[i].estimated`를 Ceres finalist pose로 해석한 부분은
정정되었다. `estimated`는 gate 처리 후의 operational/caller-facing 값이며,
Ceres가 실제 최적화한 pose는
`candidate_results[i].diagnostic_candidate`이다. 특히 build50의 기존
168.263640° Ceres 값은 잘못된 역할 매핑으로 생긴 수치이고, 실제 Ceres
diagnostic candidate 오차는 41.723008°이다. 168.263640°는 FAIL 상태에서
반환된 operational estimated 값이다. build50은 여전히 `FAIL`이며 accepted
product candidate가 아니다.

역할별 재계산 결과와 JSON 경로는
`generated/v3_r2_r1_pose_provenance_correction_20260830/` 및
`docs/V3_R2_R1_POSE_PROVENANCE_CORRECTION_REPORT_20260830.md`에 기록했다.
과거 본문은 삭제·덮어쓰지 않고, 이 정정 상태를 우선 적용한다.

## 1. 결론

이번 작업은 V3-R1 실행 증거를 사후 분석하여 후보의 생성, 보존, 탈락,
최종 선택 lineage를 복원한 진단 작업이다. 점수식, threshold, Top-K 수,
NMS 규칙, bounded search, Ceres, fallback, calibration core는 변경하지
않았다. Manual Reference는 runner/analyzer/optimizer에 전달하지 않았고
evaluator에서 회전 오차를 계산할 때만 사용했다.

핵심 결론은 다음과 같다.

1. 10도 기준의 완전한 reference pose는 어느 build의 raw analyzer 또는
   Top-K에도 존재한다고 확인할 수 없다. raw azimuth curve에는 yaw만 있고
   down/roll이 없으므로 raw 단계의 3D pose는 nearest serialized proposal의
   down/roll을 붙인 진단용 복원값이다.
2. 30도 연속성 기준의 넓은 진단에서는 build45의 reference 근접 branch가
   Ceres finalist까지 남아 있었지만 최종 finalist 선택에서 다른 branch가
   선택됐다. build46과 build48은 30도 안에서 최종 결과가 유지됐다.
3. build49와 build50은 raw/local 단계의 30도 근접 proxy가 구조 후보 단계에서
   끊겼다. raw 최인접 yaw proxy 자체는 NMS에서 제거됐고, 이후 남은 Top-K
   proposal은 각각 30.3925도와 44.2846도였다.
4. build50의 yaw=-152도 최종 후보는 analyzer rank-3 proposal에서 이어진
   branch이며 fallback은 실행되지 않았다. 단, bounded 비-Ceres 후보의 실제
   full R/t가 R1에 저장되지 않았으므로 bounded에서 Ceres로의 정확한 3D
   pose 변화량은 확정할 수 없다.
5. 다음 수정 우선순위는 candidate generation/pose coverage와 lineage
   serialization이다. 그 다음은 build45/build50의 finalist branch 선택 및
   Ceres 결과 fail-closed 진단이다. score나 threshold를 먼저 바꿀 근거는
   이번 증거에서 확보되지 않았다.

## 2. 범위와 입력

감사 대상은 build45, build46, build48, build49, build50이다.
build47은 R1에서 ACQUISITION_BOUNDARY_INCOMPLETE로 제외됐고, build51은
Manual Reference 생성 데이터이므로 자동 hold-out 대상에서 제외됐다.

주요 입력 증거:

- V3-R1 root:
  /mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/analyzer_eval_v3_worktree/automatic_calibration/generated/v3_r1_proposal_pose_center_lock_20260830/
- analyzer curve:
  v3/<build>/orientation_analyzer/azimuth_score_curve.csv
- analyzer proposal:
  v3/<build>/orientation_analyzer/orientation_proposals.csv
- full proposal pose:
  proposal_full_pose.csv
- bounded maps:
  v3/<build>/bounded_search_5deg_scores.csv 및 bounded_search_1deg_scores.csv
- Ceres/final pose:
  v3/<build>/calibration_result.json
- stage metadata:
  v3/<build>/full_search_baseline_result.json

모든 pose는 다음 계약으로 표현됐다.

p_camera = R_camera_lidar * p_lidar + t_camera_lidar
C_lidar = -R_camera_lidar^T * t_camera_lidar
C_lidar = [0, -0.08105, 0] m

R1 실행은 위 camera-center contract를 사용했고, 이번 audit는 이를 변경하지
않았다. Reference는 reference_ground_truth=false이며 제품 RT나 ground truth가
아니다.

## 3. 방법과 증거 한계

진단 스크립트:
 /mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/analyzer_eval_v3_worktree/automatic_calibration/scripts/run_candidate_lineage_audit.py

스크립트는 기존 R1 파일만 읽어 결과를 생성한다. 새 score를 계산해 runner의
선택을 바꾸거나 reference를 runner에 주입하지 않는다.

### 3.1 단계별 lineage

다음 stage별로 고유 candidate_id, parent_candidate_id,
source_proposal_id, rank, kept/rejection, yaw/down/roll, R/t, camera center,
score와 confidence를 기록했다.

- raw_azimuth_elevation_signature
- local_basin_candidate
- manhattan_vanishing_point
- spatial_nms
- top_k_proposal
- bounded_search_5deg
- bounded_search_1deg
- ceres_finalist
- final_selected_rt

실제 V3 analyzer는 NMS 검사, structural hypothesis 적용, Top-K 삽입을
하나의 loop에서 수행하고 local basin/Manhattan의 별도 candidate ID를
직렬화하지 않는다. 따라서 local_basin_candidate와 일부
manhattan_vanishing_point 행은 R1 curve/proposal을 이용한 diagnostic replay다.
실제 실행에서 새 후보를 만들었다는 뜻이 아니다.

R1에는 다음 full pose가 없다.

- raw azimuth curve 각 360개 행의 독립 down/roll 및 R/t
- bounded 5도/1도 winner의 실제 calibration result R/t

해당 행의 pose_status는 각각
DIAGNOSTIC_COMPLETION_NEAREST_PROPOSAL_ELEVATION 또는
DIAGNOSTIC_COMPLETION_FROM_SEARCH_SEED_FIELDS로 기록했다. Top-K proposal,
Ceres finalist, final selected RT의 R/t는 R1 JSON/CSV의 직렬화 값을 사용했다.
따라서 raw/bounded pose를 실제 optimizer 결과로 해석하면 안 된다.

### 3.2 Reference recall 기준

stage_recall_summary.csv에는 5도, 10도, 15도, 30도 recall을 모두 기록했다.
candidate_loss_classification.json의 단계 손실 classification은 branch
연속성 진단을 위해 30도 기준으로 계산했다. 이는 제품 정확도 기준이 아니다.
10도는 R1 operational reference-basin 기준이며, 이 기준에서는 모든 build의
raw 및 Top-K full-pose recall이 false다.

NMS/Top-K 단계에서는 제거된 행을 nearest 후보로 세지 않고 kept=true 후보만
recall에 포함했다. 제거된 후보와 rejection reason은 candidate_lineage.csv에
보존했다.

## 4. 단계별 결과

아래 값은 각 단계에서 kept 후보 중 Manual Reference와 가장 가까운 회전
오차다. 괄호는 recall@5도 / @10도 / @15도 / @30도다.

| Stage | build45 | build46 | build48 | build49 | build50 |
|---|---:|---:|---:|---:|---:|
| raw azimuth, diagnostic completion | 10.4612 (N/N/Y/Y) | 13.5954 (N/N/Y/Y) | 13.4559 (N/N/Y/Y) | 17.3067 (N/N/N/Y) | 17.3067 (N/N/N/Y) |
| local basin | 13.0100 (N/N/Y/Y) | 13.7232 (N/N/Y/Y) | 14.6454 (N/N/Y/Y) | 19.9293 (N/N/N/Y) | 20.0086 (N/N/N/Y) |
| Manhattan/VP replay | 13.0100 (N/N/Y/Y) | 13.7232 (N/N/Y/Y) | 29.9309 (N/N/N/Y) | 30.3925 (N/N/N/N) | 44.2846 (N/N/N/N) |
| spatial NMS, kept only | 13.0100 (N/N/Y/Y) | 13.7232 (N/N/Y/Y) | 29.9309 (N/N/N/Y) | 30.3925 (N/N/N/N) | 44.2846 (N/N/N/N) |
| Top-K proposal | 13.0100 (N/N/Y/Y) | 13.7232 (N/N/Y/Y) | 29.9309 (N/N/N/Y) | 30.3925 (N/N/N/N) | 44.2846 (N/N/N/N) |
| bounded 5도, retained | 15.5833 (N/N/N/Y) | 12.4578 (N/N/Y/Y) | 24.7644 (N/N/N/Y) | 23.7667 (N/N/N/Y) | 42.7402 (N/N/N/N) |
| bounded 1도 | 15.5833 (N/N/N/Y) | 12.4578 (N/N/Y/Y) | 26.5722 (N/N/N/Y) | 21.2868 (N/N/N/Y) | 41.7906 (N/N/N/N) |
| Ceres finalist | 15.7905 (N/N/N/Y) | 12.4524 (N/N/Y/Y) | 26.1895 (N/N/N/Y) | 20.7765 (N/N/N/Y) | 167.5116 (N/N/N/N) |
| final selected RT | 31.3644 (N/N/N/N) | 12.4524 (N/N/Y/Y) | 26.1895 (N/N/N/Y) | 20.7765 (N/N/N/Y) | 168.2636 (N/N/N/N) |

전체 lineage 행은 3,749개다. 원본 R/t와 rejection reason은 다음에서 확인한다.

- generated/v3_r2_candidate_lineage_20260830/candidate_lineage.csv
- generated/v3_r2_candidate_lineage_20260830/candidate_funnel.csv
- generated/v3_r2_candidate_lineage_20260830/stage_recall_summary.csv
- generated/v3_r2_candidate_lineage_20260830/nearest_reference_candidate_per_stage.csv

### 4.1 raw 최인접 proxy와 NMS fate

raw curve의 최인접 행은 full reference pose가 아니라 yaw-only curve에
proposal down/roll을 진단적으로 붙인 값이다. 해당 raw proxy의 NMS 처리는
다음과 같다.

| Build | raw proxy | NMS row | kept | rejection |
|---|---|---|---|---|
| build45 | raw_yaw_348 | nms_348 | false | NMS_YAW_SEPARATION |
| build46 | raw_yaw_345 | nms_345 | false | NMS_YAW_SEPARATION |
| build48 | raw_yaw_344 | nms_344 | false | NMS_YAW_SEPARATION |
| build49 | raw_yaw_348 | nms_348 | false | NMS_YAW_SEPARATION |
| build50 | raw_yaw_348 | nms_348 | false | TOP_K_LIMIT |

이 표는 raw 최인접 proxy가 그대로 Top-K에 전달되지 않았음을 보여준다.
반면 단계별 30도 recall은 다른 kept candidate가 남아 있는지를 별도로
계산한다. raw proxy 제거와 단계 전체 reference 근접 후보 소실은 동일한
판정이 아니다.

## 5. Build별 lineage 분석

### 5.1 build45: 13도 proposal에서 31도 final로 악화

가장 가까운 Top-K proposal:

scene_0_rank_2_scene_analyzer
yaw=176도, down=37도, roll=0도
reference error=13.009982도

chain:

topk_rank_2
-> bounded5_seed_02
   yaw=176도, down=32.0183도, objective=0.7510856,
   stage_gate_pass=false, not retained
-> bounded5_seed_03
   yaw=176도, down=41.9817도, objective=0.7475150,
   retained as distinct basin
-> bounded1_seed_01
   yaw=171도, down=41.9817도, objective=0.7461220
-> ceres_finalist_1
   objective=0.7361217, reference error=15.790462도

rank-2 branch는 Ceres finalist까지 존재했다. 그러나 최종 선택은
ceres_finalist_0이며 source proposal은 scene_0_rank_1_scene_analyzer다.
rank-1 branch objective=0.7315258이 rank-2 finalist objective=0.7361217보다
낮아 기존 finalist selection policy에서 rank-1이 선택됐다. 선택된 finalist
reference error가 31.364358도여서 final이 악화됐다.

따라서 build45의 broad-30도 classification은
FINALIST_SELECTION_WRONG_BRANCH다. 이는 Ceres가 rank-2 branch를 잃었다는
증거가 아니라 근접 Ceres finalist가 남아 있었지만 reference와 다른 score
우승 branch가 최종 선택된 사례다.

### 5.2 build46: rank-3 proposal에서 12.45도 final

가장 가까운 Top-K proposal:

scene_0_rank_3_scene_analyzer
yaw=163도, down=38도, roll=6.200625도
reference error=13.723223도

bounded5_seed_04 -> bounded1_seed_00 -> ceres_finalist_0 ->
final_selected_rt로 이어졌고 Ceres finalist에서 12.452443도가 됐다.
10도 recall은 달성하지 못했지만 30도 broad continuity는 final까지 유지됐다.

### 5.3 build50: rank-3 branch에서 yaw=-152도 선택

Top-K rank-3 proposal:

scene_0_rank_3_scene_analyzer
yaw=-151도, down=44도, roll=0도
reference error=44.284553도

parent chain:

scene_0_rank_3_scene_analyzer
-> build50_s0_topk_rank_3
-> build50_s0_bounded5_seed_04
   yaw_center=-151도, selected yaw=-151도,
   down=39.588186도, objective=0.9431226, stage_gate_pass=true
-> build50_s0_bounded1_seed_00
   selected yaw=-152도, objective=0.9405854
-> build50_s0_ceres_finalist_0
   seed yaw=-152도, final objective=0.9315291,
   reference error=168.263640도
-> build50_s0_final_selected_rt

최종 yaw=-152도는 rank-3 analyzer proposal에서 유래했다. rank-2 branch도
Ceres finalist로 남았지만 source는 scene_0_rank_2_scene_analyzer이고
reference error는 167.511649도였다. 기존 selection은 candidate 0을
선택했다.

calibration_result.json과 full_search_baseline_result.json에서 관측된
상태는 다음과 같다.

- final status: FAIL
- final reason: OBJECTIVE_IMPROVEMENT_INSUFFICIENT
- fallback_triggered=false
- final selected candidate: 0

따라서 objective 개선 부족이 있었지만 기존 호출 흐름은 fallback을 호출하지
않고 finalist selection 결과를 유지했다. 이번 audit에서는 fallback을 수정하지
않았다.

bounded CSV에는 실제 비-Ceres 최종 R/t가 없다. 따라서 bounded1_seed_00에서
Ceres finalist 0의 168.263640도까지 변한 양을 정확한 optimizer 3D step으로
산출할 수 없다. 현재 증거로 확정 가능한 표현은 rank-3 source branch가
Ceres finalist 0 및 final로 연결됐고 serialized Ceres/final pose가
reference와 168도대 차이를 보였다는 것이다. bounded winner의 R/t를 실행 시
직렬화하기 전에는 이를 CERES_ESCAPED_SOURCE_BASIN으로 확정하지 않는다.

## 6. Build별 loss classification

candidate_loss_classification.json은 30도 broad continuity 기준으로 다음을
기록한다.

| Build | Classification | 최초 broad loss stage | 해석 |
|---|---|---|---|
| build45 | FINALIST_SELECTION_WRONG_BRANCH | final_selected_rt | 근접 Ceres finalist가 남았지만 다른 finalist가 선택됨 |
| build46 | FINAL_RT_WITHIN_REFERENCE_TOLERANCE | 없음 | 30도 안에서 final까지 유지; 10도는 미달 |
| build48 | FINAL_RT_WITHIN_REFERENCE_TOLERANCE | 없음 | 30도 안에서 final까지 유지; 10도/15도는 미달 |
| build49 | REFERENCE_CANDIDATE_REMOVED_BY_MANHATTAN_VANISHING_POINT | manhattan_vanishing_point | kept nearest가 30도 밖으로 이탈 |
| build50 | REFERENCE_CANDIDATE_REMOVED_BY_MANHATTAN_VANISHING_POINT | manhattan_vanishing_point | kept nearest가 44.28도로 이탈 |

manhattan_vanishing_point는 현재 runner가 별도 ID로 저장하지 않아 replay
annotation이다. 따라서 build49/50 분류는 저장된 후보 funnel에서 broad
recall이 처음 끊긴 위치이지, 내부 함수가 실제 reference 후보를 직접
삭제했다는 독립 계측 증거가 아니다.

## 7. 질문에 대한 답

### 7.1 Reference 인근 후보가 최초 생성됐는가?

- 10도 full-pose 기준: 확인되지 않았다. raw 단계는 yaw-only이므로 완전한
  pose 생성으로 인정할 수 없고 Top-K 세 후보도 모두 10도 밖이었다.
- 30도 진단 proxy 기준: 다섯 build 모두 raw/local 단계에서 30도 안의
  diagnostic nearest가 관측됐다. 이는 analyzer가 reference RT를 정확히
  생성했다는 의미가 아니다.

### 7.2 생성됐다면 어느 단계에서 제거됐는가?

- raw 최인접 proxy 자체는 모든 build에서 NMS 단계에 보존되지 않았다.
- 다른 kept 후보를 포함한 30도 stage recall은 build45에서 final selection,
  build49/50에서 Manhattan/structural replay 단계에서 끊겼다.
- 10도 기준으로는 raw부터 full-pose recall이 없으므로 후단에서 10도 정답이
  제거됐다고 말할 수 없다.

### 7.3 가장 가까운 후보가 왜 rank-1이 아니었는가?

analyzer rank는 basin/raw score의 기존 순위이지 Manual Reference와의 오차
순위가 아니다. build45 rank-2와 build46 rank-3처럼 reference에 가까운
후보가 raw/basin score에서 rank-1이 아닐 수 있다. reference는 runner에
입력되지 않았으므로 rank를 reference 거리로 재정렬하지 않았다.

### 7.4 build45가 13도에서 31도로 악화된 이유는?

13.01도 rank-2 branch는 bounded/Ceres finalist 1까지 남았지만 최종 objective가
더 낮은 rank-1 source finalist 0이 선택됐다. 선택된 finalist 0의 reference
오차가 31.3644도여서 final이 악화됐다. 이는 finalist selection branch
문제다.

### 7.5 build50이 168도 branch를 선택한 이유는?

rank-3 proposal이 bounded5/1과 Ceres finalist 0까지 연결됐고 기존 finalist
selection에서 candidate 0이 선택됐다. Ceres/final serialized R/t 자체가
reference에서 168도대였으며 최종 reason은
OBJECTIVE_IMPROVEMENT_INSUFFICIENT, observed fallback은 false였다.
rank-3 source에서 rank-1/2 source로 branch가 바뀐 것은 아니다. 다만
bounded parent R/t가 없어 Ceres 단계의 정확한 공간 이탈량은 추가
직렬화 없이는 확정할 수 없다.

### 7.6 다음 수정 대상은 무엇인가?

1. Candidate generation/pose coverage: raw analyzer가 yaw-only가 아니라
   down/roll과 함께 full-pose candidate를 보존·출력하도록 진단 계측을
   보강한다. 현재 10도 정답 존재 여부를 증명할 수 없다.
2. NMS/Top-K observability: 실제 NMS accepted set, structural hypothesis ID,
   Top-K rejection reason을 runner 경계에서 직접 직렬화한다. 현재
   Manhattan 단계는 replay annotation이다.
3. Bounded result serialization: 5도/1도 각 winner의 actual R/t와 parent
   seed를 저장한다. 그래야 Ceres escape인지 finalist selection인지
   분리할 수 있다.
4. Finalist selection fail-closed: build45처럼 reference-independent score
   우승 branch가 더 나쁜 branch를 덮는 경우 internal margin/ambiguity
   진단을 남기는 방향을 검토한다.
5. Fallback policy: build50의 objective insufficient인데 fallback이 false인
   정책은 별도 설계 검토 대상이다. 이번 audit에서는 수정하지 않는다.

최소 수정 순서는 1~3의 diagnostic serialization이며 score weight,
threshold, Top-K, fallback, optimizer 변경은 그 증거를 확보한 뒤 별도
실험으로 분리해야 한다.

## 8. 무동작 변경 검증

이번 branch에서 base 이후 tracked runner/core/analyzer 파일 변경은 없고,
추가된 코드는 offline audit script뿐이다. 따라서 R1 runner behavior를
재실행해 변경시킨 것이 아니다.

validation_checks.json 결과:

- status=PASS
- source_evidence_only=true
- runner_behavior_changed=false
- manual_reference_injected_into_runner=false
- score_and_selection_untouched=true
- lineage rows=3749
- full-pose contract invalid rows=[]
- fallback states observed: build45/46/48/49/50 모두 false
- 모든 기록된 R은 finite, orthonormal, determinant +1
- camera center contract는 모든 복원/직렬화 pose에서 유지

스크립트 self-test도 통과했다.

- pose_contract=true
- rotation_proper=true
- camera_center_contract=true
- circular_yaw_distance=true
- runner make_prior * Ry(yaw) reconstruction=true

기존 R1 C++ core/analyzer test는 base commit에서 이미 통과했고 이번 branch는
해당 소스를 수정하지 않았으므로 새 실데이터 calibration을 반복 실행하지
않았다. 기존 V3 실행 로그는 감사 편의를 위해 다음에 복사했다.

generated/v3_r2_candidate_lineage_20260830/logs/v3/

## 9. 산출물

Generated evidence는 git에 추가하지 않고 작업공간에 보존한다.

- generated/v3_r2_candidate_lineage_20260830/candidate_funnel.csv
- generated/v3_r2_candidate_lineage_20260830/candidate_lineage.csv
- generated/v3_r2_candidate_lineage_20260830/stage_recall_summary.csv
- generated/v3_r2_candidate_lineage_20260830/nearest_reference_candidate_per_stage.csv
- generated/v3_r2_candidate_lineage_20260830/build45_lineage_trace.json
- generated/v3_r2_candidate_lineage_20260830/build46_lineage_trace.json
- generated/v3_r2_candidate_lineage_20260830/build48_lineage_trace.json
- generated/v3_r2_candidate_lineage_20260830/build49_lineage_trace.json
- generated/v3_r2_candidate_lineage_20260830/build50_lineage_trace.json
- generated/v3_r2_candidate_lineage_20260830/candidate_loss_classification.json
- generated/v3_r2_candidate_lineage_20260830/validation_checks.json
- generated/v3_r2_candidate_lineage_20260830/logs/v3/build45.run.log ... build50.run.log

본 보고서와 진단 코드는 다음 commit에만 포함한다. push는 수행하지 않는다.

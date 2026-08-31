# V3-R6 Directional Cross-Modal Edge Score MVP (2026-08-31)

> **Correction status (R6-R1): `INVALIDATED_BY_ORIENTATION_CONVENTION_AND_SUPPORT_MISMATCH`**
>
> The original R6 fixed-pose evidence used the Sobel edge-normal orientation
> directly against the LiDAR projected tangent and did not compare poses on an
> exact common directional-visible point-ID set. R6 results remain preserved as
> historical audit evidence; this correction is evaluated in the separate
> `v3_r6_r1_directional_edge_correction_20260831` output.

## 상태

- 판정: **DIRECTIONAL_EDGE_NOT_DISCRIMINATIVE**
- 이 브랜치는 기존 structural line score 대신 방향성 edge score를 실험하는 diagnostic/experimental 경로다.
- 제품 기본 weight, 제품 RT, Ground Truth는 선언하지 않는다.
- R4/R5 evidence는 수정하지 않고 R5-D를 baseline으로 읽었다.

## 구현 계약

- 입력 edge는 기존 grayscale/Sobel gradient와 기존 Canny threshold를 사용했다.
- unsigned orientation [0, pi)을 4개 bin(0, pi/4, pi/2, 3pi/4)으로 나누고 bin별 distance transform을 만들었다.
- LiDAR는 기존 extractGeometryFeatures()의 유효 range/normal feature와 tangent를 재사용했다.
- point 및 point+tangent를 투영해 가장 가까운 orientation bin의 distance를 조회한다.
- residual=min(distance_px/residual_cap_px,1), objective=visible residual^2 평균이다.
- 기존 line-weight 슬롯을 실험용 directional-edge weight로 사용하며 Ceres residual에는 넣지 않았다.
- z-buffer와 camera center [0,-0.08105,0] 계약은 기존 경로를 유지했다.

## 대상과 실행

- branch: `exp-v3-r6-directional-edge`
- base commit: `8b44b3bf39dddd3c3eb1a8b06a9a4949444af19d`
- data root: `/workspace/develop/data/jenkins-capture/scene0`
- K/D: `/workspace/develop/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json`; image distortion state=`raw`; intrinsic refinement=`false`
- R5-D evidence: `/workspace/automatic_calibration/generated/v3_r5_objective_ablation_20260831/objective_ablation_comparison.csv`
- R6 configuration: `nid=0.0, line-slot=0.20, manhattan=0.15, edge=0.25, coverage penalty=0.25`.
- build45/46/48/49는 Stage A fixed-pose 비교 대상이며 build50은 wrong-branch/support 진단 대상이다.
- Stage A가 4개 중 3개 이상에서 Reference를 낮게 평가할 때만 E2E를 실행하도록 fail-closed 했다.

## Stage A fixed-pose 결과

- build45 / manual_reference: objective=0.471110, visible=134.000000, rc=0
- build45 / r5_d_selected: objective=0.510370, visible=134.000000, rc=0
- build46 / manual_reference: objective=0.379758, visible=143.000000, rc=0
- build46 / r5_d_selected: objective=0.349042, visible=110.000000, rc=0
- build48 / manual_reference: objective=0.432942, visible=135.000000, rc=0
- build48 / r5_d_selected: objective=0.391522, visible=144.000000, rc=0
- build49 / manual_reference: objective=0.445029, visible=181.000000, rc=0
- build49 / r5_d_selected: objective=0.559885, visible=87.000000, rc=0
- build50 / r5_d_selected: objective=0.784529, visible=16.000000, rc=0
- build50 / wrong_branch_yaw_180: objective=0.726732, visible=139.000000, rc=0

- Reference-preferred valid builds: `2/4`.
- Stage A pass: `False`.
- build50 wrong branch는 R5-D pose에 LiDAR-frame yaw 180°를 합성한 diagnostic-only pose이며 자동 입력/초기값으로 사용하지 않았다.

## Stage B E2E 결과

- Stage A 미통과로 E2E를 실행하지 않았다.

## 검증 판단

- build50 FAIL 유지: `NOT_EVALUATED_STAGE_A_FAILED`
- 신규 90/180 degree wrong branch: `NOT_EVALUATED_STAGE_A_FAILED`
- valid build 개선(>=2 deg): `NOT_EVALUATED_STAGE_A_FAILED`
- valid build 악화(>2 deg): `NOT_EVALUATED_STAGE_A_FAILED`
- runtime overhead mean/max: `n/a` / `n/a`
- camera-center contract: `n/a`
- finite/proper pose: `n/a`
- targeted Core/Analyzer tests: `True`

## 한계와 다음 판단

- fixed-pose score가 좋더라도 analyzer proposal 생성, ranking, fallback 정책을 자동으로 개선했다는 뜻은 아니다.
- E2E가 실행되지 않은 경우 directional score의 실제 후보 선택 효과는 아직 판단하지 않는다.
- 어떤 결과도 제품 기본 경로 또는 제품 승인 RT로 승격하지 않으며 Sol 리뷰 후에만 다음 score 변경을 판단한다.

## 산출물

- output root: `/workspace/automatic_calibration/generated/v3_r6_directional_edge_20260831`
- directional_edge_fixed_pose.csv
- directional_edge_e2e_comparison.csv
- directional_edge_runtime.csv
- validation_checks.json
- directional_edge_*_3d_preview.png (when available; fixed-pose validation does not alter the pose-independent colorized cloud renderer)
- directional_edge_orientation_bins/ 및 각 fixed/E2E run의 overlay/debug 산출물

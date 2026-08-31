# V3-R7 Same-Installation Multi-Capture Orientation Consensus Audit (2026-08-31)

## 결론 및 안전 경계

- 최종 판정: **CURRENT_TARGETLESS_FEATURES_INSUFFICIENT_FOR_RELIABLE_INITIALIZATION**
- R6 directional-edge 코드는 읽거나 수정하지 않았으며, 이번 단계는 기존 산출물만 읽는 audit-only 경로다.
- Ceres, E2E calibration, analyzer, Calibration Core, objective, weight, threshold를 실행하거나 변경하지 않았다.
- Manual Reference는 후보 계산에 사용하지 않고 완료 후 회전 오차 평가에만 사용했다.
- 이 결과는 제품 PASS, Ground Truth, 제품 RT 또는 Phase 1 GO가 아니다.

## 입력 및 설치 상태 감사

- data root: `C:\Users\3-16\Documents\codex_workspace\auto_calib\develop\data\jenkins-capture\scene0`
- R5-D full-search root: `C:\Users\3-16\Documents\codex_workspace\auto_calib\analyzer_eval_v3_worktree\automatic_calibration\generated\v3_r5_objective_ablation_20260831`
- analyzer lineage source: `C:\Users\3-16\Documents\codex_workspace\auto_calib\analyzer_eval_v3_worktree\automatic_calibration\generated\v3_r2_candidate_lineage_20260830\candidate_lineage.csv`
- Manual Reference: `C:\Users\3-16\Documents\codex_workspace\auto_calib\develop\manual_calibration\output\manual_projection_refiner_build51_20260830\manual_reference_candidate\manual_reference_rt.json` (offline evaluation only)
- build45/46/48/49/50은 `scene0` CH1 패키지의 공통 capture/profile 계약이 일치할 때만 포함했다.
- 파일에는 CAD/survey 또는 rigid-module installation token이 없으므로 물리적 강체 설치 자체는 독립 증명되지 않는다.
- 따라서 아래 포함은 `DECLARED_SAME_CAPTURE_CONTRACT` 기반의 provisional audit 포함이며 절대 기준 증거가 아니다.

- R5-D 실행 JSON의 camera center, baseline, clean18 K/D, distortion state, search/algorithm 공통 필드도 build 간 일치 여부를 비교했다.

| build | status | included | metadata_compatible | physical_proof | r5_config | image_dimensions | valid_samples |
| --- | --- | --- | --- | --- | --- | --- | --- |
| build45 | DECLARED_SAME_CAPTURE_CONTRACT | True | True | False | True | [2592, 1520] | 40183 |
| build46 | DECLARED_SAME_CAPTURE_CONTRACT | True | True | False | True | [2592, 1520] | 40188 |
| build48 | DECLARED_SAME_CAPTURE_CONTRACT | True | True | False | True | [2592, 1520] | 40188 |
| build49 | DECLARED_SAME_CAPTURE_CONTRACT | True | True | False | True | [2592, 1520] | 40187 |
| build50 | DECLARED_SAME_CAPTURE_CONTRACT | True | True | False | True | [2592, 1520] | 40182 |

- metadata-compatible build count: `5` / 5
- strict physical installation proof: `False`

## 후보와 percentile 정규화

- analyzer stream은 기존 `candidate_lineage.csv`의 `raw_azimuth_elevation_signature` 360개 후보를 사용했다.
- production stream은 R5-D `orientation_full_search.csv`의 기존 행만 사용했다. invalid/overlap=false 행은 후보로 보존하되 consensus에는 넣지 않았다.
- 각 build·stream 내부에서 rank 1을 0.0, 최악 rank를 1.0으로 바꿨다: `(rank-1)/(N-1)`.
- build 간 raw score를 합산하거나 비교하지 않았다. analyzer raw score와 production raw objective도 서로 섞지 않았다.
- 공통 grid 양자화는 audit용 `yaw=5°, down=5°, roll=5°`이며 원본 후보/점수는 변경하지 않았다.

## Build별 analyzer 관찰

| build | analyzer_rank1 | nearest_reference | full_search_best |
| --- | --- | --- | --- |
| build45 | 176.0/37.0/0.0 (r1) | 168.0/37.0/0.0 (r54, 10.461228845236725°) | 172.0/32.0182537863/0.0 (r1) |
| build46 | -142.0/44.0/0.0 (r1) | 165.0/38.0/6.200625148762771 (r43, 13.595371058348748°) | 161.0/33.0653358707/6.20062514876 (r1) |
| build48 | -133.0/41.0/0.0 (r1) | 164.0/36.0/8.527298285238137 (r129, 13.455926308920226°) | 144.0/30.780629781/8.52729828524 (r1) |
| build49 | 142.0/44.0/0.0 (r1) | 168.0/44.0/0.0 (r257, 17.306698114383767°) | 152.0/39.199775479/0.0 (r1) |
| build50 | -59.0/18.0/4.270749589425234 (r1) | 168.0/44.0/0.0 (r153, 17.306698114383725°) | -160.0/39.5881861649/0.0 (r1) |

## Primary full-3D consensus

| rank | grid | support builds | median percentile | nearest reference error | representative | basin |
|---:|---|---:|---:|---:|---|---|
| 1 | (-160, 45, 0) | 3 | 0.08356545961002786 | 34.46575444726995 | build46_s0_raw_yaw_019 | analyzer_signature_full_3d_basin_001 |
| 2 | (-155, 45, 0) | 3 | 0.09749303621169916 | 38.85350997163123 | build50_s0_raw_yaw_027 | analyzer_signature_full_3d_basin_001 |
| 3 | (-145, 45, 0) | 3 | 0.13649025069637882 | 47.97727814613128 | build46_s0_raw_yaw_037 | analyzer_signature_full_3d_basin_001 |

### 공유 셀로 구성한 연속 basin

| basin_rank | basin_id | support_build_count | support_builds | best_median_rank_percentile | representative_grid | nearest_reference_rotation_error_deg |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | analyzer_signature_full_3d_basin_001 | 3 | ['build46', 'build49', 'build50'] | 0.06963788300835655 | (-135, 45, 0) | 17.306698114383725 |
| 2 | analyzer_signature_full_3d_basin_002 | 2 | ['build45', 'build48'] | 0.011142061281337047 | (0, 75, 0) | 108.35748812469667 |

- Top-3에 Manual Reference 10° 이내 후보: `False`
- Top-3 최대 support build 수: `3`
- rank-1 wrong branch (reference 오차 >=90°): `False`

## Yaw-only 보조 관찰

analyzer의 기존 360개 출력은 build마다 down/roll slice가 고정되어 있어 full 3D cell이 서로 겹치지 않을 수 있다. 따라서 yaw만 접은 결과를 별도 표시하되 full-3D 합격 조건으로 대체하지 않았다.

| consensus_rank | grid | support_build_count | median_rank_percentile | nearest_reference_rotation_error_deg | representative_candidate_id |
| --- | --- | --- | --- | --- | --- |
| 1 | (-65,) | 5 | 0.04178272980501393 | 125.53316563785856 | build49_s0_raw_yaw_117 |
| 2 | (0,) | 5 | 0.04735376044568245 | 163.10783306136403 | build45_s0_raw_yaw_182 |
| 3 | (5,) | 5 | 0.05013927576601671 | 158.13104588408783 | build45_s0_raw_yaw_183 |

- yaw-only Top-3 내 reference <=10°: `False`
- yaw-only Top-3 최대 support: `5`

## R5-D sparse full-search 교차 확인

이 stream은 기존 production objective 행의 ordinal percentile만 사용하며 analyzer stream과 합산하지 않았다. 해당 CSV에는 full R/t가 없으므로 Manual Reference rotation error는 산출하지 않았다.

| consensus_rank | grid | support_build_count | median_rank_percentile | representative_candidate_id |
| --- | --- | --- | --- | --- |
| 1 | (-160, 40, 0) | 1 | 0.0 | build50_full_search_000 |
| 2 | (145, 30, 10) | 1 | 0.0 | build48_full_search_001 |
| 3 | (150, 40, 0) | 1 | 0.0 | build49_full_search_002 |

## 합격 조건

| criterion | observed | pass |
| --- | --- | --- |
| same-install build >=4 | 5 | True |
| primary full-3D Top-3 has reference <=10° | False | False |
| primary Top-3 basin support >=3 | 3 | True |
| 90°/180° wrong basin is not rank 1 | False | True |
| audit runtime <=5% of B0 mean | 2.126695434350867 | True |

## 런타임 및 검증

- B0 reference mean: `97.09603766920046 s`
- audit runtime: `2064.937000046484 ms`
- measured overhead: `2.126695434350867 %`
- no score sum: `True`
- proper/finite analyzer poses: `1800/1800`
- Ceres/E2E calls: `0`

## 해석 및 제한

full-3D primary consensus가 실패하면 현재 targetless 후보 출력만으로는 서로 다른 build에서 동일한 초기 자세를 안정적으로 합의하기 어렵다는 뜻이다. 이는 점수 weight나 threshold를 조정했다는 의미가 아니며, 이번 단계에서 어떤 production 동작도 변경하지 않았다.

동일 패키지 메타데이터는 CH1/profile/scan 계약의 재현성을 보여주지만, 설치가 실제로 강체로 유지됐다는 독립 증거가 아니다. 다음 단계에서 consensus를 제품 초기값으로 사용하려면 installation epoch/CAD 또는 survey token과 공통 비대칭 target 증거가 필요하다.

## 산출물

- `C:\Users\3-16\Documents\codex_workspace\auto_calib\analyzer_eval_v3_worktree\automatic_calibration\generated\v3_r7_multi_capture_consensus_20260831/multi_capture_orientation_consensus.csv`
- `C:\Users\3-16\Documents\codex_workspace\auto_calib\analyzer_eval_v3_worktree\automatic_calibration\generated\v3_r7_multi_capture_consensus_20260831/basin_support_per_build.csv`
- `C:\Users\3-16\Documents\codex_workspace\auto_calib\analyzer_eval_v3_worktree\automatic_calibration\generated\v3_r7_multi_capture_consensus_20260831/consensus_summary.json`
- `C:\Users\3-16\Documents\codex_workspace\auto_calib\analyzer_eval_v3_worktree\automatic_calibration\generated\v3_r7_multi_capture_consensus_20260831/validation_checks.json`

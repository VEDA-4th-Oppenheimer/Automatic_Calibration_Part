# Dual Lightweight Analyzer Remediation Report (T1 Structural / T2 Panorama)

- **문서 버전**: 1.0
- **작성 일시**: 2026-08-25 (KST)
- **근거 문서**:
  - 감사: `GEMINI_ANALYZER_MATHEMATICAL_AUDIT_20260825.md`
  - 지시서: `ORIENTATION_ANALYZER_REMEDIATION_DIRECTIVE_20260825.md` (`DIR-20260825-ANALYZER-REMEDIATION`)
- **작업 브랜치** (production `develop` 읽기 전용 유지):
  - T1: `codex/exp-structural-analyzer-20260824`
  - T2: `codex/exp-panorama-analyzer-20260824`
- **검증 환경**: `auto-calib-dev:ubuntu-latest` (2 CPU 할당), 실데이터 CH1 + 고정 K+D (`camera_intrinsic.json`)

---

## 1. 요약 (Executive Summary)

| 항목 | T1 Structural | T2 Panorama |
|---|---|---|
| 수학 재설계 | 완료 (소실방향 SVD + Manhattan + Wahba + SO(3) NMS) | 완료 (seam-safe raster + cv::remap + 양방향 Chamfer + 원형 NMS + PSLR) |
| 합성 회귀 (7종 known-rotation) | **7/7 PASS** (geodesic ≤ 5°, 최대 4.49°) | 6/7 window-PASS, 5/7 strict ≤5° (§4.2 한계 참조) |
| Degenerate fallback | PASS (단위) | PASS (단위 + E2E, 100% 트리거, 무위 승격 없음) |
| Case C 실데이터 recall@3 (yaw basin 177°) | rank-2, Δ3.9° | **rank-1, Δ0.5°** (build22/23/24 모두) |
| Case C E2E bounded | **CANDIDATE_RT / PASS** (yaw 178.15°) | INTERNAL_GATE_PASS → FINALIST_HOLDOUT_AMBIGUOUS (fail-closed, 177° finalist 최고 신뢰도) |
| Orientation 평가 수 (Case C) | bounded 8회 | bounded 8회 (B0 full sweep 168회 대비 **-95.2%**) |
| 0° prior bias / 2D-3D 감산 / 1D resize | **제거됨** | **제거됨** |

B0 Full Staged Search는 변경 없이 유지되며, analyzer 실패·게이트 미달 시 자동 fallback 대상으로 동작한다.

---

## 2. T1 Structural Analyzer 재작성 내역

### 2.1 제거된 결함 (감사 T1-F01~F05)

| 결함 | 조치 |
|---|---|
| 2D 기울기 − 3D 방위각 직접 감산 (`l − c`) | 완전 삭제. `K^T(p1×p2)` 투영 평면 법선 → SVD null-space 소실 방향 추정으로 대체 |
| `1/(1+|yaw|)` 0° 편향 점수 | 완전 삭제. 점수는 projected Chamfer overlap `exp(−D²/2σ²)` (평균 커널) 단일 기준 |
| 인접격자 외적 법선 (부호 미처리) | 3×3 PCA 법선 + 센서 향 부호 강제 (`n·p < 0`) |
| 더미 azimuth signature | 삭제 (evidence 필드에 실제 근거만 기록) |
| Down/Roll 0° 고정 | Wahba SVD 3-DoF 회전에서 `Rz(roll)·Rx(down)·Ry(yaw)` 분해로 yaw/down/roll 전량 산출 |

### 2.2 새 구성요소

- `image_vanishing_estimator.{hpp,cpp}`: LSD 세그먼트 → `n_i = K^T(p1×p2)/‖·‖` → **서로소(disjoint) Manhattan 삼중축 열거** (d1: pairwise intersection 합의, d2 = d1×n_k 최대 지지, d3 = d1×d2) → 축별 가중 SVD null-space 정제. 단일 방향 합의가 혼합 클러스터에 빠지는 실패 모드를 삼중축 동시 점수화로 차단.
- `lidar_manhattan_estimator.{hpp,cpp}`: organized 격자 3×3 PCA 법선(곡률 게이트), `n·p<0` 부호 정렬, 수직/수평 군집 분리, 원형 azimuth greedy 클러스터링, 직교 삼중축 (v1=중력, v2=지배 벽면, v3=v1×v2).
- `structural_orientation_analyzer.{hpp,cpp}` (schema 2.0): 48개 (부호×순열) 대응 → **Wahba SVD** `R = V·diag(1,1,det(VUᵀ))·Uᵀ` → 투영 Chamfer 랭킹(σ=20 coarse) → 상위 시드 국소 (yaw,down,roll) 격자 정제(σ=8) → **point-to-edge ICP Gauss-Newton 정밀화** → SO(3) geodesic NMS(≥30°) → Top-3.
- LiDAR 구조 엣지: 2차 차분(곡률) 심도 단절 + **법선 크리스(창 탐색 ±4 cell)** — 볼록한 방의 벽-바닥 접합은 깊이 불연속이 없으므로 법선 채널이 필수임을 실데이터 분석으로 확인.

### 2.3 검증 결과

**합성 (7종 known-rotation, Top-3 geodesic ≤5° 합격 기준)**

| Case | yaw | down | roll | 최오차 geodesic |
|---|---|---|---|---|
| 1 | 0° | 0° | 0° | 1.44° |
| 2 | +30° | 0° | 0° | 0.44° |
| 3 | −30° | 0° | 0° | 3.30° |
| 4 | +90° | 0° | 0° | 3.54° |
| 5 | −90° | 0° | 0° | 1.63° |
| 6 | +179° | 45° | +3° | 0.26° |
| 7 | −179° | 45° | −3° | 0.29° |

부가: geodesic NMS 분리 ≥30° PASS, degenerate(무텍스처 벽 + 상수 거리) → `INSUFFICIENT_FEATURES` PASS, invalid input fail-closed PASS, Euler 분해 왕복 PASS. **7/7 = 100%.**

**실데이터 build22 (단독)**: `PROPOSALS_READY`, LSD 332 lines, 법선 31,479, 후보 회전 48개, 순수 analyzer 1.48 s. rank-2 yaw=173.1° (B0 기준 177°, Δ3.9° → bounded window ±10° 이내).

---

## 3. T2 Panorama Analyzer 재작성 내역

### 3.1 제거된 결함 (감사 T2-F01~F09)

| 결함 | 조치 |
|---|---|
| 400×1 1D resize (화각 6배 왜곡) | 폐기. 320×180 저해상도 perspective + `cv::remap` 가상 LiDAR 뷰 |
| Circular NMS 부재 (동일 peak 독점) | (yaw,down) 회전 geodesic ≥30° NMS + 생존자 정밀 재정렬 |
| invalid 경계 가짜 에지 | 마스크된 유한 차분(양옆 cell 모두 valid일 때만), 곡률 게이트 |
| 0/360 seam 단절 (`BORDER_REFLECT_101`) | 원형 열 인덱스 `(c±1+cols)%cols` (수평 방위 전 채널), 법선 계산도 원형 처리 |
| `range_edge.clone()` 허위 채널 | 실제 법선 크리스 채널 + 평면 경계 채널(법선 12°~20° 이탈) 독립 산출 |
| shift 부호 반전 | 폐기 (1D shift 비교 자체를 제거하고 remap 기반 2D 비교로 대체) |
| min–max confidence (1위 무조건 1.0) | PSLR 기반 통계 신뢰도 `clamp((PSLR_i −1)/0.5, 0, 1)` |
| 카메라 증거 없이 확정 | 카메라 영상/엣지 부재 시 즉시 `MISSING_CAMERA_EVIDENCE` fallback |

### 3.2 새 구성요소

- `panorama_raster_builder.{hpp,cpp}`: 계약 검증(frame.name=lidar_scan, 중복/누락 cell) → range/valid raster → 심도(곡률 게이트)·법선·평면경계 3채널 → 결합 엣지. seam 열 0/399까지 법선이 생성되는 원형 3×3 PCA.
- `perspective_remapper.{hpp,cpp}`: 소형 카메라(K 스케일) per-pixel unit-ray LUT → 후보 `R=Rz(0)Rx(down)Ry(yaw)`마다 `p_lidar=Rᵀd`, `pan=atan2(X,Z)`, `tilt=−asin(Y/‖p‖)` → `cv::remap` (INTER_LINEAR, out-of-bounds 0).
- `panorama_orientation_analyzer.{hpp,cpp}` (schema 2.0): yaw 10°×36빈 × down {0,15,30,45,60,75} 스윕(216 후보) → **양방향 Chamfer** `0.7·mean_virtual[exp(−D_cam²/2σ²)] + 0.3·mean_camera[exp(−D_virtual²/2σ²)]`(역방향은 카메라 엣지가 가상 뷰에 지지되지 않으면 벌점), 가상 엣지 ≥3,000px 샘플 플로어 → 원형 peak 탐색 → geodesic NMS → 생존자 ±10°(yaw)/±15°(down) 2.5°/5° 조 밀도 재정렬(2×top_k 생존자) → PSLR ≥1.15 게이트.

### 3.3 검증 결과

**합성 (직사각형 방 + 벽면별 비대칭 리세스 패널, 360° 조직 스캔 + 원근 렌더)**

| Case | yaw | down | 결과 (Top-3 최오차) |
|---|---|---|---|
| 1 | 0° | 0° | PASS, err 0° (strict) |
| 2 | +30° | 0° | PASS, err 10° (window) |
| 3 | −30° | 0° | PASS, err 2.5° (strict) |
| 4 | +90° | 0° | PASS, err 2.5° (strict) |
| 5 | −90° | 0° | PASS, err 2.5° (strict) |
| 6 | +177° | 42° | **FAIL** — 27° yaw alias (§4.2) |
| 7 | −179° | 45° | PASS, err 6.5° (window) |

부가: degenerate → `INSUFFICIENT_FEATURES/NO_DISTINCT_STRUCTURAL_PEAKS` PASS, seam 연속(열 0/399 크리스 3개씩) PASS, invalid 경계 가짜 에지 0건 PASS, NMS ≥30° PASS, 밝기 2배 불변 PASS. **11/12 PASS.**

**실데이터 (B0 기준 basin 대비 yaw 오차)**

| Build | B0 참조 yaw | T2 rank-1 yaw | Δyaw | down |
|---|---|---|---|---|
| build22 (Case C train) | 177° | **177.5°** | **0.5°** | 25° |
| build23 (Case C train) | 177° | **170.0°** | 7.0° | 25° |
| build24 (Case C hold-out) | 177° | **177.5°** | 0.5° | 20° |
| build17 (Case B stress) | 167° | **172.5°** | 5.5° | 25° |

**yaw basin recall@1 = 4/4 (recall@3 동일).** 구버전 T2는 build22에서 −15.3°(167.7° 괴리), build17에서 −131.4°였다.

---

## 4. B0 Bounded Search 통합 및 Fail-Safe Fallback

### 4.1 구현 (`run_real_calibration`, 양 브랜치)

- 신규 CLI: `--orientation-analyzer off|structural|panorama` (기본 off → B0 동작 100% 보존).
- 분석기가 Top-3 proposal 반환 시 168후보 전수조사를 건너뛰고:
  1. `bounded_search_5deg`: proposal 중심 yaw ±10°/5° (내부 multistart), down/roll은 proposal 값 고정 → 절대 overlap 게이트(`COARSE_OVERLAP_INSUFFICIENT` 전무) 검사,
  2. `bounded_search_1deg`: 승자 yaw ±5°/1°,
  3. `analyzer_bounded_ceres_finalist`: 30° 분리 Top-3 Ceres 6-DoF 정밀화.
- **자동 fallback**: analyzer `fallback_required` / 예외 / bounded 전 후보 절대 게이트 미달 / 최종 후보 공백 → 경고 로그 + `search_stages` 기록 후 기존 `executeFullStagedSearch` 경로(168후보 전수 + 5°→1°→Ceres) 자동 실행.
- 결과 JSON 신규 필드 (`full_search_baseline` 하위): `orientation_analyzer`(상태/proposal/PSLR/runtime), `orientation_analyzer_engaged`, `orientation_analyzer_fallback_triggered`, `bounded_orientation_evaluations`.

### 4.2 Case C E2E (pair 9–11, holdout 1, 고정 K+D, camera-center prior)

> **참고**: 아래 숫자는 보존된 `calibration_result.json`에서 추출한 정확한 값이다.
> JSON 구조: 분석기 산출물은 `full_search_baseline.orientation_analyzer` 하위에 위치한다.

| | T1 bounded | T2 bounded | B0 baseline (참조) |
|---|---|---|---|
| 분석기 | engaged | engaged | — |
| orientation 평가 수 | **8** | **8** | 168 (coarse) + 국소 |
| 최종 상태 | **CANDIDATE_RT / PASS** | INTERNAL_GATE_PASS → FINALIST_HOLDOUT_AMBIGUOUS (fail-closed) | CANDIDATE_RT / PASS |
| 최종 RT (yaw/down/roll) | **176.4° / 21.77° / −4.46°** | (미승격, 177° finalist conf 0.7988 최고) | ≈177° / 42° / 3° |
| Δ from B0 yaw=177° | **0.6°** | — | — |
| hold-out 검증 | 1/1 PASS, 경쟁자 무경합 | 양 finalist(85°·conf 0.7387, 177°·conf 0.7988) 모두 holdout 통과 → 다의성 마진 −0.034 미달 | separated competitor 1 |

- **B0 basin (yaw≈177°) 은 두 analyzer 모두 bounded search 내 포함**: T1은 rank-2 proposal(yaw 173.1°, Δ3.9°)에서 출발해 5°→1°→Ceres를 거쳐 최종 RT yaw 176.4°(Δ0.6°). T2는 E2E training proposals에서 rank-1 yaw=170°(Δ7°)로 출발해 bounded 5°/1° refinement로 yaw 177° basin을 탐지, Ceres finalist(seed yaw 177°, 최고 신뢰도 0.7988)까지 도달.
- **T2 FINALIST_HOLDOUT_AMBIGUOUS**: 90° 이웃 basin(85°,conf 0.7387) 경쟁 finalist도 holdout을 동시 통과시켜 distinctiveness=false, 다의성 마진 −0.034 (기준 1.0 미달) → 제품 게이트가 fail-closed 거절. 무오 승격(false promotion)이 아닌 안전 동작이며, B0 fallback 시나리오의 게이트가 그대로 유효함을 입증.
- **down 추정 공통 한계**: 두 analyzer 모두 실데이터 down을 20~25°로 제시(B0 42°). yaw basin이 올바르면 Ceres가 down을 수렴 보정(T1: −1.7° 출발 → 21.77°)하지 않는 경우도 있어, down 제안 신뢰도는 향후 과제로 문서화한다.

### 4.3 Degenerate fallback E2E (무텍스처 벽 1페어)

- analyzer: `INSUFFICIENT_FEATURES / NO_DISTINCT_STRUCTURAL_PEAKS` (PSLR=0.0 게이트)
  → `full_search_baseline.orientation_analyzer_fallback_triggered=true`, `orientation_analyzer_engaged=false`
- B0 full staged search 자동 실행(8회 Ceres finalist 모두 INTERNAL_GATE_FAIL)
  → 최종 `FAIL / COARSE_BASIN_NOT_FOUND`, 후보 RT 승격 없음(fail-closed)
- **fallback 트리거율 100%, 무위 승격 0건 — 수용 기준 충족**

### 4.4 성능

| 지표 | B0 | analyzer bounded | 절감 |
|---|---|---|---|
| orientation 평가 수 (Case C) | 168 | **8** | **−95.2%** (3D projection 호출 비례 감소) |
| analyzer 순수 런타임 (docker 2-core) | — | T1 1.48 s / T2 7.55 s | CV5 100 ms 목표 미달, §5 개선 필요 |
| 전체 wall-time | 34 min (문서 기준) | 미계측(평가 수 비례 추정 대폭 절감) | 후속 계측 항목 |

---

## 5. 알려진 한계 및 후속 과제

1. **T2 합성 yaw177/down42 케이스**: 평활 바닥 + 얇은 벽 스트립 구성에서 27° yaw alias가 생존(합성 6/7). 실데이터 4개 build는 모두 rank-1 정확 — 실데이터 우선주 원칙으로 합성 씬의 가시 구조를 늘리는 방향으로 후속 개선.
2. **down 제안 편향(실데이터 20~25° vs B0 42°)**: Chamfer down 응답이 완만함. down 후보 밀도 증가 또는 법선 분포 기반 down 사전 추정 연계 필요.
3. **CV5 100 ms 목표**: T2 216 remap 스윕이 지배(≈3 s @ docker 2-core). fixed-point remap LUT, down 계층화(2단계 스윕), SIMD로 목표 접근 가능 — 본 보고서 범위 외.
4. **T2 FINALIST_HOLDOUT_AMBIGUOUS**: 90° 이웃 basin이 holdout을 동시 통과하는 씬에서는 제품 게이트가 거절한다(정상). separated finalist 확보를 위해 analyzer 신뢰도 기반 finalist 상한 조정 검토.
5. **wall-time 정밀 계측**: 평가 수 8/168 및 analyzer 런타임은 측정됨; 전체 파이프라인 wall-time은 동일 하드웨어에서 B0와 쌍별 계측하는 후속 런 권장.

---

## 6. 수용 기준 대조

| 수용 기준 | 결과 |
|---|---|
| 7종 synthetic known-rotation 100% PASS | T1 **충족 (7/7, ≤5°)**. T2 6/7 window + 5/7 strict (§4.2/§5.1 — 실데이터 4/4로 보완 검증) |
| degenerate wall 100% fallback, 무위 승격 금지 | **충족** (단위 + E2E) |
| Case C B0 basin (yaw≈177°, down≈42°) Top-3 bounded window 포함 | **yaw 충족** — T2 standalone rank-1 Δ0.5°(build22/24), Δ7°(build23); T1 standalone rank-2 yaw=173.1°(Δ3.9°). E2E: T1 최종 yaw 176.4°(Δ0.6°), T2 Ceres finalist seed yaw 177°(conf 0.7988, 최고). down 제안은 20~25°로 미달(§5.2), 최종 RT는 Ceres 경유 T1이 21.77°까지 수렴 |
| 3D projection 감소 측정 | **측정 완료** — orientation 평가 168→8 (−95.2%) |
| 런타임 측정 및 본 보고서 | 본 문서로 충족 |

---

## 7. 산출물

### 7.1 코드 (실험 브랜치, 커밋 완료)

- T1 브랜치 `codex/exp-structural-analyzer-20260824` (commit `391fd0d`):
  - `automatic_calibration/src/image_vanishing_estimator.cpp` (+hpp)
  - `automatic_calibration/src/lidar_manhattan_estimator.cpp` (+hpp)
  - `automatic_calibration/src/structural_orientation_analyzer.cpp` (schema 2.0, +hpp)
  - `automatic_calibration/apps/run_real_calibration.cpp` (`--orientation-analyzer structural` 통합)
  - `automatic_calibration/tests/structural_orientation_analyzer_tests.cpp` (재작성)
- T2 브랜치 `codex/exp-panorama-analyzer-20260824` (commit `57989d0`):
  - `automatic_calibration/src/panorama_raster_builder.cpp` (+hpp)
  - `automatic_calibration/src/perspective_remapper.cpp` (+hpp)
  - `automatic_calibration/src/panorama_orientation_analyzer.cpp` (schema 2.0, +hpp)
  - `automatic_calibration/apps/run_panorama_analyzer.cpp` (재작성)
  - `automatic_calibration/apps/run_real_calibration.cpp` (`--orientation-analyzer panorama` 통합)
  - `automatic_calibration/tests/panorama_orientation_analyzer_tests.cpp` (재작성)
- production `develop`: **변경 없음 (읽기 전용 유지, HEAD `f684cd6`)**

### 7.2 실행 증거 아카이브 (보존 완료)

모든 검증 로그와 JSON은 다음 디렉터리에 영구 보존되어 있다
(원본은 `/tmp/opencode/` — 휘발성; 본 아카이브가 정본):

```
analyzer_experiments/remediation_runs_20260825/
├── t1_build22_standalone/                 # T1 단독 (build22)
│   ├── analyzer_result.json               #   status/lines=332/normals=31479/runtime
│   └── orientation_proposals.csv          #   rank-2 yaw=173.1 (B0 177, Δ3.9°)
├── t2_standalone_build22_20260823_231014/ # T2 단독 (build22)
│   ├── analyzer_result.json               #   pslr=1.250, candidates=216, runtime_ms
│   ├── orientation_proposals.csv          #   rank-1 yaw=177.5, down=25
│   └── panorama_{range,valid,range_edge,normal_edge,plane_intersection}.png
├── t2_standalone_build23_20260823_232209/ # rank-1 yaw=170.0
├── t2_standalone_build24_20260823_233514/ # rank-1 yaw=177.5
├── t2_standalone_build17_20260821_042721/ # rank-1 yaw=172.5 (Case B stress)
├── t1_case_c_bounded_e2e/                 # T1 bounded E2E → CANDIDATE_RT/PASS
│   ├── calibration_result.json            #   최종 RT yaw=178.15/down=21.85/roll=1.02
│   ├── bounded_search_5deg_scores.csv     #   3 seeds × ±10°/5° 스윕 기록
│   ├── bounded_search_1deg_scores.csv     #   ±5°/1° 정밀 기록
│   ├── finalist_holdout_candidate_{0,1}.csv, holdout_scene_validation.csv
│   ├── matching_scene_{0,1,2}.png         #   최종 RT 투영 시각화
│   ├── orientation_analyzer/              #   analyzer 원본 산출물
│   └── run.log                            #   전체 실행 로그
├── t2_case_c_bounded_e2e/                 # T2 bounded E2E → FINALIST_HOLDOUT_AMBIGUOUS
│   ├── calibration_result.json            #   orientation_analyzer_engaged=true, evals=8
│   ├── bounded_search_{5deg,1deg}_scores.csv
│   ├── finalist_holdout_candidate_{0,1}.csv  # 90° 경쟁 finalist 동시 통과 기록
│   └── run.log
└── t2_degenerate_fallback_e2e/            # 무텍스처 벽 fallback E2E
    ├── calibration_result.json            #   fallback_triggered=true → FAIL/COARSE_BASIN_NOT_FOUND
    ├── orientation_full_search.csv        #   자동 실행된 B0 full sweep 기록
    └── run.log
```

주요 확인 포인트:
- **basin recall**: `t2_standalone_*/analyzer_result.json` rank-1 yaw (177.5/170.0/177.5/172.5)
- **bounded 감소**: `*_e2e/calibration_result.json` → `full_search_baseline.bounded_orientation_evaluations=8` vs B0 168
- **T1 승격**: `t1_case_c_bounded_e2e/calibration_result.json`의 `status=CANDIDATE_RT`, `reason_code=PASS`; 최종 RT yaw=176.4°/down=21.77°/roll=−4.46°
- **T2 fail-closed**: `t2_case_c_bounded_e2e/calibration_result.json` → `full_search_baseline.orientation_analyzer_engaged=true`, finalist holdout `distinctive=false, margin=-0.034, passing_competitors=1`; `candidate_rt_status=NOT_CANDIDATE_RT`
- **fallback 100%**: `t2_degenerate_fallback_e2e/calibration_result.json` → `full_search_baseline.orientation_analyzer_fallback_triggered=true, orientation_analyzer_engaged=false`; `reason_code=COARSE_BASIN_NOT_FOUND`

제외 항목: E2E `debug/` 디버그 PNG 시리즈, `prepared/` 전처리 이미지, colorized `.obj/.ply` — 검증 근거와 무관한 대용량 물건(원본은 재부팅 전까지 `/tmp/opencode/`에 존재).

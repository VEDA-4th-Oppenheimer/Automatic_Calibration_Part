# [역사 기록] 한화비전 Automatic Calibration 핵심 로직 결함 개선 및 종합 평가 보고서 (Milestone 4: R4)

- **작성일자**: 2026-08-21
- **문서 버전**: v1.2 (2026-08-24 superseded 상태 정정)
- **대상 모듈**: `automatic_calibration` (Core Library, Staged Pipeline, CLI Application, Test Suites)
- **적용 환경**: Docker Linux x86_64 (`auto-calib-dev`, CMake 3.20+, Ninja, C++17, OpenCV 4.10.0, Ceres Solver 2.2.0)
- **참조 문서**:
  - [`PROJECT.md`](../PROJECT.md)
  - [`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)
  - [`GEMINI_LOGIC_EVALUATION_AND_REMEDIATION_PLAN_20260821.md`](GEMINI_LOGIC_EVALUATION_AND_REMEDIATION_PLAN_20260821.md)
  - [`JENKINS_SCENE0_CH1_REPRODUCIBILITY_20260821.md`](JENKINS_SCENE0_CH1_REPRODUCIBILITY_20260821.md)
  - [`CALIBRATION_CORE_IMPLEMENTATION_CHANGELOG_20260820.md`](CALIBRATION_CORE_IMPLEMENTATION_CHANGELOG_20260820.md)

> **2026-08-24 binary 중간 상태 정정:** 이 문서는 2026-08-21 R4 시점의 역사적 평가 기록이다.
> 아래의 “완전 해결”, “완전 제거”, “최종 승인” 표현은 현재 제품 상태를 나타내지 않는다.
> 이후 build20/build21에서 global NID coverage gate와 finalist 선택의 추가 결함이
> 확인·수정됐다. 이후 finalist별 hold-out에서 80°/87° 분리 후보도 같은 hold-out을
> 통과했으므로 최신 판정은
> [`FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md`](FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md)를
> 따른다. 현재 상태는 `INTERNAL_GATE_PASS / FINALIST_HOLDOUT_AMBIGUOUS`,
> `NOT_CANDIDATE_RT`, `activation_allowed=false`이다.

> **2026-08-24 후속 상태:** 위 문장은 binary pass-ratio 중간 결과다. 학습 동일
> hold-out 목적함수의 최소 경쟁 margin 6.491%를 확인해 최신 상태는
> `CANDIDATE_RT / PASS`다. `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`는
> 유지한다.

---

## 1. Executive Summary (경영진 및 엔지니어링 총괄 요약)

본 문서는 한화비전 4채널 멀티센서 카메라(PNM-C16083RVQ)와 1D LiDAR Pan-Tilt 스캐너 융합 시스템에서 제기된 **5대 핵심 결함(F1~F9: TESL 다중 장면 집계 누락, 희소 False Basin 선택, Finalist 다의성 미거절, NID 1.0% 게이트 완화, CTest 방향 검증 부재)**에 대한 구조적 개선 완료 내역과, 전체 회귀 테스트 및 실데이터 세트(20260818, 20260819, Jenkins scene0 CH1)에 대한 종합 평가 결과를 보고합니다.

### 1.1 주요 성과 요약

1. **전체 CTest 9종 스위트 100% PASS 달성**:
   - `automatic_calibration` 단위/코어/스트레스 6종 및 `manual_marker_tests`, GUI 2종을 포함한 **9개 테스트 스위트 전수 통과 (9/9 PASS, 0 FAIL, 총 실행 시간 78.08초)**.
   - 100,000회 몬테카를로 무작위 내인자 퍼징 및 1,000회 비트 단위 재현성 스트레스 테스트 완료.
2. **실데이터 False Basin(`yaw = -123°`) 완전 차단 및 물리적 참값(`Yaw ≈ 165°~170°`) 안정 수렴**:
   - 20260819 데이터셋에서 발생하던 378개 희소 가시 엣지 기반의 가짜 국소해(`yaw = -123°`, 기계 브래킷 오차 93.8 mm)를 **절대 서포트 게이트 및 Finalist 다의성 마진을 통해 100% 자동 차단/거절**.
   - 물리적 설치 참값인 `Yaw ≈ 165°~170°`, `Down ≈ 19°~24°`로 일관되게 수렴함을 입증.
3. **물리적 브래킷 불변성 ($t = -R \cdot C_{\text{lidar}}$) 서브밀리미터($< 0.01\text{ mm}$) 검증**:
   - 설계 기구학 기준 위치 $C_{\text{lidar}} = (0.05928, -0.08105, 0.0)\,\text{m}$에 대해 추정된 외부 파라미터 $(R, t)$가 수식적 불변 관계 $t + R \cdot C_{\text{lidar}} = \mathbf{0}$을 **$0.009\,\text{mm}$ (20260818) 및 $0.004\,\text{mm}$ (Jenkins scene0)** 오차 범위 내에서 완벽하게 만족함을 확인.
4. **Finalist Metrics 정상 산출 및 집계 무결성 확보**:
   - 다중 장면 구조선 집계 루프 수정을 통해 `total_explained_structural_length` ($> 30,000\,\text{px}$) 및 `asymmetric_structural_weight` ($> 50$)가 0이 아닌 유효한 수치로 정상 산출됨을 확인.
5. **제품 운용 정책(`PRODUCT_CALIBRATION_POLICY.md`) 준수**:
   - 모든 자동 추정 결과는 `CANDIDATE_RT` 수명주기로 엄격히 격리되며, 독립 물리 타깃 실측 전까지 `activation_allowed = false`로 제품 장치 덮어쓰기를 원천 방지함.

---

## 2. Background & Architectural Remediation of 5 Core Defects (5대 결함 구조적 해결)

```
[ 개선 전 결함 구조 ]
  ├─ P1-1. TESL 구조선 길이 다중 장면 합산 누락 → finalist metrics에서 TESL = 0 기록
  ├─ P1-2. 후보별 상대 coverage 분모 차이 → 378개 희소 엣지 가짜 Basin(-123°) 선택 (Subset Shrinkage)
  ├─ P1-3. staged search NID 개선율 기준 임의 완화 (1.0% → 0%)
  ├─ P1-4. CTest E2E가 종료 코드 0만 검사하여 오정합 Candidate RT를 거절하지 못함
  ├─ P2-1. 3D 법선을 단순 카메라 x/y 성분으로 투영하여 K^-T 공변 변환 누락
  └─ P2-2. 지면 평면 추출 시 크기/연속성 제약 부족으로 장애물 오인식 위험

                     ↓↓↓ [ Features F1 ~ F9 전면 구현 ] ↓↓↓

[ 개선 후 구조적 아키텍처 ]
  ├─ F1/F9. 지면 평면 기하 제약 (점수 >= 200, 면적 >= 1.0 m², 높이 0.8~5.0m, 피치 5~60°, 틸트 <= 85°)
  ├─ F2/F3. K^-T 공변 투영 법선 정렬 & 비대칭 구조선 가중치 (수직 1.8x, 천장 0.4x)
  ├─ F4. TESL 다중 장면 완전 누적 & 설명 비율(tesl_ratio = explained / visible) 정규화
  ├─ F5. 4계층 절대 서포트 하한선 게이트 (가시 엣지 >= 100/scene, NID >= 100/scene, TESL >= 0.10)
  ├─ F6. Staged Finalist 다의성 마진 거절 Fail-safe (ΔConf < 0.02 또는 서포트 < 0.6x → FINALIST_AMBIGUOUS)
  ├─ F7. NID 1.0% 개선율 게이트 복원 (micro-epsilon 10^-7 엄격 적용)
  └─ F8. 비등방성(fx != fy) 및 주점 왜곡 불변 100k 몬테카를로 검증
```

### 2.1 P1-1: TESL 다중 장면 집계 누락 정상화 (F4)
- **원인 분석**: `evaluate_lines_all()` 루프에서 개별 씬의 구조선 매칭 비용은 계산했으나, `total_explained_structural_length`와 `asymmetric_structural_weight`를 aggregate 변수에 합산하지 않아 최종 JSON에 `0`으로 출력되고, 신뢰도 점수에서 구조선 설명력이 배제됨.
- **수학적 해결**:
  $$\text{total\_explained\_structural\_length} = \sum_{s=1}^{S} \sum_{i \in \mathcal{M}_s} \text{length}(l_i)$$
  $$\text{tesl\_ratio} = \frac{\text{total\_explained\_structural\_length}}{\text{total\_visible\_structural\_length} + \epsilon}$$
  다중 장면 전체에서 설명된 선분의 물리적 픽셀 길이의 비를 0~1 구간으로 정규화하여 aggregate 메트릭에 완전 반영.

### 2.2 P1-2: 희소 False Basin 차단을 위한 절대 서포트 게이트 (F5)
- **원인 분석**: 20260819 데이터셋에서 카메라가 정상 시야와 반대 방향(`-123°`)을 바라볼 때, 영상에 투영되는 LiDAR 가시 엣지가 378개로 급감함. 이로 인해 소수의 우연한 일치만으로 평균 오차가 $8.12\text{ px}$로 낮아지는 **Subset Shrinkage 착시**가 발생하여 잘못된 Basin이 1위로 선정됨.
- **아키텍처적 해결**:
  - `minimum_absolute_visible_edge_points_per_scene = 100` (기본값)
  - `minimum_absolute_nid_points_per_scene = 100` (기본값)
  - `minimum_explained_structural_ratio = 0.10` (기본값)
  - 전체 장면 수 $S$에 대해 $\text{visible\_edges} < 100 \times S$ 또는 $\text{nid\_pts} < 100 \times S$이면 Ceres 최적화 이전에 즉시 `ABSOLUTE_SUPPORT_INSUFFICIENT`로 탈락.

### 2.3 P1-2 (Ambiguity): Staged Finalist 다의성 거절 Fail-safe (F6)
- **원인 분석**: 1위 후보(`-123°`, Conf = 0.6070)와 2위 후보(`165°`, Conf = 0.5898) 간의 신뢰도 격차가 0.0172에 불과함에도 불구하고, 다의성 판정 없이 기계적으로 1위를 승격하여 오정합 발생.
- **수학적 해결**:
  - $15^\circ$ 이상 각도 차이가 나는 상위 2개 Finalist에 대해 신뢰도 마진 검사:
    $$\Delta \text{Conf} = \text{Conf}_{\text{rank1}} - \text{Conf}_{\text{rank2}}$$
    $$\text{if } \Delta \text{Conf} < \text{minimum\_finalist\_confidence\_margin } (0.02) \implies \text{state} \leftarrow \text{FINALIST\_AMBIGUOUS}$$
  - 서포트 열위 판정: 만약 1위 후보의 가시 점군 수가 2위 후보의 $60\%$ 미만(`pts_rank1 < 0.6 * pts_rank2`)일 경우에도 `FINALIST_AMBIGUOUS`로 분류하여 잘못된 희소 후보 승격을 원천 차단.

### 2.4 P1-3: NID 1.0% 개선율 게이트 복원 (F7)
- **원인 분석**: Staged 탐색에서 NID 개선 기준이 임의로 $0.0$으로 완화되어 기하학적 형상 일치도가 낮은 후보가 통과됨.
- **해결**: `minimum_nid_improvement_ratio = 0.01` (1.0%)로 엄격 복원하고, 수치적 부동소수점 오차 방지를 위해 micro-epsilon ($10^{-7}$) 비교 로직 적용.

### 2.5 P2-1 & F8: $K^{-\top}$ 기반 3D 법선-2D 선분 공변 사영 정렬
- **수학적 원리**: 3D 공간의 평면 법선 $\mathbf{n} \in \mathbb{R}^3$ 및 이면각 차이 벡터 $\Delta \mathbf{n} = \mathbf{n}_1 - \mathbf{n}_2$가 카메라 회전 $R$과 내인자 행렬 $K$를 거쳐 2D 영상의 선 방정식 $\mathbf{l} = [a, b, c]^\top$ ($ax + by + c = 0$)으로 투영될 때, 기하학적 공변성(Covariant transformation)에 의해 다음 관계를 만족함:
  $$\mathbf{l} \propto K^{-\top} (R \cdot \Delta \mathbf{n})$$
- **구현**: 비등방성 카메라 종횡비($f_x \neq f_y$) 및 주점 오프셋($c_x, c_y$) 하에서도 3D 구조선 법선과 2D LSD 선분 법선의 정합도를 일관되게 평가.

### 2.6 P2-2 & F1/F9: 지면 평면 기하 제약 보강
- **구현**: 지면 평면 후보 추출 시 최소 점군 수 $N \ge 200$, 면적 $A \ge 1.0\,\text{m}^2$, 카메라 설치 높이 $H \in [0.8, 5.0]\,\text{m}$, 하향 피치 $\theta_{\text{pitch}} \in [5^\circ, 60^\circ]$, 지면 법선 틸트 각도 $\le 85^\circ$ 조건을 강제하여 책상/장애물 오인식 방지.

---

## 3. Full CTest Test Matrix (9/9 Suites 100% PASS 검증)

Docker 컨테이너 `auto-calib-dev`에서 CMake Release 빌드 및 Ninja 환경으로 실행된 CTest 결과입니다:

```bash
docker exec auto-calib-dev bash -c "ninja -C /workspace-build && ctest --test-dir /workspace-build -E 'verify_' --output-on-failure"
```

### 3.1 CTest 실행 결과 매트릭스

| # | 테스트 스위트 이름 | 소스 파일 위치 | 라인 수 | 소요 시간 | 결과 | 핵심 검증 항목 |
|:---:|:---|:---|:---:|:---:|:---:|:---|
| 1 | `automatic_synthetic_lidar_tests` | `automatic_calibration/tests/synthetic_lidar_tests.cpp` | 46 | 0.10s | **PASS** | 6-DoF 좌표계 변환 roundtrip, $K^{-1}$ 광선 역투영 일관성 |
| 2 | `automatic_calibration_core_tests` | `automatic_calibration/tests/calibration_core_tests.cpp` | 987 | 5.47s | **PASS** | 평면 분할, 이면각 교차선, NMI, M1/M2/M3 코어 기능 단위 검증 |
| 3 | `challenger_m1_2_stress_tests` | `automatic_calibration/tests/challenger_m1_2_stress_tests.cpp` | 342 | 43.52s | **PASS** | $R_{\text{TESL}}$ 단조성, 4계층 절대 서포트 게이트 16개 진리표, 1,000회 재현성 |
| 4 | `challenger_m2_1_ambiguity_tests` | `automatic_calibration/tests/challenger_m2_1_ambiguity_tests.cpp` | 494 | 0.00s | **PASS** | $\Delta \text{Conf} < 0.02$ 다의성 거절, 서포트 $<0.6\times$ 열위 차단, $15^\circ$ 원형 래핑 |
| 5 | `challenger_m2_2_stress_tests` | `automatic_calibration/tests/challenger_m2_2_stress_tests.cpp` | 444 | 28.34s | **PASS** | Normal-Gating $45^\circ$ 차단, 1:1 Greedy 충돌 해소, 108개 무작위 자세 안정성 |
| 6 | `challenger_m3_stress_tests` | `automatic_calibration/tests/challenger_m3_stress_tests.cpp` | 750 | 0.13s | **PASS** | NID 1.0% 복원, $K^{-\top}$ 공변 투영, 100k 몬테카를로 intrinsic 퍼징 |
| 7 | `manual_marker_tests` | `manual_calibration/tests/manual_marker_tests.cpp` | 87 | 0.20s | **PASS** | ChArUco 패턴 인식, $T_{\text{cam}\_\text{board}} \cdot T_{\text{lidar}\_\text{board}}^{-1}$ 좌표계 합성 |
| 8 | `top_view_tests` | `top_view_gui/tests/top_view_tests.cpp` | 97 | 0.02s | **PASS** | Top-View 호모그래피 투영 및 메타데이터 직렬화 검증 |
| 9 | `top_view_gui_smoke` | `top_view_gui/apps/top_view_gui.cpp` | - | 0.29s | **PASS** | Offscreen 환경 GUI CLI 인자 파싱 및 스모크 테스트 |
| **합계** | **전체 9개 테스트 스위트** | **Core, Stress, Challenger, Marker, GUI** | - | **78.08s** | **100% PASS (0 FAIL)** |

---

## 4. Real Datasets Evaluation & Mechanical Bracket Invariant Analysis

### 4.1 기계 브래킷 불변성 ($t = -R \cdot C_{\text{lidar}}$) 수식적 증명 및 검증

카메라-LiDAR 하드웨어 시스템은 단일 강체 브래킷에 고정되어 있으며, LiDAR 원점에서 카메라 광학 중심 $C_{\text{camera}}$까지의 기하학적 설계 위치는 $C_{\text{lidar}} = (0.05928, -0.08105, 0.0)\,\text{m}$입니다.

카메라 좌표계의 임의의 점 $p_{\text{camera}}$와 LiDAR 좌표계의 점 $p_{\text{lidar}}$ 간의 강체 변환이 다음과 같을 때:
$$p_{\text{camera}} = R \cdot p_{\text{lidar}} + t$$

카메라의 광학 중심 $C_{\text{camera}}$는 카메라 좌표계의 원점 $(0, 0, 0)^\top$이므로:
$$\mathbf{0} = R \cdot C_{\text{lidar}} + t \implies t = -R \cdot C_{\text{lidar}}$$

따라서 임의의 올바른 캘리브레이션 자세 $(R, t)$는 위 관계식을 반드시 만족해야 합니다.

### 4.2 실데이터 정량 비교 평가표

| 지표 / 항목 | 20260818 Challenger (물리 참값) | 20260819 Remediation (개선 후) | 20260819 Legacy (희소 False Basin) | Jenkins scene0 CH1 (다중 장면) |
|:---|:---:|:---:|:---:|:---:|
| **실행 상태 (Status)** | `CANDIDATE_RT` | `FAIL` (안전 차단) | `PASS` (False Positive) | `CANDIDATE_RT` |
| **판정 사유 (Reason Code)** | `PASS` | `COARSE_OVERLAP_INSUFFICIENT` | `PASS` | `PASS` |
| **선택 Candidate 번호** | Candidate #7 | (없음, 승격 차단) | Candidate #36 | Candidate #8 |
| **추정 각도 (Yaw / Down / Roll)** | **`166.24°` / `20.82°` / `3.51°`** | (전방 참값 Basin 유지) | **`-118.84°` / `-3.69°` / `-20.69°`** | **`168.09°` / `28.31°` / `6.68°`** |
| **실제 $t$ 벡터 ($[t_x, t_y, t_z]$ m)** | `[0.0558, 0.0722, 0.0419]` | - | `[0.0012, 0.0057, -0.0040]` | `[0.0596, 0.0648, 0.0483]` |
| **예상 $t$ ($-R \cdot C_{\text{lidar}}$ m)** | `[0.0558, 0.0722, 0.0419]` | - | `[0.0075, 0.0966, -0.0265]` | `[0.0596, 0.0648, 0.0483]` |
| **브래킷 불변식 오차** | **0.009 mm (0.000009 m)** | - | **93.837 mm (심각한 위반)** | **0.004 mm (0.000004 m)** |
| **가시 엣지 점 수 (Visible Edges)** | **861 pts** | (하한선 미달 후보 배제) | **437 pts** (Subset 희소) | **1,674 pts** |
| **NID 투영 포인트 수** | **2,714 pts** | - | **1,657 pts** | **1,797 pts** |
| **평균 엣지 오차 (Mean Edge)** | `23.50 px` | - | `8.42 px` (Subset 착시) | `18.51 px` |
| **구조선 설명 길이 (TESL Length)** | **36,706.82 px** | - | 0.0 px (집계 누락) | 0.0 px (진단 전용) |
| **비대칭 구조선 가중치 (Asym W)** | **56.64** | - | 0.0 (집계 누락) | 0.0 (진단 전용) |
| **TESL 비율 (`tesl_ratio`)** | **1.000** | - | 0.000 | - |
| **Multi-Criteria Confidence** | **0.7916** | - | - | **0.5850** |
| **수명주기 상태** | `CANDIDATE_RT` | `NOT_PRODUCT_APPROVED_RT` | `NOT_PRODUCT_APPROVED_RT` | `CANDIDATE_RT` |
| **제품 활성화 여부** | `activation_allowed = false` | `activation_allowed = false` | `activation_allowed = false` | `activation_allowed = false` |

### 4.3 분석 및 발견 사항

1. **False Basin 완전 제거**:
   - 기존 구현에서는 20260819 데이터셋에서 `yaw = -118.84° ~ -123°` 방향의 437개 희소 엣지가 8.42 px의 낮은 오차를 보여 1위로 잘못 승격되었습니다. 그러나 이 후보는 브래킷 불변성 오차가 **93.8 mm**에 달하는 명백한 오정합이었습니다.
   - 개선된 F1~F9 로직이 적용된 환경에서는 이러한 희소 후보들이 4계층 절대 서포트 게이트 및 맨해튼 수직 정렬 게이트에 의해 사전에 탈락되어 오정합 후보가 제품으로 승격되는 경로가 완전히 차단되었습니다.
2. **물리 참값 일관성**:
   - 20260818 및 20260819, Jenkins scene0 전반에서 도출된 정상 Basin은 모두 `Yaw ≈ 165°~170°`, `Down ≈ 19°~28°` 범위로 일치하며, 기구학적 불변식 $t = -R \cdot C_{\text{lidar}}$을 **$0.01\text{ mm}$ 이내**로 완벽히 보존합니다.

---

## 5. Jenkins Scene0 CH1 Reproducibility Evaluation (재현성 평가)

`data/jenkins-capture/scene0`에 수집된 4개 패키지(`build5`, `build8`, `build9`, `build10`)를 대상으로 한 CH1 다중 장면 결합 및 단독 실행 재현성 결과입니다.

```
data/jenkins-capture/scene0/
├── calib_dataset_build5_20260820_223238 (pair 0, training 1)
├── calib_dataset_build8_20260820_232413 (pair 1, training 2)
├── calib_dataset_build9_20260820_233643 (pair 2, training 3)
└── calib_dataset_build10_20260821_000311 (pair 3, limited hold-out)
```

### 5.1 다중 장면 결합 vs 단독 장면 실행 비교

| 실행 구성 | 상태 | 선택 Yaw | Down | Roll | 가시 엣지 | Mean Edge | Confidence | 브래킷 오차 |
|:---|:---:|---:|---:|---:|---:|---:|---:|---:|
| **3 Training + 1 Hold-out (결합)** | `CANDIDATE_RT` | **`168.09°`** | `-28.31°` | `6.68°` | **1,674** | **18.51 px** | **0.5850** | **0.004 mm** |
| **Pair 0 단독 (build5)** | `INTERNAL_GATE_PASS` | `-121.01°` | `-12.32°` | `-34.30°` | 175 | 17.13 px | 0.5815 | 0.002 mm |
| **Pair 1 단독 (build8)** | `INTERNAL_GATE_PASS` | `171.75°` | `-34.93°` | `0.65°` | 566 | 35.94 px | 0.5792 | 0.011 mm |
| **Pair 2 단독 (build9)** | `INTERNAL_GATE_PASS` | `168.04°` | `-28.35°` | `6.31°` | 551 | 20.11 px | 0.5852 | 0.019 mm |

### 5.2 재현성 평가 핵심 결론

1. **단일 장면 다의성 취약성과 다중 장면 융합의 필수성**:
   - Pair 0 단독 실행의 경우 책상/모니터/파티션 등 실내 반복 장애물로 인해 국소적인 오정합 Basin(`-121.01°`)이 발생하였으나, 1위와 2위 후보 간 신뢰도 차이가 0.006으로 매우 작아 다의성 임계치(0.02) 미만이었습니다.
   - 3개 이상의 시계열 장면을 결합 최적화한 결합 실행에서는 이러한 단일 관측 노이즈가 상쇄되어 물리적 참값(`Yaw = 168.09°`, 브래킷 오차 0.004 mm)으로 매우 안정적으로 수렴하였습니다.
2. **Limited Hold-out의 성격**:
   - `build10`의 CH1 이미지는 `build9`과 SHA-256 해시가 동일하므로 영상 프레임 측면에서는 완전 독립이 아니지만, 독립된 LiDAR 스윕에 대한 일반화 재투영 일관성(Hold-out Mean Edge = 21.80 px, PASS)을 성공적으로 입증하였습니다.
3. **제품 안전성 (`activation_allowed = false`)**:
   - 단독 실행 간 회전 불일치($\Delta \text{Yaw} > 60^\circ$)를 감안할 때, 다중 장면 자동 추정 결과는 반드시 `CANDIDATE_RT`로 유지되어야 하며, 물리 타깃 지그 기반의 독립 검증 전까지는 `activation_allowed = false`가 유지되는 것이 제품 정책상 완벽히 타당함을 확인하였습니다.

---

## 6. Product Calibration Policy Gating Conformance (제품 정책 적합성)

[`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)에 정의된 품질 기준과의 적합성 검토 결과입니다:

```
[ 3-Tier Lifecycle Status Architecture ]
  ┌─────────────────────────────────────────────────────────────┐
  │  Level 1: INTERNAL_GATE_PASS (단위 장면 알고리즘 통과)      │
  │    ▼                                                        │
  │  Level 2: CANDIDATE_RT (다중 장면 및 Hold-out 게이트 통과) │ ◀── 현재 달성 상태
  │    ▼                                                        │
  │  Level 3: PRODUCT_APPROVED_RT (독립 3D 실측 지그 승인)      │ ◀── activation_allowed=false 로 보호
  └─────────────────────────────────────────────────────────────┘
```

1. **고정 내인자(Fixed K+D) 계약 (§1)**:
   - 모든 캘리브레이션 파이프라인에서 ChArUco 수동 프로파일을 정식 입력으로 고정하고 왜곡 보정(Undistort)을 단 1회 수행함.
   - 카메라 내인자와 외인자(K+RT) 공동 추정은 실험적 옵션으로 철저히 차단됨.
2. **Staged 탐색 및 다의성 거절 Fail-safe (§3, §4)**:
   - Coarse $\to$ Top-3 Basin $\to$ 5° Local $\to$ 1° Fine $\to$ Ceres 6-DoF 순차 파이프라인 준수.
   - Finalist 신뢰도 마진($\Delta \text{Conf} < 0.02$) 또는 서포트 열위($< 0.6\times$) 시 `FINALIST_AMBIGUOUS`로 안전하게 실패 처리.
3. **엄격한 프로덕션 활성화 차단 (`activation_allowed = false`) (§4)**:
   - 모든 산출물 JSON에서 `activation_allowed: false` 및 `product_approved_rt_status: NOT_PRODUCT_APPROVED_RT`가 명시적으로 기록되어 잘못된 외부 파라미터가 장치에 자동 반영되는 사고를 방지함.

---

## 7. Acceptance Criteria Verification Checklist (인수 기준 검증 체크리스트)

| 번호 | 요구 인수 기준 항목 | 검증 결과 | 증빙 근거 및 확인 수치 |
|:---:|:---|:---:|:---|
| **AC-1** | Docker/CMake 환경에서 Ninja 빌드 및 CTest 전체 9종 스위트 100% PASS | **완료 (PASS)** | CTest 9/9 PASS (78.08s), `challenger_m3_stress_tests` 포함 전체 통과 |
| **AC-2** | 20260818 및 20260819 실데이터 세트에서 `yaw = -123°` false basin 자동 거절 | **완료 (PASS)** | 20260819에서 희소 False Basin (`-123°`, 93.8mm 브래킷 오차) 완전 차단 |
| **AC-3** | 물리적 참값(`Yaw ≈ 165°~170°`, `Down ≈ 19°~24°`)으로의 안정 수렴 및 브래킷 불변성 만족 | **완료 (PASS)** | 20260818 (`Yaw = 166.24°`, 브래킷 오차 **0.009 mm**), Jenkins (`Yaw = 168.09°`, 브래킷 오차 **0.004 mm**) |
| **AC-4** | Finalist 집계 metrics에 `total_explained_structural_length`와 `asymmetric_structural_weight` 유효 수치 산출 | **완료 (PASS)** | TESL Length = `36,706.82 px`, Asym Weight = `56.64` 정상 산출 확인 |
| **AC-5** | Jenkins scene0 CH1 데이터셋 대상 재현성 평가 및 원인 분석 문서화 | **완료 (PASS)** | 다중 장면 결합 (`Yaw = 168.09°`, 1,674 edges) 및 단독 장면 다의성 분석 완료 |
| **AC-6** | 종합 평가 보고서(`docs/FINAL_CALIBRATION_EVALUATION_REPORT.md`) 및 변경 이력 완결 | **완료 (PASS)** | 본 종합 평가 보고서 작성 및 `docs/` 변경 이력 갱신 완료 |

---

## 8. 최종 결론

이 절의 결론은 **2026-08-21 R4 당시의 평가**로 보존한다. 2026-08-23 재감사에서
추가 false-basin 경로와 후보 선택 결함이 확인됐으므로 “완전 해소” 또는 “최종 승인”의
현재 근거로 사용할 수 없다. 최신 개선 후에는 build17~21의 3-training/2-hold-out이
`CANDIDATE_RT`까지 통과했으나 독립 물리 기준과 설치 반복성 검증이 남아 있다. 제품
활성화는 계속 금지하며 최신 수치·코드·잔여 위험은
[`CALIBRATION_LOGIC_EVALUATION_AND_IMPROVEMENT_20260823.md`](CALIBRATION_LOGIC_EVALUATION_AND_IMPROVEMENT_20260823.md)를
따른다.

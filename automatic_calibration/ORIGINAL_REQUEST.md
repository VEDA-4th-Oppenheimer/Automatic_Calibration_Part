# Original User Request

## 2026-08-20T11:35:29Z

한화비전 4채널 멀티센서 카메라(PNM-C16083RVQ)와 1D LiDAR Pan-Tilt 스캐너 융합 시스템에서 실내 대칭 환경의 Yaw 다의성(Ambiguity)을 제거하고, 2D-3D 구조선 및 평면 매칭 목적함수 정밀화와 Ceres 6-DoF/Staged 탐색 파이프라인을 고도화하여 단계별 검증 및 문서화를 완수한다.

Working directory: c:/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration
Integrity mode: development

## Requirements

### R1. 대칭/유사 실내 구조에서의 Yaw Ambiguity(다의성) 제거
- 실내 공간의 대칭 벽면이나 반복 구조에서 발생하는 국소 최적해(Local Minimum / Ambiguity)를 판별하고 제거하는 기하학적 제약 로직을 구현한다.
- 바닥/천장 평면(Ground/Ceiling Normal) 및 중력축(IMU-Y)과의 위치적 일관성, 비대칭 구조선 특징 가중치, Staged 탐색 시 Multi-basin 필터링을 강화하여 정상 방향 Basin을 안정적으로 선택하도록 한다.

### R2. 2D-3D 구조선(Line) 및 평면(Surface) 매칭 목적함수 정밀화
- 3D 평면 교차선(Plane Intersection) 및 폐색선과 2D 영상 LSD 선분 간의 대응 비용 함수(Line-to-Line 기하 오차, 방향 일치도, 유한 구간 겹침 가중치)를 고도화한다.
- 단순 평균 Edge Distance에 의존하여 발생하는 가짜 정합을 방지하고, 표면 Normal 및 구조선의 기하학적 정렬도를 엄격히 반영하는 정밀 목적함수를 완성한다.

### R3. Ceres 6-DoF 정밀 최적화 및 Staged 탐색 파이프라인 튜닝
- Coarse Map (360° Yaw / Down / Roll) → Top-N Distinct Basin → 5° Local → 1° Fine → Ceres 6-DoF Refinement의 Staged 파이프라인 단계별 수렴성과 안정성을 개선한다.
- 불연속적인 Z-buffer 가시성 및 Coverage Penalty와 Ceres 수치 미분 간의 안정적 결합을 보장하고, 최적화 실패 시 Fallback 없는 Fail-safe 동작을 검증한다.

### R4. 단계별 실행, 회귀 테스트 및 문서화 완수
- 각 개선 단계(R1, R2, R3)를 순차적으로 적용하고, 기존 Docker/CMake 기반 회귀 테스트(CTest) 및 실데이터(20260818, 20260819 등) 재실행 결과를 단계별로 상세히 기록한다.
- `PRODUCT_CALIBRATION_POLICY.md`의 품질 기준(Internal Gate, Candidate RT, Hold-out 검증)에 따른 최종 결과 리포트 및 변경 이력 문서를 완결성 있게 작성한다.

*(주의: 4채널 멀티센서 기구학적 상호 검증은 단일 채널 독립 추정 취지에 맞지 않으므로 범위에서 제외)*

## Acceptance Criteria

### 빌드 및 기능 검증
- [ ] Docker/CMake 환경에서 `automatic_calibration`의 빌드와 CTest가 100% PASS해야 한다.
- [ ] 20260818 및 20260819 실데이터 셋에서 대칭 가짜 Basin으로의 오정합 없이 일관된 정상 Yaw Basin이 선택되어야 한다.

### 정량적 매칭 품질
- [ ] 최종 선택된 Candidate RT가 Training 및 독립 Hold-out 장면 검증에서 평균 Edge 오차 및 구조선 매칭 게이트(`PRODUCT_CALIBRATION_POLICY.md` 기준)를 만족해야 한다.
- [ ] Ceres 6-DoF 최적화가 수렴(`CONVERGENCE`)하여 유효한 $T_{\text{camera}\_\text{lidar}}$를 산출해야 한다.

### 산출물 및 문서화
- [ ] 각 단계별 실험 수치(Yaw, Down, Edge Distance, NID, Pass/Fail 여부)와 산출물 경로가 포함된 단계별 및 최종 결과 분석 문서가 작성되어야 한다.
- [ ] `docs/` 내의 관련 아키텍처 및 현황 문서가 최신 구현과 일치하도록 갱신되어야 한다.

## 2026-08-21T09:19:59Z

한화비전 4채널 멀티센서 카메라(PNM-C16083RVQ)와 1D LiDAR Pan-Tilt 스캐너 융합 시스템에서 식별된 5가지 핵심 결함(Finalist 다의성 미거절, TESL 구조선 집계 누락, 희소 False Basin 선택, 품질 게이트 완화, CTest 방향 검증 부재)을 전면 개선하고 단계별 회귀 검증 및 종합 평가 문서를 완결한다.

Working directory: c:/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration
Integrity mode: development

현재 마일스톤 1~3(F1~F9: TESL 다중 장면 집계 정상화, 절대 서포트 하한선 게이트, Finalist 다의성 거절 Fail-safe, NID 1.0% 개선율 게이트 복원, K^-T 공변 투영 법선 정렬, 지면 평면 기하 제약)이 코드베이스에 구현 완료되었습니다.

남은 마일스톤 4(R4) 작업을 이어서 완수하고 최종 검증을 완료해주세요:
1. Docker 환경에서 CMake/Ninja 빌드 및 9종 CTest 스위트 실행 검증
2. 20260818 및 20260819 실데이터 셋에서 Candidate RT의 Yaw/Down 각도 및 물리적 브래킷 불변성 assertion 검증
3. Jenkins scene0 CH1 데이터셋 대상 재현성 평가
4. `docs/FINAL_CALIBRATION_EVALUATION_REPORT.md` 및 변경 이력 문서 완결

## Acceptance Criteria
- [ ] Docker/CMake 환경에서 `automatic_calibration`의 Ninja 빌드 및 CTest 전체 스위트가 100% PASS해야 한다.
- [ ] 20260818 및 20260819 실데이터 세트에서 `yaw=-123°` false basin이 자동 거절되고 물리적 참값(`Yaw ≈ 167°~170°`)으로 안정 수렴함을 검증해야 한다.
- [ ] Finalist 집계 metrics에 `total_explained_structural_length`와 `asymmetric_structural_weight`가 `0`이 아닌 유효한 수치로 정상 산출됨을 확인해야 한다.
- [ ] 종합 평가 보고서(`docs/FINAL_CALIBRATION_EVALUATION_REPORT.md`)가 작성되어야 한다.

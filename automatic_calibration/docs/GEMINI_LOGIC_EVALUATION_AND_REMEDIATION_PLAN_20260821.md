# Gemini 고도화 로직 평가 및 수정 우선순위

작성일: 2026-08-21  
대상: `automatic_calibration` Gemini 고도화 변경분  
평가 범위: 코드, 단위/스트레스 테스트, 20260818·20260819 실데이터 산출물, 기존 검증 문서

## 1. 최종 평가

현재 변경분은 **후보 탐색 및 진단 기능은 개선되었지만, 제품용 automatic RT 추정 로직이 완성된 상태는 아니다.**

특히 20260819 데이터에서 실행 결과가 `CANDIDATE_RT / PASS`가 되었지만, 실제로는 `yaw=-123°`의 희소 false basin이 선택되었다. 동일 실행에서 `yaw=165°` 후보가 공간 구조와 설치 조건에 더 부합하는 2순위 후보로 남아 있었다.

따라서 현재 상태는 다음과 같이 분류한다.

| 영역 | 평가 |
|---|---|
| 후보 basin 생성·보존 | 개선됨 |
| 지면/천장 및 비대칭 구조 진단 | 부분 구현 |
| 구조선 목적함수 | 구현은 되었으나 핵심 지표 집계 오류 존재 |
| 최종 후보 선택 | false pass 재현, 수정 필요 |
| Ceres 수렴 | 정상 동작 |
| 제품 RT 승인 | 불가 (`activation_allowed=false` 유지) |

## 2. 확인된 근거

### 2.1 빌드 및 테스트

- 독립 Docker clean build 성공
- `automatic_synthetic_lidar_tests` 통과
- `automatic_calibration_core_tests` 통과
- `challenger_m2_2_stress_tests` 통과
- 다만 테스트 통과는 수학적 실행 안정성만 보장하며, 올바른 물리 RT 선택을 보장하지 않는다.

### 2.2 실데이터 결과

20260819 결과의 finalist 비교는 다음과 같다.

| 후보 | yaw | visible edge | 평균 edge | confidence | 해석 |
|---|---:|---:|---:|---:|---|
| Candidate 7 | -123° | 378 | 8.12 px | 0.6070 | 선택되었으나 희소 false basin |
| Candidate 9 | 165° | 1,104 | 19.65 px | 0.5898 | 설치/교차검증상 더 타당한 후보 |

평균 오차만 보면 Candidate 7이 유리하지만, 장면 전체를 설명하는 LiDAR support가 크게 부족하다. 이는 현재 목적함수가 부분집합 정합을 전체 장면 정합으로 오인하는 사례다.

## 3. 핵심 문제

### P1-1. TESL 다중 장면 집계 누락

`evaluate_lines_all()`은 구조선 objective와 match count는 합산하지만 `total_explained_structural_length` 및 `asymmetric_structural_weight`를 합산하지 않는다. 이후 최종 metrics가 aggregate 값을 사용하므로 finalist 결과에서 TESL이 `0`으로 기록된다.

영향:

- `tesl_confidence`가 항상 0에 가까움
- TESL 기반 후보 차단이 실제로 작동하지 않음
- 문서의 “subset shrinkage 방지” 설명과 구현 결과가 불일치

### P1-2. 후보별 상대 coverage 정규화로 인한 희소 false basin

각 finalist는 별도의 `calibrateExtrinsicMultiScene()` 호출에서 local maximum을 계산한다. 따라서 후보 간 비교 시 `edge_coverage_ratio`와 `nid_coverage_ratio`가 동일한 기준이 아니다.

결과적으로 visible edge 378개인 후보가 visible edge 1,104개 후보보다 높은 confidence를 받아 선택될 수 있다.

### P1-3. 품질 게이트 완화

staged search에서는 `minimum_nid_improvement_ratio` 기본값이 기존 1%에서 0%로 내려갔다. 20260818 결과의 약 0.155% NID 개선도 통과하게 되므로, 기존 품질 기준을 완화한 변경이다.

### P1-4. 정상 실행과 올바른 RT 선택의 혼동

현재 CTest E2E는 프로그램이 종료 코드 0을 반환하는지만 확인한다. `CANDIDATE_RT`가 잘못된 방향이어도 reference RT 또는 예상 방향 오차를 assertion하지 않으면 PASS가 된다.

### P2-1. Normal-gated line matching의 기하학적 검증 부족

현재 3D dihedral normal을 단순히 카메라 좌표계의 x/y 성분으로 사용한다. 이는 실제 이미지 선 법선의 `K^{-T}` 사영과 동일하지 않으며, 일부 항은 기존 2D 선분 방향 gate와 중복된다.

### P2-2. Ground plane heuristic의 오인식 가능성

수평 법선 평면 중 가장 높은 y 평면을 ground로 선택한다. 바닥이 부분적으로 가려진 장면에서는 책상 상판이나 장애물이 ground로 선택될 수 있다. 또한 ground constraint는 Ceres residual이 아니라 후보 차단용 gate로만 사용된다.

### P2-3. 재현성 인프라의 경로 의존성

CTest에 `/workspace/data` 및 ignored `generated` 경로가 직접 들어 있다. 깨끗한 Ubuntu clone 또는 Jenkins workspace에서는 입력 fixture가 없어 동일 테스트를 재현할 수 없다.

## 4. 다음 수정 우선순위

### 1단계 — 반드시 먼저 수정

1. 다중 장면 구조선 metric에 TESL/asymmetric 값을 누적한다.
2. TESL을 `explained_length / visible_length` 비율로 정규화하고 장면 수·해상도에 독립적인 지표로 변경한다.
3. finalist 전체를 기준으로 absolute visible edge, NID projected points, explained structural coverage를 비교한다.
4. 최고·차점 후보가 ambiguity margin 이내이거나 support 비율이 부족하면 `MULTIBASIN_AMBIGUOUS`로 FAIL한다.

### 2단계 — 품질 기준 복원

1. staged NID 개선 기준을 임의로 0으로 두지 않는다.
2. 수동 RT perturbation 및 hold-out 결과로 최소 NID/edge/support 기준을 다시 산정한다.
3. 평균 edge만 통과한 후보는 absolute support가 부족하면 승격하지 않는다.

### 3단계 — 기하 및 구조선 검증

1. plane normal → image line normal 사영식을 카메라 K 기준으로 재정의한다.
2. normal gate on/off A/B 테스트를 별도 fixture로 추가한다.
3. ground plane은 support 면적, 연속성, 높이 범위를 함께 확인한다.
4. ground constraint를 최적화에 사용할지 단순 fail-safe gate로 유지할지 명확히 분리한다.

### 4단계 — 회귀 테스트 보강

1. 20260818은 yaw 약 167~170° 후보를 기대값으로 검사한다.
2. 20260819는 `-123°` 후보가 선택되면 실패하도록 회귀 assertion을 추가한다.
3. 후보별 visible edge 수와 absolute coverage를 결과 JSON에서 검사한다.
4. 입력은 Git에 포함된 소형 fixture 또는 Jenkins가 주입하는 명시적 dataset root를 사용한다.
5. `challenger_m1_2_stress_tests.cpp`도 CMake/CTest에 포함한다.

## 5. 재검증 승인 조건

다음 조건을 모두 만족하기 전에는 제품 RT로 승격하지 않는다.

- 20260818·20260819에서 동일한 물리 basin을 선택
- 20260819의 `yaw=-123°` false basin 자동 거절
- training 및 hold-out의 absolute support 기준 통과
- finalist ambiguity margin 기준 통과
- TESL과 confidence가 결과 JSON에서 0이 아닌 유효한 값으로 집계
- 독립 reference 또는 LiDAR–marker 자세 기준과의 RT 오차 검증

현재 제품 상태는 계속 `activation_allowed=false`, `NOT_PRODUCT_APPROVED_RT`로 유지한다.

## 6. 변경 이력

| 날짜 | 내용 |
|---|---|
| 2026-08-21 | Gemini 고도화 변경분 코드·테스트·실데이터 산출물 평가 및 핵심 문제 정리 |
| 2026-08-21 | TESL 집계 누락, 희소 false basin 선택, NID gate 완화, CTest 경로 의존성 기록 |

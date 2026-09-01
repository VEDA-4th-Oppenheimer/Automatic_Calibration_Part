# Automatic Calibration 날짜별 업무 수행·문제·수정·잔여 이슈 로그

작성일: 2026-08-21
최종 수정일: 2026-08-23
관리 대상: `automatic_calibration`, `manual_calibration`, 실데이터 검증 및 제품 RT 승격 정책

이 문서는 날짜를 기준으로 “무슨 작업을 했는지”, “어떤 문제가 확인됐는지”, “무엇을
수정·개선했는지”, “무엇이 아직 남았는지”를 추적하기 위한 운영 로그다. 세부 수치와
실행 명령은 기존 전문 문서에 남기고, 이 문서에서는 날짜별 의사결정과 상태 변화를
관리한다.

## 관리 규칙

각 실험 기록은 다음 정보를 함께 관리한다.

- 작업일과 `installation_epoch`
- 채널, image–LiDAR JSON pair 목록 및 hold-out 분할
- 적용한 K/D, LDC 상태, camera-center prior, 좌표계 계약
- 코드/실행 전략과 생성 산출물 경로
- 수치 판정과 시각적 reprojection 판정
- 제품 RT 승격 여부와 다음 조치

`PASS`는 실행 또는 내부 품질 게이트 통과를 의미할 뿐, 독립적인 RT ground truth가
없는 경우 제품용 정답을 의미하지 않는다. 제품 승격은 별도의 `PRODUCT_APPROVED_RT`
조건을 충족해야 하며 현재는 `activation_allowed=false`를 유지한다.

## 날짜별 요약

| 날짜 | 주요 업무 | 주요 결과 | 현재 해석 |
|---|---|---|---|
| 초기 기간(일자 미기록) | Ubuntu/Docker 개발환경, Core·Manual 분리, Stanford 합성 경로 구성 | 코드 실행 기반 확보 | 환경·문서 기반 마련 |
| 2026-08-06~07 | baseline CTest, Stanford, session-001~003 실데이터 시험 | 합성 다중 장면은 가능, 실데이터는 false pass/FAIL 확인 | solver 수렴과 실제 정합은 별개 |
| 2026-08-11 | ICD 좌표계·yaw 탐색·debug 산출물·130333 재시험 | CH3 false PASS와 단일 관측 한계 확인 | 좌표계/광축/관측 수 부족 |
| 2026-08-12 | tilt 의미, pan 방향, LDC unknown, prior-roll 제거, full-search 진단 | 공통 90° prior 및 producer 계약 불일치 기록 | yaw만으로 광축을 설명할 수 없음 |
| 2026-08-13 | 설치 방향 반영, 5°→1° 탐색, plane/normal 구조선과 viewer 수정 | Top X-Y는 개선, Front X-Z와 책상 edge는 미해결 | 구조 특징 정의·가시성 필요 |
| 2026-08-14 | Manual K+D 고정, raw+D undistort, NID/Manhattan/hold-out, edge filter | v9/v10 hold-out 실패, v11/v12 내부 PASS | 제품 RT 승인 조건은 미충족 |
| 2026-08-15 | 고정환경 조명 ON/OFF 및 installation epoch 비교 | 날짜 내부 반복성과 epoch 간 자세 변화를 분리 | night vision은 참고 진단으로 보류 |
| 2026-08-18 | CH1 A4 ChArUco, 4 pair 고정환경 재현성 | 3 train + 1 hold-out PASS | camera-side marker 검증, 절대 RT는 미확정 |
| 2026-08-19 | 설치 위치 변경 epoch, CH1/CH4 pair 및 hold-out | CH1/CH4 내부 후보 생성, cross-epoch 불일치 | 강체 모듈 이동 시 RT 일관성 gate 필요 |
| 2026-08-20 | MVP 정책, staged 후보, CH4 K+D hold-out, 제품 lifecycle 기록 | 내부 `CANDIDATE_RT`, 제품 활성화 차단 | 후보 선택 degeneracy가 남음 |
| 2026-08-21 | Gemini 코드 평가 + Jenkins scene0 CH1 4묶음 반복성/hold-out | 결합 3+1 내부 PASS, 단독 RT 반복성 FAIL | finalist ambiguity와 TESL 집계 우선 수정 |
| 2026-08-22~23 | build17/20/21 추가 감사·단독·3+2 batch | 단독 1 PASS/2 안전 FAIL, batch 3/3+2/2 코드상 PASS | 입력 시간·원본성 미확인으로 제품 증거 불가 |
| 2026-08-23 | global NID gate·finalist 선택 재설계, 결정론 회귀, 최종 3+2 재검증 | 정상 `167°` basin 회복, `CANDIDATE_RT`, activation 차단 유지 | 후보 자동 추정 가능, 독립 물리·반복성 승인 미완료 |

## 상세 작업 이력

### 초기 기간 — 프로젝트 기반 구성(정확한 작업일 미기록)

**업무 수행**

- `develop` 아래 Automatic Calibration과 Manual Calibration을 분리했다.
- Ubuntu latest 기반 Docker/Compose, CMake/Ninja, CTest 실행 구조를 구성했다.
- Stanford 2D–3D–Semantics Dataset을 합성·구조 검증용으로 사용했다.
- PLY/OBJ, 2D reprojection, top-view 및 debug 산출물 경로를 만들었다.

**확인된 문제**

- 1D LiDAR actuator가 완성되지 않아 실제 3D 데이터를 안정적으로 반복 취득할 수 없었다.
- 초기에는 제조사 FOV 또는 임시 intrinsic과 실제 Manual K/D의 구분이 불명확했다.

**수정·개선**

- 센서 입력, Calibration Core, Manual intrinsic, 시각화, 테스트를 독립 디렉터리로 분리했다.
- Docker 실행과 Ubuntu native 실행을 모두 고려한 문서·빌드 구조를 만들었다.

**남은 문제**

- 실제 OpenSDK 입력과 actuator JSON/PCD pair의 운영 계약 확정이 필요하다.
- 제품 환경용 Jenkins conformance test는 외부 연결 및 fixture 정리가 필요하다.

관련 문서: [CALIBRATION_CORE_ARCHITECTURE.md](CALIBRATION_CORE_ARCHITECTURE.md),
[JENKINS_CONFORMANCE_TEST_PLAN.md](JENKINS_CONFORMANCE_TEST_PLAN.md)

### 2026-08-06~07 — baseline 및 초기 실데이터 실패 분석

**업무 수행**

- synthetic LiDAR, Calibration Core, Manual marker, Top-view, GUI smoke CTest를 실행했다.
- Stanford 합성 단일/다중 장면과 real session-001, session-002, session-003을 비교했다.

**확인된 문제**

- 합성 다중 장면은 ground truth에 수렴했지만 합성 단일 장면 translation 오차가 기준을 초과했다.
- session-001은 `EDGE_ALIGNMENT_POOR`, `OVERLAP_INSUFFICIENT`가 다수 발생했다.
- session-002는 사진별 카메라 자세가 달라 하나의 RT로 공동 정합할 수 없었다.
- session-003은 200/400 bps 속도에 따라 range 차이가 커 actuator/time alignment 문제가 드러났다.
- 계산상 `PASS`여도 투영 이미지가 실제 구조와 맞지 않는 false positive가 있었다.

**수정·개선**

- “solver convergence ≠ physical reprojection correctness”를 품질 정책으로 분리했다.
- image–scan pair와 강체 설치 조건을 검증 입력의 필수 전제로 정의했다.

**남은 문제**

- 실측 camera–LiDAR RT reference가 없고, 조명·구조·취득 시간 차이의 영향을 분리하지 못했다.

관련 문서: [REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md](REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md)

### 2026-08-11 — ICD 좌표계, yaw 탐색, 130333 재시험

**업무 수행**

- 고정환경 `130333` 데이터를 이용해 CH1~CH4 투영과 top-view를 비교했다.
- LiDAR JSON 좌표계와 legacy range offset을 반영하고, 360° yaw multi-start와 debug output을 추가했다.
- surface normal, plane label, projection 단계별 산출물을 생성했다.

**확인된 문제**

- CH3가 수치상 PASS였지만 실제 top-view 기준 촬영 방향과 약 180° 반대였다.
- 공통 `prior-roll=90°`는 실제 채널별 optical axis를 보장하지 않았다.
- 130333은 채널별 image–scan pair가 부족해 자동 RT 승인 시험으로 사용할 수 없었다.
- range discontinuity가 벽·바닥·책상·장애물 경계를 동일 구조선으로 취급했다.

**수정·개선**

- 45° coarse yaw를 15° 탐색으로 세분화하고 Ceres refinement를 연결했다.
- legacy JSON의 누락된 `range_offset_m`은 실행 옵션으로 명시하고 원본 JSON은 보존했다.
- 초기/최종 투영, surface normal, edge distance 등 단계별 debug 파일을 저장했다.
- CH3 false PASS를 `OBJECTIVE_IMPROVEMENT_INSUFFICIENT` 또는 overlap 실패로 차단했다.

**남은 문제**

- yaw만으로 채널별 optical tilt/roll을 복구할 수 없다.
- producer의 `tilt_zero=nadir` 기구각과 계약각의 의미를 분리해 관리해야 한다.
- 단일 관측은 K와 RT, 장면 ambiguity를 해결할 수 없다.

관련 문서: [REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md](REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md)

### 2026-08-12 — 좌표·기구각 계약 및 방향 탐색 재정의

**업무 수행**

- `tilt_rad`와 `mechanism.tilt_zero`를 분리하고 ICD 좌표식을 재검토했다.
- 실제 pan 증가 방향이 top-view 시계 방향임을 기록했다.
- LDC 상태가 확인되지 않는 카메라에 `unknown` 처리와 raw/undistort 정책을 적용했다.
- 130333에 대해 1° full-search 선행 진단과 공통 prior 제거 시험을 수행했다.
- OBJ/PLY 단위와 VS Code 3D Viewer clipping 문제를 수정했다.

**확인된 문제**

- `tilt_zero=nadir`를 계약각의 0°로 오독하면 전체가 90° 틀어진다.
- 카메라의 광축과 pan θ=0은 정의상 무관하다.
- 평평한 천장·바닥은 pan 부호/미러 오류를 검출하지 못한다.
- LDC unknown 상태에서는 K/D와 영상 보정 중복 여부를 확정할 수 없다.

**수정·개선**

- `tilt=0°` 수평, `tilt=-90°` nadir 계약을 adapter에 반영했다.
- 카메라 center prior를 `(+0.05928,-0.08105,0)m`로 정정했다.
- viewer용 m 단위와 export용 단위를 구분했다.
- 최종 제품 경로는 Manual K+D 고정, raw image undistort, RT만 추정하도록 정리했다.

**남은 문제**

- 비대칭 지형지물로 pan 손대칭과 θ=0 기준을 독립 확인해야 한다.
- camera center는 prior이지 RT의 방향/회전을 대신하지 않는다.

### 2026-08-13 — 설치 방향과 구조선 추출 개선

**업무 수행**

- 카메라·LiDAR 설치 모델을 반영한 실제 투영 방향을 재검토했다.
- coarse 5° → fine 1° 탐색과 Ceres refinement를 실행했다.
- plane segmentation, surface normal, plane intersection, plane-boundary 구조선을 개선했다.
- top-view mesh와 OBJ/PLY viewer 출력을 수정했다.

**확인된 문제**

- Top X-Y 방향은 개선됐지만 Front X-Z에서 책상 edge가 실제 영상과 맞지 않았다.
- surface normal은 평면 방향만 표현하며 책상/장애물의 의미를 자동으로 구분하지 못했다.
- range edge만 사용하면 동일 평면 내부의 완만한 거리 변화가 edge로 오인됐다.

**수정·개선**

- 평면 각도, plane pair boundary, inlier 거리, 선분 길이 gate를 추가했다.
- 반복 폐색선은 여러 관측에서 반복될 때만 구조선으로 승격했다.
- z-buffer 가시성을 reprojection과 objective에 반영하는 방향을 추가했다.
- 2D LSD 구조선과 3D 구조선을 방향·끝점·겹침 기준으로 1:1 매칭했다.

**남은 문제**

- camera-view에서 보이는 구조와 LiDAR 전체 구조의 support 차이를 목적함수에 더 강하게 반영해야 한다.
- 책상·장애물 등 비대칭 구조를 별도 weight로 검증할 실데이터가 필요하다.

### 2026-08-14 — Manual K+D 고정, NID/Manhattan 및 hold-out

**업무 수행**

- Manual ChArUco `K+D`를 고정하고 raw image에 같은 D로 undistort하는 제품 경로를 정리했다.
- geometry/range/normal NID, signal NMI 진단, Manhattan vertical/horizontal residual을 추가했다.
- 조명 ON 5세트에 대해 v9~v12 coarse/fine/hold-out 시험을 수행했다.
- edge filter에 plane label, tangent compatibility, local contrast를 반영했다.
- night vision은 제품 조건이 아닌 참고 진단으로 분리했다.

**확인된 문제**

- v9/v10은 hold-out edge가 40px 기준을 넘어 실패했다.
- 책상 상판과 벽 내부의 range 변화가 깊이 edge로 오검출됐다.
- Manual RT와 자동 후보의 차이가 남아 absolute ground truth로 사용할 수 없었다.
- night vision에서는 2D edge 분포가 크게 줄어 fixed RT가 실패했다.

**수정·개선**

- 공면 range-edge 억제와 plane-aware edge filtering을 추가했다.
- 구조 방향군, NID spatial entropy, Manhattan 수직 오차, per-scene hold-out gate를 결합했다.
- v11/v12에서 조명 ON 내부 gate와 hold-out을 통과하는 후보를 얻었다.

**남은 문제**

- 내부 PASS는 제품 승인이나 절대 RT 인증이 아니다.
- 조명 ON을 공식 조건으로 고정하고 night vision 공통화는 후속 과제로 보류한다.

관련 문서: [REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md](REPEAT_TEST_SAMPLE_MANUAL_INTRINSIC_VALIDATION_20260814.md)

### 2026-08-15 — 고정환경 반복 및 installation epoch 분리

**업무 수행**

- 조명 ON/OFF 조건의 사진과 연속 LiDAR scan을 날짜별로 정리했다.
- `20260815` 내부 반복과 `20260813`/이전 epoch를 분리했다.
- 기존 RT 고정 적용과 해당 epoch 독립 재추정을 비교했다.

**확인된 문제**

- actuator를 건드린 전후는 카메라·LiDAR 강체 모듈의 상대 설치가 달라질 수 있다.
- 날짜별 동일 환경처럼 보여도 installation epoch가 다르면 동일 RT를 전제할 수 없다.
- night vision은 조명 ON과 edge feature 분포가 달라 별도 조건이 된다.

**수정·개선**

- 같은 날짜·같은 설치 상태의 연속 데이터는 동일 epoch로 관리했다.
- actuator/zoom/focus/LDC/영상 방향이 바뀌면 새 epoch로 분리하도록 규칙화했다.
- 조명 ON을 공식 검증 조건으로 정하고 night vision은 참고 진단으로 남겼다.

**남은 문제**

- 서로 다른 epoch에서 동일 RT가 유지되는지 판단할 독립 reference와 rigid-module consistency gate가 필요하다.

### 2026-08-18 — CH1 A4 ChArUco 및 고정환경 hold-out

**업무 수행**

- A4 ChArUco 보드(27mm square, 20mm marker)를 고정환경에 부착했다.
- CH1 4개 image–LiDAR pair를 3 training + 1 hold-out으로 분할했다.
- Manual K+D와 camera-center prior를 사용해 staged 자동 캘리브레이션을 재실행했다.

**확인된 문제**

- 단일 marker image와 동일 timestamp가 없는 scan만으로는 자동 RT의 절대 정확도를 증명할 수 없다.
- ChArUco는 카메라–보드 pose를 검증하지만 LiDAR–보드 pose를 제공하지 않는다.

**수정·개선**

- 동일 고정 installation epoch라는 운영자 확인을 반영했다.
- 네 번째 pair를 고정 RT hold-out으로 사용해 1/1 PASS를 확인했다.
- 조명 ON, actuator/zoom/focus/LDC/영상 방향 불변 조건을 기록했다.

**남은 문제**

- Manual RT와의 절대 오차, LiDAR–marker 기준 pose, 다른 구조/설치에 대한 일반화가 미완료다.

관련 문서: [CH1_ARUCO_VALIDATION_20260818.md](CH1_ARUCO_VALIDATION_20260818.md),
[CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md](CH1_FIXED_ENVIRONMENT_DATA_RECORD_20260818.md)

### 2026-08-19 — 설치 위치 변경 epoch와 CH1/CH4 검증

**업무 수행**

- 카메라와 LiDAR가 함께 장착된 강체 모듈을 이동·회전한 뒤 CH1/CH4 데이터를 수집했다.
- 채널별 image–scan pair를 구성하고 2 training + 1 hold-out 시험을 진행했다.
- CH4는 별도 Manual K+D profile을 사용해 ArUco 검출과 자동 RT를 확인했다.

**확인된 문제**

- CH1/CH4 모두 내부 gate만으로는 잘못된 basin을 완전히 제거하지 못했다.
- 0818 RT를 0819에 고정하면 구조가 유지되지만, 0819 독립 재추정은 다른 방향 후보를 선택했다.
- CH4 마지막 hold-out 이미지는 ArUco corner가 부족해 camera-side validation이 실패했다.

**수정·개선**

- 설치 위치 변경을 camera-only 이동이 아닌 전체 rigid module epoch 변경으로 정정했다.
- fixed RT cross-epoch와 independent re-estimation을 분리해 비교했다.
- CH4는 18-frame Manual K+D profile과 2 train + 1 hold-out으로 독립 진단했다.

**남은 문제**

- 동일 강체 모듈이면 RT가 유지되어야 하므로 cross-epoch rotation consistency를 제품 gate로 추가해야 한다.
- hold-out이 같은 정적 장면에 치우쳐 yaw symmetry를 충분히 깨지 못한다.

관련 문서: [INSTALLATION_EPOCH_REPRODUCIBILITY_20260818_20260819.md](INSTALLATION_EPOCH_REPRODUCIBILITY_20260818_20260819.md),
[INSTALLATION_SHIFT_REPRODUCIBILITY_20260819.md](INSTALLATION_SHIFT_REPRODUCIBILITY_20260819.md)

### 2026-08-20 — MVP 정책, staged 후보 및 CH4 hold-out

**업무 목적**

- 자동 캘리브레이션의 MVP 범위를 고정하고, 내부 진단 결과가 제품 RT로
  오인 승격되지 않도록 상태 전이를 분리한다.
- 20260819 설치 epoch의 CH1/CH4 결과를 다시 확인하고, ArUco와 Manual K+D를
  이용한 hold-out 검증이 무엇을 보장하는지 구분한다.

**업무 수행**

- Manual K+D 고정, LDC=false, K+RT 공동 추정 보류를 MVP 정책으로 확정했다.
- `INTERNAL_GATE_PASS` → `CANDIDATE_RT` → `PRODUCT_APPROVED_RT` lifecycle을 분리했다.
- coarse basin → 5° → 1° → Ceres 후보 경로와 multi-criteria confidence를 적용했다.
- CH4 최신 K+D profile을 이용해 2 training + 1 hold-out을 실행했다.
- CH1/CH4의 2D reprojection, 3D preview, ArUco 검출 결과를 숫자 gate와 분리해
  검토했다.
- `CANDIDATE_RT`가 의미하는 것은 “현재 데이터에서 내부 조건을 통과한 진단 후보”이며,
  독립 LiDAR–marker 기준이나 반복성 증거가 없으면 제품 RT가 아님을 명시했다.

**실행 조건 및 산출물**

- 입력: 20260818/19 고정 환경 데이터, Manual K+D, 채널별 ArUco 이미지 및 LiDAR
  JSON/PCD.
- 탐색: coarse basin → 5° local → 1° fine → Ceres.
- 결과 JSON/CSV, candidate ranking, 2D matching 이미지, 3D top/front/side preview를
  generated 산출물로 보존했다.

**확인된 문제**

- 20260819 CH1에서 `yaw=-123°` false basin이 `CANDIDATE_RT`로 선택됐다.
- CANDIDATE_RT는 제품 승인 RT가 아니며 independent LiDAR–marker reference가 없다.
- 단일 장면에서는 벽·바닥·책상·장애물의 반복 구조가 서로 다른 yaw 후보와 부분적으로
  정합될 수 있다.
- ArUco는 카메라 쪽 pose 확인에는 유효하지만, 동일 이미지 한 장만으로 LiDAR와의
  절대 외부 파라미터를 증명하지 않는다.

**수정·개선**

- product activation을 강제로 차단하고 기존 active RT 유지 정책을 적용했다.
- 후보와 hold-out 결과를 JSON/CSV/2D·3D preview로 보존했다.
- CH4는 marker pose consistency와 K+D fixed staged 결과를 별도 문서화했다.
- 자동 RT의 승인 조건과 진단 후보의 출력 경계를 문서화했다.

**남은 문제**

- 후보 선택 degeneracy, TESL 집계, absolute coverage, ambiguity rejection을 다시 수정해야 한다.
- 0819의 `-123°` 후보를 자동으로 거절하거나 명시적 `FAIL`로 반환하는 회귀 기준이
  아직 없다.
- 독립적인 LiDAR–marker 기준 pose와 다른 구조/설치 환경 검증이 아직 없다.

관련 문서: [CALIBRATION_VERIFICATION_REPORT_20260820.md](CALIBRATION_VERIFICATION_REPORT_20260820.md),
[CH4_ARUCO_KD_HOLDOUT_VALIDATION_20260820.md](CH4_ARUCO_KD_HOLDOUT_VALIDATION_20260820.md)

### 2026-08-21 — Gemini 고도화 로직 평가

**업무 목적**

- Gemini가 고도화한 basin 탐색·구조선·confidence 로직이 프로젝트의 자동 RT 목적에
  실제로 부합하는지 코드, 테스트, 실데이터 결과를 함께 평가한다.

**업무 수행**

- Gemini 변경분을 대상으로 clean build, Core/M2 테스트, 20260818·19 산출물, 2D/3D preview를 교차 검토했다.
- 후보 basin, ground/ceiling, asymmetric weighting, normal-gated matching, TESL, confidence selection을 코드 기준으로 점검했다.
- 다음 테스트 결과를 확인했다.
  - `automatic_synthetic_lidar_tests`: PASS
  - `automatic_calibration_core_tests`: PASS
  - `challenger_m2_2_stress_tests`: PASS
- 테스트 PASS가 물리적으로 올바른 RT를 의미하는지 확인하기 위해 2D/3D 투영 결과와
  후보별 구조 지표를 별도로 비교했다.

**확인된 문제**

- TESL/asymmetric aggregate 누락으로 finalist TESL이 0이다.
- 후보별 상대 coverage 정규화 때문에 sparse false basin이 선택된다.
- staged NID 개선 기준이 0으로 완화되어 gate가 약화됐다.
- CTest는 물리 RT 정답을 검증하지 않고, CTest 경로가 `/workspace`에 의존한다.
- `INTERNAL_GATE_PASS`와 실제 방향 정확도 사이에 false positive가 남아 있다.

**수정·개선**

- 위 문제를 코드 변경 없이 평가 문서와 수정 우선순위로 기록했다.
- 제품 승격 조건과 `activation_allowed=false` 상태를 재확인했다.
- 1단계 수정 순서를 `TESL aggregate → absolute support → finalist ambiguity →
  반복성 gate`로 정리했다.

**남은 문제**

- 1단계 TESL/absolute support/ambiguity 수정 전에는 automatic RT 완성으로 판정하지 않는다.

관련 문서: [GEMINI_LOGIC_EVALUATION_AND_REMEDIATION_PLAN_20260821.md](GEMINI_LOGIC_EVALUATION_AND_REMEDIATION_PLAN_20260821.md)

### 2026-08-21 — Jenkins scene0 CH1 시간 반복성 검증

**업무 목적**

- 동일한 강체 설치 상태에서 서로 다른 시각에 얻은 CH1 image–LiDAR pair가 같은 RT를
  반복해서 산출하는지 확인한다.
- 세 pair로 RT를 추정하고 네 번째 pair는 최적화에 사용하지 않는 hold-out으로 검증한다.

**업무 수행**

- `data/jenkins-capture/scene0`의 Jenkins 패키지 4개를 검사했다.
- CH1 영상·LiDAR JSON을 패키지 단위로 pairing하고 앞 3개 training, 마지막 1개
  hold-out으로 확정했다.
- 중첩 패키지, CH1 선택, `manifest.json` 제외를 직접 지원하도록
  `run_real_calibration` 입력 탐색을 수정하고 Docker에서 빌드했다.
- 3 training + 1 hold-out 결합 실행과 training 3개 단독 실행을 같은 옵션으로 수행했다.
- 모든 stdout, JSON, CSV, 2D reprojection, 3D preview를 generated 디렉터리에 보존했다.

**입력 및 공통 실행 조건**

- 패키지: `build5`, `build8`, `build9`, `build10`.
- CH1 해상도: `2592×1520`.
- 좌표 계약: `+x right, +y down, +z forward`, JSON range formula 그대로 사용.
- K+D: Manual ChArUco profile `charuco-pass-clean18-20260814` 고정.
- image distortion: `raw` 입력에 Manual radtan D 적용.
- camera center prior: `(0.05928, -0.08105, 0) m`.
- Manual RT prior는 사용하지 않았다.
- LDC: `unknown`, zoom/focus: 고정, search: `15° → 5° → 1° → Ceres`.

**코드 수정**

- `run_real_calibration`이 Jenkins 루트를 직접 받을 수 있도록 입력 탐색을 수정했다.
- `_CH1` suffix 필터로 CH1만 선택하고, `_pan_tilt_lidar.json`만 LiDAR 입력으로
  인정하도록 했다.
- `manifest.json`을 JSON 입력에서 제외하고, 동일 부모 패키지 내부에서만 image–scan을
  pairing하도록 했다.
- scan 파일명 기준으로 정렬해 `pair-start`와 `hold-out-count` 분할이 결정론적으로
  동작하도록 했다.
- 기존 flat directory 입력 방식은 유지했다.

**확인된 문제**

- `build9`과 `build10`의 CH1 영상은 SHA-256이 같아 hold-out 영상이 독립적이지 않다.
- 결합 실행은 yaw `170°`, down `29°`, roll `-1°`, training `3/3`, limited hold-out
  `1/1`로 내부 PASS했다.
- 단독 pair 0은 잘못된 yaw `-128°`를 PASS로 선택했다.
- pair 0↔1 회전 차이 `64.548°`, pair 0↔2 `68.564°`, pair 1↔2도 `8.190°`여서
  동일 설치 RT 반복성은 FAIL이다.
- pair 0의 잘못된 후보와 물리적으로 더 타당한 후보의 confidence 차이는 약 `0.00622`인데
  finalist ambiguity 거절이 없어 더 큰 점수가 그대로 선택됐다.
- scene CSV의 TESL/asymmetric 값은 유효하지만 finalist 집계값은 0으로 남았다.
- pair0에는 `-185°`라는 더 타당한 후보가 있었지만 최종 선택기는 confidence가 약
  `0.00622` 높은 `-128°`를 선택했다.
- `-128°`는 visible edge 175개만 사용했지만, 후보별 상대 coverage 정규화 때문에
  sparse 후보가 불리하지 않았다.

**수정·개선**

- 원본 데이터를 복제·이동하지 않고 Jenkins 패키지 루트를 직접 실행할 수 있게 했다.
- channel과 scan JSON을 명시적으로 필터링하고 같은 부모 디렉터리 안에서만 pairing한다.
- 스캔 파일명 시간순으로 pair를 정렬해 `pair-start`와 hold-out 분할을 결정론적으로 만들었다.
- 숫자 PASS와 시각적 방향 판정을 분리하고 제품 activation은 계속 차단했다.
- 원시 LiDAR 반복 스캔의 팬 방향 상관을 확인해 build5의 약 60° 좌표 회전 가설을
  점검했다. build5↔build8/9의 최적 팬 이동은 약 `0~0.9°`로 확인되어, pair0의
  `-128°`는 센서 좌표계 회전보다 후보 선택 false positive로 판단했다.
- `-185°`와 `-128°` finalist의 confidence 구성 요소를 분해해 ground score가
  yaw 선택을 간접적으로 뒤집은 경로를 확인했다.

**남은 문제**

- finalist 간 confidence margin gate, TESL aggregate, absolute support 비교를 수정해야 한다.
- 독립 CH1 프레임을 포함한 진짜 hold-out을 추가해야 한다.
- 반복성 gate 통과 전에는 결합 `CANDIDATE_RT`를 제품 RT로 승격하지 않는다.
- 탐색 간격을 더 줄이는 것은 1순위 해결책이 아니다. 정상 basin은 이미 탐색됐고,
  finalist 비교·ambiguity rejection·absolute support가 먼저 수정되어야 한다.

관련 문서:
[JENKINS_SCENE0_CH1_REPRODUCIBILITY_20260821.md](JENKINS_SCENE0_CH1_REPRODUCIBILITY_20260821.md)

### 2026-08-22~23 — Jenkins scene0 CH1 build17/20/21 확장 검증

**업무 목적**

- 해상도를 수정한 build17과 새 build20/21을 테스트 목록에 추가한다.
- 추가 pair를 단독 추정하고 build17~19 training, build20~21 hold-out batch로 검증한다.
- 코드 gate PASS와 입력 데이터 conformance PASS를 분리해 판단한다.

**업무 수행**

- build17/20/21의 해상도, SHA-256, manifest, LiDAR JSON/PCD와 장면을 재감사했다.
- Docker build와 CTest 9종을 수행해 `9/9 PASS`, `111.59 s`를 확인했다.
- build17, build20, build21을 각각 CH1 단독 staged search로 실행했다.
- build17/18/19를 training, build20/21을 stress hold-out으로 분리한 5-pair batch를
  실행했다.
- 모든 JSON, CSV, 2D reprojection, 3D preview와 console log를 generated 경로에
  보존하고 시각 검토했다.

**정량 결과**

| 실행 | 상태 | yaw/down/roll | 핵심 값 |
|---|---|---|---|
| build17 단독 | `INTERNAL_GATE_PASS` | `169° / 22° / 4°` | confidence `0.789297`, 약 8분 19초 |
| build20 단독 | `FAIL / FINALIST_AMBIGUOUS` | `51° / 26° / 7°` rejected | 51°와 166° margin `0.013909 < 0.02` |
| build21 단독 | `FAIL / COARSE_OVERLAP_INSUFFICIENT` | `0° / 15° / -15°` rejected | `MANHATTAN_VERTICAL_ALIGNMENT_POOR` 동반 |
| 3 train + 2 hold-out | 코드상 `CANDIDATE_RT` | `169° / 23° / 5°` | training `3/3`, hold-out `2/2`, 약 22분 37초 |

build17 단독과 새 batch RT 차이는 회전 `1.373°`, 이동 `1.740 mm`다. 같은 yaw
basin은 반복됐지만 제품 반복성 목표 회전 `≤0.2°`를 입증하지는 못했다.

**확인된 문제와 판정**

- build17은 `2592×1520`으로 실행 가능해졌지만 `AI로 생성한 콘텐츠` 표기가 있어
  원본 pixel geometry를 보장할 수 없다.
- build20/21은 사람을 포함하며 파일명 시각 규칙을 적용하면 camera–scan 간격이 각각
  약 2시간 52분, 25시간 49분이다. manifest가 이 시각 계약을 저장하지 않는다.
- 단독 실행에서는 ambiguity/overlap gate가 잘못된 후보를 안전하게 차단했다.
- batch fixed RT는 build20/21 hold-out을 통과했다. 이는 정적 방 구조가 같은 RT로
  설명됨을 뜻하지만 camera–scan 동시성 증거는 아니다.
- 프로그램상 `CANDIDATE_RT`와 별개로 이번 batch의 데이터 conformance는 FAIL이며
  `activation_allowed=false`를 유지한다.

**남은 작업**

- resize/AI 보정 없는 원본 `2592×1520` CH1과 같은 job의 LiDAR scan을 수집한다.
- manifest에 camera/scan UTC 시각, 해시, profile과 installation epoch를 넣고 시간 차이
  preflight gate를 구현한다.
- 원본·동시·사람 없는 독립 hold-out 3쌍 이상과 동일 설치 반복 10회를 확보한다.
- 실데이터 full search는 약 8분/pair, 22분/batch이므로 Jenkins weekly/수동 항목으로 둔다.

관련 문서:
[JENKINS_SCENE0_CH1_EXPANDED_TEST_20260822.md](JENKINS_SCENE0_CH1_EXPANDED_TEST_20260822.md)

위 camera–scan 시간 불일치 판정은 당시 파일 기준 기록이다. 2026-08-23에 파일명과
EXIF를 scan 시작 시각에 맞춘 수정본이 들어왔고, 아래 재검증으로 최신 판정을 갱신했다.

### 2026-08-23 — build20/build21 영상 시각 정정 재검증

**업무 목적**

- 수정된 build20/build21 영상이 올바른 LiDAR scan과 pairing되는지 확인한다.
- 영상 변경이 단독 RT 식별과 3-training/2-hold-out 결과에 미친 영향을 기존 결과와
  동일 바이너리 A/B로 비교한다.

**입력 감사**

- build20 CH1은 `20260822_000015_CH1.jpg`, build21 CH1은
  `20260822_225748_CH1.jpg`로 교체됐다.
- 두 파일 모두 `2592×1520`이고 파일명·EXIF가 대응 LiDAR session 시작명과 일치한다.
- 새 CH1 SHA-256은 각각 `1fe7f93e...b256d8`, `ac22e5d4...2cb72`다.
- 두 scan은 약 `570.5 s`, `570.9 s` 동안 누적된다. 시작 프레임 pairing은 확인됐지만
  장면 원거리에 사람이 있어 동적 객체 위험은 남는다.
- 수정 전·후 prepared PNG와 5°/1° score CSV가 byte-identical이므로 실제 영상
  픽셀은 같고 파일명/EXIF provenance만 정정된 것으로 확인했다.

**실행 및 결과**

| 실행 | 결과 | 핵심 값 |
|---|---|---|
| build20 수정본 단독 | `FAIL / FINALIST_AMBIGUOUS` | 51°와 166° confidence margin `0.013909` |
| build21 수정본 단독 | `FAIL / COARSE_OVERLAP_INSUFFICIENT` | seed `0°/15°/-15°`, Manhattan vertical FAIL |
| build17~19 train + 수정 build20~21 hold-out | `CANDIDATE_RT / PASS` | `169°/23°/5°`, training `3/3`, hold-out `2/2` |

- batch `R`, `t`, confidence `0.785223`, margin `0.033943`과 scene별 hold-out
  지표는 수정 전 결과와 동일했다.
- 수정 전·후 hold-out 2D matching PNG와 3D preview도 각각 byte-identical이었다.
- 이번 실측 runtime은 단독 약 `9.93 min`, `10.21 min`, batch 약 `30.40 min`이었다.
- 코드 변경이 없고 직전 동일 바이너리 CTest가 `9/9 PASS`였으므로 CTest는 중복
  실행하지 않았다.

**판정과 남은 문제**

- 기존 장시간 camera–scan 불일치 해석은 수정본에 대해 철회한다.
- filename/EXIF 동기는 확인됐지만 manifest에 camera/scan UTC 시각과 파일 SHA가 없어
  자동 conformance gate는 아직 구현되지 않았다.
- 파일 provenance 정정은 캘리브레이션 입력 픽셀을 바꾸지 않았으므로 build20/21 단독
  식별 실패 원인은 그대로 남았다.
- batch는 결정론적으로 재현됐지만 training 편집 영상, 동적 객체, 독립 물리 참값과
  동일 설치 10회 통계가 없어 제품 RT로 승격하지 않는다.
- `activation_allowed=false`를 유지한다.

관련 문서:
[JENKINS_SCENE0_CH1_EXPANDED_TEST_20260822.md](JENKINS_SCENE0_CH1_EXPANDED_TEST_20260822.md)

### 2026-08-23 — 문서·코드·결과 재감사 및 finalist 로직 고도화

**업무 목적**

- 현재 문서의 완료 주장과 실제 코드·실데이터 증거가 일치하는지 재평가한다.
- corrected build20/build21에서 지속된 단독 식별 실패와 batch 후보 변동의 원인을 찾는다.
- 임계값 완화 없이 정상 basin을 보존하고 false pass를 차단하도록 staged 로직을 수정한다.

**문서·코드 감사 결과**

- 기존 최종 보고서의 “결함 완전 해결/false basin 완전 제거/최종 승인”은 최신 증거보다
  강한 주장이라 역사적 R4 평가로 정정했다.
- camera-center bracket residual은 강한 translation prior 준수값이므로 독립 정확도
  증거가 아님을 명시했다.
- 정책 문서의 “Ceres 1회”와 실제 최대 3-finalist Ceres 구현이 달라 정책을 수정했다.
- 명시적 raw 입력 + Manual K+D인 경우 `ldc_enabled=unknown`은 traceability warning이며,
  raw/rectified 상태 자체가 불명확할 때만 diagnostic-only로 분류하도록 정정했다.

**근본 원인과 반례**

- build21 정상 `yaw≈168°` basin은 full coarse에서 발견됐지만 global NID 최대값을 local
  hard gate 분모로 사용해 `COARSE_OVERLAP_INSUFFICIENT`로 전부 제거됐다.
- relative NID hard gate만 0으로 낮춘 A/B에서는 coverage confidence가 높은
  `yaw≈85°` false basin이 잘못 내부 PASS했다.
- build20은 `166°`와 `65°` objective 차이가 약 0.14%라 단독 장면 자체가 모호했다.
- batch 정상 후보는 matching objective가 7.954% 우세했지만 confidence margin은
  1.344%라 confidence-only ambiguity가 과도하게 거절했다.

**구현 변경**

- 5°/1° NID relative hard gate를 basin-local reference로 변경했다.
- global NID는 final objective/confidence soft 항목으로 유지했다.
- scene/core/pose/absolute support → objective → TESL → confidence 계층 선택을 적용했다.
- objective와 confidence margin이 모두 2% 미만일 때 ambiguity로 실패하도록 수정했다.
- 선택 support가 viable competitor의 60% 미만이면 계속 fail-closed한다.
- 결과 JSON에 objective/confidence margin과 실제 support/선택 policy를 추가했다.
- threshold pairwise comparator의 비추이성 가능성을 제거하고 명시적 단계 선택으로 바꿨다.
- 3-candidate 순환 fixture의 모든 6개 입력 순서가 같은 후보를 선택하는 회귀를 추가했다.

**검증 결과**

- Core/Challenger CTest 6종: `6/6 PASS`.
- build21 단순 NID soft A/B: 잘못된 `85°` PASS를 확인하고 폐기.
- build21 개선: 정상 `168°` 방향 회복.
- build20 개선: `166°` 진단 방향을 선택하지만 `FINALIST_AMBIGUOUS`로 안전 거절.
- build17~19 training + build20~21 hold-out:
  `167°/37.16°/7°`, training `3/3`, hold-out `2/2`, `CANDIDATE_RT`.
- 2D/3D hold-out 시각 검토에서 gross 반전은 제거됐으나 pixel ground truth 수준은 아님.

**현재 판정과 남은 문제**

- 후보 RT 자동 추정 경로는 동작하지만 제품 RT 자동화는 완료되지 않았다.
- `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`를 유지한다.
- 이전 후보와 최신 후보가 회전 약 14.299°, 이동 약 20.91 mm 달라 algorithm provenance가
  필수다.
- 독립 물리 reference, 원본·동시 hold-out 3쌍 이상, 동일 설치 10회 반복성,
  candidate별 hold-out margin이 남았다.
- full batch 약 40분의 성능은 adaptive ±3°→경계축 ±5° 확장으로 별도 최적화한다.

관련 문서:
[CALIBRATION_LOGIC_EVALUATION_AND_IMPROVEMENT_20260823.md](CALIBRATION_LOGIC_EVALUATION_AND_IMPROVEMENT_20260823.md)

### 2026-08-24 — finalist별 hold-out 식별성 구현·재검증

**업무 목적**

- 수정됐다고 전달된 build20/build21 입력을 재감사한다.
- 선택 RT만 검사하던 hold-out의 false-basin 비식별 허점을 제거한다.
- build17~21을 다시 실행해 이전 `CANDIDATE_RT` 판정이 유효한지 확인한다.

**실행 및 수정**

- build20/build21 raw·prepared SHA-256과 pair 순서를 재검증했다.
- 최대 3개 Ceres finalist 각각에 동일 hold-out을 고정 적용하는 검증을 구현했다.
- 분리 viable 후보가 선택 후보 이상의 hold-out pass ratio이면
  `FINALIST_HOLDOUT_AMBIGUOUS`로 강등하도록 fail-closed했다.
- 후보별 CSV/JSON과 Test 11을 추가했다.
- Docker 빌드, Core/Challenger/Manual/Top-view 및 20260818/19 실데이터 회귀를 실행했다.
- build17~19 training + build20~21 hold-out batch를 재실행했다.

**결과**

- 입력 픽셀과 full/5°/1° score map, 167° diagnostic RT는 이전 결과와 동일했다.
- 선택 167°, 경쟁 87°와 −106° 세 후보 모두 hold-out `2/2 PASS`였다.
- 최신 상태는 `INTERNAL_GATE_PASS / FINALIST_HOLDOUT_AMBIGUOUS`,
  `NOT_CANDIDATE_RT`, `activation_allowed=false`다.
- 전체 CTest 11개 중 9개는 PASS했다. 20260818/19 두 항목은 크래시가 아니라 각각
  `FINALIST_AMBIGUOUS`, `FINALIST_HOLDOUT_AMBIGUOUS` expected rejection으로 종료됐다.
- 두 장시간 expected-rejection을 제외한 기본 회귀는 `9/9 PASS`, `85.96 s`다.

**판단**

기존 binary scene gate는 “이 RT가 허용 범위 안인가”는 검사하지만 “경쟁 RT보다
유일하게 우수한가”를 검사하지 못했다. 현재 데이터만 보고 연속 score 가중치를 사후
조정하지 않는다. 독립 physical/perturbation fixture로 hold-out objective margin을 먼저
정의하고 검증해야 한다.

상세 기록:
[FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md](FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md)

### 2026-08-24 — finalist hold-out 연속 목적함수 고도화

**업무 목적**

- Binary `2/2 PASS` 동률을 근거 없이 모두 모호성으로 처리하는 한계를 해소한다.
- build20/21에 맞춘 사후 가중치 없이 학습과 같은 목적함수로 후보 식별성을 평가한다.
- 20260818/19와 build17~21에서 false acceptance 회귀를 확인한다.

**코드 수정**

- 장면별 Edge squared objective, geometry NID squared objective, 구조선 score weight,
  Manhattan objective, signal NMI squared objective를 `PoseSceneMetrics`에 보존했다.
- `summarizeCalibrationPoseScenes()`로 학습과 동일한 장면 집계와 composite objective를
  구현했다.
- Hold-out 모든 finalist의 최대 visible edge/NID point/spatial cell을 공통 coverage
  분모로 사용했다.
- 같은 pass-ratio tier에서 기존 2% objective margin 미달 경쟁 후보만
  `FINALIST_HOLDOUT_AMBIGUOUS`로 처리했다.
- 후보별 JSON/CSV에 목적함수 성분, coverage, 선택 대비 margin을 기록했다.
- 20260818은 exit 3과 `FINALIST_AMBIGUOUS`를 함께 검사하는 weekly expected-rejection,
  20260819는 weekly candidate regression으로 CTest를 분리했다.

**검증 결과**

| 데이터 | 결과 | 연속 hold-out 근거 |
|---|---|---|
| build17~21 | `CANDIDATE_RT / PASS` | 167° `0.763763`; 경쟁 87°/−106° 대비 최소 `6.491%` |
| 20260818 | `FAIL / FINALIST_AMBIGUOUS` | training ambiguity 유지; 무분별한 승격 없음 |
| 20260819 | `CANDIDATE_RT / PASS` | 80° `0.733585`; 165° 경쟁 후보 대비 `9.882%` |

- build17~21 및 20260818/19의 전역·1° score map 해시는 수정 전과 동일했다.
- 빠른 Core/Challenger/Manual/Top-view 회귀는 `9/9 PASS`, `88.52 s`였다.
- 모든 candidate는 `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`다.

**판단**

현재 수정은 binary 동률을 학습 목적함수로 구분하므로 기존 허점을 줄였다. 그러나 2%
margin 자체는 물리 참값 기반 false-acceptance 시험으로 아직 승인되지 않았다. 독립 물리
reference, 원본 동시 hold-out 3쌍 이상, 동일 설치 10회 반복성이 남아 있다.

### 2026-08-24 — Manhattan 특징 prior 일관성 감사·수정·1회 검증

**발견한 문제**

- training은 finalist의 `seed.prior`로 수직 소실점 축을 선택했다.
- fixed-pose/hold-out은 refined candidate RT로 수직축을 다시 선택했다.
- 이 차이는 후보가 평가 단계에서 자신에게 유리한 다른 특징 축을 고를 수 있게 해,
  “학습과 동일 목적함수”라는 계약을 깨뜨릴 위험이 있었다.

**수정**

- `evaluateCalibrationPoseScenes()`에 명시적 Manhattan feature prior를 추가했다.
- training pass-ratio와 finalist별 hold-out은 해당 finalist의 동일 training seed prior를
  재사용하도록 runner 호출을 수정했다.
- 직교 소실점 합성 fixture로 explicit prior 적용을 검사하는 Core 회귀를 추가했다.
- 이후 산출물에 prior 정책을 machine-readable하게 기록하도록 JSON metadata를 추가했다.
- 실데이터 1회 실행 뒤 metadata를 추가하고 runner만 재빌드했으므로 이번 세 JSON에는
  필드가 없고 다음 실행부터 기록된다. Prior 고정 로직 자체는 이번 실행에 적용됐다.

**1회 실행 결과**

| 데이터 | 상태 | 방향 및 hold-out 식별성 |
|---|---|---|
| build17~21 | `CANDIDATE_RT / PASS` | 167° / 37.1584° / 7°; J `0.763763`; 최소 margin `6.491%` |
| 20260818 | `FAIL / FINALIST_AMBIGUOUS` | 진단 175° / 20.0067° / 3°; training 단계 안전 거절 |
| 20260819 | `CANDIDATE_RT / PASS` | 80° / 30.0006° / 4°; J `0.733585`; margin `9.882%` |

- 세 실행의 수정 전후 `max ΔR=0`, `max Δt=0 m`였다.
- status/reason/selected/objective와 score-map SHA-256도 동일했다.
- 빠른 회귀 `9/9 PASS (77.19 s)`, 새 prior Core 회귀 `1/1 PASS (5.51 s)`였다.
- 요청에 따라 실데이터는 각각 1회만 실행했고 threshold 튜닝·재시도를 하지 않았다.

**판단**

현재 fixture의 답은 변하지 않았지만 training/hold-out 특징 계약의 잠재적 평가 누수를
제거했다. 이것은 정확도 승격이 아니라 안전성 보강이다. 물리 기준 RT, 제품용 독립
hold-out, 10회 반복성, provenance gate가 없으므로 `activation_allowed=false`를 유지한다.

### 2026-08-24 — 1회 제한 해제 및 weekly 실데이터 계약 완결

**업무 목적**

- 이전 “변경 후 각 fixture 1회” 제한을 해제하고 정식 반복 회귀 경로를 복구한다.
- 성공 케이스가 exit 0만 확인되던 Jenkins 계약 허점을 제거한다.

**코드 수정**

- 성공·예상 거절을 `tests/verify_real_calibration_result.cmake` 하나로 통합했다.
- 20260818은 exit 3, `FAIL / FINALIST_AMBIGUOUS`, `NOT_CANDIDATE_RT`를 기대한다.
- 20260819는 exit 0, `CANDIDATE_RT / PASS`, `CANDIDATE_RT`를 기대한다.
- 양쪽 모두 제품 RT 비승격, `activation_allowed=false`와 fixed Manhattan feature-prior
  policy를 JSON에서 검사한다.

**실행 결과**

- 빠른 회귀: `9/9 PASS`, `52.26 s`.
- weekly 실데이터: `2/2 PASS`, 병렬 real time `1240.91 s`.
- 개별 시간: 20260819 `1027.19 s`, 20260818 `1240.90 s`.
- 새 weekly JSON에 prior policy가 기록됐고, prior-locked 결과와 full/5°/1° score map
  SHA-256이 모두 동일했다.
- 20260818/19 모두 `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`를 유지했다.

**판단**

코드 회귀와 결과 수명주기 계약은 자동화됐지만 물리 정확도 승인 근거는 늘지 않았다.
다음 P1은 threshold 추가 조정이 아니라 독립 물리 reference, 원본·동시 product hold-out
3쌍 이상, 동일 설치 10회 반복성, 수집 manifest/provenance gate다. 현재 Jenkins
manifest를 재감사한 결과 build/session/count/created-at만 있고 camera/scan UTC,
SHA-256, profile, installation epoch가 없으므로 기존 fixture만으로 provenance gate를
완료할 수 없다. 다음 구현은 수집 producer가 이 필드를 기록하도록 계약을 먼저 바꾸는
것이다.

## 현재 미완료 항목

| 우선순위 | 미완료 항목 | 완료 판정 기준 |
|---|---|---|
| P1 | 원본·동시 product hold-out 3쌍 이상 | 편집/재사용 프레임 없이 manifest 시간 gate 통과, fixed RT 장면별 PASS |
| P1 | 동일 설치 같은 binary 반복 10회 | rotation 표준편차 `≤0.2°`, translation 표준편차 `≤10 mm` |
| P1 | 독립 물리 reference 검증 | jig/CAD/survey 또는 LiDAR-visible target 기준 RT 오차 통과 |
| P1 | 수집 manifest 시간·profile gate | camera/scan UTC 시각·해시·profile·installation epoch 자동 검증 |
| P1 | Finalist별 연속 hold-out margin 검증 | 구현 완료. 기존 2% 기준의 false acceptance를 독립 physical/perturbation fixture에서 검증 |
| P1 | Algorithm provenance freeze | source/binary version, 입력 SHA, intrinsic profile과 RT artifact를 한 묶음으로 보존 |
| P2 | 동적 객체 및 stale-frame 강건성 | 사람/이동 물체 stress에서 false activation 0건 |
| P2 | ground plane 의미 판정 | 바닥·책상·장애물 오인식 회귀 테스트 통과 |
| P2 | 실데이터 탐색 성능 | 정확도 유지 상태에서 weekly batch 시간 예산 충족 |
| P2 | Jenkins/Ubuntu 재현성 | ignored 경로 없이 fixture 또는 dataset root 주입으로 실행 |
| P3 | OpenSDK 실장 | 실제 camera/LiDAR stream에서 동일 JSON/RT 계약 검증 |

## 변경 이력

| 날짜 | 변경 |
|---|---|
| 2026-08-21 | 현재까지의 날짜별 업무·문제·수정·개선·잔여 이슈 통합 로그 신규 작성 |
| 2026-08-21 | Jenkins scene0 CH1 4묶음의 데이터 감사, 결합 3+1과 단독 3회 결과, 반복성 FAIL 및 후속 수정 우선순위 추가 |
| 2026-08-21 | 8월 20일·21일 작업을 목적·입력·실행 조건·정량 결과·원인 분석·다음 조치 기준으로 상세 보강 |
| 2026-08-23 | build17/20/21 단독 및 3-training/2-stress-hold-out 확장 실행, 코드 결과와 데이터 적격성 분리 기록 |
| 2026-08-23 | build20/build21 파일명·EXIF 수정본 재감사, 단독 2회와 batch 재실행, pixel/score/RT 동일성 기록 |
| 2026-08-23 | global NID/local basin 실패 원인 A/B, finalist 계층·dual-margin·결정론 수정, 3-training/2-hold-out 재검증 및 문서 정합성 갱신 |
| 2026-08-24 | build20/build21 해시 재감사, finalist별 binary hold-out fail-closed 구현 및 중간 `NOT_CANDIDATE_RT` 기록 |
| 2026-08-24 | 학습 동일 연속 hold-out 목적함수·공통 coverage 적용, build17~21/20260819 `CANDIDATE_RT`, 20260818 안전 거절 재검증 |
| 2026-08-24 | Manhattan 특징 prior training/hold-out 일관성 수정, explicit-prior 회귀 및 세 실데이터 1회 결과 불변 검증 |
| 2026-08-24 | 1회 제한 해제, 성공·예상 거절 공통 weekly JSON 계약 통합 및 `2/2 PASS` 검증 |
| 2026-08-24 | CH1 build5~24 전체 데이터 활용 계획 실제 실행: ChArUco 22/24 PASS, Case C primary PASS, Case A expected ambiguity rejection, Case B stress PASS, primary RT fixed 9/9 scene PASS |

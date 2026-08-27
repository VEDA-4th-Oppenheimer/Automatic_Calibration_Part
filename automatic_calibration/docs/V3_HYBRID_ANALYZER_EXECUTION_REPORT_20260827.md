# V3 Hybrid Analyzer 구현·실행 보고서

- 작성일: 2026-08-27
- 브랜치: `codex/exp-hybrid-analyzer-v3-20260827`
- 기준 B0: `develop@f684cd66`
- 대상 채널: CH1
- 결과 상태: **prototype 검증 완료, 제품 승인 기준 미달**

## 1. 구현 범위

추가:

- `include/auto_calib/hybrid_orientation_analyzer.hpp`
- `src/hybrid_orientation_analyzer.cpp`
- `apps/run_hybrid_orientation_analyzer.cpp`
- `tests/hybrid_orientation_analyzer_tests.cpp`
- `scripts/run_v3_hybrid_eval.sh`

재사용:

- T2: `panorama_raster_builder.*`
- T1: `image_vanishing_estimator.*`, `lidar_manhattan_estimator.*`

E2E 통합:

- `run_real_calibration --orientation-analyzer hybrid`
- Analyzer 제안 성공 시 bounded search
- feature/score/bounded gate 실패 시 B0 staged search fallback
- pipeline/analyzer/internal candidate/projection-scene runtime 계측

제거:

- T2 전역 perspective remap 실행 경로
- V3 build에서 legacy T2 remapper/analyzer target

## 2. 사용 데이터

### 설치 구성 A

경로: `data/real_calibration/session-const-env/repeat_test_sample/20260818`

사용 pair:

1. `20260818-143751-CH1.jpg` + `calib-20260818-143748_sweep-000001_pan_tilt_lidar.json`
2. `20260818-145847-CH1.jpg` + `calib-20260818-145912_sweep-000001_pan_tilt_lidar.json`
3. `20260818-151305-CH1.jpg` + `calib-20260818-151312_sweep-000001_pan_tilt_lidar.json`

진단 reference yaw: 169°. 이는 기존 B0 결과 기반이며 독립 ground truth가 아니다.

### 설치 구성 B

경로: `data/jenkins-capture/scene0`

- build22: training
- build23: training
- build24: hold-out

Case C B0 reference: yaw 177°, down 42°, roll 3°.

주의: standalone script는 원본 image를 읽고, E2E는 calibration pipeline에서 준비/undistort한 첫 image를 analyzer에 전달한다. 두 경로의 proposal 수치를 완전히 동일 조건으로 간주하면 안 된다.

## 3. 구현 중 시도와 결과

### 3.1 초기 V3: all-edge azimuth signature

Camera Canny edge 전체를 column signature로 축약하고 LiDAR azimuth signature와 correlation했다.

결과:

- 구성 A recall@3: 0/3
- 구성 B recall@3: 2/3
- 반복 벽/가구 edge가 azimuth alias를 만들었다.
- 구조 hypothesis의 nearest signed permutation을 down/roll로 바로 사용해 Case C down이 약 22°로 틀어졌다.

### 3.2 Gravity vanishing-point 직접 선택

Camera vanishing direction 중 camera y·z 부호와 |y| support를 이용해 gravity 후보를 골랐다.

build22에서 선택 가능한 triad가 실제 B0 vertical 방향을 충분히 포함하지 못했다. 가구/모니터/기둥 선분이 vanishing cluster에 섞였고, 실제 down 42° 대신 약 23~26°를 예측했다.

결론: 단일 VP triad를 down 정답으로 직접 사용하는 것은 불안정하다.

### 3.3 Directional azimuth/elevation signature

- azimuth: `|Gx| > 1.25|Gy|`인 vertical-dominant camera boundary만 사용
- elevation: `|Gy| > 1.25|Gx|`인 horizontal-dominant boundary 사용
- LiDAR는 각 yaw camera FOV sector의 elevation signature 생성
- down 0~75°를 1° 간격으로 cheap 1D score 평가
- VP roll은 elevation과 일치할 때만 사용

이 변경으로 구성 B recall이 3/3으로 개선됐으나 구성 A는 0/3이었다.

### 3.4 Covariance seed와 48-candidate budget

Top-3 각각에 down±sigma 두 seed를 생성했다. coarse stage를 center±5°로 바꾸고, 동일 yaw basin을 collapse한 후 1° fine와 Ceres를 수행했다.

10° 간격 center±10° 방식은 175° proposal에서 165/175/185°만 평가해 objective가 185°를 선택하면 fine ±3°가 B0 177°에 도달하지 못했다. 이를 center±5°, 5° 간격으로 수정해 170/175/180°를 평가했고 최종 yaw 178.25°를 얻었다.

## 4. Standalone 6-pair 최종 결과

B0 basin recall 기준은 Top-3 중 reference yaw±10° 존재 여부다.

| 구성 | Case | rank-1 yaw | 최소 Top-3 오차 | recall@3 | runtime |
|---|---|---:|---:|---|---:|
| A | repeat_0 | -44° | 11° | FAIL | 2.098 s |
| A | repeat_1 | -50° | 42° | FAIL | 2.417 s |
| A | repeat_2 | -51° | 15° | FAIL | 2.446 s |
| B | build22 | -134° | 2° | PASS | 3.275 s |
| B | build23 | -177° | 6° | PASS | 2.850 s |
| B | build24 | 177° | 0° | PASS | 2.654 s |

- 전체 recall: 3/6 = 50%
- 구성 A: 0/3
- 구성 B: 3/3
- standalone analyzer runtime: 평균 2.623 s, 범위 2.098~3.275 s
- 일반 입력 fallback: 0/6

Fallback 0/6은 성공 지표가 아니다. 구성 A의 세 실패에서도 fallback하지 않았으므로 confidence gate가 모호성을 감지하지 못했다.

## 5. Case C E2E 개선 이력

| Run | 핵심 변경 | B0 회전 차이 | 이동 차이 | 내부 후보 | 비고 |
|---|---|---:|---:|---:|---|
| r1 | 초기 hybrid, 구조 down 사용 | 16.951° | 24.443 mm | 계측 전 | down 약 22° 오류 |
| r2 | directional elevation signature | 5.551° | 9.073 mm | 57 | yaw/down 개선 |
| r3 | covariance seed, 48 budget, coarse 10° spacing | 3.236° | 5.695 mm | 48 | fine window가 177°를 놓침 |
| r4 | coarse center±5°, 5° 간격 | **2.394°** | **4.155 mm** | **48** | 최종 현재 결과 |

## 6. r4 최종 수치

출력: `automatic_calibration/generated/v3_case_c_build22_24_final_e2e_r4/calibration_result.json`

상태:

- lifecycle: `CANDIDATE_RT`
- reason: `PASS`
- analyzer fallback: false
- analyzer coverage: 0.994752
- analyzer perspective remaps: 0
- analyzer expensive projection evaluations: 0

Runtime/계산량:

- pipeline: 76,166.329 ms
- analyzer: 1,861.041 ms
- analyzer fraction: 2.443%
- bounded internal orientation candidates: 48
- projection-scene evaluations: 96
- B0 비교 후보: 168, 2 scene이면 336 projection-scene evaluations
- 감소율: 71.429%

최종 V3 orientation 분해:

- yaw: 178.2507°
- down: 42.5607°
- roll: 0.0226°

B0:

- yaw: 177°
- down: 42°
- roll: 3°

차이:

- rotation geodesic: 2.3943°
- translation norm: 4.155 mm
- 잔여 회전 오차의 주성분: roll predictor가 약 0°에 머문 반면 B0는 3°

최종 V3 `T_camera_lidar`:

- R row0: `[-0.999542040, -0.000290088, 0.030259321]`
- R row1: `[0.020252951, 0.736561290, 0.676067514]`
- R row2: `[-0.022483963, 0.676370743, -0.736218099]`
- t[m]: `[0.059232208, 0.058500504, 0.056162137]`

## 7. 합격 기준 판정

| 기준 | 결과 | 판정 |
|---|---:|---|
| B0 basin recall@3 ≥99% | 3/6 | FAIL |
| 정답 basin 누락 0 | 3건 누락 | FAIL |
| fallback ≤20% | 0/6 | 수치 PASS, 안전성 보류 |
| analyzer ≤전체 10% | 2.443% | PASS |
| projection 평가 ≥70% 감소 | 71.429% | PASS |
| RT 회전 1~2° | 2.394° | FAIL(근접) |
| 서로 다른 설치 구성 3개 | 2개 | PENDING |

제품 승격 결론: **불가**.

## 8. 테스트 검증

Targeted Docker test:

- `hybrid_orientation_analyzer_tests`: PASS
- known circular signature yaw recovery: PASS
- textureless constant image/scan: `CAMERA_EDGE_INSUFFICIENT` 및 fallback: PASS

최종 Docker 회귀 테스트:

- 명령 범위: `ctest --test-dir build-v3 --output-on-failure -E '^verify_'`
- 결과: **7/7 PASS, 0 FAIL**
- 최종 실행시간: 112.16 s (동일 테스트 직전 실행 89.55 s)
- real-data `verify_*`는 별도 dataset mount를 요구하므로 이 회귀 묶음에서 제외했다.

## 9. 현재 문제의 원인

### 9.1 단일 scene analyzer

E2E는 현재 첫 training image/scan 한 쌍만 분석한다. 한 장의 반복 구조나 edge alias가 Top-K 전체를 지배할 수 있다.

### 9.2 Raw/undistorted 경로 불일치

Standalone과 E2E의 image preparation이 다르다. 같은 K·D profile을 사용하더라도 undistort/crop/resolution 경로가 다르면 1D signature peak가 변한다.

### 9.3 Roll observability 부족

Camera의 짧은 수직선과 가구 edge가 실제 gravity VP를 안정적으로 만들지 못했다. LiDAR Manhattan axis만으로 camera optical roll 부호와 크기를 확정하기도 어렵다.

### 9.4 Confidence calibration 부족

현재 peak z-score와 feature count는 “B0 basin을 Top-3에 포함했는지”에 대해 calibration되지 않았다. 구성 A의 false confidence가 그 증거다.

### 9.5 표본과 설치 다양성 부족

현재는 같은 장소 계열의 설치 구성 2개뿐이다. 6개 pair는 99% recall을 주장할 수 있는 표본 수가 아니다.

## 10. 권장 다음 구현

1. 모든 training pair에서 azimuth/down score curve를 생성한다.
2. curve를 circular yaw 기준으로 정렬하고 median/trimmed mean으로 aggregate한다.
3. pair별 Top-K가 공통 basin을 지지하지 않으면 즉시 fallback한다.
4. aggregate basin covariance와 pair 간 분산을 분리 출력한다.
5. Roll은 VP 한 개가 아니라 line residual/support를 포함한 robust estimator로 바꾼다.
6. `unsafe_non_fallback_rate=0`을 합격 기준에 추가한다.
7. 설치 구성 C를 수집해 같은 코드/threshold로 blind evaluation한다.
8. 정확도 기준을 통과한 후 azimuth circular correlation을 ARM NEON으로 최적화한다.

## 11. 냉정한 평가

V3는 “analyzer로 search range와 projection 부하를 줄일 수 있는가”에는 긍정적 증거를 냈다. 216 remap 제거, analyzer 2.443%, projection 71.429% 감소는 실제 계측 결과다.

그러나 “analyzer가 정답 가능성이 높은 방향을 안정적으로 찾는가”에는 아직 부정적이다. 한 설치 구성에서 3/3 실패했고 fallback도 하지 않았다. 따라서 다음 작업은 search 간격을 더 줄이는 것이 아니라 multi-scene consensus와 fail-closed confidence를 먼저 구현하는 것이다.

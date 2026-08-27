> **최신 상태(2026-08-27 r5):** 초기 V3의 단일 장면 false acceptance를 막기 위해
> multi-scene yaw/down consensus와 fail-closed B0 fallback을 구현·검증했다.
> 최신 수치와 판정은 `V3_MULTI_SCENE_CONSENSUS_REMEDIATION_REPORT_20260827.md`를 우선한다.

# B0·T1·T2 Analyzer 실험 이력과 V3 Hybrid 설계

- 작성일: 2026-08-27
- 대상: CH1 camera–pan/tilt 1D LiDAR markerless extrinsic calibration
- 기준 브랜치: `develop` B0, T1/T2 실험 브랜치, `codex/exp-hybrid-analyzer-v3-20260827`
- 문서 상태: V3 실험 구현 및 2개 설치 구성 검증 결과 반영
- 제품 상태: **실험 단계. PRODUCT_APPROVED_RT로 승격하지 않음**

## 1. 결론

Analyzer를 비디오 코덱의 사전 분석기처럼 사용해 비싼 3D projection 탐색량을 줄이려는 방향은 타당하다. 다만 analyzer는 최종 RT를 결정하는 장치가 아니라, 정답이 있을 가능성이 높은 Top-K basin과 불확실도를 제안하는 장치여야 한다.

V3는 T2의 panorama raster/signature와 T1의 Manhattan/vanishing-point 분석을 결합하고 전역 perspective remap 216회를 제거했다. Jenkins 구성에서는 B0 yaw basin을 Top-3에 3/3 회수했고, Case C에서 비싼 orientation 후보를 168개에서 48개로 줄였다. 반면 2026-08-18 구성에서는 Top-3 yaw recall이 0/3이었고 그 상황에서도 fallback이 발생하지 않았다. 따라서 현재 V3는 계산량 감소 가능성을 입증한 prototype이지 제품 경로가 아니다.

## 2. 프로젝트 목적과 고정 전제

본 프로젝트의 목적은 설치 후 camera와 LiDAR의 외부 파라미터 `T_camera_lidar = [R|t]`를 자동 추정하는 것이다.

- 현 제품 경로에서는 camera profile의 K·D를 고정 입력으로 사용한다.
- 현재 장비는 manual ChArUco에서 얻은 K·D를 고정 profile로 사용한다.
- Analyzer가 K·D를 공동 추정하거나 lens distortion 자체를 최적화하지 않는다.
- Camera/LiDAR의 mechanical offset은 translation prior와 탐색 범위에 반영할 수 있지만 RT 정답으로 강제하지 않는다.
- B0는 offline oracle 겸 fail-safe fallback이다. B0 역시 절대 ground truth는 아니므로 독립 manual/fixture 기준과 함께 비교해야 한다.
- 최종 RT 승격은 training objective뿐 아니라 hold-out, ambiguity, absolute support 및 독립 설치 검증을 통과해야 한다.

## 3. 브랜치를 나눠 실험한 계기

### 3.1 B0 오염 방지

새 analyzer가 잘못된 방향 후보를 제안해도 기존 full-search의 동작과 증거를 보존해야 했다. T1/T2를 `develop`에 직접 누적하면 성능 회귀가 analyzer 때문인지 기존 calibration core 변경 때문인지 분리할 수 없다.

### 3.2 서로 다른 가설의 독립 평가

T1과 T2는 같은 최적화가 아니다.

- T1 가설: 2D 구조선과 3D Manhattan 구조축만으로 yaw/down/roll basin을 예측할 수 있다.
- T2 가설: organized LiDAR를 panorama로 바꾸고 camera perspective와 비교하면 방향 후보를 빠르게 고를 수 있다.

하나의 브랜치에서 두 방식을 섞으면 어느 구성요소가 recall, runtime, ambiguity에 영향을 줬는지 알기 어렵다.

### 3.3 실행 증거와 실패 보존

각 브랜치에 코드와 JSON/CSV/log evidence를 고정해 동일 입력으로 결과를 재검토할 수 있게 했다. 실패 결과도 삭제하지 않고 다음 설계의 근거로 사용한다.

### 3.4 제품 경로와 연구 경로 분리

B0는 안전한 fallback으로 유지하고 analyzer는 opt-in 실험 경로로 연결했다. Analyzer가 실패하거나 bounded overlap gate를 넘지 못하면 B0를 실행하도록 설계했다.

## 4. 브랜치와 기준점

| 구분 | 브랜치 | 코드/증거 기준 | 역할 |
|---|---|---|---|
| B0 | `develop` | `f684cd66` | 기존 full-search baseline, offline oracle, fallback |
| T1 | `codex/exp-structural-analyzer-20260824` | 코드 `391fd0d`, 증거 `64b79a6` | LSD/vanishing-point/Manhattan 구조 analyzer |
| T2 | `codex/exp-panorama-analyzer-20260824` | 코드 `57989d0`, 증거 `a098def` | LiDAR panorama + perspective-remap analyzer |
| V3 | `codex/exp-hybrid-analyzer-v3-20260827` | 본 문서와 동일 브랜치 | T2 raster/signature + T1 구조 gate, remap 제거 |

T1/T2는 추가 tuning 없이 실험 결과로 동결한다. V3는 T2 코드 기준점 `57989d0`에서 분기했으며 T1의 구조 분석 소스만 선택적으로 가져왔다.

## 5. B0, T1, T2의 의미

### 5.1 B0: full-search baseline

B0는 analyzer 없이 넓은 yaw/down 후보를 calibration objective로 직접 평가하고, coarse-to-fine 탐색과 Ceres refinement 및 hold-out gate를 수행한다.

장점:

- analyzer의 잘못된 pruning 때문에 정답 basin을 버리지 않는다.
- T1/T2/V3의 basin recall을 판정하는 offline oracle로 사용할 수 있다.
- analyzer 신뢰도가 낮을 때 fail-safe fallback으로 사용할 수 있다.

손실:

- Case C 기준 초기 orientation 후보가 168개이고 각 후보에서 image–LiDAR projection/objective 평가가 발생한다.
- CV5 2-core/4 GB 제품 경로에 그대로 적용하기에는 계산량과 지연이 크다.
- 장면에 반복 구조가 있으면 full search도 여러 유사 basin을 만들 수 있으므로 B0 결과가 곧 ground truth는 아니다.

### 5.2 T1: structural analyzer

처리 흐름:

1. Camera image에서 LSD line segment 추출
2. line 교점/방향으로 vanishing direction 추정
3. organized LiDAR grid에서 surface normal 추정
4. normal 분포로 Manhattan frame 추정
5. camera 축과 LiDAR 축의 signed permutation 후보 생성
6. Top-K 구조 방향만 bounded calibration에 전달

득:

- 전역 dense perspective remap 없이 비교적 가볍다.
- 구조가 명확한 Manhattan 장면에서는 방향 후보를 설명 가능한 형태로 만든다.
- T1 E2E에서 B0 부근 후보까지 도달했다.

실:

- 실제 장면의 수직선/수평선이 부족하거나 가구·기둥·왜곡이 섞이면 vanishing triad가 실제 gravity 축을 포함하지 않을 수 있다.
- T1 최종 RT 기록은 `yaw=176.4°`, `down=21.77°`, `roll=-4.46°`이며 단일 수치가 좋아 보여도 구조축 선택 안정성이 부족했다.
- 과거 “168→8, 95.2% 감소”에서 8은 함수 호출 횟수였고 함수 내부에서 평가한 yaw/projection 후보 수가 아니었다. 계산량 감소 근거로 직접 사용할 수 없다.

### 5.3 T2: panorama analyzer

처리 흐름:

1. JSON row/column을 400×101 LiDAR panorama raster로 복원
2. range/normal/validity 특징 생성
3. yaw/down 후보마다 camera perspective를 panorama로 remap
4. 구조 score로 Top-K proposal 생성
5. Top-K bounded search 후 Ceres/hold-out 수행

득:

- LiDAR의 360° organized topology를 직접 활용한다.
- standalone build22/23/24/17에서 rank-1 yaw가 각각 177.5°/170°/177.5°/172.5°로 B0 basin 근처에 형성됐다.
- E2E에서는 analyzer rank-1 170°에서 시작해 bounded refinement가 177°를 찾았다.
- ambiguity gate가 `distinctive=false`, margin `-0.034`, passing competitor 1로 fail-closed한 증거가 남았다.

실:

- 전역에서 약 216회의 perspective remap을 수행해 analyzer E2E runtime이 7.55 s였다.
- Analyzer 자체가 이미 비싼 2D resampling/projection을 반복해 B0 계산을 줄이는 목적과 충돌했다.
- finalist confidence 0.7988이어도 hold-out ambiguity를 해소하지 못했다.
- panorama rank가 좋다는 사실만으로 최종 RT 품질을 보장하지 못한다.

## 6. B0/T1/T2 비교

| 항목 | B0 | T1 | T2 |
|---|---|---|---|
| 주 정보 | calibration objective | 구조선/소실점/normal | panorama perspective score |
| 전역 coverage | 높음 | 구조 가설에 의존 | 높음 |
| analyzer 비용 | 없음 | 낮음~중간 | 높음 |
| 정답 basin 누락 위험 | 가장 낮음 | 구조축 오인 시 큼 | signature alias 시 큼 |
| 설명 가능성 | objective/hold-out | 축·선분으로 설명 가능 | panorama/score map으로 설명 가능 |
| 제품 판단 | fallback/oracle | 단독 채택 불가 | 단독 채택 불가 |

핵심 교훈은 “Top-1을 잘 맞히는 analyzer”보다 “정답 basin을 Top-K 안에서 절대 버리지 않고 불확실할 때 fallback하는 analyzer”가 필요하다는 것이다.

## 7. V3 Hybrid Analyzer 설계

### 7.1 재사용과 제거

- 재사용: T2 `panorama_raster_builder`와 row/column validity/range topology
- 재사용: T1 image vanishing estimator와 LiDAR Manhattan estimator
- 제거: T2의 전역 perspective remap 216회
- 유지: B0 full staged search와 hold-out gate
- 추가: directional 1D signature, Top-K basin, covariance, 내부 평가량 계측

### 7.2 처리 순서

1. JSON을 organized panorama raster로 복원한다.
2. Camera에서는 vertical-dominant edge로 azimuth signature를 만든다.
3. LiDAR panorama에서는 range/normal 구조 변화로 circular azimuth signature를 만든다.
4. 360개 yaw에 대해 1D normalized correlation만 계산한다. 이 단계는 perspective remap과 3D projection을 만들지 않는다.
5. 인접 점수를 평활화한 basin score를 만들고 30° circular NMS로 Top-3를 선택한다.
6. basin 주변 score 분포에서 `yaw_sigma_deg`를 계산한다.
7. Horizontal-dominant camera edge와 yaw별 LiDAR elevation signature로 down 후보와 `down_sigma_deg`를 계산한다.
8. T1 vanishing/gravity 결과가 elevation 결과와 일치할 때만 roll predictor를 사용한다. 불일치하면 roll=0°와 큰 covariance를 출력한다.
9. 각 Top-3에서 down±sigma 두 seed를 만들고 bounded search로 보낸다.
10. bounded candidate가 absolute overlap gate를 통과하지 못하거나 analyzer가 불충분하면 B0 full search로 fallback한다.

### 7.3 Top-K와 covariance의 의미

- `yaw_deg`, `down_deg`, `roll_deg`: proposal 중심
- `basin_score`: 중심 한 점이 아니라 인접 score를 포함한 지역 점수
- `yaw_sigma_deg`: basin 폭에 따른 yaw 불확실도
- `down_sigma_deg`: elevation score 곡선 폭에 따른 down 불확실도
- `roll_sigma_deg`: vanishing-point support와 일관성에 따른 roll 불확실도
- `search_radius_deg`: covariance 기반 bounded-search 권장 반경

Analyzer 출력은 최종 RT가 아니다. 후보 중심과 탐색 예산을 calibration core에 전달하는 proposal contract이다.

### 7.4 실제 bounded-search 예산

현재 V3 Case C 경로의 최대 내부 후보 수는 다음과 같다.

- Top-3 × down covariance 2개 = 6 seed
- coarse: 각 seed에서 yaw `center±5°`, 5° 간격 = 6×3 = 18
- basin collapse 후 최대 3개
- fine: 각 basin `±3°`, 1° 간격 = 3×7 = 21
- Ceres finalist: 각 basin `±1°`, 1° 간격 = 3×3 = 9
- 합계: 18+21+9 = **48 internal orientation candidates**
- training scene 2개 기준: **96 projection-scene evaluations**

B0 168 candidates, 2 scenes의 336 projection-scene evaluations와 비교하면 71.429% 감소다.

### 7.5 Runtime 계측 필드

- Analyzer JSON: `runtime_ms`, `image_feature_ms`, `lidar_feature_ms`, `signature_search_ms`
- 전역 분석량: `evaluated_signature_yaws=360`
- 제거 확인: `perspective_remaps=0`, `expensive_projection_evaluations=0`
- E2E: `bounded_orientation_evaluations`
- E2E 내부 후보: `bounded_internal_yaw_candidates`
- E2E 장면별 projection: `bounded_projection_scene_evaluations`
- 전체: `pipeline_runtime_ms`

함수 호출 수와 함수 내부 후보 수를 분리해 과거 T1의 계측 오류를 반복하지 않는다.

## 8. Fallback 정책

현재 fallback 조건:

- 입력/카메라 K 오류
- LiDAR coverage 부족
- camera edge 부족
- image vanishing direction 부족
- LiDAR Manhattan axis 부족
- Top-K yaw basin 부재 또는 peak z-score 부족
- bounded-search absolute overlap gate 실패
- analyzer 예외

중요한 미비점: 2026-08-18의 3쌍은 B0 basin recall에 실패했는데도 fallback하지 않았다. 따라서 현재 confidence/fallback gate는 과도하게 낙관적이다.

다음 안전 지표를 추가해야 한다.

- `unsafe_non_fallback_rate`: 정답 basin을 누락했는데 fallback하지 않은 비율, 목표 0%
- `selective_recall`: fallback을 허용했을 때 비-fallback 표본의 basin recall
- training pair 간 Top-K consensus 및 covariance overlap
- score entropy, Top-1/Top-2 basin margin, proposal rank stability

fallback 비율 20% 이하만 단독으로 최적화하면 잘못된 확신을 늘릴 수 있다.

## 9. 현재 합격 기준 평가

| 기준 | 현재 결과 | 판정 |
|---|---:|---|
| B0 basin recall@3 ≥99% | 구성 A 0/3, 구성 B 3/3, 전체 3/6 | **FAIL** |
| 잘못된 pruning으로 정답 누락 0건 | 구성 A 3건 누락 | **FAIL** |
| 일반 장면 fallback ≤20% | 0/6 | 표면상 PASS, 그러나 false confidence로 **보류** |
| Analyzer ≤전체 runtime 10% | 1.861/76.166 s = 2.443% | **PASS** |
| projection 평가량 ≥70% 감소 | 168→48, 71.429% | **PASS** |
| 최종 RT 회전 오차 1~2° 이내 | B0 대비 2.394° | **FAIL(근접)** |
| 서로 다른 설치 환경 3개 | 설치 구성 2개 | **PENDING 2/3** |
| textureless degenerate fallback | unit test에서 `CAMERA_EDGE_INSUFFICIENT` | **PASS** |

6개 표본만으로 99% recall을 통계적으로 입증할 수 없다. 현재 수치는 기능 회귀 확인용이며 제품 승인에는 구성별로 더 많은 독립 pair가 필요하다.

## 10. 설치 구성의 정확한 해석

현재 데이터는 완전히 다른 장소 3개가 아니다.

- 구성 A: `data/real_calibration/session-const-env/repeat_test_sample/20260818`
- 구성 B: `data/jenkins-capture/scene0/calib_dataset_build22~24`
- 두 구성의 차이: 동일 계열 장비를 모니터 암과 함께 이동·회전해 camera/LiDAR module pose가 바뀜
- 공통 한계: 같은 설치 장소 계열이며 배경 구조 다양성이 제한됨
- 구성 C: 아직 없음

따라서 문서에서는 “서로 다른 환경 2개”가 아니라 “설치 구성 2개”로 표현한다.

## 11. Lens distortion과 analyzer의 관계

Analyzer가 image/LiDAR 특성을 분석한다고 해서 K·D를 자동으로 해결하는 것은 아니다.

- Camera signature와 vanishing point 모두 K·D 및 rectification 상태에 영향을 받는다.
- 현 V3는 manual ChArUco K·D로 raw image를 undistort한 뒤 calibration하는 제품 전제를 유지한다.
- Lens distortion joint optimization은 별도 observability 문제이며 현재 V3 범위가 아니다.
- “distortion optimization”은 탐색 예산을 좋은 basin에 배분하는 registration 최적화 의미로만 기대할 수 있다.

## 12. 개선 우선순위

1. **Multi-scene analyzer aggregation**: 현재 E2E는 첫 training scene만 analyzer에 넣는다. 각 training pair의 yaw/down score를 median 또는 robust mean으로 결합한다.
2. **Fail-closed confidence 재설계**: pair 간 Top-K 불일치, entropy, margin, covariance overlap을 fallback 조건에 넣는다.
3. **Roll predictor 개선**: vertical line을 구조물/가구에서 분리하고 RANSAC/weighted gravity VP를 사용한다. 현재 잔여 오차는 주로 B0 roll 3°와 V3 roll 0.02° 차이다.
4. **동일 조건 B0/V3 wall-time benchmark**: warm-up 후 최소 5회 median/p95를 기록한다. 현재 B0 전체 runtime 동등 계측이 없어 wall-time 감소율을 단정하지 않는다.
5. **구성 C 수집**: 다른 벽 방향·높이·구조 분포에서 최소 3 training + 1 hold-out pair를 확보한다.
6. **정확도 확정 후 CV5 최적화**: 1D circular correlation과 Sobel reduction을 ARM NEON/SIMD 대상으로 삼는다. recall이 해결되기 전 SIMD tuning은 우선하지 않는다.

## 13. 최종 판단

V3 설계 방향은 B0의 계산량을 줄이는 목적에 부합하고 T2의 가장 큰 비용인 전역 remap을 제거했다. 그러나 현재 가장 중요한 제품 기준인 “정답 basin을 버리지 않는가”가 구성 A에서 실패했다. 따라서 V3는 **성능 최적화 후보**로 유지하고 B0 fallback을 제거해서는 안 된다.

다음 개발의 핵심은 search step을 더 촘촘하게 만드는 것이 아니라, 여러 training scene의 analyzer score를 결합하고 불확실한 장면에서 정확히 fallback하도록 만드는 것이다.

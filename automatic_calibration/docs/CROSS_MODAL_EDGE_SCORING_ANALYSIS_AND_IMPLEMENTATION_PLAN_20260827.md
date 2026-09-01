# 2D–3D 교차 도메인 점수 분석 및 구현 계획

- 작성일: 2026-08-27
- 최종 수정일: 2026-08-27
- 대상 브랜치: `codex/exp-hybrid-analyzer-v3-20260827`
- 적용 범위: CH1, B0/V3 markerless camera–pan/tilt LiDAR extrinsic calibration
- 문서 상태: **분석 완료 / 구현 전 계획**
- 제품 상태: **NOT_PRODUCT_APPROVED_RT**

연관 문서:

- [Targetless LiDAR–Camera Calibration 방법 검토](TARGETLESS_CALIBRATION_METHOD_REVIEW.md)
- [B0·T1·T2 실험 이력과 V3 계획](B0_T1_T2_ANALYZER_EXPERIMENT_HISTORY_AND_V3_PLAN_20260827.md)
- [V3 Hybrid Analyzer 실행 보고서](V3_HYBRID_ANALYZER_EXECUTION_REPORT_20260827.md)
- [V3 다중 장면 합의 보정 보고서](V3_MULTI_SCENE_CONSENSUS_REMEDIATION_REPORT_20260827.md)
- [제품 운용 정책](PRODUCT_CALIBRATION_POLICY.md)

## 1. 최종 결론

현재 오정합의 원인을 단순히 “2D 또는 3D edge 추출이 부족하다”로만 판단하면 안 된다.
더 큰 문제는 **카메라 edge와 LiDAR edge가 같은 물리 구조를 나타내는지 구분하지 않은 채
거리 또는 상관관계만으로 점수화한다는 것**이다.

현재 구현에는 다음 문제가 동시에 존재한다.

1. 카메라 Canny/Sobel에는 벽·책상 경계뿐 아니라 무늬, 문자, ChArUco 내부 패턴,
   모니터 화면, 그림자와 조명 변화가 포함된다.
2. LiDAR 쪽에는 깊이 불연속, normal 변화, 평면 교차선이 존재하지만 일부 경로에서
   `max` 또는 OR 연산으로 하나의 edge로 합쳐진다.
3. 단방향 nearest-edge 비용은 투영된 LiDAR 점 가까이에 **아무 카메라 edge나** 있으면
   낮아질 수 있다. edge의 종류와 방향이 달라도 대응으로 인정될 수 있다.
4. 현재 geometry NID는 LiDAR의 range/normal 변화량과 카메라 gradient magnitude를
   비교한다. 이는 실제 LiDAR 반사 강도와 카메라 명암을 비교하는 Koide/Pandey 계열
   intensity MI/NID와 다른 목적함수다.
5. 여러 항을 하나의 가중 합으로 더하므로, 식별력이 낮은 항의 우연한 낮은 점수가
   구조적으로 틀린 후보를 보상할 수 있다.

따라서 권장 방향은 다음과 같다.

> **V3 analyzer는 후보 영역을 줄이는 coarse proposal generator로 유지하고, fine
> calibration core는 공통 물리 구조에 기반한 channel-separated, orientation-aware,
> visibility-aware 점수로 재설계한다.** B0는 offline oracle과 fail-safe fallback으로
> 유지한다.

이 계획은 SuperGlue, SAM, FCGF 같은 무거운 모델을 제품 경로에 추가하지 않는다.
OpenCV, Eigen, Ceres와 현재 구현된 normal/plane/LSD/z-buffer 코드를 재사용한다.

## 2. 현재 처리 구조와 확인된 사실

### 2.1 현재 실행 흐름

```text
Camera image
  ├─ undistort(K, D)
  ├─ gradient / Canny distance transform
  ├─ LSD structural lines
  └─ vanishing/Manhattan evidence

LiDAR organized scan
  ├─ robust normals / plane segmentation
  ├─ range discontinuity
  ├─ normal crease
  ├─ plane-intersection segments
  └─ optional signal_strength

현재 fine score
  = geometry NID + nearest-edge + structural-line + Manhattan
    + optional signal NMI + prior/coverage

V3 analyzer
  = 축약된 camera signature ↔ 합쳐진 LiDAR panorama edge signature
    → Top-K yaw/down/roll proposal
    → bounded fine search/Ceres
    → 불확실하면 B0 fallback
```

### 2.2 코드 감사 결과

| 영역 | 현재 구현 | 확인된 문제 |
|---|---|---|
| 기본 가중치 | `--nid-weight 0.55`, `--signal-nmi-weight 0.0`, `--edge-weight 0.25`, `--line-weight 0.20`, `--manhattan-weight 0.15` | geometry NID가 가장 큰 기본 항이지만 실제 intensity NID가 아님 |
| Geometry feature | `extractGeometryFeatures()`가 인접 range 변화와 normal 각도 변화를 계산하고 `max(range_score, normal_score)`로 feature를 선별 | 서로 다른 물리 원인의 edge가 한 feature 집합에서 경쟁함 |
| Geometry NID | `normalizedInformationDistance()`가 투영된 range/normal feature와 camera gradient를 spatial tile histogram으로 비교 | 두 값 사이에 안정적인 통계 의존성이 있다는 보장이 없음 |
| Raw edge | `buildCameraEdgeDistanceTransform()`의 Gaussian blur → Canny → distance transform | texture·그림자·마커 내부선도 동일한 edge가 됨 |
| Edge score | `evaluate()`가 각 LiDAR edge point의 nearest camera edge 거리를 평균하고 20 px에서 cap | 단방향 대응, edge 종류·방향 무시, 반복 점 과대계수, 먼 후보 plateau 가능 |
| Structural line | LSD line과 LiDAR 구조 segment를 위치·방향·overlap·normal로 비교 | raw edge보다 낫지만 image line의 LiDAR-visible 여부가 분류되지 않음 |
| V3 panorama | `combinedPanoramaEdge()`가 range/normal/plane-intersection을 bitwise OR | 신뢰도가 다른 채널이 동일한 1 bit edge로 축약됨 |
| V3 signature | camera Sobel과 LiDAR edge를 행/열 1D signature로 축약하고 Pearson correlation | 2D 배치와 edge 종류가 소실되어 반복 구조 alias가 생김 |

관련 코드:

- `automatic_calibration/apps/run_real_calibration.cpp`
- `automatic_calibration/src/calibration_core.cpp`
- `automatic_calibration/src/panorama_raster_builder.cpp`
- `automatic_calibration/src/hybrid_orientation_analyzer.cpp`

## 3. 왜 잘못된 방향도 높은 점수를 받을 수 있는가

### 3.1 단방향 nearest-edge의 허점

현재 raw edge 비용은 대략 다음 질문만 한다.

> “투영된 LiDAR edge 근처에 카메라 edge pixel이 있는가?”

그러나 다음 질문은 하지 않는다.

- 같은 물체 경계인가?
- 두 edge 방향이 평행한가?
- depth discontinuity인가, 두 평면의 교차선인가?
- 카메라의 주요 구조선 중 LiDAR가 설명한 비율은 얼마인가?
- 소수의 긴 벽 경계가 수백 개 점으로 중복 계산되고 있지 않은가?

edge가 많은 실내 영상에서는 틀린 자세에서도 가까운 edge를 쉽게 찾는다. ChArUco 내부
격자처럼 LiDAR에 존재하지 않는 고대비 무늬는 잘못된 후보의 비용까지 낮출 수 있다.

### 3.2 서로 다른 LiDAR edge를 합치는 문제

다음 세 종류는 같은 신뢰도로 취급하면 안 된다.

| LiDAR 채널 | 의미 | 일반적인 신뢰도 |
|---|---|---|
| Plane intersection | 서로 다른 두 안정 평면의 교차선 | 높음 |
| Normal crease | 깊이는 연속이지만 표면 방향이 급변하는 경계 | 중간 |
| Depth discontinuity | 전경 물체 끝과 배경 사이의 range jump | 낮음~중간; beam bleeding/occlusion 영향 큼 |

현재처럼 OR 또는 `max`로 합치면 신뢰도와 오차 모델을 따로 적용할 수 없다. 특히 depth
discontinuity는 빔 폭과 샘플 간격 때문에 실제 물체 윤곽과 측정 edge 위치가 어긋날 수 있다.

### 3.3 현재 geometry NID의 해석 주의

현재 geometry NID는 다음 두 값을 비교한다.

- LiDAR: 인접 range 변화량과 normal 변화량
- Camera: grayscale gradient magnitude

두 값은 모두 “경계가 강해 보이는 정도”이지만 동일한 물리량은 아니다. 밝은 무늬는 큰
camera gradient를 만들지만 LiDAR geometry는 변하지 않을 수 있고, 반대로 색이 같은 벽과
책상의 3D 경계는 camera gradient가 약할 수 있다.

따라서 현재 geometry NID가 낮다는 이유만으로 올바른 RT라고 볼 수 없다. Koide의 NID는
가려진 점을 제거한 뒤 **실제 LiDAR intensity**와 camera intensity의 통계 의존성을 이용한다.
현재 코드의 `signal_nmi_weight` 경로가 그 개념에 더 가깝지만 기본값은 0이고,
TOFSense F2P `signal_strength`의 반복성 검증도 선행되어야 한다.

### 3.4 가중 합 하나로 PASS를 판단하는 문제

현재 composite objective는 여러 항을 더한다. 이 방식은 최적화에는 편리하지만 다음 상황을
허용한다.

```text
틀린 후보:
  geometry NID 우연히 좋음 + raw edge 좋음 + 구조선 나쁨
  → 총합은 통과 가능
```

제품 판정에서는 단순 합 외에 **독립 채널 합의**가 필요하다. 예를 들어 plane-intersection과
normal-crease 중 최소 두 채널이 각각 coverage/margin gate를 통과해야 한다. 식별력이 없는
채널은 0점 보상이 아니라 “관측 불충분”으로 처리해야 한다.

## 4. Targetless 연구에서 실제로 사용하는 대응 방식

| 방법 | 2D–3D 공통 표현과 정합 | 현재 프로젝트에서 취할 점 |
|---|---|---|
| Pandey et al. | camera grayscale와 LiDAR reflectivity 사이 mutual information을 여러 view에서 최대화 | F2P signal이 안정적일 때만 signal NMI 보조항으로 사용 |
| Taylor–Nieto | LiDAR intensity/height/surface-normal feature image와 camera image의 NMI를 전역 탐색 | 여러 feature channel을 분리해 검증하되 무거운 전역 최적화는 피함 |
| Levinson–Thrun | LiDAR depth discontinuity와 image edge distance를 많은 frame에 누적 | 현재 edge score의 근원과 유사. 단일 장면 nearest-edge만으로는 약하므로 다중 관측과 fail-safe 필요 |
| Yuan et al. | voxel plane fitting으로 depth-continuous plane intersection을 만들고 image line과 point-to-line·방향·covariance로 대응 | **가장 직접적으로 채택할 핵심:** 안정 평면 교차선, 방향성, 불확실도 |
| Koide et al. | dense LiDAR intensity image, initial correspondence/RANSAC, z-buffer, intensity NID refinement | z-buffer와 실제 signal NID 개념 채택. SuperGlue는 CV5 제품 경로에서 제외 |
| Multi-FEAT | LiDAR depth/reflectivity/foreground/ground edge와 camera semantic/Sobel edge를 분리하고 precision을 반영 | 채널 분리와 precision/coverage gate 채택. 무거운 semantic model은 제외 |
| Kang–Doh | GMM 기반 many-to-many soft edge correspondence와 coarse-to-fine 영향 범위 | hard nearest pixel 대신 제한 반경 soft-min으로 가볍게 근사 |
| MFCalib | depth-continuous, depth-discontinuous, intensity edge를 분리하고 beam model과 point-to-line residual 사용 | depth edge의 낮은 신뢰도와 beam uncertainty를 명시적으로 반영 |

공통점은 “edge를 많이 뽑아서 모두 같은 방식으로 비교”하지 않는다는 것이다. 실제로는
공통 물리량, edge 종류, 방향, 가시성, 불확실도, 다중 장면 합의를 함께 사용한다.

## 5. 권장 목표 구조

### 5.1 Camera feature channel

```text
C_plane_line   : 긴 LSD/region boundary, Manhattan support가 있는 구조선
C_crease_like  : 긴 연속 edge와 junction
C_detail       : raw Sobel/Canny, 낮은 가중치의 보조 채널
C_mask         : ChArUco 내부 패턴 등 LiDAR 비가시 texture 제외 영역
```

- sharpening/interpolation filter는 기본 경로에 넣지 않는다. 실제 geometry가 없는 edge를
  강화해 false match를 늘릴 수 있다.
- ChArUco는 reference/evaluation 용도다. 자동 RT 후보 점수의 정답 힌트로 사용하지 않는다.
- 다만 marker 내부 texture가 targetless edge score를 오염시키지 않도록 검출된 board 내부를
  mask하는 것은 허용한다. mask 적용 여부와 면적은 결과 JSON에 기록한다.

### 5.2 LiDAR feature channel

```text
L_plane_intersection : 높은 신뢰도
L_normal_crease      : 중간 신뢰도
L_depth_discontinuity: 큰 위치 불확실도를 가진 낮은 신뢰도
L_signal             : conformance 통과 시 독립 NMI 채널
```

각 채널은 별도 point/segment 목록, tangent, confidence, uncertainty를 가진다. OR 또는
`max`로 합치지 않는다.

### 5.3 Orientation-aware soft point-to-line 비용

LiDAR edge point 또는 segment의 투영 위치를 `u_i(T)`, 주변 camera line을
`(q_j, n_j, θ_j)`라고 하면 후보 pair 비용을 다음처럼 구성한다.

```text
r_ij = [n_jᵀ(u_i(T) - q_j)]² / σ_type²
     + λθ · sin²(θ_i - θ_j)
```

- 첫 항: 선에 수직한 pixel 거리
- 둘째 항: edge 방향 차이
- `σ_type`: plane intersection은 작게, depth discontinuity는 크게 설정
- z-buffer: camera에서 가려진 LiDAR point 제외
- robust loss: 일부 잘못된 line/point가 전체를 지배하지 않게 제한

한 점을 하나의 nearest pixel에 강제로 붙이지 않고, 제한 반경 내 line 후보에 soft-min을
적용한다.

```text
C_i = -τ log Σ_j exp(-r_ij / τ)
```

이는 GMM many-to-many 대응을 전체적으로 구현하지 않고도 비슷한 장점을 얻는 경량 근사다.

### 5.4 중복과 edge 밀도 정규화

- 점 개수 평균이 아니라 segment별 평균 후 segment 길이로 제한 가중한다.
- 하나의 벽 경계가 수백 점으로 검출돼도 전체 점수를 독점하지 못하게 한다.
- image를 고정 tile로 나눠 공간 coverage를 계산한다.
- 수직선만 맞고 수평 구조가 전혀 맞지 않는 후보는 coverage gate에서 거절한다.
- camera line 전체를 설명할 필요는 없지만, LiDAR-visible 구조선 대비 explained ratio를
  별도 기록한다.

### 5.5 최종 판정은 점수 합 + 독립 gate

```text
최적화용 J = 채널별 robust objective의 정규화된 합

승격 조건:
  1. high-confidence channel coverage 통과
  2. 최소 2개 독립 channel이 올바른 basin을 지지
  3. best-vs-runner-up margin 통과
  4. multi-scene score curve 합의 통과
  5. hold-out fixed-RT 검증 통과
  6. observability/uncertainty 통과
```

한 채널이 관측 불가능하면 낮은 비용으로 보상하지 않고 `UNOBSERVABLE`로 기록한다.

## 6. V3 analyzer에서 유지할 것과 바꿀 것

### 6.1 유지

- organized JSON → panorama raster
- T1 vanishing-point/Manhattan 보조 추정
- Top-K bounded search
- 모든 training scene의 합의와 fail-closed fallback
- analyzer, 내부 후보, projection-scene runtime 계측
- B0 offline oracle/full-search fallback

### 6.2 변경

현재 V3는 range/normal/plane edge를 하나로 합친 뒤 행·열 1D signature로 축약한다. 이를
다음처럼 변경한다.

```text
기존:
  OR(range, normal, plane) → 1D azimuth/elevation signature → Pearson score

변경:
  channel-separated low-resolution panorama
    × elevation band(예: 8개)
    → channel별 joint S(yaw, down)
    → scene별 score curve
    → circular median/trimmed-mean consensus
    → Top-K basin + covariance
```

전체 perspective remap을 다시 도입하지 않는다. 낮은 해상도의 band/channel signature만으로
후보 basin을 만들고, 실제 투영 평가는 Top-K에만 수행한다.

Analyzer의 역할은 distortion 또는 RT를 직접 최적화하는 것이 아니다. `K,D`는 등록된
camera profile로 고정하고 analyzer는 **가능성 높은 orientation basin을 제안**한다.

## 7. 단계별 구현 계획

각 단계는 이전 단계를 통과해야 시작한다. search step 축소나 SIMD 최적화는 점수 식별력이
확인된 뒤에만 수행한다.

### Phase 0 — Score landscape 감사 계측

목적: 코드를 크게 바꾸기 전에 어느 채널이 정답 RT를 선호하지 않는지 수치로 확정한다.

구현:

- 현재 objective의 모든 항을 candidate별로 독립 출력한다.
- reference/manual RT, 현재 auto RT, analyzer Top-K, yaw/down/roll ±1°/±5°/±10°,
  yaw ±90°/180° 후보를 같은 입력에 평가한다.
- scene별/채널별 score curve, coverage, active tile, matched segment 수를 저장한다.
- 제안 산출물:
  - `score_audit/candidate_channel_scores.csv`
  - `score_audit/score_landscape.json`
  - `score_audit/overlay_<channel>_<candidate>.png`

수정 대상:

- `automatic_calibration/include/auto_calib/calibration_core.hpp`
- `automatic_calibration/src/calibration_core.cpp`
- `automatic_calibration/apps/run_real_calibration.cpp`

종료 조건:

- high-confidence 채널에서 reference RT가 최소한 ±5° 이웃보다 낮은 local minimum이어야 한다.
- 정답을 선호하지 않는 채널은 다음 제품 점수에서 비활성화하거나 재설계 대상으로 확정한다.

### Phase 1 — Plane-intersection 우선 scorer

목적: 가장 신뢰도 높은 3D 구조로 최소한의 새 fine scorer를 만든다.

구현:

- 기존 `extractLidarPlaneIntersectionSegments()` 결과를 재사용한다.
- camera LSD line과 orientation-aware point-to-line 비용을 계산한다.
- z-buffer, segment별 정규화, spatial tile coverage, robust loss를 적용한다.
- old nearest-edge는 삭제하지 않고 비교 진단 항으로 남긴다.

테스트:

- 합성 직교 평면에서 정답 RT local minimum 확인
- yaw/down/roll 각 축 ±10° score curve 단조성/최소점 확인
- 동일 segment 점 밀도를 2배로 늘려도 점수가 크게 바뀌지 않는지 확인
- occluded segment를 추가해도 결과가 유지되는지 확인

관련 테스트 파일:

- `automatic_calibration/tests/calibration_core_tests.cpp`

종료 조건:

- CH1 Case C와 2026-08-18 구성에서 정답/B0 basin이 wrong ±90°/180° basin보다
  일관되게 우수해야 한다.

### Phase 2 — LiDAR 채널 분리와 uncertainty

목적: plane, crease, depth edge가 서로를 오염시키지 않게 한다.

구현:

- `L_plane_intersection`, `L_normal_crease`, `L_depth_discontinuity`를 별도 컨테이너로 유지한다.
- channel별 threshold, tangent, confidence, `σ_type`을 설정한다.
- depth discontinuity에는 scan angular resolution/range 기반 위치 uncertainty를 부여한다.
- 각 채널의 coverage, match ratio, margin을 별도 JSON 필드로 기록한다.

종료 조건:

- 어느 채널이 정답을 지지/반대/관측 불가인지 모든 실데이터 run에서 설명 가능해야 한다.
- 식별력 없는 채널이 전체 PASS를 만들 수 없어야 한다.

### Phase 3 — Camera geometry filtering

목적: LiDAR가 볼 수 없는 image texture가 점수를 지배하지 않게 한다.

구현:

- 긴 LSD/region boundary와 junction을 high-confidence로 분류한다.
- raw Canny/Sobel은 낮은 가중치의 detail 채널로 이동한다.
- 검출된 ChArUco board 내부 패턴은 targetless 점수에서 mask하되 reference 계산에는 유지한다.
- hard-coded “모니터/의자” object detector는 추가하지 않는다. 일반화 가능한 길이,
  연속성, 방향 support, 지역 edge 밀도만 사용한다.

종료 조건:

- ChArUco 유무에 따라 자동 RT basin이 변하지 않아야 한다.
- marker mask 전/후 score 차이는 진단 JSON에 기록되어야 한다.

### Phase 4 — 실제 `signal_strength` NMI ablation

목적: geometry NID와 실제 intensity NID를 혼동하지 않고, 유효할 때만 signal을 사용한다.

선행 conformance:

- 동일 cell 반복 coefficient of variation
- 거리/입사각 변화에 대한 보정 가능성
- saturation/invalid 비율
- scene별 entropy와 200/400 bps rank correlation

구현:

- 거리와 입사각 보정 후 signal image를 생성한다.
- z-buffer와 minimum overlap을 적용한다.
- signal NMI 단독 score landscape와 다른 geometry 채널의 합의를 비교한다.

종료 조건:

- 두 설치 구성에서 정답 basin margin과 반복성이 입증될 때만 낮은 제품 가중치를 허용한다.
- 통과하지 못하면 `signal_nmi_weight=0`을 유지한다.

### Phase 5 — V3 channel-separated joint analyzer

목적: fine scorer가 식별 가능한 basin만 적은 비용으로 제안한다.

구현:

- `combinedPanoramaEdge()`는 호환 진단용으로 유지하고 V3 score에는 개별 raster를 사용한다.
- 8×64 수준의 elevation-band/azimuth signature를 우선 benchmark한다.
- channel별 `S(yaw,down)`과 support를 만들고, 모든 training scene의 circular
  median/trimmed mean으로 합친다.
- Top-K basin, covariance, per-channel support, scene disagreement를 출력한다.
- confidence 부족 또는 scene 불일치 시 즉시 B0 fallback한다.

수정 대상:

- `automatic_calibration/include/auto_calib/panorama_raster_builder.hpp`
- `automatic_calibration/src/panorama_raster_builder.cpp`
- `automatic_calibration/include/auto_calib/hybrid_orientation_analyzer.hpp`
- `automatic_calibration/src/hybrid_orientation_analyzer.cpp`
- `automatic_calibration/tests/hybrid_orientation_analyzer_tests.cpp`

종료 조건:

- current dataset에서 correct basin recall@3 100%
- wrong basin의 unsafe non-fallback 0건
- analyzer가 pipeline runtime의 10% 이하
- B0 대비 실제 projection-scene 평가 70% 이상 감소

### Phase 6 — Multi-scene 제품 gate와 성능 최적화

목적: 좋은 한 장이 아니라 설치 상태의 반복 가능한 RT를 승인한다.

구현:

- training은 shared RT를 추정하고 sealed hold-out에는 RT를 고정 적용한다.
- 채널별 median/trimmed objective, finalist margin, RT covariance/observability를 판정한다.
- false acceptance가 측정되기 전까지 기존 2% margin을 제품 상수로 확정하지 않는다.
- 정확도 합격 후 ARM NEON/SIMD는 signature correlation과 projection/NID hot loop에만 적용한다.

종료 조건:

- 서로 다른 설치 환경 최소 3개
- 각 환경 최소 3 training + 1 sealed hold-out pair
- reference 대비 회전 1~2° 이내와 프로젝트에서 확정한 translation 허용치 통과
- fallback 비율 20% 이하, false activation 0건
- bounded 경로와 fallback 경로의 median/p95 runtime 기록

## 8. 데이터 및 검증 역할

### 8.1 현재 사용할 데이터

| 구성 | 경로/역할 | 주의 |
|---|---|---|
| 2026-08-18 | `data/real_calibration/session-const-env/repeat_test_sample/20260818` | 고정 설치 반복 관측, training/hold-out 분리 |
| 2026-08-19 | `data/real_calibration/session-const-env/repeat_test_sample/20260819` | 모듈 이동·회전 후 독립 설치 구성 진단 |
| Jenkins Case A/B/C | `data/jenkins-capture/scene0` build5~24 | Case C build22/23 training, build24 hold-out을 primary로 유지 |

Manual/ChArUco RT는 runtime 후보나 search prior로 넣지 않는다. 오직 다음 목적으로 사용한다.

- score landscape의 정답 근처 형상 확인
- auto RT 오차 산출
- false acceptance/false rejection 측정
- threshold를 고정한 뒤 sealed hold-out 평가

### 8.2 아직 부족한 데이터

현재 독립 설치 구성은 실질적으로 2개다. 같은 장소에서 모니터 암을 움직인 데이터는 중요한
재현성 시험이지만 서로 다른 구조 분포 3개를 충족하지 않는다. 제품 승인 전에는 다음 조건의
구성 C가 필요하다.

- 벽 방향·센서 높이·가구 구조가 기존과 다른 장면
- 동일 K/D profile과 좌표 계약
- 최소 3 training + 1 sealed hold-out pair
- threshold 조정 없이 blind 실행

## 9. 합격 기준과 증거 파일

| 항목 | 개발 단계 기준 | 제품 승격 전 기준 |
|---|---|---|
| Reference local minimum | 모든 high-confidence 채널 | 유지 |
| Correct basin recall@3 | 현재 데이터 100% | 충분한 표본에서 ≥99% 목표 |
| Wrong basin unsafe activation | 0건 | 0건 |
| Analyzer runtime | pipeline의 ≤10% | median/p95 모두 확인 |
| Projection 평가 감소 | B0 대비 ≥70% | 유지 |
| 회전 오차 | B0/manual reference 대비 1~2° | 독립 reference로 확인 |
| Translation 오차 | 기존 reference와 함께 기록 | 장치 요구사항으로 수치 확정 필요 |
| Fallback | 허용 | 일반 장면 ≤20% |
| 설치 다양성 | 현재 2개 | 최소 3개 |

각 E2E run은 최소 다음 증거를 남겨야 한다.

- channel별 feature 수, visible 수, matched 수와 coverage
- candidate별 raw/normalized score
- best/runner-up margin과 basin covariance
- scene별 score curve와 aggregate curve
- z-buffer 적용 전/후 point 수
- analyzer/fine/Ceres/fallback runtime
- fallback 및 fail-closed reason
- 2D overlay와 3D colorized preview

## 10. 구현 파일 전략

새 framework나 dependency를 추가하지 않는다. 우선 기존 파일에서 가장 작은 변경으로
검증한다.

```text
automatic_calibration/include/auto_calib/calibration_core.hpp
automatic_calibration/src/calibration_core.cpp
automatic_calibration/include/auto_calib/panorama_raster_builder.hpp
automatic_calibration/src/panorama_raster_builder.cpp
automatic_calibration/include/auto_calib/hybrid_orientation_analyzer.hpp
automatic_calibration/src/hybrid_orientation_analyzer.cpp
automatic_calibration/apps/run_real_calibration.cpp
automatic_calibration/tests/calibration_core_tests.cpp
automatic_calibration/tests/hybrid_orientation_analyzer_tests.cpp
```

권장 브랜치 전략:

1. 현재 V3 코드와 evidence를 동결한다.
2. V3에서 `exp-cross-modal-score` 실험 브랜치를 분기한다.
3. Phase 0~5를 각각 독립 커밋으로 유지한다.
4. 각 단계는 B0/V3 A/B 비교 결과가 없으면 다음 단계로 진행하지 않는다.
5. 작은 JSON/CSV/log만 evidence로 커밋하고 대형 PLY/OBJ/PNG는 generated 경로에 둔다.

구현 도중 header/CPP 분리가 꼭 필요해질 때만 scorer 파일을 하나 추출한다. 시작부터 추상화
계층이나 plugin 구조를 만들지 않는다.

## 11. 중단 조건과 위험 관리

다음 중 하나가 발생하면 search 범위를 줄이지 말고 B0 fallback을 유지한다.

- high-confidence 채널에서 reference RT가 local minimum이 아님
- training scene마다 Top-K basin이 다름
- two-channel agreement가 없음
- active spatial tile 또는 visible segment가 부족함
- best/runner-up margin이 hold-out에서 재현되지 않음
- K/D profile 또는 LDC/zoom/focus 상태가 불명확함

예상 위험:

| 위험 | 대응 |
|---|---|
| plane intersection이 적은 장면 | normal crease를 보조하고 관측 불충분이면 fallback |
| depth edge beam bias | 큰 covariance와 낮은 weight, 안정 segment만 사용 |
| Manhattan 반복 구조 | 다중 scene 합의와 ±90°/180° negative fixture |
| ChArUco texture 오염 | board 내부 mask, reference와 score 경로 분리 |
| signal_strength 불안정 | conformance 실패 시 weight 0 유지 |
| fine score 개선 없이 analyzer만 강화 | Phase 0/1 통과 전 V3 후보 축소 금지 |

## 12. 범위 밖

이번 계획에서 다루지 않는다.

- K/D와 RT 공동 추정
- 제조사 profile을 대체하는 targetless intrinsic 추정
- JSON/PCD 좌표 계약 변경
- Qt Top-view GUI
- CH2~CH4 제품 확장
- SuperPoint/SuperGlue/SAM/FCGF 기반 제품 inference
- current dataset만으로 `PRODUCT_APPROVED_RT` 선언

## 13. 최종 의견

구현은 진행할 가치가 있다. 다만 목표를 “analyzer가 정답 RT를 직접 찾는다”로 잡으면 다시
동일한 문제가 생긴다. analyzer는 넓은 360° 탐색에서 가능성 낮은 영역을 제거하는 역할이고,
최종 정확도는 **교차 도메인에서 실제로 공통인 구조를 구분해 비교하는 fine score**가
결정해야 한다.

가장 먼저 할 일은 Phase 0 score audit이다. 여기서 manual/reference RT가 현재 각 점수의
local minimum인지 확인하지 않은 채 5°를 1°로 줄이거나 Ceres 반복을 늘리면, 더 빠르고
정밀하게 틀린 minimum으로 수렴할 뿐이다.

우선 구현 대상은 다음 세 가지다.

1. plane-intersection 중심 orientation-aware point-to-line scorer
2. range/normal/plane edge channel 분리와 채널별 관측성 gate
3. V3의 channel-separated, elevation-banded multi-scene proposal

이 세 가지가 통과한 뒤에만 signal NMI와 ARM NEON 최적화를 추가한다. 이 순서가 CV5
2-core/4 GB 환경에서 정확도와 연산량을 함께 개선할 가능성이 가장 높다.

## 14. 참고 문헌 및 구현체

1. Pandey et al., “Automatic Targetless Extrinsic Calibration of a 3D Lidar and Camera by Maximizing Mutual Information,” AAAI 2012.  
   https://ojs.aaai.org/index.php/AAAI/article/view/8379
2. Taylor and Nieto, “Automatic Calibration of Lidar and Camera Images using Normalized Mutual Information,” ICRA 2013.  
   https://www-personal.acfr.usyd.edu.au/jnieto/Publications_files/TaylorICRA2013.pdf
3. Levinson and Thrun, “Automatic Online Calibration of Cameras and Lasers,” RSS 2013.  
   https://www.roboticsproceedings.org/rss09/p29.pdf
4. Yuan et al., “Pixel-Level Extrinsic Self Calibration of High Resolution LiDAR and Camera in Targetless Environments,” 2021.  
   https://arxiv.org/pdf/2103.01627
5. Koide et al., “General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox,” ICRA 2023.  
   https://staff.aist.go.jp/shuji.oishi/assets/papers/preprint/Calibration_ICRA2023.pdf  
   https://github.com/koide3/direct_visual_lidar_calibration
6. Multi-FEAT, multi-feature edge alignment for targetless LiDAR-camera calibration.  
   https://arxiv.org/pdf/2207.07228
7. Kang and Doh, soft/GMM edge correspondence 기반 targetless calibration.  
   https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21893
8. MFCalib, multi-feature targetless LiDAR-camera calibration.  
   https://arxiv.org/abs/2409.00992
9. Targetless 방법 모음.  
   https://github.com/Deephome/Awesome-LiDAR-Camera-Calibration

## 15. 수정 이력

| 버전 | 날짜 | 변경 내용 |
|---|---|---|
| 0.1 | 2026-08-27 | 현재 edge/NID/analyzer 점수 코드 감사, targetless 문헌 비교, channel-separated fine score 및 V3 구현 계획 최초 작성 |

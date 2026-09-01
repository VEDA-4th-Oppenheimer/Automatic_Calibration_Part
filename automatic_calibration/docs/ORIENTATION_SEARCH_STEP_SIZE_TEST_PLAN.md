# Yaw–Roll(Coarse Down) 탐색 간격 및 인접 후보 보정 시험 계획

작성일: 2026-08-12
상태: staged 구현 반영 / full-search 선행 게이트 후 benchmark 계획
대상: Automatic Calibration Core 초기 방향 탐색

## 1. 목적

후속 1° fine search를 전제로, 정답 orientation basin을 놓치지 않으면서 coarse
후보 수와 실행시간을 최소화하는 yaw/roll 간격을 선정한다. 후보 하나의 raw
objective만으로 순위를 정하지 않고 실제 각도 공간에서 인접 후보들의 점수를 반영한
보정 점수와 contiguous orientation basin을 사용한다.

## 2. 용어와 범위

- yaw: LiDAR `+Y`(pan 회전축) 기준 0~360° 외부 회전 후보
- roll/down: 현재 실행기의 `UnitX` 회전, 카메라 optical axis의 down 방향 0~90° 후보
- image roll: 카메라 영상면 자체 회전이며 이번 시험에서 제외
- raw objective `J`: NID 70% + edge distance 30% 복합 목적함수, 낮을수록 우수
- basin: yaw–down 격자에서 보정 점수가 연속적으로 우수한 연결 후보 영역

장치 tilt 범위와 카메라 외부 방향 후보는 같은 변수가 아니다. 130333 JSON의
`tilt_zero=nadir` 계약이 해결되기 전까지 해당 실데이터는 계산량과 후보 안정성
시험에만 사용하고 정확도 정답으로 사용하지 않는다.

## 3. 시험 질문

1. 어떤 yaw/down coarse 간격이 1° 기준 정답 basin을 상위 3개 안에 유지하는가?
2. 후보 평가 수와 실행시간 절감률은 얼마인가?
3. 인접 후보 보정이 고립된 false minimum 선택을 줄이는가?
4. 넓지만 평범한 plateau가 실제 정답의 좁은 peak를 부당하게 이기는가?
5. 대칭 장면의 서로 다른 방향을 `AMBIGUOUS`로 유지하는가?

## 4. 기준 탐색과 정답 정의

대표 합성·reference case는 yaw 0~359°, down 0~90°를 모두 1°로 평가해 총
32,760개 orientation의 raw objective와 overlap을 저장한다.

### 4.1 선행 게이트: 1° full search 정상 통과

본 문서의 coarse step 비교, 인접 후보 보정 튜닝 및 1° fine search 시험은
`1°×1° full search` 기준 시험이 정상 통과된 이후에만 진행한다. Full search가
실패하면 coarse 간격을 평가하지 않고 좌표계, objective, overlap 및 데이터 품질
문제를 먼저 해결한다. 실패한 full search를 기준으로 coarse 간격을 고르면 더 적은
계산량으로 동일한 오답을 재현할 위험이 있기 때문이다.

Full-search 선행 게이트의 필수 통과 조건은 다음과 같다.

- 합성 positive: ground-truth orientation basin이 유효 후보로 검출됨
- reference case: 독립 기준 RT basin이 상위 후보군에 포함됨
- ground-truth/reference가 있는 case의 최종 orientation error ≤ 2°
- projected ratio와 NID projected count가 현재 입력 품질 gate를 통과함
- 대칭·평면 negative case는 임의 PASS가 아니라 `ORIENTATION_BASINS_AMBIGUOUS`
  또는 해당 품질 실패 사유로 거절됨
- 동일 입력 10회에서 raw score map, basin ID와 최종 판정이 동일함
- `tilt_zero`, angle unit, 회전축 및 yaw wrap-around 계약 검증 완료

선행 게이트 결과는 `FULL_SEARCH_BASELINE_PASS` 또는
`FULL_SEARCH_BASELINE_FAIL`로 기록한다. 하나 이상의 필수 positive/reference case가
실패하면 전체 step-size benchmark 상태를 `BLOCKED_BY_FULL_SEARCH_BASELINE`으로
종료한다. 130333 CH1~CH4처럼 ground truth가 없는 단일 관측은 이 선행 게이트의
정확도 통과 근거로 사용할 수 없다.

1. 입력 품질/overlap gate를 통과한 후보만 유지한다.
2. 동일한 인접 후보 보정을 적용한다.
3. 연결 성분을 orientation basin으로 생성한다.
4. 합성 case는 ground-truth RT가 포함된 basin을 정답으로 한다.
5. 실데이터는 독립 reference RT가 투영되는 basin을 정답으로 한다.

정답 RT가 없는 130333 단일 관측은 정답률 계산에서 제외하고 채널별 후보 안정성,
반복 실행 결정성 및 계산량만 보고한다.

## 5. Coarse step 시험 행렬

90°와 360°를 균등 분할하는 간격을 우선 사용해 마지막 cell의 경계 편향을 막는다.

- yaw step: 5°, 10°, 15°, 20°, 30°, 45°
- down step: 5°, 10°, 15°, 30°
- 전체 조합: 24개
- 후보 수: `360/yaw_step × (90/down_step + 1)`

| yaw/down | 5° | 10° | 15° | 30° |
|---:|---:|---:|---:|---:|
| 5° | 1,368 | 720 | 504 | 288 |
| 10° | 684 | 360 | 252 | 144 |
| 15° | 456 | 240 | 168 | 96 |
| 20° | 342 | 180 | 126 | 72 |
| 30° | 228 | 120 | 84 | 48 |
| 45° | 152 | 80 | 56 | 32 |

현재 15°×15°의 168개와 제안된 5°×5°의 1,368개를 모두 포함한다.

## 6. 인접 후보 보정 점수

서로 다른 step을 공정하게 비교하기 위해 “한 칸 이웃” 대신 실제 각도 거리로
Gaussian weight를 계산한다. yaw는 0°와 360°를 연결한 주기 경계로 처리한다.

```text
dyaw(i,j)  = min(|yaw_i-yaw_j|, 360-|yaw_i-yaw_j|)
ddown(i,j) = |down_i-down_j|
w(i,j)     = exp(-0.5*((dyaw/sigma_yaw)^2+(ddown/sigma_down)^2))
```

raw objective는 case 내부 유효 후보에 대해 robust normalization한다.

```text
Z_i = (J_i - median(J_valid)) / max(IQR(J_valid), epsilon)
N_i = sum(w(i,j)*confidence_j*Z_j) / sum(w(i,j)*confidence_j)
C_i = alpha*Z_i + (1-alpha)*N_i
```

- `C_i`가 낮을수록 우수하다.
- `confidence`는 projected ratio와 NID projected count로 구성한다.
- gate를 통과하지 못한 후보는 이웃 평균에서 제외한다.
- down 0°/90° 경계는 존재하는 이웃만 사용하고 weight 합으로 재정규화한다.
- `sigma`는 dense reference score map의 autocorrelation 길이로 정하고 모든 step에서
  동일한 물리 각도를 사용한다.

초기 비교값:

- `alpha`: 1.0(raw baseline), 0.75, 0.50, 0.25
- `sigma_yaw`: 5°, 10°, 15°
- `sigma_down`: 5°, 10°, 15°

넓은 mediocre plateau 편향을 막기 위해 basin은 다음 raw eligibility도 만족해야 한다.

```text
basin_raw_best <= global_raw_best + delta_raw
```

`delta_raw`는 1%, 2%, 5%를 비교한다. 따라서 인접 보정은 명백히 나쁜 raw 후보를
승격시키지 않고 raw 경쟁력이 있는 후보 사이에서 연속 지지도를 반영한다.

## 7. Contiguous basin 생성과 선택

1. 보정 점수 `C`가 `C_best + epsilon_basin` 이내인 후보를 활성화한다.
2. yaw wrap-around를 포함한 8-neighbor 연결 성분을 구한다.
3. basin별 raw/corrected best, 면적, angular spread, overlap 평균을 기록한다.
4. 같은 basin의 여러 cell 대신 대표 후보 하나만 fine search seed로 사용한다.
5. 상위 `K=3` basin을 기본 유지하고 K=1/3/5를 비교한다.
6. 멀리 떨어진 basin 점수 차이가 작으면 모두 보존하고
   `ORIENTATION_BASINS_AMBIGUOUS`로 유지한다.

## 8. 1° Fine search

각 coarse basin의 angular bounding box를 한 coarse cell만큼 확장해 1°로 평가한다.
yaw는 wrap-around하고 down은 0~90°로 제한한다. 넓은 basin은 fine 후보 수 상한을
두고, 초과 시 서로 떨어진 local minimum 최대 3개로 분할한다.

Fine 결과 중 최종 winner 하나에만 Ceres refinement를 수행한다. Coarse의 모든
orientation이나 각 basin마다 Ceres를 실행하지 않는다. 상위 basin별 Ceres 비교는
별도 연구 benchmark에서만 허용한다.

현재 실행기의 `--search-strategy staged`는 coarse → top-3 → 5° → 1° → 단일 Ceres
경로를 사용하며, `calibration_result.json`의 `search_stages`와
`ceres_execution_policy`로 실제 실행 여부를 기록한다.

## 9. 시험 데이터

### 9.1 합성 positive

- ground-truth yaw/down을 격자 정중앙과 cell 경계에 배치
- yaw 0°/359° wrap-around, down 0°/90° 경계
- translation offset, range noise, dropout 조합
- 구조가 풍부한 비대칭 다중 평면 장면

### 9.2 합성 negative/degenerate

- 평평한 천장·바닥, 반복 복도, 대칭 벽
- 낮은 overlap, edge 부족
- 잘못된 tilt sign/zero metadata

### 9.3 Stanford 2D-3D-S

- office, hallway, lobby, storage 등 구조 다양성별 case
- known pose로 합성한 organized pan–tilt scan
- 동일 scene의 noise/dropout seed 반복

### 9.4 실제 고정환경

- 130333 CH1~CH4: 진단 안정성 및 계산량 전용
- 향후 동일 설치의 채널별 5개 이상 구조 관측: 정확도/반복성 평가
- 독립 reference RT 또는 hold-out이 없으면 정확도 PASS에 포함하지 않음

## 10. 실행 단계

### Phase A — Dense reference 생성

- 대표 case에 1°×1° score map 생성
- raw objective, overlap, corrected score, basin ID를 CSV/JSON으로 저장
- 2D heatmap과 basin label 이미지 생성
- 선행 게이트 통과 여부를 판정하고 `full_search_baseline_result.json`에 기록
- `FULL_SEARCH_BASELINE_FAIL`이면 Phase B~E를 실행하지 않고 원인 분석으로 전환

### Phase B — Step size 단독 비교

- Phase A의 `FULL_SEARCH_BASELINE_PASS`를 입력 조건으로 확인
- `alpha=1.0`으로 24개 step 조합 실행
- coarse sampling으로 정답 basin이 사라지는 간격 식별
- 통과 조합만 Phase C로 전달

### Phase C — 인접 보정 튜닝

- 통과 step에 alpha/sigma/delta 조합 적용
- false minimum 감소와 plateau 오선택을 함께 측정
- raw baseline보다 나쁜 조합 제외

### Phase D — Basin + 1° fine + Ceres

- top K basin fine search
- 최종 orientation/RT 오차와 총 계산량 측정
- ambiguity 유지 여부 검증

### Phase E — 실제 데이터 안정성

- CH1~CH4 및 반복 취득의 후보 basin 재현성 평가
- 좌표계 계약 확정 전에는 정확도 PASS 생성 금지

## 11. 평가 지표

| 지표 | 의미 |
|---|---|
| basin recall@K | 정답 basin이 coarse 상위 K개에 포함된 비율 |
| fine orientation error | 1° fine 후 ground truth/reference와 각도 오차 |
| objective regret | dense 1° 최저점 대비 최종 objective 증가율 |
| false-basin rate | 정답과 분리된 basin을 단독 1위로 선택한 비율 |
| ambiguity recall | 대칭 case를 모호성으로 유지한 비율 |
| candidate evaluations | coarse + fine 총 점수 평가 횟수 |
| Ceres runs | case당 연속 최적화 횟수 |
| runtime / peak RSS | 동일 Docker CPU 조건 wall time과 최대 메모리 |
| stability | noise seed/반복 취득 간 basin 일치율과 각도 표준편차 |

## 12. 선정 기준

먼저 1° full-search 선행 게이트가 통과되어야 한다. 이후 다음 조건을 만족하는 coarse
조합 중 후보 평가 수와 wall time이 가장 작은 조합을 선정한다.

- 합성 positive basin recall@3 ≥ 99%
- Stanford/reference basin recall@3 ≥ 95%
- 1° fine 후 orientation error ≤ 2°
- dense reference 대비 objective regret ≤ 1%
- raw baseline보다 false-basin rate 악화 없음
- degenerate ambiguity recall ≥ 95%
- 동일 입력 10회에서 basin/RT 결정성 100%

정확도 차이가 통계적으로 유의하지 않으면 더 큰 step을 선택한다. 결과는
Pareto curve(정확도 대 후보 수/시간)로 함께 보고한다.

## 13. 권장 초기안과 개선안

최종 탐색 정책은 B 방식으로 확정한다.

1. yaw 0~359°와 roll/down 0~90° 전체 후보의 raw score를 계산
2. 인접 8개 후보 가중치를 반영한 corrected score 계산
3. corrected score가 좋은 contiguous basin을 추출
4. 상위 3개 basin을 유지
5. 각 basin에서 1° fine search 수행
6. fine 결과에 대해서만 Ceres refinement를 수행
7. basin 간 최종 score, overlap, ambiguity를 비교

step-size benchmark에서는 10°×10°와 15°×15°를 주 후보로 비교하고,
5°×5°는 reference 상한선으로 사용한다. `alpha=0.5~0.75`를 초기 후보로 두며,
sigma는 dense map의 correlation 길이로 결정한다.

더 효율적인 후속 방법은 고정 격자보다 score-map pyramid 또는 adaptive
subdivision이다. 15° 또는 10°에서 유망 basin만 5°, 이후 1°로 분할하면 5°
전수조사보다 적은 후보로 같은 recall을 달성할 수 있다. 먼저 고정 step을 baseline으로
측정한 뒤 adaptive 방식을 같은 지표로 비교한다.

## 14. 산출물

```text
generated/orientation_step_benchmark/<run_id>/
  configuration.json
  full_search_baseline_result.json
  case_summary.csv
  aggregate_summary.csv
  score_maps/<case>/<step>/{raw,corrected}.csv
  heatmaps/<case>/<step>_{raw,corrected}.png
  basins/<case>/<step>.json
  fine_search/<case>/<basin>.json
  pareto_accuracy_vs_evaluations.png
  report.md
```

Git revision, Docker image digest, CPU/thread 수, 입력 checksum과 실행 시간을 기록한다.

## 15. Jenkins 운영

- PR: 소형 합성 1° full-search baseline gate와 결정성 회귀 확인
- Daily: baseline gate PASS 후에만 후보 step, alpha/sigma, curated Stanford 실행
- Weekly: baseline gate PASS 후에만 24개 step 행렬과 전체 reference bundle 실행
- Release: 고정 dense-reference golden checksum과 비교

## 16. 구현 전 확인사항

- 본 문서의 roll/down이 요구한 roll 축과 같은지 확인해야 한다. 영상면 roll까지
  포함하면 별도 3차원 orientation search가 필요하다.
- `tilt_zero` 계약을 확정해야 실데이터 정확도 평가가 가능하다.
- top basin 수와 fine 영역 상한은 실제 실행시간 예산에 맞춰 확정한다.

## 17. 수정 이력

| 버전 | 날짜 | 변경 내용 |
|---|---|---|
| 0.1 | 2026-08-12 | Coarse 간격, 인접 Gaussian 보정, contiguous basin, 1° fine search 및 선정 기준 최초 작성 |
| 0.2 | 2026-08-12 | 1°×1° full-search 정상 통과를 coarse 간격·인접 가중치 시험의 필수 선행 게이트로 추가 |
| 0.3 | 2026-08-12 | 130333 CH1~CH4 1° full-search 진단 실행 결과와 `BLOCKED_BY_REFERENCE_UNAVAILABLE` 판정 기록 |

## 18. 130333 1° Full-search 실행 기록 (2026-08-12)

입력: `data/real_calibration/session-const-env/2026-08-11/130333`
범위: 채널별 yaw 360개 × down 91개 = 32,760 orientation
실행 조건: yaw/down step 1°, 당시 실험의 제조사 FOV 기반 K 고정, 단일 관측 진단 모드

> 이 절은 2026-08-12 과거 진단 기록이다. 현재 MVP 제품 경로는 같은 profile의 Manual
> ChArUco `K+D`를 고정한다. 제조사 FOV K와 K+RT 공동 추정은 연구·민감도 진단으로만 남긴다.

아래 표와 결과 JSON의 방향 필드는 `down_deg`와 `yaw_deg`를 분리해 기록하며,
튜플로 표시할 때는 `(down_deg, yaw_deg)` 순서를 사용한다.

| 채널 | Raw best down | Raw best yaw | Raw objective | 후보 gate |
|---|---:|---:|---:|---|
| CH1 | 15° | -42° | 0.771665 | `NID_OVERLAP_INSUFFICIENT` |
| CH2 | 12° | -43° | 0.820534 | `NID_OVERLAP_INSUFFICIENT` |
| CH3 | 2° | -36° | 0.771759 | `NID_OVERLAP_INSUFFICIENT` |
| CH4 | 3° | -43° | 0.788944 | `NID_OVERLAP_INSUFFICIENT` |

모든 채널에서 CSV 헤더를 제외한 32,760개 raw score가 생성됐다. 그러나 채널별
image–LiDAR 관측이 한 쌍이고 ground-truth/reference RT가 없으며, 내부 후보도 NID
overlap gate를 통과하지 못했다. 따라서 상태는
`FULL_SEARCH_BASELINE_DIAGNOSTIC_ONLY / BLOCKED_BY_REFERENCE_UNAVAILABLE`이다.
본 계획의 선행 조건에 따라 24개 coarse step 행렬, 인접 후보 가중치 튜닝 및 1° fine
search 비교는 실행하지 않았다.

산출물:

```text
automatic_calibration/generated/real_session_const_20260811_full_search_1deg/
  ch1..ch4/
    orientation_full_search.csv
    full_search_baseline_result.json
    calibration_result.json
    matching_scene_0.png
    scene_0_colorized_lidar*.{ply,obj,png}
```




### 18.2 LDC unknown and Gaussian weighting test (2026-08-12)

LDC was not found in the camera web settings, so the default is unknown. Allowed values are true, false, and unknown; SSDR is not treated as LDC. Corrected scores use Gaussian distance weighting over valid 8-neighbors with sigma_yaw=5 deg, sigma_down=5 deg, and alpha=0.5; yaw wrap-around is circular.

CH1 1-degree full-search recorded ldc_enabled=unknown, selected basin down=15 deg and yaw=-42 deg, basin size 2, and SINGLE_OBSERVATION_DIAGNOSTIC_ONLY. CTest 5 of 5 passed.

The implementation still runs Ceres for every candidate before basin selection. A fully compute-saving coarse to top-K basin to 1-degree fine to Ceres split remains follow-up work.


### 18.3 CH1 좌표 변환 비교 (폐기된 진단 실험, 2026-08-12)

고정환경 130333 CH1에 대해 `current`, `tilt_sign_flip`, `nadir_reference` 세
케이스를 비교했다. 이 실험은 `mechanism.tilt_zero`를 계약각으로 오해한 상태에서
수행되었으므로 제품 판단에 사용하지 않는다. 현재 실행 경로에서는 세 variant와
override를 제거했으며 JSON `frame`과 `measurements[].tilt_rad`만 사용한다.

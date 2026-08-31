# V3 Analyzer Experiment Closeout

- 작성일: 2026-08-31
- 기준 R6-R2: `e8cfa77` — `Add R6-R2 directional proposal shadow audit`
- 기준 R7: `0244578` — `Add V3-R7 multi-capture consensus audit`
- 범위: B0, T1, T2, V3 Hybrid 및 R1~R7 analyzer 실험의 종합 정리
- 문서 성격: 실험 closeout 및 제품 경로 결정 기록

> 이 문서는 실험 결과를 하나의 제품 알고리즘으로 합치는 문서가 아니다. 각
> 브랜치의 코드·점수·실패 증거를 보존하고, 제품 경로와 연구 경로를 분리하기
> 위한 동결 기록이다.

## 0. 최종 상태

```text
GLOBAL_TARGETLESS_INITIALIZATION=NOT_PRODUCT_READY
ANALYZER_EXPERIMENT=FROZEN
MANUAL_REFERENCE=PROVISIONAL_ENGINEERING_NOMINAL
RECOMMENDED_PRODUCT_PATH=REFERENCE_ANCHORED_LOCAL_VERIFICATION
PRODUCT_RT_PROMOTED=false
```

추가 안전 상태:

```text
MANUAL_REFERENCE_GROUND_TRUTH=false
PRODUCT_PASS=false
PHASE0C=UNVALIDATED_HOLD
PHASE1=HOLD
```

`MANUAL_REFERENCE=PROVISIONAL_ENGINEERING_NOMINAL`은 현재 실험에서 비교할
수 있는 nominal engineering anchor라는 뜻이다. 독립 측량 ground truth,
제품 승인 RT, 자동 캘리브레이션의 정답으로 승격하지 않는다.

## 1. 프로젝트 목적과 공통 계약

목표는 설치된 camera와 1D LiDAR의 외부 파라미터를 추정하는 것이다.

```text
p_camera = R_camera_lidar * p_lidar + t_camera_lidar
```

현재 실험의 공통 물리 계약은 다음과 같다.

- camera profile의 `K/D`는 고정 입력이다. V3 실험에서 K/D 공동 추정은 하지 않았다.
- 현재 진단 입력은 manual ChArUco에서 유도한 clean18 profile을 사용한다.
- camera center 계약은 `C_lidar=(0,-0.08105,0) m`이다.
- explicit center 경로에서 `t=-R*C_lidar`를 유지한다.
- `range_offset_m=0.084`는 LiDAR 점 생성에 한 번만 적용한다.
- Manual Reference는 자동 입력, prior, analyzer, fallback에 주입하지 않고 사후 평가에만 사용한다.
- B0와 Manual Reference 모두 독립 ground truth가 아니다.

따라서 “reference에 가까운가”라는 모든 수치는 현재 provisional anchor에 대한
조건부 진단 수치이며, 제품 정확도·절대 오차·PASS를 뜻하지 않는다.

## 2. B0, T1, T2, V3 Hybrid의 목적과 차이

### 2.1 B0 — full-search baseline

B0는 analyzer 없이 넓은 orientation 후보를 기존 production objective로 직접
평가하고, bounded/fine search와 Ceres 및 품질 gate를 수행하는 기준 경로다.

역할:

- analyzer가 정답 basin을 누락했는지 비교하는 offline baseline/oracle
- analyzer 신뢰도가 낮을 때 사용할 수 있는 fail-safe fallback 후보
- 기존 production path의 회귀 비교 기준

한계:

- 후보·장면별 3D projection 평가가 많아 CV5 2-core/4 GB 환경에서 부담이 크다.
- 반복적인 벽·바닥·가구 장면에서는 B0도 여러 유사 basin을 만들 수 있다.
- B0 선택 결과도 독립 ground truth가 아니다.

### 2.2 T1 — structural analyzer

T1은 다음 구조 정보를 사용해 방향 basin을 줄이는 가설이다.

1. camera image에서 LSD line segment 추출
2. line 방향/교점으로 vanishing direction 추정
3. organized LiDAR에서 surface normal과 Manhattan 축 추정
4. camera 축과 LiDAR 축의 signed permutation 후보 생성
5. Top-K 구조 후보를 bounded calibration으로 전달

장점은 설명 가능성과 낮은 분석 비용이다. 그러나 실제 장면의 수직선이
벽·기둥이 아니라 책상·가구·장애물에서 나오거나, 수직 구조선이 부족하면
gravity/Manhattan 축을 잘못 선택한다. 기록된 T1 결과는
`yaw=176.4°`, `down=21.77°`, `roll=-4.46°`였지만 단일 결과가 구조축의
재현성을 증명하지는 못했다.

### 2.3 T2 — panorama analyzer

T2는 JSON의 row/column 구조를 LiDAR panorama로 복원한 후, camera 특징과
panorama 구조를 비교해 orientation 후보를 만드는 가설이다.

1. JSON row/column을 400×101 LiDAR panorama로 복원
2. range/normal/validity 특징 생성
3. yaw/down 후보별 panorama 비교
4. Top-K 후보 생성
5. bounded search 및 후단 calibration 수행

standalone에서는 build22/23/24/17의 rank-1 yaw가 각각
`177.5°/170°/177.5°/172.5°`로 B0 basin 부근에 형성됐다. 그러나 전역
perspective remap을 약 216회 수행해 analyzer runtime이 `7.55 s`였고,
모호성도 `distinctive=false`, margin `-0.034`, passing competitor `1`로
fail-closed되었다. 즉 panorama를 사용해도 후보가 정확한 RT라는 보장은 없다.

### 2.4 V3 Hybrid

V3는 T2의 organized panorama/range topology와 T1의 vanishing-point/
Manhattan 구조 분석을 결합한 prototype이다.

- 재사용: T2 panorama raster, row/column validity, range topology
- 재사용: T1 camera vanishing estimator와 LiDAR Manhattan estimator
- 제거 목표: 전역 perspective remap 216회
- 유지: B0 full staged search, hold-out gate, fail-safe fallback 개념
- 추가 진단: 1D signature, Top-K basin, covariance, 후보 평가량 계측

V3-R1 기록상 bounded orientation 내부 평가량은 B0의 168에서 48로
`71.43%` 줄었지만, analyzer 비용을 포함한 V3 wall time은 B0보다 약
`10.66%` 높았다. 또한 다섯 build에서 Top-K에 Manual Reference ±10°
후보가 없었다. 따라서 V3는 계산량 감소 가능성을 보인 연구 prototype이지
제품 초기화기가 아니다.

### 2.5 한눈에 보는 차이

| 경로 | 주 입력/신호 | 후보 생성 방식 | 장점 | 확인된 위험 | 최종 역할 |
|---|---|---|---|---|---|
| B0 | 기존 cross-modal production objective | 넓은 full search | basin 누락 위험이 상대적으로 낮음 | 계산량·대칭 장면 ambiguity | baseline/oracle/fail-safe 후보 |
| T1 | image line/VP + LiDAR normal/Manhattan | 구조축 기반 Top-K | 가볍고 설명 가능 | 구조축 오인, roll observability 부족 | 동결 실험 |
| T2 | organized LiDAR panorama + perspective 비교 | panorama signature Top-K | 360° topology 활용 | remap 비용, signature alias, ambiguity | 동결 실험 |
| V3 Hybrid | T1+T2 feature/signature | analyzer basin → bounded 후보 | 후보 평가량 감소 가능 | proposal recall, objective conflict, fallback 부족 | 제품 미승인 연구 prototype |

## 3. 브랜치를 나눈 이유

브랜치 분리는 다음 세 가지를 보장하기 위해 필요했다.

1. B0의 기존 동작과 산출물을 오염시키지 않는다.
2. 구조선(T1), panorama(T2), 점수 ablation, directional edge를 서로 독립 가설로 평가한다.
3. 실패가 analyzer 때문인지, objective 때문인지, Ceres/finalist 선택 때문인지
   lineage와 실행 증거로 구분한다.

따라서 실험 브랜치를 병합해 하나의 “개선된 V3”로 표현하면 실험 조건과
실패 원인이 섞인다. R1~R7은 결과와 증거를 보존하되, 이번 closeout에서는
제품 경로에 병합하지 않는다.

## 4. R1~R7에서 검증한 가설과 결과

### R1 — Proposal pose contract 및 camera-center lock

가설은 analyzer proposal을 full `R/t`로 직렬화하고, 물리 camera center를
명시하면 translation ambiguity가 제거되는지 확인하는 것이었다.

- proposal pose는 finite/proper rotation으로 직렬화되었다.
- explicit center 경로에서 `t=-R*C_lidar`가 유지되었고 center error는
  부동소수점 수준까지 줄었다.
- 그러나 camera center를 고정해도 잘못된 orientation basin은 교정되지 않았다.
- build45/46/48/49/50 모두 Top-K full-pose 기준 Manual Reference ±10°
  recall이 없었다.
- B0 대비 bounded 후보 수는 감소했지만 총 wall-time 개선은 입증되지 않았다.

결론: **pose contract/translation lock은 유효한 안전 계약이지만,
analyzer basin recall 문제의 해결책은 아니다.**

### R2 — Candidate funnel 및 finalist lineage

가설은 Reference 인근 후보가 생성되지 않은 것인지, NMS/Top-K/bounded/Ceres/
finalist 선택 중간에 사라진 것인지 구분할 수 있는지였다.

- 10° full-pose 기준으로 Reference 후보가 analyzer Top-K에 없었다.
- raw 단계의 가장 가까운 후보도 Top-K score rank와 Reference 거리 rank가 달랐다.
- build45에서는 `13.009982°`인 rank-2 branch가 Ceres finalist까지 남았지만,
  objective가 더 낮은 다른 finalist가 선택되어 최종 오차가 `31.364358°`가 됐다.
- build46은 rank-3 proposal에서 최종 `12.452443°`로 이어졌다.
- build50은 rank-3 proposal에서 `yaw=-152°` branch가 선택됐고,
  최종 reference error는 `168.263640°`, reason은
  `OBJECTIVE_IMPROVEMENT_INSUFFICIENT`, observed fallback은 `false`였다.

결론: 후보 생성/후보 순위/최종 finalist 선택을 분리해서 계측해야 하며,
현재 score winner가 Reference에 가깝다는 보장은 없다.

### R3 — Manual Reference objective discriminability

가설은 현재 production objective가 Manual Reference를 잘못된 pose보다
우수하게 평가하는지였다.

- Reference는 다섯 build에서 안정적인 local minimum으로 확인되지 않았다.
- edge alignment는 `5/5`, coverage는 `4/5`에서 Reference를 선호했지만,
  geometry NID는 `1/5`, structural line은 `2/5`, Manhattan은 `1/5`만
  Reference를 선호했다.
- build45에서 selected-minus-Reference composite delta는 `-0.014358`으로,
  unchanged production objective가 selected branch를 더 낮게 평가했다.
- 180° wrong branch와의 비교, common-support 한계, score component 충돌을
  확인하고 score 수정 전 Sol review 중단으로 분류했다.

결론: **후보 수를 늘리기 전에 objective 식별력과 support를 검토해야 한다.**

### R3-R1 — Exact common support

가설은 후보별로 다른 LiDAR point 수가 geometry NID 순위를 뒤집었는지였다.

- build45 variable support는 Reference geometry 444점, selected 175점이었다.
- exact point-ID support에서 build45 geometry margin과 composite 순위가
  Reference 우세 방향으로 바뀌었다.
- build49도 approximate/variable 결과와 exact 결과의 순위 방향이 달라졌다.
- build50 selected pair는 common geometry ID가 0이어서 비교 불가능했다.

결론: **variable-support가 geometry NID 순위를 왜곡할 수 있다는 증거는
있지만, 모든 build에 공통된 단일 원인으로 확정할 수 없다.**

### R3-R1-R1 — Exact final NID sample support

최종 spatial balancing 이후에도 실제 histogram sample count가 달라지는지
확인했다.

- forced sample mask를 적용해 비교 가능한 pair에서 range/normal sample
  count를 동일하게 맞추는 audit 경로를 검증했다.
- 그러나 corrected exact support에서 selected가 우세한 build가 3개였고,
  build50은 common sample 0으로 제외되었다.
- 최종 분류는 `GEOMETRY_NID_DISCRIMINABILITY_FAILURE`였으며, NID 수식
  변경 승인은 얻지 못했다.

결론: sample-support bias만으로 전체 오답을 설명할 수 없고, NID 변경은
동결한다.

### R4 — Geometry NID ablation

가설은 NID weight 자체가 잘못된 RT 선택의 주원인인지였다.

- Legacy `--nid-weight 0.55`와 NID-off `--nid-weight 0.0`를 같은 조건에서
  비교했다.
- build45/46/49에서 NID-off rotation error가 감소해 `3/4` eligible build에서
  개선으로 기록되었다.
- NID-off는 새 90°/180° branch나 잘못된 product PASS를 만들지 않았다.
- build50은 양 조건 모두 FAIL을 유지했다.

결론은 `NID_HARMFUL_IN_CURRENT_SCENE_SET`이지만, 이는 현재 scene set의
ablation 결과일 뿐 기본 weight 변경이나 제품 승인을 의미하지 않는다.

### R5 — Structural/Manhattan factorial ablation

NID-off 이후 남은 오차를 structural line과 Manhattan objective로 나눴다.

- line만 제거한 C는 유효 build 개선 조건을 충족하지 못했다.
- Manhattan만 유지한 D는 `2/4`에서 2° 이상 개선되어 3개 조건을 충족하지
  못했다.
- 둘 다 제거한 E는 build45/46/48/49 결과가 불안정했고 build50에서
  `INTERNAL_GATE_PASS`가 발생하여 요구된 fail-closed를 깨뜨렸다.
- 따라서 `CURRENT_CROSS_MODAL_FEATURES_INSUFFICIENT`로 종료했다.

결론: structural/Manhattan weight를 단순히 제거하는 것은 안전한 제품
해결책이 아니다. build50 false acceptance 때문에 조건 E는 특히 병합 금지다.

### R6 — Directional cross-modal edge 계열

#### R6 MVP

Sobel orientation이 edge normal이고 LiDAR projected direction이 tangent라는
문제를 발견하기 전 fixed-pose directional edge를 시험했다. Stage A에서
Reference 우세는 `2/4`뿐이어서 E2E 승격 조건을 충족하지 못했다.

#### R6-R1 orientation/support correction

카메라 edge tangent를 다음과 같이 정정했다.

```text
camera_edge_tangent = fmod(atan2(gy, gx) + pi/2, pi)
```

variable-support와 exact-common-support fixed-pose 비교는 각각 `3/4`
Reference 우세로 개선되었다. 그러나 E2E에서는:

- build45 shadow diagnostic 후보는 약 `7.263°`였으나 optimizer/gate를
  통과하지 못했다. 이후 operational 출력이 약 `168.386°`인 fallback
  prior로 처리된 것이며, directional 후보 자체가 168°로 수렴한 것은 아니다.
- build50이 요구된 FAIL 대신 `INTERNAL_GATE_PASS`
- 최대 runtime overhead `51.8221%`
- 신규 잘못된 branch와 operational gate 문제가 발생

결론: fixed-pose 판별력 개선이 E2E 안전성·수렴성으로 이어지지 않았다.

#### R6-R2 proposal-only shadow

directional edge를 composite/Ceres에 넣지 않고 기존 analyzer proposal 중
최고 proposal 하나만 shadow 후보로 추가했다.

- baseline status/reason/RT는 보존되었다.
- shadow diagnostic improvement가 요구된 `3/4`에 미달했다.
- build45/build48에서 신규 90°/180° wrong branch가 관측됐다.
- 최대 runtime overhead `22.0720%`로 15% 기준 초과
- non-converged shadow 후보는 operational로 승격하지 않았지만, shadow가
  제품 복구 경로가 될 수 있다는 증거는 없었다.

결론: R6 directional-edge 계열은 종료하고, R6 코드와 weight를 추가 수정하지
않는다.

### R7 — Same-installation multi-capture orientation consensus

목적은 단일 build 점수의 모호성을 줄이기 위해 build 간 후보 percentile을
합의하는 것이었다.

방법:

- build45/46/48/49/50의 공통 metadata/profile 계약을 비교했다.
- raw score는 합산하지 않고 각 build 내부 rank percentile로 정규화했다.
- 5° yaw/down/roll common grid에서 build별 percentile median을 계산했다.
- 연속 orientation basin과 support build 수를 별도 기록했다.
- Ceres/E2E는 실행하지 않았다.

결과:

- metadata-compatible build는 `5/5`로 최소 4개 조건을 만족했다.
- 다만 파일에 CAD/survey/rigid-installation token이 없어 물리적 강체 설치는
  독립 증명되지 않았다.
- primary full-3D consensus Top-3의 대표 후보 Reference 최근접 오차는
  `34.4658°`, `38.8535°`, `47.9773°`였고 ±10° 후보는 없었다.
- 최고 shared basin support는 `3` build였다.
- 90°/180° wrong basin은 primary rank-1이 아니었다.
- B0 평균 `97.0960 s`, R7 audit `2064.94 ms`, overhead `2.1267%`로
  runtime 조건은 통과했다.

최종 판정:

```text
CURRENT_TARGETLESS_FEATURES_INSUFFICIENT_FOR_RELIABLE_INITIALIZATION
```

결론: 다중 capture percentile consensus는 계산 비용이 낮고 분석 도구로는
유용하지만, 현재 후보 생성기가 Reference 인근의 공통 full-3D basin을
만들지 못하므로 제품 초기화기로 사용할 수 없다.

## 5. Fixed-pose 개선과 E2E 실패의 차이

이번 실험군에서 가장 중요한 구분은 “고정된 RT를 채점했을 때 잘 보이는가”와
“그 후보를 실제 optimizer/품질 gate 경로에 넣었을 때 안전하게 끝나는가”가
다르다는 점이다.

| 구분 | fixed-pose에서 관측 | E2E에서 관측 | 해석 |
|---|---|---|---|
| directional edge | R6-R1 corrected variable/exact 각각 3/4 Reference 우세 | build45의 약 7.263° diagnostic 후보가 optimizer/gate 실패 후 operational fallback prior(약 168.386°)로 처리됨, build50 false acceptance, runtime 51.8% | 고정 pose 식별력만으로 optimizer 수렴·제품 출력 안전성을 보장하지 못함 |
| NID-off | 일부 build rotation error 감소 | weight 변경 자체는 제품 경로가 아니며 structural/Manhattan 충돌 잔존 | scene-specific ablation |
| T1/T2/V3 analyzer | 후보 평가량·일부 basin proximity 개선 | Top-K Reference ±10° recall 없음, fallback/ambiguity 미흡 | proposal recall과 fail-closed가 핵심 |
| multi-capture consensus | percentile 합의 계산 가능 | R7에서는 E2E를 하지 않았고 full-3D Top-3가 ±10° 미달 | consensus를 제품 prior로 승격할 근거 부족 |

따라서 fixed-pose score가 낮다는 결과만으로 production objective 변경,
directional edge 삽입, Ceres 초기값 교체를 승인할 수 없다.

## 6. Geometry NID sample-support bias

NID는 후보별로 서로 다른 visible point와 spatial sample을 사용하면 같은
장면을 비교하지 않게 된다. build45에서:

```text
Reference geometry NID points = 444
selected 31° pose geometry NID points = 175
```

이 상태에서 production variable-support geometry margin은 selected를
유리하게 만들었고, exact point-ID/common forced sample을 적용하자 Reference
우세 방향으로 바뀌었다. 이는 다음을 의미한다.

- NID 수치 자체와 squared objective contribution을 혼동하면 안 된다.
- 후보별 coverage가 달라지는 현재 scoring 경로는 support bias에 민감하다.
- exact support 비교는 원인 진단에는 유효하지만, sample이 너무 적거나
  0이면 `NOT_COMPARABLE`이어야 한다.
- build50 common support 0은 어느 후보의 우세 근거로 사용하지 않는다.

R3-R1-R1의 최종 결과에서는 exact final sample을 맞춘 뒤에도 selected가
우세한 build가 3개였으므로, sample-support bias만으로 전체 현상을 해결할
수 없다는 결론을 유지한다. 따라서 이 closeout에서는 NID 식·weight·threshold를
변경하지 않는다.

## 7. Directional-edge variable-support 및 runtime 문제

Directional edge는 fixed-pose에서 Reference를 더 잘 구별할 가능성을 보였지만
두 문제가 남았다.

1. **orientation contract 문제**: 초기 R6는 Sobel normal과 LiDAR tangent를
   직접 비교해 90° convention 오류가 있었다. R6-R1에서 tangent 변환을
   수정했지만 이는 fixed-pose audit의 정정이지 제품 승인 증거가 아니다.
2. **support 문제**: Reference와 selected가 서로 다른 visible point를 사용해
   variable-support 결과가 될 수 있었다. exact-common-support는 진단에만
   사용했고 제품 score를 바꾸지 않았다.
3. **runtime 문제**: R6-R1 최대 overhead `51.8221%`, R6-R2 최대 overhead
   `22.0720%`로 각 단계의 허용 기준을 초과했다.
4. **E2E 문제**: build45의 약 `7.263°` diagnostic 후보는 optimizer/gate를
   통과하지 못했고 operational 출력은 약 `168.386°`인 fallback prior로
   처리되었다. 이는 directional 후보가 168°로 수렴했다는 뜻이 아니다.
   build50은 fail-closed 기대를 충족하지 못했다.

결정: directional edge는 R6 계열 실험 증거로만 보존하고, production composite,
Ceres residual, fallback, 기본 weight에 병합하지 않는다.

## 8. 현재 후보 생성기가 Reference ±10°를 생성하지 못한다는 근거

다음 증거가 서로 같은 결론을 가리킨다.

- V3-R1: build45/46/48/49/50 모두 Top-K full-pose ±10° recall `false`.
- V3-R2: Top-K 최인접 후보는 build45 `13.009982°`, build46
  `13.723223°`, build48 `29.930912°`, build49 `30.392517°`, build50
  `44.284553°`였다.
- V3-R2 lineage: build45의 13° branch는 존재했지만 최종 finalist 선택에서
  다른 branch가 이겼고, build50의 rank-3 branch는 168°대 operational
  결과로 이어졌다.
- V3-R7: 기존 raw analyzer 360 candidate inventory를 build별 percentile로
  합의해도 primary full-3D consensus Top-3 최근접 오차가
  `34.4658°/38.8535°/47.9773°`였으며 ±10° 후보가 없었다.
- R7 yaw-only consensus는 모든 build가 지지하는 셀을 만들었지만 Reference와
  멀리 떨어진 방향이므로, yaw-only support를 full-3D recall로 대체할 수 없다.

따라서 현재의 문제는 단순히 1°/5° search 간격을 더 촘촘히 하는 문제가
아니다. 후보 생성기가 full pose의 올바른 basin을 보존·제안하지 못하는
proposal coverage 문제와, 잘못된 후보를 거부하지 못하는 ambiguity 문제다.

## 9. Targetless global initialization 종료 결정

현재 targetless global initialization은 다음 이유로 제품 경로에서 종료한다.

- analyzer Top-K가 Reference 근방 full-pose 후보를 안정적으로 보존하지 못함
- production objective의 geometry/structural/Manhattan 항이 서로 충돌함
- variable support가 NID 순위에 영향을 줌
- directional edge는 fixed-pose 개선과 E2E 실패가 공존함
- fallback이 일부 잘못된/비수렴 결과를 충분히 fail-closed하지 못함
- multi-capture consensus도 Reference ±10° 공통 basin을 만들지 못함
- metadata가 동일해도 물리적 강체 설치 상태가 독립적으로 증명되지 않음

여기서 “종료”는 연구를 영구 폐기한다는 의미가 아니다. 현재 데이터와
구현으로는 제품 초기화기로 채택하지 않고, R7을 마지막 audit으로 삼아
실험 계열을 frozen 상태로 유지한다는 의미다.

## 10. 병합 금지 대상 실험 브랜치

아래 브랜치는 실험 조건 또는 진단 경로를 담고 있으므로 `develop` 또는
제품 기본 경로로 병합하지 않는다. 커밋과 증거는 보존한다.

| 실험 계열 | 브랜치/기준 커밋 | 병합 금지 이유 |
|---|---|---|
| R1 | `exp-v3-proposal-pose-center-lock` / `19a42d3` | proposal/center-lock 실험이며 제품 전체 검증이 아님 |
| R2 | `exp-v3-r2-candidate-lineage` / `d1620bb`, 정정 `8a18332` | lineage/evaluator 진단용 |
| R3 | `exp-v3-r3-objective-discriminability` / `3eef74c` | objective audit 전용 |
| R3-R1 | `exp-v3-r3-r1-exact-common-support` / `c4b110d` | exact support 진단 전용 |
| R3-R1-R1 | `exp-v3-r3-r1-r1-exact-final-sample-support` / `330eb45` | final sample audit 전용 |
| R4 | `exp-v3-r4-nid-ablation` / `4393d0d` | NID weight ablation, 기본 weight 변경 실험 아님 |
| R5 | `exp-v3-r5-objective-ablation` / `8b44b3b` | structural/Manhattan factor 실험, build50 false acceptance 포함 |
| R6 | `exp-v3-r6-directional-edge` / `97040fa` | experimental directional score |
| R6-R1 | `exp-v3-r6-r1-directional-edge-correction` / `d038e21` | orientation/support correction audit, E2E gate 실패 |
| R6-R2 | `exp-v3-r6-r2-directional-proposal-shadow` / `e8cfa77` | shadow 후보 경로, runtime/wrong branch 실패 |
| R7 | `exp-v3-r7-multi-capture-consensus-audit` / `0244578` | audit-only percentile consensus, product initialization 미승인 |

특히 R4/R5/R6의 weight·score·directional 변경을 cherry-pick해 제품 기본
경로를 구성하지 않는다. R7 커밋은 audit script와 report만 추가하며 제품
calibration path를 바꾸지 않는다.

## 11. 보존해야 할 보고서·CSV·JSON·커밋

### 11.1 기준 설계/기획 문서

- `automatic_calibration/docs/B0_T1_T2_ANALYZER_EXPERIMENT_HISTORY_AND_V3_PLAN_20260827.md`
- `automatic_calibration/docs/V3_HYBRID_ANALYZER_EXECUTION_REPORT_20260827.md`
- `automatic_calibration/docs/MANUAL_REFERENCE_PRIOR_WORKFLOW.md`
- `automatic_calibration/docs/V3_MANUAL_REFERENCE_ANALYZER_EVALUATION_REPORT_20260830.md`

### 11.2 R1/R2 pose 및 candidate evidence

- `automatic_calibration/docs/V3_R1_PROPOSAL_POSE_AND_CAMERA_CENTER_LOCK_REPORT_20260830.md`
- `automatic_calibration/generated/v3_r1_proposal_pose_center_lock_20260830/proposal_full_pose.csv`
- `.../proposal_vs_reference.csv`
- `.../reference_basin_recall.json`
- `.../final_rt_vs_reference.csv`
- `.../camera_center_contract.csv`
- `.../build50_failure_trace.json`
- `.../runtime_comparison.csv`
- `.../validation_checks.json`
- `automatic_calibration/docs/V3_R2_CANDIDATE_FUNNEL_AND_LINEAGE_REPORT_20260830.md`
- `automatic_calibration/generated/v3_r2_candidate_lineage_20260830/candidate_funnel.csv`
- `.../candidate_lineage.csv`
- `.../stage_recall_summary.csv`
- `.../nearest_reference_candidate_per_stage.csv`
- `.../build45_lineage_trace.json`
- `.../build46_lineage_trace.json`
- `.../build50_lineage_trace.json`
- `.../candidate_loss_classification.json`
- `.../validation_checks.json`
- `automatic_calibration/docs/V3_R2_R1_POSE_PROVENANCE_CORRECTION_REPORT_20260830.md`

### 11.3 R3 objective/support evidence

- `automatic_calibration/docs/V3_R3_OBJECTIVE_DISCRIMINABILITY_REPORT_20260830.md`
- `automatic_calibration/generated/v3_r3_objective_discriminability_20260830/audit_candidate_manifest.csv`
- `.../objective_components.csv`
- `.../weighted_contributions.csv`
- `.../pairwise_reference_margins.csv`
- `.../local_perturbation_landscape.csv`
- `.../common_support_comparison.csv`
- `.../build45_score_inversion_trace.json`
- `.../wrong_branch_comparison.json`
- `.../component_discriminability_summary.json`
- `.../validation_checks.json`
- `automatic_calibration/docs/V3_R3_R1_EXACT_COMMON_SUPPORT_REPORT_20260831.md`
- `automatic_calibration/generated/v3_r3_r1_exact_common_support_20260831/support_membership_summary.csv`
- `.../exact_common_support_ids.csv`
- `.../variable_vs_common_scores.csv`
- `.../common_support_pairwise_margins.csv`
- `.../histogram_support_statistics.csv`
- `.../build45_exact_support_trace.json`
- `.../build49_exact_support_trace.json`
- `.../build50_exact_support_trace.json`
- `.../validation_checks.json`
- `automatic_calibration/docs/V3_R3_R1_R1_EXACT_FINAL_SAMPLE_SUPPORT_REPORT_20260831.md`
- `automatic_calibration/generated/v3_r3_r1_r1_exact_final_sample_support_20260831/final_sample_support_summary.csv`
- `.../result_summary.json`
- `.../validation_checks.json`

### 11.4 R4/R5 objective ablation evidence

- `automatic_calibration/docs/V3_R4_GEOMETRY_NID_ABLATION_REPORT_20260831.md`
- `automatic_calibration/generated/v3_r4_geometry_nid_ablation_20260831/nid_ablation_comparison.csv`
- `.../runtime_comparison.csv`
- `.../candidate_selection_comparison.csv`
- `.../validation_checks.json`
- `automatic_calibration/docs/V3_R5_OBJECTIVE_FACTORIAL_ABLATION_REPORT_20260831.md`
- `automatic_calibration/generated/v3_r5_objective_ablation_20260831/objective_ablation_comparison.csv`
- `.../candidate_selection_comparison.csv`
- `.../runtime_comparison.csv`
- `.../validation_checks.json`

### 11.5 R6/R7 directional/consensus evidence

- `automatic_calibration/docs/V3_R6_DIRECTIONAL_CROSS_MODAL_EDGE_REPORT_20260831.md`
- `automatic_calibration/docs/V3_R6_R1_DIRECTIONAL_EDGE_CORRECTION_REPORT_20260831.md`
- `automatic_calibration/docs/V3_R6_R2_DIRECTIONAL_PROPOSAL_SHADOW_REPORT_20260831.md`
- R6/R6-R1/R6-R2 generated roots의 `directional_edge_*.csv`,
  `directional_proposal_shadow_summary.csv`, `candidate_lineage.csv`,
  `optimizer_termination.csv`, `runtime_comparison.csv`,
  `validation_checks.json`, orientation-bin/overlay/3D preview
- `automatic_calibration/docs/V3_R7_MULTI_CAPTURE_CONSENSUS_AUDIT_REPORT_20260831.md`
- `automatic_calibration/generated/v3_r7_multi_capture_consensus_20260831/input_manifest.csv`
- `.../candidate_inventory.csv`
- `.../multi_capture_orientation_consensus.csv`
- `.../basin_support_per_build.csv`
- `.../consensus_summary.json`
- `.../validation_checks.json`

R6/R7의 대형 PNG/PLY/OBJ와 raw scan은 Git 추적 대상이 아니다. 생성된
증거 디렉터리는 로컬 보관 정책에 따라 보존하고, 보고서·CSV·JSON·log의
경로와 hash를 감사 기록으로 유지한다.

## 12. 권장 제품 구조

현재 targetless global initialization을 제품 기본 경로로 사용하지 않고,
다음 순서로 운용한다.

```text
nominal factory/manual RT
        ↓
bounded local verification
        ↓
repeatability gate
        ↓
last-known-good fail-closed
```

### 12.1 Nominal factory/manual RT

- 제조사 검증 profile 또는 별도 승인된 manual/fixture calibration을 nominal
  RT로 저장한다.
- K/D profile, distortion state, camera center/CAD 계약의 provenance를 함께
  저장한다.
- 현재 Manual Reference는 이 구조를 설계하기 위한 provisional engineering
  nominal이며, 아직 제품 승인값이 아니다.

### 12.2 Bounded local verification

- nominal RT 주변의 제한된 orientation/translation만 검사한다.
- 전역 analyzer가 찾은 후보를 제품 초기값으로 무조건 수용하지 않는다.
- verification은 image–LiDAR overlap, target/edge support, pose finite/proper,
  camera-center 계약, 동일한 입력 계약을 확인하는 별도 단계다.
- 검증 실패 시 새 RT를 생성했다고 보고하지 않는다.

### 12.3 Repeatability gate

- 동일한 강체 설치 상태에서 여러 capture를 확보한다.
- capture 간 RT/score/support가 재현되지 않거나 ambiguity가 남으면 reject한다.
- 단일 장면의 높은 score나 단일 build의 PASS만으로 승격하지 않는다.
- 설치 epoch, 카메라·LiDAR 모듈 상태, K/D/distortion state, pair provenance를
  함께 기록한다.

### 12.4 Last-known-good fail-closed

- 검증을 통과한 마지막 RT만 운용값으로 유지한다.
- 새 추정이 gate를 통과하지 못하면 새 RT를 반환하지 않고 last-known-good
  또는 명시적 calibration unavailable 상태를 반환한다.
- `OBJECTIVE_IMPROVEMENT_INSUFFICIENT`, optimizer non-convergence,
  support 부족, ambiguity를 PASS로 변환하지 않는다.
- build50처럼 FAIL 상태의 safe/estimated/visualization pose를 제품 승인
  RT로 표현하지 않는다.

## 13. 전역 targetless 연구 재개 조건

연구를 다시 시작할 수 있는 조건은 다음과 같다. 이 조건들은 새 score나
threshold를 이번 closeout에서 추가하는 것이 아니라, 재개 전 필요한 증거의
체크리스트다.

1. **독립 K/D provenance**: 제조사 camera/lens profile 또는 충분한 coverage와
   pose 다양성을 가진 새 intrinsic 세트가 검증되어야 한다. clean18/all68
   중 하나를 결과가 좋다는 이유만으로 제품 K/D로 선택하지 않는다.
2. **비대칭 관측 구조**: 평평한 벽·바닥·천장처럼 180°/반복 ambiguity를 만드는
   장면만 사용하지 않고 LiDAR에서 방향을 구분할 수 있는 비대칭 입체 구조를
   포함한다.
3. **정적 image–scan pair**: scan 전후 target pose를 확인하고, LiDAR sweep
   시간 동안 target motion과 holder/background contamination을 분리한다.
4. **설치 provenance**: 동일 camera–LiDAR 강체 모듈임을 metadata만이 아니라
   CAD, survey, fixture token 또는 동등한 독립 증거로 확인한다.
5. **독립 설치 구성과 hold-out**: 하나의 장소 반복만으로 끝내지 않고 서로
   다른 설치 구성/구조에서 training과 hold-out을 분리한다. 최소 세 구성과
   각 구성의 독립 검증 pair를 확보하는 것을 재개 전 기준으로 삼는다.
6. **full-pose candidate recall 증거**: raw candidate부터 Top-K, bounded,
   finalist까지 full `R/t` lineage를 보존하고, 올바른 basin이 없어진 단계와
   이유를 관측한다. yaw-only 근접성은 full-pose recall로 대체하지 않는다.
7. **공통 support 및 ambiguity 검증**: variable-support와 exact common-support를
   모두 비교하고, support가 작으면 fail-closed한다. cross-capture consensus가
   한 build의 score winner보다 안정적인지 독립적으로 확인한다.
8. **E2E 안전성·runtime 검증**: fixed-pose 개선만으로 진행하지 않고 실제
   optimizer, gate, fallback을 포함해 non-convergence와 false acceptance가
   없는지 확인한다. CV5 2-core/4 GB에서 warm-up/median/p95 runtime을 별도로
   측정한다.
9. **제품 승격 review**: 위 증거가 모인 뒤에도 Manual Reference와 B0를 ground
   truth로 간주하지 않고, Sol/제품 책임자의 명시적 review를 거쳐서만 새로운
   알고리즘·weight·threshold를 별도 브랜치에서 평가한다.

## 14. 최종 결정

V3 실험은 “analyzer로 search range를 줄일 수 있다”는 가능성과, 그 과정에서
후보 평가량을 줄일 수 있다는 prototype 결과를 남겼다. 그러나 제품에 필요한
핵심 조건인 **정답 full-pose basin의 안정적 생성·보존, E2E fail-closed,
독립 hold-out 재현성**을 충족하지 못했다.

그러므로 현재 결정은 다음과 같다.

- R6 directional-edge 계열: 종료 및 코드 동결
- R7 multi-capture consensus: audit-only로 종료
- V3 analyzer global initialization: 제품 경로에 병합 금지
- 기존 B0/nominal RT: 제품 안전 경계 안에서 보존
- 제품 운용: `REFERENCE_ANCHORED_LOCAL_VERIFICATION`
- 연구 재개: 독립 reference/provenance/비대칭 target/full-pose recall/E2E gate
  증거 확보 후 별도 승인

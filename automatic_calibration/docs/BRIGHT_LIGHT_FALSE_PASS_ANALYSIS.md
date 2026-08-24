# 조명 켜짐 장면의 투영 오류 및 false PASS 분석

- 작성일: 2026-08-14
- 대상 데이터: `data/real_calibration/session-const-env/repeat_test_sample`
- 대상 조건: 사람 없는 고정환경, 조명 켜짐, CH1, 5회 반복
- 실행 결과: `automatic_calibration/generated/repeat_test_sample_20260813/bright`
- 상태: **PASS를 유효한 캘리브레이션 결과로 인정하지 않음**

> 이 문서는 과거 false PASS의 원인과 수정 이력을 보존한다. 2026-08-20 이후 MVP 운용
> 기준은 [`PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)이며, Manual
> ChArUco `K+D` 고정·RT 전용 추정·제품 승인 상태 분리를 따른다.

## 1. 결론

조명 켜짐 장면에서 나타난 현재 `PASS`는 실제 2D–3D 정합을 보증하는 결과가 아니다.
투영 이미지에서 LiDAR 점과 edge가 바닥·책상 하단에 몰리고, 실제 설치와 다른 광축·센서
offset이 추정되었기 때문에 false positive로 분류한다.

주원인은 조명보다 다음 네 가지의 결합이다.

1. 고정환경의 실측 camera center를 입력하지 않아 예전 데이터용 `baseline=0.28 m`가 사용됨
2. 같은 장면의 반복 scan을 독립적인 다중 장면처럼 사용함
3. Sobel gradient와 LiDAR range/normal 변화량을 NID로 비교해 반복 구조의 잘못된 basin이 선택됨
4. 구조선·물리 offset·hold-out 재투영을 요구하지 않는 현재 PASS gate

따라서 검색 간격을 15°에서 5° 또는 1°로 줄이는 것만으로는 해결되지 않는다. 먼저 물리
제약과 승인 기준을 수정하고, 이후 geometry-first coarse-to-fine 정합을 적용해야 한다.

## 2. 재현된 조명 켜짐 결과

### 2.1 입력 구성

입력 폴더에는 조명 켜짐 이미지 5장과 LiDAR JSON 5개가 있다. 파일명에 공통 capture ID가
없어 실행기는 정렬된 파일의 1:1 대응을 사용했다. 고정 장면이라는 전제에서는 사용할 수
있지만, 실제 시간 동기 보장은 아니다.

반복 측정은 noise/repeatability 평가에는 유효하다. 그러나 동일한 위치와 구도의 5개 이미지는
서로 다른 시점이나 다른 구조를 관측한 5개의 독립 calibration view가 아니다.

### 2.2 결과 수치

| 항목 | 결과 | 판단 |
|---|---:|---|
| 상태 | `PASS` | 내부 gate만 통과 |
| 선택 yaw | `60°` | 구조적 정답 근거 없음 |
| 선택 down | `90°` | 탐색 경계에 붙음 |
| 평균 edge 거리 | `27.05 px` | 현재 한계 40 px 이내이나 시각적으로 큰 오차 |
| 최종 NID | `0.9357` | 0에 가까울수록 좋은 지표에서 높은 값 |
| 구조선 | 31개 검출, 10개 projected | 장면 전체 구조를 대표하지 못함 |
| 추정 translation | `(0.27455, 0.00904, -0.01093) m` | 실측 설치와 불일치 |
| camera center 입력 | 없음 | 기본 baseline 사용 |
| LDC | `unknown` | 왜곡 상태 미확정 |

관련 산출물:

- [조명 켜짐 매칭 결과](../generated/repeat_test_sample_20260813/bright/matching_scene_0.png)
- [3D 투영 미리보기](../generated/repeat_test_sample_20260813/bright/scene_0_colorized_lidar_3d_preview.png)
- [수치 결과 JSON](../generated/repeat_test_sample_20260813/bright/calibration_result.json)
- [기존 실행 분석](../generated/repeat_test_sample_20260813/ANALYSIS.md)

### 2.3 실측 센서 중심을 반영한 재실행 (2026-08-14)

이전 false PASS의 직접 원인인 기본 `baseline=0.28 m` 사용을 분리하기 위해, 지면에서
위로 **LiDAR → 카메라** 순서라는 설치 측정값을 반영해 CH1을 재실행했다. 천장 기준
높이(카메라 551.95 mm, LiDAR 635 mm)로부터 카메라는 LiDAR보다 83.05 mm 위에
있으며, JSON `+Y=down` frame에서 다음 중심을 사용했다.

```text
C_L = (+0.05928, -0.08305, 0) m
```

실행 조건은 5° yaw/down coarse search, LDC `unknown`, zoom/focus locked, legacy range
offset 0.084 m이다. 결과는 `FAIL / MULTISTART_AMBIGUOUS`였고, contiguous 후보 보정
선택은 `down=80°`였다. 따라서 센서 중심의 부호와 위치 입력은 정상적으로 반영됐지만,
단일 고정 장면의 목적함수는 여전히 방향을 식별하지 못한다.

이번 산출물의 `matching_scene_*.png`와 3D preview에는 최종 승인 RT가 아니라
`REJECTED CANDIDATE`가 표시된다. 즉 이 이미지는 센터 오프셋 수정 후에도 남은 방향
오류를 확인하기 위한 진단 자료이며, 제품 calibration 값으로 사용하지 않는다.

산출물: `../generated/repeat_test_sample_20260814/bright_center_constrained/`

> 정정 기록: 위 실행은 당시 계산값 83.05 mm를 사용한 역사적 결과다. 모델링 치수
> 재확인 후 현재 수직 offset은 81.05 mm이며, 수평 59.28 mm와 함께
> `C_L=(+0.05928,-0.08105,0) m`를 사용한다. 정정값 동일 조건 재실행은
> `repeat_test_sample_20260814_light_on_geometry_first_5deg_v5_offset_81p05mm`에 있다.

## 3. 물리적 원인

### 3.1 잘못된 camera center prior

고정환경에서 정정한 CH1 카메라 중심은 LiDAR frame에서 다음과 같다.

```text
C_L = (+0.05928, -0.08105, 0) m
|C_L| = 0.100415 m
```

여기서 `-0.08105 m`는 카메라가 LiDAR보다 위에 있기 때문에 생기는 부호다. LiDAR
frame의 `+Y`가 아래 방향이므로, 지면→LiDAR→카메라 순서는 카메라 중심의 음의 Y로
표현된다.

카메라 중심과 LiDAR 중심의 관계는 다음으로 계산한다.

```text
t_camera_lidar = -R_camera_lidar * C_L
```

그러나 이번 실행 결과에는 `camera_center_lidar_m=null`, `baseline_m=0.28`이 기록되어
있고, 최종 translation 크기도 약 0.275 m이다. 따라서 결과는 실제 고정환경에서 가능한
센서 배치와 약 173 mm 차이가 난다.

이 상태에서 optimizer는 화면 edge 비용을 낮추는 방향으로 비현실적인 translation과 회전을
선택할 수 있다. 이것이 현재 reprojection이 다른 공간에 있어도 `PASS`가 된 가장 직접적인
원인이다.

### 3.2 down 경계 선택

후보 탐색은 `down=0, 15, ..., 90°`이고 최종 선택은 `90°`였다. 이는 카메라가 실제로
바닥을 본다는 사실과 일치할 가능성은 있지만, 탐색 공간 끝에서 선택됐다는 것은 현재
범위 안에서 최적점이 닫힌 내부 최적점인지 확인할 수 없다는 의미다. 경계 후보는 자동
승인하지 않고 검색 범위를 확장하거나 별도 진단 상태로 내려야 한다.

### 3.3 카메라와 LiDAR가 서로 다른 것을 edge로 본다

현재 카메라 특징은 Sobel gradient magnitude이고, LiDAR geometry 특징은 range
discontinuity와 surface normal 변화량이다. 두 특징은 동일한 물리 경계라는 보장이 없다.

예를 들어 다음은 카메라에 강한 edge를 만들지만 LiDAR 구조선과 일치하지 않을 수 있다.

- 캐비닛 문틈과 손잡이
- 책상 표면의 그림자·반사
- 의자·전선의 윤곽
- 조명으로 생긴 밝기 경계

반대로 LiDAR는 책상 모서리, 폐색 경계, beam bleeding을 edge로 만들 수 있지만 영상에서는
뚜렷한 선으로 보이지 않을 수 있다. 평행한 실내 구조가 많은 경우 서로 다른 방향이 비슷한
통계 점수를 만든다.

### 3.4 z-buffer가 있어도 잘못된 대응은 제거하지 못한다

현재 z-buffer는 카메라 시점에서 뒤쪽 점을 제거하는 가시성 처리다. 따라서 동일 픽셀의
앞·뒤 점 문제는 줄이지만, **보이는 바닥 점이 카메라 영상의 다른 바닥 edge에 잘못 붙는
문제**까지 해결하지는 않는다. 가시성은 필요조건이지 대응 정답을 보장하는 조건이 아니다.

## 4. 알고리즘적 원인

현재 복합 목적함수는 개념적으로 다음과 같다.

```text
J = 0.55 * NID²
  + 0.25 * edge_distance²
  + 0.20 * structural_line²
```

NID 내부에서는 LiDAR geometry value와 카메라 gradient value의 soft joint histogram을
구성한다. 이 값은 장면의 구조적 상관을 측정하지만, 어떤 3D 선이 어떤 2D 선에 대응하는지
알려주지 않는다. 따라서 “전체 통계가 좋아진 자세”와 “실제 경계가 맞는 자세”가 다를 수
있다.

현재 구조선 residual도 방향, endpoint distance, overlap을 사용하지만, 승인 가능한
3D–2D 선분 correspondence 수가 충분한지, 수평·수직 구조가 모두 관측되는지, 영상의 여러
영역에 분포하는지 검사하지 않는다. 장당 몇 개의 우연한 선분만 있어도 PASS가 가능하다.

인접 8개 후보 Gaussian 보정과 contiguous basin 선택은 raw 단일 후보의 노이즈를 줄이는
방법이다. 이번 결과의 raw best와 corrected best 모두 같은 잘못된 영역(`yaw=60°,
down=90°`)이었으므로, 후보 보정이 false PASS의 근본 원인은 아니다.

## 5. 조명 켜짐 조건에 대한 판단

운용 조건을 조명 켜짐으로 고정하는 방향은 적절하다. 조명 켜짐은 다음 장점이 있다.

- 카메라의 texture와 구조선이 안정적임
- LiDAR 점군과 시각 구조를 사람이 검증하기 쉬움
- IR/night vision보다 edge topology가 일정함

다만 조명 켜짐은 그림자와 반사 edge를 추가할 수 있다. 따라서 밝기 edge를 calibration의
주요 근거로 쓰기보다 3D 평면 교차선과 영상의 구조선 correspondence를 우선해야 한다.
night vision 데이터는 별도 robustness profile로 평가하고 밝은 영상과 한 번에 합쳐 최적화하지
않는다.

## 6. 관련 연구와 현재 구현의 차이

### Pandey et al. — MI 기반 targetless calibration

[AAAI 논문](https://ojs.aaai.org/index.php/AAAI/article/view/8379)은 카메라와 LiDAR의
surface intensity 사이 mutual information을 최대화한다. 충분한 수의 서로 다른 view를
사용하면 추정 분산과 평균 오차가 감소한다는 점을 실험으로 보였다.

현재 구현은 LiDAR reflectivity가 아니라 거리/normal 변화량과 카메라 Sobel gradient를
비교한다. TOFSense F2P `signal_strength`도 거리, 입사각, 포화, 내부 보정의 영향을 받을
수 있어 반복성 검증 전에는 Pandey 방식의 raw intensity로 사용할 수 없다.

### Koide et al. — direct NID refinement toolbox

[ICRA 논문](https://arxiv.org/abs/2302.05094)과 [공식 구현 가이드](https://koide3.github.io/direct_visual_lidar_calibration/collection/)는
카메라 intrinsic matrix와 distortion을 먼저 준비하고, rigid mounting과 동기 데이터,
LiDAR intensity texture를 요구한다. 자동 초기화는 2D–3D 대응 및 RANSAC을 사용한 뒤 direct
NID refinement로 진행한다.

현재 구현과의 차이는 다음과 같다.

- 과거 실행의 K는 제조사 FOV 중앙값이었으나, 현재 제품 경로는 Manual ChArUco K+D profile을 고정함
- 자동 2D–3D RANSAC 초기화가 없음
- 큰 yaw/down grid가 초기화 역할을 대신함
- 같은 고정 장면 반복을 독립 view처럼 사용함

### Yuan et al. — pixel-level natural edge calibration

[논문](https://arxiv.org/abs/2103.01627)은 LiDAR에서 단순 depth jump만 사용하는 대신,
voxel 내부 plane fitting으로 depth-continuous plane intersection edge를 추출한다. 또한
edge가 영상 전체에 고르게 분포해야 외부 파라미터가 잘 관측된다고 분석한다.

현재 구현도 평면 교차선 추출을 도입했지만, 이번 장면에서는 승인된 구조선이 장당 평균 2개로
적고 책상·벽·바닥 경계를 충분히 대표하지 못했다. 따라서 추출선을 계산했다는 것만으로
정합 가능성이 확보된 것은 아니다.

### Plane-constrained joint intrinsic/extrinsic calibration

[Plane-constrained BA 논문](https://arxiv.org/abs/2308.12629)은 여러 이미지에서 SfM으로
카메라 intrinsic과 visual points를 초기화한 뒤 LiDAR plane 제약과 함께 K·RT를 공동
최적화한다. “체커보드가 없다”는 것이 “한 장의 반복 영상만으로 K를 결정한다”는 뜻은 아니다.

## 7. 권장 수정 방향

### 7.1 1순위 — 물리 제약과 입력 gate

다음 조건을 만족하지 않으면 제품 `PASS`를 내지 않는다.

- camera center가 명시되어야 함
- 고정환경 CH1 현재 기준: `(+0.05928,-0.08105,0) m`
- 추정 center `C=-Rᵀt`의 실측 오차를 별도 계산
- 탐색 경계(`down=0/90`, yaw 경계) 선택 시 `SEARCH_BOUNDARY_HIT`
- raw/rectified 입력 상태 또는 K profile이 미확정이면 `DIAGNOSTIC_ONLY`. 명시적 raw
  계약과 해당 Manual K+D가 있으면 LDC UI 상태 `unknown` 자체는 traceability warning
- 실제 image–JSON pair ID가 없으면 pairing 경고

### 7.2 2순위 — 반복 데이터의 올바른 사용

조명 켜짐 5개 반복 scan은 다음처럼 처리한다.

1. 각 organized cell의 range를 median/MAD로 집계
2. 불안정 cell을 제외한 대표 LiDAR scan 생성
3. 대표 camera image 한 장을 기준으로 calibration
4. 나머지 4개는 repeatability와 hold-out 검증에 사용

K와 RT를 추정할 때는 같은 장면의 복사본을 5개 관측으로 세지 않는다. 구조가 다른 장면이나
센서 조립체 위치가 다른 3~5개의 동기 pair가 필요하다.

### 7.3 3순위 — geometry-first 목적함수

권장 시퀀스는 다음과 같다.

```text
평면 분할
  → 벽–바닥/벽–책상 3D 교차선
  → 카메라 LSD 구조선
  → RANSAC 또는 multi-start 2D–3D 선분 초기화
  → z-buffer 가시성 검사
  → 방향 + endpoint + overlap Ceres refinement
  → NID/edge는 보조 비용
```

NID/edge를 제거하지는 않지만 초기 자세와 PASS 판단의 주 근거로 사용하지 않는다. 특히
현재처럼 NID 55%로 두지 않고, 구조선 correspondence와 물리 제약을 먼저 통과시킨 후보만
NID refinement에 전달한다.

### 7.4 4순위 — 승인 gate 강화

최소한 다음을 결과 JSON과 시각화에 추가한다.

- 수평 구조선과 수직 구조선 각각의 inlier 수
- 구조선별 endpoint 오차, 방향 오차, overlap 비율
- image quadrant별 투영 coverage
- median 및 P90 reprojection error
- camera center 오차
- hold-out pair에서의 reprojection error
- 후보 basin 간 pose 분산 및 경계 여부

최종 PASS는 “내부 score가 낮다”가 아니라 “사용하지 않은 데이터에서도 같은 구조선에
재투영된다”를 포함해야 한다.

## 8. 검색 간격 정책

현재 15° coarse → 1° fine 정책은 유지할 수 있다. 다만 물리 prior와 목적함수 문제가
해결된 뒤에 적용한다.

```text
10°~15° 전체 coarse
→ 상위 2~3개 contiguous basin
→ 각 basin 주변 1° fine search
→ Ceres 연속 최적화
```

5° 전체 search는 원인 분석용 A/B 시험으로는 유용하지만, 잘못된 비용함수를 더 정밀하게
최소화할 뿐이다.

## 9. 구현 상태

### 현재 구현

- JSON 좌표계 및 `tilt_rad` 계약 적용
- LiDAR plane/normal/range feature 생성
- plane intersection structural edge 생성
- z-buffer visibility filter
- NID + edge + structural line 복합 비용
- contiguous basin 및 인접 8개 후보 Gaussian 보정
- Manual ChArUco K+D 고정 profile
- LDC `unknown` metadata 기록

### 미구현 또는 보강 필요

- 기본 `baseline=0.28`을 실데이터에서 허용하지 않는 입력 gate
- 동일 장면 반복의 median/MAD 집계와 독립성 판정
- 구조선 방향군/공간분포/coverage gate
- center prior를 포함한 제품 PASS gate
- hold-out reprojection validation
- 자동 2D–3D correspondence + RANSAC 초기화
- F2P signal strength 보정 후 NMI 보조 채널

## 10. 다음 실험

1. 조명 켜짐 5개 중 1개 대표 pair로 `camera-center=(0.05928,-0.08105,0)`을 명시한다.
2. 동일 profile의 Manual ChArUco K+D를 고정하고 RT만 탐색한다.
3. 설치 계약 RT와 최적화 RT를 각각 투영하여 비교한다.
4. 평면 교차선과 LSD 선분 대응 이미지를 별도 생성한다.
5. 나머지 반복 4개를 hold-out으로 사용한다.
6. 구조가 다른 위치에서 최소 3개 pair를 추가 취득한다.
7. 그 후 10°/15° coarse와 1° fine step을 비교한다.

## 11. 참고 문헌 및 구현

- [Pandey et al., AAAI — MI 기반 targetless calibration](https://ojs.aaai.org/index.php/AAAI/article/view/8379)
- [Koide et al., ICRA — direct NID toolbox](https://arxiv.org/abs/2302.05094)
- [Koide 공식 데이터 수집/캘리브레이션 절차](https://koide3.github.io/direct_visual_lidar_calibration/collection/)
- [Yuan et al., pixel-level edge calibration](https://arxiv.org/abs/2103.01627)
- [Li et al., plane-constrained joint intrinsic/extrinsic calibration](https://arxiv.org/abs/2308.12629)
- [현재 프로젝트 방법 검토](TARGETLESS_CALIBRATION_METHOD_REVIEW.md)

## 12. 수정 이력

| 버전 | 날짜 | 변경 내용 |
|---|---|---|
| 0.2 | 2026-08-14 | 카메라 중심 수평 59.28 mm 반영 경로 확인 및 수직 offset 83.05→81.05 mm 정정 기록 |
| 0.1 | 2026-08-14 | 조명 켜짐 repeat sample의 false PASS 수치·원인·문헌 비교·수정 우선순위 기록 |

# Targetless LiDAR–Camera Calibration 방법 검토

- 작성일: 2026-08-11
- 적용 대상: PNM-C16083RVQ + TOFSense F2P pan-tilt 3D scan
- 목표: 체커보드/ChArUco를 제품 Automatic Calibration의 필수 입력으로 사용하지 않고 카메라 intrinsic과 LiDAR-camera extrinsic을 추정

## 1. 기존 edge-only 방식의 한계

현재 Core는 LiDAR range discontinuity를 카메라 Canny edge의 distance transform에 맞춘다. 계산량이 작고 합성 검증이 쉽지만 다음 문제가 있다.

- 서로 다른 물체의 edge도 가까우면 동일 대응으로 처리한다.
- 텍스처, 조명, 그림자 edge가 많으면 false minimum이 생긴다.
- 반복 구조에서 180° 반대 방향도 낮은 비용을 가질 수 있다.
- 평면 내부 정보, LiDAR signal strength, surface normal과 semantic 정보는 버린다.
- 잘못된 초기 FoV가 선택되면 ±5° 지역 최적화로 복구하기 어렵다.

따라서 edge는 빠른 진단과 보조 비용으로 유지하되 제품 PASS의 단독 근거로 사용하지 않는다.

## 2. 논문 기반 대안

| 방법 | 핵심 근거 | 장점 | 현재 프로젝트 적합성 |
|---|---|---|---|
| MI/NMI/NID 직접 정합 | Pandey 등은 카메라 grayscale과 LiDAR reflectivity의 mutual information을 최대화했고, 여러 view가 증가할수록 분산과 평균 오차가 감소하는 결과를 보고했다. Taylor–Nieto는 LiDAR feature image와 카메라 영상의 NMI로 위치·자세·focal length를 함께 탐색했다. | edge 대응을 직접 만들 필요가 없고 intrinsic 공동 추정 근거가 있음 | **높음.** F2P signal strength가 반복 가능할 때 적용. raw intensity 단독 사용은 금지 |
| LiDAR feature image + correspondence + RANSAC + NID | Koide 등은 SuperGlue로 자동 초기 2D–3D 대응을 찾고 RANSAC으로 초기 RT를 만든 뒤 NID 직접 정합으로 refinement했다. 논문은 edge-alignment 방식보다 높은 정확도와 강건성을 보고한다. | 현재의 180°/넓은 초기 오차 문제를 coarse 단계에서 해결 가능 | **가장 유력.** organized pan-tilt scan을 depth/normal/signal 영상으로 렌더링 가능 |
| 자연 평면 기반 intrinsic-extrinsic | Tamas–Kato는 target 없이 공통 planar region을 이용하며 extrinsic에는 최소 한 평면, intrinsic-extrinsic에는 최소 두 평면을 사용했다. | 바닥·벽·문틀이 많은 실내 고정환경에 적합하며 기하학적 설명이 명확 | **높음.** 영상에서 대응 평면 또는 구조선을 안정적으로 얻는 단계가 필요 |
| Semantic mutual information | Jiang 등은 카메라와 LiDAR semantic 정보 사이 MI를 최대화하는 differentiable 목적함수를 제시했다. | 밝기와 LiDAR 반사도의 약한 직접 상관을 semantic 공통 표현으로 완화 | **중간/후순위.** 실내 데이터와 F2P scan에 맞는 segmentation 모델·학습 검증 필요 |
| Unified depth/flow 기반 학습법 | UniCalib은 카메라 추정 depth와 LiDAR dense depth를 통일하고 probabilistic flow, reliability map, PnP/RANSAC을 사용한다. | modality gap과 occlusion/outlier를 명시적으로 다룸 | **연구 후보.** 학습 의존성, 연산량, CV5 이식성과 현재 실내 도메인 검증 부담이 큼 |

## 3. 권장 알고리즘

현재 하드웨어와 데이터 밀도를 고려한 권장 순서는 다음과 같다.

1. **제조사 FOV 기반 K 초기화**
   - H 53°~100°, V 30°~54° 범위에서 초기 `fx,fy`를 생성한다.
   - `cx,cy`는 영상 중심 주변 ±5%로 제한한다.
   - 촬영 중 channel/resolution/zoom/focus/LDC를 고정한다.
2. **기하 기반 coarse initialization**
   - organized LiDAR에서 depth, surface normal, range discontinuity 영상을 만든다.
   - 카메라의 line/region feature와 LiDAR feature image의 후보 대응을 만들고 RANSAC/PnP 또는 multi-start로 180° 방향 모호성을 제거한다.
3. **NID/NMI 직접 refinement**
   - 우선 depth/normal 기반 NID를 사용한다.
   - F2P `signal_strength`는 거리·입사각 보정 후 반복성과 entropy gate를 통과할 때만 보조 채널로 추가한다.
4. **복합 품질 판정**
   - NID 개선률
   - plane/normal 일치
   - edge alignment
   - projected/inlier ratio
   - multi-start pose 분산
   - 독립 frame 재투영 검증

권장 목적함수의 개념은 다음과 같다.

```text
J = w_nid * NID(camera_feature, lidar_feature)
  + w_plane * plane_normal_error
  + w_edge * edge_distance
  + w_prior * mechanical_prior_error
```

Edge는 제거하지 않고 낮은 가중치의 보조항으로 유지한다.

## 4. F2P signal strength 적용 조건

Pandey 방식의 전제는 두 센서의 surface intensity 사이에 통계적 의존성이 있다는 것이다. TOFSense F2P의 `signal_strength`가 재료 reflectivity뿐 아니라 거리, 입사각, 노출, 포화와 내부 AGC의 영향을 받으면 raw 값으로 MI를 계산해서는 안 된다.

다음 conformance를 먼저 통과해야 한다.

- 동일 cell 5회 반복의 signal coefficient of variation
- 거리별 signal 감쇠 곡선
- 입사각별 signal 변화
- saturation/invalid 비율
- 정지 장면 histogram entropy
- 200/400 bps 간 rank correlation

통과하지 못하면 signal NMI를 제외하고 depth/normal NID를 사용한다.

## 5. 구현 상태와 다음 변경

완료:

- Automatic 실행기의 manual intrinsic 입력 제거
- PNM-C16083RVQ FOV 범위 기반 K 초기화
- 최소 3개 관측에서 공유 `fx,fy,cx,cy,R,t` 제한 공동 최적화
- organized LiDAR의 range discontinuity와 인접 surface-normal 변화량 특징 생성
- 카메라 gradient magnitude와 LiDAR geometry feature의 soft-histogram NID 계산
- `0.70 × NID² + 0.30 × normalized edge distance²` 복합 refinement
- LiDAR 수평축 기준 15° 간격 360° yaw multi-start
- 서로 떨어진 방향 후보의 목적함수 차이가 작을 때 `MULTISTART_AMBIGUOUS`로 거절
- intrinsic 경계, 관측 부족, NID 중첩·개선률 fail-safe
- 결과 JSON에 intrinsic 출처, NID, 복합 목적함수, multi-start 지표 기록

미구현:

- 자동 2D–3D correspondence와 PnP/RANSAC initialization
- 독립적인 plane/normal 일치 품질 gate
- F2P `signal_strength` 기반 NMI
- 독립 frame hold-out 재투영 검증

다음 우선순위는 **서로 다른 구조의 동기화 관측 3~5쌍으로 실제 NID/multi-start 관측성을 검증**하는 것이다. 그 다음 독립 plane/normal gate를 추가한다. F2P signal NMI는 conformance 결과가 확보된 뒤에만 보조항으로 추가한다.

## 6. 근거 문헌

1. G. Pandey et al., “Automatic Targetless Extrinsic Calibration of a 3D Lidar and Camera by Maximizing Mutual Information,” AAAI 2012.
   https://cvgl.stanford.edu/papers/pandey_jfr15.pdf
2. Z. Taylor and J. Nieto, “Automatic Calibration of Lidar and Camera Images using Normalized Mutual Information,” ICRA 2013.
   https://www-personal.acfr.usyd.edu.au/jnieto/Publications_files/TaylorICRA2013.pdf
3. K. Koide et al., “General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox,” ICRA 2023.
   https://staff.aist.go.jp/shuji.oishi/assets/papers/preprint/Calibration_ICRA2023.pdf
4. L. Tamas and Z. Kato, “Targetless Calibration of a Lidar - Perspective Camera Pair,” ICCV Workshops 2013.
   https://openaccess.thecvf.com/content_iccv_workshops_2013/W21/html/Tamas_Targetless_Calibration_of_2013_ICCV_paper.html
5. P. Jiang et al., “Calibrating LiDAR and Camera using Semantic Mutual Information,” 2021.
   https://arxiv.org/abs/2104.12023
6. S. Han et al., “UniCalib: Targetless LiDAR-camera Calibration via Probabilistic Flow on Unified Depth Representations,” WACV 2026.
   https://openaccess.thecvf.com/content/WACV2026/html/Han_UniCalib_Targetless_LiDAR-camera_Calibration_via_Probabilistic_Flow_on_Unified_Depth_WACV_2026_paper.html

## 7. 수정 이력

| 버전 | 날짜 | 변경 내용 |
|---|---|---|
| 0.1 | 2026-08-11 | edge-only 대안, 논문 근거, 프로젝트 적용 우선순위 및 F2P signal conformance 조건 최초 작성 |
| 0.2 | 2026-08-11 | depth/normal geometry NID, 360° yaw multi-start, 복합 목적함수 및 모호성 fail-safe 구현 상태 반영 |

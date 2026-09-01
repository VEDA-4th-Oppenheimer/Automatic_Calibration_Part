# Automatic Calibration 제품 운용 정책

- 작성일: 2026-08-20
- 최종 수정: 2026-08-24
- 적용 범위: `run_real_calibration`, Calibration Core 실데이터 경로
- 대상: PNM-C16083RVQ 채널별 영상 + TOFSense F2P pan-tilt LiDAR
- 상태: MVP 운용 기준 v1.3 (`CANDIDATE_RT`, 제품 승인 전·활성 금지)

이 문서는 여러 실험 문서에 흩어져 있던 현재 운용 결정을 하나로 고정한다. 과거 실험에서
사용한 제조사 FOV 초기화, K+RT 공동 추정, 단일 장면 `PASS`는 연구·진단 기록으로 보존하지만
현재 제품 경로의 승인 기준으로 사용하지 않는다.

## 1. 제품 경로의 입력과 추정 대상

제품 경로는 **카메라 내부 파라미터를 추정하지 않고**, 같은 광학 상태에서 확보한 카메라
profile의 `K + D`를 고정한 채 LiDAR–camera 외부 파라미터 `R,t`만 추정한다.

| 항목 | 제품 정책 |
|---|---|
| `K` (`fx, fy, cx, cy`) | 동일 채널·해상도·zoom·focus의 Manual ChArUco profile 사용 |
| `D` (왜곡 계수) | 위 profile의 distortion 사용 |
| ChArUco 촬영 | 제품 실행 중 요구하지 않음. profile을 사전에 만들거나 갱신할 때만 사용 |
| LDC | LDC가 K/D를 대체하지 않음. 현재 실험 경로는 raw 영상 + Manual D undistort |
| 추정 대상 | `T_camera_lidar = (R,t)` 외부 파라미터 |
| K+RT 공동 추정 | MVP 제품 경로에서 보류. 연구/진단 flag로만 보존 |
| 제조사 FOV K | 초기화·민감도 비교용 진단값. 제품 승인 입력으로 대체 사용하지 않음 |

Manual profile이 없거나 해상도·zoom·focus·raw/rectified 상태가 profile과 일치하지 않으면
제품 승인을 진행하지 않는다. 필요한 경우 실행은 진단 모드로 남길 수 있지만 결과 RT를
활성값으로 승격하지 않는다.

## 2. 영상 왜곡 계약

현재 사용 profile의 입력은 `image-distortion-state=raw`로 기록하고, Manual `K + D`로
`undistort`한 영상을 projection과 정합에 사용한다. 카메라가 LDC를 제공하더라도 LDC는
출력 영상을 바꾸는 기능일 뿐 K를 없애지 않는다.

- LDC가 `false` 또는 사용할 수 없음: raw 영상에 Manual D를 적용한다.
- LDC가 `true`: 실제 rectified 출력에 맞는 별도 K profile을 사용하고 중복 보정하지 않는다.
- LDC UI 상태가 `unknown`이어도 수집 계약이 명시적으로 `raw`이고 해당 raw profile의
  Manual `K+D`를 적용했다면 traceability warning으로 기록하고 후보 검증을 계속할 수 있다.
- raw/rectified 입력 상태 자체가 `unknown`이면 `DIAGNOSTIC_ONLY`로 취급한다.

따라서 서로 다른 LDC 상태, zoom/focus, 영상 해상도의 이미지와 K/D를 섞어 reprojection
오차를 비교하지 않는다.

## 3. 초기 방향 탐색의 역할

현재 coarse yaw/down/optical-roll 탐색과 인접 8개 후보 보정은 다음 두 목적에만 사용한다.

1. 광범위한 초기 방향에서 정합 가능 후보와 반복적인 local minimum을 진단한다.
2. 제한된 Ceres `R,t` refinement의 시작 후보를 만든다.

탐색 결과가 자연 장면 구조와 비슷해 보이거나 내부 score가 낮아도 그것만으로 제품 RT를
승인하지 않는다. 평면·벽·바닥처럼 반복/대칭 구조가 있는 장면에서는 방향이 다른 후보가
비슷한 score를 만들 수 있고, edge/NID는 잘못된 표면을 선택할 수 있다. 따라서 탐색 경계에
붙은 후보, 서로 다른 basin, 구조 방향 불일치가 있으면 `AMBIGUOUS` 또는 `DIAGNOSTIC_ONLY`
상태로 남긴다.

제품 경로의 권장 순서는 다음과 같다.

```text
Manual K+D profile 검증
  → 입력/좌표계/가시성 gate
  → coarse score map + 인접 8개 후보 가중치
  → 상위 3개 contiguous basin
  → basin별 5° local search
  → 1° local search
  → 서로 분리된 최대 3개 finalist에 Ceres R,t refinement
  → objective/TESL/confidence 계층 선택 + dual-margin ambiguity gate
  → training/hold-out/반복성/독립 기준 검증
  → 승인 또는 기존 active RT 유지
```

staged 경로에서는 coarse/5°/1° 단계가 고정 K+D로 점수만 계산한다. 서로 분리된 최대
3개 1° seed에 Ceres를 실행하고, training scene gate → core success → 자세 범위 →
absolute support → matching objective → TESL → confidence 순서로 비교한다. objective가
차순위보다 2% 이상 우세하면 낮은 objective를 사용하고, 2% 미만 near-tie에서만 TESL
10% 차이와 confidence를 tie-break로 사용한다. threshold가 포함된 pairwise comparator를
사용하지 않고 명시적 단계 선택을 반복해 입력 순서와 무관한 결정론을 보장한다.

선택 후보와 15°보다 떨어진 최강 경쟁 후보의 objective margin과 confidence margin이
동시에 각각 2% 미만이면 `FINALIST_AMBIGUOUS`로 실패한다. 선택 후보의 visible edge 또는
NID support가 경쟁 후보의 60% 미만이어도 실패한다. basin이 없거나 refinement가 실패하면
다른 `PASS` 후보를 임의로 활성화하지 않는다.

NID 상대 coverage hard gate는 5°/1°의 **같은 local yaw window 안에서만** 적용한다.
서로 다른 360° 시야 영역의 NID 점 수 최대값은 final objective/confidence의 soft
진단값으로만 사용한다. 절대 support·spatial distribution gate는 그대로 유지한다.

coarse 후보 점수는 자기 raw score만 사용하지 않고 3×3 인접 후보(가운데 제외)의 거리
가중 평균을 함께 반영한다. 이 값은 연속적인 고득점 영역을 찾기 위한 선택 보정값이지
제품 승인 근거가 아니다.

## 4. 품질 상태와 실패 처리

`PASS`라는 단어는 반드시 범위를 함께 기록한다.

| 상태 | 의미 | 활성 RT 승격 |
|---|---|---:|
| `INTERNAL_GATE_PASS` | 현재 입력의 score·overlap·구조 gate 통과 | 불가 |
| `CANDIDATE_RT` | training 및 제한된 hold-out에서 재현된 후보 | 불가 |
| `PRODUCT_APPROVED_RT` | 독립 기준과 반복성·실패 안전성까지 통과 | 가능 |
| `DIAGNOSTIC_ONLY` / `FAIL` | 원인 분석용 결과 | 불가 |

실행 결과 JSON은 `status`와 별도로 `internal_gate_status`, `candidate_rt_status`,
`product_approved_rt_status`, `activation_allowed`를 기록한다. `INTERNAL_GATE_PASS`와
`CANDIDATE_RT`를 하나의 `PASS` 문자열로 합치지 않는다.

품질 gate가 실패하면 다음을 수행한다.

1. 실패 후보 RT와 debug projection을 `candidate/diagnostic`에 보관한다.
2. 기존 `active` RT를 유지하고 후단 perception에 새 값을 전달하지 않는다.
3. `reason_code`와 입력 profile, search boundary, hold-out 결과를 report에 기록한다.
4. 원인이 입력 품질·pair·LDC/profile·관측성으로 분류될 때만 제한된 재수집/재시도를 한다.
5. threshold를 낮추거나 실패 후보를 강제로 승격하지 않는다.

제품 승인을 위해서는 최소한 다음을 별도로 확인한다.

- 최적화에 사용하지 않은 hold-out 3쌍 이상에서 동일 `R,t`가 재현됨
- 동일 설치 반복 10회에서 회전 표준편차 `≤0.2°`, 이동 표준편차 `≤10 mm`
- 초기 perturbation 복원 성공률 `≥90%`
- 독립 `T_camera_lidar` 기준(jig/CAD/survey 또는 LiDAR-visible target)과의 오차
- 퇴화 장면·잘못된 profile·pair 오류에서 false activation 0건

2026-08-24 binary finalist 검증에서 CH1 build17~21의 선택 RT와 yaw가 80°/87° 떨어진
두 viable 후보가 모두 hold-out `2/2 PASS`했다. 이후 학습과 같은 목적함수와 모든 후보가
공유하는 coverage 분모로 같은 pass-ratio tier를 비교했다. 선택 167°는 87°/−106°
후보보다 각각 `6.491%`/`12.357%` 우수해 기존 2% margin을 통과했다. 현재 상태는
`CANDIDATE_RT / PASS`지만, 동일 환경 hold-out은 독립 설치·물리 참값 검증을 대체하지
않는다. 따라서 `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`를 유지한다.

Training과 hold-out의 Manhattan 영상 특징은 finalist별 동일 training seed prior로
고정해야 한다. Hold-out에서 refined candidate RT를 사용해 수직 소실점 축을 다시
선택하면 후보 비교 대상 자체가 달라지므로 제품 경로에서 금지한다. 2026-08-24 수정 후
1회 검증에서는 기존 세 fixture의 결과가 유지됐지만, 이는 제품 정확도 승격 근거가 아닌
평가 계약 일관성 확인이다.

## 5. 범위 밖 항목

- K+RT 공동 추정은 MVP 이후 연구 항목으로 보류한다.
- F2P `signal_strength` NMI는 반복성 conformance가 통과할 때까지 가중치 0이다.
- 최종 Top-view/Qt 시각화는 별도 Qt 세션의 산출물이며 Core 승인과 분리한다.
- OpenSDK/CV5 adapter와 실제 actuator 연동은 담당 범위가 확인된 뒤 별도로 검증한다.

## 6. 문서 우선순위

이 정책과 과거 실험 기록이 충돌하면 이 문서의 MVP 운용 정책을 따른다. 실험 기록의 수치와
실패 원인은 변경하지 않고, 해당 결과가 현재 제품 정책의 근거가 아닌 진단·연구 기록임을
명시한다.

## 7. 수정 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-20 | Manual ChArUco K+D 고정, K+RT 공동 추정 보류, 진단/후보/제품 승인 상태 분리, Qt Top-view 범위 분리 정책을 최초 확정 |
| 2026-08-20 | staged 탐색(coarse → top-3 basin → 5° → 1° → 최종 seed 단일 Ceres), basin fallback 제거, RT perturbation 도구 및 상태 필드 정책 추가 |
| 2026-08-23 | 실제 구현에 맞춰 최대 3개 Ceres finalist, basin-local NID hard gate, objective/TESL/confidence 계층 선택과 dual-margin ambiguity 정책 반영 |
| 2026-08-23 | 명시적 raw 입력 계약이 있는 경우 `ldc_enabled=unknown`은 경고로 기록하되 후보 검증을 허용하도록 LDC 정책 정정 |
| 2026-08-23 | 현재 최고 상태를 3-training/2-hold-out `CANDIDATE_RT`, 제품 승인 전으로 갱신 |
| 2026-08-24 | 모든 separated finalist를 hold-out에 고정 적용하는 fail-closed 정책과 binary 중간 `NOT_CANDIDATE_RT` 반영 |
| 2026-08-24 | 학습 동일 연속 목적함수·공통 coverage·기존 2% margin을 적용해 `CANDIDATE_RT`로 갱신; 제품 활성 금지 유지 |
| 2026-08-24 | Manhattan 영상 특징을 finalist별 training seed prior에 고정하는 training/hold-out 일관성 정책 추가 |

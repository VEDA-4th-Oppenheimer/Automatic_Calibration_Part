# Manual Marker vs Automatic 비교 프로토콜

- 작성일: 2026-07-30
- 수정 이력:
  - 2026-07-31 — Galaxy Tab S7 전체 화면 ChArUco를 기본 현장 매체로 변경
  - 2026-07-30 — LiDAR 없는 2D ChArUco Manual Calibration으로 전면 재설계
  - 2026-08-14 — session-const-env 태블릿 display geometry 예비 RT의 비교 제한 추가

> 수동 RT 기준값을 새로 취득하거나 다른 세션에서 작업을 이어받을 때는
> MANUAL_RT_REFERENCE_WORKFLOW.md를 먼저 따른다.

> session-const-env의 실제 계산값과 입력/잔차는
> [`SESSION_CONST_ENV_CALIBRATION_RECORD.md`](SESSION_CONST_ENV_CALIBRATION_RECORD.md)를 따른다.

## 1. 항상 비교 가능한 항목

| 지표 | Manual Marker | Automatic Targetless |
|---|---|---|
| 준비 시간 | Tab S7 전체 화면 설정·100 mm 검증·배치 | Scanner 설치·homing |
| 데이터 취득 시간 | 여러 각도의 image 촬영 | Pan-tilt sweep |
| 필요한 작업자 수 | 기록 | 기록 |
| 표적 필요 여부 | 필요 | 불필요 |
| 입력 frame/scene 수 | ChArUco image 수 | Camera–LiDAR scene 수 |
| 실패·재시도 횟수 | 기록 | 기록 |
| 결과 계산 시간 | 기록 | 기록 |
| 내부 품질 | Reprojection RMS | Edge residual/quality gate |
| 반복성 | Intrinsic/board pose 반복 통계 | `T_camera_lidar` 반복 통계 |

Reprojection RMS와 targetless edge residual은 물리적 의미가 다르므로 숫자만 직접
우열 비교하지 않는다.

## 2. 조건부로 비교 가능한 6-DoF

Marker image만으로는 `T_camera_marker_board`만 구할 수 있다. Automatic 결과인
`T_camera_lidar`와 rotation/translation을 비교하려면 독립적인
`T_lidar_marker_board`가 필요하다.

조건:

1. 전체 화면 ChArUco를 표시한 Tab S7이 rigid jig에 고정됨
2. Board–LiDAR frame 변환을 CAD/정밀 측정/survey로 확보
3. Manual과 Automatic 측정 중 camera·LiDAR 설치 상태가 변하지 않음
4. 모든 transform이 같은 좌표축·단위·child-to-parent convention을 사용
5. Reference uncertainty를 결과에 함께 기록

조건을 만족하지 않으면 6-DoF 차이는 보고하지 않는다.

태블릿 display plane과 `display_spec`/`board_config`를 조합한
`ESTIMATED_GEOMETRY_CORRECTED` 결과는 위 독립 조건을 충족하기 전까지 진단용으로만
표시한다. 이 값을 Automatic 결과에서 역산하거나 absolute ground truth로 승격하지
않는다.

## 3. 권장 반복시험

- Manual intrinsic: 서로 다른 image set으로 3회
- Manual board pose: 동일 jig에서 10회
- Automatic: 동일 설치 상태에서 10회
- 거리·조명·scene 조건별 반복

보고값:

- Rotation geodesic difference [deg]
- Translation norm difference [m]
- 방법별 rotation/translation 반복 표준편차
- 실패율과 재시도율
- 현장 총 소요 시간

Manual 결과를 absolute ground truth로 단정하지 않는다. 정밀 reference의 불확실성을
포함해 해석한다.

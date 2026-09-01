# Reference-Anchored Local Verification 핵심 시각 증거

이 디렉터리는 2026-08-31 local verification의 전체 generated 산출물 대신,
새 PC에서도 결과를 빠르게 이해할 수 있도록 build별 비교 시트만 보존한다.

각 contact sheet는 왼쪽부터 다음 세 pose를 비교한다.

1. `A_nominal`: Manual Reference RT, offset `(0,0,0)`
2. `B_boundary_joint_winner`: 공동 점수 경계 후보 `(yaw=1, down=1, roll=5)`
3. `C_robust_interior`: 경계가 아닌 비교 후보 `(yaw=1, down=3, roll=4)`

파일:

- `key_images/01_build45_pose_comparison.png`
- `key_images/02_build46_pose_comparison.png`
- `key_images/03_build48_pose_comparison.png`
- `key_images/04_build49_pose_comparison.png`

주의:

- 이 비교는 고정 pose renderer와 기존 cached score를 이용한 시각 감사다.
- B 또는 C가 더 좋아 보이더라도 제품 RT로 승격한 결과가 아니다.
- 공동 winner가 탐색 경계에 있고 build별 최적 후보가 달라 최종 판정은
  `OBJECTIVE_DRIFT_TO_SEARCH_BOUNDARY` 및 `SCENE_DEPENDENT_LOCAL_OBJECTIVE`이다.


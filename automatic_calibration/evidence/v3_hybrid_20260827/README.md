# V3 Hybrid Analyzer evidence

작성일: 2026-08-27

이 디렉터리는 큰 PNG/PLY/OBJ 대신 검토에 필요한 최소 수치만 보존한다.

- `standalone_directional_summary.csv`: 설치 구성 A/B의 CH1 standalone Top-3 yaw recall 및 analyzer runtime
- `case_c_r4_metrics.json`: Case C build22/23 training + build24 hold-out의 E2E runtime, 후보 수, B0 대비 RT 차이, acceptance 상태

원본 실행 산출물은 개발 워크스페이스의 다음 경로에 있다.

- `automatic_calibration/generated/v3_hybrid_eval_directional_v3/`
- `automatic_calibration/generated/v3_case_c_build22_24_final_e2e_r4/`

B0 비교 결과:

- `develop/automatic_calibration/generated/jenkins_scene0_ch1_primary_build22_24/calibration_result.json`

주의: 구성 A의 reference yaw 169°와 구성 B의 177°는 B0 diagnostic reference이며 독립 측량 ground truth가 아니다.

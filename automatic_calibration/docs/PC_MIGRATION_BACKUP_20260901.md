# PC 교체용 프로젝트 백업 및 복구 가이드

- 작성일: 2026-09-01
- 목적: 기존 PC의 Automatic Calibration 개발 이력, 코드, 문서, 실험 결과와 원본 데이터를 새 PC에서 재구성한다.
- Git 아카이브 브랜치: `archive/pc-migration-20260901`
- Drive 폴더: `auto_calib_pc_migration_20260901`

## 1. 백업 시점의 핵심 상태

1. `develop`은 2026-08-24에 동결한 B0 기준선이다.
2. T1/T2/V3 analyzer 실험은 계산량 절감 가능성은 확인했지만, 서로 다른 장면에서 정답 방향을 안정적으로 식별하지 못해 제품 경로로 승격하지 않았다.
3. V3 R1~R7 결과는 `V3_ANALYZER_EXPERIMENT_CLOSEOUT_20260831.md`와 `v3_analyzer_closeout_20260831/`의 비교 이미지·로그에 보존했다.
4. 이후 제품 후보 방향은 Manual Reference RT 주변의 `reference-anchored-local` 검증이다. 이 경로도 아직 engineering diagnostic이며 `product_rt_promoted=false`이다.
5. 최신 local verification에서는 여러 build의 공동 목적함수가 탐색 경계로 이동했고, build별 최적 후보도 달랐다. 따라서 `OBJECTIVE_DRIFT_TO_SEARCH_BOUNDARY`와 `SCENE_DEPENDENT_LOCAL_OBJECTIVE`로 판정했으며 RT를 승격하지 않았다.
6. Manual Projection Refiner는 별도 브랜치에서 구현·진단 중이다. 외부 SensorsCalibration 복사본은 출처·라이선스 확인 전이라 Git이 아니라 Drive의 working-delta 묶음에만 보존한다.

## 2. Git에서 먼저 확인할 브랜치

| 역할 | 브랜치 | 백업 시점 커밋 |
|---|---|---|
| 기준선 | `develop` | `f684cd6` |
| V3 종합 아카이브 | `archive/v3-analyzer-experiments-20260831` | `4f4a13b` |
| PC 교체용 안내·핵심 증거 | `archive/pc-migration-20260901` | 이 문서를 포함한 최신 커밋 |
| Reference-anchored local verification | `feature/reference-anchored-local-verification` | `ed445f0` |
| V3 R7/제품 경로 정책 | `exp-v3-r7-multi-capture-consensus-audit` | `2ab3c97` |
| Manual Projection Refiner | `exp-manual-reference-refiner` | `43ab8d4` 이후 백업 커밋 |
| 교차 도메인 edge 후속 계획 | `exp-tripod-reference-recapture` | `8cf92a8` 이후 문서 커밋 |
| T1 구조 analyzer | `codex/exp-structural-analyzer-20260824` | `64b79a6` |
| T2 파노라마 analyzer | `codex/exp-panorama-analyzer-20260824` | `a098def` |
| 초기 V3 hybrid | `codex/exp-hybrid-analyzer-v3-20260827` | `21f816a` |

그 밖의 `exp-v3-*`, `exp-cross-modal-score`, reference 관련 실험 브랜치는 병합하지 않고 원격 브랜치와 Git bundle에 그대로 보존한다. 실험 결과를 비교할 때는 해당 브랜치의 문서와 커밋을 직접 확인한다.

## 3. 이해를 돕는 핵심 문서·이미지

### V3 analyzer

- 종합 결론: `V3_ANALYZER_EXPERIMENT_CLOSEOUT_20260831.md`
- B0/T1/T2/V3의 의미와 득실: `ANALYZER_EXPERIMENT_HISTORY_AND_V3_HYBRID_PLAN_20260826.md`
- 점수 구조의 한계와 다음 구현 순서: `CROSS_MODAL_EDGE_SCORING_ANALYSIS_AND_IMPLEMENTATION_PLAN_20260827.md`
- 시각 증거: `automatic_calibration/evidence/v3_analyzer_closeout_20260831/key_images/`
  - manual reference와 V3 결과
  - build50 wrong-branch 오투영
  - NID-off 비교
  - directional-edge 회귀
  - LiDAR panorama

### Reference-anchored local verification

- 구현 계약: `REFERENCE_ANCHORED_LOCAL_VERIFICATION_MVP.md`
- 공동 점수 감사: `LOCAL_VERIFICATION_R1_JOINT_SCORE_AUDIT_REPORT.md`
- 고정 pose 시각 감사: `LOCAL_VERIFICATION_R2_FIXED_POSE_VISUAL_AUDIT_REPORT.md`
- 비교 이미지: `automatic_calibration/evidence/reference_anchored_local_verification_20260831/key_images/`
  - build45/46/48/49 각각 nominal, boundary winner, robust interior를 한 장에서 비교한다.

### Manual Projection Refiner

- 구현 설명: 아카이브의 `MANUAL_PROJECTION_REFINER.md`; 원본 브랜치 경로는 `manual_calibration/docs/MANUAL_PROJECTION_REFINER.md`
- 대표 진단 이미지: `automatic_calibration/evidence/manual_projection_refiner_20260830/`
- 이 결과는 manual reference 주변의 진단 도구이며 자동 제품 RT 승격 증거가 아니다.

## 4. Drive 백업 구조

```text
auto_calib_pc_migration_20260901/
├── 01_git/
│   ├── auto_calib_all_refs_20260901.bundle
│   └── qt_all_refs_20260901.bundle
├── 02_docs_and_key_evidence/
│   ├── auto_calib_docs_key_evidence_20260901.zip
│   └── auto_calib_numeric_results_20260901.tar.gz
├── 03_raw_and_manual_data/
│   ├── develop_jenkins_capture_20260901.tar.gz
│   ├── develop_real_calibration_20260901.tar.gz
│   ├── manual_calibration_20260901.tar.gz
│   └── non_fixed_environment_archive_20260901.tar.gz
└── 04_uncommitted_working_delta/
    └── auto_calib_working_delta_20260901.zip
```

Git bundle은 모든 커밋·브랜치를 복구하기 위한 오프라인 안전장치다. 원본 데이터 묶음은 Git에서 제외한 이미지, JSON, PCD와 manual calibration 산출물을 보존한다. working delta는 Git에 넣지 않은 외부 소스 복사본, 임시 진단 데이터와 로컬 스크립트를 보존한다.

`auto_calib_numeric_results_20260901.tar.gz`는 generated 디렉터리 전체를 복사하지
않고 JSON, CSV, log, Markdown, text 결과 9,000개 항목을 경로 보존 상태로 모은
수치 증거 묶음이다. `/workspace/...`를 가리키던 중복 WSL 심볼릭 링크 22개는 제외했고,
그 링크의 실제 입력 파일은 raw-data 묶음에 보존한다.

## 5. 의도적으로 전체 백업하지 않은 항목

`automatic_calibration/generated/`와 analyzer worktree의 `generated/`에는 약 수십 GB의 반복 PLY/OBJ/PNG가 있다. 대부분 동일 원본과 Git 코드로 재생성할 수 있으므로 다음만 영구 보존한다.

- 판정 근거가 되는 Markdown, JSON, CSV, log
- 대표 matching/reprojection/contact-sheet 이미지
- 원본 camera image, LiDAR JSON/PCD 및 manual calibration 데이터

빌드 캐시, 실행 바이너리, 수천 개의 중복 PLY/OBJ와 모든 중간 PNG는 제외한다. 새 PC에서 특정 run을 다시 확인해야 하면 원본 데이터와 해당 브랜치로 재실행한다.

## 6. 새 PC 복구 순서

### GitHub 사용

```bash
git clone https://github.com/VEDA-4th-Oppenheimer/Automatic_Calibration_Part.git
cd Automatic_Calibration_Part
git fetch --all --tags
git switch archive/pc-migration-20260901
```

### Git bundle 사용

```bash
git clone auto_calib_all_refs_20260901.bundle Automatic_Calibration_Part
cd Automatic_Calibration_Part
git remote add origin https://github.com/VEDA-4th-Oppenheimer/Automatic_Calibration_Part.git
git fetch origin
git switch archive/pc-migration-20260901
```

1. Drive의 데이터 압축 파일을 기존 상대 경로와 동일하게 복원한다.
2. `Dockerfile`, `compose.yaml`, `DEVELOPMENT_ENVIRONMENT.md`를 기준으로 Ubuntu 개발 환경을 구성한다.
3. 기준 동작은 먼저 `develop`에서 확인한다.
4. V3 결과 재현은 해당 실험 브랜치에서 수행한다.
5. 현재 제품 후보 개발은 `feature/reference-anchored-local-verification`과 `exp-manual-reference-refiner`를 각각 별도 worktree로 만든 뒤 진행한다.
6. `generated/`는 원본 데이터와 코드로 필요한 run만 다시 생성한다.

## 7. 무결성·보안

- Drive에 올린 파일의 SHA-256은 백업 루트의 checksum manifest와 대조한다.
- 카메라 비밀번호, GitHub token, Docker credential, `.env`는 백업 묶음에 넣지 않는다.
- 외부 SensorsCalibration 복사본은 라이선스와 원본 revision을 확인한 뒤에만 Git에 반영한다.
- `product_rt_promoted=false`인 결과를 새 PC에서 제품 RT로 오인하지 않는다.

## 8. PC 교체 후 첫 확인 체크리스트

- [ ] GitHub에서 위 핵심 브랜치가 모두 보인다.
- [ ] Git bundle로 오프라인 clone이 된다.
- [ ] V3 종합 보고서의 6개 핵심 이미지가 열린다.
- [ ] local verification의 build45/46/48/49 contact sheet가 열린다.
- [ ] Jenkins/real/manual 원본 데이터 경로가 복원된다.
- [ ] B0 테스트와 reference-anchored smoke test가 새 PC에서 실행된다.
- [ ] 생성 결과는 제품 PASS가 아니라 기존 판정 상태를 그대로 유지한다.

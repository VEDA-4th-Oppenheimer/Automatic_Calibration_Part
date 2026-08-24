# CH1 전체 데이터 활용 실행 결과 보고서

- 실행일: 2026-08-24 (KST)
- 대상: `data/jenkins-capture/scene0`, CH1 only
- 실행 환경: Docker `auto-calib-dev:ubuntu-latest`, `/workspace-build`
- 공통 K+D: `manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json`
- distortion: `raw` 입력에 고정 Manual radtan K+D 적용
- LDC: `unknown`, zoom/focus: `true`
- camera center prior: `(0.05928, -0.08105, 0) m`
- 제품 활성화: 금지 (`activation_allowed=false`)

## 1. 요약 판정

계획대로 build5~24의 12개 CH1 package를 삭제하거나 하나의 optimizer에 섞지 않고
각 역할로 사용했다.

| 단계 | 입력 | exit | 결과 | 핵심 수치 |
|---|---|---:|---|---|
| ChArUco audit | 24 target, build5~24 chair/monitor | 22×0, 2×2 | `22 PASS + 2 EXPECTED FAIL` | build17 monitor 0 corner, build18 chair 5 corner |
| Case C primary | build22·23 train → build24 hold-out | 0 | `CANDIDATE_RT / PASS` | train 2/2, hold-out 1/1, margin 2.9436% |
| Case A baseline | build5/8/9 train → build10 hold-out | 3 | `FAIL / FINALIST_AMBIGUOUS` | train 3/3, hold-out 1/1, margin 1.8209% |
| Case B stress | build17/18/19 train → build20/21 hold-out | 0 | `CANDIDATE_RT / PASS` | train 3/3, hold-out 2/2, margin 6.4912% |
| Case C RT fixed→A | build5~10 전체, 재추정 없음 | 0 | `INTERNAL_GATE_PASS` | 4/4 scene |
| Case C RT fixed→B | build17~21 전체, 재추정 없음 | 0 | `INTERNAL_GATE_PASS` | 5/5 scene |

현재 결과에서 가장 신뢰할 primary 후보는 Case C build22·23 추정값이다. 그러나 Case A와
Case B에서 서로 다른 scene/조건에서도 내부 gate가 통과한 것만으로 물리적 RT 정답을
증명할 수 없으므로 제품 승격은 하지 않는다.

## 2. Step 0~2 입력 및 camera-side audit

### 2.1 입력 무결성

- package directory: 12개
- 각 package: CH1~CH4 image, JSON, PCD, manifest 존재
- JSON measurement: package당 40,400
- checksum/out-of-range 오류: 0
- valid JSON sample: 40,038~40,190
- PCD: `x y z`만 존재하며 target label 없음
- 이번 automatic 실행은 JSON을 사용하고 PCD는 좌표/시각화 교차검사로 사용

### 2.2 ChArUco audit

전체 frame에서 두 보드가 동일 ID로 존재하므로 ROI로 chair/monitor를 분리했다.

| target | ROI 정책 | 결과 |
|---|---|---|
| chair build5/8/9/10 | `850,1200,700,320` | 모두 PASS |
| chair build17 | `1650,1200,650,320` | PASS |
| chair build18 | `1500,1000,900,500` | FAIL, 5 corner < minimum 6 |
| chair build19~24 | `1200,1200,800,320` | 모두 PASS |
| monitor build5~24 | `2090,700,500,650` | build17만 FAIL, 나머지 PASS |

실제 output:

- audit root: `/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/manual_calibration/output/jenkins-scene0/ch1-all-build-charuco-audit`
- 예상 FAIL: [build17 monitor result](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/manual_calibration/output/jenkins-scene0/ch1-all-build-charuco-audit/build17/monitor/marker_pose_result.json), [build18 chair result](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/manual_calibration/output/jenkins-scene0/ch1-all-build-charuco-audit/build18/chair/marker_pose_result.json)

이 두 입력을 통과시키기 위해 corner threshold를 낮추지 않았다. ChArUco pose는
`T_camera_marker_board` camera-side 진단이며 automatic targetless 후보 점수에는 넣지 않았다.

## 3. Case C primary 결과

입력은 pair index 9~11이며 `--holdout-count 1`로 build22·23만 학습에 사용하고 build24를
평가에 남겼다.

- output: [Case C calibration_result.json](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration/generated/jenkins_scene0_ch1_primary_build22_24/calibration_result.json)
- status: `CANDIDATE_RT`
- reason: `PASS`
- training: 2/2 PASS
- hold-out: 1/1 PASS
- selected downward: `42.0°` (refined `42.0°`)
- selected optical roll: `3.0°`
- seed yaw: `-183.0°` (circular equivalent `177.0°`)
- selected hold-out objective: `0.8006294`
- minimum separated hold-out margin: `0.0294363` = `2.9436%`
- product state: `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`
- estimated/visualized translation: approximately `[0.056074, 0.061174, 0.056539] m`

생성된 주요 시각화와 score map:

- [Case C matching scene 0](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration/generated/jenkins_scene0_ch1_primary_build22_24/matching_scene_0.png)
- [Case C 3D preview scene 0](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration/generated/jenkins_scene0_ch1_primary_build22_24/scene_0_colorized_lidar_3d_preview.png)
- [Case C orientation full search CSV](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration/generated/jenkins_scene0_ch1_primary_build22_24/orientation_full_search.csv)
- [Case C 1° search CSV](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration/generated/jenkins_scene0_ch1_primary_build22_24/search_1deg_scores.csv)

Case C는 현재 후보로 보존하지만, build24는 개발 중 사람이 marker 결과를 확인한 temporal
hold-out이므로 sealed product hold-out은 아니다.

## 4. Case A baseline 결과

입력은 pair index 0~3이며 build5/8/9 학습, build10 hold-out이다. build10의 CH1 image가
build9과 byte-identical이므로 독립 hold-out으로 해석하지 않는다.

- output: [Case A calibration_result.json](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration/generated/jenkins_scene0_ch1_baseline_build5_10/calibration_result.json)
- exit: `3` (정상 품질 거절)
- status: `FAIL`
- reason: `FINALIST_AMBIGUOUS`
- training: 3/3 PASS
- hold-out: 1/1 scene gate PASS
- selected downward: `36.0°` (refined 약 `36.0009°`)
- selected optical roll: `4.0°`
- seed yaw: `-186.0°`
- selected hold-out objective: `0.7524322`
- minimum separated hold-out margin: `0.0182090` = `1.8209%`
- 요구 margin: `2%`
- product state: `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`

실패 원인은 입력 파싱이나 Docker 오류가 아니라, hold-out에서 경쟁 finalist와의 목적함수
차이가 2% 미만이라 후보를 하나로 확정하지 못한 것이다. 이 결과는 계획상 정상적인
fail-closed 회귀 결과다.

## 5. Case B stress 결과

입력은 pair index 4~8이며 build17/18/19 학습, build20/21 hold-out이다. 편집 영상·근거리
보드·사람 포함 조건을 제품 입력으로 승격하지 않고 stress 진단으로 사용했다.

- output: [Case B calibration_result.json](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration/generated/jenkins_scene0_ch1_stress_build17_21/calibration_result.json)
- status: `CANDIDATE_RT`
- reason: `PASS`
- training: 3/3 PASS
- hold-out: 2/2 PASS
- selected downward: `37.0°` (refined 약 `37.1584°`)
- selected optical roll: `7.0°`
- seed yaw: `167.0°`
- selected hold-out objective: `0.7637627`
- minimum separated hold-out margin: `0.0649124` = `6.4912%`
- product state: `NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`
- estimated/visualized translation: approximately `[0.050376, 0.063374, 0.059408] m`

Case B가 PASS한 것은 stress 조건에서 현재 내부 목적함수와 품질 gate를 통과했다는 뜻이다.
편집 영상·동적 객체가 포함되어 있으므로 제품 RT의 정확도 근거로 사용하지 않는다.

## 6. Case C RT fixed cross-condition 결과

Case C의 `calibration_result.json`을 입력 pose로 그대로 읽었으며 `--validation-pose-json`
모드로 RT를 다시 추정하지 않았다.

### 6.1 Case C RT → Case A

- output: [fixed baseline validation JSON](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration/generated/fixed_primary_on_build5_10/fixed_pose_validation_result.json)
- label: `primary_build22_23_on_baseline_build5_10`
- result: exit 0, `INTERNAL_GATE_PASS`
- scene validation: 4/4 PASS
- candidate/product/activation: `NOT_CANDIDATE_RT` / `NOT_PRODUCT_APPROVED_RT` / `false`

### 6.2 Case C RT → Case B

- output: [fixed stress validation JSON](/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop/automatic_calibration/generated/fixed_primary_on_build17_21/fixed_pose_validation_result.json)
- label: `primary_build22_23_on_stress_build17_21`
- result: exit 0, `INTERNAL_GATE_PASS`
- scene validation: 5/5 PASS
- candidate/product/activation: `NOT_CANDIDATE_RT` / `NOT_PRODUCT_APPROVED_RT` / `false`

두 fixed 결과는 primary RT를 재선택하거나 refine한 결과가 아니다. 현재 데이터가 같은
rigid camera-LiDAR installation epoch이라는 전제가 확인되지 않으면 cross-condition 결과는
`DIAGNOSTIC_ONLY`로 해석해야 한다.

## 7. 전체 결과 해석

이번 실행으로 확인된 것은 다음이다.

1. 데이터 역할을 분리하면 build5~24 전체를 낭비하지 않고 사용할 수 있다.
2. clean primary Case C는 후보를 하나로 식별했고 build24에서도 내부 gate를 통과했다.
3. baseline Case A는 training 품질은 좋지만 duplicate/유사 hold-out 때문에 finalist ambiguity가 남았다.
4. stress Case B는 현재 목적함수에서 후보와 hold-out을 통과했다.
5. primary RT를 과거 범위에 고정 적용했을 때 9개 scene이 모두 내부 gate를 통과했다.
6. 위 결과는 모두 geometry/NID/structure 기반 내부 일관성 결과이며 독립 물리 RT 오차가 아니다.

따라서 현재 결론은 다음과 같다.

```text
CH1 internal candidate pipeline       PASS (Case C, Case B)
CH1 baseline ambiguity fail-closed    PASS (Case A expected rejection)
fixed cross-condition internal gate   PASS (9/9 scenes)
camera-side ChArUco audit             22 PASS + 2 expected rejection
independent T_lidar_marker_board      MISSING
product RT approval                   NOT_ALLOWED
```

## 8. 생성 산출물 보존

```text
automatic_calibration/generated/
  jenkins_scene0_ch1_primary_build22_24/
  jenkins_scene0_ch1_baseline_build5_10/
  jenkins_scene0_ch1_stress_build17_21/
  fixed_primary_on_build5_10/
  fixed_primary_on_build17_21/

manual_calibration/output/jenkins-scene0/
  ch1-all-build-charuco-audit/
```

각 automatic output에는 `calibration_result.json`, scene validation CSV, orientation full/
corrected score map, 5°/1° score map, prepared image, matching PNG, colorized 3D preview와
OBJ/PLY가 포함된다. 원본 dataset은 수정하지 않았다.

## 9. 남은 작업

- ChArUco가 부착된 동일 board에 대해 독립 `T_lidar_marker_board` 측정
- 개발 중 확인되지 않은 sealed product hold-out 3쌍 이상 추가
- installation epoch·image/scan UTC·SHA-256·K+D/profile을 manifest에 기록
- 현재 2% margin의 false acceptance를 독립 physical fixture에서 측정
- 같은 장치 조건 10회 반복성과 다른 설치 epoch 재현성 분리 검증
- CH2~CH4는 CH1 안정화 후 별도 계획으로 확장

## 10. 변경 로그

| 날짜 | 내용 |
|---|---|
| 2026-08-24 | 계획의 24-target ChArUco audit 실행, 22 PASS/2 expected rejection 확인 |
| 2026-08-24 | Case C primary build22·23→24 실행, `CANDIDATE_RT/PASS` 기록 |
| 2026-08-24 | Case A baseline build5/8/9→10 실행, `FAIL/FINALIST_AMBIGUOUS` 기록 |
| 2026-08-24 | Case B stress build17/18/19→20/21 실행, `CANDIDATE_RT/PASS` 기록 |
| 2026-08-24 | Case C RT를 Case A/B에 fixed 적용, 9/9 scene `INTERNAL_GATE_PASS` 기록 |

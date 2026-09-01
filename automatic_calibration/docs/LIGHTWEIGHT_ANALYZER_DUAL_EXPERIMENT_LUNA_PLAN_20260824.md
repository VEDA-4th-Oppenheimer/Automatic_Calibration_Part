# 경량 Orientation Analyzer 2종 비교 실험 및 Luna 실행 지침

- 작성일: 2026-08-24 (KST)
- 대상: CH1 targetless LiDAR–camera automatic calibration
- 목표 플랫폼: Ambarella CV5 기반 CCTV, OpenSDK, CPU 2 core / RAM 4 GB
- 문서 상태: 실행 전 계획
- 제품 코드 변경 상태: **변경하지 않음**
- 최고 허용 결과: `RESEARCH_CANDIDATE`; 제품 RT 자동 활성화 금지

## 1. 결론

현재 production 작업트리에 analyzer를 바로 합치지 않는다. 동일한 승인 baseline에서 분기한
두 개의 독립 실험 코드로 다음을 비교한다.

```text
B0 — 현재 기준선
full staged search → top-3 basin → 5° → 1° → Ceres/NID

T1 — Structural Analyzer
OpenCV + 기존 C++/Eigen
→ LSD/소실점
→ organized-grid normal/plane
→ Manhattan 축 대응
→ 방위각 구조 signature
→ Top-K bounded search
→ Ceres/NID
→ full-search fallback

T2 — Panorama Analyzer
JSON row/column → native LiDAR 구조 파노라마
→ 1D azimuth signature
→ Manhattan down/roll 후보
→ Top-K 저해상도 perspective 확인
→ Top-K bounded 3D search
→ Ceres/NID
→ full-search fallback
```

두 실험의 차이는 **초기 orientation proposal 생성과 확인 방법뿐**이다. 다음 항목은 B0와
동일하게 고정한다.

- camera K+D, raw/radtan 처리, LDC/zoom/focus metadata
- LiDAR 좌표계·pan 부호·tilt 계약
- camera-center 기구 prior
- NID/edge/structure 목적함수와 가중치
- Ceres refinement
- training/hold-out 분리
- 모든 품질 gate와 `activation_allowed=false`

T1/T2 결과를 보고 기존 threshold나 hold-out 역할을 바꾸면 공정한 비교가 아니다.

## 2. 실험 질문

1. analyzer가 B0 full search의 올바른 orientation basin을 Top-3 안에 유지하는가?
2. 비싼 전체 3D projection 후보 수와 wall time을 얼마나 줄이는가?
3. 90°/180° 대칭 false basin을 줄이는가, 아니면 올바른 basin을 제거하는가?
4. analyzer가 실패할 때 B0 full search로 정확히 fallback하는가?
5. T2의 2D 파노라마 확인이 T1보다 의미 있는 정확도/식별성 향상을 주는가?
6. 추가 코드·메모리·CV5 이식 부담을 고려했을 때 어느 방식이 제품 후보로 더 단순한가?

## 3. 범위와 비범위

### 3.1 이번 실험에 포함

- CH1만 사용
- build22·23 training → build24 development hold-out을 primary case로 사용
- build5~10 baseline regression 및 build17~21 stress 진단
- synthetic known-RT positive/degenerate case
- analyzer 단독 결과, bounded search 결과, fallback 결과 저장
- B0/T1/T2 정량 비교와 시각적 검토
- host Docker 환경에서 2-thread 제한 성능 측정

### 3.2 이번 실험에 포함하지 않음

- CH2~CH4 자동 캘리브레이션
- SuperPoint, SuperGlue, PyTorch, ONNX Runtime, 신규 AI 모델
- signal-strength NMI를 제품 점수에 추가
- K+D 공동 추정
- Qt GUI 및 Top-view 통합
- OpenSDK production tree 수정 또는 자동 merge
- ChArUco 정보를 targetless proposal 점수나 finalist 선택에 사용
- `PRODUCT_APPROVED_RT` 또는 `activation_allowed=true` 생성

## 4. 변경 금지 계약

Luna는 다음을 변경하지 않는다.

1. 현재 production 경로
   `/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop`의 파일.
2. 원본 image, JSON, PCD, manifest와 기존 generated output.
3. `calibration_core.cpp/.hpp`의 목적함수 가중치와 품질 threshold.
4. K+D profile, camera-center prior, 좌표계 변환식과 pan/tilt 부호.
5. build24의 hold-out 역할.
6. Case A/B/C의 pair index와 데이터 역할.
7. Ceres solver 제한, NID/edge/structure 평가식과 product 상태 정의.
8. 결과를 통과시키기 위한 사후 ROI, score weight, confidence threshold 조정.

금지 명령:

```text
git reset --hard
git checkout -- <production file>
git clean
rm -rf
git stash
production branch에 merge/push
```

현재 production 작업트리는 다른 개발 변경을 포함할 수 있다. Luna는 이를 정리하거나
baseline으로 추측하지 않는다.

## 5. 코드 격리 및 baseline 고정

### 5.1 필수 입력

실행 전에 사용자가 승인한 다음 값이 반드시 제공돼야 한다.

```text
BASELINE_COMMIT=<현재 승인된 Calibration Core를 포함한 commit SHA>
```

`BASELINE_COMMIT`이 없거나 해당 commit이 현재 필요한 코드를 포함하는지 확인되지 않으면
Luna는 `BASELINE_NOT_FROZEN`으로 중단한다. dirty production tree의 `HEAD`를 임의로
baseline으로 선택하지 않는다.

### 5.2 worktree 구조

```text
/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/
  develop/                              # production, read-only 취급
  analyzer_experiments/
    b0_baseline/                        # detached BASELINE_COMMIT
    t1_structural/                      # codex/exp-structural-analyzer-20260824
    t2_panorama/                        # codex/exp-panorama-analyzer-20260824
    reports/
      DUAL_EXPERIMENT_COMPARISON.md
      metrics.csv
      artifact_manifest.sha256
```

Luna가 사용할 worktree 생성 계약:

```bash
REPO=/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop
EXP_ROOT=/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/analyzer_experiments

test -n "$BASELINE_COMMIT"
git -C "$REPO" cat-file -e "$BASELINE_COMMIT^{commit}"

git -C "$REPO" worktree add --detach \
  "$EXP_ROOT/b0_baseline" "$BASELINE_COMMIT"
git -C "$REPO" worktree add -b codex/exp-structural-analyzer-20260824 \
  "$EXP_ROOT/t1_structural" "$BASELINE_COMMIT"
git -C "$REPO" worktree add -b codex/exp-panorama-analyzer-20260824 \
  "$EXP_ROOT/t2_panorama" "$BASELINE_COMMIT"
```

대상 경로나 branch가 이미 존재하면 삭제·덮어쓰기하지 말고
`EXPERIMENT_WORKTREE_ALREADY_EXISTS`로 중단한다.

### 5.3 데이터 공유

`data/jenkins-capture`와 `manual_calibration/output`은 Git에 포함되지 않으므로 복사하지
않는다. Docker 실행 때 production 원본을 read-only bind mount한다.

```text
host source:
  develop/data/jenkins-capture
  develop/manual_calibration/output

container target:
  /workspace/data/jenkins-capture:ro
  /workspace/manual_calibration/output:ro
```

각 worktree의 source, build, generated output은 서로 분리한다. 같은 build directory나
output directory를 B0/T1/T2가 공유하지 않는다.

## 6. 공통 입력 계약

| 항목 | 고정값 |
|---|---|
| channel | CH1 |
| intrinsic | `/workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json` |
| distortion state | `raw` |
| LDC | `unknown` |
| zoom/focus | `true` |
| camera-center prior | `(0.05928, -0.08105, 0) m` |
| minimum scene pass ratio | `1.0` |
| threads | 최대 2 |
| product activation | 항상 `false` |

Case mapping은 기존
[`CH1_ALL_BUILD_DATA_UTILIZATION_AND_LUNA_EXECUTION_PLAN_20260824.md`](CH1_ALL_BUILD_DATA_UTILIZATION_AND_LUNA_EXECUTION_PLAN_20260824.md)의
인덱스를 그대로 사용한다.

| case | pair | 역할 | B0 기대 상태 |
|---|---|---|---|
| Case A | `0~3`, hold-out 1 | build5/8/9 → build10 | `FAIL/FINALIST_AMBIGUOUS`; 제한적 회귀 |
| Case B | `4~8`, hold-out 2 | build17/18/19 → build20/21 | `CANDIDATE_RT/PASS`; stress 진단 |
| Case C | `9~11`, hold-out 1 | build22/23 → build24 | `CANDIDATE_RT/PASS`; primary |

Case A/B의 기대 상태가 달라지면 즉시 개선으로 선언하지 않고 `BEHAVIOR_CHANGE_REVIEW`로
분류한다. 물리 RT truth가 없기 때문이다.

## 7. 공통 analyzer 출력 계약

T1/T2는 동일한 파일과 필드를 출력해야 한다.

```text
analyzer/
  analyzer_result.json
  orientation_proposals.csv
  azimuth_signature_camera.csv
  azimuth_signature_lidar.csv
  azimuth_score_curve.csv
  analyzer_timing.json
  analyzer_summary.png
```

`analyzer_result.json` 최소 필드:

```json
{
  "schema_version": "1.0",
  "mode": "structural|panorama",
  "status": "PROPOSALS_READY|INSUFFICIENT_FEATURES|INVALID_INPUT",
  "input_rows": 101,
  "input_columns": 400,
  "proposal_count": 3,
  "proposals": [
    {
      "rank": 1,
      "yaw_deg": 0.0,
      "down_deg": 0.0,
      "roll_deg": 0.0,
      "score": 0.0,
      "confidence": 0.0,
      "search_radius_deg": 10.0,
      "evidence": ["MANHATTAN", "AZIMUTH_SIGNATURE"]
    }
  ],
  "fallback_required": false,
  "fallback_reason": "",
  "runtime_ms": 0.0,
  "peak_working_memory_bytes": 0
}
```

`orientation_proposals.csv`:

```text
rank,yaw_deg,down_deg,roll_deg,raw_score,normalized_score,confidence,search_radius_deg,evidence
```

모든 각도는 degree로 기록하되 내부 계산은 radian을 사용할 수 있다. yaw는 출력 시
`[-180, 180)`으로 정규화하고 비교 시 circular distance를 사용한다.

## 8. Test 1 — Structural Analyzer 구현 지침

### 8.1 목적

현재 코드가 이미 계산하는 영상 소실점, LiDAR surface normal/plane 및 구조 edge를 full
projection search 전에 재사용해 작은 orientation 후보 집합을 만든다. 신규 외부 의존성을
추가하지 않는다.

### 8.2 처리 순서

```text
undistorted CH1 image
  → 1/4 수준의 analyzer image
  → LSD long-line 추출
  → 방향 cluster와 Manhattan vanishing direction

organized LiDAR grid
  → 인접 셀 normal
  → dominant plane-normal cluster
  → vertical/horizontal Manhattan axis
  → column별 range/normal/plane-intersection energy

camera axis ↔ LiDAR axis의 right-handed permutation 열거
  → 중력/직교 조건으로 불가능 후보 제거
  → azimuth signature로 후보 순위 결정
  → 서로 30° 이상 떨어진 Top-3
  → 각 후보 주변 bounded search
```

### 8.3 구현 제한

- 기존 `detectManhattanVanishingDirections`와 organized-grid feature 계산을 먼저 재사용한다.
- image/scan 전체에 대한 새 kd-tree 또는 PCL dependency를 추가하지 않는다.
- analyzer 단계에서 Ceres, NID, z-buffer full projection을 실행하지 않는다.
- proposal을 한 개로 hard select하지 않고 최대 3개를 유지한다.
- translation을 analyzer로 새로 추정하지 않는다. 공통 기구 prior를 그대로 사용한다.
- 기본 bounded window는 proposal 중심 yaw/down/roll 각각 `±10°`; 이후 기존 5°→1°
  local policy를 사용한다.

### 8.4 fallback

다음 중 하나면 B0 full staged search를 실행한다.

- image에서 서로 비평행인 유효 방향군을 만들 수 없음
- LiDAR에서 서로 비평행인 dominant plane-normal 방향군을 만들 수 없음
- 유효 proposal이 없음
- Top-3 bounded search가 모두 기존 절대 overlap/coverage gate를 통과하지 못함
- analyzer 입력 계약 위반

confidence threshold를 build24 결과에 맞춰 새로 만들지 않는다. 첫 실험에서는 기본 특징
가용성과 bounded-stage 절대 gate만 fallback 판단에 사용한다.

### 8.5 T1 추가 산출물

```text
image_lsd_lines.png
image_vanishing_directions.csv
lidar_dominant_plane_axes.csv
manhattan_axis_permutations.csv
```

## 9. Test 2 — Panorama Analyzer 구현 지침

### 9.1 목적

JSON의 `row`/`column` organized scan을 native angular raster로 사용한다. 고해상도 AI
matching 대신 저해상도 구조 파노라마와 OpenCV 연산만으로 yaw basin을 찾고 T1의
Manhattan down/roll 후보를 확인한다.

### 9.2 입력 검증

- `scan.rows`, `scan.columns`, `sample_count`와 measurement index를 검증한다.
- 현재 dataset의 예상 shape는 `101×400`이지만 parser에 상수로 고정하지 않는다.
- `(row,column)` 중복, 범위 초과, invalid cell, pan 0°/360° seam을 검사한다.
- JSON 좌표 계약의 pan/tilt 값을 다시 부호 변환하지 않는다.

### 9.3 파노라마 채널

최소 채널만 유지한다.

| 채널 | 저장/계산 | analyzer 점수 사용 |
|---|---|---|
| range | mm `uint16` 또는 transient float | edge 생성에 사용 |
| valid mask | `uint8` | coverage 정규화 |
| range edge | `uint8` | 사용 |
| normal edge | `uint8` | 사용 |
| plane-intersection support | `uint8` | 사용 |
| signal strength | 진단 PNG/통계만 | **초기 실험 점수에서는 제외** |
| point index | 필요할 때만 integer map | 최종 3D point 복원 |

빈 셀을 가로질러 range를 보간하거나 구조 edge를 새로 만들지 않는다. 작은 시각화용
보간본과 scoring용 원본 valid mask를 분리한다.

### 9.4 처리 순서

```text
JSON row/column
  → native range/valid raster
  → 인접 셀 normal/range discontinuity
  → 400-column azimuth structural signature
  → camera structural signature와 circular comparison
  → 서로 떨어진 yaw peak Top-K

Manhattan analyzer
  → down/roll/axis permutation

yaw × down/roll proposal
  → 최대 width 320의 perspective structure image
  → camera edge distance transform과 chamfer/coverage 비교
  → Top-3 bounded 3D search
```

카메라 이미지는 종횡비를 유지한 최대 width 320으로 축소하고 K를 같은 비율로 조정한다.
카메라 unit-ray lookup은 한 번만 만들고 proposal별 변환에는 재사용한다. perspective 확인은
`cv::remap` 또는 동등한 고정 lookup 방식으로 구현한다.

### 9.5 perspective 확인 점수

초기 실험에서는 새 복합 가중치를 발명하지 않는다.

1. camera structural edge의 distance transform 생성
2. candidate LiDAR range/normal/plane-intersection edge를 sampling
3. valid coverage 미달 후보 제외
4. 평균 chamfer distance로 순위 결정
5. 동률에 가까운 후보는 모두 Top-3로 보존

signal strength와 grayscale NMI는 conformance가 완료되기 전까지 tie-break에도 사용하지
않는다.

### 9.6 fallback

T1 조건에 더해 다음이면 B0 full staged search를 실행한다.

- JSON organized shape/index 계약 위반
- valid panorama coverage 부족
- perspective comparison의 유효 structural point가 없음
- Top-3 bounded search가 모두 기존 절대 gate 실패

### 9.7 T2 추가 산출물

```text
panorama_range.png
panorama_valid.png
panorama_range_edge.png
panorama_normal_edge.png
panorama_plane_intersection.png
panorama_signal_diagnostic.png
perspective_candidate_<rank>.png
perspective_candidate_<rank>_overlay.png
```

## 10. 공통 bounded-search와 fallback 계약

Analyzer가 만든 proposal은 기존 coarse 결과와 같은 `R` seed 형식으로 변환한다. 후단에는
새 목적함수를 넣지 않는다.

```text
proposal Top-3
  → proposal별 ±10° local window, 5° step
  → winner 주변 ±5°, 1° step
  → 기존 finalist/Ceres/NID
```

fallback은 별도 알고리즘이 아니라 현재 B0 full staged search 함수를 그대로 호출해야 한다.

```text
fallback_triggered=true
fallback_reason=<정해진 reason>
analyzer_runtime_ms=<analyzer만>
fallback_runtime_ms=<B0 full search만>
total_runtime_ms=<전체>
```

fallback 경로의 최종 결과와 score map은 같은 commit의 B0 실행과 수치상 동일해야 한다.

## 11. 구현 파일 관리 지침

각 branch에서만 다음과 유사한 최소 변경을 허용한다. 실제 파일명은 기존 구조와 충돌하지
않는 범위에서 정하되 새 프레임워크나 factory를 만들지 않는다.

### 11.1 T1 branch

```text
automatic_calibration/include/auto_calib/structural_orientation_analyzer.hpp
automatic_calibration/src/structural_orientation_analyzer.cpp
automatic_calibration/tests/structural_orientation_analyzer_tests.cpp
automatic_calibration/apps/run_real_calibration.cpp       # proposal 연결만
automatic_calibration/CMakeLists.txt                       # target/source/test 등록만
automatic_calibration/docs/T1_STRUCTURAL_ANALYZER_REPORT.md
```

### 11.2 T2 branch

```text
automatic_calibration/include/auto_calib/panorama_orientation_analyzer.hpp
automatic_calibration/src/panorama_orientation_analyzer.cpp
automatic_calibration/tests/panorama_orientation_analyzer_tests.cpp
automatic_calibration/apps/run_real_calibration.cpp       # proposal 연결만
automatic_calibration/CMakeLists.txt                       # target/source/test 등록만
automatic_calibration/docs/T2_PANORAMA_ANALYZER_REPORT.md
```

두 branch 사이에서 구현 commit을 cherry-pick하지 않는다. 공통 baseline과 출력 schema만
동일하게 유지한다. 비교가 끝나기 전 production 코드로 복사하지 않는다.

## 12. Docker 실행 격리

worktree에는 production의 ignored Docker 파일이 없을 수 있으므로 현재 빌드 image를
`docker run`으로 사용한다. 아래 template에서 `WT`와 `NAME`만 바꾼다.

```bash
PROD=/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/develop
WT=/mnt/c/Users/3-16/Documents/codex_workspace/auto_calib/analyzer_experiments/t1_structural
NAME=auto-calib-analyzer-t1

mkdir -p "$WT/build"

docker run --rm --name "$NAME" \
  --cpus=2 \
  -v "$WT":/workspace \
  -v "$WT/build":/workspace-build \
  -v "$PROD/data/jenkins-capture":/workspace/data/jenkins-capture:ro \
  -v "$PROD/manual_calibration/output":/workspace/manual_calibration/output:ro \
  -w /workspace \
  auto-calib-dev:ubuntu-latest \
  bash -lc 'cmake -S /workspace -B /workspace-build -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build /workspace-build -j2'
```

T2는 `WT=t2_panorama`, `NAME=auto-calib-analyzer-t2`, B0는 `WT=b0_baseline`,
`NAME=auto-calib-analyzer-b0`를 사용한다. 같은 container name으로 두 실행을 동시에 시작하지
않는다. 실행은 B0 → T1 → T2 순서로 직렬 수행한다.

빌드 뒤 test 또는 calibration을 실행할 때도 동일 mount를 반복한다. host에서 binary를 직접
실행하지 않는다. 다음은 T1 non-real test 예시다.

```bash
docker run --rm --name auto-calib-analyzer-t1-test \
  --cpus=2 \
  -v "$WT":/workspace \
  -v "$WT/build":/workspace-build \
  -v "$PROD/data/jenkins-capture":/workspace/data/jenkins-capture:ro \
  -v "$PROD/manual_calibration/output":/workspace/manual_calibration/output:ro \
  -w /workspace \
  auto-calib-dev:ubuntu-latest \
  ctest --test-dir /workspace-build -LE real_data --output-on-failure
```

실데이터 실행은 같은 wrapper의 마지막 명령만 다음으로 바꾼다.

```text
/usr/bin/time -v /workspace-build/bin/run_real_calibration <Section 14 options>
```

Docker wrapper, 실제 binary option, exit code를 보고서에 함께 보존한다. `/usr/bin/time -v`가
image에 없으면 설치하지 말고 binary 내부 timing과 Docker stats를 사용하며 그 차이를 기록한다.

## 13. 테스트 단계

### Phase 0 — Preflight

각 worktree에서 기록한다.

```text
BASELINE_COMMIT
branch/HEAD
git status --short
compiler/OpenCV/Eigen/Ceres version
Docker image ID
input package inventory와 SHA-256
K+D SHA-256
```

세 worktree가 같은 baseline에서 시작하지 않았으면 실행하지 않는다.

### Phase 1 — Unit/synthetic

T1 필수 검사:

- known Manhattan rotation의 Top-3 proposal recall
- yaw 359°/0° circular distance
- 90°/180° 대칭에서 복수 후보 보존
- image line 부족 시 fallback
- LiDAR plane-normal 부족 시 fallback

T2 필수 검사:

- `rows×columns` raster mapping
- 중복/out-of-range cell 거절
- invalid mask가 edge로 생성되지 않음
- 359°/0° panorama seam
- known yaw circular score peak
- perspective ray lookup과 K scaling
- panorama coverage 부족 시 fallback

공통 검사:

- analyzer failure 뒤 B0 fallback 결과가 baseline과 동일
- Top-K 이외 후보에서 Ceres를 실행하지 않음
- 입력 파일을 쓰지 않음
- 같은 입력 3회 analyzer proposal JSON hash가 동일

### Phase 2 — B0 기준선

`b0_baseline`에서 기존 명령을 그대로 사용해 최소 Case C를 한 번 실행한다. 기존 generated
결과는 정확성 참고로 사용할 수 있지만 runtime 비교에는 같은 Docker image에서 새로 실행한
B0를 사용한다.

출력명:

```text
automatic_calibration/generated/analyzer_eval/b0_case_c
```

### Phase 3 — analyzer-only 개발

- synthetic와 build22만 사용한다.
- build24를 열어 score/threshold를 조정하지 않는다.
- analyzer 출력 schema, Top-3 separation, ±10° window를 먼저 고정한다.
- analyzer-only 실행 5회의 median runtime과 peak memory를 기록한다.
- parameters를 고정한 commit을 만든 뒤 Phase 4로 진행한다.

### Phase 4 — Case C 공정 비교

동일한 pair와 hold-out을 사용한다.

```text
pair-start=9
pair-count=3
holdout-count=1
```

출력명:

```text
T1: automatic_calibration/generated/analyzer_eval/t1_case_c
T2: automatic_calibration/generated/analyzer_eval/t2_case_c
```

Phase 4를 시작한 뒤 analyzer parameter를 변경하면 build24는 더 이상 독립 development
hold-out으로 부르지 않고 회귀 fixture로 내린다.

### Phase 5 — Case A/B regression/stress

Case C에서 fatal input/runtime 오류가 없을 때만 실행한다.

```text
Case A: pair-start=0, pair-count=4, holdout-count=1
Case B: pair-start=4, pair-count=5, holdout-count=2
```

출력명:

```text
t1_case_a, t1_case_b
t2_case_a, t2_case_b
```

Case A의 B0 `FINALIST_AMBIGUOUS`가 T1/T2에서 PASS로 바뀌어도 물리 정답 개선으로 선언하지
않는다. `BEHAVIOR_CHANGE_REVIEW`로 기록하고 projection과 separated finalist를 검토한다.

### Phase 6 — fallback fixture

다음 두 case를 반드시 실행한다.

- line/plane이 부족한 synthetic degenerate input
- analyzer proposal을 만들 수 있지만 bounded-stage absolute gate가 모두 실패하는 input

두 경우 모두 `fallback_triggered=true`이고 B0와 같은 최종 status/reason/RT 또는 같은
fail-closed 판정이 나와야 한다.

## 14. 실행 명령 template

B0 명령은 기존 Luna 계획의 Case A/B/C 명령을 그대로 사용한다. T1/T2는 branch에서 추가한
analyzer option만 다르고 나머지는 동일해야 한다.

```bash
/workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output OUTPUT_DIR \
  --debug-output OUTPUT_DIR/debug \
  --pair-start PAIR_START --pair-count PAIR_COUNT --holdout-count HOLDOUT_COUNT \
  --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --search-strategy staged --minimum-scene-pass-ratio 1.0 \
  --orientation-analyzer structural_or_panorama
```

CLI option 이름을 다르게 구현하면 보고서에 이유와 실제 `--help` 출력을 기록한다. B0는
analyzer option을 받지 않는다.

장시간 실행은 한 번만 시작한다. 60초 무출력을 이유로 중복 시작하지 않고 process 상태를
확인한다. exit `3`은 runtime 오류가 아니라 품질 gate 정상 거절로 처리한다.

## 15. 비교 지표

### 15.1 Proposal 성능

- synthetic GT `proposal_recall@1`, `proposal_recall@3`
- B0 selected/finalist basin과 T1/T2 proposal의 circular angular distance
- distinct proposal 수와 90°/180° alias 보존 여부
- analyzer insufficient/fallback 횟수와 이유
- bounded search가 평가한 orientation 수
- 전체 3D projection 호출 수

실데이터에서 B0 basin 포함 여부는 **baseline-basin recall**이며 물리 정확도가 아니다.

### 15.2 최종 calibration

- final status/reason code
- training/hold-out scene pass
- minimum separated hold-out objective margin
- B0 대비 rotation geodesic difference
- B0 대비 translation norm difference
- raw edge residual p50/p90 및 30 px 초과 비율
- geometry NID, visible edge/structure support, Manhattan vertical error
- Ceres iteration과 objective improvement

### 15.3 자원

- analyzer-only wall/user/system time
- bounded-search time
- fallback time
- total wall time
- peak RSS
- 실행 thread 수
- output artifact 크기
- 새 source line과 binary size 증가량

성능은 full end-to-end를 각 case 한 번 실행하고, 빠른 analyzer-only 구간은 5회 median으로
보고한다. 장시간 B0 전체 실행을 의미 없이 반복하지 않는다.

## 16. 공정 비교 및 engineering 목표

아래는 제품 승인 기준이 아니라 실험 선택 기준이다.

### 16.1 필수 조건

- synthetic positive에서 Top-3 proposal recall 100%
- degenerate case에서 임의 후보 확정 없이 full-search fallback
- Case C에서 B0 primary basin이 Top-3 bounded window 안에 포함
- build24를 이용한 사후 parameter 수정 없음
- 기존 K+D, 목적함수, gate, Ceres와 좌표 계약 변경 없음
- B0/T1/T2 원본 입력 SHA-256 동일
- 모든 결과에서 `activation_allowed=false`

### 16.2 성능 목표

- non-fallback Case C의 전체 3D projection 후보 평가 수가 B0의 50% 이하
- total wall time이 B0의 70% 이하를 목표로 함
- analyzer 추가 peak memory 64 MiB 이하
- 외부 AI runtime/model weight 신규 의존성 0

성능 목표를 못 채웠더라도 정확성 증거는 보존한다. 하지만 복잡도만 늘고 후보 평가 수나
runtime이 줄지 않으면 제품 후보로 채택하지 않는다.

### 16.3 T1/T2 선택 규칙

```text
T1이 필수 조건 충족
T2가 T1보다 proposal recall/fallback/false-basin을 개선하지 못함
→ T1 선택: 더 단순함

T1이 올바른 basin을 자주 놓침
T2 perspective 확인이 이를 반복적으로 복구
→ T2 선택 후보

둘 다 올바른 basin 누락 또는 fallback 과다
→ 채택 보류, B0 유지
```

시각적으로 한 장이 좋아 보인다는 이유로 T2를 선택하지 않는다.

## 17. 시각 검토 계약

각 Case C scene에서 B0/T1/T2의 다음 파일을 같은 순서로 나란히 본다.

1. `matching_scene_*.png`
2. `debug/scene_*/06_projection_final.png`
3. `debug/scene_*/07_projection_final_edges.png`
4. `scene_*_colorized_lidar_3d_preview.png`
5. analyzer proposal/score curve와 T2 perspective overlay

검토자는 다음을 기록한다.

```text
yaw 방향이 실제 CH1 방향과 일치하는가
down 방향이 바닥/벽 구조와 일치하는가
책상·모니터·벽 경계가 같은 방향으로 이동했는가
일부 texture edge만 우연히 맞춘 것은 아닌가
가시성 밖 LiDAR point가 점수를 지배하지 않는가
T1/T2가 서로 다른 false basin을 선택하지 않았는가
```

내부 PASS와 육안 projection이 충돌하면 `PASS_BUT_VISUALLY_SUSPECT`로 기록하고 채택하지
않는다.

## 18. 완료 보고서

T1 branch:

```text
automatic_calibration/docs/T1_STRUCTURAL_ANALYZER_REPORT.md
```

T2 branch:

```text
automatic_calibration/docs/T2_PANORAMA_ANALYZER_REPORT.md
```

공통 comparison:

```text
analyzer_experiments/reports/DUAL_EXPERIMENT_COMPARISON.md
analyzer_experiments/reports/metrics.csv
analyzer_experiments/reports/artifact_manifest.sha256
```

최종 비교표 필수 형식:

| case | mode | analyzer status | top-3 yaw/down/roll | fallback | pose eval 수 | total time | RSS | final RT | train/hold-out | status/reason | visual review |
|---|---|---|---|---|---:|---:|---:|---|---|---|---|

보고서에는 반드시 구분한다.

- 관측 사실
- baseline 대비 변화
- 원인 추론
- 물리 truth 부재로 판단할 수 없는 항목
- 제품 채택 권고와 보류 조건

## 19. 코드 및 결과 리뷰 체크리스트

Luna는 완료 전에 다음을 검사한다.

```text
[ ] production develop 파일을 수정하지 않았음
[ ] T1/T2가 동일 BASELINE_COMMIT에서 시작함
[ ] T1/T2 branch가 분리됨
[ ] 원본 데이터와 K+D가 read-only mount였음
[ ] objective/gate/Ceres/좌표계 diff가 없음
[ ] ChArUco 정보가 proposal score에 없음
[ ] analyzer output schema가 T1/T2 동일함
[ ] yaw wrap-around unit test가 있음
[ ] analyzer failure와 bounded failure fallback test가 있음
[ ] fallback 결과가 B0와 동일함
[ ] build24 확인 후 parameter를 바꾸지 않았음
[ ] `git diff --check` PASS
[ ] 신규 unit test PASS
[ ] 기존 non-real CTest PASS
[ ] Case A/B/C exit code와 JSON reason을 함께 기록함
[ ] matching/edge/3D preview를 수치 결과와 함께 검토함
[ ] product status가 승격되지 않았음
[ ] branch commit, 실행 명령, runtime, artifact hash가 기록됨
```

Luna는 merge 또는 production 반영을 하지 않는다. 최종 상태는 다음 중 하나다.

```text
REVIEW_READY_T1
REVIEW_READY_T2
REVIEW_READY_BOTH
EXPERIMENT_REJECTED_WITH_EVIDENCE
BLOCKED_<reason>
```

이후 primary Codex/팀원이 두 branch diff와 결과를 별도로 리뷰한 뒤에만 production 적용
여부를 결정한다.

## 20. Luna 작업 지시문

아래 블록을 Luna 작업 요청에 그대로 사용한다.

```text
목표:
LIGHTWEIGHT_ANALYZER_DUAL_EXPERIMENT_LUNA_PLAN_20260824.md를 완전히 읽고,
동일한 BASELINE_COMMIT에서 Test 1 Structural Analyzer와 Test 2 Panorama Analyzer를
서로 다른 git worktree/branch에 구현·검증한다. production develop은 읽기 전용이다.

필수 입력:
BASELINE_COMMIT=<사용자가 승인한 SHA>

절대 금지:
- production develop 수정/정리/reset/stash/clean
- 원본 dataset/K+D 수정
- T1/T2 코드 공유 또는 비교 전 merge
- K+D, objective weight, gate, Ceres, 좌표계 변경
- build24 결과를 본 뒤 parameter 조정
- ChArUco를 targetless 후보 선택에 사용
- product RT 승인 또는 activation true
- 장시간 명령 중복 실행

실행 순서:
1. BASELINE_COMMIT과 입력 hash를 확인한다. 없으면 BASELINE_NOT_FROZEN으로 중단한다.
2. b0_baseline/t1_structural/t2_panorama worktree를 생성한다.
3. B0 Case C 기준 실행과 unit/non-real regression을 기록한다.
4. T1을 synthetic+build22에서 개발하고 parameter/schema를 고정·commit한다.
5. T1 Case C, Case A/B, fallback fixture를 실행하고 T1 보고서를 작성한다.
6. T2를 BASELINE_COMMIT에서 독립 개발하고 같은 순서로 실행·보고한다.
7. B0/T1/T2의 proposal recall, fallback, pose evaluation 수, runtime, RSS, final RT,
   hold-out, raw residual, NID, projection 시각화를 공통 표로 비교한다.
8. git diff --check, unit/non-real CTest, fallback 동일성, artifact hash를 확인한다.
9. merge/push하지 말고 REVIEW_READY_BOTH 또는 명확한 BLOCKED reason으로 종료한다.

보고 방식:
각 phase마다 worktree/branch/commit, 명령, 시작·종료 시각, exit code, output 경로,
status/reason, analyzer proposals, fallback 여부, runtime/RSS, 다음 허용 단계를 기록한다.
품질 exit 3은 runtime 오류와 구분한다. 수치 PASS만으로 방향 정답을 선언하지 않고 PNG/3D
projection과 물리 truth 부재를 함께 기록한다.
```

## 21. 완료 정의

다음이 모두 있으면 실행 완료다. PASS가 필수는 아니며 안전한 실패도 유효한 결과다.

1. B0/T1/T2가 동일 baseline·입력·profile로 실행됨
2. T1/T2 unit/synthetic와 fallback fixture 결과가 있음
3. Case C 공정 비교가 있음
4. 가능하면 Case A/B regression/stress가 있음
5. proposal, runtime, memory, final RT, gate, raw residual이 한 표로 비교됨
6. 필수 PNG/3D artifact를 사람이 검토할 수 있게 연결함
7. branch별 diff와 commit이 보존됨
8. production code와 원본 데이터가 변경되지 않음
9. 채택/보류 근거와 물리 truth 한계가 기록됨
10. production merge는 수행하지 않고 review 요청 상태로 종료함

## 22. 변경 로그

| 날짜 | 내용 |
|---|---|
| 2026-08-24 | Structural Analyzer와 Panorama Analyzer를 동일 baseline에서 비교하는 최초 계획 작성 |
| 2026-08-24 | 별도 worktree/branch, read-only input, B0 fallback 및 공통 artifact schema 고정 |
| 2026-08-24 | Luna 실행 지시, 완료 후 비교 보고서와 production 비병합 리뷰 계약 추가 |

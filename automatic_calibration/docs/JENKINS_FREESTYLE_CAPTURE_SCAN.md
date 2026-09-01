# Jenkins Freestyle 데이터셋 수집 자동화 (`data_storage`)

문서 상태: 운영 가이드

최종 수정: 2026-08-24

## 1. 목적

Jenkins(`http://172.20.33.193:8080/job/data_storage/`) 자동화의 핵심 목적은 **실제 센서 환경에서 2D CCTV 스냅샷과 3D LiDAR 스캔 데이터를 자동으로 수집·패키징하고, 수집된 데이터를 기반으로 캘리브레이션 테스트를 수행**하는 것이다.

SSH 접속 없이 Raspberry Pi의 ADTS 웹 서비스(`http://172.20.26.191:8080`) 및 CCTV SUNAPI HTTP API를 100% 순수 Shell 스크립트로 호출하여 제어하며, 작업 완료 및 실패 상태를 Slack으로 실시간 통보한다.

---

## 2. Jenkins `data_storage` 구조 및 연계 파이프라인

현재 Jenkins의 `data_storage` Folder에는 3개의 개별 Freestyle Job이 구성되어 순차적으로 연계된다.

```text
[ 정기 스케줄 (월~금) ]
        │
        ▼
[ 1. cctv_capture ]  ──(성공 시)──►  [ 2. 3d_scan ]  ──(성공 시)──►  [ 3. dataset_pack ]  ──►  [ calibration_test ]
  (CCTV CH1~4 스냅샷)                (하드웨어 점검 & 스캔)            (세션 압축 및 아티팩트)     (캘리브레이션 테스트)
        │                                   │                                  │
        └───────────────────────────────────┼──────────────────────────────────┘
                                            ▼
                                [ Slack 알림 (성공/실패) ]
```

| Job 이름 | 역할 | 주요 산출물 | 연계 트리거 |
|---|---|---|---|
| **`cctv_capture`** | CCTV 1~4채널 스냅샷 순차 수집 | `cctv_capture/*.jpg` | 빌드 성공 시 `3d_scan` 트리거 |
| **`3d_scan`** | STM32 점검 → REARM → HOME → SCAN → PCD/JSON 회수 | `lidar_scan/*.pcd`, `*_pan_tilt_lidar.json` | 빌드 성공 시 `dataset_pack` 트리거 |
| **`dataset_pack`** | 수집된 이미지와 LiDAR 데이터를 묶어 압축 및 아티팩트 보관 | `calib_dataset_build*.tar.gz`, `manifest.json` | 아티팩트 등록 및 테스트 입력 제공 |

---

## 3. Job별 세부 설정 및 동작 시퀀스

### 3.1 `cctv_capture` (CCTV 이미지 수집 & 스케줄러)
- **실행 방식**: Freestyle Project (100% Pure Shell)
- **주요 파라미터**: `CCTV_IP` (`172.20.32.43`), `CCTV_CHANNELS` (`1 2 3 4`), `CCTV_PROFILE` (`1`)
- **Credentials**: `CCTV_USER` (`admin`), `CCTV_PASSWORD` (`5hanwha!`)
- **빌드 후 조치**:
  - `Archive the artifacts`: `cctv_capture/*.jpg`
  - `Build other projects`: `3d_scan` (안정적인 빌드만 유발)
  - `Slack Notifications`: 빌드 성공 및 실패 알림 발송

### 3.2 `3d_scan` (3D LiDAR 스캔 및 다운로드)
- **실행 방식**: Freestyle Project (100% Pure Shell)
- **주요 파라미터**: `SCANNER_IP` (`172.20.26.191:8080`), `SCAN_TIMEOUT_SEC` (`600`)
- **제어 시퀀스 & 안전 장치**:
  1. `/api/state` 호출하여 STM32 하드웨어 통신(`link_alive: true`) 확인 (DOWN 시 즉시 실패 처리)
  2. `DISARM` 상태 감지 시 `/api/cmd/rearm` 전송 후 `IDLE` 전환 대기
  3. `/api/cmd/home` 전송 후 `HOMED: true (Y)` 및 `IDLE` 복귀 완료 대기
  4. `/api/cmd/scan` 전송 후 진행률(`percent`) 실시간 모니터링
  5. **[에러 감지]**: 스캔 중 장비가 `DISARM` 또는 에러로 비정상 중단되면 즉시 실패(`exit 1`) 처리하여 불완전한 파일 생성 방지
  6. 정상 완료(`IDLE` 복귀) 확인 후 `.pcd` 파일 및 `_pan_tilt_lidar.json` 파일 다운로드 (`lidar_scan/`)
- **빌드 후 조치**:
  - `Archive the artifacts`: `lidar_scan/*`
  - `Build other projects`: `dataset_pack` (안정적인 빌드만 유발)
  - `Slack Notifications`: 빌드 성공 및 실패 알림 발송

### 3.3 `dataset_pack` (데이터셋 압축 및 아티팩트 보관)
- **실행 방식**: Freestyle Project (100% Pure Shell)
- **동작**:
  - `cctv_capture`의 이미지들과 `3d_scan`의 PCD/JSON 파일을 탐색하여 세션 임시 디렉터리로 복사
  - 수집 메타데이터 `manifest.json` 생성
  - `tar -czf` 또는 `zip`으로 단일 압축 파일 생성 (`calib_dataset_build<N>_<TIMESTAMP>.tar.gz`)
- **빌드 후 조치**:
  - `Archive the artifacts`: `*.zip, *.tar.gz`
  - `Slack Notifications`: 패키징 완료 및 아티팩트 생성 알림 발송

---

## 4. 정기 실행 스케줄 설정 (`cctv_capture`)

정기 데이터 수집은 파이프라인의 시작점인 `cctv_capture`의 **[주기적으로 빌드 (Build periodically)]**에 등록되어 주간 업무 시간대에 맞추어 자동 실행된다.

```cron
TZ=Asia/Seoul
# [8시대] 08:00, 08:12, 08:25, 08:40 (월~금)
0,12,25,40 8 * * 1-5

# [9시대] 09:00, 09:12, 09:25 (월~금)
0,12,25 9 * * 1-5

# [13시대] 13:05, 13:18 (월~금)
5,18 13 * * 1-5
```

---

## 5. Slack 알림 연동

- **인증 방식**: Slack Bot User OAuth Token (`xoxb-...`) 기반 (Jenkins Credential `Secret text` 등록)
- **알림 채널**: 지정된 알림 채널 (예: `#build-alerts`)
- **알림 시점**:
  - 빌드 성공 시: 수집 완료 상태 및 생성된 아티팩트 링크 안내
  - 빌드 실패 시: 하드웨어 통신 두절, 스캔 중단(`DISARM`), 카메라 응답 실패 시 즉시 장애 통보

---

## 6. 수집 데이터셋 구조 및 테스트 연계

`dataset_pack` Job의 아티팩트로 보관되는 압축 파일 내부 구조:

```text
calib_dataset_build<N>_<TIMESTAMP>/
├── YYYYMMDD_HHMMSS_CH1.jpg
├── YYYYMMDD_HHMMSS_CH2.jpg
├── YYYYMMDD_HHMMSS_CH3.jpg
├── YYYYMMDD_HHMMSS_CH4.jpg
├── calib-YYYYMMDD-HHMMSS_sweep-000001.pcd
├── calib-YYYYMMDD-HHMMSS_sweep-000001_pan_tilt_lidar.json
└── manifest.json
```

- **테스트 연계**:
  정기 수집으로 아티팩트가 생성되면, 후속 `calibration_test` Job에서 해당 압축 패키지를 내려받아 압축 해제 후 `run_real_calibration`을 실행하여 캘리브레이션 정확도 및 회귀 테스트를 수행한다.

## 5. 2026-08-21~23 `scene0` CH1 현재 검증 데이터

현재 데이터 루트는 다음과 같다.

```text
data/jenkins-capture/scene0/
```

현재 scan 정렬 순서는 `build5`, `build8`, `build9`, `build10`, `build17`, `build18`,
`build19`, `build20`, `build21`, `build22`, `build23`, `build24`의 12개다. 기존
`build5~10` 네 패키지 결과는
2026-08-21 baseline으로 유지하고, `build17~21`은 2026-08-22~23 확장 진단으로
분리한다. `build22~24`는 2026-08-24 CH1 ChArUco reference 계획으로 별도 분리한다.

| 범위 | 현재 용도 | 데이터 제약 |
|---|---|---|
| build5/8/9 | 과거 3-training baseline | 같은 코드 binary로 재실행한 최신 반복성 값은 아님 |
| build10 | 과거 limited hold-out | CH1이 build9과 byte-identical |
| build17/18/19 | 확장 3-training 진단 | build17 편집 영상, build18 근거리 보드, build19 사람 포함 |
| build20/21 | 확장 2-dynamic-object-hold-out | 수정 영상의 파일명·EXIF는 scan 시작명과 일치, 사람 포함 및 manifest 자동 계약 없음 |
| build22/23 | 신규 2-training | 정적 장면, CH1의 두 ChArUco ROI 검출 PASS |
| build24 | 신규 1-hold-out | 시간상 마지막 독립 image/scan, ChArUco ROI 검출 PASS |

`run_real_calibration`은 현재 다음 규칙으로 Jenkins 패키지 루트를 직접 읽는다.

- `--camera-channel 1`: `_CH1` 영상만 선택
- `_pan_tilt_lidar.json`: scan JSON만 선택하고 `manifest.json` 제외
- 같은 하위 패키지 디렉터리의 image와 scan만 pairing
- scan 파일명 기준 시간순 정렬
- `--holdout-count 1`: 마지막 pair를 최적화에서 제외

실행 예:

```bash
/workspace-build/bin/run_real_calibration \
  --input-dir /workspace/data/jenkins-capture/scene0 \
  --output /workspace/automatic_calibration/generated/jenkins_scene0_ch1_20260821_3train_1holdout \
  --pair-start 0 --pair-count 4 --camera-channel 1 \
  --ldc-enabled unknown --zoom-focus-locked true \
  --manual-intrinsic-json /workspace/manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json \
  --image-distortion-state raw \
  --camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0 \
  --search-strategy staged --holdout-count 1 --minimum-scene-pass-ratio 1.0
```

주의: `build9`과 `build10`의 CH1 영상은 동일 파일이므로 현재 마지막 pair는 완전한 독립
hold-out이 아니다. 상세 결과와 단독 RT 반복성 FAIL 분석은
[`JENKINS_SCENE0_CH1_REPRODUCIBILITY_20260821.md`](JENKINS_SCENE0_CH1_REPRODUCIBILITY_20260821.md)를
기준으로 한다.

확장 시험은 `--pair-start 4 --pair-count 5 --holdout-count 2`로 build17/18/19를
training, build20/21을 hold-out으로 실행했다. 과거 선택 RT 단독 검증은
`CANDIDATE_RT`를 반환했지만, 2026-08-24 최대 3개 finalist별 재검증에서 선택 167°와
경쟁 87°/−106°가 모두 hold-out `2/2 PASS`했다. Binary 중간 상태는
`FINALIST_HOLDOUT_AMBIGUOUS`였지만, 학습 동일 연속 목적함수에서 선택 167°의 최소
경쟁 margin이 `6.491% > 2%`로 확인돼 최신 상태는 `CANDIDATE_RT / PASS`다.
`NOT_PRODUCT_APPROVED_RT`, `activation_allowed=false`는 유지한다.
2026-08-23 수정본에서 build20/21의 영상 파일명·EXIF는 각각 scan 시작명과 일치했다.
다만 수정 전·후 prepared pixel과 score map은 동일했고, build17 편집 영상·동적 객체·
manifest 자동 검증 부재 때문에 제품 evidence로 승격하지 않는다. 수정본 build20/21
단독 실행도 각각 `FINALIST_AMBIGUOUS`, `COARSE_OVERLAP_INSUFFICIENT`로 동일하게 실패했다.

최신 후보별 결과는
[`FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md`](FINALIST_HOLDOUT_DISTINCTIVENESS_20260824.md)를
우선한다.

정확한 테스트 목록, 실행 명령, runtime, RT, scene CSV와 다음 수집 조건은
[`JENKINS_SCENE0_CH1_EXPANDED_TEST_20260822.md`](JENKINS_SCENE0_CH1_EXPANDED_TEST_20260822.md)를
기준으로 한다.

## 7. 2026-08-24 build22~24 CH1 ChArUco 계약

build22~24는 같은 설치·고정 장면에서 약 12~13분 간격으로 수집된 세 개의 독립
패키지다. CH1에는 같은 ID의 A4 ChArUco가 파란 의자 위와 우측 모니터 앞에 각각 한 장씩
있다. 전체 frame 검출은 보드가 작고 같은 ID가 중복되어 실패하므로 다음 ROI로 보드를
분리한다.

```text
monitor: x=2090, y=700,  w=500, h=650
chair:   x=1200, y=1200, w=800, h=320
```

현재 역할은 다음처럼 고정한다.

```text
training: build22, build23
hold-out: build24
```

두 보드 모두 세 build에서 camera-side pose PASS다. `estimate_marker_pose --roi`는 crop
좌표만큼 `cx,cy`를 이동하므로 출력은 계속 `T_camera_marker_board` in
`camera_optical`이다. 이 marker pose는 camera-side reference로 사용한다.

다만 전체 RT 정답은 같은 보드의 독립 `T_lidar_marker_board`까지 있어야 한다.
현재 평면 A4 문양은 LiDAR point에서 ID가 식별되지 않으므로 상태를
`RT_REFERENCE_INCOMPLETE`로 유지하고, marker pose만으로 `T_camera_lidar`를 만들거나
제품 RT를 승인하지 않는다.

입력별 수치, ROI overlay, 실행 명령과 다음 시험 순서는
[`JENKINS_SCENE0_CH1_BUILD22_24_CHARUCO_REFERENCE_PLAN_20260824.md`](JENKINS_SCENE0_CH1_BUILD22_24_CHARUCO_REFERENCE_PLAN_20260824.md)를
따른다.

## 8. build5~24 전체 활용 정책

수집한 12개 package를 모두 보존하고 사용하되 한 번의 optimization에 섞지 않는다.

```text
Case A baseline: build5/8/9 training → build10 limited hold-out
Case B stress:   build17/18/19 diagnostic training → build20/21 dynamic hold-out
Case C primary:  build22/23 clean training → build24 development hold-out
Cross-check:     Case C RT를 Case A/B에 fixed pose로 적용하며 재추정하지 않음
```

전체 CH1의 chair/monitor ChArUco 24 target 수동 audit는 `22 PASS + 2 expected FAIL`이다.
ChArUco는 camera-side K+D/pose 진단이며 targetless RT 후보 점수에는 넣지 않는다. PCD와
JSON은 같은 scan의 두 표현이므로 독립 표본으로 중복 집계하지 않고, target label이 없는
PCD를 marker RT truth로 사용하지 않는다.

pair index, ROI registry, 정확한 Docker 명령, exit code와 Luna 보고/중단 계약은
[`CH1_ALL_BUILD_DATA_UTILIZATION_AND_LUNA_EXECUTION_PLAN_20260824.md`](CH1_ALL_BUILD_DATA_UTILIZATION_AND_LUNA_EXECUTION_PLAN_20260824.md)를
단일 실행 기준으로 사용한다.

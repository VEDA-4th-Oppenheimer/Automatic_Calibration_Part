# OpenSDK CCTV 자동 외부 캘리브레이션(RT) 통합 인계서

| 항목 | 내용 |
|---|---|
| 문서 목적 | 현재 자동 캘리브레이션 Core를 Hanwha Vision OpenSDK 애플리케이션에 안전하게 연결하기 위한 개발 계약과 구현 지침 제공 |
| 대상 독자 | OpenSDK/CCTV 애플리케이션 담당자, 센서·STM32 담당자, 자동 캘리브레이션 알고리즘 담당자, 검증 담당자와 각 담당자의 Codex 작업 세션 |
| 최초 작성일 | 2026-08-20 |
| 최종 수정일 | 2026-08-20 |
| 적용 브랜치 기준 | `develop` 작업 트리의 현재 구현 |
| OpenSDK 기준 | 로컬 제공 문서 및 SDK 이미지 `26.05.19`, 대상 SoC `cv5` |
| 대상 카메라 | Hanwha Vision PNM-C16083RVQ |
| 현재 성숙도 | RT 후보 추정과 내부/hold-out 진단 가능. 제품 RT 자동 활성화는 아직 금지 |

## 1. 문서의 결론

현재 저장소는 카메라 영상과 팬·틸트 1D LiDAR 스캔을 입력받아 LiDAR 좌표계의 점을 카메라 좌표계로 옮기는 외부 파라미터 `T_camera_lidar = [R | t]` 후보를 계산할 수 있다.

OpenSDK 담당자는 알고리즘을 새로 구현하지 않는다. 다음 데이터 파이프라인과 실행 껍데기를 만든다.

1. OpenSDK에서 지정 채널의 원본 프레임을 수신한다.
2. 프레임 메모리를 소유 버퍼로 복사하고 캘리브레이션 작업 스레드로 전달한다.
3. STM32/LiDAR가 만든 완결된 스캔 JSON과 영상을 명시적으로 한 쌍으로 묶는다.
4. 현재 Calibration Core 실행 파일을 별도 작업 프로세스로 실행한다.
5. `calibration_result.json`을 엄격하게 검증한다.
6. 현 단계에서는 결과를 후보·진단 영역에만 보관하고, 활성 RT를 자동 교체하지 않는다.

알고리즘 담당자는 같은 기간에 Core의 모호성 제거, 독립 반복성, 품질 게이트 및 제품 승인 로직을 고도화한다. 두 작업은 아래의 입력·출력 계약을 유지하는 한 서로 독립적으로 진행할 수 있다.

## 2. 프로젝트 목적과 팀 분리

### 2.1 최종 목적

고정 설치된 CCTV 카메라와 팬·틸트 1D LiDAR가 동일 공간을 관측할 때, 별도의 현장 타깃 없이 두 센서 사이의 외부 파라미터 RT를 추정하고 그 결과를 Top-view 등 후속 영상 융합에 사용하는 것이 최종 목적이다.

카메라 내부 파라미터 `K, D`는 이 RT와 다른 값이다. 현재 제품 가정은 카메라·렌즈·해상도·zoom/focus 프로파일에 맞는 제조사 프로파일을 사용한다는 것이다. 현재 사용 카메라에서는 해당 프로파일을 직접 확보하지 못해, 개발 검증 동안 ChArUco로 측정한 고정 `K, D`를 대신 사용한다. 이것은 RT를 수동으로 넣는 것이 아니며, RT 추정은 계속 targetless 자동 경로로 수행한다.

### 2.2 담당 범위

| 담당 | 구현할 내용 | 구현하지 않을 내용 |
|---|---|---|
| OpenSDK 담당 | 영상 수신, 채널 매핑, 프레임 소유권, LiDAR 수신, pair/session 구성, Core 실행, 결과 저장, 상태 노출, watchdog와 rollback | Core 점수식 재작성, 임의의 PASS 기준 완화, 후보 RT 자동 활성화 |
| 센서·STM32 담당 | 문서 계약에 맞는 완결된 JSON 스캔, 단위·좌표계·valid/checksum 보장, 고유 scan ID 제공 | 카메라 좌표계로 사전 회전, 카메라 RT를 JSON에 주입 |
| 알고리즘 담당 | feature 추출, 방향 탐색, Ceres 최적화, 품질 판정, 독립 검증, 회귀 테스트 | OpenSDK event loop나 CCTV UI 구현 |
| 검증 담당 | 고정/변경 설치 데이터셋, 독립 hold-out, 결과 재현성, 실패 주입, 제품 승인 증거 관리 | 단일 예쁜 투영 이미지만으로 제품 RT 승인 |

### 2.3 병렬 개발을 위한 경계

OpenSDK 파이프라인은 RT 결과의 수학적 의미와 결과 lifecycle만 고정해서 개발한다. 내부 목적함수나 탐색 간격이 나중에 바뀌어도 입력 JSON, 카메라 프로파일, `T_camera_lidar` 의미, 결과 승인 규칙이 유지되면 OpenSDK 코드는 바뀌지 않아야 한다.

## 3. 현재 구현 상태와 중요한 제한

### 3.1 이미 구현된 것

- 카메라 원본 영상의 렌즈 왜곡 보정 경로
- 팬·틸트 LiDAR JSON 로딩과 조직화된 point cloud 구성
- 2D 영상 edge, gradient, 구조선과 LiDAR range/normal/plane/구조선 feature 구성
- 360° yaw와 제한된 down/optical-roll 방향 후보 탐색
- 이웃 후보 점수를 반영한 연속 고득점 영역 평가
- 3개 내외의 서로 다른 basin을 이용한 coarse-to-fine 탐색
- Ceres 기반 6-DoF RT 연속 최적화
- z-buffer 가시성, 투영 coverage, geometry NID, edge/structural/Manhattan 조건 평가
- 학습 pair와 검색에 쓰지 않은 hold-out pair의 분리 평가
- 상세 디버그 PNG, PLY/OBJ, CSV 및 JSON 결과 출력
- CTest 회귀 테스트

### 3.2 아직 구현·검증이 끝나지 않은 것

- 서로 다른 설치 데이터에서 동일 rigid 모듈의 RT가 허용 오차 안에서 반복되는지에 대한 제품 수준 승인
- 독립 기준 RT와의 정량 비교 및 자동 `PRODUCT_APPROVED_RT` 승격
- OpenSDK CV5 장치에서 OpenCV/Ceres 등 의존성의 교차 빌드·성능·메모리 검증
- OpenSDK 영상 시각과 LiDAR sweep 사이의 실시간 동기화 계약 확정
- 전원 장애 또는 프로세스 중단 중에도 활성 RT를 보호하는 장치 내 영속화 구현
- 다채널 동시 실행의 자원 예산과 채널별 프로파일 격리

### 3.3 현재 결과를 바로 제품에 쓰면 안 되는 이유

2026-08-18 CH1 데이터와 2026-08-19 CH1 데이터는 내부 및 hold-out 장면 검사를 통과한 후보를 만들었지만, 동일하게 결합된 카메라·LiDAR 모듈이라는 조건에서 두 후보 회전 차이가 약 `76.45°`, 이동 차이가 약 `0.078 m`였다. 이는 단일 데이터셋 내부의 높은 점수가 곧 물리적으로 유일한 RT임을 뜻하지 않는다는 증거다.

따라서 현재 실행기는 의도적으로 `product_approved_rt_status="NOT_PRODUCT_APPROVED_RT"`, `activation_allowed=false`를 기록한다. OpenSDK 담당자는 프로세스 종료 코드가 0이거나 내부 gate가 PASS라는 이유만으로 활성 RT를 교체해서는 안 된다.

2026-08-24에는 build17~21에서 선택 167°와 80°/87° 떨어진 두 finalist가 동일
hold-out을 모두 `2/2 PASS`하는 것도 확인했다. 이후 학습과 같은 연속 목적함수와 공통
coverage로 비교한 최소 margin은 `6.491%`여서 최신 상태는 `CANDIDATE_RT / PASS`다.
그러나 `product_approved_rt_status=NOT_PRODUCT_APPROVED_RT`와
`activation_allowed=false`이므로 OpenSDK가 자동 활성화하면 안 된다.

### 3.4 OpenSDK bring-up용 현재 기준 산출물

아래 두 20260820 산출물은 schema/smoke용 역사 fixture다. 현재 후보 회귀 기준은
`automatic_calibration/generated/jenkins_scene0_ch1_20260824_build17_21_objective_holdout_prior_locked/`
이다. 장치는 이 `CANDIDATE_RT`를 수신·보관할 수 있지만 상위 제품 승인 없이 활성 RT로
적용하지 않는지를 검증해야 한다.

| 데이터/산출물 | pair 구성 | 현재 선택 자세 | lifecycle | OpenSDK에서의 용도 |
|---|---|---|---|---|
| `automatic_calibration/generated/implementation_edge_soft_staged_20260820/` | 2026-08-18 CH1, train 3 + hold-out 1 | yaw 약 169°, down 21°, roll 3° | `CANDIDATE_RT`, activation false | host와 CV5가 같은 입력·binary에서 같은 schema/근사 결과를 내는 smoke 기준 |
| `automatic_calibration/generated/implementation_edge_soft_staged_20260819_20260820/` | 2026-08-19 CH1, train 2 + hold-out 1 | yaw 약 -118°, down 22°, roll 13° | `CANDIDATE_RT`, activation false | 독립 job 처리와 제품 승격 차단 검증 |

두 디렉터리의 `calibration_result.json`, `matching_scene_*.png`, `scene_*_colorized_lidar_3d_preview.png`를 함께 보관한다. 이 값은 물리 ground truth가 아니라 현재 구현의 회귀 기준이다. 알고리즘이 정당하게 개선되면 숫자는 바뀔 수 있으므로 binary Git revision과 preset hash를 항상 함께 비교한다.

## 4. 최소 OpenSDK 구조

초기 통합은 다음 네 부분이면 충분하다.

| 구성요소 | 책임 |
|---|---|
| `CalibrationComponent` | OpenSDK lifecycle 및 RawImage event 처리, 구성 로딩, 상태 조회 |
| bounded frame queue | SDK 버퍼 수명이 끝나기 전에 복사한 영상의 제한된 대기열 |
| session/pair assembler | 영상 capture와 완결된 LiDAR sweep을 명시적 ID로 연결하고 staging 디렉터리 생성 |
| calibration worker | 별도 스레드에서 Core 프로세스 실행, 결과 검증, 후보 저장과 상태 변경 |

흐름은 `OpenSDK RawImage event → owned frame queue → pair/session assembler ← LiDAR JSON inbox → calibration worker → Calibration Core → result validator → candidate store`이다. 검증된 제품 승격 기능이 생긴 뒤에만 `candidate store → atomic active RT store` 경로를 활성화한다.

초기 버전에 별도 서비스, 복수의 추상 인터페이스 계층 또는 메시지 브로커를 추가할 필요는 없다. 한 컴포넌트, 한 작업 스레드, 제한된 큐, 하나의 명시적 상태 머신으로 시작한다.

## 5. OpenSDK 대상 환경

로컬 `OpenSDK_Document` 기준으로 다음을 사용한다.

| 항목 | 값 또는 지침 |
|---|---|
| OpenSDK 개발 이미지 | `26.05.19` |
| 호스트 | Ubuntu 기반 Docker 개발환경 |
| 언어 | C++17 |
| 대상 장치 | PNM-C16083RVQ |
| SoC | `cv5` |
| 지원 펌웨어 기준 | `v25.02.25` 이상 |
| SDK 기본 경로 | `/opt/opensdk` |
| 빌드 변수 | `APP_NAME`, `SDK_VER`, `SOC=cv5` |
| 배포물 | OpenSDK `.cap` 패키지 |

SDK 문서에는 `toolchain.cmake` 직접 사용보다 Docker의 `set_toolchain.sh` 기반 구성이 권장된다. 실제 SDK 이미지에서 제공되는 컴파일러, sysroot, ABI를 사용해야 하며 WSL의 호스트 라이브러리를 `.cap`에 그대로 복사하면 안 된다.

OpenCV, Eigen, Ceres Solver, yaml-cpp, nlohmann-json의 CV5 교차 빌드 가능 여부와 라이선스/패키징 정책은 첫 번째 P0 검증 항목이다. 특히 Ceres와 그 선형대수 의존성이 장치 이미지에 기본 제공된다고 가정하지 않는다.

### 5.1 권장 OpenSDK application 형태와 디렉터리

SDK는 native application과 장치 내부 Dockerized application을 구분한다. 현재 Core는 C++ 실행 파일과 native library로 구성되어 있으므로 1차 검증은 native application으로 시작한다. 장치 내 Docker 사용은 자원·권한·배포 이점이 실제로 확인될 때 별도 결정한다.

SDK template의 주요 위치와 이 프로젝트에서 넣을 내용은 다음과 같다.

| SDK template 위치 | 이 프로젝트의 내용 |
|---|---|
| `app/src/<component_name>/` | `CalibrationComponent`, frame queue, pair assembler, worker, result validator source |
| `app/src/<component_name>/manifests/` | RawImage source와 component association |
| `app/src/PLifeCycleManagermanifest.json` | component lifecycle와 stub/skeleton 연결 |
| `app/bin/` | 빌드·설치된 실행 파일과 SDK lifecycle binary |
| `app/libs/<component_name>/` | component library와 manifest |
| `app/libs/` | CV5로 교차 빌드한 제3자 shared library. host Ubuntu/WSL binary 사용 금지 |
| `app/res/` | 기본 설정, schema, camera profile 등 읽기 전용 배포 자원 |
| `app/storage/` | candidate/active RT, job 상태, 설정, 제한된 진단 로그 등 영속 데이터 |
| `config/app_manifest.json` | 앱 이름, version, `ChannelType`, 권한과 storage 접근 정책 |
| project root | `docker-compose.yml`, 최종 `<AppName>.cap` |

camera intrinsic profile, 지원 schema, 기본 algorithm preset은 버전 관리되는 `app/res/`에 넣는다. 실행 중 만들어지는 RT와 job 결과는 `app/storage/` 계열에 둔다. 앱 upgrade 시 storage를 유지할지 선택할 수 있으므로, result schema migration과 incompatible active RT 처리도 정의해야 한다.

### 5.2 프로젝트 생성·빌드·패키징

SDK CLI 기준 작업 흐름은 다음과 같다.

1. `opensdk_new_project -n <app_name> -v <app_version> -c cv5 -s 26.05.19` 형태로 template를 만든다. 설치된 CLI 버전에서 `-c` 지원 여부와 정확한 option은 `opensdk_new_project --help`로 다시 확인한다.
2. single-channel CH1 bring-up이면 manifest의 `ChannelType`을 `Single`로 설정하고 source를 `SPMgrVideoRaw_${APPCHANNEL}` 형태로 연결한다. multi-channel 전환 전에는 이 구성을 유지한다.
3. component CMake에서 현재 Core target과 CV5용 dependency를 링크하고 필요한 shared library를 `app/libs`에 설치한다.
4. project root에서 `APP_NAME=<app_name> SDK_VER=26.05.19 SOC=cv5 docker compose up`을 실행한다.
5. Docker entrypoint가 `/opt/opensdk/common/set_toolchain.sh "$SOC"`를 source했는지 build log에서 확인한다.
6. CMake configure, build, install 및 `opensdk_packager`가 완료되면 project root의 `<app_name>.cap`을 확인한다.
7. SDK CLI의 `opensdk_install -a <app_name> -i <device_ip> -c <device_channel> -u <device_id> -w <device_password>` 형태로 시험 장치에 설치한다. credential은 저장소·로그·문서에 실제 값으로 기록하지 않는다.

SDK 문서의 예제 build 내부 흐름은 `app/build`에서 `cmake -DSOC=${SOC} ..`, `make`, `make install` 후 root에서 `opensdk_packager`를 실행하는 구조다. 생성 template의 `docker-compose.yml`을 우선 사용하고, deprecated `toolchain.cmake`를 새 구현의 기준으로 삼지 않는다.

package 생성 성공은 runtime 성공을 뜻하지 않는다. 설치 후 component 시작, 모든 shared library resolve, RawImage event 수신, storage 쓰기, child process 생성 권한 및 resource warning event를 각각 확인한다.

## 6. OpenSDK component와 thread 계약

### 6.1 lifecycle

OpenSDK component는 일반적으로 다음 lifecycle을 갖는다.

| 함수 | 이 통합에서의 책임 |
|---|---|
| `Initialize()` | 설정 검증, 채널/profile 로딩, 큐 생성, 작업 스레드 시작, 구독 등록 |
| `ProcessAEvent(Event*)` | RawImage event 역직렬화, metadata 검증, 필요한 plane을 소유 메모리로 복사, 큐에 non-blocking enqueue |
| `Finalize()` | 새 작업 차단, worker 종료 요청, 진행 중 child process 정리 정책 실행, 큐 해제, 마지막 상태 안전 저장 |

OpenSDK scheduler가 `ProcessAEvent`를 호출하므로 이 함수 안에서 이미지 변환, JSON 파싱, Ceres 실행, 파일 압축 또는 네트워크 대기를 수행하지 않는다. 같은 scheduler의 다른 component까지 지연될 수 있다.

### 6.2 RawImage source

로컬 API 문서에서 원본 영상 event는 `SPMgrVideoRaw_0` 계열 source와 `GroupSPMgrVideoRaw2`, `IPStreamProviderManagerVideoRaw::EEventType::eVideoRawData`를 사용한다. 역직렬화 결과인 `IPVideoFrameRaw`에서 `RawImage`를 얻는다.

관련 SDK header는 `i_p_stream_provider_manager_video_raw.h` 계열이다. 실제 sample과 SDK API의 `eventToArgumentBuffer`/역직렬화 관례를 그대로 사용하고, event blob의 내부 layout을 `reinterpret_cast`만으로 추측하지 않는다. `RawImage::format`은 YUV/NV12/NV21/RGB/BGR 등일 수 있으므로 BGR 고정으로 가정하지 않고 장치에서 수신한 format별 변환 테스트를 둔다.

확인할 필드는 다음과 같다.

| 필드 | 사용 목적 |
|---|---|
| `chan_id` | UI 채널과 SDK source의 매핑 검증 |
| `seq`, `pts` | 프레임 식별과 시간 순서 진단 |
| `format` | BGR/RGB/NV12/NV21/YUV 변환 경로 선택 |
| `width`, `height`, `pitch` | profile 일치와 plane 복사 |
| `num_planes`, `plane` | 포맷별 안전한 deep copy |
| `dmabuf_fd` | zero-copy 연구용. 초기 구현에서는 SDK 수명 계약을 모르면 소유 복사를 우선 |

사용자가 부르는 `CH1`과 SDK의 `chan_id=0`이 같다고 이름만 보고 단정하지 않는다. 일반적인 single-channel source는 0-based일 가능성이 높으므로, 장치에서 화면에 보이는 채널과 event의 `chan_id`, 해상도, 프레임 내용을 한 번 기록해 매핑 테이블을 확정한다. 권장 설정 표기는 `ui_channel=1`, `sdk_chan_id=0`, `source=SPMgrVideoRaw_0`처럼 세 값을 모두 저장하는 것이다.

### 6.3 메모리 소유권

`RawImage`의 plane pointer는 event callback 종료 후에도 유효하다고 가정하지 않는다. 큐 항목은 최소한 다음을 소유해야 한다.

- 변환 전 또는 BGR 변환 후의 실제 pixel byte 배열
- width, height, pitch, pixel format
- SDK channel ID, sequence, PTS
- UTC/monotonic capture timestamp
- camera profile ID와 영상 설정 snapshot

큐가 찼을 때 scheduler를 막지 말고 오래된 미사용 프레임을 버리거나 새 프레임을 거부한다. 어느 정책을 택했는지 drop count로 노출한다. 캘리브레이션 trigger와 연관된 프레임은 별도 pin 상태를 두어 일반 preview drop과 구분한다.

## 7. RT 좌표계 계약

### 7.1 RT의 정확한 의미

Core가 반환하는 값은 `T_camera_lidar`이다.

`p_camera = R_camera_lidar × p_lidar + t_camera_lidar`

| 기호 | 의미 | 단위 |
|---|---|---|
| `p_lidar` | LiDAR 축 교점 좌표계의 3D 점 | m |
| `R_camera_lidar` | LiDAR 방향을 카메라 방향으로 회전하는 3×3 행렬 | 무차원 |
| `t_camera_lidar` | LiDAR 원점이 카메라 좌표계에서 위치하는 3×1 벡터 | m |
| `p_camera` | 카메라 optical 좌표계의 점 | m |

카메라 중심을 LiDAR 좌표계에서 표현한 물리 설치값 `C_L`과 `t`는 같은 값이 아니다.

`C_L = -Rᵀt`, 따라서 `t = -R C_L`

현재 기계 prior는 `C_L=(+0.05928, -0.08105, 0) m`이다. 즉 현재 LiDAR 좌표계의 `+y down` 정의에서 카메라는 LiDAR 중심축으로부터 수평으로 59.28 mm 떨어지고 LiDAR보다 81.05 mm 위에 있다. 이 값을 결과 JSON의 `translation_m`에 그대로 넣으면 안 된다. 각 회전 후보마다 `t=-R C_L`로 계산한다.

### 7.2 카메라 투영

왜곡 보정된 영상과 그에 맞는 `K`를 사용할 때, `p_camera=(x_c,y_c,z_c)`에 대해 `z_c>0.05 m`인 점만 다음처럼 투영한다.

`u = fx × x_c / z_c + cx`, `v = fy × y_c / z_c + cy`

카메라 좌표계는 영상과 일관되게 `+x right`, `+y down`, `+z forward`이다.

### 7.3 역변환

카메라 점을 LiDAR 좌표계로 되돌릴 때는 `p_lidar = Rᵀ × (p_camera - t)`를 쓴다. 단순히 translation 부호만 바꾸면 안 된다.

### 7.4 결과 행렬 검증

결과를 저장하기 전에 다음을 확인한다.

- 모든 행렬·벡터 원소가 finite인지
- `RᵀR`이 단위행렬에 충분히 가까운지
- `det(R)`이 `+1`에 충분히 가까운지
- translation과 camera center가 장치의 물리 범위 안인지
- 입력 camera profile, 채널, 해상도와 활성 파이프라인이 동일한지
- 결과 lifecycle이 실제 활성화를 허용하는지

## 8. LiDAR JSON 입력 계약

### 8.1 기준 좌표식

현재 실제 loader와 schema 1.2 샘플이 사용하는 좌표계는 다음과 같다.

| 축/각도 | 정의 |
|---|---|
| `+x` | 오른쪽 |
| `+y` | 아래쪽 |
| `+z` | 전방 |
| pan 증가 | 실제 장치 Top-view 기준 시계 방향 |
| 계약 tilt 0 | 수평 |
| 계약 tilt 음수 | 아래 방향, 실측 범위 약 `-π/2..0` |

점 변환식은 다음이다.

`r = distance_m + sensor.range_offset_m`

`x = r × cos(tilt_rad) × sin(pan_rad)`

`y = -r × sin(tilt_rad)`

`z = r × cos(tilt_rad) × cos(pan_rad)`

`mechanism.tilt_zero=nadir`는 모터 기구 영점 metadata이다. `measurements[].tilt_rad`의 좌표 계약을 nadir 기준으로 바꾸는 필드가 아니다. 이 둘을 혼동하면 전체 point cloud가 90° 회전한다.

### 8.2 필수 root 정보

생산 입력은 최소 다음 의미를 보존한다.

| 영역 | 필수 의미 |
|---|---|
| 버전 | `interface_version`, `schema_version` |
| 식별 | `session_id`, `scan_id`, producer/firmware 식별자 |
| 센서 | 모델과 `range_offset_m` |
| 좌표 | frame 이름, right-handed 여부, 축·pan·tilt convention과 range formula |
| 단위 | 거리 m, 각도 rad, 시간 단위 |
| scan | rows, columns, pan/tilt 범위, 샘플 수 |
| mechanism | 기구 영점과 angle source 진단 정보 |
| measurements | row, column, pan, tilt, distance, signal, valid/checksum 상태 |

### 8.3 point 수용 조건

현재 loader는 사용 point에 대해 다음을 요구하거나 적용한다.

- `valid=true`
- `checksum_valid=true`
- `distance_m`이 null이 아님
- 기본 유효 거리 `0.30..20.0 m`
- 기본 `signal_strength >= 1000`
- 조직화 index는 `row × columns + column`

동일 row/column이 중복되면 현재 loader의 뒤 항목이 앞 항목을 덮을 수 있다. producer가 중복 cell을 만들지 않고, 누락·중복·범위 초과를 진단에 기록하는 것이 안전하다.

### 8.4 알려진 schema 불일치

저장소의 `automatic_calibration/schemas/pan_tilt_lidar_scan.schema.json`은 아직 `frames`를 사용하는 구 schema 1.0 형태이고, 실제 데이터와 loader는 단수 `frame`을 사용하는 schema 1.2 계열이다. OpenSDK 개발자가 이 구 schema만 보고 새 producer를 만들면 안 된다.

통합 기준은 실제 loader, 현재 schema 1.2 샘플, `PAN_TILT_LIDAR_JSON_INTERFACE.md`의 좌표식이다. 제품 배포 전에는 schema 파일과 interface 문서의 버전 표기를 하나로 정리해야 한다. 이 항목은 P0 계약 정리 작업으로 추적한다.

PLY, PCD, OBJ는 사람이 보는 진단 산출물일 뿐 Calibration Core의 정식 입력 계약이 아니다.

## 9. 카메라 profile 계약

### 9.1 현재 개발 profile

| 항목 | 값 |
|---|---|
| profile ID | `charuco-pass-clean18-20260814` |
| 해상도 | 2592×1520 |
| `fx`, `fy` | 2033.9019520107618, 2037.7796376946073 |
| `cx`, `cy` | 1337.029701465088, 745.3700555812936 |
| distortion model | OpenCV radtan |
| `D` | `[-0.5653174395, 0.3445938561, -0.0039145366, 0.0008182749, -0.1080941249]` |
| 영상 상태 | raw, LDC disabled/미제공 |
| flip/mirror | 사용 안 함 |
| corridor view | 0° |
| zoom/focus | 설정 후 고정 |

원본 영상에는 `cv::undistort`를 한 번 적용한 후 feature와 투영을 계산한다. 이미 LDC가 적용된 영상에 다시 `D`를 적용하면 double-undistortion이 발생한다.

### 9.2 profile fingerprint

OpenSDK job에는 최소 다음 camera 설정을 함께 저장한다.

- camera model과 device ID
- UI channel과 SDK channel ID
- width, height, pixel format
- K/D profile ID 및 파일 hash
- LDC 상태: `true`, `false`, `unknown`
- 영상 distortion state: `raw`, `rectified`, `unknown`
- zoom/focus 설정 또는 장치가 제공하는 고정 profile 식별값
- flip, mirror, corridor-view/rotation

해상도, zoom, focus, digital crop, rotation 또는 distortion state가 profile과 다르면
작업을 중단한다. 영상의 `raw`/`rectified` 상태가 `unknown`이면 추측하지 말고 진단
상태로 보류한다. 카메라 UI의 LDC 지원 여부가 `unknown`이어도 OpenSDK adapter가 raw
stream임을 명시적으로 보장하고 해당 raw K+D profile을 사용하면 warning을 기록한 뒤
후보 검증을 계속할 수 있다.

### 9.3 제조사 K/D가 생겼을 때

ChArUco 파일을 제조사 camera+lens profile로 교체할 수 있다. 단, 동일한 camera model이라는 이유만으로 채널·zoom·focus가 다른 profile을 재사용하면 안 된다. LDC가 켜져 있어도 rectified 출력에 대응하는 유효 `K`는 여전히 필요하다.

## 10. image–scan pairing과 session 계약

### 10.1 pair의 의미

한 pair는 동일한 고정 설치 상태와 충분히 동일한 장면을 나타내는 다음 두 데이터의 명시적 연결이다.

- 한 camera capture
- 한 완결된 LiDAR 360° pan × down sweep

파일 이름 시간은 현재 실험 데이터에서 동기화 기준이 아니었으므로, 운영 파이프라인에서는 이름 정렬에 의미를 맡기지 않는다. trigger/capture ID 또는 운영자가 확정한 association을 기록한다.

### 10.2 권장 pair metadata

| 필드 | 설명 |
|---|---|
| `job_id` | 한 번의 캘리브레이션 작업 ID |
| `session_id` | 설치 상태가 유지된 수집 묶음 |
| `pair_id` | 명시적 image–scan 연결 ID |
| `ui_channel`, `sdk_chan_id` | 채널 오해 방지 |
| `image_capture_id`, `image_pts` | 영상 식별 |
| `scan_id` | LiDAR sweep 식별 |
| `association_method` | hardware trigger, time window, operator confirmed 등 |
| `max_time_delta_ms` | 시간 association을 쓸 때의 실제 차이 |
| `installation_id` | 장치가 함께 이동해도 rigid 결합이 같은지 추적 |
| `camera_profile_id` | K/D와 영상 설정 결합 |
| `role` | `train` 또는 `holdout` |

### 10.3 현재 CLI adapter의 주의점

현재 `run_real_calibration`은 input directory의 영상 목록과 JSON 목록을 각각 사전순 정렬한 뒤 같은 index끼리 묶는다. 영상과 JSON 수가 다르면 실패하고, 결과 PNG를 같은 디렉터리에 넣으면 잘못 입력될 수 있다.

OpenSDK adapter는 job마다 격리된 staging directory를 만들고 `000_image.jpg`, `000_scan.json`, `001_image.jpg`, `001_scan.json`처럼 zero-padded 이름만 배치한다. 디버그 이미지와 결과 파일은 별도 output directory에 둔다. 장기적으로는 명시적 pair manifest를 Core adapter가 직접 읽게 개선한다.

현재 hold-out은 정렬된 pair 중 마지막 `N`개다. 따라서 staging 단계에서 train pair를 먼저, hold-out pair를 마지막에 배치하고 manifest에도 역할을 중복 기록한다.

## 11. Calibration Core 동작 순서

### 11.1 입력 검증과 전처리

1. 영상 수, scan 수, 해상도, K/D profile을 확인한다.
2. LiDAR JSON의 schema, 좌표 convention, 단위, range offset을 확인한다.
3. raw 영상이면 고정 K/D로 한 번 왜곡 보정한다.
4. 거리, signal, valid/checksum 조건을 통과한 LiDAR point만 사용한다.

제품 기본 경로에서는 K/D를 RT와 함께 움직이지 않는다. 현재 CLI에 intrinsic refinement 옵션이 있더라도 연구용이며 OpenSDK 제품 preset에서는 비활성화한다.

### 11.2 2D feature 구성

| feature | 생성 방법 | 하는 역할 |
|---|---|---|
| edge distance | grayscale 영상의 Canny edge와 distance transform | 투영된 LiDAR edge가 영상 edge에서 몇 pixel 떨어졌는지 평가 |
| gradient geometry | Sobel gradient magnitude, blur, normalize | 단순 색이 아니라 밝기 변화 구조와 LiDAR depth/normal 구조의 정보 유사성 평가 |
| grayscale intensity | 보정된 회색조 영상 | LiDAR `signal_strength`와 NMI를 연구할 때 사용. 현재 기본 weight는 0 |
| 2D segments | LSD 선분과 방향 군집 | 벽·기둥·책상 등 유한 구조선의 방향과 위치 평가 |
| vanishing/Manhattan cues | 수직·수평 방향 지원 | 후보가 장면의 주요 구조 방향과 맞는지 보조 평가 |

edge-only 방식은 반복되는 벽, 바닥, 천장 구조에서 잘못된 방향도 좋은 점수를 줄 수 있다. 그래서 현재 구현은 gradient 정보, 구조선, Manhattan 방향 및 visibility/coverage를 함께 사용한다.

Manhattan 소실점 중 수직축 선택은 finalist별 training seed prior로 고정하며 같은 축을
hold-out에서도 재사용한다. OpenSDK adapter는 이 선택을 다시 계산하거나 candidate RT에
맞춰 바꾸지 않고 Core 결과를 그대로 전달해야 한다.

### 11.3 LiDAR feature 구성

1. 조직화된 scan의 인접 cell을 이용해 surface normal을 계산한다.
2. 큰 range discontinuity를 넘어 normal을 계산하지 않아 서로 다른 표면을 섞지 않는다.
3. range 변화와 normal 변화를 이용해 geometry feature를 만든다.
4. 평면을 분할하고 이웃 재할당 및 coplanar merge를 수행한다.
5. 평면 교차선과 경계 후보를 만든다.
6. 여러 scan에서 지속되는 occlusion line은 구조선 후보로 사용한다. 단일 scan occlusion은 진단 정보로 제한한다.
7. 계산량을 제한하면서 공간 전체를 유지하도록 feature point를 최대 약 5,000개로 stratified sample한다.

장애물 표면도 정상적인 평면일 수 있으므로 모든 plane boundary가 벽–바닥 경계인 것은 아니다. 구조선 점수는 유한 선분의 위치·방향·겹침과 영상에서의 대응을 함께 봐야 한다.

### 11.4 후보 방향 생성

현재 product-oriented staged search는 yaw, down, optical roll 후보를 격자로 생성한다.

- yaw는 LiDAR 주위를 도는 카메라 광축 방위다.
- down은 수평에서 아래로 향하는 정도다.
- optical roll은 카메라 광축을 중심으로 영상이 회전한 정도다.
- 각 후보의 rotation에서 물리 camera center prior를 이용해 `t=-R C_L`를 만든다.
- full yaw 범위에서 0°와 360° 이웃 관계는 circular하게 처리한다.

현재 검증용 명시 preset은 yaw 5°, down 0..30°/5°, optical roll -15..15°/5° coarse 간격을 사용한 뒤 국소 5° 및 1° 탐색과 Ceres 최적화로 이어진다. 이 값은 영구 제품 상수가 아니라 현재 장착 범위에 대한 검증 preset이다.

### 11.5 후보 점수

후보마다 다음을 계산한다.

| 항목 | 현재 기본 가중치/역할 |
|---|---|
| geometry NID | `0.55`, 낮을수록 좋음 |
| signal NMI | `0.0`, 현재 제품 경로에서 비활성 |
| edge residual | `0.25`, 낮을수록 좋음 |
| structural line | `0.20`, 높을수록 좋게 목적함수에 변환 |
| Manhattan alignment | `0.15`, 구조 방향 gate/보조 점수 |
| coverage/visibility | 절대·상대 gate와 penalty |

가중치는 확률이 아니므로 합이 1일 필요가 없다.

가시성은 quarter-resolution z-buffer를 사용하며 가까운 표면 기준 약 10 mm 범위의 점만 보이는 것으로 취급한다. 이 처리 없이 뒤쪽 벽이나 바닥 edge까지 영상에 겹치면 잘못된 후보가 유리해진다.

geometry NID는 영상 gradient와 LiDAR range/normal feature를 2×2 공간 tile과 16-bin soft histogram으로 비교한다. `NID = 1 - MI / joint_entropy` 형태이며 낮을수록 두 modality의 구조적 정보가 잘 맞는다는 뜻이다.

coverage는 단순히 화면 안에 점 몇 개가 들어온 후보가 매우 좋은 점수를 받는 현상을 막는다. 현재 구현은 NID/spatial 상대 coverage가 약 0.5보다 작은 후보를 배제하고, edge relative coverage에도 soft penalty를 준다.

### 11.6 인접 후보 보정

한 격자점의 우연한 raw score만으로 고르지 않는다. 같은 sign/focal/roll layer의 인접 8개 후보를 이용해 다음처럼 보정한다.

`S_corrected = 0.8 × S_raw + 0.2 × weighted_mean(S_adjacent)`

이웃 weight는 yaw/down 각각 약 5° sigma의 Gaussian 거리 가중치다. overlap 및 structural 조건을 통과한 이웃만 사용한다. 이 보정은 넓고 연속적인 고득점 basin을 선호하게 하지만, 잘못된 대칭 구조가 넓게 이어지는 경우까지 정답으로 보장하지는 않는다.

### 11.7 coarse-to-fine과 multi-start

1. 보정 점수 상위 영역을 찾는다.
2. 서로 yaw가 최소 약 30° 떨어진 basin을 최대 3개 유지한다.
3. 각 basin 주변을 5° 간격, 반경 약 10°로 다시 탐색한다.
4. 다시 1° 간격, 반경 약 5°로 탐색한다.
5. 서로 다른 최종 후보를 Ceres에 넣는다.

한 후보만 최적화하면 반복 구조의 local optimum을 정답으로 오인할 수 있으므로, multi-start 결과 사이 점수와 자세 차이도 모호성 판단에 사용한다.

### 11.8 Ceres 연속 최적화

Ceres는 angle-axis rotation 3개와 translation 3개, 총 6-DoF를 최적화한다. 현재 기본 제품 경로에서 K/D는 고정한다.

| 항목 | 현재 설정 |
|---|---|
| 최대 iteration | 150 |
| 선형 solver | `DENSE_QR` |
| thread | 1 |
| seed 주변 rotation bound | 약 10° |
| seed 주변 translation bound | 약 0.1 m |
| camera-center prior sigma | 약 0.005 m |

수치 미분 중 correspondence가 갑자기 바뀌지 않도록 후보별 visible set을 고정하고, 최종 자세에서는 visibility를 다시 계산한다.

### 11.9 train과 hold-out

train pair는 탐색과 최적화에 사용한다. hold-out pair는 후보를 선택한 뒤에만 평가한다. 따라서 hold-out PASS는 같은 데이터를 재사용한 training score보다 강한 증거지만, 설치 조건 전체의 제품 재현성을 증명하지는 않는다.

현재 단일 실행에서 hold-out이 통과해도 독립 설치/반복 실행의 RT 일관성 검사가 별도로 필요하다.

### 11.10 품질 gate와 실패 코드

Core는 점수가 가장 좋은 숫자를 무조건 반환하지 않고 다음 계열을 검사한다.

- 입력·설정 오류
- `COARSE_OVERLAP_INSUFFICIENT`
- NID, entropy, spatial coverage 부족
- edge alignment와 projected coverage 부족
- signal 조건 실패
- Manhattan support 또는 `MANHATTAN_VERTICAL_ALIGNMENT_POOR`
- structural finite overlap 부족
- `MULTISTART_AMBIGUOUS`
- 목적함수/NID 개선 부족
- 기계 prior에서 과도한 이탈
- optimizer failure

예를 들어 `MANHATTAN_VERTICAL_ALIGNMENT_POOR`는 2D에서 검출된 수직 구조와 투영된 3D 구조의 방향 정렬이 기준보다 나쁘거나, 신뢰할 수 있는 수직 구조 자체가 부족하다는 뜻이다. 이는 Ceres가 실행되지 않았다는 뜻이 아니라 최종 후보를 물리적으로 신뢰할 증거가 부족하다는 뜻이다.

## 12. 코드 위치와 책임

| 파일 | 책임 | OpenSDK 담당자의 사용 방식 |
|---|---|---|
| `automatic_calibration/include/auto_calib/synthetic_lidar.hpp` | `CameraModel`, `Transform`, `Point`, `Scan`, observation 기본 타입과 좌표 변환 | RT/단위/ownership 의미 확인 |
| `automatic_calibration/include/auto_calib/calibration_core.hpp` | feature, 설정, 평가, calibration 결과의 공개 C++ API | 향후 direct-link adapter의 기준 header |
| `automatic_calibration/src/calibration_core.cpp` | feature 생성, scoring, z-buffer, multi-start, Ceres, gate 구현 | 복제하지 말고 library로 링크 |
| `automatic_calibration/apps/run_real_calibration.cpp` | 실제 파일 입력, K/D 처리, staged search orchestration, 결과·디버그 산출물, lifecycle JSON | 초기 OpenSDK worker가 실행할 기준 adapter |
| `automatic_calibration/tests/` | 합성/회귀/계약 테스트 | OpenSDK 변경 전후 host 회귀 검증 |
| `automatic_calibration/CMakeLists.txt` | C++17 및 Eigen/OpenCV/Ceres/yaml/json 의존성 | CV5용 빌드 target 추가 시 기준 |

### 12.1 `calibration_core.hpp` 공개 함수

| 함수 | 입력/출력 의미 | 사용 시점 |
|---|---|---|
| `buildCameraEdgeDistanceTransform` | BGR 영상에서 edge distance field와 edge 수 생성 | 2D edge 진단/점수 구성 |
| `detectManhattanVanishingDirections` | 영상, K에서 소실 방향 후보와 inlier/residual 생성 | 수직·수평 구조 진단 |
| `segmentLidarPlanes` | organized scan을 normal 기반 평면 label/모델로 분할 | 3D 구조선 전처리 |
| `extractLidarEdgePoints` | scan 또는 plane segmentation에서 range/normal edge point 생성 | edge 목적함수 |
| `extractLidarPlaneIntersectionSegments` | 인접 plane 교차선과 승인/거절 진단 생성 | 벽–바닥·벽–구조 경계 |
| `extractLidarPlaneBoundarySegments` | 승인 plane과 미분류 geometry 경계 생성 | 책상 등 불완전 plane 보완 |
| `extractLidarOcclusionSegments` | 단일 scan 거리 급변의 occlusion 후보 생성 | 기본적으로 진단 |
| `retainPersistentLidarOcclusionSegments` | 여러 observation에서 반복되는 occlusion만 유지 | 안정 구조선 보완 |
| `extractLidarStructuralSegments` | 현재 구조선 추출 경로의 편의 함수 | 단일 scan API |
| `calibrateExtrinsic` | 한 영상/scan과 mechanical prior에서 `CalibrationResult` 생성 | 단일 관측 진단 중심 |
| `calibrateExtrinsicMultiScene` | 여러 `CalibrationObservation`으로 공통 RT 최적화 | 실제 다중 pair Core 호출 |
| `evaluateSignalNmiPose(s)` | 고정 RT 하나/여러 개의 signal-strength NMI 평가 | signal 식별력 연구·진단 |
| `evaluateCalibrationPoseScenes` | 고정 RT를 장면별 gate metric으로 평가 | hold-out/fixed-pose 검증 |
| `calculatePoseError` | 두 transform의 rotation degree와 translation meter 차이 | ground truth/반복성 비교 |

### 12.2 `CalibrationResult`의 Core-level 의미

| 필드 | 의미 |
|---|---|
| `success` | 해당 Core 내부 gate를 통과한 estimated pose 여부 |
| `candidate_available` | gate와 무관하게 분석 가능한 최선 후보가 있는지 |
| `internal_gate_pass` | Core 내부 품질 판정 |
| `state`, `reason_code` | machine-readable 내부 상태와 원인 |
| `estimated_t_camera_lidar` | 내부 성공 경로 transform |
| `candidate_t_camera_lidar` | 거절될 수 있는 진단 후보 transform |
| `estimated_camera`, `candidate_camera` | 해당 pose와 함께 평가된 camera model |
| `metrics` | 투영, edge, NID, 구조선, Manhattan, coverage, solver 수치 |
| `coarse_orientation_scores` | coarse yaw 후보별 세부 점수와 overlap 상태 |
| `solver_summary` | Ceres 종료/iteration 진단 |

이 구조체의 `success=true`는 `run_real_calibration`이 추가하는 train/hold-out 및 제품 lifecycle 승인과 같은 의미가 아니다. OpenSDK 1차 adapter는 Core 구조체를 직접 해석하지 않고 최종 `calibration_result.json` 계약을 따른다.

중요한 현재 구조상 `run_real_calibration.cpp`에는 파일 adapter뿐 아니라 일부 전처리·탐색 orchestration과 결과 lifecycle 작성도 들어 있다. OpenSDK component에서 이 코드를 복사하면 알고리즘 담당자의 수정이 두 군데로 갈라진다.

따라서 1차 통합은 기존 executable을 worker process로 실행한다. 파이프라인이 안정된 뒤 알고리즘 담당자가 orchestration을 재사용 가능한 library API로 추출하면 direct link로 바꾼다.

## 13. 공개 C++ 타입 사용상의 주의

### 13.1 `Transform`

`Transform`은 rotation과 meter 단위 translation을 가지며 `lidarToCamera()`와 `cameraToLidar()`를 제공한다. 행렬 메모리 배치에 의존해 binary dump하지 말고 결과 JSON의 3×3 배열과 3-vector를 명시적으로 직렬화한다.

### 13.2 `CameraModel`

`CameraModel`은 3×3 K와 width/height를 보유한다. D는 현재 실제 실행 adapter의 camera profile 경로에서 관리되고 raw image를 사전 보정한다. 향후 API를 정리할 때 K와 distortion state/profile fingerprint를 하나의 immutable input으로 묶는 것이 안전하다.

### 13.3 `Scan`과 `Point`

`Scan`은 row-major organized grid다. `Point`는 xyz 외에도 range, precision, signal strength, pan/tilt, timestamp, row/column 및 flags를 가진다. feature가 조직화된 인접 관계를 사용하므로 유효 point만 모은 비조직 PLY로 대체하면 안 된다.

### 13.4 동기·thread safety

현재 공개 calibration 함수는 동기 호출이다. 호출이 끝날 때까지 observation의 image와 scan은 살아 있어야 한다. 동일 mutable object를 여러 calibration worker에서 공유하지 않는다. CV5 1차 구현은 worker 하나로 직렬 실행하고 Ceres도 한 thread로 유지한다.

## 14. 1차 통합 방식: 기존 실행 파일을 worker로 사용

### 14.1 이 방식을 먼저 쓰는 이유

- 현재 실데이터에서 실행된 것과 동일한 코드 경로를 보존한다.
- OpenSDK용으로 algorithm orchestration을 다시 작성하지 않는다.
- Core가 변경되어도 실행 파일과 결과 schema만 교체하면 된다.
- child process crash가 OpenSDK scheduler process 전체를 직접 손상시키는 위험을 줄인다.

### 14.2 실행 흐름

1. component가 충분한 train/hold-out pair를 확정한다.
2. job 전용 staging/output directory를 만든다.
3. camera profile과 입력 fingerprint를 job manifest에 기록한다.
4. worker가 `posix_spawn` 계열로 `run_real_calibration`을 실행한다.
5. stdout/stderr를 크기 제한 로그로 저장한다.
6. timeout, exit status, signal 종료를 수집한다.
7. 결과 JSON이 완전히 생성되고 parse 가능한지 확인한다.
8. lifecycle과 RT를 검증해 candidate store에 atomic rename으로 기록한다.
9. staging 보존 기간과 용량 상한에 따라 오래된 진단 데이터를 정리한다.

shell 문자열을 조립해 실행하지 말고 argument vector를 사용한다. job ID와 파일명은 허용 문자와 base directory 내부 경로인지 검증해 command injection과 path traversal을 막는다.

### 14.3 현재 검증 preset

OpenSDK 최초 smoke test는 다음 옵션 의미를 고정한다.

| 옵션 | 값 |
|---|---|
| search | `--search-strategy staged` |
| camera center | `--camera-center-x-m 0.05928 --camera-center-y-m -0.08105 --camera-center-z-m 0` |
| yaw coarse | `--yaw-step-deg 5` |
| down | `--down-min-deg 0 --down-max-deg 30 --down-step-deg 5` |
| optical roll | `--optical-roll-min-deg -15 --optical-roll-max-deg 15 --optical-roll-step-deg 5` |
| distortion | `--ldc-enabled false --image-distortion-state raw` |
| intrinsic | 현재 manual intrinsic JSON 경로 |
| hold-out | 최소 `--holdout-count 1`, 제품 승격 검증에서는 더 강화 |
| scene pass ratio | `--minimum-scene-pass-ratio 0.8` |

장치의 설치 허용 범위가 확정되기 전에는 search boundary를 임의로 더 좁히지 않는다. 성능 최적화는 정답 basin 재현이 확인된 뒤 단계별로 수행한다.

### 14.4 프로세스 종료 코드

| 종료 코드 | 현재 의미 | OpenSDK 처리 |
|---|---|---|
| 0 | 해당 실행의 최종 내부/hold-out 상태가 성공 경로 | 결과 JSON을 추가 검증. 활성 승인으로 해석 금지 |
| 3 | quality/diagnostic failure | 후보·디버그 보존, 활성 RT 유지 |
| 1 | 예외 또는 일반 실행 오류 | 작업 실패, 입력·로그 보존, retry 정책 적용 |
| signal/timeout | crash 또는 deadline 초과 | child 종료/회수, 활성 RT 유지, resource 진단 |

종료 코드는 빠른 프로세스 진단일 뿐 RT lifecycle의 source of truth가 아니다.

## 15. 결과 JSON 소비 계약

### 15.1 반드시 읽을 상태 필드

| 필드 | 의미 |
|---|---|
| `status` | 실행 결과의 상위 lifecycle 상태 |
| `internal_gate_status` | 같은 실행 내부 품질 gate |
| `candidate_rt_status` | 진단 후보 존재 여부 |
| `product_approved_rt_status` | 제품 승인 근거가 충족되었는지 |
| `activation_allowed` | 자동 활성화 허용의 최종 boolean |
| `reason_code` | 실패/보류의 기계 판독 원인 |
| camera/profile/distortion metadata | 결과가 어느 영상 프로파일에서 계산됐는지 |
| pair/train/hold-out metadata | 어떤 증거로 결과가 만들어졌는지 |
| algorithm/search config | 실행 재현에 필요한 설정 |

### 15.2 RT 필드의 구분

| 필드 계열 | 용도 |
|---|---|
| `estimated_t_camera_lidar` | 실행 결과 RT. 실패 시 prior가 남을 수 있어 상태 없이 소비 금지 |
| `diagnostic_candidate_t_camera_lidar` | gate가 거부해도 분석용으로 남긴 최선 후보 |
| `visualization_t_camera_lidar` | PNG/PLY/OBJ 진단 시각화용. 제품 활성 RT로 사용 금지 |
| `mechanical_installation_prior...` | 물리 설치 prior. 캘리브레이션 결과가 아님 |

transform JSON은 `rotation_matrix`의 3개 row와 `translation_m`의 길이 3 배열로 저장된다.

### 15.3 현재 안전한 활성화 조건

다음 조건을 모두 만족할 때만 향후 active store를 교체할 수 있다.

1. `status`가 정확히 `PRODUCT_APPROVED_RT`이다.
2. `product_approved_rt_status`가 정확히 `PRODUCT_APPROVED_RT`이다.
3. `activation_allowed`가 true이다.
4. rotation/translation 수학 검증을 통과한다.
5. camera profile, channel, resolution, LDC/rotation/zoom/focus fingerprint가 현재 stream과 일치한다.
6. input/result schema version을 adapter가 지원한다.
7. 제품 정책이 요구하는 독립 hold-out, 반복성, 기준 RT, fail-safe 증거 ID가 존재한다.

현재 실행기는 1~3을 만족시키지 않으므로 candidate만 저장하는 것이 정상 동작이다.

### 15.4 원자적 저장과 rollback

장치에는 최소 `factory_rt`, `active_rt`, `candidate_rt`, `last_known_good_rt`를 구분한다.

- 먼저 임시 파일에 내용과 checksum을 쓴다.
- `fsync` 후 같은 filesystem 안에서 rename한다.
- active 교체 직전에 기존 값을 last-known-good로 보존한다.
- 부팅 시 partial file이나 checksum 오류를 거부한다.
- 결과에는 algorithm version, Git revision, job ID, camera profile hash, LiDAR schema/version, 생성 시각을 함께 저장한다.

자동 활성화가 아직 금지된 기간에는 `candidate_rt`만 갱신하며 fusion pipeline은 기존 `active_rt`를 계속 사용한다.

## 16. 장치 상태 머신

권장 최소 상태는 다음과 같다.

| 상태 | 설명 | 허용 전이 |
|---|---|---|
| `IDLE` | 수집/계산 없음 | `COLLECTING` |
| `COLLECTING` | pair 수집 및 검증 | `READY`, `FAILED`, `CANCELLED` |
| `READY` | job 입력 확정 | `RUNNING`, `CANCELLED` |
| `RUNNING` | worker process 실행 | `CANDIDATE_READY`, `REJECTED`, `FAILED` |
| `CANDIDATE_READY` | 분석 가능한 RT 후보 저장 | 향후 승인 절차 또는 `IDLE` |
| `REJECTED` | 알고리즘 실행은 됐으나 gate 실패 | `IDLE`, 새 수집 |
| `FAILED` | 입력/프로세스/시스템 오류 | `IDLE`, 제한된 retry |
| `ACTIVE` | 제품 승인 RT가 atomically 적용됨 | rollback 또는 새 승인 |

현재 버전은 `CANDIDATE_READY`에서 `ACTIVE`로 자동 전이하지 않는다.

## 17. 제안하는 job manifest와 상태 API

아래는 현재 Core API가 아니라 OpenSDK adapter가 새로 정의할 최소 계약이다.

### 17.1 job manifest

| 필드 | 필수 | 설명 |
|---|---|---|
| `adapter_schema_version` | 예 | OpenSDK adapter 계약 버전 |
| `job_id`, `session_id`, `installation_id` | 예 | 추적과 독립성 판단 |
| `camera_profile` | 예 | K/D hash, 영상 상태, 채널, 해상도, 설정 fingerprint |
| `lidar_profile` | 예 | producer, firmware, schema, frame convention |
| `pairs` | 예 | 명시적 image/scan 경로, ID, role |
| `algorithm_preset` | 예 | 검증된 CLI option set 이름과 hash |
| `requested_by`, `requested_at` | 예 | 감사 추적 |

### 17.2 상태 조회

운영/UI에 최소 다음을 노출한다.

- 현재 state, progress stage, 시작/경과 시간
- 수집 pair 수와 train/hold-out 수
- 마지막 reason code와 짧은 설명
- drop frame 수, invalid scan 수, pairing 실패 수
- worker exit/timeout/resource peak
- candidate RT 존재 여부와 job ID
- active RT version과 last-known-good version

## 18. 디버그 산출물

| 산출물 | 용도 |
|---|---|
| `calibration_result.json` | 기계 판독 결과와 lifecycle source |
| `matching_scene_N.png` | 최종 2D 영상에 LiDAR 투영 결과 |
| `debug/scene_N/05_projection_initial.png` | 초기/prior 투영 |
| `debug/scene_N/06_projection_final.png` | z-buffer 적용 최종 투영 |
| `debug/scene_N/07_projection_final_edges.png` | 실제 점수에 사용된 LiDAR edge와 영상 edge 비교 |
| `04_lidar_surface_normals.ply` | surface normal 진단 |
| `04a_lidar_plane_labels.ply` | plane 분할 진단 |
| `04b_lidar_plane_pair_candidates.csv` | plane pair/구조선 후보 수치 |
| colorized LiDAR PLY/OBJ/preview | 3D 공간에서 영상 색이 어디에 투영됐는지 확인 |

PNG/PLY/OBJ는 사람이 방향을 확인하는 필수 진단 자료지만 활성화 판단의 유일한 근거가 아니다. OpenSDK 장치에서는 저장 공간 때문에 모든 중간 파일을 영구 보존하지 말고, 실패·후보 작업은 충분히 보존하고 정상 smoke 작업은 rolling quota를 적용한다.

## 19. 검증 계획과 완료 기준

### 19.1 P0: OpenSDK 파이프라인 bring-up

- PNM-C16083RVQ/CV5에서 CH1 RawImage event 수신
- UI CH1 ↔ SDK `chan_id` 실측 매핑
- 2592×1520 및 pixel format/pitch 변환 정확성 확인
- callback 종료 후에도 안전한 owned buffer 확인
- LiDAR schema 1.2 JSON 수신/파일화와 checksum 검증
- 명시적 pair manifest 및 격리 staging 검증
- CV5용 모든 Core 의존성 교차 빌드·동적 라이브러리 로딩 확인
- scheduler thread가 calibration 중에도 지연되지 않는지 확인
- 결과 JSON parser와 candidate-only 저장 확인
- schema 1.0/1.2 문서 불일치 해소

### 19.2 host 회귀

- host Ubuntu 빌드 성공
- `ctest --test-dir <build-dir> --output-on-failure` 전체 통과
- 현재 2026-08-18 CH1 dataset으로 알려진 후보/상태가 허용 tolerance 안에서 재현
- 2026-08-19 CH1 dataset을 별도 독립 job으로 실행
- 두 결과의 큰 RT 불일치가 제품 승격을 반드시 막는지 확인
- raw/LDC, resolution, profile, flip/mirror mismatch가 명확한 reason code로 중단되는지 확인

### 19.3 장치 실패 주입

- 잘린 JSON, null distance, 중복 cell, 잘못된 단위
- 영상/scan 수 불일치와 잘못된 pair ID
- queue overflow와 dropped frame
- worker timeout, crash, out-of-memory
- 결과 JSON partial write, checksum mismatch
- 재부팅 중 candidate 저장
- 활성 RT가 있는 상태의 실패 작업

모든 경우 기존 active/last-known-good RT가 훼손되지 않아야 한다.

### 19.4 제품 승격 정책

현재 문서화된 목표 기준은 적어도 다음 증거를 요구한다.

| 항목 | 목표 |
|---|---|
| 독립 hold-out | 최소 3 pair 이상 |
| 반복 실행 | 약 10회 |
| rotation 표준편차 | 0.2° 이하 목표 |
| translation 표준편차 | 10 mm 이하 목표 |
| 초기 perturbation 회복률 | 90% 이상 목표 |
| 독립 기준 RT | 별도 측정/검증 증거 필요 |
| false activation | 0회 |

이 수치는 단일 장면 내부 PASS를 제품 승인으로 바꾸기 위한 최소 정책 초안이며, 실제 장치 공차와 사용 목적에 따라 검증 담당자가 최종 확정한다.

### 19.5 장치 자원 측정

각 job에 wall time, CPU time, peak RSS, output 용량, queue drop, scheduler latency를 기록한다. 먼저 정확한 결과와 안전한 lifecycle을 검증한 뒤 coarse step, feature 수, 영상 해상도 및 debug level로 성능을 조정한다.

장치 자원상 full Core가 불가능하면 fallback은 OpenSDK가 capture/pairing만 수행하고 입력을 외부 Ubuntu worker로 전송하는 것이다. 이 경우에도 본 문서의 입력·RT·결과 lifecycle 계약은 그대로 유지한다.

## 20. OpenSDK 담당자 구현 순서

### P0 — 데이터가 안전하게 흐르게 만들기

1. SDK 26.05.19 CV5 sample app을 빌드·설치한다.
2. CH1 RawImage metadata와 한 장의 owned BGR frame을 저장해 host 이미지와 비교한다.
3. LiDAR schema 1.2 완결 sweep을 inbox에 넣고 좌표/단위 validator를 만든다.
4. explicit pair/session manifest와 job staging을 만든다.
5. 현재 Core executable과 모든 runtime library를 CV5로 패키징한다.
6. worker thread에서 child process를 실행하고 timeout/exit/log를 수집한다.
7. 결과 JSON을 candidate-only로 atomic 저장한다.
8. OpenSDK scheduler latency와 메모리 사용을 측정한다.

### P1 — 운영 안정성

1. 상태 API와 reason code 노출
2. 큐 overflow, cancellation, retry, watchdog
3. 저장 quota와 실패 산출물 회수
4. camera/LiDAR profile fingerprint와 mismatch 차단
5. 부팅 복구와 last-known-good 보호
6. 알고리즘 binary/config/schema version 호환성 검사

### P2 — 제품 승인 연결

1. 알고리즘 측의 반복성/독립 RT 검증 결과 schema 수용
2. 서명되거나 checksum이 있는 approval record 저장
3. 제한된 조건에서 candidate → active atomic promotion
4. health check 실패 시 자동 rollback
5. CH2~CH4 채널별 profile·RT·작업 격리와 자원 스케줄링

## 21. OpenSDK 담당자가 변경하면 안 되는 것

- LiDAR 좌표를 임의로 90°/180° 돌려 영상에 맞춰 보이게 하지 않는다.
- pan 부호 또는 tilt 의미를 adapter 내부에서 숨겨서 뒤집지 않는다.
- camera center prior를 `translation_m`으로 직접 기록하지 않는다.
- K/D mismatch를 무시하거나 raw 영상에 undistort를 두 번 하지 않는다.
- edge가 눈으로 비슷하다는 이유로 gate threshold를 낮추지 않는다.
- `estimated_t_camera_lidar`를 status 검사 없이 활성화하지 않는다.
- 동일 pair를 이름만 바꿔 독립 hold-out 또는 반복 증거로 세지 않는다.
- `run_real_calibration.cpp`의 알고리즘 부분을 OpenSDK 저장소에 복사하지 않는다.

## 22. Codex 작업 세션용 인계 프롬프트

팀원은 새 Codex 세션에 이 문서와 저장소를 제공한 뒤 다음 요청을 사용할 수 있다.

> Hanwha Vision OpenSDK 26.05.19, PNM-C16083RVQ, CV5용 자동 외부 캘리브레이션 adapter를 구현한다. 먼저 `automatic_calibration/docs/OPENSDK_RT_INTEGRATION_HANDOFF.md`, `include/auto_calib/synthetic_lidar.hpp`, `include/auto_calib/calibration_core.hpp`, `apps/run_real_calibration.cpp`, OpenSDK Programming Guide/API/Supported Devices 문서를 읽고 계약을 대조한다. 1차 구현은 한 OpenSDK component, bounded owned-frame queue, explicit image–scan pair/session assembler, 한 worker thread, 기존 `run_real_calibration` child process, strict result validator 및 candidate-only atomic store로 제한한다. `ProcessAEvent`에서 calibration이나 파일 I/O를 실행하지 않는다. UI CH1과 SDK channel ID를 실측해 기록한다. `T_camera_lidar`는 `p_C=R p_L+t`, 단위 m이고 camera center prior와 t를 혼동하지 않는다. 현재 `activation_allowed=false`이므로 active RT를 자동 교체하는 코드를 활성화하지 않는다. OpenSDK CV5 교차 빌드와 Ceres/OpenCV runtime 의존성, scheduler latency, peak memory를 검증하고 테스트 결과와 남은 blocker를 문서화한다. 기존 Calibration Core 알고리즘은 복사하거나 임의 변경하지 않는다.

Codex가 구현 전에 반드시 확인해야 할 항목은 다음이다.

- 대상 OpenSDK sample component와 manifest의 실제 경로
- RawImage source 및 UI/SDK 채널 매핑
- SDK buffer 수명과 pixel format 변환
- CV5 sysroot에서 Core 의존성 링크 가능 여부
- LiDAR JSON schema 1.2 기준과 stale schema 파일의 차이
- active RT가 아닌 candidate-only lifecycle

## 23. 담당자 간 전달물

### OpenSDK 담당자 → 알고리즘 담당자

- 장치에서 저장한 원본 한 프레임과 metadata
- 완결 LiDAR JSON 및 pair manifest
- 실행한 exact algorithm preset과 binary Git revision
- `calibration_result.json`과 실패 stdout/stderr
- wall time, CPU, peak RSS, scheduler latency, output 용량
- 장치에서 생성한 투영 PNG/PLY 또는 host로 회수한 동일 산출물

### 알고리즘 담당자 → OpenSDK 담당자

- versioned Core binary/library와 runtime dependency 목록
- 지원 input/result schema version
- 검증된 preset ID와 option 목록
- reason code 목록과 운영 분류
- 변경된 RT lifecycle 및 migration 지침
- regression dataset의 expected 범위

## 24. 관련 문서

- [`automatic_calibration/README.md`](../README.md)
- [`automatic_calibration/docs/PRODUCT_CALIBRATION_POLICY.md`](PRODUCT_CALIBRATION_POLICY.md)
- [`automatic_calibration/docs/PAN_TILT_LIDAR_JSON_INTERFACE.md`](PAN_TILT_LIDAR_JSON_INTERFACE.md)
- [`automatic_calibration/docs/CAMERA_PNM_C16083RVQ_SPECIFICATION.md`](CAMERA_PNM_C16083RVQ_SPECIFICATION.md)
- [`automatic_calibration/docs/REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md`](REAL_CALIBRATION_TEST_FAILURE_ANALYSIS_REPORT.md)
- [`automatic_calibration/docs/CURRENT_PROGRESS_AND_STATUS.md`](CURRENT_PROGRESS_AND_STATUS.md)
- [`manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json`](../../manual_calibration/output/session-const-env/intrinsic/camera_intrinsic.json)
- [`OpenSDK_Document/HTML/HanwhaVision_OpenPlatform_Programming_Guide_26.05.19.html`](../../../OpenSDK_Document/HTML/HanwhaVision_OpenPlatform_Programming_Guide_26.05.19.html)
- [`OpenSDK_Document/HTML/HanwhaVision_OpenPlatform_SDK_API_26.05.19.html`](../../../OpenSDK_Document/HTML/HanwhaVision_OpenPlatform_SDK_API_26.05.19.html)
- [`OpenSDK_Document/HTML/HanwhaVision_OpenPlatform_Supported_Devices_26.05.19.html`](../../../OpenSDK_Document/HTML/HanwhaVision_OpenPlatform_Supported_Devices_26.05.19.html)

## 25. 수정 이력

| 날짜 | 버전 | 수정 내용 | 작성/수정 주체 |
|---|---|---|---|
| 2026-08-20 | 1.0 | 현재 Calibration Core 구현, RT/카메라/LiDAR 계약, OpenSDK event·thread 구조, candidate lifecycle, 장치 통합·검증·인계 절차 최초 작성 | Codex, 사용자 요구 기반 |

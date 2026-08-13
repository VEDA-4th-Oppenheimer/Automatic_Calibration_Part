# 개발환경 운영 가이드

- 작성일: 2026-08-13
- 대상 브랜치: `develop`
- 기본 실행 환경: Ubuntu native
- Docker 상태: 선택적인 로컬 개발/CI 보조 환경

## 1. 목적과 원칙

이 프로젝트는 Docker 컨테이너가 없어도 Ubuntu에서 소스 checkout 후 바로 빌드·테스트할
수 있어야 한다. 따라서 Git 저장소의 기본 경로는 호스트 파일시스템의 다음 명령으로
정의한다.

```bash
./scripts/install-ubuntu-deps.sh
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

Dockerfile과 `compose.yaml`은 개발자 간 동일한 도구 버전을 맞추거나 Jenkins에서 격리된
환경을 만들 때만 사용한다. Docker 명령을 실행하지 않아도 제품 코드와 단위 테스트의
빌드·실행에는 영향이 없어야 한다.

## 2. 지원 범위와 전제

| 항목 | 기준 |
|---|---|
| OS | Ubuntu 22.04/24.04 LTS 계열, x86_64 기준 |
| C++ | C++17, GCC 또는 Clang |
| Build | CMake 3.20 이상, Ninja 권장 |
| 자동 캘리브레이션 | Eigen3, OpenCV, Ceres Solver, yaml-cpp, nlohmann-json |
| 수동 캘리브레이션 | OpenCV `aruco` 모듈, nlohmann-json |
| Top-View GUI | Qt 6 Widgets, OpenCV, nlohmann-json |
| 테스트 | CTest와 프로젝트별 native test executable |
| 대용량 데이터 | Git 외부 경로에 보관하고 CLI로 경로 전달 |

`ubuntu:latest` 컨테이너는 패키지 설치 재현용 참고값일 뿐, 애플리케이션이 컨테이너
전용 경로(`/workspace`, `/workspace-build`)를 요구한다는 뜻은 아니다.

## 3. 최초 설치

저장소 루트에서 실행한다.

```bash
cd /path/to/Automatic_Calibration_Part
./scripts/install-ubuntu-deps.sh
./scripts/verify-dev-env.sh
```

설치 스크립트는 다음을 포함한다.

- 컴파일러와 디버거: `build-essential`, `gdb`
- 빌드 도구: `cmake`, `ninja-build`, `pkg-config`
- 자동 캘리브레이션: `libeigen3-dev`, `libopencv-dev`, `libceres-dev`,
  `libyaml-cpp-dev`, `nlohmann-json3-dev`
- 수동 캘리브레이션: OpenCV 개발 패키지의 `aruco` 모듈
- GUI: `qt6-base-dev`, `qt6-base-dev-tools`
- 보조 도구: Git, Python 3, `clang-format`

일반 사용자는 `sudo` 권한이 필요하다. 사내 이미지나 최소 Ubuntu 이미지에서는 먼저
`sudo`를 설치하거나 root 셸에서 스크립트를 실행한다.

환경 확인 스크립트는 OS, 컴파일러, CMake, Ninja, Python과 Eigen/OpenCV/PCL/yaml-cpp
패키지 버전을 출력한다. 버전 출력은 CI 로그와 함께 보관해 재현성 문제를 추적한다.

## 4. 빌드 프로파일

### 4.1 전체 workspace 빌드(권장)

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --parallel
```

실행 파일은 `build/bin/`에 생성된다.

### 4.2 증분 빌드

```bash
cmake --build build --parallel
```

### 4.3 독립 모듈 빌드

전체 GUI 의존성이 필요 없는 Core 개발에서는 자동 캘리브레이션만 빌드할 수 있다.

```bash
cmake -S automatic_calibration -B build/automatic -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/automatic --parallel
ctest --test-dir build/automatic --output-on-failure
```

수동 캘리브레이션과 Top-View GUI는 각각의 `CMakeLists.txt`를 독립 진입점으로 사용할
수 있지만, 공통 산출물 경로와 테스트를 한 번에 확인하려면 전체 workspace 빌드를
사용한다.

## 5. 테스트와 검증

```bash
ctest --test-dir build --output-on-failure
```

테스트 목록을 먼저 확인하려면 다음을 실행한다.

```bash
ctest --test-dir build -N
```

GUI smoke test는 화면을 띄우지 않는 offscreen 모드로 실행된다. WSLg/X11이 없는 서버에서도
전체 테스트가 실행되어야 한다.

```bash
QT_QPA_PLATFORM=offscreen ctest --test-dir build --output-on-failure
```

빌드와 테스트가 끝난 뒤 실행 파일의 도움말을 확인한다.

```bash
build/bin/run_real_calibration --help
build/bin/run_synthetic_calibration --help
build/bin/calibration_top_view_gui --help
```

## 6. 모듈별 실행

### Automatic Calibration

상세 옵션과 실제 데이터 제약은
[`automatic_calibration/README.md`](../automatic_calibration/README.md)에 둔다.

```bash
build/bin/run_real_calibration \
  --input-dir /path/to/image-scan-pairs \
  --output automatic_calibration/generated/result \
  --camera-channel 1 \
  --ldc-enabled unknown
```

실제 입력과 생성 결과는 Git에 넣지 않는다. `--output`은 `automatic_calibration/generated/`
또는 별도의 작업 디렉터리를 사용한다.

### Manual Calibration

```bash
build/bin/generate_charuco_board \
  --display-profile galaxy-tab-s7 \
  --output-dir manual_calibration/output/tablet-board
```

ChArUco 촬영·intrinsic·marker pose 절차는
[`manual_calibration/docs/USAGE.md`](../manual_calibration/docs/USAGE.md)를 따른다.

### Top-View GUI

WSLg 또는 X11 display가 있는 개발 PC에서 실행한다.

```bash
build/bin/calibration_top_view_gui \
  --image /path/to/camera.png \
  --camera /path/to/camera_intrinsic.json \
  --rt /path/to/automatic_result.json
```

GUI가 없는 서버에서는 `--smoke-test` 또는 offscreen CTest만 실행한다.

## 7. 데이터와 산출물 관리

다음 항목은 크기·개인정보·재현 산출물 특성 때문에 Git에 커밋하지 않는다.

- `data/`, `archive/`
- `manual_calibration/data/`, `manual_calibration/output/`
- `automatic_calibration/generated/`
- `*.pcd`, `*.ply`, `*.obj`
- `build/`와 Python cache

데이터는 로컬 디스크나 Jenkins workspace에 두고, 명령행 인자로 절대 경로 또는 저장소
외부 경로를 전달한다. 재현에 필요한 작은 schema/example JSON과 문서는 Git에 포함한다.

카메라 접근 정보는 소스나 문서에 기록하지 않는다. 스냅샷 스크립트는 다음 환경변수를
사용하며, 비밀번호가 없으면 실행 중 프롬프트를 표시한다.

```bash
export CCTV_BASE_URL="http://camera-host"
export CCTV_USER="admin"
export CCTV_PASSWORD="<사용자 환경의 비밀값>"
./scripts/capture-cctv-snapshot.sh 1 /tmp/camera-capture
```

## 8. Docker 선택 사용

Ubuntu에 패키지를 설치할 수 없거나 Jenkins에서 실행 환경을 고정해야 할 때만 사용한다.

```bash
docker compose build
docker compose up -d
docker compose exec -T dev cmake -S /workspace -B /workspace-build -G Ninja
docker compose exec -T dev cmake --build /workspace-build
docker compose exec -T dev ctest --test-dir /workspace-build --output-on-failure
docker compose down
```

Docker/VS Code Dev Container를 사용할 때는 개인 경로를 저장소에 기록하지 않도록
`.env.example`을 복사해 로컬 `.env`를 만든다.

```bash
cp .env.example .env
# .env의 STANFORD_AREA1_PATH를 실제 데이터 경로로 수정
```

컨테이너의 `/workspace`와 `/workspace-build`는 Docker 내부 경로다. native 실행 명령에
이를 복사하지 않는다. WSL에서 UID 1000 충돌이 발생하면 다음처럼 현재 사용자 UID/GID를
전달한다.

```bash
DEV_UID="$(id -u)" DEV_GID="$(id -g)" docker compose build
```

Docker 빌드가 실패해도 native 경로의 CMake/CTest 결과와 분리해 기록한다.

## 9. WSL 및 GUI 주의사항

- WSLg를 사용하는 경우 Linux 경로(`/mnt/c/...`)를 입력으로 전달할 수 있다.
- Qt 창이 필요한 실행은 WSLg 또는 X11 display가 필요하다.
- Jenkins/headless 환경에서는 `QT_QPA_PLATFORM=offscreen`을 사용한다.
- Windows 경로를 C++ CLI에 직접 전달하지 말고 WSL 경로로 변환한다.
- 생성 파일을 Windows 탐색기에서 확인하더라도 원본 입력은 Git 외부에 유지한다.

## 10. 장애 대응

| 증상 | 확인/조치 |
|---|---|
| `Could not find Eigen3/OpenCV/Ceres` | `./scripts/install-ubuntu-deps.sh` 후 `./scripts/verify-dev-env.sh` 실행 |
| `Qt6Widgets` 또는 display 오류 | GUI 없는 환경이면 `QT_QPA_PLATFORM=offscreen`; GUI PC면 WSLg/X11 확인 |
| `nlohmann_json` 미검출 | `nlohmann-json3-dev` 설치 여부와 `pkg-config` 경로 확인 |
| 빌드 결과가 예전 코드와 다름 | 별도 `build/`를 사용하거나 CMake 재구성 후 재빌드 |
| Docker `UID is not unique` | `DEV_UID=$(id -u) DEV_GID=$(id -g) docker compose build` |
| 데이터 파일이 Git status에 나타남 | `.gitignore` 대상인지 확인하고 대용량 입력은 저장소 외부로 이동 |
| 카메라 snapshot 인증 실패 | `CCTV_BASE_URL`, `CCTV_USER`, `CCTV_PASSWORD`를 환경변수로 설정 |

## 11. 재현 로그

실험 또는 CI 결과에는 최소한 다음을 함께 기록한다.

```bash
git rev-parse --short HEAD
git status --short
cmake --version | head -n 1
g++ --version | head -n 1
./scripts/verify-dev-env.sh
```

자동 캘리브레이션의 입력 데이터 경로, 카메라 채널, LDC 상태, zoom/focus 상태, 실행
옵션과 출력 디렉터리도 결과 JSON/보고서에 남긴다.

## 수정 로그

| 날짜 | 내용 |
|---|---|
| 2026-08-13 | Ubuntu native를 Git 저장소의 기본 실행 경로로 정의하고 설치·빌드·테스트·Docker 선택 사용·데이터 보안 정책을 문서화 |

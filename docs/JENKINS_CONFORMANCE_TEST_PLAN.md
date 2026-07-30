# Jenkins CI/CD Conformance Test 운영 계획

## 1. 목적

Stanford 2D-3D-S 기반 합성 pan-tilt LiDAR 데이터로 Calibration Core를
지속적으로 검증한다.

검증 범위:

- C++17 빌드 및 단위 테스트
- Stanford RGB/depth/pose 입력 conformance
- Synthetic `PointCloudPackage` schema 및 재현성
- Calibration ground-truth 복원 정확도
- 노이즈, dropout, overlap 부족에 대한 강건성
- 처리 시간과 메모리 회귀

OpenSDK/CV5 빌드 및 실기기 호환성은 이 Jenkins job의 범위에서 제외한다.

## 2. 권장 Jenkins 구성

### 2.1 Jenkins node

전용 Linux/WSL 또는 Linux VM agent에 다음 label을 부여한다.

```text
auto-calib && docker && stanford2d3ds
```

권장 최소 자원:

| 항목 | 권장값 |
|---|---:|
| CPU | 8 core 이상 |
| RAM | 16 GB 이상 |
| Docker storage | 50 GB 이상 |
| Dataset storage | 100 GB 이상 |
| Workspace storage | 30 GB 이상 |

동일 데이터 cache를 사용하는 job은 동시 실행으로 인한 I/O 포화를 막기 위해
concurrency를 제한한다.

```groovy
options {
    disableConcurrentBuilds()
    timestamps()
    timeout(time: 2, unit: 'HOURS')
}
```

## 3. Dataset과 test asset 배치

### 3.1 권장 방식: workspace 외부 read-only cache

Jenkins workspace는 SCM checkout, `cleanWs()`, node 재할당 또는 관리자의
cleanup 정책에 의해 삭제될 수 있다. 31 GB 이상의 원본 데이터셋을 workspace에
영구 보관하지 않는다.

전용 agent에 다음 구조로 한 번만 배치한다.

```text
/srv/jenkins-datasets/
  stanford-2d3ds/
    area_1/
      3d/
      data/
      pano/
      raw/
    checksums/
      area_1.sha256
    VERSION
```

권한 예시:

```bash
sudo chown -R root:jenkins /srv/jenkins-datasets/stanford-2d3ds
sudo find /srv/jenkins-datasets/stanford-2d3ds -type d -exec chmod 0750 {} \;
sudo find /srv/jenkins-datasets/stanford-2d3ds -type f -exec chmod 0640 {} \;
```

Jenkins job은 원본을 read-only bind mount한다.

```bash
export STANFORD_AREA1_PATH=/srv/jenkins-datasets/stanford-2d3ds/area_1
docker compose up -d
```

`compose.yaml`의 mount:

```yaml
volumes:
  - ${STANFORD_AREA1_PATH}:/datasets/stanford2d3ds/area_1:ro
```

장점:

- workspace cleanup과 무관하게 데이터 유지
- 원본 변조 방지
- 여러 build가 같은 데이터를 재사용
- 매 build마다 31 GB를 복사하지 않음

### 3.2 workspace에 test file을 미리 둘 경우

반드시 Jenkins가 관리하는 별도 persistent workspace 또는 custom workspace를
사용한다.

```text
/var/lib/jenkins/persistent/auto-calib/
  datasets/stanford-2d3ds/area_1/
  cases/
  expected/
```

Pipeline에서 custom workspace를 지정할 수 있다.

```groovy
agent {
    node {
        label 'auto-calib && docker && stanford2d3ds'
        customWorkspace '/var/lib/jenkins/persistent/auto-calib/workspace'
    }
}
```

이 경우 다음 규칙을 지킨다.

1. `cleanWs()`로 persistent dataset 경로를 삭제하지 않는다.
2. SCM checkout 경로와 dataset 경로를 분리한다.
3. dataset은 read-only로 Docker에 mount한다.
4. 생성 결과는 `build-output/${BUILD_NUMBER}`에 기록한다.
5. 오래된 생성 결과만 별도 cleanup job으로 삭제한다.
6. 동일 workspace를 공유하면 `disableConcurrentBuilds()`를 사용한다.

가능하지만 workspace 외부 cache 방식이 더 안전하다.

### 3.3 Artifact repository를 사용하는 방식

새 Jenkins node가 자주 생성되는 환경에서는 dataset 전체 또는 선정된 case bundle을
다음 저장소에 보관한다.

- S3/MinIO
- Nexus raw repository
- Artifactory generic repository
- 사내 NAS

예시 bundle:

```text
stanford-conformance-v1/
  manifest.json
  cases/
    C001/
      rgb.png
      depth.png
      pose.json
    C002/
      ...
  SHA256SUMS
```

Daily job은 작은 curated bundle만 내려받고, weekly/monthly job은 node-local
전체 dataset cache를 사용한다.

## 4. Test case 저장 구조

SCM에서 test 정의와 기대값을 관리한다. 대용량 원본 RGB/depth/point cloud는
SCM에 commit하지 않는다.

```text
develop/
  conformance/
    suites/
      commit.yaml
      daily.yaml
      weekly.yaml
      monthly.yaml
      release.yaml
    cases/
      C001_office_nominal.yaml
      C002_hallway_degenerate.yaml
      C003_multiplane.yaml
      C004_low_overlap.yaml
      C005_depth_dropout.yaml
      C006_range_noise.yaml
      C007_bad_initial_pose.yaml
      C008_invalid_schema.yaml
      C009_determinism.yaml
      C010_multi_scene.yaml
    expected/
      C001.json
      C002.json
      ...
    schema/
      conformance_case.schema.json
```

Case 파일은 원본 파일을 복사하지 않고 globally unique한 `frame_id`로 참조한다.

```yaml
case_id: C001_office_nominal
enabled: true
tags: [smoke, nominal, office]

source:
  dataset: stanford_2d3ds
  area: area_1
  frame_id: camera_0004591bfdc749a88db196a5d8b345cb_office_6_frame_0

synthetic_lidar:
  shape: [121, 321]
  pan_range_deg: [-40, 40]
  tilt_range_deg: [-25, 25]
  pixel_stride: 2
  noise_stddev_m: 0.0
  dropout_probability: 0.0
  seed: 7

ground_truth:
  translation_m: [0.15, -0.02, 0.08]
  rotation_rpy_deg: [2.0, -4.0, 6.0]

expected:
  generation_status: PASS
  minimum_valid_ratio: 0.40
  deterministic: true
```

## 5. Test schedule과 목록

### 5.1 Commit/PR test

트리거:

- Pull request
- main branch push
- 수동 실행

목표 실행 시간: 5분 이내

| ID | 시험 | 판정 |
|---|---|---|
| PR-001 | CMake configure/build | exit code 0 |
| PR-002 | C++ unit tests | 100% pass |
| PR-003 | CLI `--help` 및 argument validation | 기대 exit code |
| PR-004 | C001 단일 frame package 생성 | status PASS |
| PR-005 | PCD organized shape | `121 × 321` |
| PR-006 | Manifest/JSON parse | schema valid |
| PR-007 | C009 동일 seed 재현성 | golden hash 또는 두 실행 hash 일치 |

권장 case 수: 1~3개.

### 5.2 Daily test

권장 시간: 매일 02:00 KST. Jenkins hash cron을 사용하면 부하를 분산할 수 있다.

```groovy
triggers {
    cron('TZ=Asia/Seoul\nH 2 * * *')
}
```

Jenkins에 설치된 cron parser가 `TZ=`를 지원하는지 확인한다. 지원하지 않으면
controller timezone을 기준으로 환산하고 실제 첫 실행 시각을 확인한다.

목표 실행 시간: 15~30분.

| ID | 시험 | Case/조건 |
|---|---|---|
| D-001 | 전체 unit test | 모든 CTest |
| D-002 | Nominal office | C001 |
| D-003 | Hallway degeneracy | C002 |
| D-004 | Multi-plane room | C003 |
| D-005 | Low overlap rejection | C004 |
| D-006 | Depth dropout | 10%, 20%, 40% |
| D-007 | Range noise | 5, 10, 20 mm |
| D-008 | Initial pose perturbation | roll/pitch/yaw ±2°, ±5° |
| D-009 | Schema negative tests | 필드 누락, 잘못된 shape/unit |
| D-010 | Determinism | 고정 seed 2회 |
| D-011 | Output integrity | PCD/EXR/PNG/JSON/YAML 존재 및 parse |
| D-012 | Runtime regression | 최근 baseline 대비 허용 범위 |

권장 dataset: 10~20개 curated frame.

### 5.3 Weekly test

권장 시간: 일요일 03:00 KST.

```groovy
triggers {
    cron('TZ=Asia/Seoul\nH 3 * * 0')
}
```

목표 실행 시간: 1~3시간.

| ID | 시험 | Case/조건 |
|---|---|---|
| W-001 | Room category coverage | office/hallway/conference/WC/pantry |
| W-002 | Frame coverage | 100~500 frame |
| W-003 | Noise × dropout matrix | 4 × 4 조합 |
| W-004 | 6-DoF perturbation sweep | 축별 positive/negative |
| W-005 | Scan resolution sweep | 61×161, 121×321, 241×641 |
| W-006 | FOV sweep | narrow/nominal/wide |
| W-007 | Range gate sweep | near/mid/far |
| W-008 | Multi-scene | 3/5/10 scene |
| W-009 | Repeated seed test | 10 seeds |
| W-010 | Memory and runtime | peak RSS, p50/p95 |
| W-011 | Dataset checksum | selected file 및 manifest checksum |
| W-012 | Docker rebuild | no-cache 또는 base image validation |

### 5.4 Monthly test

권장 시간: 매월 첫째 일요일.

목표 실행 시간: 야간 장시간 허용.

| ID | 시험 | 내용 |
|---|---|---|
| M-001 | area_1 broad regression | 가능한 전체 또는 stratified 전체 |
| M-002 | Full parameter matrix | noise/dropout/FOV/resolution/pose |
| M-003 | Long-run stability | 반복 생성 및 resource leak |
| M-004 | Golden baseline refresh candidate | 차이 보고만 생성 |
| M-005 | Dependency inventory | compiler/CMake/OpenCV/PCL/Ceres 버전 |
| M-006 | Docker image rebuild | 최신 `ubuntu:latest` 영향 확인 |
| M-007 | Dataset integrity | 전체 SHA-256 검증 |
| M-008 | Trend report | 정확도, 실패율, runtime, memory 추세 |

Golden 결과는 monthly job이 자동 교체하지 않는다. 사람이 차이를 검토하고 별도
승인된 commit으로 갱신한다.

### 5.5 Release candidate test

Release tag 또는 수동 승인 후 실행한다.

| ID | 시험 | 필수 조건 |
|---|---|---|
| R-001 | Commit + daily + weekly suite | 전부 PASS |
| R-002 | 고정 Docker image digest | 기록 필수 |
| R-003 | 고정 dataset version/checksum | 기록 필수 |
| R-004 | Calibration accuracy gate | 승인된 threshold |
| R-005 | Known failure behavior | 기대 FAIL reason 일치 |
| R-006 | Report completeness | JUnit/JSON/plots/artifacts |
| R-007 | Reproducibility rerun | 동일 결과 |

## 6. Jenkins job 구성

권장 job:

```text
auto-calib-pr
auto-calib-daily
auto-calib-weekly
auto-calib-monthly
auto-calib-release
auto-calib-dataset-verify
auto-calib-cleanup
```

Parameter 예시:

```groovy
parameters {
    choice(
        name: 'SUITE',
        choices: ['commit', 'daily', 'weekly', 'monthly', 'release'],
        description: '실행할 conformance suite'
    )
    string(
        name: 'CASE_FILTER',
        defaultValue: '',
        description: '특정 case/tag만 실행'
    )
    booleanParam(
        name: 'REBUILD_IMAGE',
        defaultValue: false,
        description: 'Docker image를 다시 빌드'
    )
}
```

## 7. Jenkinsfile 예시

```groovy
pipeline {
    agent {
        label 'auto-calib && docker && stanford2d3ds'
    }

    options {
        disableConcurrentBuilds()
        timestamps()
        timeout(time: 3, unit: 'HOURS')
        buildDiscarder(logRotator(
            numToKeepStr: '30',
            artifactNumToKeepStr: '10'
        ))
    }

    parameters {
        choice(
            name: 'SUITE',
            choices: ['commit', 'daily', 'weekly', 'monthly', 'release'],
            description: 'Conformance suite'
        )
        string(name: 'CASE_FILTER', defaultValue: '')
        booleanParam(name: 'REBUILD_IMAGE', defaultValue: false)
    }

    environment {
        STANFORD_AREA1_PATH =
            '/srv/jenkins-datasets/stanford-2d3ds/area_1'
        COMPOSE_PROJECT_NAME = "auto-calib-${BUILD_NUMBER}"
    }

    stages {
        stage('Checkout') {
            steps {
                checkout scm
            }
        }

        stage('Dataset preflight') {
            steps {
                sh '''
                    test -d "$STANFORD_AREA1_PATH/data/rgb"
                    test -d "$STANFORD_AREA1_PATH/data/depth"
                    test -d "$STANFORD_AREA1_PATH/data/pose"
                    test -r "$STANFORD_AREA1_PATH/3d/pointcloud.mat"
                '''
            }
        }

        stage('Build environment') {
            steps {
                dir('develop') {
                    sh '''
                        if [ "$REBUILD_IMAGE" = "true" ]; then
                            docker compose build --pull
                        fi
                        docker compose up -d
                    '''
                }
            }
        }

        stage('Build') {
            steps {
                dir('develop') {
                    sh '''
                        docker compose exec -T dev \
                          cmake -S /workspace -B /workspace-build -G Ninja
                        docker compose exec -T dev \
                          cmake --build /workspace-build
                    '''
                }
            }
        }

        stage('Unit tests') {
            steps {
                dir('develop') {
                    sh '''
                        mkdir -p "build-output/${BUILD_NUMBER}"
                        docker compose exec -T dev \
                          ctest --test-dir /workspace-build \
                          --output-on-failure \
                          --output-junit \
                          /workspace/build-output/${BUILD_NUMBER}/ctest.xml
                    '''
                }
            }
        }

        stage('Conformance') {
            steps {
                dir('develop') {
                    sh '''
                        # conformance runner 구현 후 사용:
                        # docker compose exec -T dev \
                        #   /workspace-build/run_conformance \
                        #   --suite /workspace/conformance/suites/${SUITE}.yaml \
                        #   --dataset /datasets/stanford2d3ds/area_1 \
                        #   --output /workspace/build-output/${BUILD_NUMBER}

                        # 현재 단일 smoke case:
                        OPENCV_IO_ENABLE_OPENEXR=1 \
                        docker compose exec -T \
                          -e OPENCV_IO_ENABLE_OPENEXR=1 dev \
                          /workspace-build/generate_synthetic_scan \
                          --dataset-root \
                            /datasets/stanford2d3ds/area_1 \
                          --output \
                            /workspace/build-output/${BUILD_NUMBER}/C001 \
                          --columns 321 --rows 121 --pixel-stride 2 \
                          --noise-stddev-m 0.005 \
                          --dropout 0.01 --seed 7
                    '''
                }
            }
        }
    }

    post {
        always {
            junit(
                testResults: 'develop/build-output/*/ctest.xml',
                allowEmptyResults: true
            )
            archiveArtifacts(
                artifacts:
                    'develop/build-output/**/*',
                fingerprint: true,
                allowEmptyArchive: true
            )
            dir('develop') {
                sh 'docker compose down --remove-orphans || true'
            }
        }
    }
}
```

## 8. Test runner가 생성해야 할 결과

각 case:

```text
build-output/<build-number>/
  summary.json
  junit.xml
  C001/
    result.json
    manifest.yaml
    calibration/
    cloud/
    qa/
  metrics/
    runtime.csv
    memory.csv
    accuracy.csv
```

`result.json` 권장 필드:

```json
{
  "case_id": "C001_office_nominal",
  "status": "PASS",
  "reason_codes": [],
  "translation_error_m": null,
  "rotation_error_deg": null,
  "valid_ratio": 0.64,
  "runtime_ms": 120,
  "peak_rss_mb": 180,
  "input_hash": "",
  "output_hash": "",
  "software_version": "",
  "docker_image_digest": ""
}
```

Calibration optimizer가 구현되기 전에는 accuracy 필드를 `null`로 두고 생성,
schema, topology와 determinism만 판정한다.

## 9. 보존과 cleanup 정책

| 산출물 | 보존 |
|---|---|
| PR 결과 | 최근 10~30 build |
| Daily summary/JUnit | 30~90일 |
| Weekly summary | 6~12개월 |
| Monthly trend | 장기 보존 |
| Release 결과 | release 수명 동안 |
| 원본 dataset | 별도 cache, 자동 삭제 금지 |
| 생성 PCD/이미지 | 실패 case 우선 보존 |

성공한 모든 PCD를 장기간 보관하면 저장공간이 빠르게 증가한다. 성공 build는
summary와 hash 위주로 보존하고, 실패한 case의 입력 설정과 결과 package를
archive한다.

Cleanup job은 다음만 대상으로 한다.

```text
workspace/build-output/*
Docker build cache
오래된 Jenkins artifact
```

다음은 삭제 대상에서 제외한다.

```text
/srv/jenkins-datasets/**
conformance/cases/**
conformance/expected/**
release baseline/**
```

## 10. Dataset 초기 배치 절차

1. Jenkins agent를 중지하거나 관련 job 실행을 잠시 막는다.
2. dataset을 임시 경로에 복사한다.
3. 디렉터리 구조와 RGB/depth/pose 개수를 확인한다.
4. 전체 또는 주요 파일의 SHA-256을 생성한다.
5. 최종 cache 경로로 atomic rename한다.
6. 소유자와 read-only 권한을 설정한다.
7. `auto-calib-dataset-verify` job을 실행한다.
8. Daily job에서 작은 smoke case를 실행한다.

예시:

```bash
rsync -a --info=progress2 \
  /source/area_1/ \
  /srv/jenkins-datasets/stanford-2d3ds/area_1.tmp/

find /srv/jenkins-datasets/stanford-2d3ds/area_1.tmp \
  -type f -print0 |
  sort -z |
  xargs -0 sha256sum \
  > /srv/jenkins-datasets/stanford-2d3ds/checksums/area_1.sha256

mv \
  /srv/jenkins-datasets/stanford-2d3ds/area_1.tmp \
  /srv/jenkins-datasets/stanford-2d3ds/area_1
```

기존 dataset을 교체할 때는 새 버전을 별도 디렉터리에 검증한 뒤 symlink를
전환한다.

```text
/srv/jenkins-datasets/stanford-2d3ds/
  area_1-v1/
  area_1-v2/
  area_1-current -> area_1-v2
```

## 11. 운영 체크리스트

- [ ] Jenkins agent label과 Docker 권한 확인
- [ ] dataset cache가 workspace 외부인지 확인
- [ ] Docker mount가 read-only인지 확인
- [ ] dataset version 및 SHA-256 기록
- [ ] case YAML과 expected 결과를 SCM에서 관리
- [ ] PR/daily/weekly/monthly job 분리
- [ ] Jenkins controller timezone 확인
- [ ] JUnit과 JSON summary archive
- [ ] 실패 case 결과 package 보존
- [ ] Golden baseline 자동 갱신 금지
- [ ] Docker image digest 기록
- [ ] dataset cleanup 제외 규칙 확인

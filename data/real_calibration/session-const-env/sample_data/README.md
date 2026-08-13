# Session-const-env CH1 sample

- 측정 세션: `20260811-130333`
- 센서: TOFSense-F2P 1D LiDAR
- 채널: CH1
- 총 크기: 약 27 MB

## 파일

| 파일 | 용도 |
|---|---|
| `20260811-130429-CH1.png` | calibration 입력으로 사용하는 원본 CH1 카메라 이미지 |
| `calib-20260811-130333_sweep-000001_pan_tilt_lidar.json` | calibration 입력으로 사용하는 LiDAR sweep JSON |
| `ch1_top_view_image.png` | 사람이 결과를 비교하기 위한 참고용 Top-View 이미지. 입력 pair에 포함하지 않음 |

실행기는 입력 디렉터리 안의 이미지와 JSON을 sorted order로 1:1 매칭한다. 따라서
참고용 Top-View PNG까지 같은 디렉터리에 넣어 실행하면 pair 수가 맞지 않아 실패한다.

## Ubuntu native 실행

저장소 루트에서 먼저 빌드한다.

```bash
./scripts/install-ubuntu-deps.sh
cmake -S . -B build -G Ninja
cmake --build build --parallel
```

샘플 실행용 입력 pair만 임시 디렉터리에 복사한다.

```bash
sample_dir=data/real_calibration/session-const-env/sample_data
run_dir="/tmp/auto-calib-session-const-ch1"
rm -rf "${run_dir}"
mkdir -p "${run_dir}"
cp "${sample_dir}/20260811-130429-CH1.png" "${run_dir}/camera_ch1.png"
cp "${sample_dir}/calib-20260811-130333_sweep-000001_pan_tilt_lidar.json" \
  "${run_dir}/scan.json"

build/bin/run_real_calibration \
  --input-dir "${run_dir}" \
  --output /tmp/auto-calib-session-const-ch1-result \
  --camera-channel 1 \
  --ldc-enabled unknown \
  --legacy-range-offset-m 0.084
```

Docker를 사용하는 경우에도 동일한 파일을 `/workspace` 아래에서 참조할 수 있지만,
저장소의 기본 실행 경로는 위 Ubuntu native 명령이다.

## 예상 결과와 제한

이 샘플은 관측 1개만 포함한다. 따라서 현재 calibration core 정책상 실제 보정값을
확정하지 않고 다음 진단 상태를 반환한다.

```text
SINGLE_OBSERVATION_DIAGNOSTIC_ONLY / FAIL
```

이는 실행 실패가 아니라 공동 intrinsic/extrinsic 최적화에 필요한 최소 3개 이상의
구조적으로 다른 관측이 없다는 의미다. 출력 디렉터리에서 다음 결과를 확인할 수 있다.

- `calibration_result.json`: 상태, 입력 pair, 후보 RT와 제한 사유
- `matching_scene_0.png`: 단일 관측의 투영 진단 이미지
- `debug/scene_0/`: `--debug-output`를 추가했을 때의 특징·평면·투영 중간 결과

중간 산출물을 생성하려면 실행 명령에 다음을 추가한다.

```bash
--debug-output /tmp/auto-calib-session-const-ch1-debug
```

여러 관측으로 PASS 여부를 검증하려면 같은 설치 상태에서 서로 다른 시점/장면의
카메라 이미지와 LiDAR JSON pair를 3개 이상 준비해야 한다.

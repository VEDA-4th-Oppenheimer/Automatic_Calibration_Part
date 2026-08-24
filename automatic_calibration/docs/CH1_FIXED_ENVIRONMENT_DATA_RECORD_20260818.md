# CH1 고정환경 데이터 수집 확인 기록

작성일: 2026-08-18  
최종 운영자 확인: 2026-08-18  
대상: `session-const-env/repeat_test_sample/20260818`  
검증 범위: CH1, 조명 ON, A4 ChArUco 보드 부착 상태  
installation epoch: `session-const-env-20260818-ch1-fixed`

## 1. 운영자 확인

2026-08-18에 수집한 세트는 사용자가 재확인한 바와 같이 카메라·LiDAR·pan-tilt actuator의 설치 상태와 촬영
공간을 유지한 **고정환경 반복 데이터**다. 현재 디렉터리에 있는 네 세트 전체가 같은 설치
epoch에 포함되며, 수집 중 카메라·LiDAR·actuator를 이동시키지 않았고 보드와 장면도
동일하게 유지했다. 운영자 확인에 따라 파일명 시각이 서로 다르다는 이유만으로 서로
다른 외부 파라미터를 적용해야 하는 데이터로 분류하지 않는다.

이 기록의 “고정환경”은 센서의 상대적인 설치 자세, pan-tilt 기준, 카메라 영상 설정,
보드 위치 및 정적 장면이 해당 수집 회차 동안 유지됐다는 뜻이다. 따라서 네 파일은 한
회차의 반복성·hold-out 검증에 함께 사용할 수 있다. 반대로 actuator를 건드렸거나
카메라의 zoom/focus/LDC/영상 방향을 변경한 회차는 이 기록에 합치지 않고 새
`installation_epoch`로 기록한다.

이 확인은 이번 데이터의 설치 epoch를 정의하는 운영자 기록이다. 이미지와 JSON의
wall-clock 시각이 정확히 동기화됐다는 뜻은 아니며, 정적 장면에서 같은 센서 상대 자세를
반복 사용한다는 의미다. 이후 actuator를 건드리거나 카메라 zoom/focus/LDC/영상 방향을
변경하면 새 epoch를 만들고 별도 기록한다.

### 1.1 최근 추가 파일에 대한 운영자 확인

다음 네 image–scan 입력은 모두 같은 고정환경 회차로 보존한다.

- CH1 이미지 4개: `20260818-143751`, `145847`, `151305`, `155208`
- LiDAR JSON 4개: `calib-20260818-143748`, `145912`, `151312`, `154229`
- 장치 이동, actuator 재홈밍, 카메라 설치 변경, 보드·장면 변경: 없음(운영자 확인)
- 조명 조건: 조명 ON 공식 검증 그룹

따라서 이번 회차에서는 물리적인 동시 촬영 pair가 없다는 이유로 데이터를 폐기하지
않고, 문서에 고정한 결정론적 입력 연결을 사용한다. 단, 동적 장면·시간 동기화·다른
installation epoch의 일반화 검증에는 이 확인을 재사용하지 않는다.

## 2. 입력 파일 목록

| scene | CH1 이미지 | LiDAR scan JSON | 용도 |
|---:|---|---|---|
| 0 | `20260818-143751-CH1.jpg` | `calib-20260818-143748_sweep-000001_pan_tilt_lidar.json` | RT 추정(training) |
| 1 | `20260818-145847-CH1.jpg` | `calib-20260818-145912_sweep-000001_pan_tilt_lidar.json` | RT 추정(training) |
| 2 | `20260818-151305-CH1.jpg` | `calib-20260818-151312_sweep-000001_pan_tilt_lidar.json` | RT 추정(training) |
| 3 | `20260818-155208-CH1.jpg` | `calib-20260818-154229_sweep-000001_pan_tilt_lidar.json` | 고정 RT hold-out |

입력 디렉터리:

```text
data/real_calibration/session-const-env/repeat_test_sample/20260818/
```

네 이미지 모두 CH1이며, A4 보드가 부착된 동일한 설치 상태에서 촬영됐다. JSON은
연속 sweep 파일이다. JSON 내부에는 wall-clock 촬영시각이 없고 monotonic timestamp만
있으므로 파일명 시각 차이만으로 센서 간 시간 동기를 주장하지 않는다.

## 3. Pairing 해석

고정된 공간과 센서 상대 자세에서는 extrinsic 검증의 핵심이 장면의 동일한 구조와
설치 epoch 유지 여부다. 따라서 이번 데이터는 파일명 기준으로 결정론적으로 연결해
실행할 수 있으며, 매번 동일한 사진과 동일한 JSON을 다시 짝지어야만 유효하다는 뜻은
아니다.

다만 다음 조건은 계속 지킨다.

1. 조명 상태가 다른 이미지를 같은 조명 그룹의 검증 입력으로 섞지 않는다.
2. 다른 설치 epoch의 scan/image를 같은 RT 추정 세트로 합치지 않는다.
3. 실제 움직임이 있었거나 장면이 바뀐 경우에는 시간 근접 pair와 별도 manifest를
   사용하고, 고정환경 데이터로 재분류하지 않는다.
4. 센서 시간 동기화 자체를 검증해야 하는 시험은 별도 timestamp/manifest를 수집한다.

## 4. 검증 데이터 분할

처음 세 쌍으로 RT를 추정하고 네 번째 쌍에는 RT를 고정해 적용한다.

```text
training:  scene 0, 1, 2
hold-out:  scene 3
```

이 분할은 같은 고정 설치에서의 반복 재현성을 확인하기 위한 것이다. 구조가 서로
다른 공간으로 이동했을 때의 일반화나 Manual RT와의 절대 정확도 인증을 의미하지
않는다.

## 5. 현재 판정과 산출물

- ChArUco: 네 이미지 모두 `17/17 marker`, `24/24 corner` 검출 PASS
- 자동 RT 추정: `ch1_20260818_three_pair_v1`에서 training `3/3 PASS`
- 고정 RT hold-out: `ch1_20260818_holdout_155208_fixed_v1`에서 `1/1 PASS`
- 최신 4세트 재실행: `ch1_20260818_four_pair_recheck_v2`에서 training `3/3 PASS`, hold-out `1/1 PASS`
- 현재 상태: `candidate PASS / fixed-environment hold-out PASS`
- 운영 RT 교체: 보류. Manual RT와의 절대 차이 및 독립 기하학적 기준은 아직 확정하지 않음

주요 산출물:

```text
automatic_calibration/generated/ch1_20260818_three_pair_v1/
automatic_calibration/generated/ch1_20260818_holdout_155208_fixed_v1/
automatic_calibration/generated/ch1_20260818_four_pair_recheck_v2/
manual_calibration/output/session-const-env/validation/20260818-repeat/
```

## 6. 후속 시험 규칙

- 현재 20260818 데이터는 고정환경 기준 회차로 보존한다.
- actuator 또는 카메라를 건드리면 `installation_epoch`를 증가시키고 새 RT를 추정한다.
- 조명 ON이 공식 제품 조건이다. 조명 OFF/나이트비전은 별도 참고 그룹으로 관리한다.
- 다음 승인 단계에서는 이 epoch와 다른 날짜의 고정환경 회차를 추가해 RT의 회전·이동
  분산을 계산한다.

## 수정 이력

| 날짜 | 변경 내용 |
|---|---|
| 2026-08-18 | 최신 운영자 확인을 반영해 `repeat_test_sample/20260818` 현재 네 세트 전체를 동일한 고정환경·동일 설치 epoch로 확정하고, 고정환경의 의미와 새 epoch 분리 조건을 명시 |
| 2026-08-18 | 최근 추가된 20260818 CH1 네 세트가 동일 설치·동일 장면의 고정환경 데이터임을 재확인하고, `session-const-env-20260818-ch1-fixed` epoch와 결정론적 pairing 범위를 명시 |
| 2026-08-18 | 사용자의 최신 확인(“고정된 환경”)을 반영해 동일 epoch 판정을 유지하고, pairing을 시간 동기화가 아닌 offline 입력 연결로 재확인 |
| 2026-08-18 | 최근 추가 세트를 포함한 4세트 재실행 결과(`3 training + 1 hold-out`, `PASS`)와 기존 hold-out 수치 일치를 기록 |

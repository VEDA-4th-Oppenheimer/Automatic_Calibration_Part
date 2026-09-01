# 1D LiDAR Pan-Tilt JSON Interface

작성일: 2026-07-29  
인터페이스 버전: 1.0  
문서 revision: 1.0.1

## 목적

TOFSense F2P와 pan-tilt actuator 담당 팀이 Calibration Core 팀에 전달해야 하는 한 번의 전체 sweep 데이터 형식을 정의한다.

- JSON 파일 하나는 완성된 sweep 하나를 나타낸다.
- 실제 좌표 `x/y/z`는 전달하지 않는다. Calibration Core adapter가 `distance_m`, `pan_rad`, `tilt_rad`로 계산한다.
- invalid 거리에는 JSON에서 허용되지 않는 `NaN`을 쓰지 않고 `null`을 사용한다.
- 각도는 radian, 거리는 meter, host timestamp는 nanosecond로 고정한다.
- F2P의 `signal_strength`는 정규화하지 않은 `uint16` 원본값을 전달한다.
- 이 JSON이 센서/엑추에이터 팀과 Calibration 팀 사이의 golden reference다. STM32 binary packet이나 PCD/Protobuf transport는 이 계약으로 변환되는 하위 구현이다.

## 필수 파일

- `schemas/pan_tilt_lidar_scan.schema.json`: JSON Schema Draft 2020-12
- `schemas/pan_tilt_lidar_scan.example.json`: 전달 예제

## 좌표계

외부 JSON의 `frame` 객체가 측정 좌표계의 기준이다.

```text
+x: right
+y: down
+z: forward
pan positive: right
measurement tilt positive: up
```

`measurements[].tilt_rad`는 계약각이다. `mechanism.tilt_zero`는 모터 기구축의 홈 의미이며,
계약각의 원점을 정의하지 않는다.

```text
measurement tilt=0°   : 수평 전방(+z)
measurement tilt=-90° : 수직 아래(+y), 천장 설치 시 바닥
measurement range     : -90° ~ 0° (130333 실측)
```

거리와 계약각으로부터 내부 `lidar_scan` point를 계산하는 식:

```text
x = range * cos(tilt) * sin(pan)
y = -range * sin(tilt)
z = range * cos(tilt) * cos(pan)
```

JSON의 `pan_rad`, `tilt_rad`, `distance_m`은 위 식의 `pan`, `tilt`, `range`로 해석한다.
`mechanism.tilt_zero=nadir`를 보고 계약각의 0°가 nadir라고 해석하면 안 된다.
실장 확인 결과 pan 값 증가는 Top-view 기준 시계 방향이며, JSON의 `pan+ right`
계약과 함께 사용한다.

카메라 중심 offset은 LiDAR JSON 필드가 아니다. 원점은 팬/틸트 축 교점이며, 카메라 중심과의
상대 위치는 외부 파라미터 `t_camera_lidar`가 추정할 대상이다. `range_offset_m`은 축교점과
LiDAR 발광면의 거리 보정으로 point range에만 적용한다.

PLY/OBJ는 내부 `lidar_scan` 좌표계와 mm 단위로 export하며, 계산 내부 단위는 meter다. viewer용 파일은 다음 표시 변환만
적용한다.

```text
viewer_z_up = (lidar_x, lidar_z, -lidar_y)
```

외부 ICD에서 별도 표기하는 `x_icd=d*cos(phi)*cos(theta)` 식을 사용할 경우에는 그 좌표계와
현재 `lidar_scan` 좌표계를 별도 이름으로 유지해야 하며, 두 식을 동일한 식으로 문서화하지 않는다.

organized scan ordering:

```text
index = row * columns + column
row 0 = tilt_max
last row = tilt_min
column 0 = pan_min
last column = pan_max
```

## Measurement 필수 필드

| 필드 | 타입 | 단위 | 의미 |
|---|---|---|---|
| `sequence` | uint | - | sweep 내부 수신 순번 |
| `row`, `column` | uint | - | organized scan cell |
| `timestamp_ns` | uint64 | ns | 동기화된 최종 측정 시각 |
| `device_time_ms` | uint32 | ms | F2P system time 원본 |
| `encoder_timestamp_ns` | uint64 | ns | 보간에 사용한 encoder 기준 시각 |
| `pan_rad`, `tilt_rad` | number | rad | 측정 순간 각도 |
| `pan_encoder_count`, `tilt_encoder_count` | uint | count | encoder 원본값 |
| `distance_m` | number/null | m | 유효 거리 또는 null |
| `distance_status` | uint8 | raw | F2P 원본 상태 |
| `signal_strength` | uint16 | raw | F2P 원본 신호 세기 |
| `range_precision_m` | number/null | m | 변환된 거리 정밀도 |
| `checksum_valid` | bool | - | NLink checksum 결과 |
| `interpolation_valid` | bool | - | encoder 보간 결과 |
| `valid` | bool | - | Calibration 입력 사용 여부 |
| `quality_flags` | string[] | - | invalid 원인 또는 품질 상태 |

## Firmware transport 변환

| Firmware/수신 값 | JSON 필드 | 변환 |
|---|---|---|
| `sequence_number` | `sequence` | 값 보존 |
| `stm32_timestamp_us` | `timestamp_ns` | Clock offset/drift 보정 후 ns로 변환 |
| F2P system time | `device_time_ms` | 원본 ms 값 보존 |
| `pan_encoder_raw`, `tilt_encoder_raw` | `pan_encoder_count`, `tilt_encoder_count` | 값 보존 |
| `distance_mm` | `distance_m` | `× 0.001` |
| `range_precision_mm` | `range_precision_m` | `× 0.001` |
| Packet checksum | `checksum_valid` | 검증 결과를 bool로 기록 |
| Motor/scan 상태 | `valid`, `quality_flags` | 가속·방향전환·fault 규칙 적용 |

STM32 packet의 byte offset, endianness와 CRC 계산식은 firmware protocol 문서에서 관리한다. Calibration 팀은 transport 세부사항에 의존하지 않고 이 JSON 계약만 사용한다.

## Valid 판정

기본 판정:

```text
valid =
    checksum_valid
    AND interpolation_valid
    AND distance_status is accepted by the F2P parser
    AND min_range <= distance_m <= max_range
    AND actuator is not in an excluded acceleration/direction-change interval
```

`distance_status`의 허용 code는 사용하는 F2P firmware 문서와 실측 결과로 확정하고 parser 설정에 기록한다. 원본 status는 항상 JSON에 남긴다.

invalid sample 규칙:

```json
{
  "distance_m": null,
  "valid": false,
  "quality_flags": ["DISTANCE_STATUS_INVALID"]
}
```

## 일관성 검증

파일 인수 시 다음 조건을 검사한다.

1. `measurements.length == scan.sample_count`
2. `scan.valid_count == count(measurement.valid == true)`
3. `row < rows`, `column < columns`
4. `(row, column)` 중복 없음
5. `started_at_ns <= timestamp_ns <= ended_at_ns`
6. valid sample은 `distance_m != null`
7. invalid sample은 `VALID_RANGE` flag를 가지지 않음
8. `checksum_error_count` 등 diagnostics가 measurement flags와 일치

## 파일명

권장 형식:

```text
<session_id>_<scan_id>_pan_tilt_lidar.json
```

예:

```text
calib-20260729-001_sweep-000001_pan_tilt_lidar.json
```

## 전송 및 운영

JSON은 인터페이스 검증, replay, 장애 분석용 기준 포맷이다. 실제 고속 연속 운용에서 파일 크기나 parsing 시간이 문제가 되면 동일 field 계약을 유지한 PCD/Protobuf/MCAP을 추가할 수 있다. 포맷을 변경하더라도 JSON을 golden reference로 유지한다.

### 향후 권장 메타데이터 — 현재 구현의 선행조건 아님 (2026-08-12)

현재 schema 1.1 실측 데이터의 `pan_rad`, `tilt_rad`, `distance_m`은 Calibration
Core 입력으로 사용할 수 있다. 다음 필드는 지금 즉시 추가하거나 기존 JSON을 수정해야
하는 필수사항이 아니며, 이후 producer schema를 정리할 때 자체 설명성과 장애 분석성을
높이기 위한 권장사항으로만 관리한다.

- `sensor.range_offset_m`
- `frame.origin`, `frame.range_formula`
- `frame.pan_positive_direction = clockwise_top_view`
- `frame.pan_zero_reference = mechanical_home`
- pan 영점 보정이 `measurements[].pan_rad`에 이미 반영됐는지 나타내는 명시적 상태

현재 오정합은 위 메타데이터 부재가 직접 원인이 아니므로 producer 변경을 기다리지 않고
Calibration Core 목적함수의 가시성 및 구조선 대응을 먼저 수정한다.

## 수정 로그

| 버전 | 수정일 | 수정 내용 |
|---|---|---|
| 1.0 | 2026-07-29 | TOFSense F2P, encoder, timestamp, organized scan, diagnostics를 포함하는 최초 JSON 계약 |
| 1.0.1 | 2026-07-29 | 계획서와 단위·필드명을 통일하고 firmware-to-canonical 변환 규칙 명시 |

## 수정 로그 추가 (2026-08-12)

`mechanism.tilt_zero=nadir`는 기구축 홈 메타데이터이고 `measurements[].tilt_rad`는 좌표식에 사용하는 계약각임을 명시했다. 계약각은 0° 수평, 음수 아래 방향으로 처리하며, `tilt_zero` 필드만 보고 +90° 보정하지 않는다. 카메라 중심 offset은 JSON에 넣지 않고 외부 파라미터 추정 대상으로 분리했다.

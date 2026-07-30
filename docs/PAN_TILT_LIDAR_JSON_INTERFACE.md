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

`lidar_scan` 좌표계는 다음과 같다.

```text
+x: right
+y: down
+z: forward
pan positive: right
tilt positive: up
```

거리와 각도로부터 point를 계산하는 식:

```text
x = range * cos(tilt) * sin(pan)
y = -range * sin(tilt)
z = range * cos(tilt) * cos(pan)
```

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

## 수정 로그

| 버전 | 수정일 | 수정 내용 |
|---|---|---|
| 1.0 | 2026-07-29 | TOFSense F2P, encoder, timestamp, organized scan, diagnostics를 포함하는 최초 JSON 계약 |
| 1.0.1 | 2026-07-29 | 계획서와 단위·필드명을 통일하고 firmware-to-canonical 변환 규칙 명시 |

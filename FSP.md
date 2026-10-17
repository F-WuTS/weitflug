# Flyby Serial Protocol (FSP)

FSP streams FC measurements over UART and accepts companion RC commands through
the MSP receiver path. Packet definitions live in `src/main/fsp/fsp_frame.h`.

## Build and configuration

SpeedyBee F405 MINI (CRSF, DShot and attitude estimation):

```sh
./build_f405_mini.sh
ENABLE_PT2_FILTER_IMU=0 FSP_TARGET_HZ=600 ./build_f405_mini.sh
```

Foxeer H743 (Ghost receiver, DShot, OSD and VTX):

```sh
./build.sh
```

The H7 recipe uses the default 1100 Hz FSP ceiling. The script-specific defaults
in the table below refer to `build_f405_mini.sh`.

Assign serial function `524288` (`FUNCTION_FSP`) to a dedicated UART and select
its MSP baud rate. The link uses non-inverted 8N1.

| Define | Default | Meaning |
| --- | --- | --- |
| `USE_FSP` | Defined | Enable FSP |
| `FSP_PERIOD_HZ` | 1100 Hz; script: 600 Hz | I/O task rate and capture-rate ceiling |
| `FSP_PID_LOCKSTEP` | Defined | Capture every Nth PID iteration |
| `ENABLE_PT2_FILTER_IMU` | Undefined/off; script: 1 | Independent 80 Hz PT2 telemetry filter |
| `FSP_ENABLE_MAVLINK` | Undefined/off | Include the MAVLink tunnel |
| `FSP_MAVLINK_TUNNEL_SIZE` | 64 bytes when enabled | Tunnel capacity per packet |
| `ACC_TASK_PERIOD_HZ` | 1000 Hz | Initial accelerometer/attitude task rate |
| `PERF_DISABLE_ATTITUDE` | Undefined/off | Remove attitude estimation and related navigation features |
| `ESC_XR8_PRO` | Undefined/off | Select car ESC telemetry |
| `USE_MSP_PUSH` | Undefined/off | Enable the older MSPv2 streaming task |
| `USE_ACC_RPM_FILTER` | Defined with DShot telemetry | Enable accelerometer RPM filtering |

`FSP_TARGET_HZ` accepts 300, 600 or 1100. Presence-based options (`#ifdef`)
are disabled by removing their definition, rather than setting them to zero.

## Telemetry and timing

A valid heartbeat or RC packet starts telemetry and renews its **500 ms lease**.
The heartbeat carries magic `0x54534C` (little-endian bytes `4c 53 54`). Send it
periodically for telemetry-only operation. Lease expiry stops transmission and
clears queued samples. RC command freshness has its own 200 ms timer.

PID-loop captures are batched in pairs. The low-priority FSP task drains a
four-sample queue; overflow drops new captures. Lockstep sampling uses
`N = HZ_TO_INTERVAL_US(FSP_PERIOD_HZ) / gyro.targetLooptime + 1` PID iterations.
A 2026 Hz PID loop with a 600 Hz ceiling yields about 506 samples/s.

Timestamps are 32-bit FC microseconds (71.6-minute wrap): send-task time in
the header, PID snapshot time in each sample. Hosts handle clock mapping.

## Packets

Packets use little-endian C layouts, COBS and a `0x00` delimiter. CRC-8 uses
polynomial `0xD5`, seed `0xFF`, over bytes preceding the final CRC. Reserved
fields are zero. Version must match: `0x05`, or `0x85` with MAVLink.

| Packet | Decoded size |
| --- | ---: |
| Heartbeat | 12 bytes |
| RC command | 24 bytes |
| Telemetry: two sensor frames | 116 bytes |
| Telemetry with 64-byte MAVLink tunnel | 180 bytes |

Each 52-byte sensor frame carries timestamp, acceleration, gyro, WXYZ attitude,
RC channels, four RPM values and voltage. Scale acceleration by `1 / acc1G`,
gyro by `gyroDpsLsb`, quaternion by `1 / 32768`, voltage by `0.01 V`.
RC telemetry reflects receiver-selected `rcData`. See `fsp_frame.h` for offsets
and field types; the packet buffer limit is 255 decoded bytes.

## RC override

FSP fills raw receiver channels in **AETR1234** order:
`roll, pitch, throttle, yaw, 0, 0, AUX3, AUX4`. Configure a matching map and
mask only the channels the companion supplies.

| State | Selected primary channels |
| --- | --- |
| Override released | Radio input |
| Override selected, command younger than 200 ms | Companion input |
| Override selected, command absent or expired | Roll/pitch/yaw 1500, throttle 1000 |

Fresh commands resume immediately. Unmasked channels retain radio input;
auxiliary channels return to radio input on timeout. Fallback values follow
`rcmap` and normal receiver-range calibration.

MSP arrivals request receiver processing independently of
`msp_override_failsafe`; that setting controls whether they also renew the RX
signal deadline. The timeout applies to conventional MSP override commands too.

## Other changes from stock Betaflight

Relative to common ancestor `50a3bd593`:

- UART function, PID capture hook, low-priority I/O task and startup integration.
- Independent 80 Hz PT2 telemetry filtering; updates must be within 4 ms.
  Disabled: use existing FC-filtered measurements.
- Accelerometer RPM/notch filters (`acc_rpm_filter_*`, `acc_notch_hz`,
  `acc_notch_cutoff`), parameter group 700, and filtered acceleration for attitude.
- MPU6000 ranges changed to +/-8 g and +/-1000 degrees/s.
- Configurable accelerometer/attitude task rates, replacing mode-dependent
  attitude scheduling. Optional removal of attitude/navigation features.
- MAVLink FC-to-companion tunnel with a 512-byte buffer.
- XR8 Pro layout: RPM/current/voltage/temperature in `rpm[]`, and
  reverse/throttle1/throttle2 in throttle/AUX3/AUX4; host selects by build.
- Legacy `MSP2_PUSH=0x3100`: 1 kHz, paired 45-byte payloads on USART1;
  16-bit MSP push IDs and a complete-packet helper.
- `MSP_SET_RAW_RC` returns `MSP_RESULT_NO_REPLY`.

## Test

```sh
make -C src/test test_rx_msp_override_unittest CC=gcc CXX=g++
```

Covers startup defaults, timeout, recovery, switch release, channel masks,
timer wraparound and receiver channel mapping.

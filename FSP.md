# Flyby Serial Protocol (FSP)

FSP streams FC measurements over UART and accepts scheduled companion actions
(RC commands, rates or direct motor outputs). Packet definitions live in
`src/main/fsp/fsp_frame.h`.

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

The H7 recipe uses the default 1100 Hz FSP rate. The script-specific defaults
in the table below refer to `build_f405_mini.sh`.

Assign serial function `524288` (`FUNCTION_FSP`) to a dedicated UART and select
its MSP baud rate, e.g. `serial UART2 524288 115200 57600 0 115200`. The link
uses non-inverted 8N1.

| Define | Default | Meaning |
| --- | --- | --- |
| `USE_FSP` | Defined | Enable FSP |
| `FSP_PERIOD_HZ` | 1100 Hz; script: 600 Hz | Default for `fsp_telemetry_hz` |
| `FSP_PID_LOCKSTEP` | Defined | Capture every Nth PID iteration |
| `ENABLE_PT2_FILTER_IMU` | Undefined/off; script: 1 | Independent 80 Hz PT2 telemetry filter |
| `FSP_ENABLE_MAVLINK` | Undefined/off | Include the MAVLink tunnel |
| `FSP_MAVLINK_TUNNEL_SIZE` | 64 bytes | Maximum MAVLink bytes per packet |
| `ACC_TASK_PERIOD_HZ` | 1000 Hz | Initial accelerometer/attitude task rate |
| `PERF_DISABLE_ATTITUDE` | Undefined/off | Remove attitude estimation and related navigation features |
| `ESC_XR8_PRO` | Undefined/off | Select car ESC telemetry |
| `USE_MSP_PUSH` | Undefined/off | Enable the older MSPv2 streaming task |
| `USE_ACC_RPM_FILTER` | Defined with DShot telemetry | Enable accelerometer RPM filtering |

`FSP_TARGET_HZ` accepts 300, 600 or 1100. Presence-based options (`#ifdef`)
are disabled by removing their definition, rather than setting them to zero.

CLI settings (parameter group `fspConfig`, applied at boot):

| Setting | Default | Range | Meaning |
| --- | --- | --- | --- |
| `fsp_telemetry_hz` | `FSP_PERIOD_HZ` | 50..4000 | FSP task rate and capture-rate ceiling |
| `fsp_telemetry_timeout_ms` | 500 | 50..5000 | Telemetry lease after the last heartbeat or action |
| `fsp_action_timeout_ms` | 200 | 10..2000 | Neutral outputs this long after the last applied action |
| `fsp_action_max_ahead_ticks` | 128 | 1..4096 | Actions further ahead are rejected (`HORIZON`) |

## Telemetry and timing

A valid heartbeat or action starts telemetry and renews its lease
(`fsp_telemetry_timeout_ms`). Send heartbeats periodically for
telemetry-only operation. Lease expiry stops transmission and clears queued
samples.

PID-loop captures are batched in pairs. The low-priority FSP task drains a
four-sample queue; overflow drops new captures. Lockstep sampling uses
`N = HZ_TO_INTERVAL_US(fsp_telemetry_hz) / targetPidLooptime + 1` PID iterations.
A 2026 Hz PID loop with a 600 Hz ceiling yields about 506 samples/s.

Timestamps are 32-bit FC microseconds (71.6-minute wrap): send time in the
header, PID snapshot time in each sample. PID ticks are 32-bit and count
executed PID iterations; ticks dropped by scheduler overruns are not counted
(see `skippedTicks` in the stats). Hosts handle clock mapping.

## Packets

Each packet is `[header][payload][crc]`, little-endian C layouts, COBS-encoded
with a `0x00` delimiter. The 8-byte header holds version (`0x08`), packet type,
two reserved bytes and the sender timestamp. CRC-8 uses polynomial `0xD5`, seed
`0xFF`, over header and payload. Reserved fields are zero. Packets with a wrong
version, CRC, type or payload size are ignored.

| Type | Direction | Payload | Decoded size |
| ---: | --- | --- | ---: |
| 1 Heartbeat | host → FC | none | 9 bytes |
| 2 State | FC → host | two 68-byte sensor frames | 145 bytes |
| 3 Action | host → FC | `fspPacketAction_t` | 25 bytes |
| 4 MAVLink | FC → host | 1..64 tunnel bytes | 10..73 bytes |
| 5 Action ack | FC → host | `fspPacketActionAck_t` | 21 bytes |
| 6 Stats | FC → host | `fspPacketStats_t` | 29 bytes |

Each sensor frame carries timestamp, PID tick, acceleration, gyro, WXYZ attitude,
RC channels, four RPM values, battery voltage, battery current and the action
state of that tick. Scale acceleration by `1 / acc1G`, gyro by `gyroDpsLsb`,
quaternion by `1 / 32768`, voltage by `0.01 V` and current by `0.01 A`.
Both battery values are the filtered FC battery meter readings, so they stay 0
with the corresponding meter source disabled; current is signed and negative
while charging. RC telemetry reflects receiver-selected `rcData`.
See `fsp_frame.h` for offsets and field types.

The FC answers every action with an ack (apply tick, PID tick at receipt,
result, queue depth) and every heartbeat with a stats packet (rejection
counters, skipped PID ticks, measured PID period in ns). The host controls the
stats rate through its heartbeat rate. Acks and stats are dropped when the UART
TX buffer is full.

## Scheduled actions

An action carries a target PID tick (`applyTick`), a mode and four signed
16-bit values. Rate and motor actions are scheduled on their target tick; RC
actions are applied immediately (see below):

| Mode | Values |
| --- | --- |
| 1 RC | roll, pitch, yaw, throttle in 1000..2000 us |
| 2 Rates | roll/pitch/yaw in 0.1 deg/s (±20000), collective 0..10000 |
| 3 Motors | motor indices 0..3 in 0..10000; zero stops the motor, positive values map to the configured low/high output endpoints |

RC actions work like the RC packet of earlier protocol versions: they are fed
into the MSP override as soon as they are received, `applyTick` is ignored and
they never enter the queue or replace the active scheduled action. Their ack
echoes `applyTick` and reports `OK` or `INVALID` (out-of-range values or nonzero
reserved bytes) only. There is no FSP authority gate; the MSP override switch
and its timeout decide whether the values are used.

The four-entry FIFO for rate and motor actions accepts increasing target ticks, at most
`fsp_action_max_ahead_ticks` ahead. The PID task applies an action only on its
target tick; late, full-queue, out-of-order, too-far-ahead and invalid actions
are rejected with the code in the ack. Targeting `pidTick + N` of a received
state gives a fixed N-iteration state-to-action delay; compare state
timestamps or `skippedTicks` to detect scheduler gaps.

Rates enter before rate-error calculation, with radio feedforward excluded;
collective uses the normal mixer. Direct motor mode bypasses PID/mixing.

Rate and motor actions need the physical MSP override switch, a healthy radio,
arming, Acro, four motors and a non-3D/non-launch/non-crash-flip configuration.

Any authority change flushes pending and active actions. The active action is
held between updates; after `fsp_action_timeout_ms` without a new one, rate and
motor modes output motor-stop values. RC actions fall back through the RC
override timeout below. Disarmed actions rehearse timing while keeping normal
disarmed outputs.

Each sensor frame reports the scheduled (rate/motor) action in effect on its tick: `actionTick`
(target tick, 0 = none), `actionMode`, `queueDepth` and `actionFlags`:
bit 0 rate/motor authority,
bit 1 the action drove the outputs, bit 2 timed out.

Host tests for the allocation-free scheduler:

```sh
cc -std=gnu11 -Wall -Wextra -Werror -Isrc/main \
  src/test/fsp/scheduled_actions_test.c src/main/fsp/fsp_actions.c \
  -o /tmp/fsp-actions-test && /tmp/fsp-actions-test
```

## RC override

RC actions fill raw receiver channels in **AETR** order:
`roll, pitch, throttle, yaw`; all other channels are zero. Configure a matching
map and set `msp_override_channels_mask` to channels 0..3 only, so auxiliary
channels stay with the radio.

| State | Selected primary channels |
| --- | --- |
| Override released | Radio input |
| Override selected, command younger than 200 ms | Companion input |
| Override selected, command absent or expired | Roll/pitch/yaw 1500, throttle 1000 |

Fresh commands resume immediately. Unmasked channels retain radio input.
Fallback values follow `rcmap` and normal receiver-range calibration.

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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#define FSP_VERSION 0x07
#define FSP_SENSOR_FRAME_BATCH_COUNT 2
#define FSP_MAVLINK_TUNNEL_SIZE 64
#define FSP_CRC_POLY 0xD5
#define FSP_QUATERNION_SCALE (1 << (sizeof(int16_t) * 8 - 1))

// Frame layout (COBS-decoded):
// [Header (fspPacketHeader_t)] [Payload (variable)] [CRC over header and payload (1 byte)]
#define FSP_FRAME_HEADER(frame, size) ((size) > sizeof(fspPacketHeader_t) ? (const fspPacketHeader_t *)(frame) : NULL)
#define FSP_FRAME_CRC(frame, size) ((size) > sizeof(fspPacketHeader_t) ? ((const uint8_t *)(frame) + (size) - 1) : NULL)
#define FSP_FRAME_PAYLOAD_LENGTH(size) ((size) > sizeof(fspPacketHeader_t) ? (size) - sizeof(fspPacketHeader_t) - 1 : 0)
// Fixed-size payloads: NULL unless the frame has exactly the size of `type`.
#define FSP_FRAME_PAYLOAD(frame, size, type)                                                                           \
    ((size) == sizeof(fspPacketHeader_t) + sizeof(type) + 1                                                            \
         ? (const type *)((const uint8_t *)(frame) + sizeof(fspPacketHeader_t))                                        \
         : NULL)
// Variable-size payloads (MAVLink): NULL unless 1..max bytes long.
#define FSP_FRAME_PAYLOAD_VAR(frame, size, type, max)                                                                  \
    (FSP_FRAME_PAYLOAD_LENGTH(size) >= 1 && FSP_FRAME_PAYLOAD_LENGTH(size) <= (max)                                    \
         ? (const type *)((const uint8_t *)(frame) + sizeof(fspPacketHeader_t))                                        \
         : NULL)

typedef enum {
    FSP_PACKET_TYPE_HEARTBEAT = 0x01,  // host -> FC, enable telemetry; header only, no payload
    FSP_PACKET_TYPE_STATE = 0x02,      // FC -> host, contains FSP_SENSOR_FRAME_BATCH_COUNT sensor frames
    FSP_PACKET_TYPE_ACTION = 0x03,     // host -> FC, contains one action to be applied
    FSP_PACKET_TYPE_MAVLINK = 0x04,    // FC -> host, contains MAVLink messages tunneled from the FC
    FSP_PACKET_TYPE_ACTION_ACK = 0x05, // FC -> host, acknowledge received action to measure latency
    FSP_PACKET_TYPE_STATS = 0x06,      // FC -> host, statistics
} fspPacketType_e;

typedef enum {
    // values[] = roll, pitch, yaw, throttle as RC commands in 1000..2000 us, applied as MSP override
    FSP_ACTION_MODE_RC = 1,
    // values[0..2] = roll, pitch, yaw rate in 0.1 deg/s (-20000..20000); values[3] = collective 0..10000
    FSP_ACTION_MODE_RATES = 2,
    // values[i] = output of Betaflight motor index i, 0..10000
    FSP_ACTION_MODE_MOTORS = 3,
} fspActionMode_e;

typedef enum {
    FSP_ACTION_FLAG_AUTHORITY = (1U << 0),     // armed, MSP override selected, Acro, radio healthy
    FSP_ACTION_FLAG_OUTPUT_ACTIVE = (1U << 1), // an action drove the outputs on this tick
    FSP_ACTION_FLAG_TIMED_OUT = (1U << 2),     // no new action within the action timeout, outputs neutral
} fspActionFlag_e;

typedef enum {
    FSP_ACTION_OK = 0,
    FSP_ACTION_INVALID = 1,
    FSP_ACTION_LATE = 2,
    FSP_ACTION_FULL = 3,
    FSP_ACTION_ORDER = 4,
    FSP_ACTION_HORIZON = 5,
    FSP_ACTION_AUTHORITY_CHANGED = 6,
} fspActionResult_e;

typedef struct {
    uint8_t version;
    uint8_t packetType;
    uint8_t reserved[2];
    uint32_t timestamp;
} fspPacketHeader_t;

typedef struct {
    int16_t roll, pitch, yaw, throttle;
    int16_t aux3, aux4;
} fspRcData_t;

typedef struct {
    int16_t x, y, z;
} fspVec_t;

typedef struct {
    int16_t w, x, y, z;
} fspAtt_t;

typedef struct {
    uint32_t timestamp;
    uint32_t pidTick;
    fspVec_t acc;
    int16_t acc1G;
    float gyroDpsLsb;
    fspVec_t gyro;
    fspAtt_t attitude;
    fspRcData_t rc;
    uint16_t rpm[4];
    uint16_t batteryVoltage;
    uint32_t actionTick; // applyTick of the action in effect on this tick, 0 = none
    uint8_t actionMode;  // fspActionMode_e of that action, 0 = none
    uint8_t actionFlags; // fspActionFlag_e bits
    uint8_t queueDepth;  // actions still waiting
    uint8_t reserved;
} fspSensorFrame_t;

typedef struct {
    fspSensorFrame_t sensorFrames[FSP_SENSOR_FRAME_BATCH_COUNT];
} fspPacketState_t;

typedef struct {
    uint32_t applyTick; // PID tick on which the action takes effect
    int16_t values[4];  // meaning and range depend on mode, see fspActionMode_e
    uint8_t mode;       // fspActionMode_e
    uint8_t reserved[3];
} fspPacketAction_t;

typedef struct {
    uint8_t data[FSP_MAVLINK_TUNNEL_SIZE]; // variable length, see FSP_FRAME_PAYLOAD_VAR
} fspPacketMavlink_t;

typedef struct {
    uint32_t applyTick; // echoes the action
    uint32_t recvTick;  // PID tick at receipt; applyTick - recvTick is the host's margin
    uint8_t result;     // fspActionResult_e
    uint8_t queueDepth; // after enqueue
    uint8_t reserved[2];
} fspPacketActionAck_t;

typedef struct {
    uint32_t lateCount;     // rejected with FSP_ACTION_LATE
    uint32_t fullCount;     // rejected with FSP_ACTION_FULL
    uint32_t rejectedCount; // all other rejections
    uint32_t skippedTicks;  // PID ticks lost to scheduler overruns (dt > 1.5 * period)
    uint32_t pidPeriodNs;   // measured PID period (EMA of dt), not the nominal looptime
} fspPacketStats_t;

// Wire sizes; host and FC both rely on natural alignment without padding.
static_assert(sizeof(fspPacketHeader_t) == 8, "header wire size");
static_assert(sizeof(fspSensorFrame_t) == 64, "sensor frame wire size");
static_assert(sizeof(fspPacketState_t) == 64 * FSP_SENSOR_FRAME_BATCH_COUNT, "state wire size");
static_assert(sizeof(fspPacketAction_t) == 16, "action wire size");
static_assert(sizeof(fspPacketMavlink_t) == FSP_MAVLINK_TUNNEL_SIZE, "mavlink wire size");
static_assert(sizeof(fspPacketActionAck_t) == 12, "action ack wire size");
static_assert(sizeof(fspPacketStats_t) == 20, "stats wire size");

#ifdef __cplusplus
}
#endif

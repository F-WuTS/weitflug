#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

/*
10000101
^      ^
|      └── version = 5 (example)
└──────── MAVLink enabled/ disabled
 */
#define FSP_VERSION_MASK 0x7F  // 0111 1111 (lower 7 bits)
#define MAVLINK_FLAG_MASK 0x80 // 1000 0000 (MSB)
#define MAKE_FSP_VERSION(version, mavlink_enabled)                                                                     \
    (((version) & FSP_VERSION_MASK) | ((mavlink_enabled) ? MAVLINK_FLAG_MASK : 0))
#define FSP_GET_VERSION(v) ((v) & FSP_VERSION_MASK)
#define FSP_IS_MAVLINK_ENABLED(v) (((v) & MAVLINK_FLAG_MASK) != 0)

// actual FSP version number
#define FSP_RAW_VERSION_NUM 0x06

#ifdef FSP_ENABLE_MAVLINK
#ifndef FSP_MAVLINK_TUNNEL_SIZE
#define FSP_MAVLINK_TUNNEL_SIZE 64
#endif
#define FSP_VERSION MAKE_FSP_VERSION(FSP_RAW_VERSION_NUM, 1)
#else
#define FSP_VERSION MAKE_FSP_VERSION(FSP_RAW_VERSION_NUM, 0)
#endif

#define FSP_SENSOR_FRAME_BATCH_COUNT 2
#define FSP_CRC_POLY 0xD5
#define FSP_QUATERNION_SCALE (1 << (sizeof(int16_t) * 8 - 1))

typedef struct {
    uint8_t version;
    uint32_t reserved : 24;
    uint32_t timestamp;
} fspPacketHeader_t;

typedef struct {
    int16_t roll, pitch, yaw, throttle;
    int16_t aux3, aux4;
} fspRcData_t;

typedef struct {
    int16_t roll, pitch, yaw;
} fspAttData_t;

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
} fspSensorFrame_t;

#ifdef FSP_ENABLE_MAVLINK
typedef struct {
    uint8_t data[FSP_MAVLINK_TUNNEL_SIZE];
} fspMavlinkTunnel_t;
#endif

typedef struct {
    fspPacketHeader_t header;
    fspRcData_t rc;
    uint32_t reserved : 24;
    uint8_t crc;
} fspFcRxPacket_t;

typedef struct {
    fspPacketHeader_t header;
    fspSensorFrame_t sensorFrames[FSP_SENSOR_FRAME_BATCH_COUNT];
#ifdef FSP_ENABLE_MAVLINK
    fspMavlinkTunnel_t mavlink;
#endif
    uint32_t reserved2 : 24;
    uint8_t crc;
} fspFcTxPacket_t;

static_assert((offsetof(fspFcTxPacket_t, header.version) == 0), "version must be at offset 0");
static_assert((offsetof(fspFcTxPacket_t, crc) == sizeof(fspFcTxPacket_t) - 1), "crc must be at the end of the packet");
static_assert((offsetof(fspFcRxPacket_t, crc) == sizeof(fspFcRxPacket_t) - 1), "crc must be at the end of the packet");

#ifdef __cplusplus
}
#endif

// Distinct packet size identifies the telemetry heartbeat.
#define FSP_TELEMETRY_HEARTBEAT_MAGIC 0x54534C
#define FSP_TELEMETRY_TIMEOUT_US 500000U
typedef struct {
    fspPacketHeader_t header;
    uint32_t magic : 24;
    uint8_t crc;
} fspTelemetryHeartbeat_t;
static_assert(sizeof(fspTelemetryHeartbeat_t) == 12, "heartbeat wire size");
static_assert(offsetof(fspTelemetryHeartbeat_t, crc) == 11, "heartbeat CRC offset");

#define FSP_ACTION_RATES 1U
#define FSP_ACTION_MOTORS 2U
#define FSP_ACTION_QUEUE_CAPACITY 4U
#define FSP_ACTION_TIMEOUT_US 200000U
#define FSP_ACTION_MAX_AHEAD_TICKS 128U
#define MSP2_FSP_ACTION 0x3F00
#define MSP2_FSP_ACTION_STATUS 0x3F01

// Rates: roll/pitch/yaw in 0.1 deg/s, collective in 0..10000.
// Motors: BF motor indices 0..3, each in 0..10000.
typedef struct {
    fspPacketHeader_t header;
    uint32_t sequence;
    uint32_t sourceTick;
    uint32_t applyTick;
    int16_t values[4];
    uint8_t mode;
    uint8_t flags;
    uint8_t reserved;
    uint8_t crc;
} fspActionPacket_t;

typedef enum {
    FSP_ACTION_OK = 0, FSP_ACTION_INVALID = 1, FSP_ACTION_LATE = 2,
    FSP_ACTION_FULL = 3, FSP_ACTION_ORDER = 4, FSP_ACTION_HORIZON = 5,
    FSP_ACTION_AUTHORITY_CHANGED = 6,
} fspActionResult_e;

typedef struct {
    fspPacketHeader_t header;
    uint32_t pidTick;
    uint32_t pidPeriodUs;
    uint32_t receivedSequence;
    uint32_t consumedSequence;
    uint32_t requestedTick;
    uint32_t consumedTick;
    uint32_t consumedTimeUs;
    uint32_t motorWriteTimeUs;
    uint32_t lateCount;
    uint32_t fullCount;
    uint32_t rejectedCount;
    int16_t values[4];
    uint16_t motors[4]; // normalized external outputs: 1000..2000
    uint8_t queueDepth;
    uint8_t mode;
    uint8_t result;
    uint8_t authority;
    uint8_t outputActive;
    uint8_t timedOut;
    uint8_t authorityFlags; // armed, override, Acro, radio healthy, supported (bits 0..4)
    uint8_t crc;
} fspActionStatus_t;
static_assert(sizeof(fspActionPacket_t) == 32, "action wire size");
static_assert(offsetof(fspActionPacket_t, crc) == 31, "action CRC offset");
static_assert(sizeof(fspActionStatus_t) == 76, "action status wire size");
static_assert(offsetof(fspActionStatus_t, crc) == 75, "action status CRC offset");

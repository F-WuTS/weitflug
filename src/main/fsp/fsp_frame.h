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
#define FSP_VERSION_MASK   0x7F  // 0111 1111 (lower 7 bits)
#define MAVLINK_FLAG_MASK  0x80  // 1000 0000 (MSB)
#define MAKE_FSP_VERSION(version, mavlink_enabled) ( ((version) & FSP_VERSION_MASK) | ((mavlink_enabled) ? MAVLINK_FLAG_MASK : 0) )
#define FSP_GET_VERSION(v)        ((v) & FSP_VERSION_MASK)
#define FSP_IS_MAVLINK_ENABLED(v) (((v) & MAVLINK_FLAG_MASK) != 0)

// actual FSP version number
#define FSP_RAW_VERSION_NUM 0x05

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
    uint32_t reserved: 24;
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
    uint32_t reserved: 24;
    uint8_t crc;
} fspFcRxPacket_t;

typedef struct {
    fspPacketHeader_t header;
    fspSensorFrame_t sensorFrames[FSP_SENSOR_FRAME_BATCH_COUNT];
#ifdef FSP_ENABLE_MAVLINK
    fspMavlinkTunnel_t mavlink;
#endif
    uint32_t reserved2: 24;
    uint8_t crc;
} fspFcTxPacket_t;

static_assert((offsetof(fspFcTxPacket_t, header.version) == 0), "version must be at offset 0");
static_assert((offsetof(fspFcTxPacket_t, crc) == sizeof(fspFcTxPacket_t) - 1), "crc must be at the end of the packet");
static_assert((offsetof(fspFcRxPacket_t, crc) == sizeof(fspFcRxPacket_t) - 1), "crc must be at the end of the packet");

#ifdef __cplusplus
}
#endif

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

/*
11000101
^      ^
|^     └──Version = 5 (example)
|└─────── Car firmware mode
└──────── MAVLink enabled/ disabled
 */
#define FSP_VERSION_MASK 0x3F       // 0011 1111 (lower 7 bits)
#define MAVLINK_FLAG_MASK 0x80      // 1000 0000 (bit 7)
#define CAR_FIRMWARE_FLAG_MASK 0x40 // 0100 0000 (bit 6)
#define MAKE_FSP_VERSION(version, mavlink_enabled, car_firmware_mode)                                                  \
    (((version) & FSP_VERSION_MASK) | ((mavlink_enabled) ? MAVLINK_FLAG_MASK : 0) |                                    \
     ((car_firmware_mode) ? CAR_FIRMWARE_FLAG_MASK : 0))
#define FSP_GET_VERSION(v) ((v) & FSP_VERSION_MASK)
#define FSP_IS_MAVLINK_ENABLED(v) (((v) & MAVLINK_FLAG_MASK) != 0)
#define FSP_IS_CAR_FIRMWARE_MODE(v) (((v) & CAR_FIRMWARE_FLAG_MASK) != 0)

// actual FSP version number
#define FSP_RAW_VERSION_NUM 0x05

#ifdef USE_THROTTLE_CONTROL
#define FSP_CAR_FIRMWARE_FLAG 1
#else
#define FSP_CAR_FIRMWARE_FLAG 0
#endif

#ifdef FSP_ENABLE_MAVLINK
#ifndef FSP_MAVLINK_TUNNEL_SIZE
#define FSP_MAVLINK_TUNNEL_SIZE 64
#define FSP_MAVLINK_FLAG 1
#endif
#define FSP_VERSION MAKE_FSP_VERSION(FSP_RAW_VERSION_NUM, 1)
#else
#define FSP_MAVLINK_FLAG 0
#endif

#define FSP_VERSION MAKE_FSP_VERSION(FSP_RAW_VERSION_NUM, FSP_MAVLINK_FLAG, FSP_CAR_FIRMWARE_FLAG)

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
#if FSP_CAR_FIRMWARE_FLAG
    int16_t aux1, aux2;
#endif
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
#if FSP_CAR_FIRMWARE_FLAG
    uint32_t erpm;
#else
    uint16_t rpm[4];
#endif
    uint16_t batteryVoltage;
#if FSP_CAR_FIRMWARE_FLAG
    uint16_t esc_current;
    uint16_t esc_temperature;
    uint8_t esc_reverse;
    uint8_t esc_throttle;
    uint16_t esc_battery;
    uint16_t controller_throttle;
#endif
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

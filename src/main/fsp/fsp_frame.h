#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <assert.h>

#define FSP_VERSION 0x03
#define FSP_SENSOR_FRAME_BATCH_COUNT 2
#define FSP_MAVLINK_TUNNEL_SIZE 64
#define FSP_CRC_POLY 0xD5

typedef struct {
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
    int16_t roll, pitch;
    uint16_t yaw;
} fspAtt_t;

typedef struct {
    fspPacketHeader_t header;
    fspVec_t acc;
    int16_t acc1G;
    float gyroDpsLsb;
    fspVec_t gyro;
    fspAtt_t attitude;
    fspRcData_t rc;
    uint16_t rpm[4];
    uint16_t batteryVoltage;
} fspSensorFrame_t;

typedef struct {
    uint8_t data[FSP_MAVLINK_TUNNEL_SIZE];
} fspMavlinkTunnel_t;

typedef struct {
    fspPacketHeader_t header;
    fspRcData_t rc;
    uint8_t crc;
} fspFcRxPacket_t;

typedef struct {
    uint8_t version;
    uint32_t reserved : 24;
    fspSensorFrame_t sensorFrames[FSP_SENSOR_FRAME_BATCH_COUNT];
    fspMavlinkTunnel_t mavlink;
    uint32_t reserved2 : 24;
    uint8_t crc;
} fspFcTxPacket_t;

static_assert((offsetof(fspFcTxPacket_t, version) == 0), "version must be at offset 0");
static_assert((offsetof(fspFcTxPacket_t, crc) == sizeof(fspFcTxPacket_t) - 1), "crc must be at the end of the packet");

#ifdef __cplusplus
}
#endif

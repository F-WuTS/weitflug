#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FSP_VERSION 0x01
#define FSP_SENSOR_FRAME_BATCH_COUNT 2
#define FSP_MAVLINK_TUNNEL_SIZE 64

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
} fspFcRxPacket_t;

typedef struct {
    fspSensorFrame_t sensorFrames[FSP_SENSOR_FRAME_BATCH_COUNT];
    fspMavlinkTunnel_t mavlink;
} fspFcTxPacket_t;

#ifdef __cplusplus
}
#endif

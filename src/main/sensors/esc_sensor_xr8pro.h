#pragma once

#include "sensors/esc_sensor.h"

// Hobbywing XR8 Pro ESC Packet Structure (32 bytes total):
// All multi-byte values are little-endian
typedef struct {
    uint16_t header;          // 0-1: 0x01fe (byte 0 is 0xfe, byte 1 is 0x01)
    uint8_t unknown1[7];      // 2-8: unknown
    uint8_t throttle1;        // 9: throttle in % (uint8)
    uint8_t throttle2;        // 10: throttle in % (uint8)
    uint8_t reverse;          // 11: reverse (00 / 02)
    uint8_t unknown2;         // 12: unknown
    uint16_t rpm;             // 13-14: RPM in units of 10
    uint16_t voltage;         // 15-16: Voltage in 0.1 V
    int16_t current;          // 17-18: Current in 0.1 A
    int16_t escTemperature;   // 19-20: ESC temperature in °C
    int16_t motorTemperature; // 21-22: Motor temperature in °C
    uint8_t unknown3[7];      // 23-29
    uint16_t crc;             // CRC-16/MODBUS of bytes 0-29
} __attribute__((packed)) xr8ProTelemetryFrame_t;

const xr8ProTelemetryFrame_t *escSensorXR8ProFrame(void);

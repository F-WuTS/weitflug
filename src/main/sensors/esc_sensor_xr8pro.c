/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(ESC_XR8_PRO)

#include "common/time.h"
#include "config/feature.h"
#include "drivers/serial.h"
#include "io/serial.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "esc_sensor.h"

PG_REGISTER_WITH_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig, PG_ESC_SENSOR_CONFIG, 0);

PG_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig, .halfDuplex = 0);

#define ESC_SENSOR_BAUDRATE 115200
#define TELEMETRY_FRAME_SIZE 32
#define TELEMETRY_TIMEOUT_US 5000 // 5 ms timeout to reset rx buffer

static uint8_t telemetryBuffers[2][TELEMETRY_FRAME_SIZE] = {0};
static volatile uint8_t *rxBuffer = telemetryBuffers[0];
static volatile size_t rxBufferIdx = 0;

static serialPort_t *escSensorPort = NULL;

static escSensorData_t escSensorData;

static size_t lastRxCount = 0;
static timeUs_t lastRxTimeUs = 0;

escSensorData_t *getEscSensorData(uint8_t motorNumber)
{

    if (!featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return NULL;
    }

    if (motorNumber != 0 && motorNumber != ESC_SENSOR_COMBINED) {
        return NULL;
    }

    return &escSensorData;
}

void startEscDataRead(uint8_t *frameBuffer, uint8_t frameLength)
{
    UNUSED(frameBuffer);
    UNUSED(frameLength);
}

uint8_t getNumberEscBytesRead(void) { return 0; }

uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen)
{
    UNUSED(Buf);
    UNUSED(BufLen);
    return 0;
}

// Receive ISR callback
static void escSensorDataReceive(uint16_t c, void *data)
{
    UNUSED(data);
    if (rxBufferIdx < TELEMETRY_FRAME_SIZE) {
        rxBuffer[rxBufferIdx++] = (uint8_t)c;
    }
}

bool escSensorInit(void)
{
    escSensorData.dataAge = ESC_DATA_INVALID;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESC_SENSOR);
    if (!portConfig) {
        return false;
    }

    portOptions_e options = SERIAL_NOT_INVERTED;

    // Initialize serial port
    escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, escSensorDataReceive, NULL,
                                   ESC_SENSOR_BAUDRATE, MODE_RX, options);

    return escSensorPort != NULL;
}

static uint16_t calculateCrc16Modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            }
            else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static bool decodeEscFrame(const uint8_t *telemetryBuffer)
{
    // # Assignments:
    // - 0-1: header (fe01)
    // - 2-8: unknown
    // - 9: throttle in % (uint8)
    // - 10: throttle in % (uint8)
    // - 11: reverse (00 / 02)
    // - 12: unknown
    // - 13-14: RPM in units of 10 (uint16 little endian)
    // - 15-16: Voltage in 0.1 V (uint16 little endian)
    // - 17-18: Current in 0.1 A (uint16 little endian)
    // - 19-20: ESC temperature in °C (uint16 little endian)
    // - 21-22: Motor temperature in °C (uint16 little endian)
    // - 23-29: Unknown
    // - 30-31: CRC-16/MODBUS of bytes 0-29 (uint16 little endian)

    // PACKET_FORMAT = "<2s 7x BBB x 5H 7x H"
    // header, throttle1, throttle2, reverse, rpm, voltage, current, esc_temp, motor_temp, crc =
    // struct.unpack(PACKET_FORMAT, packet) if header != b'\xFE\x01':
    //     print("Invalid header")
    //     continue

    // Check CRC
    uint16_t receivedCrc = telemetryBuffer[30] | (telemetryBuffer[31] << 8);
    uint16_t calculatedCrc = calculateCrc16Modbus(telemetryBuffer, 30);
    if (receivedCrc != calculatedCrc) {
        return false;
    }

    // Check header
    if (telemetryBuffer[0] != 0xFE || telemetryBuffer[1] != 0x01) {
        return false;
    }

    escSensorData.dataAge = 0;
    escSensorData.temperature = telemetryBuffer[19] | (telemetryBuffer[20] << 8);
    escSensorData.voltage = (telemetryBuffer[15] | (telemetryBuffer[16] << 8)) * 10;
    escSensorData.current = (telemetryBuffer[17] | (telemetryBuffer[18] << 8)) * 10;
    escSensorData.rpm = (telemetryBuffer[13] | (telemetryBuffer[14] << 8)) * 10;

    return true;
}

static void increaseDataAge(void)
{
    if (escSensorData.dataAge < ESC_DATA_INVALID) {
        escSensorData.dataAge++;
    }
}

void escSensorProcess(timeUs_t currentTimeUs)
{
    if (!escSensorPort) {
        return;
    }

    if (rxBufferIdx != lastRxCount) {
        lastRxCount = rxBufferIdx;
        lastRxTimeUs = currentTimeUs;
    }

    if (rxBufferIdx == TELEMETRY_FRAME_SIZE) {
        // Swap buffers
        const uint8_t *completedBuffer = (const uint8_t *)rxBuffer;
        rxBuffer = (rxBuffer == telemetryBuffers[0]) ? telemetryBuffers[1] : telemetryBuffers[0];
        rxBufferIdx = 0;
        if (!decodeEscFrame(completedBuffer)) {
            increaseDataAge();
        }
    }
    else if (cmpTimeUs(currentTimeUs, lastRxTimeUs) >= TELEMETRY_TIMEOUT_US) {
        // No new data received for a while, reset buffer index to start fresh
        if (rxBufferIdx > 0) {
            increaseDataAge();
            rxBufferIdx = 0;
        }
    }
}

#endif

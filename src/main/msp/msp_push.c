#include <stdint.h>

#include "platform.h"

#include "drivers/accgyro/accgyro.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "msp/msp.h"
#include "msp/msp_serial.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include "msp/msp_push.h"

/*
 * Instead of request/response, we use a push model for certain MSP commands.
 * Sends data to the host without waiting for a request.
 *
 * Called periodically by the scheduler.
 */
void taskHandleMspPush(timeUs_t currentTimeUs)
{
    const uint32_t currentTimeMs = currentTimeUs;
    int16_t accX = lrintf(acc.accADC[0]);
    int16_t accY = lrintf(acc.accADC[1]);
    int16_t accZ = lrintf(acc.accADC[2]);
    int16_t gyroX = lrintf(gyro.gyroADCf[0]); // filtered gyro data
    int16_t gyroY = lrintf(gyro.gyroADCf[1]);
    int16_t gyroZ = lrintf(gyro.gyroADCf[2]);

    int16_t cmd = MSP2_PUSH;
    uint8_t data[] = {
        (uint8_t)(currentTimeMs & 0xFF),
        (uint8_t)((currentTimeMs >> 8) & 0xFF),
        (uint8_t)((currentTimeMs >> 16) & 0xFF),
        (uint8_t)((currentTimeMs >> 24) & 0xFF),
        (uint8_t)(accX & 0xFF),
        (uint8_t)((accX >> 8) & 0xFF),
        (uint8_t)(accY & 0xFF),
        (uint8_t)((accY >> 8) & 0xFF),
        (uint8_t)(accZ & 0xFF),
        (uint8_t)((accZ >> 8) & 0xFF),
        (uint8_t)(gyroX & 0xFF),
        (uint8_t)((gyroX >> 8) & 0xFF),
        (uint8_t)(gyroY & 0xFF),
        (uint8_t)((gyroY >> 8) & 0xFF),
        (uint8_t)(gyroZ & 0xFF),
        (uint8_t)((gyroZ >> 8) & 0xFF),
    };
    int len = sizeof(data);

    mspSerialPush(SERIAL_PORT_USART3, cmd, data, len, MSP_DIRECTION_REPLY, MSP_V2_NATIVE);
}

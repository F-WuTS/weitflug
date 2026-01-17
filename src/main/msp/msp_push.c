#include <stdint.h>
#include <math.h>

#include "common/streambuf.h"

#include "sensors/acceleration.h"
#include "sensors/gyro_init.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "drivers/dshot.h"

#include "flight/imu.h"
#include "flight/position.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "rx/rx.h"

#include "msp/msp.h"
#include "msp/msp_serial.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include "msp/msp_push.h"

#ifdef USE_MULTI_GYRO
#define ACTIVE_GYRO ((gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_2) ? &gyro.gyroSensor2 : &gyro.gyroSensor1)
#else
#define ACTIVE_GYRO (&gyro.gyroSensor1)
#endif

#define MSP_PUSH_FRAME_SIZE 43
#define MSP_PUSH_BATCH_SIZE 2

/*
 * Instead of request/response, we use a push model for certain MSP commands.
 * Sends data to the host without waiting for a request.
 *
 * Called periodically by the scheduler.
 */
void taskHandleMspPush(timeUs_t currentTimeUs)
{
    static uint8_t frameBuffer[MSP_PUSH_FRAME_SIZE * MSP_PUSH_BATCH_SIZE];
    static uint8_t* framePtr = frameBuffer;

    // Calculate the attitude in 0.001 degree units. 180 deg = 18000
    int16_t roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (18000.0f / M_PIf));
    int16_t pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (18000.0f / M_PIf));
    int16_t yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (18000.0f / M_PIf)));

    if (yaw < 0) {
        yaw += 36000;
    }

    // currentTimeUs (4 bytes)
    framePtr[0] = (uint8_t)(currentTimeUs & 0xFF);
    framePtr[1] = (uint8_t)((currentTimeUs >> 8) & 0xFF);
    framePtr[2] = (uint8_t)((currentTimeUs >> 16) & 0xFF);
    framePtr[3] = (uint8_t)((currentTimeUs >> 24) & 0xFF);

    // acc data (6 bytes)
    int16_t accX = lrintf(acc.accADCf[0]);
    framePtr[4] = (uint8_t)(accX & 0xFF);
    framePtr[5] = (uint8_t)((accX >> 8) & 0xFF);
    int16_t accY = lrintf(acc.accADCf[1]);
    framePtr[6] = (uint8_t)(accY & 0xFF);
    framePtr[7] = (uint8_t)((accY >> 8) & 0xFF);
    int16_t accZ = lrintf(acc.accADCf[2]);
    framePtr[8] = (uint8_t)(accZ & 0xFF);
    framePtr[9] = (uint8_t)((accZ >> 8) & 0xFF);

    // gyro data (6 bytes)
    int16_t gyroX = gyroRateDps(0);
    framePtr[10] = (uint8_t)(gyroX & 0xFF);
    framePtr[11] = (uint8_t)((gyroX >> 8) & 0xFF);
    int16_t gyroY = gyroRateDps(1);
    framePtr[12] = (uint8_t)(gyroY & 0xFF);
    framePtr[13] = (uint8_t)((gyroY >> 8) & 0xFF);
    int16_t gyroZ = gyroRateDps(2);
    framePtr[14] = (uint8_t)(gyroZ & 0xFF);
    framePtr[15] = (uint8_t)((gyroZ >> 8) & 0xFF);

    // attitude data (6 bytes)
    framePtr[16] = (uint8_t)(roll & 0xFF);
    framePtr[17] = (uint8_t)((roll >> 8) & 0xFF);
    framePtr[18] = (uint8_t)(pitch & 0xFF);
    framePtr[19] = (uint8_t)((pitch >> 8) & 0xFF);
    framePtr[20] = (uint8_t)(yaw & 0xFF);
    framePtr[21] = (uint8_t)((yaw >> 8) & 0xFF);

    // RC data (8 bytes)
    int16_t rcRoll = lrintf(rcData[ROLL]);
    framePtr[22] = (uint8_t)(rcRoll & 0xFF);
    framePtr[23] = (uint8_t)((rcRoll >> 8) & 0xFF);
    int16_t rcPitch = lrintf(rcData[PITCH]);
    framePtr[24] = (uint8_t)(rcPitch & 0xFF);
    framePtr[25] = (uint8_t)((rcPitch >> 8) & 0xFF);
    int16_t rcYaw = lrintf(rcData[YAW]);
    framePtr[26] = (uint8_t)(rcYaw & 0xFF);
    framePtr[27] = (uint8_t)((rcYaw >> 8) & 0xFF);
    int16_t rcThrottle = lrintf(rcData[THROTTLE]);
    framePtr[28] = (uint8_t)(rcThrottle & 0xFF);
    framePtr[29] = (uint8_t)((rcThrottle >> 8) & 0xFF);

    // RC Aux3 data (2 bytes)
    int16_t rcAux3 = lrintf(rcData[AUX3]);
    framePtr[30] = (uint8_t)(rcAux3 & 0xFF);
    framePtr[31] = (uint8_t)((rcAux3 >> 8) & 0xFF);

    // DShot RPM data (8 bytes)
    uint16_t rpm0 = getDshotRpm(0);
    framePtr[32] = (uint8_t)(rpm0 & 0xFF);
    framePtr[33] = (uint8_t)((rpm0 >> 8) & 0xFF);
    uint16_t rpm1 = getDshotRpm(1);
    framePtr[34] = (uint8_t)(rpm1 & 0xFF);
    framePtr[35] = (uint8_t)((rpm1 >> 8) & 0xFF);
    uint16_t rpm2 = getDshotRpm(2);
    framePtr[36] = (uint8_t)(rpm2 & 0xFF);
    framePtr[37] = (uint8_t)((rpm2 >> 8) & 0xFF);
    uint16_t rpm3 = getDshotRpm(3);
    framePtr[38] = (uint8_t)(rpm3 & 0xFF);
    framePtr[39] = (uint8_t)((rpm3 >> 8) & 0xFF);

    // Battery voltage (2 bytes)
    uint16_t voltage = getBatteryVoltage();
    framePtr[40] = (uint8_t)(voltage & 0xFF);
    framePtr[41] = (uint8_t)((voltage >> 8) & 0xFF);

    uint8_t is_frame_end = framePtr == (frameBuffer + ((MSP_PUSH_BATCH_SIZE - 1) * MSP_PUSH_FRAME_SIZE)) ? 1 : 0;
    framePtr[42] = (uint8_t)(is_frame_end & 0xFF);

    framePtr += MSP_PUSH_FRAME_SIZE;

    if (framePtr >= frameBuffer + (MSP_PUSH_FRAME_SIZE * MSP_PUSH_BATCH_SIZE)) {
        // When we've filled the batch, send it

        for (size_t i = 0; i < MSP_PUSH_BATCH_SIZE; i++) {
            mspSerialPush(
                SERIAL_PORT_USART3,
                MSP2_PUSH,
                &frameBuffer[i * MSP_PUSH_FRAME_SIZE],
                MSP_PUSH_FRAME_SIZE,
                MSP_DIRECTION_REPLY,
                MSP_V2_NATIVE
            );
        }
        framePtr = frameBuffer;
    }
}

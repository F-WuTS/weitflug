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

/*
 * Instead of request/response, we use a push model for certain MSP commands.
 * Sends data to the host without waiting for a request.
 *
 * Called periodically by the scheduler.
 */
void taskHandleMspPush(timeUs_t currentTimeUs)
{
    static uint8_t mspSerialOutBuf[MSP_PORT_OUTBUF_SIZE];

    mspPacket_t reply = {
        .buf = { .ptr = mspSerialOutBuf, .end = ARRAYEND(mspSerialOutBuf), },
        .cmd = -1,
        .flags = 0,
        .result = 0,
        .direction = MSP_DIRECTION_REPLY,
    };
    uint8_t *outBufHead = reply.buf.ptr;

    // Calculate the attitude in 0.001 degree units. 180 deg = 18000
    int16_t roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (18000.0f / M_PIf));
    int16_t pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (18000.0f / M_PIf));
    int16_t yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (18000.0f / M_PIf)));

    if (yaw < 0) {
        yaw += 36000;
    }

    // Timestamp (used for latency measurements)
    sbufWriteU32(&reply.buf, currentTimeUs);

    reply.cmd = MSP2_PUSH_FAST;
    sbufWriteU16(&reply.buf, lrintf(acc.accADCf[0]));
    sbufWriteU16(&reply.buf, lrintf(acc.accADCf[1]));
    sbufWriteU16(&reply.buf, lrintf(acc.accADCf[2]));

    sbufWriteU16(&reply.buf, gyroRateDps(0));
    sbufWriteU16(&reply.buf, gyroRateDps(1));
    sbufWriteU16(&reply.buf, gyroRateDps(2));

    sbufWriteU16(&reply.buf, roll);
    sbufWriteU16(&reply.buf, pitch);
    sbufWriteU16(&reply.buf, yaw);

    sbufWriteU16(&reply.buf, rcData[ROLL]);
    sbufWriteU16(&reply.buf, rcData[PITCH]);
    sbufWriteU16(&reply.buf, rcData[YAW]);
    sbufWriteU16(&reply.buf, rcData[THROTTLE]);

    sbufWriteU16(&reply.buf, getDshotRpm(0));
    sbufWriteU16(&reply.buf, getDshotRpm(1));
    sbufWriteU16(&reply.buf, getDshotRpm(2));
    sbufWriteU16(&reply.buf, getDshotRpm(3));

    sbufWriteU16(&reply.buf, getBatteryVoltage());

    sbufSwitchToReader(&reply.buf, outBufHead);
    mspSerialPush2(SERIAL_PORT_USART3, &reply, MSP_V2_NATIVE);
}

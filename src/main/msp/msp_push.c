#include <stdint.h>
#include <math.h>

#include "common/streambuf.h"

#include "sensors/acceleration.h"
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

    // This function is executed with 360Hz
    // 60 and 120Hz data also includes the 360Hz data

    static int counter = 0;
    counter = (counter + 1) % 8;

    // Timestamp (used for latency measurements)
    sbufWriteU32(&reply.buf, currentTimeUs);

    // 480Hz data
    reply.cmd = MSP2_PUSH_480;
    sbufWriteU16(&reply.buf, lrintf(acc.accADC[0]));
    sbufWriteU16(&reply.buf, lrintf(acc.accADC[1]));
    sbufWriteU16(&reply.buf, lrintf(acc.accADC[2]));

    sbufWriteU16(&reply.buf, lrintf(gyro.gyroADCf[0] / ACTIVE_GYRO->gyroDev.scale));
    sbufWriteU16(&reply.buf, lrintf(gyro.gyroADCf[1] / ACTIVE_GYRO->gyroDev.scale));
    sbufWriteU16(&reply.buf, lrintf(gyro.gyroADCf[2] / ACTIVE_GYRO->gyroDev.scale));

    // 120Hz data
    if (counter == 1 || counter == 5) {
        reply.cmd = MSP2_PUSH_120;

        sbufWriteU16(&reply.buf, attitude.values.roll);
        sbufWriteU16(&reply.buf, attitude.values.pitch);
        sbufWriteU16(&reply.buf, attitude.values.yaw);

        // See rc_controls.h for the definition of rcData
        sbufWriteU16(&reply.buf, rcData[ROLL]);
        sbufWriteU16(&reply.buf, rcData[PITCH]);
        sbufWriteU16(&reply.buf, rcData[YAW]);
        sbufWriteU16(&reply.buf, rcData[THROTTLE]);
        sbufWriteU16(&reply.buf, rcData[AUX1]);
        sbufWriteU16(&reply.buf, rcData[AUX2]);
    }

    // 60Hz data
    if (counter == 0) {
        reply.cmd = MSP2_PUSH_60;

        sbufWriteU32(&reply.buf, getEstimatedAltitudeCm());
        sbufWriteU16(&reply.buf, getEstimatedVario());

        sbufWriteU16(&reply.buf, getBatteryVoltage());
        sbufWriteU16(&reply.buf, (int16_t)constrain(getAmperage(), -0x8000, 0x7FFF));
    }

    sbufSwitchToReader(&reply.buf, outBufHead);
    mspSerialPush2(SERIAL_PORT_USART3, &reply, MSP_V2_NATIVE);
}

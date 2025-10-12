#include <stdint.h>
#include <math.h>

#include "fsp/fsp.h"

#include "common/streambuf.h"

#include "drivers/serial.h"

#include "io/serial.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "drivers/dshot.h"

#include "flight/imu.h"
#include "flight/position.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "rx/rx.h"
#include "rx/msp.h"

#ifdef USE_MULTI_GYRO
#define ACTIVE_GYRO ((gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_2) ? &gyro.gyroSensor2 : &gyro.gyroSensor1)
#else
#define ACTIVE_GYRO (&gyro.gyroSensor1)
#endif

static serialPort_t *fspPort = NULL;

void taskFspTx(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // uint32_t current_time_us = currentTimeUs;

    // uint16_t acc_x = lrintf(acc.accADC[0]);
    // uint16_t acc_y = lrintf(acc.accADC[1]);
    // uint16_t acc_z = lrintf(acc.accADC[2]);

    // uint16_t att_roll = attitude.values.roll;
    // uint16_t att_pitch = attitude.values.pitch;
    // uint16_t att_yaw = attitude.values.yaw;

    // uint16_t battery = getBatteryVoltage();
    // // Scale battery voltage (mV) from 21000-24000mV range to 0-255
    // uint8_t battery_scaled =
    //     (battery <= 2100)
    //         ? 0
    //         : ((battery >= 2600) ? 255
    //                              : (uint8_t)(((battery - 2100) * 255) / 500));

    // uint8_t frame[] = {
    //     0xFF, // Frame start marker
    //     current_time_us & 0xFF,
    //     (current_time_us >> 8) & 0xFF,
    //     (current_time_us >> 16) & 0xFF,
    //     (current_time_us >> 24) & 0xFF,
    //     acc_x & 0xFF,
    //     (acc_x >> 8) & 0xFF,
    //     acc_y & 0xFF,
    //     (acc_y >> 8) & 0xFF,
    //     acc_z & 0xFF,
    //     (acc_z >> 8) & 0xFF,
    //     att_roll & 0xFF,
    //     (att_roll >> 8) & 0xFF,
    //     att_pitch & 0xFF,
    //     (att_pitch >> 8) & 0xFF,
    //     att_yaw & 0xFF,
    //     (att_yaw >> 8) & 0xFF,
    //     battery_scaled & 0xFF,
    //     0x00, // TODO: Placeholder for Checksum / CRC
    //     0xFE // Frame end marker
    // };

    uint32_t debug = 0;

    if (!isSerialTransmitBufferEmpty(fspPort) && ((int)serialTxBytesFree(fspPort) < 20)) {
        debug = 1; // TX buffer full
    }



    // Testing Frame to test throughput, drops and bit flips.
    uint8_t frame[] = {
        0xFF, // Frame start marker
        debug & 0xFF,
        (debug >> 8) & 0xFF,
        (debug >> 16) & 0xFF,
        (debug >> 24) & 0xFF,
        0xA0,
        0xA1,
        0xA2,
        0xA3,
        0xA4,
        0xA5,
        0xA6,
        0xA7,
        0xA8,
        0xA9,
        0xAA,
        0xAB,
        0xAC,
        0x00, // TODO: Placeholder for Checksum / CRC
        0xFE // Frame end marker
    };

    serialBeginWrite(fspPort);
    serialWriteBufNoFlush(fspPort, frame, sizeof(frame));
    serialEndWrite(fspPort);
}

void taskFspRx(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    if (serialRxBytesWaiting(fspPort)) {
        uint8_t recv_buffer[16];
        uint8_t recv_buffer_i = 0;

        while (serialRxBytesWaiting(fspPort) && recv_buffer_i < 16) {
            const uint8_t byte = serialRead(fspPort);
            recv_buffer[recv_buffer_i] = byte;

            recv_buffer_i++;
        }

        if (recv_buffer_i == 8) {
            rxMspFrameReceive((uint16_t*) recv_buffer, 4);
        }
    }
}

/**
 * Attempt to open the out device. Returns true if successful.
 */
bool fspSerialOpen(void)
{
    const serialPortConfig_t *portConfig = serialFindPortConfiguration(SERIAL_PORT_USART3);

    portOptions_e portOptions = SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1 | SERIAL_UNIDIR;

    if (!portConfig) {
        return false;
    }

    fspPort = openSerialPort(portConfig->identifier, FUNCTION_FSP, NULL, NULL, 1000000,
        MODE_RXTX, portOptions);

    return fspPort != NULL;
}

void fspInit(void)
{
    fspSerialOpen();
}

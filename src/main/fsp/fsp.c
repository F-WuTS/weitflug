#include <math.h>
#include <stdint.h>
#include <string.h>

#include "fsp/fsp.h"
#include "fsp/fsp_cobs.h"
#include "fsp/fsp_frame.h"

#include "common/maths.h"
#include "common/utils.h"
#include "drivers/dshot.h"
#include "drivers/serial.h"
#include "fc/rc_controls.h"
#include "flight/imu.h"
#include "io/serial.h"
#include "msp/msp.h"
#include "rx/rx.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro_init.h"

#define FSP_MAX_PACKET_SIZE 255

typedef struct {
    struct serialPort_s *port;
    mspDescriptor_t descriptor;
    fspCobsDecoder_t cobsDecoder;
    // Buffers are aligned to 4 bytes to allow reinterpreting as fspFcRxPacket_t/fspFcTxPacket_t without copying
    uint8_t inBuf[FSP_MAX_PACKET_SIZE + 2] __attribute__((aligned(4)));  // +2 for COBS overhead
    uint8_t outBuf[FSP_MAX_PACKET_SIZE + 2] __attribute__((aligned(4))); // +2 for COBS overhead
    fspFcTxPacket_t txPacket;
    size_t batchIndex;
} fspState_t;

static fspState_t fspState;

// Forward declare from rx/msp.h, because of a missing typedef in the header
void rxMspFrameReceive(const uint16_t *frame, int channelCount);

void fspInit(void)
{
    memset(&fspState, 0, sizeof(fspState_t));

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_FSP);
    if (portConfig) {
        portOptions_e options = SERIAL_NOT_INVERTED;
        if (serialType(portConfig->identifier) == SERIALTYPE_UART ||
            serialType(portConfig->identifier) == SERIALTYPE_LPUART ||
            serialType(portConfig->identifier) == SERIALTYPE_PIOUART) {
            // TODO: SERIAL_CHECK_TX is broken on F7, disable it until it is fixed
#if !defined(STM32F7) || defined(USE_F7_CHECK_TX)
            options |= SERIAL_CHECK_TX;
#endif
        }

        serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, NULL,
                                                  baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, options);
        if (serialPort) {
            fspState.port = serialPort;
            fspState.descriptor = mspDescriptorAlloc();
            fspCobsDecoderInit(&fspState.cobsDecoder, fspState.inBuf, sizeof(fspState.inBuf));
        }
    }
}

static void fspFillFrame(fspFcSensorFrame_t *frame, timeUs_t currentTimeUs)
{
    frame->header.timestamp = currentTimeUs;
    frame->acc.x = lrintf(acc.accADCf.x);
    frame->acc.y = lrintf(acc.accADCf.y);
    frame->acc.z = lrintf(acc.accADCf.z);
    frame->gyro.x = gyroRateDps(0);
    frame->gyro.y = gyroRateDps(1);
    frame->gyro.z = gyroRateDps(2);
    frame->attitude.roll = lrintf(atan2_approx(rMat.m[2][1], rMat.m[2][2]) * (18000.0f / M_PIf));
    frame->attitude.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat.m[2][0])) * (18000.0f / M_PIf));
    long yaw = lrintf((-atan2_approx(rMat.m[1][0], rMat.m[0][0]) * (18000.0f / M_PIf)));
    if (yaw < 0) {
        yaw += 36000;
    }
    frame->attitude.yaw = yaw;
    frame->rc.roll = lrintf(rcData[ROLL]);
    frame->rc.pitch = lrintf(rcData[PITCH]);
    frame->rc.yaw = lrintf(rcData[YAW]);
    frame->rc.throttle = lrintf(rcData[THROTTLE]);
    frame->rc.aux3 = lrintf(rcData[AUX3]);
    frame->rc.aux4 = lrintf(rcData[AUX4]);
    for (int i = 0; i < 4; i++) {
        frame->rpm[i] = getDshotRpm(0);
    }
    frame->batteryVoltage = getBatteryVoltage();
}

void fspUpdate(timeUs_t currentTimeUs)
{
    if (!fspState.port) {
        return;
    }

    fspFillFrame(&fspState.txPacket.sensorFrames[fspState.batchIndex++], currentTimeUs);
    if (fspState.batchIndex >= FSP_SENSOR_FRAME_BATCH_COUNT) {
        size_t encodedLength;
        if (fspCobsEncode((uint8_t *)&fspState.txPacket, sizeof(fspState.txPacket), fspState.outBuf,
                          sizeof(fspState.outBuf), &encodedLength)) {
            serialWriteBuf(fspState.port, fspState.outBuf, encodedLength);
        }
        fspState.batchIndex = 0;
    }

    while (serialRxBytesWaiting(fspState.port)) {
        const uint8_t c = serialRead(fspState.port);
        size_t decodedLength;
        switch (fspCobsDecoderPush(&fspState.cobsDecoder, c, &decodedLength)) {
        case FSP_COBS_DECODER_DONE:
            if (decodedLength == sizeof(fspFcRxPacket_t)) {
                fspFcRxPacket_t *rxPacket = (fspFcRxPacket_t *)fspState.inBuf;
                uint16_t frame[] = {
                    [ROLL] = rxPacket->rc.roll,         [PITCH] = rxPacket->rc.pitch, [YAW] = rxPacket->rc.yaw,
                    [THROTTLE] = rxPacket->rc.throttle, [AUX3] = rxPacket->rc.aux3,   [AUX4] = rxPacket->rc.aux4,
                };
                rxMspFrameReceive(frame, ARRAYLEN(frame));
            }
            break;
        case FSP_COBS_DECODER_ERROR_INVALID_INPUT:
            break;
        case FSP_COBS_DECODER_ERROR_BUFFER_OVERFLOW:
            break;
        case FSP_COBS_DECODER_IN_PROGRESS:
            break;
        }
    }
}

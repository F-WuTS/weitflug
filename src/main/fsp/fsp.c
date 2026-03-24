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
#define FSP_MAVLINK_BUFFER_SIZE 512

typedef struct {
    struct serialPort_s *port;
    mspDescriptor_t descriptor;
    fspCobsDecoder_t cobsDecoder;
    // Buffers are aligned to 4 bytes to allow reinterpreting as fspFcRxPacket_t/fspFcTxPacket_t without copying
    uint8_t inBuf[FSP_MAX_PACKET_SIZE + 2] __attribute__((aligned(4)));  // +2 for COBS overhead
    uint8_t outBuf[FSP_MAX_PACKET_SIZE + 2] __attribute__((aligned(4))); // +2 for COBS overhead
    fspFcTxPacket_t txPacket;
    size_t batchIndex;
    uint8_t mavlinkPacketBuffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t mavlinkBuffer[FSP_MAVLINK_BUFFER_SIZE];
    size_t mavlinkBufferHead, mavlinkBufferTail;
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

static void fspFillFrame(fspSensorFrame_t *frame, timeUs_t currentTimeUs)
{
    frame->header.timestamp = currentTimeUs;
    frame->acc.x = lrintf(acc.accADCf.x);
    frame->acc.y = lrintf(acc.accADCf.y);
    frame->acc.z = lrintf(acc.accADCf.z);
    frame->acc1G = acc.dev.acc_1G;
    frame->gyro.x = gyroRateDps(0);
    frame->gyro.y = gyroRateDps(1);
    frame->gyro.z = gyroRateDps(2);
    frame->gyroDpsLsb = gyro.scale;
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

static bool fspMavlinkBufferWrite(const uint8_t *data, size_t len)
{
    if (fspState.mavlinkBufferHead < fspState.mavlinkBufferTail) {
        size_t available = fspState.mavlinkBufferTail - fspState.mavlinkBufferHead;
        if (len > available) {
            return false;
        }
        memcpy(fspState.mavlinkBuffer + fspState.mavlinkBufferHead, data, len);
        fspState.mavlinkBufferHead += len;
    }
    else {
        size_t available = sizeof(fspState.mavlinkBuffer) - (fspState.mavlinkBufferHead - fspState.mavlinkBufferTail);
        if (len > available) {
            return false;
        }
        size_t headToEnd = sizeof(fspState.mavlinkBuffer) - fspState.mavlinkBufferHead;
        if (len <= headToEnd) {
            memcpy(fspState.mavlinkBuffer + fspState.mavlinkBufferHead, data, len);
            fspState.mavlinkBufferHead += len;
        }
        else {
            memcpy(fspState.mavlinkBuffer + fspState.mavlinkBufferHead, data, headToEnd);
            memcpy(fspState.mavlinkBuffer, data + headToEnd, len - headToEnd);
            fspState.mavlinkBufferHead = len - headToEnd;
        }
    }
    return true;
}

static size_t fspMavlinkBufferRead(uint8_t *data, size_t len)
{
    if (fspState.mavlinkBufferTail <= fspState.mavlinkBufferHead) {
        size_t available = fspState.mavlinkBufferHead - fspState.mavlinkBufferTail;
        if (len > available) {
            len = available;
        }
        memcpy(data, fspState.mavlinkBuffer + fspState.mavlinkBufferTail, len);
        fspState.mavlinkBufferTail += len;
    }
    else {
        size_t available = sizeof(fspState.mavlinkBuffer) - (fspState.mavlinkBufferTail - fspState.mavlinkBufferHead);
        if (len > available) {
            len = available;
        }
        size_t tailToEnd = sizeof(fspState.mavlinkBuffer) - fspState.mavlinkBufferTail;
        if (len <= tailToEnd) {
            memcpy(data, fspState.mavlinkBuffer + fspState.mavlinkBufferTail, len);
            fspState.mavlinkBufferTail += len;
        }
        else {
            memcpy(data, fspState.mavlinkBuffer + fspState.mavlinkBufferTail, tailToEnd);
            memcpy(data + tailToEnd, fspState.mavlinkBuffer, len - tailToEnd);
            fspState.mavlinkBufferTail = len - tailToEnd;
        }
    }
    return len;
}

void fspUpdate(timeUs_t currentTimeUs)
{
    if (!fspState.port) {
        return;
    }

    fspState.txPacket.version = FSP_VERSION;
    fspFillFrame(&fspState.txPacket.sensorFrames[fspState.batchIndex++], currentTimeUs);

    // Tunnel recorded MAVLink messages from buffer
    size_t mavlinkLen = fspMavlinkBufferRead(fspState.txPacket.mavlink.data, sizeof(fspState.txPacket.mavlink.data));
    memset(fspState.txPacket.mavlink.data + mavlinkLen, 0, sizeof(fspState.txPacket.mavlink.data) - mavlinkLen);

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

void fspHandleMavlinkMessage(const mavlink_message_t *msg, const mavlink_status_t *status)
{
    (void)status;
    uint16_t length = mavlink_msg_to_send_buffer(fspState.mavlinkPacketBuffer, msg);
    fspMavlinkBufferWrite(fspState.mavlinkPacketBuffer, length);
}

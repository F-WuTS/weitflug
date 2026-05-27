#include <stdint.h>
#include <string.h>

#include "fsp/fsp.h"
#include "flight/throttle_control.h"
#include "fsp/fsp_cobs.h"
#include "fsp/fsp_frame.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/time.h"
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
#include "sensors/esc_sensor_xr8pro.h"
#include "sensors/gyro_init.h"

#include "platform.h"

#define FSP_MAX_PACKET_SIZE 255
#define FSP_SENSOR_FRAME_QUEUE_SIZE (FSP_SENSOR_FRAME_BATCH_COUNT + 3)

#ifdef FSP_ENABLE_MAVLINK
#define FSP_MAVLINK_BUFFER_SIZE 512
#endif

typedef struct {
    struct serialPort_s *port;
    mspDescriptor_t descriptor;
    fspCobsDecoder_t cobsDecoder;
    // Buffers are aligned to 4 bytes to allow reinterpreting as fspFcRxPacket_t/fspFcTxPacket_t without copying
    uint8_t inBuf[FSP_MAX_PACKET_SIZE + 2] __attribute__((aligned(4)));  // +2 for COBS overhead
    uint8_t outBuf[FSP_MAX_PACKET_SIZE + 2] __attribute__((aligned(4))); // +2 for COBS overhead
#ifdef FSP_PID_LOCKSTEP
    size_t captureCounter;
    size_t captureInterval;
#else
    timeUs_t nextCaptureTimeUs;
    bool nextCaptureTimeValid;
#endif
    fspSensorFrame_t sensorFrameQueue[FSP_SENSOR_FRAME_QUEUE_SIZE];
    size_t sensorFrameQueueHead, sensorFrameQueueTail;
#ifdef FSP_ENABLE_MAVLINK
    uint8_t mavlinkPacketBuffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t mavlinkBuffer[FSP_MAVLINK_BUFFER_SIZE];
    size_t mavlinkBufferHead, mavlinkBufferTail;
#endif
    float escToRpmScale;
} fspState_t;

static fspState_t fspState;

// Forward declare from rx/msp.h, because of a missing typedef in the header
void rxMspFrameReceive(const uint16_t *frame, int channelCount);

void fspInit(void)
{
    memset(&fspState, 0, sizeof(fspState_t));

#ifdef FSP_PID_LOCKSTEP
    // Capture sample at every Nth PID loop iteration, N is chosen to capture the sample at
    // the largest possible frequency <= FSP_PERIOD_HZ so the queue is certain to be consumed
    // fast enough regardless of batching.
    fspState.captureInterval = HZ_TO_INTERVAL_US(FSP_PERIOD_HZ) / gyro.targetLooptime + 1;
#endif

    fspState.escToRpmScale = 10.0f / (motorConfig()->motorPoleCount / 2.0f);

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_FSP);
    if (portConfig) {
        portOptions_e options = SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1 | SERIAL_PARITY_NO;
        serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, NULL,
                                                  baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, options);
        if (serialPort) {
            fspState.port = serialPort;
            fspState.descriptor = mspDescriptorAlloc();
            fspCobsDecoderInit(&fspState.cobsDecoder, fspState.inBuf, sizeof(fspState.inBuf));
        }
    }
}

#ifdef FSP_ENABLE_MAVLINK
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

void fspHandleMavlinkMessage(const mavlink_message_t *msg, const mavlink_status_t *status)
{
    (void)status;
    uint16_t length = mavlink_msg_to_send_buffer(fspState.mavlinkPacketBuffer, msg);
    fspMavlinkBufferWrite(fspState.mavlinkPacketBuffer, length);
}
#endif

static void fspSendFrames(timeUs_t currentTimeUs)
{
    fspFcTxPacket_t txPacket = {
        .header =
            {
                .version = FSP_VERSION,
                .timestamp = currentTimeUs,
            },
    };

    // Copy sensor frames from queue to packet
    for (size_t i = 0; i < FSP_SENSOR_FRAME_BATCH_COUNT; i++) {
        txPacket.sensorFrames[i] = fspState.sensorFrameQueue[fspState.sensorFrameQueueTail];
        fspState.sensorFrameQueueTail = (fspState.sensorFrameQueueTail + 1) % FSP_SENSOR_FRAME_QUEUE_SIZE;
    }

#ifdef FSP_ENABLE_MAVLINK
    // Tunnel recorded MAVLink messages from buffer
    size_t mavlinkLen = fspMavlinkBufferRead(txPacket.mavlink.data, sizeof(txPacket.mavlink.data));
    memset(txPacket.mavlink.data + mavlinkLen, 0, sizeof(txPacket.mavlink.data) - mavlinkLen);
#endif

    size_t encodedLength;
    txPacket.crc = crc8_update(0xFF, &txPacket, sizeof(txPacket) - 1, FSP_CRC_POLY);
    if (fspCobsEncode((uint8_t *)&txPacket, sizeof(txPacket), fspState.outBuf, sizeof(fspState.outBuf),
                      &encodedLength)) {
        serialWriteBuf(fspState.port, fspState.outBuf, encodedLength);
    }
}

static void fspReceiveFrames(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    while (serialRxBytesWaiting(fspState.port)) {
        const uint8_t c = serialRead(fspState.port);
        size_t decodedLength;
        switch (fspCobsDecoderPush(&fspState.cobsDecoder, c, &decodedLength)) {
        case FSP_COBS_DECODER_DONE:
            if (decodedLength == sizeof(fspFcRxPacket_t)) {
                fspFcRxPacket_t *rxPacket = (fspFcRxPacket_t *)fspState.inBuf;
                uint8_t crc = crc8_update(0xFF, rxPacket, sizeof(*rxPacket) - 1, FSP_CRC_POLY);
                if (crc != rxPacket->crc) {
                    break;
                }
                if (rxPacket->header.version != FSP_VERSION) {
                    break;
                }

                uint16_t frame[] = {
                    [0] = rxPacket->rc.roll, [1] = rxPacket->rc.pitch, [2] = rxPacket->rc.throttle,
                    [3] = rxPacket->rc.yaw,  [6] = rxPacket->rc.aux3,  [7] = rxPacket->rc.aux4,
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

void fspUpdate(timeUs_t currentTimeUs)
{
    if (!fspState.port) {
        return;
    }

    // Check if FSP_SENSOR_FRAME_BATCH_COUNT frames are available in the queue
    size_t framesAvailable =
        (fspState.sensorFrameQueueHead + FSP_SENSOR_FRAME_QUEUE_SIZE - fspState.sensorFrameQueueTail) %
        FSP_SENSOR_FRAME_QUEUE_SIZE;
    if (framesAvailable >= FSP_SENSOR_FRAME_BATCH_COUNT) {
        fspSendFrames(currentTimeUs);
    }

    fspReceiveFrames(currentTimeUs);
}

#ifdef FSP_PID_LOCKSTEP
bool fspFrameDue(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    fspState.captureCounter++;
    if (fspState.captureCounter >= fspState.captureInterval) {
        fspState.captureCounter = 0;
        return true;
    }
    return false;
}
#else
bool fspFrameDue(timeUs_t currentTimeUs)
{
    // Initialize next capture time if not valid
    if (!fspState.nextCaptureTimeValid) {
        fspState.nextCaptureTimeValid = true;
        fspState.nextCaptureTimeUs = currentTimeUs;
    }

    // Check if it's time for the next capture
    bool due = cmpTimeUs(currentTimeUs, fspState.nextCaptureTimeUs) >= 0;

    if (due) {
        // Schedule next capture time
        const timeDelta_t targetDelta = 1000000 / FSP_PERIOD_HZ;
        fspState.nextCaptureTimeUs += targetDelta;
    }

    return due;
}
#endif

void fspPushSensorFrame(timeUs_t currentTimeUs)
{
    if (!fspFrameDue(currentTimeUs)) {
        return;
    }

    // Check if there is space in the queue
    if (((fspState.sensorFrameQueueHead + 1) % FSP_SENSOR_FRAME_QUEUE_SIZE) == fspState.sensorFrameQueueTail) {
        // No space in the queue, drop the frame
        return;
    }

    // Fill frame
    fspSensorFrame_t *frame = &fspState.sensorFrameQueue[fspState.sensorFrameQueueHead];
    fspState.sensorFrameQueueHead = (fspState.sensorFrameQueueHead + 1) % FSP_SENSOR_FRAME_QUEUE_SIZE;

    frame->timestamp = currentTimeUs;
    frame->acc.x = lrintf(acc.accADCf.x);
    frame->acc.y = lrintf(acc.accADCf.y);
    frame->acc.z = lrintf(acc.accADCf.z);
    frame->acc1G = acc.dev.acc_1G;
    frame->gyro.x = gyroRateDps(0);
    frame->gyro.y = gyroRateDps(1);
    frame->gyro.z = gyroRateDps(2);
    frame->gyroDpsLsb = gyro.scale;
    frame->attitude.w = lrintf(imuAttitudeQuaternion.w * FSP_QUATERNION_SCALE);
    frame->attitude.x = lrintf(imuAttitudeQuaternion.x * FSP_QUATERNION_SCALE);
    frame->attitude.y = lrintf(imuAttitudeQuaternion.y * FSP_QUATERNION_SCALE);
    frame->attitude.z = lrintf(imuAttitudeQuaternion.z * FSP_QUATERNION_SCALE);
    frame->rc.roll = lrintf(rcData[ROLL]);
    frame->rc.pitch = lrintf(rcData[PITCH]);
    frame->rc.yaw = lrintf(rcData[YAW]);
    frame->rc.throttle = lrintf(rcData[THROTTLE]);
    frame->rc.aux3 = lrintf(rcData[AUX3]);
    frame->rc.aux4 = lrintf(rcData[AUX4]);

#if defined(ESC_XR8_PRO) && FSP_CAR_FIRMWARE_FLAG
    escSensorData_t *esc = getEscSensorData(0);
    xr8ProTelemetryFrame_t *escFrame = (xr8ProTelemetryFrame_t *)escSensorXR8ProFrame();
    if (esc && escFrame) {
        frame->rpm = escFrame->rpm * fspState.escToRpmScale;
        frame->esc_current = esc->current;
        frame->esc_temperature = esc->temperature;
        frame->esc_reverse = escFrame->reverse;
        frame->esc_throttle = escFrame->throttle1;
        frame->controller_throttle = lrintf(getControlledThrottle() * 10000.0f);
    }
#elif defined(USE_DSHOT_TELEMETRY)
    for (int i = 0; i < 4; i++) {
        frame->rpm[i] = getDshotRpm(i);
    }
#endif

#if defined (USE_THROTTLE_CONTROL)
    frame->controller_throttle = lrintf(getControlledThrottle() * 10000.0f);
#else
    frame->controller_throttle = 0;
#endif
    frame->batteryVoltage = getBatteryVoltage();
}

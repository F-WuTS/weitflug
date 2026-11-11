#include <stdint.h>
#include <string.h>

#include "fsp/fsp.h"
#include "drivers/time.h"
#include "fsp/fsp_cobs.h"
#include "fsp/fsp_frame.h"
#include "fsp/fsp_actions.h"
#include "config/feature.h"
#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "drivers/motor.h"
#include "flight/pid.h"

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
#include "pg/fsp.h"

#include "platform.h"

#define FSP_MAX_PACKET_SIZE 255
// COBS adds one code byte per 254 data bytes plus the delimiter
#define FSP_COBS_ENCODED_SIZE(n) ((n) + (n) / 254 + 2)
#define FSP_FRAME_SIZE(payloadSize) (sizeof(fspPacketHeader_t) + (payloadSize) + 1)
// One slot stays empty to tell a full queue from an empty one: four frames of slack for a late FSP task
#define FSP_SENSOR_FRAME_QUEUE_SIZE 5

#ifdef FSP_ENABLE_MAVLINK
#define FSP_MAVLINK_BUFFER_SIZE 512
#endif

typedef struct {
    struct serialPort_s *port;
    mspDescriptor_t descriptor;
    bool telemetryEnabled;
    timeUs_t lastTelemetryRequestUs;
    fspCobsDecoder_t cobsDecoder;
    timeUs_t telemetryTimeoutUs;
    // Buffers are aligned to 4 bytes so decoded frames can be read through the fsp_frame.h structs in place
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
    timeUs_t statusIntervalUs;
    timeUs_t nextStatusUs;
#ifdef FSP_ENABLE_MAVLINK
    uint8_t mavlinkPacketBuffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t mavlinkBuffer[FSP_MAVLINK_BUFFER_SIZE];
    size_t mavlinkBufferHead, mavlinkBufferTail;
#endif
} fspState_t;

static fspState_t fspState;
static fspActionScheduler_t actions;
#if defined(ENABLE_PT2_FILTER_IMU) && ENABLE_PT2_FILTER_IMU
#include "fsp/fsp_imu_filter.h"
static fspImuPt2Filter_t telemetryAcc, telemetryGyro;
void fspImuAccelerometerUpdate(const float values[3], unsigned rateHz, timeUs_t timestampUs)
{
    fspImuPt2FilterUpdate(&telemetryAcc, values, rateHz, timestampUs);
}
void fspImuGyroscopeUpdate(const float values[3], unsigned rateHz, timeUs_t timestampUs)
{
    fspImuPt2FilterUpdate(&telemetryGyro, values, rateHz, timestampUs);
}
#endif


// Forward declare from rx/msp.h, because of a missing typedef in the header
void rxMspFrameReceive(const uint16_t *frame, int channelCount);

void fspInit(void)
{
    memset(&fspState, 0, sizeof(fspState_t));
    fspState.telemetryTimeoutUs = fspConfig()->telemetry_timeout_ms * 1000U;
    fspState.statusIntervalUs = HZ_TO_INTERVAL_US(fspConfig()->status_hz);
    fspActionsInit(&actions, fspConfig()->action_timeout_ms * 1000U, fspConfig()->action_max_ahead_ticks,
                   targetPidLooptime);

#ifdef FSP_PID_LOCKSTEP
    // Capture sample at every Nth PID loop iteration, N is chosen to capture the sample at
    // the largest possible frequency <= fsp_telemetry_hz so the queue is certain to be consumed
    // fast enough regardless of batching.
    fspState.captureInterval = HZ_TO_INTERVAL_US(fspConfig()->telemetry_hz) / targetPidLooptime + 1;
#endif

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

// Frame, CRC and COBS-encode one packet; nothing is written unless it fits the TX buffer.
static bool fspSendPacket(fspPacketType_e type, const void *payload, size_t payloadSize)
{
    uint8_t frame[FSP_FRAME_SIZE(FSP_MAX_PAYLOAD_SIZE)] __attribute__((aligned(4)));
    if (FSP_FRAME_SIZE(payloadSize) > sizeof(frame)) {
        return false;
    }
    const fspPacketHeader_t header = {
        .version = FSP_VERSION,
        .packetType = type,
        .timestamp = micros(),
    };
    memcpy(frame, &header, sizeof(header));
    memcpy(frame + sizeof(header), payload, payloadSize);
    const size_t crcOffset = sizeof(header) + payloadSize;
    frame[crcOffset] = crc8_update(0xFF, frame, crcOffset, FSP_CRC_POLY);

    size_t encodedLength;
    if (!fspCobsEncode(frame, crcOffset + 1, fspState.outBuf, sizeof(fspState.outBuf), &encodedLength) ||
        serialTxBytesFree(fspState.port) < encodedLength) {
        return false;
    }
    serialWriteBuf(fspState.port, fspState.outBuf, encodedLength);
    return true;
}

static bool fspTxFits(size_t payloadSize)
{
    return serialTxBytesFree(fspState.port) >= FSP_COBS_ENCODED_SIZE(FSP_FRAME_SIZE(payloadSize));
}

// One sensor frame per packet: a capture leaves with the next FSP task run instead of waiting for a batch.
static void fspSendState(void)
{
    fspPacketState_t state;
    state.frame = fspState.sensorFrameQueue[fspState.sensorFrameQueueTail];
    fspState.sensorFrameQueueTail = (fspState.sensorFrameQueueTail + 1) % FSP_SENSOR_FRAME_QUEUE_SIZE;
    fspSendPacket(FSP_PACKET_TYPE_STATE, &state, sizeof(state));
}

static void fspSendStatus(timeUs_t currentTimeUs)
{
    fspPacketStatus_t status = {0};
    status.pidTick = actions.pidTick;
    status.timestamp = currentTimeUs;
    status.attitude.w = lrintf(imuAttitudeQuaternion.w * FSP_QUATERNION_SCALE);
    status.attitude.x = lrintf(imuAttitudeQuaternion.x * FSP_QUATERNION_SCALE);
    status.attitude.y = lrintf(imuAttitudeQuaternion.y * FSP_QUATERNION_SCALE);
    status.attitude.z = lrintf(imuAttitudeQuaternion.z * FSP_QUATERNION_SCALE);
    status.aux3 = lrintf(rcData[AUX3]);
    status.aux4 = lrintf(rcData[AUX4]);
#if defined(ESC_XR8_PRO)
    xr8ProTelemetryFrame_t *escFrame = (xr8ProTelemetryFrame_t *)escSensorXR8ProFrame();
    if (escFrame) {
        status.aux3 = escFrame->throttle1;
        status.aux4 = escFrame->throttle2;
    }
#endif
    status.batteryVoltage = getBatteryVoltageLatest();
    status.batteryAmperage = constrain(getAmperageLatest(), INT16_MIN, INT16_MAX);
    status.gyroDpsLsb = gyro.scale;
    status.acc1G = acc.dev.acc_1G;
    fspActionsStatus(&actions, &status);
    fspSendPacket(FSP_PACKET_TYPE_STATUS, &status, sizeof(status));
}

#ifdef FSP_ENABLE_MAVLINK
static void fspSendMavlink(void)
{
    if (fspState.mavlinkBufferHead == fspState.mavlinkBufferTail || !fspTxFits(sizeof(fspPacketMavlink_t))) {
        return;
    }
    fspPacketMavlink_t mavlink;
    const size_t length = fspMavlinkBufferRead(mavlink.data, sizeof(mavlink.data));
    if (length) {
        fspSendPacket(FSP_PACKET_TYPE_MAVLINK, &mavlink, length);
    }
}
#endif

static void fspRenewTelemetryLease(timeUs_t currentTimeUs)
{
    if (!fspState.telemetryEnabled) {
        fspState.nextStatusUs = currentTimeUs; // the host gets the scales with the first frames
    }
    fspState.telemetryEnabled = true;
    fspState.lastTelemetryRequestUs = currentTimeUs;
}

// RC actions go straight into the MSP override, like the mainline RC packet.
// Channels beyond throttle/yaw are not sent and stay with the radio.
static void fspApplyRcAction(const fspPacketAction_t *action)
{
    const uint16_t frame[] = {
        [0] = action->values[0], // roll
        [1] = action->values[1], // pitch
        [2] = action->values[3], // throttle
        [3] = action->values[2], // yaw
    };
    rxMspFrameReceive(frame, ARRAYLEN(frame));
}

static void fspReceiveAction(const fspPacketAction_t *action, timeUs_t currentTimeUs)
{
    fspRenewTelemetryLease(currentTimeUs);
    fspActionResult_e result;
    if (action->mode == FSP_ACTION_MODE_RC) {
        result = fspActionsCheckRc(&actions, action);
        if (result == FSP_ACTION_OK) {
            fspApplyRcAction(action);
        }
    } else {
        result = fspActionsEnqueue(&actions, action);
    }
    const fspPacketActionAck_t ack = {
        .applyTick = action->applyTick,
        .recvTick = actions.pidTick,
        .result = result,
        .queueDepth = actions.count,
    };
    fspSendPacket(FSP_PACKET_TYPE_ACTION_ACK, &ack, sizeof(ack));
}

static void fspReceiveHeartbeat(timeUs_t currentTimeUs)
{
    fspRenewTelemetryLease(currentTimeUs);
    fspSendPacket(FSP_PACKET_TYPE_STATS, &actions.stats, sizeof(actions.stats));
}

static void fspHandleFrame(const uint8_t *frame, size_t size, timeUs_t currentTimeUs)
{
    const fspPacketHeader_t *header = FSP_FRAME_HEADER(frame, size);
    if (!header || header->version != FSP_VERSION ||
        crc8_update(0xFF, frame, size - 1, FSP_CRC_POLY) != *FSP_FRAME_CRC(frame, size)) {
        return;
    }

    switch (header->packetType) {
    case FSP_PACKET_TYPE_HEARTBEAT:
        if (FSP_FRAME_PAYLOAD_LENGTH(size) == 0) {
            fspReceiveHeartbeat(currentTimeUs);
        }
        break;
    case FSP_PACKET_TYPE_ACTION: {
        const fspPacketAction_t *action = FSP_FRAME_PAYLOAD(frame, size, fspPacketAction_t);
        if (action) {
            fspReceiveAction(action, currentTimeUs);
        }
        break;
    }
    default:
        break;
    }
}

static void fspReceiveFrames(timeUs_t currentTimeUs)
{
    for (unsigned budget = 0; budget < 128 && serialRxBytesWaiting(fspState.port); budget++) {
        const uint8_t c = serialRead(fspState.port);
        size_t decodedLength;
        if (fspCobsDecoderPush(&fspState.cobsDecoder, c, &decodedLength) == FSP_COBS_DECODER_DONE) {
            fspHandleFrame(fspState.inBuf, decodedLength, currentTimeUs);
        }
    }
}

void fspUpdate(timeUs_t currentTimeUs)
{
    if (!fspState.port) {
        return;
    }

    fspReceiveFrames(currentTimeUs);
    if (fspState.telemetryEnabled &&
        cmpTimeUs(currentTimeUs, fspState.lastTelemetryRequestUs) >= (timeDelta_t)fspState.telemetryTimeoutUs) {
        fspState.telemetryEnabled = false;
    }
    if (!fspState.telemetryEnabled) {
        fspState.sensorFrameQueueHead = fspState.sensorFrameQueueTail = 0;
        return;
    }

    // Send every queued capture that fits, so a late task run catches up instead of dropping later frames
    while (fspState.sensorFrameQueueHead != fspState.sensorFrameQueueTail && fspTxFits(sizeof(fspPacketState_t))) {
        fspSendState();
    }
    if (cmpTimeUs(currentTimeUs, fspState.nextStatusUs) >= 0 && fspTxFits(sizeof(fspPacketStatus_t))) {
        fspSendStatus(currentTimeUs);
        fspState.nextStatusUs += fspState.statusIntervalUs;
        // After a pause (lease expiry) restart the schedule instead of sending a burst
        if (cmpTimeUs(currentTimeUs, fspState.nextStatusUs) >= 0) {
            fspState.nextStatusUs = currentTimeUs + fspState.statusIntervalUs;
        }
    }
#ifdef FSP_ENABLE_MAVLINK
    fspSendMavlink();
#endif
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
        const timeDelta_t targetDelta = HZ_TO_INTERVAL_US(fspConfig()->telemetry_hz);
        fspState.nextCaptureTimeUs += targetDelta;
    }

    return due;
}
#endif

static int16_t encodeGyro(int axis)
{
    float counts = gyro.gyroADCf[axis] / gyro.rawSensorDev->scale;
    counts = constrainf(counts, (float)INT16_MIN, (float)INT16_MAX);
    return (int16_t)lrintf(counts);
}

void fspPushSensorFrame(timeUs_t currentTimeUs)
{
    if (!fspState.telemetryEnabled || !fspFrameDue(currentTimeUs)) {
        return;
    }

#if defined(ENABLE_PT2_FILTER_IMU) && ENABLE_PT2_FILTER_IMU
    // Sensor hooks may run after the PID snapshot; check age against micros().
    const timeUs_t sampleNowUs = micros();
    if (!fspImuPt2FilterFresh(&telemetryAcc, sampleNowUs) || !fspImuPt2FilterFresh(&telemetryGyro, sampleNowUs)) return;
#endif
    // Check if there is space in the queue
    if (((fspState.sensorFrameQueueHead + 1) % FSP_SENSOR_FRAME_QUEUE_SIZE) == fspState.sensorFrameQueueTail) {
        // No space in the queue, drop the frame
        return;
    }

    // Fill frame
    fspSensorFrame_t *frame = &fspState.sensorFrameQueue[fspState.sensorFrameQueueHead];
    fspState.sensorFrameQueueHead = (fspState.sensorFrameQueueHead + 1) % FSP_SENSOR_FRAME_QUEUE_SIZE;

    memset(frame, 0, sizeof(*frame)); // slots are reused; fields a build does not fill stay zero
    frame->pidTick = actions.pidTick;
    frame->timestamp = currentTimeUs;
    fspActionsFrameState(&actions, frame);
#if defined(ENABLE_PT2_FILTER_IMU) && ENABLE_PT2_FILTER_IMU
    frame->acc.x = lrintf(telemetryAcc.second[0]);
    frame->acc.y = lrintf(telemetryAcc.second[1]);
    frame->acc.z = lrintf(telemetryAcc.second[2]);
    frame->gyro.x = lrintf(telemetryGyro.second[0] / gyro.scale);
    frame->gyro.y = lrintf(telemetryGyro.second[1] / gyro.scale);
    frame->gyro.z = lrintf(telemetryGyro.second[2] / gyro.scale);
#else
    frame->acc.x = lrintf(acc.accADCf.x);
    frame->acc.y = lrintf(acc.accADCf.y);
    frame->acc.z = lrintf(acc.accADCf.z);
    frame->gyro.x = encodeGyro(0);
    frame->gyro.y = encodeGyro(1);
    frame->gyro.z = encodeGyro(2);
#endif
    frame->rc.roll = lrintf(rcData[ROLL]);
    frame->rc.pitch = lrintf(rcData[PITCH]);
    frame->rc.yaw = lrintf(rcData[YAW]);
    frame->rc.throttle = lrintf(rcData[THROTTLE]);

#if defined(ESC_XR8_PRO)
    escSensorData_t *esc = getEscSensorData(0);
    if (esc) {
        frame->rpm[0] = esc->rpm;
        frame->rpm[1] = esc->current;
        frame->rpm[2] = esc->voltage;
        frame->rpm[3] = esc->temperature;
    }
    xr8ProTelemetryFrame_t *escFrame = (xr8ProTelemetryFrame_t *)escSensorXR8ProFrame();
    if (escFrame) {
        frame->rc.throttle = escFrame->reverse;
    }
#elif defined(USE_DSHOT_TELEMETRY)
    for (int i = 0; i < 4; i++) {
        frame->rpm[i] = getDshotRpm(i);
    }
#endif
}

void fspActionPidTick(timeUs_t currentTimeUs)
{
    // Both the FSP task and PID task execute on the cooperative scheduler.
    const bool selected = IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE);
    const bool healthy = !failsafeIsActive() && isRxReceivingSignal();
    const bool armed = ARMING_FLAG(ARMED);
    const bool acro = !FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | GPS_RESCUE_MODE | ALT_HOLD_MODE | POS_HOLD_MODE);
    const bool supported = !featureIsEnabled(FEATURE_3D) && getMotorCount() == 4 &&
        !isCrashFlipModeActive() && !isLaunchControlActive();
    const uint8_t authority = (selected && healthy ? FSP_AUTHORITY_RC : 0) |
                              (armed && acro && supported ? FSP_AUTHORITY_CONTROL : 0);
    fspActionsTick(&actions, currentTimeUs, authority);
}

static bool fspActionModeActive(fspActionMode_e mode)
{
    return fspActionsOutputActive(&actions) && actions.active.mode == mode;
}

bool fspActionRateSetpoint(unsigned axis, float *rate)
{
    if (!fspActionModeActive(FSP_ACTION_MODE_RATES) || axis >= 3) return false;
    *rate = actions.timedOut ? 0.0f : actions.active.values[axis] * 0.1f;
    return true;
}

bool fspActionCollective(float *collective)
{
    if (!fspActionModeActive(FSP_ACTION_MODE_RATES)) return false;
    *collective = actions.timedOut ? 0.0f : actions.active.values[3] * 0.0001f;
    return true;
}

bool fspActionMotorOutputs(float *outputs, unsigned count)
{
    if (count != 4) return false;
    const bool motors = fspActionModeActive(FSP_ACTION_MODE_MOTORS);
    // A timed-out rate action also stops the motors
    if (!motors && !(actions.timedOut && fspActionModeActive(FSP_ACTION_MODE_RATES))) return false;
    for (unsigned i = 0; i < 4; i++) {
        const float value = actions.timedOut ? 0.0f : actions.active.values[i] * 0.0001f;
        outputs[i] = value == 0.0f ? motorConvertFromExternal(1000) :
            getMotorOutputLow() + value * (getMotorOutputHigh() - getMotorOutputLow());
    }
    pidResetIterm();
    return true;
}

bool fspActionBypassPid(void)
{
    return fspActionModeActive(FSP_ACTION_MODE_MOTORS) ||
        (actions.timedOut && fspActionModeActive(FSP_ACTION_MODE_RATES));
}

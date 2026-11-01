#pragma once

#include "common/time.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

void fspInit(void);
void fspUpdate(timeUs_t currentTimeUs);
void fspPushSensorFrame(timeUs_t currentTimeUs);
#ifdef FSP_ENABLE_MAVLINK
void fspHandleMavlinkMessage(const mavlink_message_t *msg, const mavlink_status_t *status);
#endif

#if defined(ENABLE_PT2_FILTER_IMU) && ENABLE_PT2_FILTER_IMU
void fspImuAccelerometerUpdate(const float values[3], unsigned rateHz, timeUs_t timestampUs);
void fspImuGyroscopeUpdate(const float values[3], unsigned rateHz, timeUs_t timestampUs);
#endif

#include "fsp_frame.h"
void fspActionPidTick(timeUs_t currentTimeUs);
void fspActionMotorWritten(timeUs_t currentTimeUs);
uint8_t fspReceiveAction(const fspActionPacket_t *action, timeUs_t receivedTimeUs);
void fspGetActionStatus(fspActionStatus_t *status);
bool fspActionRateSetpoint(unsigned axis, float *rate);
bool fspActionCollective(float *collective);
bool fspActionMotorOutputs(float *outputs, unsigned count);
bool fspActionBypassPid(void);

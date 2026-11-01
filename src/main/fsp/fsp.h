#pragma once

#include <stdbool.h>

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

// Scheduled actions, called from the PID loop
void fspActionPidTick(timeUs_t currentTimeUs);
bool fspActionRateSetpoint(unsigned axis, float *rate);
bool fspActionCollective(float *collective);
bool fspActionMotorOutputs(float *outputs, unsigned count);
bool fspActionBypassPid(void);

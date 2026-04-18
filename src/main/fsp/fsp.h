#pragma once

#include "common/time.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

void fspInit(void);
void fspUpdate(timeUs_t currentTimeUs);
void fspPushSensorFrame(timeUs_t currentTimeUs);
void fspHandleMavlinkMessage(const mavlink_message_t *msg, const mavlink_status_t *status);

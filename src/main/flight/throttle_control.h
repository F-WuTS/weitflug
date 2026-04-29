#pragma once

#include "common/time.h"

#define THROTTLE_CONTROL_TASK_RATE_HZ 100

void throttleControlInit(void);
void throttleControlUpdate(timeUs_t currentTimeUs);
float getControlledThrottle();

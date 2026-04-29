#pragma once

#include <stdint.h>

#include "pg/pg.h"

typedef struct throttleControlConfig_s
{
    uint16_t throttle_min;
    uint16_t throttle_max;
    uint16_t throttle_min_rpm;
    uint16_t throttle_max_rpm;
    uint16_t throttle_kp;
    uint16_t throttle_ki;
    uint16_t throttle_max_integral;
} throttleControlConfig_t;

PG_DECLARE(throttleControlConfig_t, throttleControlConfig);

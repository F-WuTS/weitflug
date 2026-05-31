#pragma once

#include <stdint.h>

#include "pg/pg.h"

#define THROTTLE_FF_DRAG_CURVE_SIZE 2
#define THROTTLE_FF_ACCEL_CURVE_SIZE 2
#define THROTTLE_FF_BRAKE_CURVE_SIZE 1

#define THROTTLE_FF_VOLTAGE_MIN_SCALE 1e-1f
#define THROTTLE_FF_MAX_ACCEL_SCALE 1e-1f
#define THROTTLE_FF_DRAG_SCALE_2 1e-13f
#define THROTTLE_FF_DRAG_SCALE_1 1e-9f
#define THROTTLE_FF_ACCEL_SCALE_2 1e-7f
#define THROTTLE_FF_ACCEL_SCALE_1 1e-6f
#define THROTTLE_FF_BRAKE_SCALE_1 1e-6f

typedef struct throttleControlConfig_s
{
    uint16_t throttle_min;
    uint16_t throttle_max;
    uint16_t throttle_min_rpm;
    uint16_t throttle_max_rpm;
    uint16_t throttle_kp;
    uint16_t throttle_ki;
    uint16_t throttle_kd;
    uint16_t throttle_max_integral;
    uint8_t throttle_esc_deadband;
    uint8_t throttle_esc_brake_strength;
    uint16_t throttle_ff_voltage_min;
    uint16_t throttle_ff_motor_kv;
    uint16_t throttle_ff_max_accel;
    uint16_t throttle_ff_drag_curve[THROTTLE_FF_DRAG_CURVE_SIZE];
    uint16_t throttle_ff_accel_curve[THROTTLE_FF_ACCEL_CURVE_SIZE];
    uint16_t throttle_ff_brake_curve[THROTTLE_FF_BRAKE_CURVE_SIZE];
    uint8_t throttle_rpm_filter_cuttoff_hz;
    uint8_t throttle_rpm_setpoint_tau_ms;
    uint16_t throttle_brake_disable_rpm;
} throttleControlConfig_t;

PG_DECLARE(throttleControlConfig_t, throttleControlConfig);

#include "platform.h"

#ifdef USE_THROTTLE_CONTROL

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/throttle_control.h"

PG_REGISTER_WITH_RESET_TEMPLATE(throttleControlConfig_t, throttleControlConfig, PG_THROTTLE_CONTROL_CONFIG, 8);

PG_RESET_TEMPLATE(throttleControlConfig_t, throttleControlConfig,
    .throttle_min = 1550,
    .throttle_max = 2000,
    .throttle_min_rpm = 0,
    .throttle_max_rpm = 15000,
    .throttle_kp = 250,
    .throttle_ki = 150,
    .throttle_kd = 0,
    .throttle_max_integral = 10000,
    .throttle_esc_deadband = 3,
    .throttle_esc_brake_strength = 20,
    .throttle_ff_voltage_min = 140,
    .throttle_ff_motor_kv = 0,
    .throttle_ff_max_accel = 0,
    .throttle_ff_drag_curve = {0, 0},
    .throttle_ff_accel_curve = {0, 0},
    .throttle_ff_brake_curve = {0},
    .throttle_rpm_filter_cuttoff_hz = 25,
    .throttle_rpm_setpoint_tau_ms = 50,
    .throttle_brake_disable_rpm = 800,
);

#endif // USE_THROTTLE_CONTROL

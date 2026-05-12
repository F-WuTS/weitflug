#include "platform.h"

#ifdef USE_THROTTLE_CONTROL

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/throttle_control.h"

PG_REGISTER_WITH_RESET_TEMPLATE(throttleControlConfig_t, throttleControlConfig, PG_THROTTLE_CONTROL_CONFIG, 2);

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
);

#endif // USE_THROTTLE_CONTROL

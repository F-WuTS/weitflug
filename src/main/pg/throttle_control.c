#include "platform.h"

#ifdef USE_THROTTLE_CONTROL

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/throttle_control.h"

PG_REGISTER_WITH_RESET_TEMPLATE(throttleControlConfig_t, throttleControlConfig, PG_THROTTLE_CONTROL_CONFIG, 1);

PG_RESET_TEMPLATE(throttleControlConfig_t, throttleControlConfig,
    .throttle_min = 1550,
    .throttle_max = 2000,
    .throttle_min_rpm = 500,
    .throttle_max_rpm = 5000,
    .throttle_kp = 5,
    .throttle_ki = 1,
    .throttle_max_integral = 1000,
);

#endif // USE_THROTTLE_CONTROL

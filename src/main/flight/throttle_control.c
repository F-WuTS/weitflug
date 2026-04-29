#include "flight/throttle_control.h"

#include "common/maths.h"
#include "fc/rc_controls.h"
#include "pg/throttle_control.h"
#include "sensors/esc_sensor.h"

#define THROTTLE_MIN 1550.0f
#define THROTTLE_MAX 2000.0f
#define THROTTLE_MIN_RPM 500.0f
#define THROTTLE_MAX_RPM 5000.0f
#define ERROR_INTEGRAL_LIMIT 1000.0f

static struct {
    float controlledThrottle;
    float kp;
    float ki;
    float errorIntegral;
    float maxIntegral;
} tcRuntime;

void throttleControlInit(void)
{
    tcRuntime.controlledThrottle = 0.0f;
    tcRuntime.errorIntegral = 0.0f;
    tcRuntime.kp = (float)throttleControlConfig()->throttle_kp;
    tcRuntime.ki = (float)throttleControlConfig()->throttle_ki / THROTTLE_CONTROL_TASK_RATE_HZ;
    tcRuntime.maxIntegral = (float)throttleControlConfig()->throttle_max_integral;
}

void throttleControlUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    float throttleMin = throttleControlConfig()->throttle_min;
    float throttleMax = throttleControlConfig()->throttle_max;
    float throttleMinRpm = throttleControlConfig()->throttle_min_rpm;
    float throttleMaxRpm = throttleControlConfig()->throttle_max_rpm;
    float throttleSetpoint = rcCommand[PITCH];

    if (throttleSetpoint < throttleMin) {
        tcRuntime.controlledThrottle = 0.0f;
        return;
    }

    throttleSetpoint = constrainf(throttleSetpoint, throttleMin, throttleMax);
    throttleSetpoint = scaleRangef(throttleSetpoint, throttleMin, throttleMax, throttleMinRpm, throttleMaxRpm);

    escSensorData_t *escData = getEscSensorData(0);
    float currentRpm = (float)escData->rpm;

    float error = throttleSetpoint - currentRpm;
    tcRuntime.errorIntegral += tcRuntime.ki * error;
    tcRuntime.errorIntegral = constrainf(tcRuntime.errorIntegral, -tcRuntime.maxIntegral, tcRuntime.maxIntegral);

    tcRuntime.controlledThrottle = tcRuntime.kp * error + tcRuntime.errorIntegral;
    tcRuntime.controlledThrottle = constrainf(tcRuntime.controlledThrottle, 0.0f, 1.0f);
}

float getControlledThrottle(void) { return tcRuntime.controlledThrottle; }

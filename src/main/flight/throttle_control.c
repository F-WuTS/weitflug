#include "flight/throttle_control.h"

#include "build/debug.h"
#include "common/maths.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "pg/throttle_control.h"
#include "platform.h"
#include "rx/rx.h"
#include "sensors/esc_sensor.h"

#include <math.h>

static struct {
    float controlledThrottle;
    float kp;
    float ki;
    float kd;
    float kff;
    float errorIntegral;
    float maxIntegral;
    float lastError;
} tcRuntime;

static float mapThrottle(float throttle)
{
    float deadband = (float)throttleControlConfig()->throttle_esc_deadband * 0.01f;
    float escBrakeStrength = (float)throttleControlConfig()->throttle_esc_brake_strength * 0.01f;
    if (throttle >= 0.0f) {
        return scaleRangef(throttle, 0.0f, 1.0f, 0.5f + deadband, 1.0f);
    }
    else {
        return scaleRangef(throttle, -escBrakeStrength, 0.0f, 0.0f, (0.5f - deadband));
    }
}

void throttleControlInit(void)
{
    tcRuntime.controlledThrottle = 0.0f;
    tcRuntime.errorIntegral = 0.0f;
    tcRuntime.lastError = 0.0f;
    tcRuntime.kp = (float)throttleControlConfig()->throttle_kp * 1e-7f;
    tcRuntime.ki = (float)throttleControlConfig()->throttle_ki * 1e-7f / THROTTLE_CONTROL_TASK_RATE_HZ;
    tcRuntime.kd = (float)throttleControlConfig()->throttle_kd * 1e-9f * THROTTLE_CONTROL_TASK_RATE_HZ;
    tcRuntime.kff = 1.0f / (float)throttleControlConfig()->throttle_motor_kv;
    tcRuntime.maxIntegral = (float)throttleControlConfig()->throttle_max_integral * 1e-5f;
}

void throttleControlUpdate(timeUs_t currentTimeUs)
{
    (void)currentTimeUs;

    if (!ARMING_FLAG(ARMED)) {
        return;
    }

    float throttleMin = throttleControlConfig()->throttle_min;
    float throttleMax = throttleControlConfig()->throttle_max;
    float throttleMinRpm = throttleControlConfig()->throttle_min_rpm;
    float throttleMaxRpm = throttleControlConfig()->throttle_max_rpm;
    float throttleSetpoint = rcData[PITCH];

    throttleSetpoint = constrainf(throttleSetpoint, throttleMin, throttleMax);
    throttleSetpoint = scaleRangef(throttleSetpoint, throttleMin, throttleMax, throttleMinRpm, throttleMaxRpm);

    escSensorData_t *escData = getEscSensorData(0);
    float currentRpm = (float)escData->rpm;
    float voltage = escData->voltage * 0.01f;

    float error = throttleSetpoint - currentRpm;
    tcRuntime.errorIntegral += tcRuntime.ki * error;
    tcRuntime.errorIntegral = constrainf(tcRuntime.errorIntegral, -tcRuntime.maxIntegral, tcRuntime.maxIntegral);

    float throttle = (tcRuntime.kff / voltage * throttleSetpoint +   // feedforward term
                      tcRuntime.kp * error +                         // proportional term
                      tcRuntime.errorIntegral +                      // integral term
                      tcRuntime.kd * (error - tcRuntime.lastError)); // derivative term
    throttle = constrainf(throttle, -1.0f, 1.0f);
    tcRuntime.controlledThrottle = mapThrottle(throttle);
    tcRuntime.lastError = error;

    DEBUG_SET(DEBUG_WING_SETPOINT, 0, lrintf(throttleSetpoint));
    DEBUG_SET(DEBUG_WING_SETPOINT, 1, lrintf(currentRpm));
    DEBUG_SET(DEBUG_WING_SETPOINT, 2, lrintf(error));
    DEBUG_SET(DEBUG_WING_SETPOINT, 3, lrintf(throttle * 10000.0f));
    DEBUG_SET(DEBUG_WING_SETPOINT, 4, lrintf(tcRuntime.controlledThrottle * 10000.0f));

    if (rcData[PITCH] < throttleMin) {
        tcRuntime.controlledThrottle = 0.0f;
        tcRuntime.errorIntegral = 0.0f;
    }
}

float getControlledThrottle(void) { return tcRuntime.controlledThrottle; }

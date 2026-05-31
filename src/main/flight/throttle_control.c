#include "flight/throttle_control.h"

#include "build/debug.h"
#include "common/filter.h"
#include "common/maths.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "pg/motor.h"
#include "pg/throttle_control.h"
#include "platform.h"
#include "platform/platform.h"
#include "rx/rx.h"
#include "sensors/esc_sensor.h"
#include "sensors/esc_sensor_xr8pro.h"
#include "target/common_post.h"

#include <math.h>

FAST_DATA_ZERO_INIT static struct {
    float controlledThrottle;
    float kp;
    float ki;
    float kd;
    float esc_lsb_to_rpm;
    float voltage_min;
    float k_rpm_ff;
    float ff_accel_scale;
    float dragCurve[THROTTLE_FF_DRAG_CURVE_SIZE];
    float accelCurve[THROTTLE_FF_ACCEL_CURVE_SIZE];
    float brakeCurve[THROTTLE_FF_BRAKE_CURVE_SIZE];
    float errorIntegral;
    float maxIntegral;
    float lastError;
    pt2Filter_t rpmFilter;
    pt1Filter_t rpmSetpointFilter;
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
#if defined(ESC_XR8_PRO)
    tcRuntime.esc_lsb_to_rpm = 10.0f / (motorConfig()->motorPoleCount / 2.0f);
#else
    tcRuntime.esc_lsb_to_rpm = 100.0f / (motorConfig()->motorPoleCount / 2.0f);
#endif
    tcRuntime.voltage_min = (float)throttleControlConfig()->throttle_ff_voltage_min * THROTTLE_FF_VOLTAGE_MIN_SCALE;
    tcRuntime.kp = (float)throttleControlConfig()->throttle_kp * 1e-5f;
    tcRuntime.ki = (float)throttleControlConfig()->throttle_ki * 1e-5f / THROTTLE_CONTROL_TASK_RATE_HZ;
    tcRuntime.kd = (float)throttleControlConfig()->throttle_kd * 1e-7f * THROTTLE_CONTROL_TASK_RATE_HZ;
    tcRuntime.k_rpm_ff = 1.0f / (float)throttleControlConfig()->throttle_ff_motor_kv;
    tcRuntime.ff_accel_scale =
        (float)throttleControlConfig()->throttle_ff_max_accel * THROTTLE_FF_MAX_ACCEL_SCALE / 500.0f;
    tcRuntime.dragCurve[0] = (float)throttleControlConfig()->throttle_ff_drag_curve[0] * THROTTLE_FF_DRAG_SCALE_2;
    tcRuntime.dragCurve[1] = (float)throttleControlConfig()->throttle_ff_drag_curve[1] * THROTTLE_FF_DRAG_SCALE_1;
    tcRuntime.accelCurve[0] = (float)throttleControlConfig()->throttle_ff_accel_curve[0] * THROTTLE_FF_ACCEL_SCALE_2;
    tcRuntime.accelCurve[1] = (float)throttleControlConfig()->throttle_ff_accel_curve[1] * THROTTLE_FF_ACCEL_SCALE_1;
    tcRuntime.brakeCurve[0] = (float)throttleControlConfig()->throttle_ff_brake_curve[0] * THROTTLE_FF_BRAKE_SCALE_1;
    tcRuntime.maxIntegral = (float)throttleControlConfig()->throttle_max_integral * 1e-5f;

    float dT = 1.0f / THROTTLE_CONTROL_TASK_RATE_HZ;
    if (throttleControlConfig()->throttle_rpm_filter_cuttoff_hz > 0) {
        float cutoffHz = (float)throttleControlConfig()->throttle_rpm_filter_cuttoff_hz;
        pt2FilterInit(&tcRuntime.rpmFilter, pt2FilterGain(cutoffHz, dT));
    }
    pt1FilterInit(&tcRuntime.rpmSetpointFilter,
                  pt1FilterGainFromDelay(throttleControlConfig()->throttle_rpm_setpoint_tau_ms * 1e-3f, dT));
}

FAST_CODE void throttleControlUpdate(timeUs_t currentTimeUs)
{
    (void)currentTimeUs;

    if (!ARMING_FLAG(ARMED)) {
        // Disarmed: coast
        tcRuntime.controlledThrottle = 0.5f;
        tcRuntime.errorIntegral = 0.0f;
        return;
    }

    float throttleMin = throttleControlConfig()->throttle_min;
    float throttleMax = throttleControlConfig()->throttle_max;
    float throttleMinRpm = throttleControlConfig()->throttle_min_rpm;
    float throttleMaxRpm = throttleControlConfig()->throttle_max_rpm;
    float throttleSetpoint = rcData[PITCH];
    float accelerationSetpoint = rcData[ROLL] - 1500.0f;

    throttleSetpoint = constrainf(throttleSetpoint, throttleMin, throttleMax);
    throttleSetpoint = scaleRangef(throttleSetpoint, throttleMin, throttleMax, throttleMinRpm, throttleMaxRpm);
    accelerationSetpoint = accelerationSetpoint * tcRuntime.ff_accel_scale;

    float filteredThrottleSetpoint = pt1FilterApply(&tcRuntime.rpmSetpointFilter, throttleSetpoint);

    escSensorData_t *escData = getEscSensorData(0);
#if defined(ESC_XR8_PRO)
    // Use higher resolution if available from XR8 Pro telemetry
    xr8ProTelemetryFrame_t *escFrame = (xr8ProTelemetryFrame_t *)escSensorXR8ProFrame();
    float currentRpm = escFrame ? (float)escFrame->rpm * tcRuntime.esc_lsb_to_rpm : 0.0f;
#else
    float currentRpm = escData ? (float)escData->rpm * tcRuntime.esc_lsb_to_rpm : 0.0f;
#endif

    if (throttleControlConfig()->throttle_rpm_filter_cuttoff_hz > 0) {
        currentRpm = pt2FilterApply(&tcRuntime.rpmFilter, currentRpm);
    }

    float voltage = MAX(escData->voltage * 0.01f, tcRuntime.voltage_min);

    float error = throttleSetpoint - currentRpm;
    float filteredError = filteredThrottleSetpoint - currentRpm;
    tcRuntime.errorIntegral += tcRuntime.ki * error;
    tcRuntime.errorIntegral = constrainf(tcRuntime.errorIntegral, -tcRuntime.maxIntegral, tcRuntime.maxIntegral);

    float rpm_ff = tcRuntime.k_rpm_ff / voltage * throttleSetpoint; // rpm feedforward term
    // Drag feedforward terms
    float drag_ff = (tcRuntime.dragCurve[0] * currentRpm * currentRpm + tcRuntime.dragCurve[1] * currentRpm) / voltage;

    // PID Terms, PID output is acceleration
    float pid = tcRuntime.kp * error +                                // proportional term
                tcRuntime.errorIntegral +                             // integral term
                tcRuntime.kd * (filteredError - tcRuntime.lastError); // derivative term
    if (rcData[AUX3] >= 1500) {
        // If AUX3 is high, disable PID and only use feedforward (for testing/tuning)
        tcRuntime.errorIntegral = 0.0f;
        pid = 0.0f;
    }

    float total_accel = pid + accelerationSetpoint;

    // Acceleration feedforward terms
    float throttle_accel;
    if (total_accel >= 0.0f) {
        throttle_accel =
            (tcRuntime.accelCurve[0] * total_accel * total_accel + tcRuntime.accelCurve[1] * total_accel) / voltage;
    }
    else {
        throttle_accel = tcRuntime.brakeCurve[0] * total_accel;
    }

    float throttle = constrainf(rpm_ff + drag_ff + throttle_accel, -1.0f, 1.0f);
    tcRuntime.controlledThrottle = mapThrottle(throttle);
    tcRuntime.lastError = filteredError;

    if (currentRpm < throttleControlConfig()->throttle_brake_disable_rpm) {
        // If below certain RPM, disable braking (to prevent sticking when trying to take off)
        tcRuntime.controlledThrottle = MAX(tcRuntime.controlledThrottle, 0.5f);
    }

    DEBUG_SET(DEBUG_WING_SETPOINT, 0, lrintf(throttleSetpoint));
    DEBUG_SET(DEBUG_WING_SETPOINT, 1, lrintf(currentRpm));
    DEBUG_SET(DEBUG_WING_SETPOINT, 2, lrintf(error));
    DEBUG_SET(DEBUG_WING_SETPOINT, 3, lrintf(throttle * 10000.0f));
    DEBUG_SET(DEBUG_WING_SETPOINT, 4, lrintf(tcRuntime.controlledThrottle * 10000.0f));
    DEBUG_SET(DEBUG_WING_SETPOINT, 5, lrintf(accelerationSetpoint * 100.0f));

    if (rcData[PITCH] < throttleMin) {
        // Armed but throttle stick below minimum -> brake with 15% force
        tcRuntime.controlledThrottle = mapThrottle(-0.15f);
        tcRuntime.errorIntegral = 0.0f;
    }
}

float getControlledThrottle(void) { return tcRuntime.controlledThrottle; }

#include "pid_core.h"

static float pid_core_limit(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

void PIDCore_Init(pid_core_t *pid, float kp, float ki, float kd,
    float dt_sec, float integral_min, float integral_max,
    float output_min, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt_sec = dt_sec;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->output_min = output_min;
    pid->output_max = output_max;
    PIDCore_Reset(pid);
}

void PIDCore_Reset(pid_core_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->derivative_ready = 0U;
}

float PIDCore_RunConditional(pid_core_t *pid, float target, float feedback,
    uint8_t integrate_enabled)
{
    float error = target - feedback;
    float derivative = 0.0f;
    float integral_candidate;
    float output_unlimited;
    uint8_t saturated_with_error;

    if (pid->derivative_ready != 0U && pid->dt_sec > 0.0f) {
        derivative = (error - pid->previous_error) / pid->dt_sec;
    } else {
        pid->derivative_ready = 1U;
    }

    integral_candidate = pid->integral;
    if (integrate_enabled != 0U) {
        integral_candidate += pid->ki * error * pid->dt_sec;
    }
    integral_candidate = pid_core_limit(integral_candidate,
        pid->integral_min, pid->integral_max);

    output_unlimited = pid->kp * error +
        integral_candidate + pid->kd * derivative;
    saturated_with_error =
        ((output_unlimited > pid->output_max) && (error > 0.0f)) ||
        ((output_unlimited < pid->output_min) && (error < 0.0f));

    if ((integrate_enabled != 0U) &&
        (saturated_with_error == 0U)) {
        pid->integral = integral_candidate;
    }

    output_unlimited = pid->kp * error + pid->integral +
        pid->kd * derivative;
    output_unlimited = pid_core_limit(output_unlimited,
        pid->output_min, pid->output_max);
    pid->previous_error = error;
    return output_unlimited;
}

float PIDCore_Run(pid_core_t *pid, float target, float feedback)
{
    return PIDCore_RunConditional(pid, target, feedback, 1U);
}

void PIDCore_DecayIntegral(pid_core_t *pid, float retain_ratio)
{
    retain_ratio = pid_core_limit(retain_ratio, 0.0f, 1.0f);
    pid->integral *= retain_ratio;
    if ((pid->integral < 0.0001f) &&
        (pid->integral > -0.0001f)) {
        pid->integral = 0.0f;
    }
}

void PIDCore_SetOutputLimits(pid_core_t *pid, float output_min,
    float output_max)
{
    if (output_min <= output_max) {
        pid->output_min = output_min;
        pid->output_max = output_max;
    }
}

#include "stepper_axis.h"

#include "step_motor.h"

#define STEPPER_AXIS_DEG_PER_PULSE \
    (360.0f / (STEPPER_AXIS_STEPS_PER_REV * STEPPER_AXIS_GEAR_RATIO))
#define STEPPER_AXIS_POS_LIMIT_PULSES \
    ((int32_t)(STEPPER_AXIS_MAX_ANGLE_DEG / \
        STEPPER_AXIS_DEG_PER_PULSE + 0.5f))
#define STEPPER_AXIS_NEG_LIMIT_PULSES \
    (-((int32_t)((-STEPPER_AXIS_MIN_ANGLE_DEG) / \
        STEPPER_AXIS_DEG_PER_PULSE + 0.5f)))

/* Estimated angle from emitted pulses; there is no encoder in this system. */
static volatile int32_t axis_pulse_count = 0;
static volatile int32_t axis_zero_pulse_count = 0;
static volatile float axis_angle_estimate_deg = 0.0f;

static float stepper_limit(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

void stepper_axis_init(void)
{
    axis_pulse_count = 0;
    axis_zero_pulse_count = 0;
    axis_angle_estimate_deg = 0.0f;
}

void stepper_axis_set_velocity(float rpm)
{
    rpm = stepper_limit(rpm,
        -STEPPER_AXIS_MAX_RPM, STEPPER_AXIS_MAX_RPM);
    if (stepper_axis_velocity_allowed(rpm) == 0U) {
        step_motor_stop(2U);
        return;
    }
    step_motor_set_velocity(rpm, 2U);
}

void stepper_axis_stop(void)
{
    step_motor_stop(2U);
}

float stepper_axis_get_angle_estimate(void)
{
    int32_t pulse_count;
    int32_t zero_count;
    float angle;

    __disable_irq();
    pulse_count = axis_pulse_count;
    zero_count = axis_zero_pulse_count;
    __enable_irq();

    angle = (float)(pulse_count - zero_count) *
        STEPPER_AXIS_DEG_PER_PULSE;
    axis_angle_estimate_deg = angle;
    return angle;
}

int32_t stepper_axis_get_pulse_count(void)
{
    int32_t pulse_count;

    __disable_irq();
    pulse_count = axis_pulse_count;
    __enable_irq();
    return pulse_count;
}

void stepper_axis_calibrate_zero(void)
{
    __disable_irq();
    axis_zero_pulse_count = axis_pulse_count;
    __enable_irq();
    axis_angle_estimate_deg = 0.0f;
}

uint8_t stepper_axis_velocity_allowed(float rpm)
{
#if (STEPPER_AXIS_SOFT_LIMIT_EN != 0U)
    int32_t relative_pulses = axis_pulse_count - axis_zero_pulse_count;

    if ((rpm > 0.0f) &&
        (relative_pulses >= STEPPER_AXIS_POS_LIMIT_PULSES)) {
        return 0U;
    }
    if ((rpm < 0.0f) &&
        (relative_pulses <= STEPPER_AXIS_NEG_LIMIT_PULSES)) {
        return 0U;
    }
#else
    (void)rpm;
#endif

    return 1U;
}

void stepper_axis_on_pulse(uint8_t direction)
{
    int32_t relative_pulses;

    if (direction != 0U) {
        axis_pulse_count++;
    } else {
        axis_pulse_count--;
    }

#if (STEPPER_AXIS_SOFT_LIMIT_EN != 0U)
    relative_pulses = axis_pulse_count - axis_zero_pulse_count;
    if ((relative_pulses >= STEPPER_AXIS_POS_LIMIT_PULSES) ||
        (relative_pulses <= STEPPER_AXIS_NEG_LIMIT_PULSES)) {
        step_motor_stop(2U);
    }
#else
    (void)relative_pulses;
#endif
}

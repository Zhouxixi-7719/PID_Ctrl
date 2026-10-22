#include "step_motor.h"
#include "Delay.h"
#include "ball_step_ctrl.h"
#include "stepper_axis.h"

#define STEP_MOTOR_STEPS_PER_REV    6400U
/* STEP_Motor2位置控制速度；越大转动越快，但过高可能丢步或啸叫。 */
#define STEP_MOTOR2_POSITION_RPM    35
#define STEP_MOTOR2_RAIL_GEAR_RATIO 1.0f
#define HORIZONTAL_MOTOR_ID             2U
#define VERTICAL_MOTOR_ID               1U
#define AIM_TARGET_X                    20.0f
#define HORIZONTAL_KV_RPM_PER_PIXEL     1.2f
#define VERTICAL_KV_RPM_PER_PIXEL       1.2f
#define HORIZONTAL_DEADBAND             5.0f
#define VERTICAL_DEADBAND               5.0f
#define HORIZONTAL_DIRECTION_INVERT     1U
#define VERTICAL_DIRECTION_INVERT       0U

static uint8_t step_direction_1 = 1U;
static uint8_t step_running_1 = 0U;
static uint8_t step_direction_2 = 1U;
static volatile uint8_t step_running_2 = 0U;
static volatile int32_t stepper2_position_pulses = 0;
static float stepper2_zero_offset_deg =
    STEP_MOTOR2_ZERO_OFFSET_DEFAULT_DEG;
static float stepper2_angle_integral = 0.0f;
static float stepper2_angle_last_error = 0.0f;
static uint8_t stepper2_angle_pid_ready = 0U;

static float step_motor2_limit_float(float value, float min, float max)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

void step_motor2_angle_pid_reset(void)
{
    stepper2_angle_integral = 0.0f;
    stepper2_angle_last_error = 0.0f;
    stepper2_angle_pid_ready = 0U;
}

void step_motor_init(void)
{
    step_direction_1 = 1U;
    step_running_1 = 0U;
    step_direction_2 = 1U;
    step_running_2 = 0U;
    stepper2_position_pulses = 0;
    stepper2_zero_offset_deg = STEP_MOTOR2_ZERO_OFFSET_DEFAULT_DEG;
    step_motor2_angle_pid_reset();

    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_RST2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_SLP2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DCY2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR2_PIN);

    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_RST1_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_SLP1_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DCY1_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR1_PIN);
    delay_ms(2);

    DL_Timer_enableInterrupt(DCC_100_PWM2_INST,
        DL_TIMER_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(DCC_100_PWM2_INST_INT_IRQN);
    NVIC_EnableIRQ(DCC_100_PWM2_INST_INT_IRQN);

    step_motor_stop(1U);
    step_motor_stop(2U);
}

void step_motor_dir_set(uint8_t direction, uint8_t stepper_id)
{
    if (stepper_id == 1U) {
        if (direction == 0U) {
            DL_GPIO_clearPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR1_PIN);
        } else {
            DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR1_PIN);
        }
        step_direction_1 = (direction != 0U) ? 1U : 0U;
    } else if (stepper_id == 2U) {
        if (direction == 0U) {
            DL_GPIO_clearPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR2_PIN);
        } else {
            DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR2_PIN);
        }
        step_direction_2 = (direction != 0U) ? 1U : 0U;
    }
}

void step_motor_stop(uint8_t stepper_id)
{
    if (stepper_id == 1U) {
        DL_Timer_stopCounter(DCC_100_PWM1_INST);
        DL_Timer_setTimerCount(DCC_100_PWM1_INST, 0U);
        DL_GPIO_clearPins(GPIO_DCC_100_PWM1_C0_PORT,
            GPIO_DCC_100_PWM1_C0_PIN);
        step_running_1 = 0U;
    } else if (stepper_id == 2U) {
        DL_Timer_stopCounter(DCC_100_PWM2_INST);
        DL_Timer_setTimerCount(DCC_100_PWM2_INST, 0U);
        DL_GPIO_clearPins(GPIO_DCC_100_PWM2_C0_PORT,
            GPIO_DCC_100_PWM2_C0_PIN);
        step_running_2 = 0U;
    }
}

void step_motor_start(uint8_t stepper_id)
{
    if (stepper_id == 1U && step_running_1 == 0U) {
        DL_Timer_startCounter(DCC_100_PWM1_INST);
        step_running_1 = 1U;
    } else if (stepper_id == 2U && step_running_2 == 0U) {
        DL_Timer_startCounter(DCC_100_PWM2_INST);
        step_running_2 = 1U;
    }
}

/* speed is RPM; STEP_MOTOR_STEPS_PER_REV is the configured pulse count. */
void step_set_speed(uint16_t speed, uint8_t stepper_id)
{
    uint32_t frequency_hz;
    uint32_t period_counts;
    uint32_t load_value;
    uint32_t current_count;

    if ((stepper_id != 1U && stepper_id != 2U) || speed == 0U) {
        return;
    }

    frequency_hz = ((uint32_t)speed * STEP_MOTOR_STEPS_PER_REV) / 60U;
    if (frequency_hz == 0U) {
        frequency_hz = 1U;
    }

    if (stepper_id == 1U) {
        period_counts = DCC_100_PWM1_INST_CLK_FREQ / frequency_hz;
    } else {
        period_counts = DCC_100_PWM2_INST_CLK_FREQ / frequency_hz;
    }
    if (period_counts < 2U) {
        period_counts = 2U;
    }
    load_value = period_counts - 1U;

    if (stepper_id == 1U) {
        current_count = DL_Timer_getTimerCount(DCC_100_PWM1_INST);
        DL_Timer_setLoadValue(DCC_100_PWM1_INST, load_value);
        DL_Timer_setCaptureCompareValue(DCC_100_PWM1_INST,
            period_counts / 2U, GPIO_DCC_100_PWM1_C0_IDX);
        if (current_count > load_value) {
            DL_Timer_setTimerCount(DCC_100_PWM1_INST, load_value);
        }
    } else {
        current_count = DL_Timer_getTimerCount(DCC_100_PWM2_INST);
        DL_Timer_setLoadValue(DCC_100_PWM2_INST, load_value);
        DL_Timer_setCaptureCompareValue(DCC_100_PWM2_INST,
            period_counts / 2U, GPIO_DCC_100_PWM2_C0_IDX);
        if (current_count > load_value) {
            DL_Timer_setTimerCount(DCC_100_PWM2_INST, load_value);
        }
    }
}

void step_motor_set_velocity(float rpm, uint8_t stepper_id)
{
    float abs_rpm;
    uint8_t direction;
    uint8_t current_direction;

    if (stepper_id == 1U) {
        current_direction = step_direction_1;
    } else if (stepper_id == 2U) {
        current_direction = step_direction_2;
    } else {
        return;
    }

    if ((stepper_id == 2U) &&
        (stepper_axis_velocity_allowed(rpm) == 0U)) {
        step_motor_stop(2U);
        return;
    }

    abs_rpm = (rpm >= 0.0f) ? rpm : -rpm;
    if (abs_rpm < 0.01f) {
        step_motor_stop(stepper_id);
        return;
    }

    if (abs_rpm < STEP_MOTOR_MIN_RPM) {
        abs_rpm = STEP_MOTOR_MIN_RPM;
    } else if (abs_rpm > STEP_MOTOR_MAX_RPM) {
        abs_rpm = STEP_MOTOR_MAX_RPM;
    }

    direction = (rpm >= 0.0f) ? 1U : 0U;
    if (direction != current_direction) {
        step_motor_stop(stepper_id);
        step_motor_dir_set(direction, stepper_id);
    }

    step_set_speed((uint16_t)(abs_rpm + 0.5f), stepper_id);
    step_motor_start(stepper_id);
}

void step_motor2_set_target_angle(float angle_deg)
{
    float current_angle;
    float angle_error;
    float derivative;
    float integral_candidate;
    float rpm_unlimited;
    float rpm_command;
    float abs_error;

    current_angle = step_motor2_get_current_angle();
    angle_error = angle_deg - current_angle;
    abs_error = (angle_error >= 0.0f) ? angle_error : -angle_error;

    if (abs_error <= STEP_MOTOR2_ANGLE_DEADBAND_DEG) {
        step_motor_set_velocity(0.0f, 2U);
        step_motor2_angle_pid_reset();
        return;
    }

    if (stepper2_angle_pid_ready == 0U) {
        stepper2_angle_last_error = angle_error;
        derivative = 0.0f;
        stepper2_angle_pid_ready = 1U;
    } else {
        derivative = (angle_error - stepper2_angle_last_error) /
            STEP_MOTOR2_ANGLE_PID_DT_SEC;
        stepper2_angle_last_error = angle_error;
    }

    integral_candidate = step_motor2_limit_float(
        stepper2_angle_integral +
        angle_error * STEP_MOTOR2_ANGLE_PID_DT_SEC,
        -STEP_MOTOR2_ANGLE_INTEGRAL_LIMIT,
        STEP_MOTOR2_ANGLE_INTEGRAL_LIMIT);
    rpm_unlimited = STEP_MOTOR2_ANGLE_DEFAULT_KP * angle_error +
        STEP_MOTOR2_ANGLE_DEFAULT_KI * integral_candidate +
        STEP_MOTOR2_ANGLE_DEFAULT_KD * derivative;

    if (!(((rpm_unlimited > STEP_MOTOR2_ANGLE_MAX_RPM) &&
            (angle_error > 0.0f)) ||
          ((rpm_unlimited < -STEP_MOTOR2_ANGLE_MAX_RPM) &&
            (angle_error < 0.0f)))) {
        stepper2_angle_integral = integral_candidate;
    }

    rpm_command = step_motor2_limit_float(rpm_unlimited,
        -STEP_MOTOR2_ANGLE_MAX_RPM, STEP_MOTOR2_ANGLE_MAX_RPM);

    step_motor_set_velocity(rpm_command, 2U);
}

void step_motor2_stop_position(void)
{
    __disable_irq();
    step_motor_stop(2U);
    __enable_irq();
    step_motor2_angle_pid_reset();
}

static float step_motor2_get_raw_angle(void)
{
    int32_t position_pulses;

    __disable_irq();
    position_pulses = stepper2_position_pulses;
    __enable_irq();

    return (float)position_pulses * 360.0f /
        ((float)STEP_MOTOR_STEPS_PER_REV * STEP_MOTOR2_RAIL_GEAR_RATIO);
}

void step_motor2_calibrate_zero(void)
{
    stepper2_zero_offset_deg = step_motor2_get_raw_angle();
    step_motor2_angle_pid_reset();
}

float step_motor2_get_current_angle(void)
{
    return step_motor2_get_raw_angle() - stepper2_zero_offset_deg;
}

float step_motor2_get_zero_offset(void)
{
    return stepper2_zero_offset_deg;
}

void DCC_100_PWM2_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(DCC_100_PWM2_INST) !=
        DL_TIMER_IIDX_LOAD) {
        return;
    }

    DL_Timer_clearInterruptStatus(DCC_100_PWM2_INST,
        DL_TIMER_INTERRUPT_LOAD_EVENT);

    if (step_running_2 == 0U) {
        return;
    }

    if (step_direction_2 != 0U) {
        stepper2_position_pulses++;
    } else {
        stepper2_position_pulses--;
    }
    stepper_axis_on_pulse(step_direction_2);
}

static void apply_axis_error(float error, uint8_t motor_id,
    float deadband, float kv_rpm_per_pixel, uint8_t direction_invert)
{
    float abs_error = (error >= 0.0f) ? error : -error;
    float rpm;

    if (abs_error <= deadband) {
        step_motor_set_velocity(0.0f, motor_id);
        return;
    }

    rpm = error * kv_rpm_per_pixel;
    if (direction_invert != 0U) {
        rpm = -rpm;
    }

    step_motor_set_velocity(rpm, motor_id);
}

void step_motor_apply_horizontal_position(uint16_t x)
{
    /* 电机2由小球滑轨控制时，不允许视觉瞄准接口抢占它。 */
    if (BallStepCtrl_IsEnabled() != 0U) {
        return;
    }

    apply_axis_error(AIM_TARGET_X - (float)x, HORIZONTAL_MOTOR_ID,
        HORIZONTAL_DEADBAND, HORIZONTAL_KV_RPM_PER_PIXEL,
        HORIZONTAL_DIRECTION_INVERT);
}

void step_motor_apply_error(int16_t err_x, int16_t err_y)
{
    /* 电机2与小球滑轨共用，球控使能时只允许电机1执行瞄准误差。 */
    if (BallStepCtrl_IsEnabled() == 0U) {
        apply_axis_error((float)err_x, HORIZONTAL_MOTOR_ID,
            HORIZONTAL_DEADBAND, HORIZONTAL_KV_RPM_PER_PIXEL,
            HORIZONTAL_DIRECTION_INVERT);
    }

    apply_axis_error((float)err_y, VERTICAL_MOTOR_ID, VERTICAL_DEADBAND,
        VERTICAL_KV_RPM_PER_PIXEL, VERTICAL_DIRECTION_INVERT);
}

void step_motor_stop_aim(void)
{
    if (BallStepCtrl_IsEnabled() == 0U) {
        step_motor_set_velocity(0.0f, HORIZONTAL_MOTOR_ID);
    }
    step_motor_set_velocity(0.0f, VERTICAL_MOTOR_ID);
}

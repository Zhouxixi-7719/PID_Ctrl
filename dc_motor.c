#include "dc_motor.h"
#include "ti_msp_dl_config.h"

static int16_t dc_motor_pwm_l = 0;
static int16_t dc_motor_pwm_r = 0;

static int16_t dc_motor_limit_signed(int16_t pwm)
{
    if (pwm > DC_MOTOR_PWM_MAX) {
        return DC_MOTOR_PWM_MAX;
    }

    if (pwm < -DC_MOTOR_PWM_MAX) {
        return -DC_MOTOR_PWM_MAX;
    }

    return pwm;
}

static uint16_t dc_motor_abs_limit(int16_t pwm)
{
    int32_t value = pwm;

    if (value < 0) {
        value = -value;
    }

    if (value > DC_MOTOR_PWM_MAX) {
        value = DC_MOTOR_PWM_MAX;
    }

    return (uint16_t)value;
}

void dc_motor_init(void)
{
    dc_motor_stop();
    DL_TimerG_startCounter(DC_MOTOR_PWM_INST);
}

void dc_motor_set_speed_l(int16_t pwm)
{
    int16_t logical_pwm;

    pwm = dc_motor_limit_signed(pwm);
    dc_motor_pwm_l = pwm;
    logical_pwm = pwm;
    pwm = (int16_t)(logical_pwm * DC_MOTOR_DIR_L);

    uint16_t duty = dc_motor_abs_limit(pwm);

    if (pwm > 0) {
        DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
    } else if (pwm < 0) {
        DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT,
            DC_MOTOR_AIN1_PIN | DC_MOTOR_AIN2_PIN);
    }

    DL_TimerG_setCaptureCompareValue(DC_MOTOR_PWM_INST, duty,
        GPIO_DC_MOTOR_PWM_C1_IDX);
}

void dc_motor_set_speed_r(int16_t pwm)
{
    int16_t logical_pwm;

    pwm = dc_motor_limit_signed(pwm);
    dc_motor_pwm_r = pwm;
    logical_pwm = pwm;
    pwm = (int16_t)(logical_pwm * DC_MOTOR_DIR_R);

    uint16_t duty = dc_motor_abs_limit(pwm);

    if (pwm > 0) {
        DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
    } else if (pwm < 0) {
        DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT,
            DC_MOTOR_BIN1_PIN | DC_MOTOR_BIN2_PIN);
    }

    DL_TimerG_setCaptureCompareValue(DC_MOTOR_PWM_INST, duty,
        GPIO_DC_MOTOR_PWM_C0_IDX);
}

int16_t dc_motor_get_pwm_l(void)
{
    return dc_motor_pwm_l;
}

int16_t dc_motor_get_pwm_r(void)
{
    return dc_motor_pwm_r;
}

void dc_motor_stop(void)
{
    dc_motor_pwm_l = 0;
    dc_motor_pwm_r = 0;

    DL_TimerG_setCaptureCompareValue(DC_MOTOR_PWM_INST, 0,
        GPIO_DC_MOTOR_PWM_C1_IDX);
    DL_TimerG_setCaptureCompareValue(DC_MOTOR_PWM_INST, 0,
        GPIO_DC_MOTOR_PWM_C0_IDX);
    DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT,
        DC_MOTOR_AIN1_PIN | DC_MOTOR_AIN2_PIN);
    DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT,
        DC_MOTOR_BIN1_PIN | DC_MOTOR_BIN2_PIN);
}

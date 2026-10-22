#include "speed_ctrl.h"

#include "dc_encoder.h"
#include "dc_motor.h"
#include "gray_sensor.h"
#include "pid.h"
#include "ti_msp_dl_config.h"

volatile float g_target_speed = 0.0f;

static pid_ctrl_t speed_pid_l;
static pid_ctrl_t speed_pid_r;
static volatile uint8_t speed_ctrl_enabled = 0U;
static volatile float speed_feedback_l = 0.0f;
static volatile float speed_feedback_r = 0.0f;
static volatile float speed_target_l = 0.0f;
static volatile float speed_target_r = 0.0f;
static volatile float speed_run_target = SPEED_CTRL_DEFAULT_TARGET_CM_S;
static volatile float speed_profile_target = 0.0f;
static volatile float speed_profile_speed = 0.0f;
static volatile float speed_profile_accel = 0.0f;
static volatile uint8_t speed_smooth_stopping = 0U;
static volatile int16_t speed_pwm_l = 0;
static volatile int16_t speed_pwm_r = 0;
static float speed_previous_target_l = 0.0f;
static float speed_previous_target_r = 0.0f;
static track_state_t speed_previous_track_state = TRACK_STATE_LINE_FOLLOW;

static float SpeedCtrl_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float SpeedCtrl_LimitFloat(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void SpeedCtrl_UpdateProfile(void)
{
    const float dt_sec = (float)SPEED_CTRL_PERIOD_MS / 1000.0f;
    float error = speed_profile_target - speed_profile_speed;
    float accel_limit;
    float target_accel;
    float accel_step;
    float next_speed;

    if (SpeedCtrl_Abs(error) <= SPEED_CTRL_PROFILE_TARGET_EPS_CM_S) {
        speed_profile_speed = speed_profile_target;
        speed_profile_accel = 0.0f;
        g_target_speed = speed_profile_speed;
        return;
    }

    accel_limit = ((SpeedCtrl_Abs(speed_profile_target) <
        SpeedCtrl_Abs(speed_profile_speed)) ||
        ((speed_profile_target * speed_profile_speed) < 0.0f)) ?
        SPEED_CTRL_PROFILE_DECEL_CM_S2 :
        SPEED_CTRL_PROFILE_ACCEL_CM_S2;
    target_accel = SpeedCtrl_LimitFloat(
        error / SPEED_CTRL_PROFILE_RESPONSE_S, accel_limit);
    accel_step = SPEED_CTRL_PROFILE_JERK_CM_S3 * dt_sec;
    speed_profile_accel += SpeedCtrl_LimitFloat(
        target_accel - speed_profile_accel, accel_step);
    next_speed = speed_profile_speed + speed_profile_accel * dt_sec;

    if (((error >= 0.0f) && (next_speed >= speed_profile_target)) ||
        ((error < 0.0f) && (next_speed <= speed_profile_target))) {
        next_speed = speed_profile_target;
        speed_profile_accel = 0.0f;
    }
    speed_profile_speed = next_speed;
    g_target_speed = speed_profile_speed;
}

static uint8_t SpeedCtrl_TargetSignChanged(float previous, float current)
{
    return (((previous > 0.0f) && (current < 0.0f)) ||
        ((previous < 0.0f) && (current > 0.0f))) ? 1U : 0U;
}

/*
 * 是否处于"原地转向"类状态——这类状态绕开速度环，直接给固�?PWM�? *
 * 丢线直行必须归到非转向类：那时车要正常前进，差速由航向环给出，
 * 结构和普通循迹完全一样，只是误差来源从灰度换成了航向角�? */
static uint8_t SpeedCtrl_IsTurnMode(track_state_t state)
{
    return ((state != TRACK_STATE_LINE_FOLLOW) &&
        (state != TRACK_STATE_LOST_STRAIGHT)) ? 1U : 0U;
}

static int16_t SpeedCtrl_FloatToPwm(float output)
{
    if (output >= 0.0f) {
        return (int16_t)(output + 0.5f);
    }

    return (int16_t)(output - 0.5f);
}

static int16_t SpeedCtrl_LimitPwm(int16_t pwm, int16_t limit)
{
    if (pwm > limit) {
        return limit;
    }

    if (pwm < -limit) {
        return -limit;
    }

    return pwm;
}


void SpeedCtrl_Init(void)
{
    PID_Init(&speed_pid_l, SPEED_CTRL_DEFAULT_KP, SPEED_CTRL_DEFAULT_KI,
        SPEED_CTRL_DEFAULT_KD, -SPEED_CTRL_INTEGRAL_LIMIT,
        SPEED_CTRL_INTEGRAL_LIMIT, -(float)DC_MOTOR_PWM_MAX,
        (float)DC_MOTOR_PWM_MAX);
    PID_Init(&speed_pid_r, SPEED_CTRL_DEFAULT_KP, SPEED_CTRL_DEFAULT_KI,
        SPEED_CTRL_DEFAULT_KD, -SPEED_CTRL_INTEGRAL_LIMIT,
        SPEED_CTRL_INTEGRAL_LIMIT, -(float)DC_MOTOR_PWM_MAX,
        (float)DC_MOTOR_PWM_MAX);

    SpeedCtrl_Stop();
}

void SpeedCtrl_Start(float target_cm_s)
{
    __disable_irq();
    g_target_speed = 0.0f;
    speed_run_target = target_cm_s;
    speed_profile_target = target_cm_s;
    speed_profile_speed = 0.0f;
    speed_profile_accel = 0.0f;
    speed_smooth_stopping = 0U;
    PID_Reset(&speed_pid_l);
    PID_Reset(&speed_pid_r);
    speed_target_l = 0.0f;
    speed_target_r = 0.0f;
    speed_pwm_l = 0;
    speed_pwm_r = 0;
    speed_previous_target_l = 0.0f;
    speed_previous_target_r = 0.0f;
    speed_previous_track_state = TRACK_STATE_LINE_FOLLOW;
    speed_ctrl_enabled = 1U;
    __enable_irq();
}

void SpeedCtrl_Stop(void)
{
    __disable_irq();
    g_target_speed = 0.0f;
    speed_profile_target = 0.0f;
    speed_profile_speed = 0.0f;
    speed_profile_accel = 0.0f;
    speed_smooth_stopping = 0U;
    speed_ctrl_enabled = 0U;
    PID_Reset(&speed_pid_l);
    PID_Reset(&speed_pid_r);
    speed_target_l = 0.0f;
    speed_target_r = 0.0f;
    speed_pwm_l = 0;
    speed_pwm_r = 0;
    speed_previous_target_l = 0.0f;
    speed_previous_target_r = 0.0f;
    speed_previous_track_state = TRACK_STATE_LINE_FOLLOW;
    __enable_irq();

    dc_motor_stop();
}

void SpeedCtrl_RequestSmoothStop(void)
{
    __disable_irq();
    if (speed_ctrl_enabled != 0U) {
        speed_profile_target = 0.0f;
        speed_smooth_stopping = 1U;
    }
    __enable_irq();
}

void SpeedCtrl_SetRunSpeed(float target_cm_s)
{
    __disable_irq();
    speed_run_target = target_cm_s;
    if (speed_smooth_stopping == 0U) {
        speed_profile_target = target_cm_s;
    }
    __enable_irq();
}

float SpeedCtrl_GetRunSpeed(void)
{
    return speed_run_target;
}

void SpeedCtrl_SetPid(float kp, float ki, float kd)
{
    __disable_irq();
    PID_SetParams(&speed_pid_l, kp, ki, kd);
    PID_SetParams(&speed_pid_r, kp, ki, kd);
    __enable_irq();
}

void SpeedCtrl_Task10ms(void)
{
    float output_l;
    float output_r;
    float target_l;
    float target_r;
    int16_t pwm_l;
    int16_t pwm_r;
    int16_t turn_pwm;
    track_state_t track_state;
    uint8_t turn_mode;

    speed_feedback_l = Encoder_GetSpeed_L();
    speed_feedback_r = Encoder_GetSpeed_R();

    if (speed_ctrl_enabled == 0U) {
        return;
    }

    SpeedCtrl_UpdateProfile();

    if ((speed_smooth_stopping != 0U) &&
        (SpeedCtrl_Abs(speed_profile_speed) <=
            SPEED_CTRL_STOP_TARGET_EPS_CM_S) &&
        (SpeedCtrl_Abs(speed_feedback_l) <= SPEED_CTRL_STOP_SPEED_EPS_CM_S) &&
        (SpeedCtrl_Abs(speed_feedback_r) <= SPEED_CTRL_STOP_SPEED_EPS_CM_S)) {
        speed_target_l = 0.0f;
        speed_target_r = 0.0f;
        speed_pwm_l = 0;
        speed_pwm_r = 0;
        PID_Reset(&speed_pid_l);
        PID_Reset(&speed_pid_r);
        dc_motor_stop();
        return;
    }

    Track_ApplyTarget(g_target_speed, &target_l, &target_r);
    track_state = Track_GetState();
    turn_mode = SpeedCtrl_IsTurnMode(track_state);

    /*
     * 只在"进出原地转向"这个边界上复位，而不是任何状态变化都复位�?     * 普通循迹和丢线直行之间来回切换时车一直在正常前进，速度是连续的�?     * 复位反而会让速度抖一下�?     */
    if (turn_mode != SpeedCtrl_IsTurnMode(speed_previous_track_state)) {
        PID_Reset(&speed_pid_l);
        PID_Reset(&speed_pid_r);
        /*
         * 一并清测速窗口。原地转弯时左右轮反向，窗口里会残留反向
         * 计数，不清的话出弯后头几十毫秒的速度反馈是错的�?         */
        Encoder_ResetSpeedWindow();
    }
    speed_previous_track_state = track_state;

    /*
     * 必须�?SpeedCtrl_IsTurnMode() 而不是直接比�?LINE_FOLLOW�?     * 否则 TRACK_STATE_LOST_STRAIGHT 会落进下面的固定 PWM 分支�?     * 而且既不�?STOP 也不�?TURNING_RIGHT，最终掉进末尾的 else�?     * 执行"左轮反转、右轮正�?——丢线直行就变成了原地打转�?     */
    if (turn_mode != 0U) {
        speed_target_l = 0.0f;
        speed_target_r = 0.0f;

        if ((track_state == TRACK_STATE_TURN_STOP_LEFT) ||
            (track_state == TRACK_STATE_TURN_STOP_RIGHT)) {
            pwm_l = 0;
            pwm_r = 0;
        } else if (track_state == TRACK_STATE_TURNING_RIGHT) {
            turn_pwm = Track_GetTurnPwm();
            pwm_l = turn_pwm;
            pwm_r = -turn_pwm;
        } else {
            turn_pwm = Track_GetTurnPwm();
            pwm_l = -turn_pwm;
            pwm_r = turn_pwm;
        }

        pwm_l = SpeedCtrl_LimitPwm(pwm_l, SPEED_CTRL_TURN_PWM_LIMIT);
        pwm_r = SpeedCtrl_LimitPwm(pwm_r, SPEED_CTRL_TURN_PWM_LIMIT);
        speed_pwm_l = pwm_l;
        speed_pwm_r = pwm_r;
        dc_motor_set_speed_l(pwm_l);
        dc_motor_set_speed_r(pwm_r);
        return;
    }

    speed_target_l = target_l;
    speed_target_r = target_r;

    if (SpeedCtrl_TargetSignChanged(speed_previous_target_l, target_l) != 0U) {
        PID_Reset(&speed_pid_l);
    }
    if (SpeedCtrl_TargetSignChanged(speed_previous_target_r, target_r) != 0U) {
        PID_Reset(&speed_pid_r);
    }
    speed_previous_target_l = target_l;
    speed_previous_target_r = target_r;

    output_l = PID_Calc(&speed_pid_l, target_l, speed_feedback_l);
    output_r = PID_Calc(&speed_pid_r, target_r, speed_feedback_r);
    pwm_l = SpeedCtrl_FloatToPwm(output_l);
    pwm_r = SpeedCtrl_FloatToPwm(output_r);

    speed_pwm_l = pwm_l;
    speed_pwm_r = pwm_r;
    dc_motor_set_speed_l(pwm_l);
    dc_motor_set_speed_r(pwm_r);
}

uint8_t SpeedCtrl_IsEnabled(void)
{
    return speed_ctrl_enabled;
}

uint8_t SpeedCtrl_IsAtRest(void)
{
    return ((SpeedCtrl_Abs(speed_profile_speed) <=
            SPEED_CTRL_STOP_TARGET_EPS_CM_S) &&
        (SpeedCtrl_Abs(speed_feedback_l) <= SPEED_CTRL_STOP_SPEED_EPS_CM_S) &&
        (SpeedCtrl_Abs(speed_feedback_r) <= SPEED_CTRL_STOP_SPEED_EPS_CM_S)) ?
        1U : 0U;
}

float SpeedCtrl_GetTarget(void)
{
    return g_target_speed;
}

float SpeedCtrl_GetTargetL(void)
{
    return speed_target_l;
}

float SpeedCtrl_GetTargetR(void)
{
    return speed_target_r;
}

float SpeedCtrl_GetSpeedL(void)
{
    return speed_feedback_l;
}

float SpeedCtrl_GetSpeedR(void)
{
    return speed_feedback_r;
}

float SpeedCtrl_GetProfileAcceleration(void)
{
    return speed_profile_accel;
}


int16_t SpeedCtrl_GetPwmL(void)
{
    return speed_pwm_l;
}

int16_t SpeedCtrl_GetPwmR(void)
{
    return speed_pwm_r;
}

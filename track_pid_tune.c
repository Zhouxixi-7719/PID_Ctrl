#include "track_pid_tune.h"

#include "gray_sensor.h"
#include "ball_step_ctrl.h"
#include "Key.h"
#include "speed_ctrl.h"
#include "step_motor.h"

static uint8_t track_pid_tune_active = 0U;
static track_pid_tune_item_t track_pid_tune_item =
    TRACK_PID_TUNE_ITEM_KP;
static track_pid_tune_param_t track_pid_tune_selected =
    TRACK_PID_TUNE_PARAM_KP;

static float track_pid_tune_kp = TRACK_TURN_KP;
static float track_pid_tune_ki = TRACK_TURN_KI;
static float track_pid_tune_kd = TRACK_TURN_KD;

static float TrackPidTune_LimitFloat(float value, float min, float max)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

static int16_t TrackPidTune_LimitInt16(int16_t value, int16_t min,
    int16_t max)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

static void TrackPidTune_ApplyPid(void)
{
    Track_SetPid(track_pid_tune_kp, track_pid_tune_ki, track_pid_tune_kd);
}

static void TrackPidTune_SelectPidItem(void)
{
    if (track_pid_tune_item == TRACK_PID_TUNE_ITEM_KI) {
        track_pid_tune_selected = TRACK_PID_TUNE_PARAM_KI;
    } else if (track_pid_tune_item == TRACK_PID_TUNE_ITEM_KD) {
        track_pid_tune_selected = TRACK_PID_TUNE_PARAM_KD;
    } else {
        track_pid_tune_selected = TRACK_PID_TUNE_PARAM_KP;
    }
}

static void TrackPidTune_AdjustCurrent(int8_t dir)
{
    float run_speed;
    int16_t turn_pwm;
    int16_t turn_ms;
    int16_t ball_reference;

    switch (track_pid_tune_item) {
        case TRACK_PID_TUNE_ITEM_KP:
            track_pid_tune_kp = TrackPidTune_LimitFloat(
                track_pid_tune_kp + (float)dir * TRACK_PID_KP_STEP,
                TRACK_PID_KP_MIN, TRACK_PID_KP_MAX);
            TrackPidTune_ApplyPid();
            break;

        case TRACK_PID_TUNE_ITEM_KI:
            track_pid_tune_ki = TrackPidTune_LimitFloat(
                track_pid_tune_ki + (float)dir * TRACK_PID_KI_STEP,
                TRACK_PID_KI_MIN, TRACK_PID_KI_MAX);
            TrackPidTune_ApplyPid();
            break;

        case TRACK_PID_TUNE_ITEM_KD:
            track_pid_tune_kd = TrackPidTune_LimitFloat(
                track_pid_tune_kd + (float)dir * TRACK_PID_KD_STEP,
                TRACK_PID_KD_MIN, TRACK_PID_KD_MAX);
            TrackPidTune_ApplyPid();
            break;

        case TRACK_PID_TUNE_ITEM_SPEED:
            run_speed = SpeedCtrl_GetRunSpeed() +
                (float)dir * TRACK_TUNE_SPEED_STEP_CM_S;
            run_speed = TrackPidTune_LimitFloat(run_speed,
                TRACK_TUNE_SPEED_MIN_CM_S, TRACK_TUNE_SPEED_MAX_CM_S);
            SpeedCtrl_SetRunSpeed(run_speed);
            break;

        case TRACK_PID_TUNE_ITEM_TURN_PWM:
            turn_pwm = (int16_t)(Track_GetTurnPwm() +
                dir * TRACK_TUNE_TURN_PWM_STEP);
            turn_pwm = TrackPidTune_LimitInt16(turn_pwm,
                TRACK_TUNE_TURN_PWM_MIN, TRACK_TUNE_TURN_PWM_MAX);
            Track_SetTurnPwm(turn_pwm);
            break;

        case TRACK_PID_TUNE_ITEM_TURN_STOP_MS:
            turn_ms = (int16_t)Track_GetTurnStopMs() +
                dir * TRACK_TUNE_TURN_MS_STEP;
            turn_ms = TrackPidTune_LimitInt16(turn_ms,
                TRACK_TUNE_TURN_STOP_MS_MIN, TRACK_TUNE_TURN_STOP_MS_MAX);
            Track_SetTurnStopMs((uint16_t)turn_ms);
            break;

        case TRACK_PID_TUNE_ITEM_TURN_LOCK_MS:
            turn_ms = (int16_t)Track_GetTurnLockMs() +
                dir * TRACK_TUNE_TURN_MS_STEP;
            turn_ms = TrackPidTune_LimitInt16(turn_ms,
                TRACK_TUNE_TURN_LOCK_MS_MIN, TRACK_TUNE_TURN_LOCK_MS_MAX);
            Track_SetTurnLockMs((uint16_t)turn_ms);
            break;

        case TRACK_PID_TUNE_ITEM_BALL_REF:
            ball_reference = BallStepCtrl_GetReference() + dir;
            BallStepCtrl_SetReference(ball_reference);
            break;

        case TRACK_PID_TUNE_ITEM_BALL_KP:
            BallStepCtrl_SetKp(BallStepCtrl_GetKp() +
                (float)dir * 0.01f);
            break;

        case TRACK_PID_TUNE_ITEM_BALL_KI:
            BallStepCtrl_SetKi(BallStepCtrl_GetKi() +
                (float)dir * TRACK_PID_KI_STEP);
            break;

        case TRACK_PID_TUNE_ITEM_BALL_KD:
            BallStepCtrl_SetKd(BallStepCtrl_GetKd() +
                (float)dir * TRACK_PID_KD_STEP);
            break;

        case TRACK_PID_TUNE_ITEM_BALL_ENABLE:
            BallStepCtrl_SetEnabled((dir > 0) ? 1U : 0U);
            break;

        case TRACK_PID_TUNE_ITEM_BALL_ZERO:
            /* 只允许 K3:+ 执行校准，校准前停止球控制。 */
            if (dir > 0) {
                BallStepCtrl_SetEnabled(0U);
                step_motor2_calibrate_zero();
            }
            break;

        default:
            break;
    }
}

void TrackPidTune_Init(void)
{
    track_pid_tune_active = 0U;
    track_pid_tune_item = TRACK_PID_TUNE_ITEM_KP;
    track_pid_tune_selected = TRACK_PID_TUNE_PARAM_KP;
    track_pid_tune_kp = TRACK_TURN_KP;
    track_pid_tune_ki = TRACK_TURN_KI;
    track_pid_tune_kd = TRACK_TURN_KD;
    TrackPidTune_ApplyPid();
}

void TrackPidTune_ProcessKey(uint8_t key)
{
    if (key == 0U) {
        return;
    }

    if (track_pid_tune_active == 0U) {                                  //短按进入菜单，置标志位之后while循环每次就
        if (key == KEY_EVENT_KEY1_SHORT) {                              //只会进入这一段调整参数的函数中
            track_pid_tune_active = 1U;
            track_pid_tune_item = TRACK_PID_TUNE_ITEM_KP;
            track_pid_tune_selected = TRACK_PID_TUNE_PARAM_KP;
        }
        return;
    }

    if (key == KEY_EVENT_KEY1_LONG) {                                   //长按退出菜单
        TrackPidTune_Close();
    } else if (key == KEY_EVENT_KEY1_SHORT) {
        track_pid_tune_item = (track_pid_tune_item_t)
            ((track_pid_tune_item + 1U) % TRACK_PID_TUNE_ITEM_COUNT);
        TrackPidTune_SelectPidItem();
    } else if (key == KEY_EVENT_KEY2_SHORT) {
        TrackPidTune_AdjustCurrent(-1);
    } else if (key == KEY_EVENT_KEY3_SHORT) {
        TrackPidTune_AdjustCurrent(1);
    }
}

void TrackPidTune_Close(void)
{
    track_pid_tune_active = 0U;
}

uint8_t TrackPidTune_IsDisplayActive(void)
{
    return track_pid_tune_active;
}

track_pid_tune_param_t TrackPidTune_GetSelected(void)
{
    return track_pid_tune_selected;
}

track_pid_tune_item_t TrackPidTune_GetMenuItem(void)
{
    return track_pid_tune_item;
}

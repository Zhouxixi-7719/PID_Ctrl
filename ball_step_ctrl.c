#include "ball_step_ctrl.h"

#include "step_motor.h"

#define BALL_STEP_INPUT_POSITION      0U
#define BALL_STEP_INPUT_DIRECT_ERROR  1U

static volatile uint8_t ball_step_enabled = BALL_STEP_DEFAULT_ENABLED;
static volatile uint8_t ball_step_position_valid = 0U;
static volatile uint8_t ball_step_filter_ready = 0U;
static volatile uint8_t ball_step_input_mode = BALL_STEP_INPUT_POSITION;
static volatile int16_t ball_step_reference = BALL_STEP_DEFAULT_REFERENCE;
static volatile float ball_step_kp = BALL_STEP_DEFAULT_KP;
static volatile float ball_step_ki = BALL_STEP_DEFAULT_KI;
static volatile float ball_step_kd = BALL_STEP_DEFAULT_KD;
static volatile uint16_t ball_step_raw_position = 0U;
static volatile float ball_step_filtered_position = 0.0f;
static volatile float ball_step_direct_error = 0.0f;
static volatile float ball_step_error = 0.0f;
static volatile float ball_step_integral = 0.0f;
static volatile float ball_step_last_error = 0.0f;
static volatile float ball_step_derivative = 0.0f;
static volatile uint8_t ball_step_pid_ready = 0U;
static volatile float ball_step_velocity = 0.0f;
static volatile float ball_step_predicted_error = 0.0f;
static volatile float ball_step_braking_angle = 0.0f;
static volatile uint8_t ball_step_velocity_ready = 0U;
static volatile uint8_t ball_step_kalman_ready = 0U;
static volatile float ball_step_kalman_position = 0.0f;
static volatile float ball_step_kalman_velocity = 0.0f;
static volatile float ball_step_kalman_p00 = 0.0f;
static volatile float ball_step_kalman_p01 = 0.0f;
static volatile float ball_step_kalman_p10 = 0.0f;
static volatile float ball_step_kalman_p11 = 0.0f;
static volatile float ball_step_output_angle = 0.0f;
static volatile uint32_t ball_step_last_position_ms = 0U;
static volatile uint8_t ball_step_settling = 0U;
static volatile uint16_t ball_step_settle_confirm_count = 0U;
#if BALL_STEP_ADAPTIVE_OUTPUT_ENABLE
static float ball_step_adaptive_scale = 1.0f;
static float ball_step_adaptive_peak_error = 0.0f;
static uint8_t ball_step_adaptive_error_sign = 0U;
#endif

static float BallStepCtrl_LimitFloat(float value, float min, float max)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

static int16_t BallStepCtrl_LimitReference(int16_t reference)
{
    if (reference < 0) {
        return 0;
    }

    if (reference > 999) {
        return 999;
    }

    return reference;
}

static void BallStepCtrl_ResetPidState(void)
{
    ball_step_integral = 0.0f;
    ball_step_last_error = 0.0f;
    ball_step_derivative = 0.0f;
    ball_step_pid_ready = 0U;
}

#if BALL_STEP_ADAPTIVE_OUTPUT_ENABLE
static void BallStepCtrl_ResetAdaptiveState(void)
{
    ball_step_adaptive_scale = 1.0f;
    ball_step_adaptive_peak_error = 0.0f;
    ball_step_adaptive_error_sign = 0;
}

static void BallStepCtrl_UpdateAdaptiveScale(float position_error)
{
    float abs_error;
    float target_scale;
    uint8_t current_sign;

    abs_error = (position_error >= 0.0f) ? position_error :
        -position_error;
    current_sign = 0U;
    if (position_error > BALL_STEP_ADAPTIVE_ZERO_CROSS_ERROR) {
        current_sign = 1U;
    } else if (position_error < -BALL_STEP_ADAPTIVE_ZERO_CROSS_ERROR) {
        current_sign = 2U;
    }

    if (abs_error > ball_step_adaptive_peak_error) {
        ball_step_adaptive_peak_error = abs_error;
    }

    if (current_sign == 0) {
        return;
    }

    if ((ball_step_adaptive_error_sign != 0) &&
        (current_sign != ball_step_adaptive_error_sign)) {
        target_scale = ball_step_adaptive_peak_error /
            BALL_STEP_ADAPTIVE_ERROR_RANGE;
        target_scale = BallStepCtrl_LimitFloat(
            target_scale,
            BALL_STEP_ADAPTIVE_MIN_SCALE,
            1.0f);
        ball_step_adaptive_scale +=
            BALL_STEP_ADAPTIVE_SCALE_ALPHA *
            (target_scale - ball_step_adaptive_scale);
        ball_step_adaptive_peak_error = abs_error;
    }

    ball_step_adaptive_error_sign = current_sign;
}
#endif

static void BallStepCtrl_ResetKalmanState(void)
{
    ball_step_kalman_ready = 0U;
    ball_step_kalman_position = 0.0f;
    ball_step_kalman_velocity = 0.0f;
    ball_step_kalman_p00 = 0.0f;
    ball_step_kalman_p01 = 0.0f;
    ball_step_kalman_p10 = 0.0f;
    ball_step_kalman_p11 = 0.0f;
}

static void BallStepCtrl_InitKalman(float position)
{
    ball_step_kalman_ready = 1U;
    ball_step_kalman_position = position;
    ball_step_kalman_velocity = 0.0f;
    ball_step_kalman_p00 = BALL_STEP_KALMAN_MEASUREMENT_NOISE;
    ball_step_kalman_p01 = 0.0f;
    ball_step_kalman_p10 = 0.0f;
    ball_step_kalman_p11 =
        BALL_STEP_KALMAN_INITIAL_VELOCITY_VARIANCE;
}

static void BallStepCtrl_UpdateKalman(float measurement, float dt_sec)
{
    float dt2;
    float dt3;
    float dt4;
    float acceleration_variance;
    float predicted_position;
    float predicted_velocity;
    float predicted_p00;
    float predicted_p01;
    float predicted_p10;
    float predicted_p11;
    float innovation;
    float innovation_variance;
    float gain_position;
    float gain_velocity;
    float updated_p00;
    float updated_p01;
    float updated_p10;
    float updated_p11;

    dt2 = dt_sec * dt_sec;
    dt3 = dt2 * dt_sec;
    dt4 = dt2 * dt2;
    acceleration_variance =
        BALL_STEP_KALMAN_ACCELERATION_STDDEV *
        BALL_STEP_KALMAN_ACCELERATION_STDDEV;

    /* Constant-velocity prediction with white acceleration noise. */
    predicted_position = ball_step_kalman_position +
        dt_sec * ball_step_kalman_velocity;
    predicted_velocity = ball_step_kalman_velocity;
    predicted_p00 = ball_step_kalman_p00 +
        dt_sec * (ball_step_kalman_p01 + ball_step_kalman_p10) +
        dt2 * ball_step_kalman_p11 +
        acceleration_variance * dt4 * 0.25f;
    predicted_p01 = ball_step_kalman_p01 +
        dt_sec * ball_step_kalman_p11 +
        acceleration_variance * dt3 * 0.5f;
    predicted_p10 = ball_step_kalman_p10 +
        dt_sec * ball_step_kalman_p11 +
        acceleration_variance * dt3 * 0.5f;
    predicted_p11 = ball_step_kalman_p11 +
        acceleration_variance * dt2;

    innovation = measurement - predicted_position;
    innovation_variance = predicted_p00 +
        BALL_STEP_KALMAN_MEASUREMENT_NOISE;
    if (innovation_variance <= 0.0f) {
        BallStepCtrl_InitKalman(measurement);
        return;
    }

    gain_position = predicted_p00 / innovation_variance;
    gain_velocity = predicted_p10 / innovation_variance;
    ball_step_kalman_position = predicted_position +
        gain_position * innovation;
    ball_step_kalman_velocity = predicted_velocity +
        gain_velocity * innovation;

    updated_p00 = (1.0f - gain_position) * predicted_p00;
    updated_p01 = (1.0f - gain_position) * predicted_p01;
    updated_p10 = predicted_p10 - gain_velocity * predicted_p00;
    updated_p11 = predicted_p11 - gain_velocity * predicted_p01;

    /* Keep covariance symmetric after floating-point rounding. */
    ball_step_kalman_p00 = (updated_p00 > 0.0f) ? updated_p00 : 0.0f;
    ball_step_kalman_p01 = (updated_p01 + updated_p10) * 0.5f;
    ball_step_kalman_p10 = ball_step_kalman_p01;
    ball_step_kalman_p11 = (updated_p11 > 0.0f) ? updated_p11 : 0.0f;

    ball_step_filtered_position = ball_step_kalman_position;
    ball_step_velocity = BallStepCtrl_LimitFloat(
        ball_step_kalman_velocity,
        -BALL_STEP_VELOCITY_MAX, BALL_STEP_VELOCITY_MAX);
}

static void BallStepCtrl_ResetMotionState(void)
{
    BallStepCtrl_ResetPidState();
    BallStepCtrl_ResetKalmanState();
    ball_step_velocity = 0.0f;
    ball_step_predicted_error = 0.0f;
    ball_step_braking_angle = 0.0f;
    ball_step_velocity_ready = 0U;
    ball_step_settling = 0U;
    ball_step_settle_confirm_count = 0U;
#if BALL_STEP_ADAPTIVE_OUTPUT_ENABLE
    BallStepCtrl_ResetAdaptiveState();
#endif
}

void BallStepCtrl_Init(void)
{
    ball_step_enabled = BALL_STEP_DEFAULT_ENABLED;
    ball_step_position_valid = 0U;
    ball_step_filter_ready = 0U;
    ball_step_input_mode = BALL_STEP_INPUT_POSITION;
    ball_step_reference = BALL_STEP_DEFAULT_REFERENCE;
    ball_step_kp = BALL_STEP_DEFAULT_KP;
    ball_step_ki = BALL_STEP_DEFAULT_KI;
    ball_step_kd = BALL_STEP_DEFAULT_KD;
    ball_step_raw_position = 0U;
    ball_step_filtered_position = 0.0f;
    ball_step_direct_error = 0.0f;
    ball_step_error = 0.0f;
    BallStepCtrl_ResetMotionState();
    ball_step_output_angle = 0.0f;
    ball_step_last_position_ms = 0U;

    BallStepCtrl_Stop();
}

void BallStepCtrl_UpdatePosition(uint16_t position, uint8_t valid,
    uint32_t now_ms)
{
    uint32_t sample_dt_ms;

    if (ball_step_input_mode != BALL_STEP_INPUT_POSITION) {
        BallStepCtrl_ResetMotionState();
    }

    ball_step_raw_position = position;
    ball_step_input_mode = BALL_STEP_INPUT_POSITION;

    if (valid == 0U) {
        ball_step_position_valid = 0U;
        ball_step_filter_ready = 0U;
        BallStepCtrl_ResetMotionState();
        return;
    }

    if ((ball_step_kalman_ready == 0U) ||
        (ball_step_filter_ready == 0U)) {
        BallStepCtrl_InitKalman((float)position);
        ball_step_filtered_position = (float)position;
        ball_step_velocity = 0.0f;
        ball_step_velocity_ready = 0U;
    } else {
        sample_dt_ms = (uint32_t)(now_ms - ball_step_last_position_ms);
        if ((sample_dt_ms > 0U) &&
            (sample_dt_ms < BALL_STEP_INPUT_TIMEOUT_MS)) {
            BallStepCtrl_UpdateKalman((float)position,
                (float)sample_dt_ms / 1000.0f);
            ball_step_velocity_ready = 1U;
        } else {
            BallStepCtrl_InitKalman((float)position);
            ball_step_filtered_position = (float)position;
            ball_step_velocity = 0.0f;
            ball_step_velocity_ready = 0U;
        }
    }

    ball_step_filter_ready = 1U;
    ball_step_position_valid = 1U;
    ball_step_last_position_ms = now_ms;
}

void BallStepCtrl_UpdateError(int16_t error, uint8_t valid, uint32_t now_ms)
{
    if (ball_step_input_mode != BALL_STEP_INPUT_DIRECT_ERROR) {
        BallStepCtrl_ResetMotionState();
    }

    ball_step_input_mode = BALL_STEP_INPUT_DIRECT_ERROR;
    ball_step_filter_ready = 0U;
    BallStepCtrl_ResetKalmanState();
    ball_step_direct_error = (float)error;

    if (valid == 0U) {
        ball_step_position_valid = 0U;
        BallStepCtrl_ResetMotionState();
        return;
    }

    ball_step_position_valid = 1U;
    ball_step_last_position_ms = now_ms;
}

void BallStepCtrl_Task10ms(uint32_t now_ms)
{
    float target_angle;
    float angle_delta;
#if !BALL_STEP_USE_VELOCITY_BRAKE
    float abs_error;
#endif
    float derivative_raw;
    float integral_candidate;
    float pid_angle;
    float control_error;
    float control_abs_error;
    float predicted_position;
    float abs_velocity;
    float target_abs_angle;
    uint8_t command_direction_positive;
#if BALL_STEP_SETTLE_ENABLE
    uint8_t settle_position_ready;
    uint8_t settle_velocity_ready;
#endif
#if BALL_STEP_ASYMMETRIC_OUTPUT_ENABLE
    float direction_gain_scale;
    uint8_t limited_direction;
#endif
#if BALL_STEP_NONLINEAR_OUTPUT_ENABLE
    float nonlinear_scale;
#endif
#if !BALL_STEP_USE_VELOCITY_BRAKE
    float brake_distance;
    float brake_lead_distance;
    float required_deceleration;
#endif

    abs_velocity = 0.0f;

    if ((ball_step_position_valid != 0U) &&
        ((uint32_t)(now_ms - ball_step_last_position_ms) >=
            BALL_STEP_INPUT_TIMEOUT_MS)) {
        ball_step_position_valid = 0U;
        ball_step_filter_ready = 0U;
    }

    if ((ball_step_enabled == 0U) ||
        (ball_step_position_valid == 0U)) {
        ball_step_output_angle = 0.0f;
        BallStepCtrl_ResetMotionState();
        BallStepCtrl_Stop();
        return;
    }

    if (ball_step_input_mode == BALL_STEP_INPUT_DIRECT_ERROR) {
        ball_step_error = ball_step_direct_error;
    } else {
        ball_step_error = (float)ball_step_reference -
            ball_step_filtered_position;
    }

#if BALL_STEP_ADAPTIVE_OUTPUT_ENABLE
    if (ball_step_input_mode == BALL_STEP_INPUT_POSITION) {
        BallStepCtrl_UpdateAdaptiveScale(ball_step_error);
    } else {
        BallStepCtrl_ResetAdaptiveState();
    }
#endif

    control_error = ball_step_error;
    ball_step_predicted_error = ball_step_error;
    ball_step_braking_angle = 0.0f;

    if (ball_step_input_mode == BALL_STEP_INPUT_POSITION) {
        predicted_position = ball_step_filtered_position +
            ball_step_velocity * BALL_STEP_PREDICTION_TIME_SEC;
        control_error = (float)ball_step_reference - predicted_position;
        ball_step_predicted_error = control_error;

        abs_velocity = (ball_step_velocity >= 0.0f) ?
            ball_step_velocity : -ball_step_velocity;
#if !BALL_STEP_USE_VELOCITY_BRAKE
        abs_error = (ball_step_error >= 0.0f) ? ball_step_error :
            -ball_step_error;
#endif

        if (abs_velocity > BALL_STEP_VELOCITY_DEADBAND) {
#if BALL_STEP_USE_VELOCITY_BRAKE
            /* 直接对速度施加反向倾角，形成速度阻尼，避免依赖
             * 小球是否已经越过目标点的方向判断。 */
            ball_step_braking_angle =
                -BALL_STEP_VELOCITY_BRAKE_GAIN * ball_step_velocity;
            ball_step_braking_angle = BallStepCtrl_LimitFloat(
                ball_step_braking_angle,
                -BALL_STEP_VELOCITY_BRAKE_MAX_ANGLE,
                BALL_STEP_VELOCITY_BRAKE_MAX_ANGLE);
#else
            /* 备用：按剩余距离计算达到目标点所需的减速度。 */
            if ((BALL_STEP_EFFECTIVE_DECEL_PER_DEG > 0.0f) &&
                (((ball_step_error * ball_step_velocity) > 0.0f) ||
                 (abs_error <= BALL_STEP_DEFAULT_DEADBAND))) {
                brake_lead_distance = abs_velocity *
                    BALL_STEP_BRAKE_LEAD_TIME_SEC;
                brake_distance = abs_error - brake_lead_distance;
                brake_distance = (brake_distance >
                    BALL_STEP_BRAKE_MIN_DISTANCE) ? brake_distance :
                    BALL_STEP_BRAKE_MIN_DISTANCE;
                required_deceleration = (ball_step_velocity *
                    ball_step_velocity) / (2.0f * brake_distance);
                ball_step_braking_angle =
                    required_deceleration /
                    BALL_STEP_EFFECTIVE_DECEL_PER_DEG *
                    BALL_STEP_BRAKE_ANGLE_GAIN;
                if (ball_step_velocity > 0.0f) {
                    ball_step_braking_angle = -ball_step_braking_angle;
                }
            }
#endif
        }
    }

    control_abs_error = (control_error >= 0.0f) ? control_error :
        -control_error;

#if BALL_STEP_SETTLE_ENABLE
    /* 稳定状态使用进入/退出两套阈值，避免在目标附近反复抖动切换。 */
    settle_position_ready =
        (control_abs_error <= BALL_STEP_SETTLE_ENTER_ERROR) ? 1U : 0U;
    settle_velocity_ready =
        (abs_velocity <= BALL_STEP_SETTLE_ENTER_VELOCITY) ? 1U : 0U;

    if (ball_step_input_mode != BALL_STEP_INPUT_POSITION) {
        ball_step_settling = 0U;
        ball_step_settle_confirm_count = 0U;
    } else if (ball_step_settling != 0U) {
        if ((control_abs_error > BALL_STEP_SETTLE_EXIT_ERROR) ||
            (abs_velocity > BALL_STEP_SETTLE_EXIT_VELOCITY)) {
            ball_step_settling = 0U;
            ball_step_settle_confirm_count = 0U;
        }
    } else if ((settle_position_ready != 0U) &&
        (settle_velocity_ready != 0U)) {
        if (ball_step_settle_confirm_count <
            BALL_STEP_SETTLE_CONFIRM_COUNT) {
            ball_step_settle_confirm_count++;
        }
        if (ball_step_settle_confirm_count >=
            BALL_STEP_SETTLE_CONFIRM_COUNT) {
            ball_step_settling = 1U;
        }
    } else {
        ball_step_settle_confirm_count = 0U;
    }
#endif

    /* 根据最终物理输出方向选择位置控制强度。误差方向会受方向反转
     * 宏影响，因此这里先换算成实际滑轨的正负角度方向。 */
    command_direction_positive = (control_error >= 0.0f) ? 1U : 0U;
    if (((ball_step_input_mode == BALL_STEP_INPUT_POSITION) &&
            (BALL_STEP_DIRECTION_INVERT != 0U)) ||
        ((ball_step_input_mode == BALL_STEP_INPUT_DIRECT_ERROR) &&
            (BALL_STEP_ERROR_DIRECTION_INVERT != 0U))) {
        command_direction_positive =
            (command_direction_positive != 0U) ? 0U : 1U;
    }

#if BALL_STEP_ASYMMETRIC_OUTPUT_ENABLE
    limited_direction =
        (command_direction_positive ==
            BALL_STEP_LIMITED_DIRECTION_POSITIVE) ? 1U : 0U;
    direction_gain_scale = (limited_direction != 0U) ?
        BALL_STEP_LIMITED_POSITION_GAIN_SCALE :
        BALL_STEP_FREE_POSITION_GAIN_SCALE;
#endif

    if (control_abs_error <= BALL_STEP_DEFAULT_DEADBAND) {
        /* Keep the I term active inside the P/D deadband so it can
         * overcome static friction without allowing D to chatter. */
        ball_step_derivative = 0.0f;
        ball_step_last_error = control_error;
        ball_step_pid_ready = 1U;

        integral_candidate = BallStepCtrl_LimitFloat(
            ball_step_integral + control_error * BALL_STEP_PID_DT_SEC,
            -BALL_STEP_INTEGRAL_LIMIT, BALL_STEP_INTEGRAL_LIMIT);
        ball_step_integral = integral_candidate;
        pid_angle = ball_step_ki * ball_step_integral;
    } else {
        if (ball_step_pid_ready == 0U) {
            ball_step_last_error = control_error;
            ball_step_derivative = 0.0f;
            ball_step_pid_ready = 1U;
        } else {
            derivative_raw = (control_error - ball_step_last_error) /
                BALL_STEP_PID_DT_SEC;
            ball_step_derivative +=
                BALL_STEP_DERIVATIVE_FILTER_ALPHA *
                (derivative_raw - ball_step_derivative);
            ball_step_last_error = control_error;
        }

        integral_candidate = BallStepCtrl_LimitFloat(
            ball_step_integral + control_error * BALL_STEP_PID_DT_SEC,
            -BALL_STEP_INTEGRAL_LIMIT, BALL_STEP_INTEGRAL_LIMIT);
        pid_angle = ball_step_kp * control_error +
            ball_step_ki * integral_candidate +
            ball_step_kd * ball_step_derivative;

        /* Stop integrating while the output is saturated in the same
         * direction as the remaining error. */
        if (!(((pid_angle > BALL_STEP_OUTPUT_ANGLE_MAX_DEG) &&
                (control_error > 0.0f)) ||
              ((pid_angle < BALL_STEP_OUTPUT_ANGLE_MIN_DEG) &&
                (control_error < 0.0f)))) {
            ball_step_integral = integral_candidate;
            pid_angle = ball_step_kp * control_error +
                ball_step_ki * ball_step_integral +
                ball_step_kd * ball_step_derivative;
        }
    }

#if BALL_STEP_NONLINEAR_OUTPUT_ENABLE
    /* 只缩放位置 PID 分量；速度反向刹车保持完整，避免高速时刹车不足。 */
    if (BALL_STEP_NONLINEAR_ERROR_RANGE > 0.0f) {
        nonlinear_scale = control_abs_error /
            BALL_STEP_NONLINEAR_ERROR_RANGE;
        nonlinear_scale = BallStepCtrl_LimitFloat(
            nonlinear_scale,
            BALL_STEP_NONLINEAR_MIN_SCALE,
            1.0f);
        pid_angle *= nonlinear_scale;
    }
#endif

#if BALL_STEP_ASYMMETRIC_OUTPUT_ENABLE
    pid_angle *= direction_gain_scale;
#endif

#if BALL_STEP_ADAPTIVE_OUTPUT_ENABLE
    if (ball_step_input_mode == BALL_STEP_INPUT_POSITION) {
        /* 稳定状态及其后的震荡收敛阶段，使用最近几次过零峰值
         * 计算出的动态比例；速度阻尼项保持独立。 */
        pid_angle *= ball_step_adaptive_scale;
    }
#endif

    target_angle = pid_angle + ball_step_braking_angle;

    if (((ball_step_input_mode == BALL_STEP_INPUT_POSITION) &&
            (BALL_STEP_DIRECTION_INVERT != 0U)) ||
        ((ball_step_input_mode == BALL_STEP_INPUT_DIRECT_ERROR) &&
            (BALL_STEP_ERROR_DIRECTION_INVERT != 0U))) {
        target_angle = -target_angle;
    }

    target_angle = BallStepCtrl_LimitFloat(target_angle,
        BALL_STEP_OUTPUT_ANGLE_MIN_DEG, BALL_STEP_OUTPUT_ANGLE_MAX_DEG);

    /* 速度阻尼可能与位置 PID 抵消到接近零，但小球仍未回到目标点。
     * 此时给一个很小的目标方向倾角，保证电机继续把小球拉回中心。 */
    target_abs_angle = (target_angle >= 0.0f) ? target_angle : -target_angle;
    if ((control_abs_error > BALL_STEP_DEFAULT_DEADBAND) &&
        (target_abs_angle < BALL_STEP_RECENTER_MIN_ANGLE_DEG)) {
        target_angle = (command_direction_positive != 0U) ?
            BALL_STEP_RECENTER_MIN_ANGLE_DEG :
            -BALL_STEP_RECENTER_MIN_ANGLE_DEG;
    }

#if BALL_STEP_ASYMMETRIC_OUTPUT_ENABLE
    /* 速度阻尼仍可输出完整反向角度，但最终目标角度不能超过受限方向
     * 的物理安全范围。 */
    if ((target_angle >= 0.0f) &&
        (BALL_STEP_LIMITED_DIRECTION_POSITIVE != 0U)) {
        if (target_angle > BALL_STEP_LIMITED_OUTPUT_ANGLE_DEG) {
            target_angle = BALL_STEP_LIMITED_OUTPUT_ANGLE_DEG;
        }
    } else if ((target_angle < 0.0f) &&
        (BALL_STEP_LIMITED_DIRECTION_POSITIVE == 0U)) {
        if (target_angle < -BALL_STEP_LIMITED_OUTPUT_ANGLE_DEG) {
            target_angle = -BALL_STEP_LIMITED_OUTPUT_ANGLE_DEG;
        }
    }
#endif

#if BALL_STEP_SETTLE_ENABLE
    if (ball_step_settling != 0U) {
        target_abs_angle = (target_angle >= 0.0f) ? target_angle :
            -target_angle;
        if (target_abs_angle > BALL_STEP_SETTLE_MAX_ANGLE) {
            target_angle = (target_angle >= 0.0f) ?
                BALL_STEP_SETTLE_MAX_ANGLE :
                -BALL_STEP_SETTLE_MAX_ANGLE;
        }
    }
#endif

    angle_delta = target_angle - ball_step_output_angle;
    angle_delta = BallStepCtrl_LimitFloat(angle_delta,
        -BALL_STEP_MAX_ANGLE_STEP_DEG, BALL_STEP_MAX_ANGLE_STEP_DEG);
    ball_step_output_angle += angle_delta;

    step_motor2_set_target_angle(ball_step_output_angle);
}

void BallStepCtrl_Stop(void)
{
    ball_step_direct_error = 0.0f;
    ball_step_error = 0.0f;
    BallStepCtrl_ResetMotionState();
    /* Keep the command state aligned with the last logical motor position. */
    ball_step_output_angle = step_motor2_get_current_angle();
    step_motor2_stop_position();
}

void BallStepCtrl_SetEnabled(uint8_t enabled)
{
    ball_step_enabled = (enabled != 0U) ? 1U : 0U;

    if (ball_step_enabled == 0U) {
        BallStepCtrl_Stop();
    }
}

uint8_t BallStepCtrl_IsEnabled(void)
{
    return ball_step_enabled;
}

void BallStepCtrl_SetReference(int16_t reference)
{
    int16_t limited_reference;

    limited_reference = BallStepCtrl_LimitReference(reference);
    if (limited_reference != ball_step_reference) {
        ball_step_reference = limited_reference;
        ball_step_settling = 0U;
        ball_step_settle_confirm_count = 0U;
#if BALL_STEP_ADAPTIVE_OUTPUT_ENABLE
        BallStepCtrl_ResetAdaptiveState();
#endif
    }
}

int16_t BallStepCtrl_GetReference(void)
{
    return ball_step_reference;
}

void BallStepCtrl_SetKp(float kp)
{
    ball_step_kp = BallStepCtrl_LimitFloat(kp, 0.0f, BALL_STEP_KP_MAX);
}

float BallStepCtrl_GetKp(void)
{
    return ball_step_kp;
}

void BallStepCtrl_SetKi(float ki)
{
    ball_step_ki = BallStepCtrl_LimitFloat(ki, 0.0f, BALL_STEP_KI_MAX);
    BallStepCtrl_ResetPidState();
}

float BallStepCtrl_GetKi(void)
{
    return ball_step_ki;
}

void BallStepCtrl_SetKd(float kd)
{
    ball_step_kd = BallStepCtrl_LimitFloat(kd, 0.0f, BALL_STEP_KD_MAX);
    BallStepCtrl_ResetPidState();
}

float BallStepCtrl_GetKd(void)
{
    return ball_step_kd;
}

float BallStepCtrl_GetFilteredPosition(void)
{
    return ball_step_filtered_position;
}

float BallStepCtrl_GetVelocity(void)
{
    return ball_step_velocity;
}

float BallStepCtrl_GetPredictedError(void)
{
    return ball_step_predicted_error;
}

float BallStepCtrl_GetBrakingAngle(void)
{
    return ball_step_braking_angle;
}

uint16_t BallStepCtrl_GetRawPosition(void)
{
    return ball_step_raw_position;
}

uint8_t BallStepCtrl_IsPositionValid(void)
{
    return ball_step_position_valid;
}

float BallStepCtrl_GetError(void)
{
    return ball_step_error;
}

float BallStepCtrl_GetOutputAngle(void)
{
    return ball_step_output_angle;
}

uint8_t BallStepCtrl_IsSettling(void)
{
    return ball_step_settling;
}

float BallStepCtrl_GetAdaptiveScale(void)
{
#if BALL_STEP_ADAPTIVE_OUTPUT_ENABLE
    return ball_step_adaptive_scale;
#else
    return 1.0f;
#endif
}

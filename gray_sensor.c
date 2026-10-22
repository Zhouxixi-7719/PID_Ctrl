#include "gray_sensor.h"

#include "dc_encoder.h"
#include "mpu6050.h"
#include "oled.h"
#include "pid.h"
#include "speed_ctrl.h"
#include "ti_msp_dl_config.h"
#include "yaw.h"

/* OLED 灰度调试界面的刷新周期，避免屏幕刷新过快占用太多时间。 */
#define GRAY_SENSOR_REFRESH_MS        100U

/* 灰度传感器数量：L4 L3 L2 L1 R1 R2 R3 R4 共 8 路。 */
#define TRACK_SENSOR_COUNT            8U

/* 左侧 4 路传感器掩码，bit7~bit4 对应 L4~L1。 */
#define TRACK_LEFT_MASK               0xF0U

/* 右侧 4 路传感器掩码，bit3~bit0 对应 R1~R4。 */
#define TRACK_RIGHT_MASK              0x0FU

/*
 * 每路传感器对应的位置权重。
 * 负数表示黑线在车体左侧，正数表示黑线在车体右侧。
 */
static const int8_t track_weights[TRACK_SENSOR_COUNT] = {
    -7, -5, -3, -1, 1, 3, 5, 7
};

/* 循迹位置环 PID 控制器，只负责把灰度误差转换成左右轮差速。 */
static pid_ctrl_t track_pid;

/* 循迹功能使能标志：1 表示 Track_ApplyTarget 会按灰度修正左右轮速度。 */
static volatile uint8_t track_enabled = 0U;

/* 8 路灰度有效位缓存，bit7~bit0 对应 L4 L3 L2 L1 R1 R2 R3 R4。 */
static volatile uint8_t track_sensor_bits = 0U;

/* 当前循迹误差，负数向左修正，正数向右修正。 */
static volatile int16_t track_error = 0;

/* 大面积黑线/十字检测标志，需要外部读取后手动清除。 */
static volatile uint8_t track_cross_flag = 0U;

/* 当前循迹状态机状态：普通循迹、停车准备转弯、正在原地转弯。 */
static volatile track_state_t track_state = TRACK_STATE_LINE_FOLLOW;

static volatile track_line_mode_t track_line_mode = TRACK_LINE_MODE_STRAIGHT;

/* 直角弯原地转向时，编码器估算出的累计转角，单位度。 */
static volatile float track_turn_angle_deg = 0.0f;

/* 直角弯失败标志：超过最大角度仍未检测到中线时置 1。 */
static volatile uint8_t track_turn_fail_flag = 0U;

static volatile int16_t track_turn_pwm = TRACK_TURN_PWM;
static volatile uint16_t track_turn_stop_ms = TRACK_TURN_STOP_MS;
static volatile uint16_t track_turn_lock_ms = TRACK_TURN_LOCK_MS;

/* 上一次有效循迹误差，短暂丢线时沿用它继续往原方向找线。 */
static int16_t track_last_valid_error = 0;


/* 出弯时中间两路连续检测到黑线的次数。 */
static uint8_t track_center_found_count = 0U;

/* 直角弯冷却计数，出弯后一段时间内不再触发新的直角弯。 */
static uint8_t track_corner_cooldown_count = 0U;

/* 当前直角弯状态已经持续的时间，单位 ms。 */
static uint16_t track_turn_elapsed_ms = 0U;

/* 进入直角弯前保存的基础速度，出弯后恢复使用。 */
static float track_saved_speed = 0.0f;

/* 本次直角弯是否正在用陀螺仪测角(1)，掉线时会就地回退为 0。 */
#if (TRACK_TURN_USE_GYRO != 0U)
static volatile uint8_t track_turn_gyro_active = 0U;
#endif

/* 进入原地转向瞬间的航向角，作为本次转弯的角度基准。 */
#if ((TRACK_CORNER_EN != 0U) && (TRACK_TURN_USE_GYRO != 0U))
static float track_turn_yaw_base = 0.0f;
#endif

#if (TRACK_LOST_STRAIGHT_EN != 0U)
/* 航向保持 PID，只在丢线直行状态下使用。 */
static pid_ctrl_t track_lost_pid;

/* 丢线瞬间锁存的航向角。进入状态时写一次，之后只读。 */
static float track_lost_yaw_base = 0.0f;

/* 连续全白周期数，达到 TRACK_LOST_ENTER_N 才认定丢线。 */
static uint8_t track_lost_count = 0U;

/* 丢线期间连续压线周期数，达到 TRACK_LOST_EXIT_N 才认定找回。 */
static uint8_t track_found_count = 0U;

/* 丢线直行已持续的时间，单位 ms。 */
static uint16_t track_lost_elapsed_ms = 0U;

/* 本次丢线的航向角是否还在用陀螺仪(1)，掉线后切 0 改用编码器。 */
static uint8_t track_lost_use_gyro = 1U;
#endif

/* 读取单个 GPIO 引脚原始电平，返回 0 或 1。 */
static uint8_t read_gray_pin(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1U : 0U;
}

/* 把原始 GPIO 电平转换成“是否检测到黑线”的布尔值。 */
static uint8_t gray_sensor_pin_is_black(GPIO_Regs *port, uint32_t pin)
{
    uint8_t level = read_gray_pin(port, pin);

#if (GRAY_SENSOR_BLACK_LEVEL == 0U)
    return (level == 0U) ? 1U : 0U;
#else
    return (level != 0U) ? 1U : 0U;
#endif
}

/* 读取 8 路灰度并打包成 bit 位，方便后续用掩码判断位置。 */
static uint8_t gray_sensor_read_line_bits(void)
{
    uint8_t bits = 0U;

    bits |= (uint8_t)(gray_sensor_pin_is_black(GRAY_SENSOR_L4_PORT,
        GRAY_SENSOR_L4_PIN) << 7U);
    bits |= (uint8_t)(gray_sensor_pin_is_black(GRAY_SENSOR_L3_PORT,
        GRAY_SENSOR_L3_PIN) << 6U);
    bits |= (uint8_t)(gray_sensor_pin_is_black(GRAY_SENSOR_L2_PORT,
        GRAY_SENSOR_L2_PIN) << 5U);
    bits |= (uint8_t)(gray_sensor_pin_is_black(GRAY_SENSOR_L1_PORT,
        GRAY_SENSOR_L1_PIN) << 4U);
    bits |= (uint8_t)(gray_sensor_pin_is_black(GRAY_SENSOR_R1_PORT,
        GRAY_SENSOR_R1_PIN) << 3U);
    bits |= (uint8_t)(gray_sensor_pin_is_black(GRAY_SENSOR_R2_PORT,
        GRAY_SENSOR_R2_PIN) << 2U);
    bits |= (uint8_t)(gray_sensor_pin_is_black(GRAY_SENSOR_R3_PORT,
        GRAY_SENSOR_R3_PIN) << 1U);
    bits |= gray_sensor_pin_is_black(GRAY_SENSOR_R4_PORT, GRAY_SENSOR_R4_PIN);

    return bits;
}

/* float 绝对值函数，避免额外引入 math 库。 */
static float track_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* float 限幅函数，把 value 夹在 min~max 之间。 */
static float track_limit_float(float value, float min, float max)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

/*
 * 用左右编码器增量估算本周期车体转过的角度，单位度。
 * 正值表示右轮走得多，即车体左转(逆时针)，与陀螺仪的符号约定一致。
 *
 * 直角弯测角和丢线航向都会回退到它，所以放在通用工具区，
 * 不要塞进任何一个特性的条件编译块里。
 */
static float track_encoder_delta_deg(void)
{
    float left_cm = (float)Encoder_GetLastDelta_L() *
        DC_WHEEL_CIRCUMFERENCE_CM / DC_ENCODER_COUNTS_PER_REV;
    float right_cm = (float)Encoder_GetLastDelta_R() *
        DC_WHEEL_CIRCUMFERENCE_CM / DC_ENCODER_COUNTS_PER_REV;
    float delta_rad = (right_cm - left_cm) / TRACK_WHEEL_BASE_CM;

    return delta_rad * TRACK_RAD_TO_DEG;
}

/*
 * 根据循迹误差计算自适应前进速度。
 * 直道误差接近 0 时接近 base_speed；误差越大，速度越接近最低比例。
 */
static float track_calc_adaptive_speed(float base_speed, float error)
{
#if (TRACK_SPEED_ADAPTIVE_EN != 0U)
    /* error_ratio 表示当前误差占最大误差的比例，范围被限制在 0~1。 */
    float error_ratio = track_abs_float(error) / TRACK_SPEED_FULL_ERROR;

    /* speed_ratio 是最终速度比例，最低不会低于 TRACK_SPEED_MIN_RATIO。 */
    float speed_ratio;

    error_ratio = track_limit_float(error_ratio, 0.0f, 1.0f);
    speed_ratio = 1.0f - (1.0f - TRACK_SPEED_MIN_RATIO) * error_ratio;
    return base_speed * speed_ratio;
#else
    (void)error;
    return base_speed;
#endif
}

#if 0
static float track_apply_gyro_damping(float diff, int16_t error)
{
    float abs_error = track_abs_float((float)error);
    float yaw_rate;

    if ((track_gyro_enabled == 0U) || (Yaw_IsFaulted() != 0U)) {
        return diff;
    }

    yaw_rate = Yaw_GetRate() * TRACK_GYRO_SIGN;
    if (abs_error <= (float)TRACK_GYRO_DAMP_CENTER_ERROR) {
        return diff;
    }

    if ((abs_error < (float)TRACK_GYRO_DAMP_MIN_ERROR) &&
        (track_abs_float(yaw_rate) < TRACK_GYRO_DAMP_MIN_RATE_DPS)) {
        return diff;
    }

    return diff + yaw_rate * TRACK_GYRO_DAMP_K;
}

static void track_reset_straight_heading(void)
{
#if (TRACK_STRAIGHT_HEADING_HOLD_EN != 0U)
    track_straight_heading_reference = 0.0f;
    track_straight_heading_ready = 0U;
    track_straight_heading_capture_count = 0U;
#endif
}

static float track_apply_center_gyro_damping(float diff, int16_t error,
    uint8_t bits)
{
    float yaw_rate;
#if (TRACK_STRAIGHT_HEADING_HOLD_EN != 0U)
    float yaw_angle;
    float heading_error;
    float heading_scale;
    float abs_position_error;
    uint8_t center_stable;
#endif

    if ((track_gyro_enabled == 0U) || (Yaw_IsFaulted() != 0U) ||
        (track_line_mode != TRACK_LINE_MODE_STRAIGHT)) {
        track_reset_straight_heading();
        return diff;
    }

    yaw_rate = Yaw_GetRate() * TRACK_GYRO_SIGN;

#if (TRACK_STRAIGHT_HEADING_HOLD_EN != 0U)
    yaw_angle = Yaw_GetAngle() * TRACK_GYRO_SIGN;
    if (track_straight_heading_ready == 0U) {
        center_stable = ((track_abs_float((float)error) <=
            (float)TRACK_GYRO_DAMP_CENTER_ERROR) &&
            ((bits & TRACK_TURN_EXIT_CENTER_MASK) != 0U)) ? 1U : 0U;
        if ((center_stable != 0U) &&
            (track_abs_float(yaw_rate) <=
            TRACK_STRAIGHT_HEADING_CAPTURE_RATE_DPS)) {
            if (track_straight_heading_capture_count <
                TRACK_STRAIGHT_HEADING_CAPTURE_N) {
                track_straight_heading_capture_count++;
            }
            if (track_straight_heading_capture_count >=
                TRACK_STRAIGHT_HEADING_CAPTURE_N) {
                track_straight_heading_reference = yaw_angle;
                track_straight_heading_ready = 1U;
            }
        } else {
            track_straight_heading_capture_count = 0U;
        }
    } else {
        /* 基准在整段直线内保持，短暂偏线时也不跟着当前蛇形方向重标。 */
        heading_error = yaw_angle - track_straight_heading_reference;
        heading_error = track_limit_float(heading_error,
            -TRACK_STRAIGHT_HEADING_ERROR_MAX_DEG,
            TRACK_STRAIGHT_HEADING_ERROR_MAX_DEG);
        abs_position_error = track_abs_float((float)error);
        if (abs_position_error <=
            TRACK_STRAIGHT_HEADING_FADE_START_ERROR) {
            heading_scale = 1.0f;
        } else if (abs_position_error >=
            TRACK_STRAIGHT_HEADING_FADE_END_ERROR) {
            heading_scale = 0.0f;
        } else {
            heading_scale = (TRACK_STRAIGHT_HEADING_FADE_END_ERROR -
                abs_position_error) /
                (TRACK_STRAIGHT_HEADING_FADE_END_ERROR -
                TRACK_STRAIGHT_HEADING_FADE_START_ERROR);
        }
        diff += heading_error * TRACK_STRAIGHT_HEADING_KP * heading_scale;
    }
#endif

    if (track_abs_float(yaw_rate) >
        TRACK_CENTER_GYRO_RATE_DEADBAND_DPS) {
        diff += yaw_rate * TRACK_CENTER_GYRO_DAMP_K;
    }

    /* 最终直线差速统一由 track_limit_straight_diff() 做一次硬限幅。 */
    return diff;
}

static float track_apply_diff_slew(float diff_target, float line_speed,
    float min_step, float speed_ratio)
{
    float max_step = min_step + track_abs_float(line_speed) * speed_ratio;
    float delta = diff_target - track_line_diff_output;

    delta = track_limit_float(delta, -max_step, max_step);
    track_line_diff_output += delta;
    return track_line_diff_output;
}

static void track_reset_line_mode(void)
{
    track_line_mode = TRACK_LINE_MODE_STRAIGHT;
    track_curve_enter_count = 0U;
    track_straight_count = 0U;
    track_line_diff_output = 0.0f;
    track_filtered_error = 0.0f;
    track_filtered_error_ready = 0U;
    track_reset_straight_heading();
}

static uint8_t track_is_curve_candidate(int16_t error, uint8_t bits)
{
    float abs_error = track_abs_float((float)error);
    uint8_t by_error =
        (abs_error >= (float)TRACK_LINE_CURVE_ENTER_ERROR) ? 1U : 0U;
    uint8_t by_sensor =
        (((bits & TRACK_LINE_CURVE_SENSOR_MASK) != 0U) &&
        (abs_error >= (float)TRACK_LINE_CURVE_SENSOR_ERROR)) ? 1U : 0U;

    return ((by_error != 0U) || (by_sensor != 0U)) ? 1U : 0U;
}

static uint8_t track_is_straight_candidate(int16_t error, uint8_t bits)
{
    uint8_t center_seen =
        ((bits & TRACK_TURN_EXIT_CENTER_MASK) != 0U) ? 1U : 0U;
    uint8_t error_small =
        (track_abs_float((float)error) <= (float)TRACK_LINE_STRAIGHT_ERROR) ?
        1U : 0U;

    return ((center_seen != 0U) && (error_small != 0U)) ? 1U : 0U;
}

static float track_filter_error(int16_t error)
{
    if (track_filtered_error_ready == 0U) {
        track_filtered_error = (float)error;
        track_filtered_error_ready = 1U;
    } else {
        track_filtered_error += TRACK_ERROR_FILTER_ALPHA *
            ((float)error - track_filtered_error);
    }

    return track_filtered_error;
}

static float track_get_straight_control_error(float error)
{
    float abs_error = track_abs_float(error);

    if (abs_error <= TRACK_STRAIGHT_ERROR_DEADBAND) {
        return 0.0f;
    }

    return error;
}

static float track_apply_straight_output_shape(float output, float error)
{
    float abs_error = track_abs_float(error);

    if (abs_error <= TRACK_STRAIGHT_ERROR_DEADBAND) {
        return 0.0f;
    }

    if (abs_error <= TRACK_STRAIGHT_CENTER_ERROR) {
        return output * TRACK_STRAIGHT_CENTER_OUTPUT_RATIO;
    }

    return output;
}

static float track_limit_straight_diff(float diff, float line_speed)
{
    float diff_limit = track_abs_float(line_speed) *
        TRACK_STRAIGHT_DIFF_LIMIT_RATIO;

    return track_limit_float(diff, -diff_limit, diff_limit);
}

static void track_update_line_mode(int16_t error, uint8_t bits)
{
    if (track_line_mode == TRACK_LINE_MODE_STRAIGHT) {
        if (track_is_curve_candidate(error, bits) != 0U) {
            if (track_curve_enter_count < TRACK_LINE_CURVE_ENTER_N) {
                track_curve_enter_count++;
            }
        } else {
            track_curve_enter_count = 0U;
        }

        if (track_curve_enter_count >= TRACK_LINE_CURVE_ENTER_N) {
            track_line_mode = TRACK_LINE_MODE_CURVE;
            track_straight_count = 0U;
        }
        return;
    }

    if (track_is_straight_candidate(error, bits) != 0U) {
        if (track_straight_count < TRACK_LINE_STRAIGHT_N) {
            track_straight_count++;
        }
    } else {
        track_straight_count = 0U;
    }

    if (track_straight_count >= TRACK_LINE_STRAIGHT_N) {
        track_line_mode = TRACK_LINE_MODE_STRAIGHT;
        track_curve_enter_count = 0U;
    }
}

/* 统计 8 路灰度有效位中有多少路检测到黑线。 */
#endif

static void track_reset_line_mode(void)
{
    track_line_mode = TRACK_LINE_MODE_STRAIGHT;
}

static uint8_t track_count_bits(uint8_t bits)
{
    uint8_t count = 0U;
    uint8_t i;

    for (i = 0U; i < TRACK_SENSOR_COUNT; i++) {
        if ((bits & (uint8_t)(0x80U >> i)) != 0U) {
            count++;
        }
    }

    return count;
}

#if (TRACK_CORNER_EN != 0U)
/* 先用 mask 取出关心的传感器区域，再统计该区域内的有效位数量。 */
static uint8_t track_count_mask_bits(uint8_t bits, uint8_t mask)
{
    return track_count_bits((uint8_t)(bits & mask));
}
#endif

/*
 * 根据灰度 bit 位求循迹误差。
 * 多路同时检测到黑线时使用加权重心，能处理圆弧弯压到多路的情况。
 */
static int16_t track_solve_error(uint8_t bits)
{
    uint8_t count = track_count_bits(bits);
    int16_t weight_sum = 0;
    uint8_t i;

    /* 短暂丢线时沿用上一次误差，让车继续朝原来的方向找线。 */
    if (count == 0U) {
        return track_last_valid_error;
    }

    /* 7 路及以上认为是大面积黑线/十字，不再给普通循迹误差。 */
    if (count >= 7U) {
        track_cross_flag = 1U;
        return 0;
    }

    /* 对所有检测到黑线的传感器位置求平均，得到离散重心误差。 */
    for (i = 0U; i < TRACK_SENSOR_COUNT; i++) {
        if ((bits & (uint8_t)(0x80U >> i)) != 0U) {
            weight_sum += track_weights[i];
        }
    }

    track_last_valid_error = weight_sum / (int16_t)count;
    return track_last_valid_error;
}

#if (TRACK_CORNER_EN != 0U)
/* 更新直角弯识别上下文，目前主要维护出弯后的冷却计数。 */
static void track_update_corner_context(uint8_t bits)
{
    (void)bits;
    if (track_corner_cooldown_count > 0U) {
        track_corner_cooldown_count--;
    }
}
#endif

/* 复位直角弯运行状态，clear_fail 决定是否连失败标志一起清掉。 */
static void track_reset_turn_runtime(uint8_t clear_fail)
{
#if (TRACK_TURN_USE_GYRO != 0U)
    /* 可能是在转弯中途被 Track_Disable 打断，务必把直采模式交回去 */
    if (track_turn_gyro_active != 0U) {
        track_turn_gyro_active = 0U;
        Yaw_EndCritical();
    }
#endif

#if (TRACK_LOST_STRAIGHT_EN != 0U)
    track_lost_count = 0U;
    track_found_count = 0U;
    track_lost_elapsed_ms = 0U;
    track_lost_use_gyro = 1U;
#endif

    track_state = TRACK_STATE_LINE_FOLLOW;
    track_turn_elapsed_ms = 0U;
    track_turn_angle_deg = 0.0f;
    track_saved_speed = 0.0f;
    track_reset_line_mode();
    track_center_found_count = 0U;
    track_corner_cooldown_count = 0U;
    if (clear_fail != 0U) {
        track_turn_fail_flag = 0U;
    }
}

#if (TRACK_CORNER_EN != 0U)
/* 进入直角弯状态机，保存进入前速度并清零转弯计时/角度。 */
static void track_enter_turn(track_state_t new_state, float target_speed)
{
    track_state = new_state;
    track_turn_elapsed_ms = 0U;
    track_turn_angle_deg = 0.0f;
    track_turn_fail_flag = 0U;
    track_saved_speed = target_speed;
    track_center_found_count = 0U;
    track_reset_line_mode();
    PID_Reset(&track_pid);

    g_target_speed = track_saved_speed;
}

/* 结束直角弯并回到普通循迹，failed 非 0 表示超角度失败退出。 */
static void track_finish_turn(uint8_t failed)
{
#if (TRACK_TURN_USE_GYRO != 0U)
    /* 转弯结束，把 I2C 交回主循环的 Yaw_Task */
    if (track_turn_gyro_active != 0U) {
        track_turn_gyro_active = 0U;
        Yaw_EndCritical();
    }
#endif

    if (failed != 0U) {
        track_turn_fail_flag = 1U;
    }

    if (track_saved_speed != 0.0f) {
        g_target_speed = track_saved_speed;
    }

    track_state = TRACK_STATE_LINE_FOLLOW;
    track_turn_elapsed_ms = 0U;
    track_saved_speed = 0.0f;
    track_center_found_count = 0U;
    track_corner_cooldown_count = TRACK_CORNER_COOLDOWN_N;
    track_last_valid_error = 0;
    track_error = 0;
    track_reset_line_mode();
    PID_Reset(&track_pid);
}

/* 根据左右编码器增量估算本周期车体转过的角度。 */
static void track_accumulate_turn_angle_encoder(void)
{
    track_turn_angle_deg += track_encoder_delta_deg();
}

/*
 * 更新本次直角弯已转过的角度。
 *
 * 优先用陀螺仪：取进入转弯瞬间的航向角作基准，当前角度减基准就是
 * 转过的量，不需要逐拍累加，也就不受调用频率抖动影响。
 * 陀螺仪没标定或读故障时回退到编码器差速估算，保证转弯仍能收尾。
 */
static void track_accumulate_turn_angle(void)
{
#if (TRACK_TURN_USE_GYRO != 0U)
    if (track_turn_gyro_active != 0U) {
        if ((track_gyro_enabled != 0U) && (Yaw_IsFaulted() == 0U)) {
            track_turn_angle_deg = (Yaw_GetAngle() - track_turn_yaw_base) *
                TRACK_GYRO_SIGN;
            return;
        }

        /*
         * 转弯途中陀螺仪掉线：就地切回编码器，把已经转过的角度当作
         * 新起点继续累加，不推倒重来。
         */
        track_turn_gyro_active = 0U;
        Yaw_EndCritical();
    }
#endif

    track_accumulate_turn_angle_encoder();
}

/*
 * 直角弯入口检测。
 * 左 4 路全压线认为需要左转，右 4 路全压线认为需要右转。
 */
static uint8_t track_detect_corner(float target_speed)
{
#if (TRACK_CORNER_EN != 0U)
    uint8_t left_count = track_count_mask_bits(track_sensor_bits,
        TRACK_LEFT_MASK);
    uint8_t right_count = track_count_mask_bits(track_sensor_bits,
        TRACK_RIGHT_MASK);

    if (track_corner_cooldown_count != 0U) {
        return 0U;
    }

    if ((left_count == 4U) && (right_count == 4U)) {
        track_cross_flag = 1U;
        return 0U;
    }

    if (left_count == 4U) {
        track_enter_turn(TRACK_STATE_TURN_STOP_LEFT, target_speed);
        return 1U;
    }

    if (right_count == 4U) {
        track_enter_turn(TRACK_STATE_TURN_STOP_RIGHT, target_speed);
        return 1U;
    }
#else
    (void)target_speed;
#endif

    return 0U;
}

/* 出弯检测：中间两路 L1/R1 同时检测到黑线时认为找回中线。 */
static uint8_t track_turn_exit_ready(uint8_t bits)
{
    return ((bits & TRACK_TURN_EXIT_CENTER_MASK) ==
        TRACK_TURN_EXIT_CENTER_MASK) ? 1U : 0U;
}

/*
 * 直角弯状态机处理。
 * 停车阶段输出 0；转弯阶段由 speed_ctrl.c 使用固定 PWM 原地转向。
 */
static void track_process_turning(float *target_l, float *target_r)
{
    float angle_abs;

    g_target_speed = track_saved_speed;
    track_turn_elapsed_ms += SPEED_CTRL_PERIOD_MS;

    if ((track_state == TRACK_STATE_TURN_STOP_LEFT) ||
        (track_state == TRACK_STATE_TURN_STOP_RIGHT)) {
        if (track_turn_elapsed_ms >= track_turn_stop_ms) {
            track_state = (track_state == TRACK_STATE_TURN_STOP_LEFT) ?
                TRACK_STATE_TURNING_LEFT : TRACK_STATE_TURNING_RIGHT;
            track_turn_elapsed_ms = 0U;
            track_turn_angle_deg = 0.0f;
            track_center_found_count = 0U;

#if (TRACK_TURN_USE_GYRO != 0U)
            /*
             * 原地转向正式开始。切到直采模式并锁定角度基准——停车
             * 阶段车是静止的，此刻的航向角就是本次转弯的零点。
             *
             * 必须同时确认已标定：没标过的话 bias 是 0，读数里带着
             * 几十 °/s 的原始零偏，积出来的角度完全不能用，那还不如
             * 老老实实回退到编码器估算。
             */
            if ((track_gyro_enabled != 0U) &&
                (Yaw_IsFaulted() == 0U) &&
                (MPU6050_IsCalibrated() != 0U)) {
                Yaw_BeginCritical();
                track_turn_yaw_base = Yaw_GetAngle();
                track_turn_gyro_active = 1U;
            } else {
                track_turn_gyro_active = 0U;
            }
#endif
        }
        *target_l = 0.0f;
        *target_r = 0.0f;
        return;
    }

    track_accumulate_turn_angle();
    angle_abs = track_abs_float(track_turn_angle_deg);

    /* 锁定期之后，连续多次看到中间两路压线才确认出弯。 */
    if (track_turn_elapsed_ms >= track_turn_lock_ms) {
        if (track_turn_exit_ready(track_sensor_bits) != 0U) {
            if (track_center_found_count < TRACK_TURN_CENTER_CONFIRM_N) {
                track_center_found_count++;
            }
            if (track_center_found_count >= TRACK_TURN_CENTER_CONFIRM_N) {
                track_finish_turn(0U);
                *target_l = g_target_speed;
                *target_r = g_target_speed;
                return;
            }
        } else {
            track_center_found_count = 0U;
        }
    }

    /* 保险退出：转角太大仍没找到中线，结束转弯并置失败标志。 */
    if (angle_abs >= TRACK_TURN_ANGLE_MAX_DEG) {
        track_finish_turn(1U);
        *target_l = g_target_speed;
        *target_r = g_target_speed;
        return;
    }

    *target_l = 0.0f;
    *target_r = 0.0f;
}
#endif

#if (TRACK_LOST_STRAIGHT_EN != 0U)
/*
 * 结束丢线直行，回到普通循迹。
 * failed 非 0 表示是超时或陀螺仪掉线导致的异常退出。
 */
static void track_finish_lost(uint8_t failed)
{
    if (failed != 0U) {
        track_turn_fail_flag = 1U;
    }

    track_state = TRACK_STATE_LINE_FOLLOW;
    track_lost_count = 0U;
    track_found_count = 0U;
    track_lost_elapsed_ms = 0U;
    track_lost_use_gyro = 1U;
    track_reset_line_mode();

    /*
     * 这里刻意不清 track_turn_angle_deg：退出后 OLED 的 A: 位会保留
     * 本次丢线最终偏了多少度，是判断航向环方向对不对的关键依据。
     * 直角弯的 track_finish_turn 同样不清，行为一致。
     */

    /*
     * 灰度 PID 在丢线期间没有运行，它的 previous_error 还停在丢线前，
     * 直接交回去的话 D 项会炸出一个尖峰，必须复位。
     */
    PID_Reset(&track_pid);
    track_last_valid_error = 0;
    track_error = 0;
}

/* 进入丢线直行：锁存当前航向作为基准。 */
static void track_enter_lost(void)
{
    track_state = TRACK_STATE_LOST_STRAIGHT;
    track_lost_elapsed_ms = 0U;
    track_found_count = 0U;
    track_turn_angle_deg = 0.0f;
    track_reset_line_mode();
    PID_Reset(&track_lost_pid);

    /* 只在这里写一次。若每周期都刷新基准，误差恒为 0，等于没有约束。 */
    track_lost_yaw_base = Yaw_GetAngle();

    /*
     * 决定航向来源。陀螺仪不可用就直接从编码器起步——有回退兜底，
     * 所以不必像以前那样因为 MPU 不可信就整个放弃丢线直行。
     */
    track_lost_use_gyro = ((track_gyro_enabled != 0U) &&
        (Yaw_IsFaulted() == 0U) &&
        (MPU6050_IsCalibrated() != 0U)) ? 1U : 0U;

    /*
     * 这里不占用 Yaw 关键区(不禁 OLED 刷新)。
     *
     * 关键区是给直角弯用的：那是 64°/s 的高速原地转、只持续几百毫秒，
     * 角度晚几十毫秒就差十几度。丢线直行完全不同——Kp 只有 0.15，
     * 修正速率约 38°/s，车速 15cm/s，是个慢过程，OLED 那几十毫秒
     * 延迟带来的滞后不到 2°，可以忽略。
     *
     * 反过来，丢线恰恰是最需要盯着屏幕看的状态，冻结显示会让这个
     * 功能完全无法调试。
     */
}

/*
 * 丢线检测。返回 1 表示本周期刚进入丢线直行状态。
 * 不再因为陀螺仪不可信就拒绝进入——航向来源有编码器兜底，
 * 具体用哪个由 track_enter_lost() 决定。
 */
static uint8_t track_detect_lost(void)
{
    if (track_sensor_bits != 0U) {
        track_lost_count = 0U;
        return 0U;
    }

    if (track_lost_count < TRACK_LOST_ENTER_N) {
        track_lost_count++;
    }

    if (track_lost_count < TRACK_LOST_ENTER_N) {
        return 0U;
    }

    track_enter_lost();
    return 1U;
}

/*
 * 更新本次丢线已偏离锁定航向的角度。
 *
 * 优先用陀螺仪：当前航向减去进入瞬间锁存的基准，不逐拍累加，
 * 因而不受调用频率抖动影响。
 *
 * 陀螺仪掉线则就地切到编码器差速估算，把已经偏出的角度当新起点
 * 继续累加。丢线时车是正常前进而非原地转，轮子基本不打滑，
 * 编码器航向在这个工况下反而相当可靠——所以没有必要因为 MPU
 * 掉一下线就把车停死。
 */
static void track_lost_update_angle(void)
{
    if (track_lost_use_gyro != 0U) {
        if ((track_gyro_enabled != 0U) && (Yaw_IsFaulted() == 0U)) {
            track_turn_angle_deg =
                (Yaw_GetAngle() - track_lost_yaw_base) * TRACK_GYRO_SIGN;
            return;
        }
        track_lost_use_gyro = 0U;
    }

    track_turn_angle_deg += track_encoder_delta_deg();
}

/*
 * 丢线直行处理：把"走直线"变成"把航向拉回锁存值"的闭环。
 */
static void track_process_lost_straight(float base_speed, float *target_l,
    float *target_r)
{
    float yaw_err;
    float turn_output;
    float diff;
    float diff_limit;

    track_lost_elapsed_ms += SPEED_CTRL_PERIOD_MS;

    /* 重新压线，连续确认后交回灰度循迹。 */
    if (track_sensor_bits != 0U) {
        if (track_found_count < TRACK_LOST_EXIT_N) {
            track_found_count++;
        }
        if (track_found_count >= TRACK_LOST_EXIT_N) {
            track_finish_lost(0U);
            *target_l = base_speed;
            *target_r = base_speed;
            return;
        }
    } else {
        track_found_count = 0U;
    }

    /* 超时：已经跑出赛道，停车，等待重新启动。 */
    if (track_lost_elapsed_ms >= TRACK_LOST_TIMEOUT_MS) {
        track_finish_lost(1U);
        g_target_speed = 0.0f;
        *target_l = 0.0f;
        *target_r = 0.0f;
        return;
    }

    /*
     * yaw_err > 0 表示车相对锁定航向逆时针偏了(左偏)，需要右转修正；
     * 而下面的 diff > 0 正好是左轮快=右转，符号天然对上，
     * 于是差速换算能和灰度环共用同一份写法。
     *
     * track_turn_angle_deg 同时充当 OLED 的 A: 显示位——两种状态下
     * 它的含义都是"相对基准的角度偏差"。
     */
    track_lost_update_angle();
    yaw_err = track_turn_angle_deg;

    turn_output = PID_CalcWithDFilter(&track_lost_pid, yaw_err, 0.0f,
        TRACK_D_FILTER_ALPHA);

    diff = turn_output * base_speed * TRACK_K;

    /*
     * 硬钳位，保证两轮恒为正转。PID 已经限幅到 TRACK_LOST_OUTPUT_LIMIT，
     * 这里是第二道保险——一旦某侧反转，"直行修正"就变成"原地旋转"，
     * 而航向误差还在继续增大，会一直转到超时为止。
     */
    diff_limit = track_abs_float(base_speed) * TRACK_LOST_DIFF_RATIO;
    diff = track_limit_float(diff, -diff_limit, diff_limit);

    *target_l = base_speed + diff;
    *target_r = base_speed - diff;
}
#endif

/* 初始化循迹 PID，随后调用 Track_Disable 清空状态。 */
void Track_Init(void)
{
    PID_Init(&track_pid, TRACK_TURN_KP, TRACK_TURN_KI, TRACK_TURN_KD,
        -TRACK_TURN_OUTPUT_LIMIT, TRACK_TURN_OUTPUT_LIMIT,
        -TRACK_TURN_OUTPUT_LIMIT, TRACK_TURN_OUTPUT_LIMIT);

#if (TRACK_LOST_STRAIGHT_EN != 0U)
    /*
     * 航向环用自己那份更小的限幅，绝不能沿用灰度环的
     * TRACK_TURN_OUTPUT_LIMIT——那个值允许 diff 超过基础速度、
     * 让一侧轮子反转，结果就是丢线后原地打转。
     */
    PID_Init(&track_lost_pid, TRACK_LOST_KP, TRACK_LOST_KI, TRACK_LOST_KD,
        -TRACK_LOST_OUTPUT_LIMIT, TRACK_LOST_OUTPUT_LIMIT,
        -TRACK_LOST_OUTPUT_LIMIT, TRACK_LOST_OUTPUT_LIMIT);
#endif

    Track_Disable();
}

/* 使能循迹，并清除可能残留的误差、十字、直角弯状态。 */
void Track_Enable(void)
{
    __disable_irq();
    track_enabled = 1U;
    track_cross_flag = 0U;
    track_turn_fail_flag = 0U;
    track_last_valid_error = 0;
    track_error = 0;
    PID_Reset(&track_pid);
    track_reset_turn_runtime(1U);
    __enable_irq();
}

/* 关闭循迹，并清除状态；电机是否停转由 speed_ctrl 模块决定。 */
void Track_Disable(void)
{
    __disable_irq();
    track_enabled = 0U;
    track_cross_flag = 0U;
    track_turn_fail_flag = 0U;
    track_last_valid_error = 0;
    track_error = 0;
    PID_Reset(&track_pid);
    track_reset_turn_runtime(1U);
    __enable_irq();
}

/* 手动复位循迹状态，保持 Track_IsEnabled 的使能状态不变。 */
void Track_Reset(void)
{
    __disable_irq();
    track_cross_flag = 0U;
    track_turn_fail_flag = 0U;
    track_last_valid_error = 0;
    track_error = 0;
    PID_Reset(&track_pid);
    track_reset_turn_runtime(1U);
    __enable_irq();
}

/* 设置循迹 PID 参数；关中断避免 SysTick 中同时计算 PID。 */
void Track_SetPid(float kp, float ki, float kd)
{
    __disable_irq();
    PID_SetParams(&track_pid, kp, ki, kd);
    __enable_irq();
}

/* 读取当前循迹 PID 参数；关中断保证三个参数读取一致。 */
void Track_GetPid(float *kp, float *ki, float *kd)
{
    __disable_irq();
    *kp = track_pid.kp;
    *ki = track_pid.ki;
    *kd = track_pid.kd;
    __enable_irq();
}

#if 0
void Track_SetGyroEnabled(uint8_t enabled)
{
    __disable_irq();
    track_gyro_enabled = (enabled != 0U) ? 1U : 0U;
    track_reset_straight_heading();

    if (track_gyro_enabled == 0U) {
#if (TRACK_TURN_USE_GYRO != 0U)
        if (track_turn_gyro_active != 0U) {
            track_turn_gyro_active = 0U;
            Yaw_EndCritical();
        }
#endif
#if (TRACK_LOST_STRAIGHT_EN != 0U)
        track_lost_use_gyro = 0U;
#endif
    }
    __enable_irq();
}

uint8_t Track_GetGyroEnabled(void)
{
    return track_gyro_enabled;
}
#endif

void Track_SetTurnPwm(int16_t pwm)
{
    __disable_irq();
    track_turn_pwm = pwm;
    __enable_irq();
}

int16_t Track_GetTurnPwm(void)
{
    return track_turn_pwm;
}

void Track_SetTurnStopMs(uint16_t stop_ms)
{
    __disable_irq();
    track_turn_stop_ms = stop_ms;
    __enable_irq();
}

uint16_t Track_GetTurnStopMs(void)
{
    return track_turn_stop_ms;
}

void Track_SetTurnLockMs(uint16_t lock_ms)
{
    __disable_irq();
    track_turn_lock_ms = lock_ms;
    __enable_irq();
}

uint16_t Track_GetTurnLockMs(void)
{
    return track_turn_lock_ms;
}

/*
 * 循迹核心函数：读取灰度、判断直角弯、计算循迹误差，
 * 最终把基础速度 base_speed 合成为左右轮目标速度。
 */
void Track_ApplyTarget(float base_speed, float *target_l, float *target_r)
{
    float turn_output;
    float line_speed;
    float diff;

    *target_l = base_speed;
    *target_r = base_speed;

    track_sensor_bits = gray_sensor_read_line_bits();

    /* 老版行为：未使能或速度为 0 时完整复位循迹/直角弯状态。 */
    if ((track_enabled == 0U) || (base_speed == 0.0f)) {
        track_reset_turn_runtime(1U);
        return;
    }

#if (TRACK_CORNER_EN != 0U)
    /* 非普通循迹状态时，优先运行直角弯状态机。 */
    if (track_state != TRACK_STATE_LINE_FOLLOW) {
        track_process_turning(target_l, target_r);
        return;
    }

    track_update_corner_context(track_sensor_bits);
    /* 普通循迹状态下先检查是否需要进入直角弯。 */
    if (track_detect_corner(base_speed) != 0U) {
        *target_l = g_target_speed;
        *target_r = g_target_speed;
        return;
    }
#endif

    track_error = track_solve_error(track_sensor_bits);

    /*
     * 约定：track_error > 0 表示黑线在车体右侧，需要让车向右修正；
     * track_error < 0 表示黑线在车体左侧，需要让车向左修正。
     */
    line_speed = track_calc_adaptive_speed(base_speed, (float)track_error);
    turn_output = PID_CalcWithDFilter(&track_pid, (float)track_error, 0.0f,
        TRACK_D_FILTER_ALPHA);

    /* diff 为左右轮差速量，正值表示左轮目标速度更高、右轮更低。 */
    diff = turn_output * line_speed * TRACK_K;
    *target_l = line_speed + diff;
    *target_r = line_speed - diff;
}

/* 以下 Getter/Flag 函数主要给 OLED 显示和上层状态判断使用。 */
uint8_t Track_IsEnabled(void)
{
    return track_enabled;
}

uint8_t Track_GetSensorBits(void)
{
    return track_sensor_bits;
}

int16_t Track_GetError(void)
{
    return track_error;
}

uint8_t Track_HasCross(void)
{
    return track_cross_flag;
}

void Track_ClearCross(void)
{
    track_cross_flag = 0U;
}

track_state_t Track_GetState(void)
{
    return track_state;
}

track_line_mode_t Track_GetLineMode(void)
{
    return track_line_mode;
}

float Track_GetTurnAngle(void)
{
    return track_turn_angle_deg;
}

uint8_t Track_GetTurnFail(void)
{
    return track_turn_fail_flag;
}

/* 初始化 OLED，并立即显示一次灰度调试页面。 */
void gray_sensor_debug_init(void)
{
    OLED_Init();
    gray_sensor_debug_update(0U);
}

/* 单独的灰度调试显示：展示 8 路 GPIO 原始电平，不经过黑线极性转换。 */
void gray_sensor_debug_update(uint32_t now_ms)
{
    static uint32_t last_refresh_ms = 0U;
    static uint8_t first_refresh = 1U;
    u8 left_values[] = "0  0  0  0";
    u8 right_values[] = "0  0  0  0";

    if (first_refresh == 0U &&
        (uint32_t)(now_ms - last_refresh_ms) < GRAY_SENSOR_REFRESH_MS) {
        return;
    }
    first_refresh = 0U;
    last_refresh_ms = now_ms;

    left_values[0] = read_gray_pin(GRAY_SENSOR_L4_PORT,
        GRAY_SENSOR_L4_PIN) ? '1' : '0';
    left_values[3] = read_gray_pin(GRAY_SENSOR_L3_PORT,
        GRAY_SENSOR_L3_PIN) ? '1' : '0';
    left_values[6] = read_gray_pin(GRAY_SENSOR_L2_PORT,
        GRAY_SENSOR_L2_PIN) ? '1' : '0';
    left_values[9] = read_gray_pin(GRAY_SENSOR_L1_PORT,
        GRAY_SENSOR_L1_PIN) ? '1' : '0';

    right_values[0] = read_gray_pin(GRAY_SENSOR_R1_PORT,
        GRAY_SENSOR_R1_PIN) ? '1' : '0';
    right_values[3] = read_gray_pin(GRAY_SENSOR_R2_PORT,
        GRAY_SENSOR_R2_PIN) ? '1' : '0';
    right_values[6] = read_gray_pin(GRAY_SENSOR_R3_PORT,
        GRAY_SENSOR_R3_PIN) ? '1' : '0';
    right_values[9] = read_gray_pin(GRAY_SENSOR_R4_PORT,
        GRAY_SENSOR_R4_PIN) ? '1' : '0';

    OLED_ShowString(0, 0, (u8 *)"GRAY SENSOR", 12);
    OLED_ShowString(0, 16, (u8 *)"L4 L3 L2 L1", 12);
    OLED_ShowString(0, 28, left_values, 12);
    OLED_ShowString(0, 40, (u8 *)"R1 R2 R3 R4", 12);
    OLED_ShowString(0, 52, right_values, 12);
    OLED_Refresh();
}

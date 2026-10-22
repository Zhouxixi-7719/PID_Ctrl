#include "pid.h"

/* 通用限幅函数，把 value 限制在 min~max 之间。 */
static float PID_Limit(float value, float min, float max)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

/* 初始化 PID 控制器，所有历史状态通过 PID_Reset 统一清零。 */
void PID_Init(pid_ctrl_t *pid, float kp, float ki, float kd,
    float integral_min, float integral_max, float output_min, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->output_min = output_min;
    pid->output_max = output_max;
    PID_Reset(pid);
}

/* 运行时修改 PID 参数；修改后重置状态，避免旧积分/微分影响新参数。 */
void PID_SetParams(pid_ctrl_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    PID_Reset(pid);
}

/* 清空 PID 的动态状态，不改变 kp/ki/kd 和限幅范围。 */
void PID_Reset(pid_ctrl_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_derivative = 0.0f;
    pid->derivative_ready = 0U;
}

/*
 * 普通 PID 计算。
 * target 是目标值，feedback 是反馈值，error = target - feedback。
 */
float PID_Calc(pid_ctrl_t *pid, float target, float feedback)
{
    float error = target - feedback;
    float derivative = error - pid->previous_error;
    float output;

    /* 积分项直接累加（在每次的10ms的调用中累加）* error，之后做积分限幅。 */
    pid->integral += pid->ki * error;
    pid->integral = PID_Limit(pid->integral,
        pid->integral_min, pid->integral_max);

    /* PID 三项合成后再做输出限幅。 */
    output = pid->kp * error + pid->integral + pid->kd * derivative;
    output = PID_Limit(output, pid->output_min, pid->output_max);

    pid->previous_error = error;

    return output;
}

/*
 * 带 D 项低通滤波的 PID 计算。
 * derivative_alpha 越小，D 项越平滑；越大，D 项越接近原始差分。
 */
float PID_CalcWithDFilter(pid_ctrl_t *pid, float target, float feedback,
    float derivative_alpha)
{
    float error = target - feedback;
    float raw_derivative = error - pid->previous_error;
    float derivative;
    float output;

    derivative_alpha = PID_Limit(derivative_alpha, 0.0f, 1.0f);

    /* 第一次计算没有历史微分，直接把 D 项置 0，避免上电瞬间猛跳。 */
    if (pid->derivative_ready == 0U) {
        derivative = 0.0f;
        pid->derivative_ready = 1U;
    } else {
        /* 一阶 IIR：新微分 = 旧微分 + alpha * (原始微分 - 旧微分)。 */
        derivative = pid->previous_derivative +
            derivative_alpha * (raw_derivative - pid->previous_derivative);
    }

    /* 下面的积分、输出限幅逻辑与普通 PID 保持一致。 */
    pid->integral += pid->ki * error;
    pid->integral = PID_Limit(pid->integral,
        pid->integral_min, pid->integral_max);

    output = pid->kp * error + pid->integral + pid->kd * derivative;
    output = PID_Limit(output, pid->output_min, pid->output_max);

    pid->previous_error = error;
    pid->previous_derivative = derivative;

    return output;
}

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

/* 通用 PID 控制器状态结构体，速度环和循迹环都会复用这个结构。 */
typedef struct {
    float kp;                    /* 比例系数：误差越大，输出变化越大。 */
    float ki;                    /* 积分系数：消除长期稳态误差。 */
    float kd;                    /* 微分系数：抑制误差快速变化带来的超调。 */
    float integral;              /* 积分累计值，已经包含 ki。 */
    float previous_error;        /* 上一次误差，用于计算本次微分。 */
    float previous_derivative;   /* 上一次滤波后的微分值。 */
    uint8_t derivative_ready;    /* 微分滤波是否已有历史值，0 表示首次计算。 */
    float integral_min;          /* 积分项下限，防止积分过大。 */
    float integral_max;          /* 积分项上限，防止积分过大。 */
    float output_min;            /* PID 输出下限。 */
    float output_max;            /* PID 输出上限。 */
} pid_ctrl_t;

/* 初始化 PID 参数、积分限幅、输出限幅，并清空历史状态。 */
void PID_Init(pid_ctrl_t *pid, float kp, float ki, float kd,
    float integral_min, float integral_max, float output_min, float output_max);

/* 修改 kp/ki/kd，并清空积分和微分历史，避免旧参数残留影响新参数。 */
void PID_SetParams(pid_ctrl_t *pid, float kp, float ki, float kd);

/* 清空积分、上次误差、微分滤波历史。 */
void PID_Reset(pid_ctrl_t *pid);

/* 普通 PID 计算，D 项直接使用 error - previous_error。 */
float PID_Calc(pid_ctrl_t *pid, float target, float feedback);

/* 带 D 项一阶低通滤波的 PID 计算，用于灰度循迹这种离散跳变误差。 */
float PID_CalcWithDFilter(pid_ctrl_t *pid, float target, float feedback,
    float derivative_alpha);

#endif /* PID_H_ */

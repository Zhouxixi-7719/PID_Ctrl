#ifndef PID_CORE_H_
#define PID_CORE_H_

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float dt_sec;
    float integral;
    float previous_error;
    float integral_min;
    float integral_max;
    float output_min;
    float output_max;
    uint8_t derivative_ready;
} pid_core_t;

void PIDCore_Init(pid_core_t *pid, float kp, float ki, float kd,
    float dt_sec, float integral_min, float integral_max,
    float output_min, float output_max);
void PIDCore_Reset(pid_core_t *pid);
float PIDCore_Run(pid_core_t *pid, float target, float feedback);
float PIDCore_RunConditional(pid_core_t *pid, float target, float feedback,
    uint8_t integrate_enabled);
void PIDCore_DecayIntegral(pid_core_t *pid, float retain_ratio);
void PIDCore_SetOutputLimits(pid_core_t *pid, float output_min,
    float output_max);

#endif

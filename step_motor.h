#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define STEP_MOTOR_MIN_RPM    2.0f /* 低速控制下限，避免小角度时步进过大。 */
/* 所有步进电机的RPM上限；必须不小于各电机的目标速度。 */
#define STEP_MOTOR_MAX_RPM    100.0f /* 所有步进电机的速度总上限。 */

/* 伪内角度环：反馈值来自软件脉冲计数角度。 */
#define STEP_MOTOR2_ANGLE_PID_DT_SEC       0.01f
#define STEP_MOTOR2_ANGLE_DEFAULT_KP       5.0f
#define STEP_MOTOR2_ANGLE_DEFAULT_KI       0.0f
#define STEP_MOTOR2_ANGLE_DEFAULT_KD       0.02f
#define STEP_MOTOR2_ANGLE_INTEGRAL_LIMIT   20.0f
#define STEP_MOTOR2_ANGLE_DEADBAND_DEG     0.15f
#define STEP_MOTOR2_ANGLE_MAX_RPM          40.0f     //步进电机最大转速
#define STEP_MOTOR2_ZERO_OFFSET_DEFAULT_DEG 0.0f /* 滑轨水平零点的默认角度偏移。 */

void step_motor_init(void);
void step_motor_dir_set(uint8_t direction, uint8_t stepper_id);
void step_motor_stop(uint8_t stepper_id);
void step_motor_start(uint8_t stepper_id);
void step_set_speed(uint16_t speed, uint8_t stepper_id);
void step_motor_set_velocity(float rpm, uint8_t stepper_id);
void step_motor2_set_target_angle(float angle_deg);
void step_motor2_stop_position(void);
void step_motor2_angle_pid_reset(void);
void step_motor2_calibrate_zero(void);
float step_motor2_get_current_angle(void);
float step_motor2_get_zero_offset(void);
void step_motor_apply_horizontal_position(uint16_t x);
void step_motor_apply_error(int16_t err_x, int16_t err_y);
void step_motor_stop_aim(void);

#endif

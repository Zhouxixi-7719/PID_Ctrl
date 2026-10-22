#ifndef SPEED_CTRL_H_
#define SPEED_CTRL_H_

#include <stdint.h>

#define SPEED_CTRL_PERIOD_MS           10U
#define SPEED_CTRL_DEFAULT_TARGET_CM_S 50.0f
#define SPEED_CTRL_DEFAULT_KP          80.0f
#define SPEED_CTRL_DEFAULT_KI          20.0f
#define SPEED_CTRL_DEFAULT_KD          0.0f
#define SPEED_CTRL_INTEGRAL_LIMIT      3000.0f
#define SPEED_CTRL_PROFILE_ACCEL_CM_S2   10.0f
#define SPEED_CTRL_PROFILE_DECEL_CM_S2   25.0f
#define SPEED_CTRL_PROFILE_JERK_CM_S3    50.0f
#define SPEED_CTRL_PROFILE_RESPONSE_S     0.30f
#define SPEED_CTRL_PROFILE_TARGET_EPS_CM_S 0.1f
#define SPEED_CTRL_STOP_TARGET_EPS_CM_S   0.5f
#define SPEED_CTRL_STOP_SPEED_EPS_CM_S    2.0f
#define SPEED_CTRL_TURN_PWM_LIMIT      4500  /* 直角状态的PWM安全上限 */

extern volatile float g_target_speed;

void SpeedCtrl_Init(void);
void SpeedCtrl_Start(float target_cm_s);
void SpeedCtrl_Stop(void);
void SpeedCtrl_RequestSmoothStop(void);
void SpeedCtrl_SetRunSpeed(float target_cm_s);
float SpeedCtrl_GetRunSpeed(void);
void SpeedCtrl_SetPid(float kp, float ki, float kd);
void SpeedCtrl_Task10ms(void);
uint8_t SpeedCtrl_IsEnabled(void);
uint8_t SpeedCtrl_IsAtRest(void);
float SpeedCtrl_GetTarget(void);
float SpeedCtrl_GetTargetL(void);
float SpeedCtrl_GetTargetR(void);
float SpeedCtrl_GetSpeedL(void);
float SpeedCtrl_GetSpeedR(void);
float SpeedCtrl_GetProfileAcceleration(void);
int16_t SpeedCtrl_GetPwmL(void);
int16_t SpeedCtrl_GetPwmR(void);

#endif /* SPEED_CTRL_H_ */

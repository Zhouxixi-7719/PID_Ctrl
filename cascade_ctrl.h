#ifndef CASCADE_CTRL_H_
#define CASCADE_CTRL_H_

#include <stdint.h>

/* Select the serial field used as the ball position. */
#define CASCADE_INPUT_DISTANCE_MM       0U
#define CASCADE_INPUT_MOVE_Y            1U
#define CASCADE_INPUT_MODE              CASCADE_INPUT_DISTANCE_MM

#if (CASCADE_INPUT_MODE != CASCADE_INPUT_DISTANCE_MM) && \
    (CASCADE_INPUT_MODE != CASCADE_INPUT_MOVE_Y)
#error "CASCADE_INPUT_MODE must select one explicit position field"
#endif

/* Position input and safety limits. */
#define CASCADE_TARGET_POSITION         107.0f
#define CASCADE_INPUT_MIN                 0.0f
#define CASCADE_INPUT_MAX               999.0f
#define CASCADE_INPUT_TIMEOUT_MS        400U

/* The main loop schedules the controller at 5 ms. */
#define CASCADE_INNER_PERIOD_MS           5U
#define CASCADE_OUTER_PERIOD_MS          10U

/* Ball position error -> rail target angle. */
#define CASCADE_POSITION_KP_DEG_PER_MM     0.28f
#define CASCADE_POSITION_DEADBAND_MM       2.0f             //死区保持区
#define CASCADE_POSITION_HOLD_EXIT_MM      3.0f             //死区退出区
#define CASCADE_POSITION_KI_DEG_PER_MM_S   0.52f            //积分速度
#define CASCADE_POSITION_I_MAX_DEG         4.0f             //条件积分最大角度
#define CASCADE_POSITION_I_ERROR_MAX_MM   30.0f
#define CASCADE_POSITION_I_VELOCITY_MAX   10.0f
#define CASCADE_POSITION_I_CONFIRM_MS     50U
#define CASCADE_POSITION_I_FF_PAUSE_DEG    0.20f
#define CASCADE_POSITION_I_SLOPE_RESIDUAL_MAX_MPS2  0.05f
#define CASCADE_POSITION_KD_DEG_PER_MM_PER_S 0.025f
#define CASCADE_POSITION_D_MAX_DEG          3.0f
#define CASCADE_STICTION_ERROR_MIN_MM       9.0f
#define CASCADE_STICTION_VELOCITY_MAX       5.0f
#define CASCADE_STICTION_CONFIRM_MS        60U
#define CASCADE_STICTION_MIN_ANGLE_DEG      1.20f
#define CASCADE_TARGET_ANGLE_MAX_DEG       18.0f             //总倾角限制
#define CASCADE_TARGET_ANGLE_SLEW_DEG      0.50f
#define CASCADE_ANGLE_DIRECTION_INVERT     0U

/* Rail angle error -> stepper speed. */
#define CASCADE_ANGLE_KP_RPM_PER_DEG      8.0f
#define CASCADE_ANGLE_DEADBAND_DEG         0.10f
#define CASCADE_RPM_MAX                   30.0f
#define CASCADE_RPM_SLEW                   5.0f

/* Filtered ball velocity is used by the derivative braking term. */
#define CASCADE_VELOCITY_FILTER_ALPHA      0.25f
#define CASCADE_VELOCITY_MAX             500.0f
#define CASCADE_HOLD_VELOCITY_MAX          6.0f

/* Vehicle acceleration feedforward from the speed-profile command. */      /*启动/停车前馈*/
#define CASCADE_MOTION_FF_ENABLE            1U
#define CASCADE_VEHICLE_FF_FORWARD_PROJECTION  0.70710678f
#define CASCADE_VEHICLE_FF_GAIN             5.05f
#define CASCADE_VEHICLE_FF_DIRECTION        -1.0f
#define CASCADE_VEHICLE_FF_MAX_DEG          100.0f
#define CASCADE_GRAVITY_MPS2                9.80665f
#define CASCADE_RAD_TO_DEG                 57.2957795f

/* Relative slope estimate from the startup-referenced MPU acceleration. */
#define CASCADE_SLOPE_AXIS_X                 0.70710678f
#define CASCADE_SLOPE_AXIS_Y                 0.70710678f
#define CASCADE_SLOPE_AXIS_Z                 0.0f
#define CASCADE_SLOPE_FILTER_ALPHA           0.02f
#define CASCADE_SLOPE_REST_CONFIRM_MS      500U
#define CASCADE_SLOPE_CMD_ACCEL_MAX_MPS2     0.03f
#define CASCADE_SLOPE_FF_DIRECTION           1.0f
#define CASCADE_SLOPE_FF_MAX_DEG             2.0f

/* Small measured-acceleration trim after the command feedforward starts. */
#define CASCADE_IMU_TRIM_GAIN                0.25f
#define CASCADE_IMU_TRIM_MAX_DEG             0.50f
#define CASCADE_IMU_TRIM_START_DELAY_MS    100U
#define CASCADE_IMU_TRIM_SLEW_DEG            0.02f
#define CASCADE_IMU_COMMAND_MATCH_SIGN        1.0f

void CascadeCtrl_Init(void);
uint8_t CascadeCtrl_SetSerialPosition(uint16_t position, uint8_t valid,
    uint32_t now_ms);
void CascadeCtrl_SetTarget(float target_position);
void CascadeCtrl_SetEnabled(uint8_t enabled);
void CascadeCtrl_SetMotionFeedforwardEnabled(uint8_t enabled);
void CascadeCtrl_Task5ms(uint32_t now_ms);
void CascadeCtrl_Stop(void);

float CascadeCtrl_GetRawPosition(void);
float CascadeCtrl_GetTarget(void);
uint8_t CascadeCtrl_IsEnabled(void);
float CascadeCtrl_GetPositionError(void);
float CascadeCtrl_GetPositionVelocity(void);
float CascadeCtrl_GetVelocityReference(void);
float CascadeCtrl_GetAngleReference(void);
float CascadeCtrl_GetAngleFeedback(void);
float CascadeCtrl_GetRpmCommand(void);
float CascadeCtrl_GetEstimatedPosition(void);
float CascadeCtrl_GetPredictedPosition(void);
float CascadeCtrl_GetBallAcceleration(void);
float CascadeCtrl_GetBallFeedforwardAcceleration(void);
float CascadeCtrl_GetBallFeedforwardAngle(void);
float CascadeCtrl_GetRailAcceleration(void);
float CascadeCtrl_GetFeedforwardAngle(void);
float CascadeCtrl_GetPositionIntegralAngle(void);
float CascadeCtrl_GetPositionDerivativeAngle(void);
float CascadeCtrl_GetSlopeCompensationAngle(void);
float CascadeCtrl_GetImuTrimAngle(void);
uint8_t CascadeCtrl_IsAngleSaturated(void);
uint8_t CascadeCtrl_IsRecoveryActive(void);
uint8_t CascadeCtrl_IsHoldActive(void);
uint8_t CascadeCtrl_IsInputValid(void);

#endif

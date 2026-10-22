#ifndef BALL_STEP_CTRL_H_
#define BALL_STEP_CTRL_H_

#include <stdint.h>

/*
 * 小球位置控制器参数。
 * 这里的输出角度是滑轨角度，不是步进电机轴角度。
 */

/* ==================== 1. 基础控制与安全边界 ==================== */
#define BALL_STEP_CTRL_PERIOD_MS              10U /* 控制任务周期，单位：毫秒。 */
#define BALL_STEP_INPUT_TIMEOUT_MS            400U /* 超时没有有效数据时停止输出。 */
#define BALL_STEP_DEFAULT_REFERENCE           100U /* 默认小球目标位置。 */
#define BALL_STEP_DEFAULT_ENABLED             1U /* 默认使能控制器，1为使能。 */
#define BALL_STEP_OUTPUT_ANGLE_MIN_DEG        (-18.0f) /* 滑轨最小输出角度，单位：度。 */
#define BALL_STEP_OUTPUT_ANGLE_MAX_DEG        (30.0f) /* 滑轨最大输出角度，单位：度。 */

/* ==================== 2. 位置环 PID 参数 ==================== */
#define BALL_STEP_DEFAULT_KP                  12.0f /* 位置比例增益，越大响应越强。 */
#define BALL_STEP_KP_MAX                      70.0f /* Kp 的最大可设置值。 */
#define BALL_STEP_DEFAULT_KI                  0.1f /* 位置积分增益，用于消除稳态误差。 */
#define BALL_STEP_KI_MAX                      5.0f /* Ki 的最大可设置值。 */
#define BALL_STEP_DEFAULT_KD                  0.0f /* 位置微分增益，用于抑制运动和过冲。 */
#define BALL_STEP_KD_MAX                      20.0f /* Kd 的最大可设置值。 */
#define BALL_STEP_INTEGRAL_LIMIT              60.0f /* 积分限幅，防止积分饱和。 */
#define BALL_STEP_DERIVATIVE_FILTER_ALPHA     0.25f /* 微分项低通滤波系数。 */
#define BALL_STEP_PID_DT_SEC                  ((float)BALL_STEP_CTRL_PERIOD_MS / 1000.0f) /* PID 计算周期，单位：秒。 */

/* ==================== 3. 位置预测与速度估计 ==================== */
#define BALL_STEP_PREDICTION_TIME_SEC         0.30f /* 位置预测时间，越大越早修正。 */
#define BALL_STEP_VELOCITY_FILTER_ALPHA       0.25f /* 速度滤波系数，越大响应越快但噪声越多。 */
#define BALL_STEP_VELOCITY_DEADBAND           0.05f /* 低于该速度时不触发速度刹车。 */
#define BALL_STEP_VELOCITY_MAX                500.0f /* 估计速度的最大限幅。 */

/* ==================== 4. 卡尔曼滤波参数 ==================== */
#define BALL_STEP_KALMAN_MEASUREMENT_NOISE    0.5f /* 测量噪声方差，越大越不信任视觉测量。 */
#define BALL_STEP_KALMAN_ACCELERATION_STDDEV 600.0f /* 加速度标准差，越大越容易跟随快速运动。 */
#define BALL_STEP_KALMAN_INITIAL_VELOCITY_VARIANCE 400.0f /* 初始速度不确定度。 */

/* ==================== 5. 提前刹车与运动阻尼 ==================== */
#define BALL_STEP_BRAKE_LEAD_TIME_SEC         0.55f /* 刹车提前时间，越大越早刹车，用于补偿延迟。 */
#define BALL_STEP_BRAKE_MIN_DISTANCE          2.0f /* 刹车距离计算使用的最小距离。 */
#define BALL_STEP_EFFECTIVE_DECEL_PER_DEG     40.0f /* 滑轨每倾斜 1 度对应的估计减速度。 */
#define BALL_STEP_BRAKE_ANGLE_GAIN            0.5f /* 距离刹车角度增益，越大刹车越强。 */
#define BALL_STEP_USE_VELOCITY_BRAKE          0U /* 速度刹车开关，1为启用，0为关闭。 */
#define BALL_STEP_VELOCITY_BRAKE_GAIN         0.06f /* 速度刹车增益，速度越大反向角度越大。 */
#define BALL_STEP_VELOCITY_BRAKE_MAX_ANGLE    20.0f /* 速度刹车角度的最大限幅。 */

/* ==================== 6. 非线性输出与方向补偿 ==================== */
#define BALL_STEP_NONLINEAR_OUTPUT_ENABLE     0U /* 位置输出非线性缩放开关。 */
#define BALL_STEP_NONLINEAR_ERROR_RANGE       20.0f /* 误差达到该值时恢复完整位置 PID 输出。 */
#define BALL_STEP_NONLINEAR_MIN_SCALE         0.25f /* 小误差时位置 PID 输出保留的最小比例。 */
#define BALL_STEP_ASYMMETRIC_OUTPUT_ENABLE    0U /* 两个方向使用不同输出力度的开关。 */
#define BALL_STEP_LIMITED_DIRECTION_POSITIVE  1U /* 物理正角度方向是否为空间受限方向。 */
#define BALL_STEP_LIMITED_POSITION_GAIN_SCALE 0.50f /* 受限方向的位置 PID 输出比例。 */
#define BALL_STEP_FREE_POSITION_GAIN_SCALE    1.00f /* 空间较大方向的位置 PID 输出比例。 */
#define BALL_STEP_LIMITED_OUTPUT_ANGLE_DEG    18.0f /* 受限方向的最大滑轨倾角。 */
#define BALL_STEP_RECENTER_MIN_ANGLE_DEG      0.50f /* 合成角度过小时仍用于回中心的最小角度。 */

/* ==================== 7. 目标附近稳定状态 ==================== */
#define BALL_STEP_SETTLE_ENABLE               0U /* 目标附近稳定状态开关。 */
#define BALL_STEP_SETTLE_ENTER_ERROR          3.0f /* 进入稳定状态的位置误差阈值。 */
#define BALL_STEP_SETTLE_EXIT_ERROR           5.0f /* 退出稳定状态的位置误差阈值。 */
#define BALL_STEP_SETTLE_ENTER_VELOCITY       15.0f /* 进入稳定状态的速度阈值。 */
#define BALL_STEP_SETTLE_EXIT_VELOCITY        25.0f /* 退出稳定状态的速度阈值。 */
#define BALL_STEP_SETTLE_MAX_ANGLE            2.0f /* 稳定状态下目标角度的最大值。 */
#define BALL_STEP_SETTLE_CONFIRM_COUNT        10U /* 连续满足条件多少次后进入稳定状态。 */

/* ==================== 8. 震荡幅度自适应输出 ==================== */
#define BALL_STEP_ADAPTIVE_OUTPUT_ENABLE      0U /* 震荡幅度自适应输出开关。 */
#define BALL_STEP_ADAPTIVE_ERROR_RANGE        20.0f /* 误差达到该值时自适应比例恢复为 1。 */
#define BALL_STEP_ADAPTIVE_MIN_SCALE          0.25f /* 自适应输出比例的最小值。 */
#define BALL_STEP_ADAPTIVE_SCALE_ALPHA        0.25f /* 每次过零时比例的调整速度。 */
#define BALL_STEP_ADAPTIVE_ZERO_CROSS_ERROR   0.5f /* 小于该误差时不判定新的方向过零。 */

/* ==================== 9. 输出变化速度与方向 ==================== */
#define BALL_STEP_DEFAULT_DEADBAND            3.0f /* 位置误差死区。 */
#define BALL_STEP_MAX_ANGLE_STEP_DEG          1.5f /* 每个控制周期允许的最大角度变化。 */
#define BALL_STEP_DIRECTION_INVERT            0U /* 位置控制方向反向开关，1为反向。 */
#define BALL_STEP_ERROR_DIRECTION_INVERT      0U /* 直接误差方向反向开关，1为反向。 */

void BallStepCtrl_Init(void);
void BallStepCtrl_Task10ms(uint32_t now_ms);
void BallStepCtrl_UpdatePosition(uint16_t position, uint8_t valid,
    uint32_t now_ms);
void BallStepCtrl_UpdateError(int16_t error, uint8_t valid,
    uint32_t now_ms);
void BallStepCtrl_Stop(void);

void BallStepCtrl_SetEnabled(uint8_t enabled);
uint8_t BallStepCtrl_IsEnabled(void);
void BallStepCtrl_SetReference(int16_t reference);
int16_t BallStepCtrl_GetReference(void);
void BallStepCtrl_SetKp(float kp);
float BallStepCtrl_GetKp(void);
void BallStepCtrl_SetKi(float ki);
float BallStepCtrl_GetKi(void);
void BallStepCtrl_SetKd(float kd);
float BallStepCtrl_GetKd(void);
float BallStepCtrl_GetFilteredPosition(void);
float BallStepCtrl_GetVelocity(void);
float BallStepCtrl_GetPredictedError(void);
float BallStepCtrl_GetBrakingAngle(void);
uint16_t BallStepCtrl_GetRawPosition(void);
uint8_t BallStepCtrl_IsPositionValid(void);
float BallStepCtrl_GetError(void);
float BallStepCtrl_GetOutputAngle(void);
uint8_t BallStepCtrl_IsSettling(void);
float BallStepCtrl_GetAdaptiveScale(void);

#endif /* BALL_STEP_CTRL_H_ */

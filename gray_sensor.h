#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include <stdint.h>

/* 灰度模块输出黑线时的有效电平：1 表示检测到黑线时 GPIO 为高电平。 */
#define GRAY_SENSOR_BLACK_LEVEL        1U

/* 循迹位置环 PID 初始参数，实际运行时仍可通过按键调参缓存修改。 */
#define TRACK_TURN_KP                  0.95f
#define TRACK_TURN_KI                  0.0f
#define TRACK_TURN_KD                  1.0f

/* 循迹 PID 输出到左右轮差速的比例系数。 */
#define TRACK_K                        0.1f

/* 老版本循迹主链不使用陀螺仪修正。 */
#define TRACK_GYRO_DAMP_EN             0U
#define TRACK_GYRO_DAMP_K              0.03f
#define TRACK_GYRO_DAMP_CENTER_ERROR   1
#define TRACK_GYRO_DAMP_MIN_ERROR      3
#define TRACK_GYRO_DAMP_MIN_RATE_DPS   25.0f
#define TRACK_CENTER_GYRO_DAMP_K        0.06f                   //抑制摆动的角速度阻尼
#define TRACK_CENTER_GYRO_RATE_DEADBAND_DPS 3.0f

/*
 * 直线航向保持：中心灰度稳定且角速度足够小时锁存当前航向。
 * 之后用航向误差和角速度阻尼抵抗机械偏航，进入弯道或丢线时释放。
 */
#define TRACK_STRAIGHT_HEADING_HOLD_EN  0U
#define TRACK_STRAIGHT_HEADING_CAPTURE_N 5U
#define TRACK_STRAIGHT_HEADING_CAPTURE_RATE_DPS 8.0f
#define TRACK_STRAIGHT_HEADING_KP       0.35f                   //直线航向拉回力度
#define TRACK_STRAIGHT_HEADING_ERROR_MAX_DEG 10.0f
#define TRACK_STRAIGHT_HEADING_FADE_START_ERROR 1.0f
#define TRACK_STRAIGHT_HEADING_FADE_END_ERROR   3.0f

#define TRACK_DIFF_SLEW_EN             0U
#define TRACK_DIFF_SLEW_MIN_CM_S       4.0f
#define TRACK_DIFF_SLEW_SPEED_RATIO    0.25f
#define TRACK_CURVE_CENTER_FAST_ERROR  2
#define TRACK_STRAIGHT_DIFF_SLEW_EN    0U
#define TRACK_STRAIGHT_DIFF_SLEW_MIN_CM_S 3.0f
#define TRACK_STRAIGHT_DIFF_SLEW_SPEED_RATIO 0.10f
#define TRACK_STRAIGHT_DIFF_LIMIT_RATIO 0.30f
#define TRACK_ERROR_FILTER_ALPHA       0.55f
#define TRACK_STRAIGHT_ERROR_DEADBAND  0.20f
#define TRACK_STRAIGHT_CENTER_ERROR    1.0f
#define TRACK_STRAIGHT_CENTER_OUTPUT_RATIO 0.65f

#define TRACK_LINE_CURVE_ENTER_ERROR   4
#define TRACK_LINE_CURVE_ENTER_N       4U
#define TRACK_LINE_CURVE_SENSOR_MASK   0xC3U
#define TRACK_LINE_CURVE_SENSOR_ERROR  3
#define TRACK_LINE_STRAIGHT_ERROR      1
#define TRACK_LINE_STRAIGHT_N          6U

/* D 项一阶低通系数：越小越平滑，越大越跟手；范围建议 0~1。 */
#define TRACK_D_FILTER_ALPHA           0.35f

/* 弯道自适应降速开关：1 开启，0 关闭后循迹速度不随误差变化。 */
#define TRACK_SPEED_ADAPTIVE_EN        1U

/* 自适应降速最低比例：误差最大时，前进速度最低降到基础速度的 55%。 */
#define TRACK_SPEED_MIN_RATIO          0.55f

/* 误差归一化基准：8 路权重最大绝对值为 7，所以这里用 7。 */
#define TRACK_SPEED_FULL_ERROR         7.0f

/* 左右轮差速最大比例，用来限制循迹 PID 输出，避免目标速度差过大。 */
#define TRACK_DIFF_LIMIT_RATIO         1.4f
#define TRACK_TURN_OUTPUT_LIMIT        (TRACK_DIFF_LIMIT_RATIO / TRACK_K)

/* 直角弯识别与原地转弯功能开关。 */
#define TRACK_CORNER_EN                0U

/* 左右轮中心距，编码器估算转向角度时使用，单位 cm。 */
#define TRACK_WHEEL_BASE_CM            13.5f

/*
 * 直角弯转向角的来源：1 用陀螺仪，0 用编码器差速估算。
 *
 * 原地转弯是轮子最容易打滑的工况，打滑时编码器照样计数但车没转到位，
 * 估出来的角度偏大。陀螺仪测的是车体真实角速度，不受打滑影响。
 * 陀螺仪未标定或读失败时会自动回退到编码器估算，不影响转弯能跑完。
 */
#define TRACK_TURN_USE_GYRO            0U

/*
 * 陀螺仪 Z 轴与编码器角度约定的符号关系。
 *
 * 编码器约定 (right_cm - left_cm) 为正，即右轮快、车左转(逆时针)为正。
 * 陀螺仪符号取决于模块的安装方向，需实测：进 ENC/MPU TEST 页，
 * 手动把车逆时针(左)转约 90°，看 AZ 显示为正还是负。
 * 显示为正填 1.0f，为负填 -1.0f。
 */
#define TRACK_GYRO_SIGN                1.0f

/*
 * 丢线航向保持开关：1 开启。
 *
 * 关闭后退回原有策略——丢线时冻结最后一次灰度误差继续喂给 PID。
 * 那是开环猜测：误差恒定不变，积分持续累积，车会越转越猛，而且
 * 完全不知道车头朝哪。开启后改为锁存丢线瞬间的航向角做闭环保持。
 *
 * 陀螺仪未标定或读故障时会自动退回冻结误差策略。
 */
#define TRACK_LOST_STRAIGHT_EN         0U

/* 连续多少个周期 8 路全白才认定丢线。防虚线、反光、过缝隙的误判。 */
#define TRACK_LOST_ENTER_N             3U

/* 连续多少个周期重新压线才认定找回中线。同样是防抖。 */
#define TRACK_LOST_EXIT_N              2U

/*
 * 丢线直行的超时上限，单位 ms。超时说明已经跑出赛道，直接停车，
 * 否则车会一路直冲。作用等同于直角弯的 TRACK_TURN_ANGLE_MAX_DEG。
 */
#define TRACK_LOST_TIMEOUT_MS          10000U

/*
 * 丢线直行允许的最大差速比例(相对基础速度)。
 *
 * 这是防原地打转的硬保护。灰度环的 TRACK_DIFF_LIMIT_RATIO 是 1.4，
 * 允许 diff 超过基础速度、让一侧轮子反转——那是直角弯急转需要的权限。
 * 但"保持直线"绝不需要反转：一旦某侧反转，车就从"直行修正"变成
 * "原地旋转"，而且航向误差还在增大，会一直转到超时。
 *
 * 取 0.5 表示最坏情况下 target 落在 [0.5, 1.5] 倍基础速度之间，
 * 两轮恒为正转，车只会画一道弧线而不会掉头。
 */
#define TRACK_LOST_DIFF_RATIO          0.5f
#define TRACK_LOST_OUTPUT_LIMIT        (TRACK_LOST_DIFF_RATIO / TRACK_K)

/*
 * 航向保持 PID。
 *
 * 注意 Kp 的量级不能照搬灰度环：灰度误差是无量纲权重(满量程 ±7)，
 * 而航向误差的单位是度。按 15° 偏差对应约 0.4 倍基础速度的差速来算：
 *   diff = out * base * TRACK_K = 0.4 * base  ->  out = 2.0
 *   Kp   = 2.0 / 15° ≈ 0.13
 * 所以这里取 0.15 起调，而不是灰度环那个量级的 1.3。
 *
 * 不用积分项：丢线是短时状态，积分来不及起作用，反而会在退出时
 * 留下残留；而且这个环没有需要消除的稳态误差。
 */
#define TRACK_LOST_KP                  0.15f
#define TRACK_LOST_KI                  0.0f
#define TRACK_LOST_KD                  0.10f

/* 直角弯完成后的冷却周期数，避免刚出弯马上重复识别直角弯。 */
#define TRACK_CORNER_COOLDOWN_N        30U

/* 直角弯原地转动时使用的固定 PWM。 */
#define TRACK_TURN_PWM                 2300

/* 检测到直角弯后先停车的时间，单位 ms。 */
#define TRACK_TURN_STOP_MS             50U

/* 开始转弯后的锁定时间，锁定期内不允许通过中间传感器提前出弯。 */
#define TRACK_TURN_LOCK_MS             60U

/* 中间传感器连续检测到黑线多少次后确认出弯。 */
#define TRACK_TURN_CENTER_CONFIRM_N    1U

/* 出弯检测只看中间两路 L1/R1；bit4=L1，bit3=R1，所以 mask 为 0x18。 */
#define TRACK_TURN_EXIT_CENTER_MASK    0x18U

/* 编码器估算的最大转向角，超过仍没找到中线则强制结束并置失败标志。 */
#define TRACK_TURN_ANGLE_MAX_DEG       100.0f

/* 弧度转角度系数。 */
#define TRACK_RAD_TO_DEG               57.29578f

typedef enum {
    TRACK_LINE_MODE_STRAIGHT = 0,
    TRACK_LINE_MODE_CURVE
} track_line_mode_t;

typedef enum {
    TRACK_STATE_LINE_FOLLOW = 0,       /* 正常灰度循迹状态。 */
    TRACK_STATE_TURN_STOP_LEFT,        /* 检测到左直角弯后的短暂停车状态。 */
    TRACK_STATE_TURN_STOP_RIGHT,       /* 检测到右直角弯后的短暂停车状态。 */
    TRACK_STATE_TURNING_LEFT,          /* 左直角弯原地转向状态。 */
    TRACK_STATE_TURNING_RIGHT,         /* 右直角弯原地转向状态。 */
    TRACK_STATE_LOST_STRAIGHT          /* 丢线后靠陀螺仪保持航向直行。 */
} track_state_t;

/* 初始化循迹 PID 和内部状态。 */
void Track_Init(void);

/* 使能循迹功能，并清除上一次循迹/转弯运行状态。 */
void Track_Enable(void);

/* 关闭循迹功能，并清除上一次循迹/转弯运行状态。 */
void Track_Disable(void);

/* 手动复位循迹状态，常用于重新开始一轮测试。 */
void Track_Reset(void);

/* 设置循迹 PID 参数，按键调参模块最终会调用这里。 */
void Track_SetPid(float kp, float ki, float kd);

/* 读取当前实际生效的循迹 PID 参数，用于 OLED 显示。 */
void Track_GetPid(float *kp, float *ki, float *kd);

/* 运行时开关循迹所使用的陀螺仪反馈；关闭后丢线/转角自动回退到编码器。 */
void Track_SetTurnPwm(int16_t pwm);
int16_t Track_GetTurnPwm(void);
void Track_SetTurnStopMs(uint16_t stop_ms);
uint16_t Track_GetTurnStopMs(void);
void Track_SetTurnLockMs(uint16_t lock_ms);
uint16_t Track_GetTurnLockMs(void);

/* 根据基础速度计算当前循迹差速，主要用于外部快速获取转向量。 */
float Track_CalcDiff(float target_speed);

/* 根据灰度传感器和基础速度，合成左右轮目标速度。 */
void Track_ApplyTarget(float base_speed, float *target_l, float *target_r);

/* 查询循迹是否使能。 */
uint8_t Track_IsEnabled(void);

/* 获取 8 路灰度有效位，bit7~bit0 对应 L4 L3 L2 L1 R1 R2 R3 R4。 */
uint8_t Track_GetSensorBits(void);

/* 获取当前循迹误差，负数表示线偏左，正数表示线偏右。 */
int16_t Track_GetError(void);

/* 查询是否检测到大面积黑线/十字。 */
uint8_t Track_HasCross(void);

/* 清除十字标志。 */
void Track_ClearCross(void);

/* 获取当前循迹/直角弯状态机状态。 */
track_state_t Track_GetState(void);
track_line_mode_t Track_GetLineMode(void);

/* 获取当前编码器估算的直角弯转向角度。 */
float Track_GetTurnAngle(void);

/* 查询直角弯是否因超过最大角度而失败退出。 */
uint8_t Track_GetTurnFail(void);

/* 灰度传感器单独调试显示初始化。 */
void gray_sensor_debug_init(void);

/* 灰度传感器单独调试显示刷新，now_ms 为系统毫秒计数。 */
void gray_sensor_debug_update(uint32_t now_ms);

#endif

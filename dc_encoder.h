#ifndef DC_ENCODER_H_
#define DC_ENCODER_H_

#include <stdint.h>

#define DC_ENCODER_LINES_PER_REV       13.0f
#define DC_ENCODER_GEAR_RATIO          8.56f
#define DC_ENCODER_QUADRATURE          4.0f
#define DC_WHEEL_DIAMETER_CM           6.5f
#define DC_ENCODER_SAMPLE_PERIOD_S     0.01f
#define DC_ENCODER_PI                  3.1415926f
#define DC_ENCODER_DIR_L               1
#define DC_ENCODER_DIR_R               -1
#define DC_ENCODER_SWAP_LR             1U
#define DC_ENCODER_COUNTS_PER_REV      (DC_ENCODER_LINES_PER_REV * \
                                         DC_ENCODER_GEAR_RATIO * \
                                         DC_ENCODER_QUADRATURE)
#define DC_WHEEL_CIRCUMFERENCE_CM      (DC_WHEEL_DIAMETER_CM * DC_ENCODER_PI)

/*
 * 测速滑动窗口长度(单位：采样周期)。
 *
 * 每圈 13*8.56*4 = 445 个计数，轮周长 20.4cm。15cm/s 时每个 10ms
 * 周期只有约 3.3 个计数，也就是说速度反馈的量化步长高达
 * 20.4/445/0.01 = 4.6 cm/s —— 反馈只能落在 0/4.6/9.2/13.8 这些
 * 离散点上。左右轮各自在 3 和 4 个计数之间抖动，换算出来就差
 * 4.6cm/s，速度环 PID 会把这个量化噪声直接放大成 PWM 乱跳。
 *
 * 用 N 个周期的计数总和算速度，量化误差降到 1/N，代价是速度反馈
 * 多出约 N/2 个周期的延迟。5 个周期 = 50ms 窗口约 16 个计数，
 * 误差降到 0.9cm/s，25ms 的延迟对循迹车可以接受。
 */
#define DC_ENCODER_SPEED_WINDOW_N      5U

#if ((DC_ENCODER_DIR_L != 1) && (DC_ENCODER_DIR_L != -1))
#error "DC_ENCODER_DIR_L must be 1 or -1"
#endif

#if ((DC_ENCODER_DIR_R != 1) && (DC_ENCODER_DIR_R != -1))
#error "DC_ENCODER_DIR_R must be 1 or -1"
#endif

#if ((DC_ENCODER_SWAP_LR != 0U) && (DC_ENCODER_SWAP_LR != 1U))
#error "DC_ENCODER_SWAP_LR must be 0U or 1U"
#endif

void dc_encoder_init(void);
int32_t dc_encoder_get_count_l(void);
int32_t dc_encoder_get_count_r(void);
int32_t dc_encoder_get_delta_l(void);
int32_t dc_encoder_get_delta_r(void);
int32_t Encoder_GetLastDelta_L(void);
int32_t Encoder_GetLastDelta_R(void);
void Encoder_TestReset(void);
int32_t Encoder_TestGetCountL(void);
int32_t Encoder_TestGetCountR(void);
float Encoder_GetSpeed_L(void);
float Encoder_GetSpeed_R(void);

/*
 * 清空测速滑动窗口。原地转弯时左右轮反向，窗口里会残留反向计数，
 * 出弯后必须清掉，否则前几十毫秒的速度反馈是错的。
 */
void Encoder_ResetSpeedWindow(void);

#endif /* DC_ENCODER_H_ */

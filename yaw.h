#ifndef YAW_H_
#define YAW_H_

#include <stdint.h>

/* 积分周期，和 SpeedCtrl_Task10ms 同频 */
#define YAW_PERIOD_MS                  10U

/* 温度补偿刷新周期。自热是分钟量级的过程，1 秒一次足够 */
#define YAW_TEMP_UPDATE_MS             1000U

/*
 * 单次积分允许的最大 dt。主循环被 OLED 刷新之类的慢操作堵住时，
 * dt 会异常变大，超过这个值就按上限截断，避免一次性积进一大块角度。
 */
#define YAW_MAX_DT_MS                  100U

/*
 * I2C 读失败后的重试间隔。MPU 没接或掉线时，每次读都要跑满超时
 * (约十几毫秒)，若还按 10ms 周期硬试会把主循环拖死，所以降频重试。
 */
#define YAW_FAULT_RETRY_MS             1000U

void Yaw_Init(void);

/*
 * 在 SysTick 的 10ms 节拍里调用。只置标志位，不做任何 I2C 操作——
 * MPU 的读是阻塞式轮询，100kHz 下读 2 字节约 400us，放在中断里
 * 会长时间堵住 ISR，影响 UART 接收和其它定时任务。
 */
void Yaw_Tick(void);

/* 在主循环里调用，真正执行 I2C 读和积分。now_ms 传 system_ms */
void Yaw_Task(uint32_t now_ms);

/*
 * 关键区标记。
 *
 * 角度始终由主循环的 Yaw_Task 更新。问题在于主循环会被 OLED 刷屏
 * 堵住几十毫秒，而转弯时角速度可达 200°/s，这点延迟就是十几度误差。
 *
 * 解决办法不是把 I2C 搬进中断(那会因为阻塞轮询卡死 1ms 的 SysTick)，
 * 而是在直角弯这种短暂时段让 OLED 让路：显示模块查询 Yaw_IsCritical()
 * 为真时跳过刷新，主循环一圈就回到 1ms 内，Yaw_Task 自然准时。
 * 转弯只有几百毫秒，屏幕短暂不刷新没有影响。
 */
void Yaw_BeginCritical(void);
void Yaw_EndCritical(void);
uint8_t Yaw_IsCritical(void);

/* 累积航向角，单位 °，逆时针为正（取决于模块安装方向） */
float Yaw_GetAngle(void);

/* 最近一次读到的角速度，单位 °/s，已减零偏、已过死区 */
float Yaw_GetRate(void);

/* Startup-pose-referenced chassis acceleration from the shared MPU sample. */
float Yaw_GetAccelX(void);
float Yaw_GetAccelY(void);
float Yaw_GetAccelZ(void);

/* 角度清零，不影响零偏标定 */
void Yaw_Reset(void);

/* 最近一次读是否失败，1 表示当前处于降频重试状态 */
uint8_t Yaw_IsFaulted(void);

/*
 * 与外部绝对参考(编码器差速航向 / 视觉)做一阶互补滤波。
 * alpha 越接近 1 越信任陀螺仪，典型 0.98。
 * 需要拿到轮距和编码器换算系数后，在 10ms 任务里周期性调用。
 */
void Yaw_FuseReference(float reference_deg, float alpha);

#endif /* YAW_H_ */

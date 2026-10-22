#include "yaw.h"

#include "mpu6050.h"

static volatile uint8_t yaw_pending = 0U;
static uint8_t yaw_started = 0U;
static uint8_t yaw_faulted = 0U;
static uint32_t yaw_last_ms = 0U;
static uint32_t yaw_last_temp_ms = 0U;              //上次更新温度补偿的时刻
static uint32_t yaw_last_try_ms = 0U;
static volatile float yaw_angle_deg = 0.0f;
static volatile float yaw_rate_dps = 0.0f;
static volatile float yaw_accel_x_mps2 = 0.0f;
static volatile float yaw_accel_y_mps2 = 0.0f;
static volatile float yaw_accel_z_mps2 = 0.0f;

/* 关键区标记：转弯期间置位，OLED 见到它就跳过刷新 */
static volatile uint8_t yaw_critical = 0U;

void Yaw_Init(void)
{
    yaw_pending = 0U;
    yaw_started = 0U;
    yaw_faulted = 0U;
    yaw_critical = 0U;
    yaw_last_ms = 0U;
    yaw_last_temp_ms = 0U;
    yaw_last_try_ms = 0U;
    yaw_angle_deg = 0.0f;
    yaw_rate_dps = 0.0f;
    yaw_accel_x_mps2 = 0.0f;
    yaw_accel_y_mps2 = 0.0f;
    yaw_accel_z_mps2 = 0.0f;

    MPU6050_SetDeadband(MPU6050_DEFAULT_DEADBAND_DPS);
    MPU6050_SetTempCoeff(MPU6050_DEFAULT_TEMP_COEFF);
}

void Yaw_BeginCritical(void)
{
    yaw_critical = 1U;
}

void Yaw_EndCritical(void)
{
    /*
     * 只清标记。关键区期间 Yaw_Task 一直在正常积分、时间基准也一直
     * 在维护，所以这里不能重置 yaw_started，否则会白丢一拍积分。
     */
    yaw_critical = 0U;
}

uint8_t Yaw_IsCritical(void)
{
    return yaw_critical;
}

/*
 * 关键区期间不允许有任何东西长时间占住主循环，否则 Yaw_Task 无法
 * 按 10ms 准时更新。目前唯一的大头是 OLED 刷屏，由显示模块查询
 * Yaw_IsCritical() 自行让路。
 *
 * 注意：这里绝不能改成在 SysTick 里直接读传感器。MPU 的读是阻塞
 * 轮询，读失败时要跑满超时循环(最坏几十毫秒)，而 SysTick 周期只有
 * 1ms —— 电机上地面后 I2C 失败率升高，ISR 会被卡死，system_ms
 * 停止计数、编码器测速和速度环全部失效。
 */

void Yaw_Tick(void)
{
    yaw_pending = 1U;
}

void Yaw_Task(uint32_t now_ms)
{
    uint32_t dt_ms;
    mpu6050_motion_t motion;

    if (yaw_pending == 0U) {
        return;
    }
    yaw_pending = 0U;

    /*
     * 故障状态下降频重试。这里不能用 MPU6050_IsOk() 当准入条件直接
     * return —— 那个标志只有读成功才会置回 1，不读就永远恢复不了。
     */
    if (yaw_faulted != 0U) {
        if ((now_ms - yaw_last_try_ms) < YAW_FAULT_RETRY_MS) {
            return;
        }
    }
    yaw_last_try_ms = now_ms;

    (void)MPU6050_ReadMotion(&motion);

    if (MPU6050_IsOk() == 0U) {
        /* 读失败：进入/保持故障态，这一拍不积分，别把 0 当成真实值 */
        yaw_faulted = 1U;
        yaw_started = 0U;  /* 恢复后重新对齐时间基准，不补积那段空档 */
        return;
    }

    if (yaw_faulted != 0U) {
        yaw_faulted = 0U;
    }

    yaw_rate_dps = motion.gyro_z_dps;
    yaw_accel_x_mps2 = motion.accel_x_mps2;
    yaw_accel_y_mps2 = motion.accel_y_mps2;
    yaw_accel_z_mps2 = motion.accel_z_mps2;

    /* 第一次(或故障恢复后)只对齐时间基准，不积分 */
    if (yaw_started == 0U) {
        yaw_started = 1U;
        yaw_last_ms = now_ms;
        yaw_last_temp_ms = now_ms;
        return;
    }

    dt_ms = now_ms - yaw_last_ms;
    if (dt_ms == 0U) {
        return;
    }
    yaw_last_ms = now_ms;

    if (dt_ms > YAW_MAX_DT_MS) {
        dt_ms = YAW_MAX_DT_MS;
    }

    yaw_angle_deg += yaw_rate_dps * ((float)dt_ms / 1000.0f);

    if ((now_ms - yaw_last_temp_ms) >= YAW_TEMP_UPDATE_MS) {
        yaw_last_temp_ms = now_ms;
        (void)MPU6050_UpdateTempComp();
    }
}

uint8_t Yaw_IsFaulted(void)
{
    return yaw_faulted;
}

float Yaw_GetAngle(void)
{
    return yaw_angle_deg;
}

float Yaw_GetRate(void)
{
    return yaw_rate_dps;
}

float Yaw_GetAccelX(void)
{
    return yaw_accel_x_mps2;
}

float Yaw_GetAccelY(void)
{
    return yaw_accel_y_mps2;
}

float Yaw_GetAccelZ(void)
{
    return yaw_accel_z_mps2;
}

void Yaw_Reset(void)
{
    yaw_angle_deg = 0.0f;
    yaw_rate_dps = 0.0f;
    /*
     * 一并重新对齐时间基准。手动标定会阻塞约 1 秒，若不重置，
     * 标定结束后的第一拍会拿 1 秒的 dt 去乘标定前的旧角速度。
     */
    yaw_started = 0U;
}

void Yaw_FuseReference(float reference_deg, float alpha)
{
    float a = alpha;

    if (a < 0.0f) {
        a = 0.0f;
    } else if (a > 1.0f) {
        a = 1.0f;
    }

    yaw_angle_deg = (a * yaw_angle_deg) + ((1.0f - a) * reference_deg);
}

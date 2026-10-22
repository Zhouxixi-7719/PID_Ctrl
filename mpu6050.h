#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/* I2C / 通讯层错误码，由 MPU6050_GetLastError() 返回 */
#define MPU6050_ERROR_NONE             0U
#define MPU6050_ERROR_PARAM            1U
#define MPU6050_ERROR_TIMEOUT_IDLE     2U
#define MPU6050_ERROR_TIMEOUT_BUSY     3U
#define MPU6050_ERROR_I2C_STATUS       4U
#define MPU6050_ERROR_TIMEOUT_RX       5U
#define MPU6050_ERROR_BAD_WHO_AM_I     6U

/* 标定结果码，由 MPU6050_GetLastCalResult() 返回 */
#define MPU6050_CAL_NONE               0U  /* 从未标定 */
#define MPU6050_CAL_OK                 1U  /* 标定成功 */
#define MPU6050_CAL_ERR_PARAM          2U  /* 采样点数为 0 */
#define MPU6050_CAL_ERR_IO             3U  /* 标定途中 I2C 读失败 */
#define MPU6050_CAL_ERR_MOVING         4U  /* 采样期间车在动，极差超限 */

/* 标定默认参数：500 点 x 2ms 约 1 秒 */
#define MPU6050_CAL_SAMPLES            500U
#define MPU6050_CAL_DELAY_MS           2U

/*
 * 静止判据：标定期间角速度极差(max-min)超过该值就认为车在动。
 * ±1000dps 量程下 1 LSB = 0.0305 dps，静止时经 44Hz DLPF 后
 * 噪声通常在 ±0.5 dps 内，取 3.0 dps 留足余量。
 */
#define MPU6050_CAL_MOTION_RANGE_DPS   3.0f

/*
 * 死区：标定后残余噪声仍会被积分累积，|gz| 小于该值时直接归零。
 * 代价是极慢速转向会丢一点角度，循迹车可以接受。设 0 即关闭。
 */
#define MPU6050_DEFAULT_DEADBAND_DPS   0.5f

/*
 * 零偏温度系数 k，单位 (°/s)/°C，补偿式：bias = bias0 + k * (T - T0)。
 * 默认 0 等于不补偿，行为与不加温补时一致。
 * 实测拟合方法：冷机标定记下 bias0/T0，让它自热到稳定后再读一次
 * 未补偿的静止角速度 gz1 和温度 T1，则 k = gz1 / (T1 - T0)。
 */
#define MPU6050_DEFAULT_TEMP_COEFF     0.0f

typedef struct {
    /* Acceleration is zero-referenced to the startup calibration pose. */
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} mpu6050_motion_t;

uint8_t MPU6050_Init(void);
uint8_t MPU6050_IsOk(void);
uint8_t MPU6050_ReadWhoAmI(uint8_t *id);
uint8_t MPU6050_GetI2CAddress(void);
uint8_t MPU6050_GetLastError(void);
uint8_t MPU6050_GetLastWhoAmI(void);
uint32_t MPU6050_GetLastStatus(void);

/* 角速度：以下两个都返回已减零偏、已过死区的值，单位 °/s */
float MPU6050_GetGyroZ(void);      /* 读 6 字节(GYRO_XOUT~ZOUT) */
float MPU6050_GetGyroZFast(void);  /* 只读 2 字节(GYRO_ZOUT)，控制环用这个 */

/* 未做任何补偿的原始角速度，标定和排查时用 */
/* Read accel, temperature and gyro in one 14-byte burst. */
uint8_t MPU6050_ReadMotion(mpu6050_motion_t *motion);

float MPU6050_GetGyroZRaw(void);

/* 零偏标定：必须在车完全静止、电机不转时调用。返回 1 成功 / 0 失败 */
uint8_t MPU6050_CalibrateGyroZ(uint16_t samples, uint16_t sample_delay_ms);
uint8_t MPU6050_CalibrateMotion(uint16_t samples, uint16_t sample_delay_ms);
uint8_t MPU6050_GetLastCalResult(void);
uint8_t MPU6050_IsCalibrated(void);
float MPU6050_GetLastCalRange(void);  /* 上次标定的极差，用来判断标定质量 */

void MPU6050_SetGyroZBias(float bias);
float MPU6050_GetGyroZBias(void);      /* 标定基准零偏 bias0 */
float MPU6050_GetEffectiveBias(void);  /* 温补之后实际扣掉的零偏 */

/* 温度，单位 °C */
uint8_t MPU6050_ReadTemperature(float *celsius);
float MPU6050_GetLastTemperature(void);
float MPU6050_GetCalTemperature(void);  /* 标定时刻的 T0 */

/*
 * 更新温度补偿。会额外产生一次 I2C 读，不要每个控制周期都调，
 * 每秒一次足够（自热是分钟量级的过程）。
 */
uint8_t MPU6050_UpdateTempComp(void);

void MPU6050_SetTempCoeff(float k);
float MPU6050_GetTempCoeff(void);

void MPU6050_SetDeadband(float dps);
float MPU6050_GetDeadband(void);

#endif

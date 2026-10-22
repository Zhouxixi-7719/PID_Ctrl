#include "mpu6050.h"
#include "Delay.h"
#include "ti_msp_dl_config.h"

#define MPU6050_ADDR                 0x68U              //I2C从机地址，AD0引脚接地时
#define MPU6050_ADDR_ALT             0x69U              //I2C从机地址，AD0引脚接高时
#define MPU6050_WHO_AM_I_VALUE       0x68U              //
#define MPU6500_WHO_AM_I_VALUE       0x70U              //
#define MPU6050_WHO_AM_I             0x75U              //身份寄存器地址
#define MPU6050_PWR_MGMT_1           0x6BU              //电源管理：唤醒芯片+选时钟源
#define MPU6050_SMPLRT_DIV           0x19U              //采样分屏率
#define MPU6050_CONFIG               0x1AU              //数字低通滤波带宽
#define MPU6050_GYRO_CONFIG          0x1BU              //陀螺仪量程
#define MPU6050_ACCEL_CONFIG         0x1CU              //加速度计量程
#define MPU6050_TEMP_OUT_H           0x41U              //温度数据
#define MPU6050_GYRO_XOUT_H          0x43U              //陀螺仪X轴数据     
#define MPU6050_GYRO_ZOUT_H          0x47U              //陀螺仪Z轴数据

/*
 * I2C 轮询超时上限。循环体约 10 cycles，@80MHz 时 20000 次约 2.5ms，
 * 而一次正常的 2 字节读只要约 400us，余量足够。
 * 一次读里最多有 4 处这样的等待，所以这个值直接决定 MPU 掉线时
 * 主循环被拖住的最长时间(此处约 10ms)。原来的 100000 会拖到 50ms，
 * 电机干扰导致读失败时会把整个主循环拖垮。
 */
#define MPU6050_ACCEL_XOUT_H         0x3BU
#define MPU6050_I2C_TIMEOUT_LOOPS    20000U
#define MPU6050_I2C_GUARD_CYCLES     (CPUCLK_FREQ / 25000U)
#define MPU6050_GYRO_SCALE_1000DPS   32.8f
#define MPU6050_ACCEL_SCALE_2G       16384.0f
#define MPU6050_GRAVITY_MPS2         9.80665f

/* 片上温度换算，来自 MPU6050 数据手册：T(°C) = raw/340 + 36.53 */
#define MPU6050_TEMP_LSB_PER_C       340.0f
#define MPU6050_TEMP_OFFSET_C        36.53f

static volatile uint8_t mpu6050_ok = 0U;
static uint8_t mpu6050_addr = MPU6050_ADDR;
static volatile uint8_t mpu6050_last_error = MPU6050_ERROR_NONE;
static volatile uint8_t mpu6050_last_who_am_i = 0U;
static volatile uint32_t mpu6050_last_status = 0U;

/* 零偏与补偿状态 */
static float mpu6050_gz_bias = 0.0f;        /* 标定基准零偏 bias0 */
static float mpu6050_gz_bias_eff = 0.0f;    /* 温补后实际扣掉的零偏 */
static float mpu6050_cal_temp_c = 0.0f;     /* 标定时刻温度 T0 */
static float mpu6050_last_temp_c = 0.0f;
static float mpu6050_temp_coeff = MPU6050_DEFAULT_TEMP_COEFF;
static float mpu6050_deadband = MPU6050_DEFAULT_DEADBAND_DPS;
static float mpu6050_cal_range = 0.0f;
static volatile uint8_t mpu6050_cal_result = MPU6050_CAL_NONE;
static volatile uint8_t mpu6050_calibrated = 0U;
static float mpu6050_accel_zero_x = 0.0f;
static float mpu6050_accel_zero_y = 0.0f;
static float mpu6050_accel_zero_z = 0.0f;

static uint8_t mpu6050_read_regs_at(uint8_t addr, uint8_t reg, uint8_t *buf,
    uint8_t len);

static uint8_t mpu6050_is_supported_id(uint8_t id)
{
    return ((id == MPU6050_WHO_AM_I_VALUE) ||
        (id == MPU6500_WHO_AM_I_VALUE)) ? 1U : 0U;
}

static void mpu6050_set_error(uint8_t error)
{
    mpu6050_last_error = error;
    mpu6050_last_status = DL_I2C_getControllerStatus(I2C_MPU6050_INST);
}

static uint8_t mpu6050_wait_idle(void)
{
    uint32_t guard = 0U;

    while ((DL_I2C_getControllerStatus(I2C_MPU6050_INST) &
        DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (++guard > MPU6050_I2C_TIMEOUT_LOOPS) {
            mpu6050_set_error(MPU6050_ERROR_TIMEOUT_IDLE);
            DL_I2C_resetControllerTransfer(I2C_MPU6050_INST);
            return 0U;
        }
    }

    return 1U;
}

static uint8_t mpu6050_check_error(void)
{
    if ((DL_I2C_getControllerStatus(I2C_MPU6050_INST) &
        DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        mpu6050_set_error(MPU6050_ERROR_I2C_STATUS);
        DL_I2C_resetControllerTransfer(I2C_MPU6050_INST);
        return 0U;
    }

    return 1U;
}

static uint8_t mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    tx[0] = reg;
    tx[1] = value;

    if (mpu6050_wait_idle() == 0U) {
        return 0U;
    }

    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    (void)DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST, tx, sizeof(tx));
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, mpu6050_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, sizeof(tx));
    delay_cycles(MPU6050_I2C_GUARD_CYCLES);

    if (mpu6050_wait_idle() == 0U) {
        return 0U;
    }

    return mpu6050_check_error();
}

static uint8_t mpu6050_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return mpu6050_read_regs_at(mpu6050_addr, reg, buf, len);
}

static int16_t mpu6050_decode_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

static uint8_t mpu6050_read_motion_raw(int16_t raw[7])
{
    uint8_t buf[14];
    uint8_t i;

    if (mpu6050_read_regs(MPU6050_ACCEL_XOUT_H, buf, sizeof(buf)) == 0U) {
        mpu6050_ok = 0U;
        return 0U;
    }

    for (i = 0U; i < 7U; i++) {
        raw[i] = mpu6050_decode_i16(&buf[i * 2U]);
    }
    mpu6050_ok = 1U;
    return 1U;
}

static uint8_t mpu6050_read_regs_at(uint8_t addr, uint8_t reg, uint8_t *buf,
    uint8_t len)
{
    uint8_t i;
    uint32_t guard;

    if ((buf == 0) || (len == 0U)) {
        mpu6050_set_error(MPU6050_ERROR_PARAM);
        return 0U;
    }

    if (mpu6050_wait_idle() == 0U) {
        return 0U;
    }

    /*
     * Use two STOP-separated transfers for MSPM0 polling mode. The MPU keeps
     * the register pointer after the sub-address write, and this avoids RX
     * FIFO timeouts seen with repeated-start on this DriverLib path.
     */
    DL_I2C_resetControllerTransfer(I2C_MPU6050_INST);
    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    (void)DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST, &reg, 1U);
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    delay_cycles(MPU6050_I2C_GUARD_CYCLES);

    if ((mpu6050_wait_idle() == 0U) || (mpu6050_check_error() == 0U)) {
        return 0U;
    }

    DL_I2C_flushControllerRXFIFO(I2C_MPU6050_INST);
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    delay_cycles(MPU6050_I2C_GUARD_CYCLES);

    for (i = 0U; i < len; i++) {
        guard = 0U;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_MPU6050_INST)) {
            if (++guard > MPU6050_I2C_TIMEOUT_LOOPS) {
                mpu6050_set_error(MPU6050_ERROR_TIMEOUT_RX);
                DL_I2C_resetControllerTransfer(I2C_MPU6050_INST);
                return 0U;
            }
        }
        buf[i] = DL_I2C_receiveControllerData(I2C_MPU6050_INST);
    }

    if ((mpu6050_wait_idle() == 0U) || (mpu6050_check_error() == 0U)) {
        return 0U;
    }

    return 1U;
}

uint8_t MPU6050_ReadWhoAmI(uint8_t *id)
{
    uint8_t ok = mpu6050_read_regs(MPU6050_WHO_AM_I, id, 1U);

    if ((ok != 0U) && (id != 0)) {
        mpu6050_last_who_am_i = *id;
        mpu6050_last_error = MPU6050_ERROR_NONE;
        mpu6050_last_status = DL_I2C_getControllerStatus(I2C_MPU6050_INST);
    }

    return ok;
}

uint8_t MPU6050_Init(void)
{
    uint8_t id = 0U;
    uint8_t read_ok;

    mpu6050_ok = 0U;
    mpu6050_last_error = MPU6050_ERROR_NONE;
    mpu6050_last_who_am_i = 0U;
    delay_ms(100U);

    mpu6050_addr = MPU6050_ADDR;
    read_ok = MPU6050_ReadWhoAmI(&id);
    if ((read_ok == 0U) || (mpu6050_is_supported_id(id) == 0U)) {
        mpu6050_addr = MPU6050_ADDR_ALT;
        read_ok = MPU6050_ReadWhoAmI(&id);
        if ((read_ok == 0U) || (mpu6050_is_supported_id(id) == 0U)) {
            if (read_ok != 0U) {
                mpu6050_last_who_am_i = id;
                mpu6050_set_error(MPU6050_ERROR_BAD_WHO_AM_I);
            }
            return 0U;
        }
    }

    /* Wake up and use X gyro PLL as the clock source. */
    if (mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x01U) == 0U) {
        return 0U;
    }
    delay_ms(10U);

    /* 1kHz gyro sample clock / (1 + 9) = 100Hz sample rate. */
    if (mpu6050_write_reg(MPU6050_SMPLRT_DIV, 0x09U) == 0U) {
        return 0U;
    }

    /* DLPF_CFG=3: gyro bandwidth about 44Hz, suitable for 10ms control. */
    if (mpu6050_write_reg(MPU6050_CONFIG, 0x03U) == 0U) {
        return 0U;
    }

    /* FS_SEL=2: +/-1000 deg/s, 32.8 LSB/(deg/s). */
    if (mpu6050_write_reg(MPU6050_GYRO_CONFIG, 0x10U) == 0U) {
        return 0U;
    }

    if (mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x00U) == 0U) {
        return 0U;
    }

    /* 复位所有标定状态：重新初始化后必须重新标定 */
    mpu6050_gz_bias = 0.0f;
    mpu6050_gz_bias_eff = 0.0f;
    mpu6050_cal_temp_c = 0.0f;
    mpu6050_cal_range = 0.0f;
    mpu6050_cal_result = MPU6050_CAL_NONE;
    mpu6050_calibrated = 0U;
    mpu6050_accel_zero_x = 0.0f;
    mpu6050_accel_zero_y = 0.0f;
    mpu6050_accel_zero_z = 0.0f;

    mpu6050_ok = 1U;
    return 1U;
}

uint8_t MPU6050_IsOk(void)
{
    return mpu6050_ok;
}

uint8_t MPU6050_GetI2CAddress(void)
{
    return mpu6050_addr;
}

uint8_t MPU6050_GetLastError(void)
{
    return mpu6050_last_error;
}

uint8_t MPU6050_GetLastWhoAmI(void)
{
    return mpu6050_last_who_am_i;
}

uint32_t MPU6050_GetLastStatus(void)
{
    return mpu6050_last_status;
}

void MPU6050_SetGyroZBias(float bias)
{
    mpu6050_gz_bias = bias;
    mpu6050_gz_bias_eff = bias;
    mpu6050_calibrated = 1U;
}

float MPU6050_GetGyroZBias(void)
{
    return mpu6050_gz_bias;
}

float MPU6050_GetEffectiveBias(void)
{
    return mpu6050_gz_bias_eff;
}

uint8_t MPU6050_IsCalibrated(void)
{
    return mpu6050_calibrated;
}

uint8_t MPU6050_GetLastCalResult(void)
{
    return mpu6050_cal_result;
}

float MPU6050_GetLastCalRange(void)
{
    return mpu6050_cal_range;
}

void MPU6050_SetTempCoeff(float k)
{
    mpu6050_temp_coeff = k;
}

float MPU6050_GetTempCoeff(void)
{
    return mpu6050_temp_coeff;
}

void MPU6050_SetDeadband(float dps)
{
    mpu6050_deadband = (dps < 0.0f) ? 0.0f : dps;
}

float MPU6050_GetDeadband(void)
{
    return mpu6050_deadband;
}

float MPU6050_GetLastTemperature(void)
{
    return mpu6050_last_temp_c;
}

float MPU6050_GetCalTemperature(void)
{
    return mpu6050_cal_temp_c;
}

/* 读原始 GYRO_ZOUT，成功返回 1。raw 为 int16 LSB，不做任何补偿。 */
static uint8_t mpu6050_read_gz_lsb(int16_t *raw)
{
    uint8_t buf[2];

    if (mpu6050_read_regs(MPU6050_GYRO_ZOUT_H, buf, sizeof(buf)) == 0U) {
        mpu6050_ok = 0U;
        return 0U;
    }

    *raw = (int16_t)(((uint16_t)buf[0] << 8U) | (uint16_t)buf[1]);
    mpu6050_ok = 1U;
    return 1U;
}

/* 统一的补偿链：扣温补后的零偏，再过死区 */
static float mpu6050_compensate(float dps)
{
    float value = dps - mpu6050_gz_bias_eff;

    if ((value < mpu6050_deadband) && (value > -mpu6050_deadband)) {
        value = 0.0f;
    }

    return value;
}

uint8_t MPU6050_ReadTemperature(float *celsius)
{
    uint8_t buf[2];
    int16_t raw;

    if (celsius == 0) {
        mpu6050_set_error(MPU6050_ERROR_PARAM);
        return 0U;
    }

    if (mpu6050_read_regs(MPU6050_TEMP_OUT_H, buf, sizeof(buf)) == 0U) {
        return 0U;
    }

    raw = (int16_t)(((uint16_t)buf[0] << 8U) | (uint16_t)buf[1]);
    *celsius = ((float)raw / MPU6050_TEMP_LSB_PER_C) + MPU6050_TEMP_OFFSET_C;
    mpu6050_last_temp_c = *celsius;
    return 1U;
}

uint8_t MPU6050_UpdateTempComp(void)
{
    float temp_c;

    /* 没标定过就没有 T0 基准，温补无从谈起 */
    if (mpu6050_calibrated == 0U) {
        return 0U;
    }

    if (MPU6050_ReadTemperature(&temp_c) == 0U) {
        return 0U;
    }

    mpu6050_gz_bias_eff = mpu6050_gz_bias +
        (mpu6050_temp_coeff * (temp_c - mpu6050_cal_temp_c));
    return 1U;
}

float MPU6050_GetGyroZRaw(void)
{
    int16_t raw_z;

    if (mpu6050_read_gz_lsb(&raw_z) == 0U) {
        return 0.0f;
    }

    return (float)raw_z / MPU6050_GYRO_SCALE_1000DPS;
}

float MPU6050_GetGyroZ(void)
{
    uint8_t buf[6];
    int16_t raw_z;

    if (mpu6050_read_regs(MPU6050_GYRO_XOUT_H, buf, sizeof(buf)) == 0U) {
        mpu6050_ok = 0U;
        return 0.0f;
    }

    raw_z = (int16_t)(((uint16_t)buf[4] << 8U) | (uint16_t)buf[5]);
    mpu6050_ok = 1U;
    return mpu6050_compensate((float)raw_z / MPU6050_GYRO_SCALE_1000DPS);
}

float MPU6050_GetGyroZFast(void)
{
    int16_t raw_z;

    if (mpu6050_read_gz_lsb(&raw_z) == 0U) {
        return 0.0f;
    }

    return mpu6050_compensate((float)raw_z / MPU6050_GYRO_SCALE_1000DPS);
}

/*
 * 零偏标定。必须在车完全静止、电机不转时调用，否则电机振动会被
 * 当成真实角速度平均进 bias 里。
 *
 * 采样期间同时记录极差(max-min)，超过 MPU6050_CAL_MOTION_RANGE_DPS
 * 就判定车在动，拒绝这次标定并保留原有 bias，避免标歪之后一路漂。
 *
 * 累加用 int32 累加原始 LSB 而不是 float 累加 dps，避免上百个数
 * 连加时的浮点舍入误差。
 */
uint8_t MPU6050_ReadMotion(mpu6050_motion_t *motion)
{
    int16_t raw[7];
    float accel_scale;

    if (motion == 0) {
        mpu6050_set_error(MPU6050_ERROR_PARAM);
        return 0U;
    }
    if (mpu6050_read_motion_raw(raw) == 0U) {
        return 0U;
    }

    accel_scale = MPU6050_GRAVITY_MPS2 / MPU6050_ACCEL_SCALE_2G;
    motion->accel_x_mps2 = (float)raw[0] * accel_scale -
        mpu6050_accel_zero_x;
    motion->accel_y_mps2 = (float)raw[1] * accel_scale -
        mpu6050_accel_zero_y;
    motion->accel_z_mps2 = (float)raw[2] * accel_scale -
        mpu6050_accel_zero_z;
    motion->gyro_x_dps = (float)raw[4] / MPU6050_GYRO_SCALE_1000DPS;
    motion->gyro_y_dps = (float)raw[5] / MPU6050_GYRO_SCALE_1000DPS;
    motion->gyro_z_dps = mpu6050_compensate(
        (float)raw[6] / MPU6050_GYRO_SCALE_1000DPS);
    mpu6050_last_temp_c = ((float)raw[3] / MPU6050_TEMP_LSB_PER_C) +
        MPU6050_TEMP_OFFSET_C;
    return 1U;
}

uint8_t MPU6050_CalibrateGyroZ(uint16_t samples, uint16_t sample_delay_ms)
{
    uint16_t i;
    int32_t sum_lsb = 0;                //累计所有采样值
    int16_t raw_z;                      //每次从寄存器读到的16位原始数据
    int16_t min_lsb = INT16_MAX;        //采样期间记录的最小值
    int16_t max_lsb = INT16_MIN;        //采样期间记录的最小值
    float range_dps;                    //两者之差换算成 °/s
    float new_bias;                     //最终算出的零偏
    float temp_c;

    if (samples == 0U) {
        mpu6050_cal_result = MPU6050_CAL_ERR_PARAM;
        return 0U;
    }

    for (i = 0U; i < samples; i++) {
        if (mpu6050_read_gz_lsb(&raw_z) == 0U) {
            mpu6050_cal_result = MPU6050_CAL_ERR_IO;
            return 0U;
        }

        sum_lsb += (int32_t)raw_z;
        if (raw_z < min_lsb) {
            min_lsb = raw_z;
        }
        if (raw_z > max_lsb) {
            max_lsb = raw_z;
        }

        delay_ms(sample_delay_ms);
    }

    range_dps = (float)((int32_t)max_lsb - (int32_t)min_lsb) /
        MPU6050_GYRO_SCALE_1000DPS;
    mpu6050_cal_range = range_dps;

    if (range_dps > MPU6050_CAL_MOTION_RANGE_DPS) {
        /* 采样期间车在动，保留原 bias 不动 */
        mpu6050_cal_result = MPU6050_CAL_ERR_MOVING;
        return 0U;
    }

    new_bias = ((float)sum_lsb / (float)samples) / MPU6050_GYRO_SCALE_1000DPS;

    /* 记下 T0；温度读不到也不算标定失败，只是温补退化为不补偿 */
    if (MPU6050_ReadTemperature(&temp_c) != 0U) {
        mpu6050_cal_temp_c = temp_c;
    } else {
        mpu6050_cal_temp_c = mpu6050_last_temp_c;
    }

    mpu6050_gz_bias = new_bias;
    mpu6050_gz_bias_eff = new_bias;
    mpu6050_calibrated = 1U;
    mpu6050_cal_result = MPU6050_CAL_OK;
    return 1U;
}

uint8_t MPU6050_CalibrateMotion(uint16_t samples, uint16_t sample_delay_ms)
{
    uint16_t i;
    int32_t sum_ax = 0;
    int32_t sum_ay = 0;
    int32_t sum_az = 0;
    int32_t sum_gz = 0;
    int16_t min_gz = INT16_MAX;
    int16_t max_gz = INT16_MIN;
    int16_t raw[7];
    float range_dps;
    float accel_scale;

    if (samples == 0U) {
        mpu6050_cal_result = MPU6050_CAL_ERR_PARAM;
        return 0U;
    }

    for (i = 0U; i < samples; i++) {
        if (mpu6050_read_motion_raw(raw) == 0U) {
            mpu6050_cal_result = MPU6050_CAL_ERR_IO;
            return 0U;
        }
        sum_ax += (int32_t)raw[0];
        sum_ay += (int32_t)raw[1];
        sum_az += (int32_t)raw[2];
        sum_gz += (int32_t)raw[6];
        if (raw[6] < min_gz) {
            min_gz = raw[6];
        }
        if (raw[6] > max_gz) {
            max_gz = raw[6];
        }
        delay_ms(sample_delay_ms);
    }

    mpu6050_last_temp_c = ((float)raw[3] / MPU6050_TEMP_LSB_PER_C) +
        MPU6050_TEMP_OFFSET_C;

    range_dps = (float)((int32_t)max_gz - (int32_t)min_gz) /
        MPU6050_GYRO_SCALE_1000DPS;
    mpu6050_cal_range = range_dps;
    if (range_dps > MPU6050_CAL_MOTION_RANGE_DPS) {
        mpu6050_cal_result = MPU6050_CAL_ERR_MOVING;
        return 0U;
    }

    mpu6050_gz_bias = ((float)sum_gz / (float)samples) /
        MPU6050_GYRO_SCALE_1000DPS;
    mpu6050_gz_bias_eff = mpu6050_gz_bias;
    accel_scale = MPU6050_GRAVITY_MPS2 / MPU6050_ACCEL_SCALE_2G;
    mpu6050_accel_zero_x = ((float)sum_ax / (float)samples) * accel_scale;
    mpu6050_accel_zero_y = ((float)sum_ay / (float)samples) * accel_scale;
    mpu6050_accel_zero_z = ((float)sum_az / (float)samples) * accel_scale;
    mpu6050_cal_temp_c = mpu6050_last_temp_c;
    mpu6050_calibrated = 1U;
    mpu6050_cal_result = MPU6050_CAL_OK;
    return 1U;
}

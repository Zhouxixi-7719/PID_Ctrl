/*
 * i2c_override.c — 覆盖 SysConfig 生成的 I2C 初始化
 *
 * 原因：SysConfig 生成的 I2C 初始化不完整，缺少：
 *   - resetControllerTransfer
 *   - setTimerPeriod （波特率设置）
 *   - setControllerTXFIFOThreshold / setControllerRXFIFOThreshold
 *   - enableControllerClockStretching
 *   - enableController （I2C 根本没打开）
 *
 * 利用 SYSCONFIG_WEAK 优先级覆盖。
 */

#include "ti_msp_dl_config.h"

static void i2c_config_controller(I2C_Regs *i2c, uint8_t timer_period)
{
    static const DL_I2C_ClockConfig i2c_clock_config = {
        .clockSel    = DL_I2C_CLOCK_BUSCLK,
        .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
    };

    DL_I2C_setClockConfig(i2c, (DL_I2C_ClockConfig *) &i2c_clock_config);
    DL_I2C_setAnalogGlitchFilterPulseWidth(i2c,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(i2c);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(i2c);
    DL_I2C_setTimerPeriod(i2c, timer_period);
    DL_I2C_setControllerTXFIFOThreshold(i2c, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(i2c, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(i2c);

    /* 打开 I2C 模块 */
    DL_I2C_enableController(i2c);
}

void SYSCFG_DL_I2C_OLED_init(void)
{
    /* 100kHz @80MHz BUSCLK */
    i2c_config_controller(I2C_OLED_INST, 39U);
}

void SYSCFG_DL_I2C_MPU6050_init(void)
{
    /* 100kHz @80MHz BUSCLK, use this while bringing up MPU6050. */
    i2c_config_controller(I2C_MPU6050_INST, 39U);
}

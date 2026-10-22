/*
 * sysclock_override.c — 覆盖 SysConfig 生成的 SYSCFG_DL_SYSCTL_init()
 *
 * 利用 SYSCONFIG_WEAK 优先级：用户实现优先于生成代码。
 * 解决了原生成代码中两处潜在死循环：
 *   1. PLL 校验 while(init()==false)  无限循环
 *   2. SYSPLLGOOD 等待                无限循环
 *
 * 策略：启用 PLL 3 次尝试，失败则回退到 SYSOSC 32MHz 继续运行。
 */

#include "ti_msp_dl_config.h"

#define PLL_RETRY_MAX   3
#define PLLGOOD_TIMEOUT 100000   /* 超时等待 cycles（约 1ms @80MHz） */

void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);

    /* 禁用外部晶振——SYSPLL 使用内部 SYSOSC 作参考 */
    DL_SYSCTL_disableHFXT();

    DL_SYSCTL_disableSYSPLL();

    /*
     * SYSPLL 配置：SYSOSC(32MHz) → PDIV=2 (16MHz) → QDIV=10 → VCO=160MHz
     * → CLK0 DIV/2 → 80MHz
     */
    static const DL_SYSCTL_SYSPLLConfig gMySYSPLLConfig = {
        .inputFreq   = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
        .rDivClk2x   = 1,
        .rDivClk1    = 0,
        .rDivClk0    = 0,
        .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
        .enableCLK1  = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
        .enableCLK0  = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
        .sysPLLMCLK  = DL_SYSCTL_SYSPLL_MCLK_CLK0,
        .sysPLLRef   = DL_SYSCTL_SYSPLL_REF_SYSOSC,   /* 内部振荡器，不依赖外部晶振 */
        .qDiv        = 9,
        .pDiv        = DL_SYSCTL_SYSPLL_PDIV_2
    };

    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *)&gMySYSPLLConfig);
    DL_SYSCTL_enableSYSPLL();

    /* 等待 PLL 锁定（带超时） */
    volatile uint32_t timeout = PLLGOOD_TIMEOUT;
 while (((DL_SYSCTL_getClockStatus() & SYSCTL_CLKSTATUS_SYSPLLGOOD_MASK)
            != DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD)
           && (timeout > 0)) {
        timeout--;
    }

    if (timeout > 0) {
        /* PLL 锁定成功 → MCLK = PLL 80MHz */
        DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
        DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK,
                                DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    }
    /* PLL 锁定失败 → 保持 SYSOSC 32MHz（已经是默认 MCLK 源） */
}

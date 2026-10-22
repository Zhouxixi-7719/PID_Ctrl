#include "Delay.h"
#include "ti_msp_dl_config.h"

void delay_ms(uint32_t ms)
{
    uint32_t i;
    /* CPUCLK_FREQ / 1000 = 每秒毫秒数对应的 cycles */
    for (i = 0; i < ms; i++) {
        delay_cycles(CPUCLK_FREQ / 1000);
    }
}

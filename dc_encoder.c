#include "dc_encoder.h"
#include "ti_msp_dl_config.h"

static volatile int32_t encoder_count_l = 0;
static volatile int32_t encoder_count_r = 0;
static volatile int32_t encoder_last_delta_l = 0;
static volatile int32_t encoder_last_delta_r = 0;
static volatile int32_t encoder_test_count_l = 0;
static volatile int32_t encoder_test_count_r = 0;

/* 测速滑动窗口：保存最近 N 个周期的计数增量及其总和 */
static int32_t encoder_win_l[DC_ENCODER_SPEED_WINDOW_N];
static int32_t encoder_win_r[DC_ENCODER_SPEED_WINDOW_N];
static int32_t encoder_win_sum_l = 0;
static int32_t encoder_win_sum_r = 0;
static uint8_t encoder_win_idx_l = 0U;
static uint8_t encoder_win_idx_r = 0U;

/* 窗口总和换算成 cm/s：总计数 -> 圈数 -> 路程，再除以窗口时长 */
#define DC_ENCODER_WINDOW_SECONDS   (DC_ENCODER_SAMPLE_PERIOD_S * \
                                     (float)DC_ENCODER_SPEED_WINDOW_N)

void Encoder_ResetSpeedWindow(void)
{
    uint8_t i;

    for (i = 0U; i < DC_ENCODER_SPEED_WINDOW_N; i++) {
        encoder_win_l[i] = 0;
        encoder_win_r[i] = 0;
    }

    encoder_win_sum_l = 0;
    encoder_win_sum_r = 0;
    encoder_win_idx_l = 0U;
    encoder_win_idx_r = 0U;
}

void dc_encoder_init(void)
{
    encoder_count_l = 0;
    encoder_count_r = 0;
    encoder_last_delta_l = 0;
    encoder_last_delta_r = 0;
    encoder_test_count_l = 0;
    encoder_test_count_r = 0;
    Encoder_ResetSpeedWindow();

    DL_GPIO_clearInterruptStatus(DC_ENCODER_PORT,
        DC_ENCODER_L_ENCA_PIN | DC_ENCODER_L_ENCB_PIN |
            DC_ENCODER_R_ENCA_PIN | DC_ENCODER_R_ENCB_PIN);

    NVIC_SetPriority(DC_ENCODER_INT_IRQN, 0);
    NVIC_EnableIRQ(DC_ENCODER_INT_IRQN);
}

int32_t dc_encoder_get_count_l(void)
{
#if (DC_ENCODER_SWAP_LR != 0U)
    return encoder_count_r;
#else
    return encoder_count_l;
#endif
}

int32_t dc_encoder_get_count_r(void)
{
#if (DC_ENCODER_SWAP_LR != 0U)
    return encoder_count_l;
#else
    return encoder_count_r;
#endif
}

int32_t dc_encoder_get_delta_l(void)
{
    int32_t delta;

    __disable_irq();
#if (DC_ENCODER_SWAP_LR != 0U)
    delta = encoder_count_r;
    encoder_count_r = 0;
#else
    delta = encoder_count_l;
    encoder_count_l = 0;
#endif
    __enable_irq();

    return delta;
}

int32_t dc_encoder_get_delta_r(void)
{
    int32_t delta;

    __disable_irq();
#if (DC_ENCODER_SWAP_LR != 0U)
    delta = encoder_count_l;
    encoder_count_l = 0;
#else
    delta = encoder_count_r;
    encoder_count_r = 0;
#endif
    __enable_irq();

    return delta;
}

float Encoder_GetSpeed_L(void)
{
    int32_t delta = dc_encoder_get_delta_l() * DC_ENCODER_DIR_L;

    /* 单周期增量仍要保留：直角弯的编码器测角用的是它，不是窗口和 */
    encoder_last_delta_l = delta;

    /* 滑动窗口：扣掉即将被覆盖的最老一项，写入本周期的新值 */
    encoder_win_sum_l -= encoder_win_l[encoder_win_idx_l];
    encoder_win_l[encoder_win_idx_l] = delta;
    encoder_win_sum_l += delta;
    encoder_win_idx_l++;
    if (encoder_win_idx_l >= DC_ENCODER_SPEED_WINDOW_N) {
        encoder_win_idx_l = 0U;
    }

    return (float)encoder_win_sum_l * DC_WHEEL_CIRCUMFERENCE_CM /
        DC_ENCODER_COUNTS_PER_REV / DC_ENCODER_WINDOW_SECONDS;
}

float Encoder_GetSpeed_R(void)
{
    int32_t delta = dc_encoder_get_delta_r() * DC_ENCODER_DIR_R;

    encoder_last_delta_r = delta;

    encoder_win_sum_r -= encoder_win_r[encoder_win_idx_r];
    encoder_win_r[encoder_win_idx_r] = delta;
    encoder_win_sum_r += delta;
    encoder_win_idx_r++;
    if (encoder_win_idx_r >= DC_ENCODER_SPEED_WINDOW_N) {
        encoder_win_idx_r = 0U;
    }

    return (float)encoder_win_sum_r * DC_WHEEL_CIRCUMFERENCE_CM /
        DC_ENCODER_COUNTS_PER_REV / DC_ENCODER_WINDOW_SECONDS;
}

int32_t Encoder_GetLastDelta_L(void)
{
    return encoder_last_delta_l;
}

int32_t Encoder_GetLastDelta_R(void)
{
    return encoder_last_delta_r;
}

void Encoder_TestReset(void)
{
    __disable_irq();
    encoder_test_count_l = 0;
    encoder_test_count_r = 0;
    __enable_irq();
}

int32_t Encoder_TestGetCountL(void)
{
#if (DC_ENCODER_SWAP_LR != 0U)
    return encoder_test_count_r * DC_ENCODER_DIR_L;
#else
    return encoder_test_count_l * DC_ENCODER_DIR_L;
#endif
}

int32_t Encoder_TestGetCountR(void)
{
#if (DC_ENCODER_SWAP_LR != 0U)
    return encoder_test_count_l * DC_ENCODER_DIR_R;
#else
    return encoder_test_count_r * DC_ENCODER_DIR_R;
#endif
}

void GROUP1_IRQHandler(void)
{
    uint32_t group = DL_Interrupt_getStatusGroup(DL_INTERRUPT_GROUP_1,
        DL_INTERRUPT_GROUP1_GPIOA);

    if ((group & DL_INTERRUPT_GROUP1_GPIOA) != 0U) {
        uint32_t status = DL_GPIO_getEnabledInterruptStatus(DC_ENCODER_PORT,
            DC_ENCODER_L_ENCA_PIN | DC_ENCODER_L_ENCB_PIN |
                DC_ENCODER_R_ENCA_PIN | DC_ENCODER_R_ENCB_PIN);

        if ((status & DC_ENCODER_L_ENCA_PIN) != 0U) {
            uint32_t a = DL_GPIO_readPins(DC_ENCODER_PORT,
                DC_ENCODER_L_ENCA_PIN);
            uint32_t b = DL_GPIO_readPins(DC_ENCODER_PORT,
                DC_ENCODER_L_ENCB_PIN);
            int32_t step = (a != b) ? 1 : -1;
            encoder_count_l += step;
            encoder_test_count_l += step;
        }

        if ((status & DC_ENCODER_L_ENCB_PIN) != 0U) {
            uint32_t a = DL_GPIO_readPins(DC_ENCODER_PORT,
                DC_ENCODER_L_ENCA_PIN);
            uint32_t b = DL_GPIO_readPins(DC_ENCODER_PORT,
                DC_ENCODER_L_ENCB_PIN);
            int32_t step = (a == b) ? 1 : -1;
            encoder_count_l += step;
            encoder_test_count_l += step;
        }

        if ((status & DC_ENCODER_R_ENCA_PIN) != 0U) {
            uint32_t a = DL_GPIO_readPins(DC_ENCODER_PORT,
                DC_ENCODER_R_ENCA_PIN);
            uint32_t b = DL_GPIO_readPins(DC_ENCODER_PORT,
                DC_ENCODER_R_ENCB_PIN);
            int32_t step = (a != b) ? 1 : -1;
            encoder_count_r += step;
            encoder_test_count_r += step;
        }

        if ((status & DC_ENCODER_R_ENCB_PIN) != 0U) {
            uint32_t a = DL_GPIO_readPins(DC_ENCODER_PORT,
                DC_ENCODER_R_ENCA_PIN);
            uint32_t b = DL_GPIO_readPins(DC_ENCODER_PORT,
                DC_ENCODER_R_ENCB_PIN);
            int32_t step = (a == b) ? 1 : -1;
            encoder_count_r += step;
            encoder_test_count_r += step;
        }

        DL_GPIO_clearInterruptStatus(DC_ENCODER_PORT, status);
    }
}

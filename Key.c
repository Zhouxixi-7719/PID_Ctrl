#include "Key.h"
#include "ti_msp_dl_config.h"

#define KEY_COUNT                      4U
#define KEY_1                          0U
#define KEY_2                          1U
#define KEY_3                          2U
#define KEY_4                          3U

#define KEY_RELEASED                   0U
#define KEY_PRESSED                    1U

#define KEY_DEBOUNCE_TICKS             2U
#define KEY_LONG_PRESS_TICKS           100U

#define KEY_FLAG_KEY1_SHORT            (1U << 0U)
#define KEY_FLAG_KEY2_SHORT            (1U << 1U)
#define KEY_FLAG_KEY3_SHORT            (1U << 2U)
#define KEY_FLAG_KEY1_LONG             (1U << 3U)
#define KEY_FLAG_KEY4_SHORT            (1U << 4U)
#define KEY_FLAG_KEY4_LONG             (1U << 5U)

#ifndef KEY_KEY4_PORT
#define KEY_KEY4_PORT                  (GPIOB)
#endif

#ifndef KEY_KEY4_PIN
#define KEY_KEY4_PIN                   (DL_GPIO_PIN_27)
#endif

static volatile uint16_t key_event_flags = 0U;

static uint8_t Key_GetState(uint8_t key)
{
    if (key == KEY_1) {
        return (DL_GPIO_readPins(KEY_KEY1_PORT, KEY_KEY1_PIN) == 0U) ?
            KEY_PRESSED : KEY_RELEASED;
    }

    if (key == KEY_2) {
        return (DL_GPIO_readPins(KEY_KEY2_PORT, KEY_KEY2_PIN) == 0U) ?
            KEY_PRESSED : KEY_RELEASED;
    }

    if (key == KEY_3) {
        return (DL_GPIO_readPins(KEY_KEY3_PORT, KEY_KEY3_PIN) == 0U) ?
            KEY_PRESSED : KEY_RELEASED;
    }

    if (key == KEY_4) {
        return (DL_GPIO_readPins(KEY_KEY4_PORT, KEY_KEY4_PIN) == 0U) ?
            KEY_PRESSED : KEY_RELEASED;
    }

    return KEY_RELEASED;
}

static void Key_SetEvent(uint8_t event)
{
    if (event == KEY_EVENT_KEY1_SHORT) {
        key_event_flags |= KEY_FLAG_KEY1_SHORT;
    } else if (event == KEY_EVENT_KEY2_SHORT) {
        key_event_flags |= KEY_FLAG_KEY2_SHORT;
    } else if (event == KEY_EVENT_KEY3_SHORT) {
        key_event_flags |= KEY_FLAG_KEY3_SHORT;
    } else if (event == KEY_EVENT_KEY1_LONG) {
        key_event_flags |= KEY_FLAG_KEY1_LONG;
    } else if (event == KEY_EVENT_KEY4_SHORT) {
        key_event_flags |= KEY_FLAG_KEY4_SHORT;
    } else if (event == KEY_EVENT_KEY4_LONG) {
        key_event_flags |= KEY_FLAG_KEY4_LONG;
    }
}

void Key_Init(void)
{
    __disable_irq();
    key_event_flags = 0U;
    __enable_irq();
}

void Key_Tick(void)
{
    static uint8_t sample_state[KEY_COUNT] = {0U};
    static uint8_t stable_state[KEY_COUNT] = {0U};
    static uint8_t stable_ticks[KEY_COUNT] = {0U};
    static uint8_t key1_long_sent = 0U;
    static uint8_t key4_long_sent = 0U;
    static uint16_t press_ticks[KEY_COUNT] = {0U};
    uint8_t i;
    uint8_t raw_state;

    for (i = 0U; i < KEY_COUNT; i++) {
        raw_state = Key_GetState(i);

        if (raw_state == sample_state[i]) {
            if (stable_ticks[i] < KEY_DEBOUNCE_TICKS) {
                stable_ticks[i]++;
            }
        } else {
            sample_state[i] = raw_state;
            stable_ticks[i] = 1U;
        }

        if ((stable_ticks[i] >= KEY_DEBOUNCE_TICKS) &&
            (sample_state[i] != stable_state[i])) {
            stable_state[i] = sample_state[i];

            if (stable_state[i] == KEY_PRESSED) {
                press_ticks[i] = 0U;
                if (i == KEY_1) {
                    key1_long_sent = 0U;
                } else if (i == KEY_4) {
                    key4_long_sent = 0U;
                }
            } else {
                if (i == KEY_1) {
                    if (key1_long_sent == 0U) {
                        Key_SetEvent(KEY_EVENT_KEY1_SHORT);
                    }
                    key1_long_sent = 0U;
                } else if (i == KEY_2) {
                    Key_SetEvent(KEY_EVENT_KEY2_SHORT);
                } else if (i == KEY_3) {
                    Key_SetEvent(KEY_EVENT_KEY3_SHORT);
                } else if (i == KEY_4) {
                    if (key4_long_sent == 0U) {
                        Key_SetEvent(KEY_EVENT_KEY4_SHORT);
                    }
                    key4_long_sent = 0U;
                }
                press_ticks[i] = 0U;
            }
        }

        if (stable_state[i] == KEY_PRESSED) {
            if (press_ticks[i] < 0xFFFFU) {
                press_ticks[i]++;
            }

            if ((i == KEY_1) && (key1_long_sent == 0U) &&
                (press_ticks[i] >= KEY_LONG_PRESS_TICKS)) {
                Key_SetEvent(KEY_EVENT_KEY1_LONG);
                key1_long_sent = 1U;
            } else if ((i == KEY_4) && (key4_long_sent == 0U) &&
                (press_ticks[i] >= KEY_LONG_PRESS_TICKS)) {
                Key_SetEvent(KEY_EVENT_KEY4_LONG);
                key4_long_sent = 1U;
            }
        }
    }
}

uint8_t Key_GetNum(void)
{
    uint8_t event = KEY_EVENT_NONE;

    __disable_irq();

    if ((key_event_flags & KEY_FLAG_KEY1_LONG) != 0U) {
        key_event_flags &= (uint16_t)~KEY_FLAG_KEY1_LONG;
        event = KEY_EVENT_KEY1_LONG;
    } else if ((key_event_flags & KEY_FLAG_KEY4_LONG) != 0U) {
        key_event_flags &= (uint16_t)~KEY_FLAG_KEY4_LONG;
        event = KEY_EVENT_KEY4_LONG;
    } else if ((key_event_flags & KEY_FLAG_KEY1_SHORT) != 0U) {
        key_event_flags &= (uint16_t)~KEY_FLAG_KEY1_SHORT;
        event = KEY_EVENT_KEY1_SHORT;
    } else if ((key_event_flags & KEY_FLAG_KEY2_SHORT) != 0U) {
        key_event_flags &= (uint16_t)~KEY_FLAG_KEY2_SHORT;
        event = KEY_EVENT_KEY2_SHORT;
    } else if ((key_event_flags & KEY_FLAG_KEY3_SHORT) != 0U) {
        key_event_flags &= (uint16_t)~KEY_FLAG_KEY3_SHORT;
        event = KEY_EVENT_KEY3_SHORT;
    } else if ((key_event_flags & KEY_FLAG_KEY4_SHORT) != 0U) {
        key_event_flags &= (uint16_t)~KEY_FLAG_KEY4_SHORT;
        event = KEY_EVENT_KEY4_SHORT;
    }

    __enable_irq();

    return event;
}

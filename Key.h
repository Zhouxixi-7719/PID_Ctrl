#ifndef KEY_H_
#define KEY_H_

#include <stdint.h>

#define KEY_EVENT_NONE                 0U
#define KEY_EVENT_KEY1_SHORT           1U
#define KEY_EVENT_KEY2_SHORT           2U
#define KEY_EVENT_KEY3_SHORT           3U
#define KEY_EVENT_KEY4_SHORT           4U
#define KEY_EVENT_KEY1_LONG            11U
#define KEY_EVENT_KEY4_LONG            14U

void Key_Init(void);
void Key_Tick(void);
uint8_t Key_GetNum(void);

#endif /* KEY_H_ */

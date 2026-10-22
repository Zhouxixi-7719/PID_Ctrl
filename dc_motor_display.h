#ifndef DC_MOTOR_DISPLAY_H_
#define DC_MOTOR_DISPLAY_H_

#include <stdint.h>

void dc_motor_display_init(void);
void dc_motor_display_update(uint32_t now_ms);
void Display_UpdateVisionState(uint8_t cmd, uint16_t x, uint16_t y,
    uint16_t distance_mm, uint8_t valid);

/* 开机阶段(主循环还没跑起来)在屏上顶两行显示提示 */
void Display_ShowBootMessage(const char *line0, const char *line1);
void TestMenu_Tick(void);
void TestMenu_ProcessKey(uint8_t key);
void TestMenu_StartMainRunStopMonitor(void);
void TestMenu_ClearMainRunStopMonitor(void);
uint8_t TestMenu_IsDisplayActive(void);
uint8_t TestMenu_IsTimerActive(void);

#endif /* DC_MOTOR_DISPLAY_H_ */

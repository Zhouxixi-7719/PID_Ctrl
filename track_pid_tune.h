#ifndef TRACK_PID_TUNE_H_
#define TRACK_PID_TUNE_H_

#include <stdint.h>

/* 每按一次 UP/DOWN 时，Kp/Ki/Kd 分别变化的步长。 */
#define TRACK_PID_KP_STEP              0.05f
#define TRACK_PID_KI_STEP              0.01f
#define TRACK_PID_KD_STEP              0.05f

/* 按键调参允许的参数范围，防止误操作把 PID 调到不可控。 */
#define TRACK_PID_KP_MIN               0.0f
#define TRACK_PID_KP_MAX               3.0f
#define TRACK_PID_KI_MIN               0.0f
#define TRACK_PID_KI_MAX               1.0f
#define TRACK_PID_KD_MIN               0.0f
#define TRACK_PID_KD_MAX               3.0f

/*每次调节参数时的步长和参数的范围*/
#define TRACK_TUNE_SPEED_STEP_CM_S     1.0f
#define TRACK_TUNE_SPEED_MIN_CM_S      5.0f
#define TRACK_TUNE_SPEED_MAX_CM_S      75.0f

#define TRACK_TUNE_TURN_PWM_STEP       100
#define TRACK_TUNE_TURN_PWM_MIN        800
#define TRACK_TUNE_TURN_PWM_MAX        3000

#define TRACK_TUNE_TURN_MS_STEP        10U
#define TRACK_TUNE_TURN_STOP_MS_MIN    0U
#define TRACK_TUNE_TURN_STOP_MS_MAX    300U
#define TRACK_TUNE_TURN_LOCK_MS_MIN    0U
#define TRACK_TUNE_TURN_LOCK_MS_MAX    300U

typedef enum {
    TRACK_PID_TUNE_PARAM_KP = 0,       /* 当前选中调节 Kp。 */
    TRACK_PID_TUNE_PARAM_KI,           /* 当前选中调节 Ki。 */
    TRACK_PID_TUNE_PARAM_KD            /* 当前选中调节 Kd。 */
} track_pid_tune_param_t;

typedef enum {
    TRACK_PID_TUNE_ITEM_KP = 0,
    TRACK_PID_TUNE_ITEM_KI,
    TRACK_PID_TUNE_ITEM_KD,
    TRACK_PID_TUNE_ITEM_SPEED,
    TRACK_PID_TUNE_ITEM_TURN_PWM,
    TRACK_PID_TUNE_ITEM_TURN_STOP_MS,
    TRACK_PID_TUNE_ITEM_TURN_LOCK_MS,
    TRACK_PID_TUNE_ITEM_BALL_REF,
    TRACK_PID_TUNE_ITEM_BALL_KP,
    TRACK_PID_TUNE_ITEM_BALL_KI,
    TRACK_PID_TUNE_ITEM_BALL_KD,
    TRACK_PID_TUNE_ITEM_BALL_ENABLE,
    TRACK_PID_TUNE_ITEM_BALL_ZERO,
    TRACK_PID_TUNE_ITEM_COUNT
} track_pid_tune_item_t;

/* 初始化按键调参模块，并把调参缓存初始化为 gray_sensor.h 里的宏定义初值。 */
void TrackPidTune_Init(void);

void TrackPidTune_ProcessKey(uint8_t key);

/* 关闭调参界面预留接口，目前没有额外动作。 */
void TrackPidTune_Close(void);

/* 查询调参显示界面是否占用屏幕，目前 OLED 与电机显示共用所以返回 0。 */
uint8_t TrackPidTune_IsDisplayActive(void);

/* 获取当前选中的 PID 参数，用于 OLED 显示高亮。 */
track_pid_tune_param_t TrackPidTune_GetSelected(void);

track_pid_tune_item_t TrackPidTune_GetMenuItem(void);

#endif /* TRACK_PID_TUNE_H_ */

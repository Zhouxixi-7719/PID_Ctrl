#include "ti_msp_dl_config.h"
#include "dc_encoder.h"
#include "dc_motor.h"
#include "dc_motor_display.h"
#include "ball_step_ctrl.h"
#include "ball_route_ctrl.h"
#include "cascade_ctrl.h"
#include "gray_sensor.h"
#include "Key.h"
#include "mpu6050.h"
#include "speed_ctrl.h"
#include "step_motor.h"
#include "stepper_axis.h"
#include "track_pid_tune.h"
#include "vision_proto.h"
#include "uart.h"
#include "yaw.h"
#include "Delay.h"

#define MAIN_MAX_VISION_FRAMES_PER_LOOP  4U
#define MAIN_DEBUG_UART_ENABLE            0U
#define MAIN_DEBUG_UART_PERIOD_MS         20U
#define MAIN_USE_CASCADE_CTRL             1U

/*
 * 上电到标定之间的等待。MPU6050 上电需要几十毫秒稳定，芯片自热
 * 也会持续改变零偏，多等一会儿标出来的 bias 更接近实际工作温度。
 */
#define MPU_WARMUP_MS                   2000U

static volatile uint32_t system_ms = 0U;
static volatile uint8_t ball_step_task_due = 0U;
static volatile uint8_t cascade_task_due = 0U;
#if MAIN_DEBUG_UART_ENABLE
static uint32_t main_debug_uart_last_ms = 0U;
#endif

void SysTick_Handler(void)
{
    static uint8_t speed_tick_ms = 0U;
    static uint8_t cascade_tick_ms = 0U;

    system_ms++;

    cascade_tick_ms++;
    if (cascade_tick_ms >= CASCADE_INNER_PERIOD_MS) {
        cascade_tick_ms = 0U;
        cascade_task_due = 1U;
    }

    speed_tick_ms++;
    if (speed_tick_ms >= SPEED_CTRL_PERIOD_MS) {
        speed_tick_ms = 0U;
        SpeedCtrl_Task10ms();
        ball_step_task_due = 1U;
        Key_Tick();
        TestMenu_Tick();
        /*
         * 只置标志。I2C 读和积分一律在主循环的 Yaw_Task 里做——
         * MPU 的读是阻塞轮询，失败时要跑满超时循环，绝不能放进
         * 这个 1ms 周期的 ISR。
         */
        Yaw_Tick();
    }
}

int main(void)
{
    vp_frame_t frame;
    uint8_t tx_buf[16];
    uint16_t x;
    uint16_t y;
    uint16_t distance_mm;
    int16_t err_x;
    int16_t err_y;
    uint8_t distance_valid;
    uint8_t key_num;
    uint8_t frame_count;

    SYSCFG_DL_init();
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);             //设置中断频率
    NVIC_SetPriority(SysTick_IRQn, 2);
    DL_Timer_startCounter(SERVO_INST);
    DL_Timer_setCaptureCompareValue(SERVO_INST, 50, GPIO_SERVO_C1_IDX);

    dc_motor_init();
    step_motor_init();
    BallStepCtrl_Init();
    stepper_axis_init();
    CascadeCtrl_Init();
    BallRouteCtrl_Init();
#if MAIN_USE_CASCADE_CTRL
    BallStepCtrl_SetEnabled(0U);
#endif
    dc_encoder_init();
    Key_Init();
    Track_Init();
    TrackPidTune_Init();
    SpeedCtrl_Init();
    dc_motor_display_init();

    /*
     * 陀螺仪零偏标定：必须在车完全静止、电机不转时做。
     * 此处电机还没被使能(SpeedCtrl 默认停止)，是最干净的时机。
     * 标定失败(比如上电时车被拿在手里)不阻塞启动，之后可以进
     * ENC/MPU TEST 菜单按 KEY2 手动重标。
     */
    if (MPU6050_Init() != 0U) {
        Display_ShowBootMessage("MPU WARMUP...", "KEEP CAR STILL");
        delay_ms(MPU_WARMUP_MS);

        Display_ShowBootMessage("CALIBRATING...", "KEEP CAR STILL");
        if (MPU6050_CalibrateMotion(MPU6050_CAL_SAMPLES,
                MPU6050_CAL_DELAY_MS) != 0U) {
            Display_ShowBootMessage("MPU CAL OK", "");
        } else {
            Display_ShowBootMessage("MPU CAL FAILED", "K4->TEST, K2 RETRY");
            delay_ms(1500U);
        }
    } else {
        Display_ShowBootMessage("MPU INIT FAILED", "CHECK I2C WIRING");
        delay_ms(1500U);
    }

    Yaw_Init();

    NVIC_EnableIRQ(vision_proto_INST_INT_IRQN);

    while (1) {
        Yaw_Task(system_ms);

        frame_count = 0U;
        while ((frame_count < MAIN_MAX_VISION_FRAMES_PER_LOOP) &&
            vp_get_frame(&frame)) {
            frame_count++;
            switch (frame.cmd) {
                case VP_CMD_MOVE:
                    if (vp_parse_move(&frame, &x, &y)) {
#if MAIN_USE_CASCADE_CTRL
#if CASCADE_INPUT_MODE == CASCADE_INPUT_MOVE_Y
                            BallRouteCtrl_SetSerialPosition(y, 1U, system_ms);
#endif
#else
                        BallStepCtrl_UpdatePosition(y, 1U, system_ms);
#endif
                        uart_send(tx_buf,
                            vp_pack_ack(tx_buf, VP_CMD_MOVE));
                    }
                    break;

                case VP_CMD_ERR:
                    if (vp_parse_error(&frame, &err_x, &err_y)) {
#if !MAIN_USE_CASCADE_CTRL
                        if (BallStepCtrl_IsEnabled() != 0U) {
                            BallStepCtrl_UpdateError(err_x, 1U, system_ms);
                        }
#endif
                        uart_send(tx_buf,
                            vp_pack_ack(tx_buf, VP_CMD_ERR));
                    }
                    break;

                case VP_CMD_DISTANCE_STATE:
                    if (vp_parse_distance_state(&frame, &x, &y,
                            &distance_mm, &distance_valid)) {
                        Display_UpdateVisionState(frame.cmd, x, y,
                            distance_mm, distance_valid);
#if MAIN_USE_CASCADE_CTRL
#if CASCADE_INPUT_MODE == CASCADE_INPUT_DISTANCE_MM
                            BallRouteCtrl_SetSerialPosition(distance_mm,
                                distance_valid, system_ms);
#endif
#else
                        BallStepCtrl_UpdatePosition(distance_mm,
                            distance_valid,
                            system_ms);
#endif
                        uart_send(tx_buf,
                            vp_pack_ack(tx_buf, VP_CMD_DISTANCE_STATE));
                    }
                    break;

                case VP_CMD_STOP:
                    /* 启停由物理按键负责，忽略上位机停止命令。 */
                    uart_send(tx_buf,
                        vp_pack_ack(tx_buf, VP_CMD_STOP));
                    break;

                default:
                    break;
            }
        }

        if (ball_step_task_due != 0U) {
            __disable_irq();
            ball_step_task_due = 0U;
            __enable_irq();
#if !MAIN_USE_CASCADE_CTRL
            BallStepCtrl_Task10ms(system_ms);
#endif
        }

        if (cascade_task_due != 0U) {
            __disable_irq();
            cascade_task_due = 0U;
            __enable_irq();
#if MAIN_USE_CASCADE_CTRL
            CascadeCtrl_Task5ms(system_ms);
            BallRouteCtrl_Task5ms(system_ms);
#endif
        }

        key_num = Key_GetNum();
        if ((TestMenu_IsDisplayActive() != 0U) ||
            (key_num == KEY_EVENT_KEY4_SHORT) ||
            (key_num == KEY_EVENT_KEY4_LONG) ||
            ((TestMenu_IsTimerActive() != 0U) &&
            ((key_num == KEY_EVENT_KEY2_SHORT) ||
                (key_num == KEY_EVENT_KEY3_SHORT)))) {
            if (key_num == KEY_EVENT_KEY4_SHORT) {
                TrackPidTune_Close();
            }
            TestMenu_ProcessKey(key_num);
        } else if ((TrackPidTune_IsDisplayActive() != 0U) ||
            (key_num == KEY_EVENT_KEY1_SHORT) ||
            (key_num == KEY_EVENT_KEY1_LONG)) {
            TrackPidTune_ProcessKey(key_num);
        } else {
            switch (key_num) {
                case KEY_EVENT_KEY2_SHORT:
                    SpeedCtrl_Stop();
                    Track_Disable();
                    TestMenu_ClearMainRunStopMonitor();
                    TrackPidTune_Close();
                    break;

                case KEY_EVENT_KEY3_SHORT:
                    Track_Enable();
                    SpeedCtrl_Start(SpeedCtrl_GetRunSpeed());
                    TestMenu_StartMainRunStopMonitor();
                    break;

                default:
                    break;
            }
        }

        dc_motor_display_update(system_ms);

#if MAIN_DEBUG_UART_ENABLE
        if ((uint32_t)(system_ms - main_debug_uart_last_ms) >=
            MAIN_DEBUG_UART_PERIOD_MS) {
            main_debug_uart_last_ms = system_ms;
            uart_debug_send_next(system_ms);
        }
#endif
    }
}

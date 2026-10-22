#include "dc_motor_display.h"

#include "ball_step_ctrl.h"
#include "ball_route_ctrl.h"
#include "cascade_ctrl.h"
#include "dc_encoder.h"
#include "gray_sensor.h"
#include "Key.h"
#include "mpu6050.h"
#include "oled.h"
#include "speed_ctrl.h"
#include "step_motor.h"
#include "stepper_axis.h"
#include "track_pid_tune.h"
#include "yaw.h"

#define DC_MOTOR_DISPLAY_REFRESH_MS    100U
#define DC_MOTOR_DISPLAY_LINE_COUNT    5U
#define DC_MOTOR_DISPLAY_LINE_CHARS    21U

#define TEST_MENU_PAGE_MAIN            0U
#define TEST_MENU_PAGE_TIMER           1U
#define TEST_MENU_PAGE_STRAIGHT        2U
#define TEST_MENU_PAGE_BALL            3U
#define TEST_MENU_PAGE_ROUTE           4U
#define TEST_MENU_PAGE_COUNT           5U
#define TEST_MENU_TIMER_FINISH_MASK    0x3CU
#define TEST_MENU_TIMER_STOP_SETTLE_MS 100U

static char display_line_cache[DC_MOTOR_DISPLAY_LINE_COUNT]
    [DC_MOTOR_DISPLAY_LINE_CHARS + 1U];

static const uint8_t display_line_y[DC_MOTOR_DISPLAY_LINE_COUNT] = {
    0U, 16U, 28U, 40U, 52U
};

static volatile uint8_t test_menu_active = 0U;
static volatile uint8_t test_menu_page = TEST_MENU_PAGE_MAIN;
static volatile uint8_t test_menu_timer_armed = 0U;
static volatile uint8_t test_menu_timer_running = 0U;
static volatile uint8_t test_menu_timer_decelerating = 0U;
static volatile uint8_t main_run_stop_armed = 0U;
static volatile uint16_t test_menu_timer_stop_settle_ms = 0U;
static volatile uint32_t test_menu_timer_elapsed_ms = 0U;
static volatile float test_menu_timer_max_error_mm = 0.0f;
static uint8_t test_menu_cal_request = 0U;
static volatile uint8_t display_vision_seen = 0U;
static volatile uint8_t display_vision_cmd = 0U;
static volatile uint8_t display_vision_valid = 0U;
static volatile uint16_t display_vision_x = 0U;
static volatile uint16_t display_vision_y = 0U;
static volatile uint16_t display_vision_distance_mm = 0U;

static int16_t float_to_int16(float value);
static uint8_t append_signed_int4(char *buf, uint8_t index, int16_t value);
static uint8_t append_unsigned_hundredth(char *buf, uint8_t index,
    float value);

static uint8_t line_equals(const char *left, const char *right)
{
    uint8_t i;

    for (i = 0U; i <= DC_MOTOR_DISPLAY_LINE_CHARS; i++) {
        if (left[i] != right[i]) {
            return 0U;
        }
    }

    return 1U;
}

static void make_padded_line(char *dst, const char *src)
{
    uint8_t i = 0U;

    while ((i < DC_MOTOR_DISPLAY_LINE_CHARS) && (src[i] != '\0')) {
        dst[i] = src[i];
        i++;
    }

    while (i < DC_MOTOR_DISPLAY_LINE_CHARS) {
        dst[i++] = ' ';
    }

    dst[DC_MOTOR_DISPLAY_LINE_CHARS] = '\0';
}

static void copy_line(char *dst, const char *src)
{
    uint8_t i;

    for (i = 0U; i <= DC_MOTOR_DISPLAY_LINE_CHARS; i++) {
        dst[i] = src[i];
    }
}

static void reset_line_cache(void)
{
    uint8_t row;
    uint8_t col;

    for (row = 0U; row < DC_MOTOR_DISPLAY_LINE_COUNT; row++) {
        for (col = 0U; col <= DC_MOTOR_DISPLAY_LINE_CHARS; col++) {
            display_line_cache[row][col] = '\0';
        }
    }
}

static void show_line_if_changed(uint8_t row, const char *src)
{
    char padded[DC_MOTOR_DISPLAY_LINE_CHARS + 1U];

    if (row >= DC_MOTOR_DISPLAY_LINE_COUNT) {
        return;
    }

    make_padded_line(padded, src);
    if (line_equals(display_line_cache[row], padded) != 0U) {
        return;
    }

    copy_line(display_line_cache[row], padded);
    OLED_ShowString(0, display_line_y[row], (u8 *)padded, 12);
}

static uint8_t append_text(char *buf, uint8_t index, const char *text)
{
    while ((text[0] != '\0') && (index < DC_MOTOR_DISPLAY_LINE_CHARS)) {
        buf[index++] = text[0];
        text++;
    }

    buf[index] = '\0';
    return index;
}

static uint8_t append_unsigned_int4(char *buf, uint8_t index, uint16_t value)
{
    if (value > 9999U) {
        value = 9999U;
    }

    buf[index++] = (char)('0' + (value / 1000U) % 10U);
    buf[index++] = (char)('0' + (value / 100U) % 10U);
    buf[index++] = (char)('0' + (value / 10U) % 10U);
    buf[index++] = (char)('0' + value % 10U);
    buf[index] = '\0';
    return index;
}

static uint8_t append_unsigned_tenth(char *buf, uint8_t index, float value)
{
    uint16_t tenths;

    if (value < 0.0f) {
        value = 0.0f;
    }

    tenths = (uint16_t)float_to_int16(value * 10.0f);
    if (tenths > 999U) {
        tenths = 999U;
    }

    buf[index++] = (char)('0' + (tenths / 100U) % 10U);
    buf[index++] = (char)('0' + (tenths / 10U) % 10U);
    buf[index++] = '.';
    buf[index++] = (char)('0' + tenths % 10U);
    buf[index] = '\0';
    return index;
}

static uint8_t append_signed_int6(char *buf, uint8_t index, int32_t value)
{
    uint32_t magnitude;
    uint32_t div = 100000U;

    if (value < 0) {
        buf[index++] = '-';
        magnitude = (uint32_t)(-value);
    } else {
        buf[index++] = '+';
        magnitude = (uint32_t)value;
    }

    if (magnitude > 999999U) {
        magnitude = 999999U;
    }

    while (div != 0U) {
        buf[index++] = (char)('0' + (magnitude / div) % 10U);
        div /= 10U;
    }

    buf[index] = '\0';
    return index;
}

/* 输出形如 "+12.34"，两位小数，用来看零偏这种小量 */
static uint8_t append_signed_tenth(char *buf, uint8_t index, float value)
{
    int16_t tenths = float_to_int16(value * 10.0f);
    uint16_t magnitude;

    if (tenths < 0) {
        buf[index++] = '-';
        magnitude = (uint16_t)(-tenths);
    } else {
        buf[index++] = '+';
        magnitude = (uint16_t)tenths;
    }

    if (magnitude > 9999U) {
        magnitude = 9999U;
    }

    buf[index++] = (char)('0' + (magnitude / 1000U) % 10U);
    buf[index++] = (char)('0' + (magnitude / 100U) % 10U);
    buf[index++] = (char)('0' + (magnitude / 10U) % 10U);
    buf[index++] = '.';
    buf[index++] = (char)('0' + magnitude % 10U);
    buf[index] = '\0';
    return index;
}

static uint8_t append_signed_small_tenth(char *buf, uint8_t index,
    float value)
{
    int16_t tenths = float_to_int16(value * 10.0f);
    uint16_t magnitude;

    if (tenths < 0) {
        buf[index++] = '-';
        magnitude = (uint16_t)(-tenths);
    } else {
        buf[index++] = '+';
        magnitude = (uint16_t)tenths;
    }
    if (magnitude > 99U) {
        magnitude = 99U;
    }
    buf[index++] = (char)('0' + (magnitude / 10U));
    buf[index++] = '.';
    buf[index++] = (char)('0' + (magnitude % 10U));
    buf[index] = '\0';
    return index;
}

static void test_menu_enter(uint8_t page)
{
    test_menu_active = 1U;
    test_menu_page = page;
    test_menu_cal_request = 0U;
    if ((page == TEST_MENU_PAGE_TIMER) ||
        (page == TEST_MENU_PAGE_STRAIGHT)) {
        test_menu_timer_armed = 1U;
    }
    if (page == TEST_MENU_PAGE_MAIN) {
        Yaw_Reset();
        Encoder_TestReset();
    }
    reset_line_cache();
}

static void test_menu_exit(void)
{
    if ((test_menu_timer_running != 0U) ||
        (test_menu_timer_decelerating != 0U)) {
        SpeedCtrl_Stop();
        Track_Disable();
    }
    test_menu_active = 0U;
    test_menu_page = TEST_MENU_PAGE_MAIN;
    test_menu_timer_armed = 0U;
    test_menu_timer_running = 0U;
    test_menu_timer_decelerating = 0U;
    test_menu_timer_stop_settle_ms = 0U;
    reset_line_cache();
}

#if 0
static void test_menu_show_main(uint32_t now_ms)
{
    char line[DC_MOTOR_DISPLAY_LINE_CHARS + 1U];
    uint8_t i;

    (void)now_ms;

    if (test_menu_page == TEST_MENU_PAGE_TIMER) {
        test_menu_show_timer();
        OLED_Refresh();
        return;
    }

    test_menu_show_main(now_ms);
    OLED_Refresh();
    return;

    if (test_menu_cal_request != 0U) {
        test_menu_run_calibration();
    }

    i = append_text(line, 0U, "MPU ");
    (void)append_text(line, i, test_menu_cal_text());
    show_line_if_changed(0U, line);

    i = append_text(line, 0U, "EL:");
    (void)append_signed_int6(line, i, Encoder_TestGetCountL());
    show_line_if_changed(1U, line);

    i = append_text(line, 0U, "ER:");
    (void)append_signed_int6(line, i, Encoder_TestGetCountR());
    show_line_if_changed(2U, line);

    /* 瑙掗€熷害鍜岀疮绉埅鍚戣閮藉彇鑷?Yaw 妯″潡锛岄伩鍏嶅拰瀹冩姠 I2C 閲嶅璇?*/
    i = append_text(line, 0U, "GZ:");
    i = append_signed_tenth(line, i, Yaw_GetRate());
    i = append_text(line, i, " AZ:");
    (void)append_signed_tenth(line, i, Yaw_GetAngle());
    show_line_if_changed(3U, line);

    i = append_text(line, 0U, "AX:");
    i = append_signed_tenth(line, i, Yaw_GetAccelX());
    i = append_text(line, i, " Y:");
    (void)append_signed_tenth(line, i, Yaw_GetAccelY());
    show_line_if_changed(4U, line);
}
#endif

static void test_menu_show_timer(void)
{
    char line[DC_MOTOR_DISPLAY_LINE_CHARS + 1U];
    uint32_t elapsed_cs = test_menu_timer_elapsed_ms / 10U;
    float raw_error = 0.0f;
    uint8_t i;

    if (elapsed_cs > 9999U) {
        elapsed_cs = 9999U;
    }

    show_line_if_changed(0U, "RACE TIMER");

    i = append_text(line, 0U, "TIME:");
    line[i++] = (char)('0' + (elapsed_cs / 1000U) % 10U);
    line[i++] = (char)('0' + (elapsed_cs / 100U) % 10U);
    line[i++] = '.';
    line[i++] = (char)('0' + (elapsed_cs / 10U) % 10U);
    line[i++] = (char)('0' + elapsed_cs % 10U);
    line[i] = '\0';
    (void)append_text(line, i, "S");
    show_line_if_changed(1U, line);

    if (test_menu_timer_running != 0U) {
        show_line_if_changed(2U, "STATE:RUN");
    } else if (test_menu_timer_decelerating != 0U) {
        show_line_if_changed(2U, "STATE:BRAKE");
    } else {
        show_line_if_changed(2U, "STATE:PAUSE");
    }

    i = append_text(line, 0U, "P:");
    if (CascadeCtrl_IsInputValid() != 0U) {
        i = append_unsigned_int4(line, i,
            (uint16_t)CascadeCtrl_GetRawPosition());
        raw_error = CascadeCtrl_GetTarget() -
            CascadeCtrl_GetRawPosition();
    } else {
        i = append_text(line, i, "----");
    }
    i = append_text(line, i, BallRouteCtrl_IsCenterHold() != 0U ?
        " HOLD" : " OFF");
    i = append_text(line, i, " V:");
    (void)append_text(line, i, CascadeCtrl_IsInputValid() != 0U ?
        "OK" : "--");
    show_line_if_changed(3U, line);

    i = append_text(line, 0U, "E:");
    i = append_signed_tenth(line, i, raw_error);
    i = append_text(line, i, " M:");
    (void)append_signed_tenth(line, i, test_menu_timer_max_error_mm);
    show_line_if_changed(4U, line);
}

static void test_menu_show_straight(void)
{
    char line[DC_MOTOR_DISPLAY_LINE_CHARS + 1U];
    float average_speed = (SpeedCtrl_GetSpeedL() +
        SpeedCtrl_GetSpeedR()) * 0.5f;
    uint8_t i;

    show_line_if_changed(0U, "STRAIGHT RUN");

    i = append_text(line, 0U, "SET:");
    i = append_unsigned_tenth(line, i, SpeedCtrl_GetRunSpeed());
    i = append_text(line, i, " ACT:");
    (void)append_signed_tenth(line, i, average_speed);
    show_line_if_changed(1U, line);

    if (test_menu_timer_running != 0U) {
        show_line_if_changed(2U, "STATE:RUN");
    } else if (test_menu_timer_decelerating != 0U) {
        show_line_if_changed(2U, "STATE:BRAKE");
    } else {
        show_line_if_changed(2U, "STATE:READY");
    }

    i = append_text(line, 0U, "L:");
    i = append_signed_tenth(line, i, SpeedCtrl_GetSpeedL());
    i = append_text(line, i, " R:");
    (void)append_signed_tenth(line, i, SpeedCtrl_GetSpeedR());
    show_line_if_changed(3U, line);

    i = append_text(line, 0U, "A:");
    i = append_signed_tenth(line, i,
        SpeedCtrl_GetProfileAcceleration());
    i = append_text(line, i, BallRouteCtrl_IsCenterHold() != 0U ?
        " HOLD:ON" : " HOLD:OFF");
    show_line_if_changed(4U, line);
}

static void test_menu_show_ball(void)
{
    char line[DC_MOTOR_DISPLAY_LINE_CHARS + 1U];
    uint8_t i;
    uint8_t cascade_active = (BallStepCtrl_IsEnabled() == 0U) ?
        1U : 0U;
    uint8_t valid;
    uint16_t raw_position;
    int16_t reference;
    float position_error;
    float velocity_or_rpm;
    float output_angle;
    float current_angle;

    if (cascade_active != 0U) {
        valid = CascadeCtrl_IsInputValid();
        raw_position = (uint16_t)CascadeCtrl_GetRawPosition();
        reference = (int16_t)CascadeCtrl_GetTarget();
        position_error = CascadeCtrl_GetPositionError();
        velocity_or_rpm = CascadeCtrl_GetRpmCommand();
        output_angle = CascadeCtrl_GetAngleReference();
        current_angle = CascadeCtrl_GetAngleFeedback();
    } else {
        valid = BallStepCtrl_IsPositionValid();
        raw_position = BallStepCtrl_GetRawPosition();
        reference = BallStepCtrl_GetReference();
        position_error = BallStepCtrl_GetError();
        velocity_or_rpm = BallStepCtrl_GetVelocity();
        output_angle = BallStepCtrl_GetOutputAngle();
        current_angle = step_motor2_get_current_angle();
    }

    show_line_if_changed(0U, cascade_active != 0U ?
        "BALL CASCADE" : "BALL CTRL ON");

    i = append_text(line, 0U, "RAW:");
    if (valid != 0U) {
        i = append_unsigned_int4(line, i, raw_position);
    } else {
        i = append_text(line, i, "----");
    }
    i = append_text(line, i, " REF:");
    (void)append_signed_int4(line, i, reference);
    show_line_if_changed(1U, line);

    i = append_text(line, 0U, "ERR:");
    i = append_signed_tenth(line, i, position_error);
    i = append_text(line, i, cascade_active != 0U ? " RPM:" : " V:");
    (void)append_signed_tenth(line, i, velocity_or_rpm);
    show_line_if_changed(2U, line);

    i = append_text(line, 0U, "M2:");
    i = append_signed_tenth(line, i, current_angle);
    i = append_text(line, i, cascade_active != 0U ? " T:" : " CMD:");
    (void)append_signed_tenth(line, i, output_angle);
    show_line_if_changed(3U, line);

    if (valid == 0U) {
        show_line_if_changed(4U, "POS:LOST");
    } else if (cascade_active != 0U) {
        switch (BallRouteCtrl_GetState()) {
            case BALL_ROUTE_WAIT_START:
                show_line_if_changed(4U, "ROUTE:WAIT");
                break;
            case BALL_ROUTE_TO_FIRST_TARGET:
                show_line_if_changed(4U, "ROUTE:TO76");
                break;
            case BALL_ROUTE_TO_FINAL_TARGET:
                show_line_if_changed(4U, "ROUTE:TO164");
                break;
            case BALL_ROUTE_FINISHED:
                show_line_if_changed(4U, "ROUTE:DONE");
                break;
            case BALL_ROUTE_HOLD_CENTER:
                show_line_if_changed(4U, "ROUTE:CENTER");
                break;
            default:
                show_line_if_changed(4U, "ROUTE:ERROR");
                break;
        }
    } else if ((cascade_active == 0U) &&
        (BallStepCtrl_IsSettling() != 0U)) {
        show_line_if_changed(4U, "POS:STABLE");
    } else {
        show_line_if_changed(4U, "POS:VALID");
    }
}

static void test_menu_show_route(void)
{
    char line[DC_MOTOR_DISPLAY_LINE_CHARS + 1U];
    uint8_t i;
    uint8_t valid = CascadeCtrl_IsInputValid();
    uint16_t raw_position = (uint16_t)CascadeCtrl_GetRawPosition();
    int16_t target = (int16_t)CascadeCtrl_GetTarget();

    if (CascadeCtrl_IsAngleSaturated() != 0U) {
        show_line_if_changed(0U, "ROUTE CTRL SAT");
    } else if (CascadeCtrl_IsHoldActive() != 0U) {
        show_line_if_changed(0U, "ROUTE CTRL HOLD");
    } else {
        show_line_if_changed(0U, BallRouteCtrl_IsEnabled() != 0U ?
            "ROUTE CTRL ON" : "ROUTE CTRL OFF");
    }

    i = append_text(line, 0U, "RAW:");
    if (valid != 0U) {
        i = append_unsigned_int4(line, i, raw_position);
    } else {
        i = append_text(line, i, "----");
    }
    i = append_text(line, i, " T:");
    i = append_signed_int4(line, i, target);
    line[i] = '\0';
    show_line_if_changed(1U, line);

    i = append_text(line, 0U, "ERR:");
    i = append_signed_tenth(line, i, CascadeCtrl_GetPositionError());
    i = append_text(line, i, " V:");
    i = append_signed_tenth(line, i,
        CascadeCtrl_GetPositionVelocity());
    line[i] = '\0';
    show_line_if_changed(2U, line);

    switch (BallRouteCtrl_GetState()) {
        case BALL_ROUTE_WAIT_START:
            i = append_text(line, 0U, "STATE:WAIT ");
            i = append_signed_int4(line, i,
                (int16_t)BALL_ROUTE_START_POSITION);
            line[i] = '\0';
            show_line_if_changed(3U, line);
            break;
        case BALL_ROUTE_HOLD_CENTER:
            i = append_text(line, 0U, "STATE:HOLD ");
            i = append_signed_int4(line, i,
                (int16_t)BALL_ROUTE_START_POSITION);
            line[i] = '\0';
            show_line_if_changed(3U, line);
            break;
        case BALL_ROUTE_TO_FIRST_TARGET:
            i = append_text(line, 0U, "STATE:TO ");
            i = append_signed_int4(line, i,
                (int16_t)BALL_ROUTE_FIRST_TARGET);
            line[i] = '\0';
            show_line_if_changed(3U, line);
            break;
        case BALL_ROUTE_TO_FINAL_TARGET:
            i = append_text(line, 0U, "STATE:TO ");
            i = append_signed_int4(line, i,
                (int16_t)BALL_ROUTE_FINAL_TARGET);
            line[i] = '\0';
            show_line_if_changed(3U, line);
            break;
        case BALL_ROUTE_FINISHED:
            show_line_if_changed(3U, "STATE:DONE");
            break;
        default:
            show_line_if_changed(3U, "STATE:ERROR");
            break;
    }

    i = append_text(line, 0U, "I:");
    i = append_signed_small_tenth(line, i,
        CascadeCtrl_GetPositionIntegralAngle());
    i = append_text(line, i, " D:");
    i = append_signed_small_tenth(line, i,
        CascadeCtrl_GetPositionDerivativeAngle());
    i = append_text(line, i, " S:");
    i = append_signed_small_tenth(line, i,
        CascadeCtrl_GetSlopeCompensationAngle() +
        CascadeCtrl_GetImuTrimAngle());
    line[i] = '\0';
    show_line_if_changed(4U, line);
}

void TestMenu_Tick(void)
{
    float error_abs;
    uint8_t bits;

    if ((test_menu_timer_running == 0U) &&
        (test_menu_timer_decelerating == 0U) &&
        (main_run_stop_armed == 0U)) {
        return;
    }

    if ((Track_IsEnabled() == 0U) || (SpeedCtrl_IsEnabled() == 0U)) {
        test_menu_timer_running = 0U;
        test_menu_timer_decelerating = 0U;
        test_menu_timer_stop_settle_ms = 0U;
        main_run_stop_armed = 0U;
        return;
    }

    if (CascadeCtrl_IsInputValid() != 0U) {
        error_abs = CascadeCtrl_GetTarget() -
            CascadeCtrl_GetRawPosition();
        if (error_abs < 0.0f) {
            error_abs = -error_abs;
        }
        if (error_abs > test_menu_timer_max_error_mm) {
            test_menu_timer_max_error_mm = error_abs;
        }
    }

    if (test_menu_timer_decelerating != 0U) {
        if (SpeedCtrl_IsAtRest() != 0U) {
            test_menu_timer_stop_settle_ms += SPEED_CTRL_PERIOD_MS;
            if (test_menu_timer_stop_settle_ms >=
                TEST_MENU_TIMER_STOP_SETTLE_MS) {
                test_menu_timer_decelerating = 0U;
                test_menu_timer_stop_settle_ms = 0U;
                SpeedCtrl_Stop();
                Track_Disable();
                main_run_stop_armed = 0U;
            }
        } else {
            test_menu_timer_stop_settle_ms = 0U;
        }
        return;
    }

    bits = Track_GetSensorBits();
    /* 中间四路同时压黑，认为已经压到启停线。 */
    if ((bits & TEST_MENU_TIMER_FINISH_MASK) == TEST_MENU_TIMER_FINISH_MASK) {
        test_menu_timer_running = 0U;
        test_menu_timer_decelerating = 0U;
        test_menu_timer_stop_settle_ms = 0U;
        main_run_stop_armed = 0U;
        SpeedCtrl_Stop();
        Track_Disable();
        return;
    }

    test_menu_timer_elapsed_ms += SPEED_CTRL_PERIOD_MS;
}

void TestMenu_StartMainRunStopMonitor(void)
{
    test_menu_timer_elapsed_ms = 0U;
    test_menu_timer_max_error_mm = 0.0f;
    test_menu_timer_decelerating = 0U;
    test_menu_timer_stop_settle_ms = 0U;
    main_run_stop_armed = 1U;
}

void TestMenu_ClearMainRunStopMonitor(void)
{
    main_run_stop_armed = 0U;
    test_menu_timer_decelerating = 0U;
    test_menu_timer_stop_settle_ms = 0U;
}

void TestMenu_ProcessKey(uint8_t key)
{
    if (key == KEY_EVENT_KEY4_SHORT) {
        if (test_menu_active == 0U) {
            test_menu_enter(TEST_MENU_PAGE_TIMER);
        } else {
            if ((test_menu_timer_running != 0U) ||
                (test_menu_timer_decelerating != 0U)) {
                SpeedCtrl_Stop();
                Track_Disable();
                test_menu_timer_running = 0U;
                test_menu_timer_decelerating = 0U;
                test_menu_timer_stop_settle_ms = 0U;
            }
            test_menu_page = (uint8_t)((test_menu_page + 1U) %
                TEST_MENU_PAGE_COUNT);
            test_menu_timer_armed =
                ((test_menu_page == TEST_MENU_PAGE_TIMER) ||
                (test_menu_page == TEST_MENU_PAGE_STRAIGHT)) ? 1U : 0U;
            if (test_menu_timer_armed == 0U) {
                test_menu_timer_running = 0U;
                test_menu_timer_decelerating = 0U;
                test_menu_timer_stop_settle_ms = 0U;
            }
            reset_line_cache();
        }
    } else if (key == KEY_EVENT_KEY4_LONG) {
        if (test_menu_active != 0U) {
            test_menu_exit();
        }
    } else if (key == KEY_EVENT_KEY2_SHORT) {
        if ((test_menu_active != 0U) &&
            (test_menu_page == TEST_MENU_PAGE_MAIN)) {
            /* 手动重标定：实际执行放到刷新流程里，好先把提示画出来 */
            test_menu_cal_request = 1U;
        } else if ((test_menu_active != 0U) &&
            ((test_menu_page == TEST_MENU_PAGE_TIMER) ||
            (test_menu_page == TEST_MENU_PAGE_STRAIGHT))) {
            test_menu_timer_running = 0U;
            test_menu_timer_decelerating = 0U;
            test_menu_timer_stop_settle_ms = 0U;
            SpeedCtrl_Stop();
            Track_Disable();
        } else if ((test_menu_active != 0U) &&
            (test_menu_page == TEST_MENU_PAGE_BALL)) {
            return;
        } else if ((test_menu_active != 0U) &&
            (test_menu_page == TEST_MENU_PAGE_ROUTE)) {
            BallRouteCtrl_StartCenterHold();
            reset_line_cache();
        } else if (test_menu_timer_armed != 0U) {
            test_menu_timer_running = 0U;
            test_menu_timer_decelerating = 0U;
            test_menu_timer_stop_settle_ms = 0U;
            SpeedCtrl_Stop();
            Track_Disable();
        }
    } else if (key == KEY_EVENT_KEY3_SHORT) {
        if ((test_menu_active != 0U) &&
            (test_menu_page == TEST_MENU_PAGE_MAIN)) {
            Yaw_Reset();
        } else if ((test_menu_active != 0U) &&
            ((test_menu_page == TEST_MENU_PAGE_TIMER) ||
            (test_menu_page == TEST_MENU_PAGE_STRAIGHT))) {
            Track_Enable();
            SpeedCtrl_Start(SpeedCtrl_GetRunSpeed());
            test_menu_timer_elapsed_ms = 0U;
            test_menu_timer_max_error_mm = 0.0f;
            test_menu_timer_decelerating = 0U;
            test_menu_timer_stop_settle_ms = 0U;
            test_menu_timer_running = 1U;
        } else if ((test_menu_active != 0U) &&
            (test_menu_page == TEST_MENU_PAGE_BALL)) {
            return;
        } else if ((test_menu_active != 0U) &&
            (test_menu_page == TEST_MENU_PAGE_ROUTE)) {
            BallRouteCtrl_Start();
            reset_line_cache();
        } else if (test_menu_timer_armed != 0U) {
            Track_Enable();
            SpeedCtrl_Start(SpeedCtrl_GetRunSpeed());
            test_menu_timer_elapsed_ms = 0U;
            test_menu_timer_max_error_mm = 0.0f;
            test_menu_timer_decelerating = 0U;
            test_menu_timer_stop_settle_ms = 0U;
            test_menu_timer_running = 1U;
        }
    }
}

uint8_t TestMenu_IsDisplayActive(void)
{
    return test_menu_active;
}

uint8_t TestMenu_IsTimerActive(void)
{
    return ((test_menu_active != 0U) &&
        (test_menu_timer_armed != 0U)) ?
        1U : 0U;
}

void Display_UpdateVisionState(uint8_t cmd, uint16_t x, uint16_t y,
    uint16_t distance_mm, uint8_t valid)
{
    __disable_irq();
    display_vision_seen = 1U;
    display_vision_cmd = cmd;
    display_vision_x = x;
    display_vision_y = y;
    display_vision_distance_mm = distance_mm;
    display_vision_valid = (valid != 0U) ? 1U : 0U;
    __enable_irq();
}

static const char *get_menu_item_name(track_pid_tune_item_t item)
{
    switch (item) {
        case TRACK_PID_TUNE_ITEM_KP:
            return "TRACK KP";
        case TRACK_PID_TUNE_ITEM_KI:
            return "TRACK KI";
        case TRACK_PID_TUNE_ITEM_KD:
            return "TRACK KD";
        case TRACK_PID_TUNE_ITEM_SPEED:
            return "RUN SPEED";
        case TRACK_PID_TUNE_ITEM_TURN_PWM:
            return "TURN PWM";
        case TRACK_PID_TUNE_ITEM_TURN_STOP_MS:
            return "STOP MS";
        case TRACK_PID_TUNE_ITEM_TURN_LOCK_MS:
            return "LOCK MS";
        case TRACK_PID_TUNE_ITEM_BALL_REF:
            return "BALL REF";
        case TRACK_PID_TUNE_ITEM_BALL_KP:
            return "BALL KP";
        case TRACK_PID_TUNE_ITEM_BALL_KI:
            return "BALL KI";
        case TRACK_PID_TUNE_ITEM_BALL_KD:
            return "BALL KD";
        case TRACK_PID_TUNE_ITEM_BALL_ENABLE:
            return "BALL CTRL";
        case TRACK_PID_TUNE_ITEM_BALL_ZERO:
            return "BALL ZERO";
        default:
            return "";
    }
}

static void build_menu_value_line(char *line, track_pid_tune_item_t item)
{
    float kp;
    float ki;
    float kd;
    uint8_t i = 0U;

    i = append_text(line, i, "VAL:");
    if ((item == TRACK_PID_TUNE_ITEM_KP) ||
        (item == TRACK_PID_TUNE_ITEM_KI) ||
        (item == TRACK_PID_TUNE_ITEM_KD)) {
        Track_GetPid(&kp, &ki, &kd);
        if (item == TRACK_PID_TUNE_ITEM_KP) {
            (void)append_unsigned_hundredth(line, i, kp);
        } else if (item == TRACK_PID_TUNE_ITEM_KI) {
            (void)append_unsigned_hundredth(line, i, ki);
        } else {
            (void)append_unsigned_hundredth(line, i, kd);
        }
    } else if (item == TRACK_PID_TUNE_ITEM_SPEED) {
        i = append_unsigned_tenth(line, i, SpeedCtrl_GetRunSpeed());
        (void)append_text(line, i, "CM/S");
    } else if (item == TRACK_PID_TUNE_ITEM_TURN_PWM) {
        (void)append_unsigned_int4(line, i, (uint16_t)Track_GetTurnPwm());
    } else if (item == TRACK_PID_TUNE_ITEM_TURN_STOP_MS) {
        i = append_unsigned_int4(line, i, Track_GetTurnStopMs());
        (void)append_text(line, i, "MS");
    } else if (item == TRACK_PID_TUNE_ITEM_TURN_LOCK_MS) {
        i = append_unsigned_int4(line, i, Track_GetTurnLockMs());
        (void)append_text(line, i, "MS");
    } else if (item == TRACK_PID_TUNE_ITEM_BALL_REF) {
        (void)append_signed_int4(line, i, BallStepCtrl_GetReference());
    } else if (item == TRACK_PID_TUNE_ITEM_BALL_KP) {
        (void)append_unsigned_hundredth(line, i, BallStepCtrl_GetKp());
    } else if (item == TRACK_PID_TUNE_ITEM_BALL_KI) {
        (void)append_unsigned_hundredth(line, i, BallStepCtrl_GetKi());
    } else if (item == TRACK_PID_TUNE_ITEM_BALL_KD) {
        (void)append_unsigned_hundredth(line, i, BallStepCtrl_GetKd());
    } else if (item == TRACK_PID_TUNE_ITEM_BALL_ENABLE) {
        (void)append_text(line, i, BallStepCtrl_IsEnabled() != 0U ?
            "ON" : "OFF");
    } else if (item == TRACK_PID_TUNE_ITEM_BALL_ZERO) {
        i = append_signed_tenth(line, i,
            step_motor2_get_zero_offset());
        (void)append_text(line, i, "DEG");
    }
}

static void dc_motor_display_update_menu(void)
{
    char line[DC_MOTOR_DISPLAY_LINE_CHARS + 1U];
    track_pid_tune_item_t item = TrackPidTune_GetMenuItem();
    uint8_t i = 0U;

    show_line_if_changed(0U, "PARAM MENU");

    i = append_text(line, i, "ITEM:");
    (void)append_text(line, i, get_menu_item_name(item));
    show_line_if_changed(1U, line);

    build_menu_value_line(line, item);
    show_line_if_changed(2U, line);

    show_line_if_changed(3U, "K1:NEXT HOLD EXIT");
    show_line_if_changed(4U, "K2:-    K3:+");

    if ((Track_IsEnabled() != 0U) && (SpeedCtrl_IsEnabled() != 0U)) {
        OLED_RefreshOnePage();
    } else {
        OLED_Refresh();
    }
}

static const char *test_menu_cal_text(void)
{
    switch (MPU6050_GetLastCalResult()) {
        case MPU6050_CAL_OK:
            return "CAL OK";
        case MPU6050_CAL_ERR_MOVING:
            return "CAL MOVE!";
        case MPU6050_CAL_ERR_IO:
            return "CAL IOERR";
        case MPU6050_CAL_ERR_PARAM:
            return "CAL PARAM";
        default:
            return "UNCAL";
    }
}

/*
 * 执行一次手动标定。会阻塞约 1 秒，期间 OLED 不刷新，所以先把
 * 提示画出去再开始采样。标定要求车静止，此处顺带把电机停掉。
 */
static void test_menu_run_calibration(void)
{
    test_menu_cal_request = 0U;

    SpeedCtrl_Stop();
    /*
     * 保险：标定要在主循环里连续占用 I2C 约 1 秒，若此时 Yaw 还停在
     * 关键区直采模式，SysTick 会和标定抢总线。正常流程走不到这里，
     * 但状态残留的代价是总线错乱，这一行很便宜。
     */
    Yaw_EndCritical();

    show_line_if_changed(0U, "CALIBRATING...");
    show_line_if_changed(1U, "KEEP CAR STILL");
    show_line_if_changed(2U, "");
    show_line_if_changed(3U, "");
    show_line_if_changed(4U, "");
    OLED_Refresh();

    (void)MPU6050_CalibrateMotion(MPU6050_CAL_SAMPLES, MPU6050_CAL_DELAY_MS);

    Yaw_Reset();
    reset_line_cache();
}

static void dc_motor_display_update_test(uint32_t now_ms)
{
    char line[DC_MOTOR_DISPLAY_LINE_CHARS + 1U];
    uint8_t i;

    (void)now_ms;

    if (test_menu_page == TEST_MENU_PAGE_BALL) {
        test_menu_show_ball();
        OLED_Refresh();
        return;
    }

    if (test_menu_page == TEST_MENU_PAGE_ROUTE) {
        test_menu_show_route();
        OLED_Refresh();
        return;
    }

    if (test_menu_page == TEST_MENU_PAGE_TIMER) {
        test_menu_show_timer();
        OLED_Refresh();
        return;
    }

    if (test_menu_page == TEST_MENU_PAGE_STRAIGHT) {
        test_menu_show_straight();
        OLED_Refresh();
        return;
    }

    if (test_menu_cal_request != 0U) {
        test_menu_run_calibration();
    }

    i = append_text(line, 0U, "MPU ");
    (void)append_text(line, i, test_menu_cal_text());
    show_line_if_changed(0U, line);

    i = append_text(line, 0U, "EL:");
    (void)append_signed_int6(line, i, Encoder_TestGetCountL());
    show_line_if_changed(1U, line);

    i = append_text(line, 0U, "ER:");
    (void)append_signed_int6(line, i, Encoder_TestGetCountR());
    show_line_if_changed(2U, line);

    /* 角速度和累积航向角都取自 Yaw 模块，避免和它抢 I2C 重复读 */
    i = append_text(line, 0U, "GZ:");
    i = append_signed_tenth(line, i, Yaw_GetRate());
    i = append_text(line, i, " AZ:");
    (void)append_signed_tenth(line, i, Yaw_GetAngle());
    show_line_if_changed(3U, line);

    i = append_text(line, 0U, "AX:");
    i = append_signed_tenth(line, i, Yaw_GetAccelX());
    i = append_text(line, i, " Y:");
    (void)append_signed_tenth(line, i, Yaw_GetAccelY());
    show_line_if_changed(4U, line);

    if ((Track_IsEnabled() != 0U) && (SpeedCtrl_IsEnabled() != 0U)) {
        OLED_RefreshOnePage();
    } else {
        OLED_Refresh();
    }
}

static void format_sensor_bits(char *buf, uint8_t bits)
{
    uint8_t i;

    buf[0] = 'S';
    buf[1] = ':';
    for (i = 0U; i < 8U; i++) {
        buf[2U + i] = ((bits & (uint8_t)(0x80U >> i)) != 0U) ? '1' : '0';
    }
    buf[10] = '\0';
}

static uint8_t append_signed_int3(char *buf, uint8_t index, int16_t value)
{
    uint16_t magnitude;

    if (value < 0) {
        buf[index++] = '-';
        magnitude = (uint16_t)(-value);
    } else {
        buf[index++] = '+';
        magnitude = (uint16_t)value;
    }

    if (magnitude > 999U) {
        magnitude = 999U;
    }

    buf[index++] = (char)('0' + (magnitude / 100U) % 10U);
    buf[index++] = (char)('0' + (magnitude / 10U) % 10U);
    buf[index++] = (char)('0' + magnitude % 10U);
    return index;
}

static uint8_t append_signed_int4(char *buf, uint8_t index, int16_t value)
{
    uint16_t magnitude;

    if (value < 0) {
        buf[index++] = '-';
        magnitude = (uint16_t)(-value);
    } else {
        buf[index++] = '+';
        magnitude = (uint16_t)value;
    }

    if (magnitude > 9999U) {
        magnitude = 9999U;
    }

    buf[index++] = (char)('0' + (magnitude / 1000U) % 10U);
    buf[index++] = (char)('0' + (magnitude / 100U) % 10U);
    buf[index++] = (char)('0' + (magnitude / 10U) % 10U);
    buf[index++] = (char)('0' + magnitude % 10U);
    buf[index] = '\0';
    return index;
}

static int16_t float_to_int16(float value)
{
    if (value < 0.0f) {
        return (int16_t)(value - 0.5f);
    }

    return (int16_t)(value + 0.5f);
}

static uint8_t append_unsigned_hundredth(char *buf, uint8_t index, float value)
{
    uint16_t hundredths;

    if (value < 0.0f) {
        value = 0.0f;
    }

    hundredths = (uint16_t)float_to_int16(value * 100.0f);
    if (hundredths > 999U) {
        hundredths = 999U;
    }

    buf[index++] = (char)('0' + (hundredths / 100U) % 10U);
    buf[index++] = '.';
    buf[index++] = (char)('0' + (hundredths / 10U) % 10U);
    buf[index++] = (char)('0' + hundredths % 10U);
    return index;
}

static uint8_t append_pid_value(char *buf, uint8_t index, char label,
    float value, track_pid_tune_param_t param)
{
    if (TrackPidTune_GetSelected() == param) {
        buf[index++] = '[';
        buf[index++] = label;
        buf[index++] = ']';
    } else {
        buf[index++] = label;
    }

    return append_unsigned_hundredth(buf, index, value);
}

static uint8_t format_speed_pair(char *buf, char label, float left, float right)
{
    uint8_t i = 0U;

    buf[i++] = label;
    buf[i++] = ':';
    i = append_signed_int4(buf, i, float_to_int16(left * 10.0f));
    buf[i++] = '/';
    i = append_signed_int4(buf, i, float_to_int16(right * 10.0f));
    buf[i] = '\0';
    return i;
}

static uint8_t format_pwm_pair(char *buf, int16_t left, int16_t right)
{
    uint8_t i = 0U;

    buf[i++] = 'P';
    buf[i++] = ':';
    i = append_signed_int4(buf, i, left);
    buf[i++] = '/';
    i = append_signed_int4(buf, i, right);
    buf[i] = '\0';
    return i;
}

static void format_track_state(char *buf, int16_t angle)
{
    track_state_t state = Track_GetState();
    const char *state_text = "FOL";
    uint8_t i = 0U;

    if (state == TRACK_STATE_LINE_FOLLOW) {
        state_text = (Track_GetLineMode() == TRACK_LINE_MODE_CURVE) ?
            "FOC" : "FOS";
    } else if (state == TRACK_STATE_TURN_STOP_LEFT) {
        state_text = "SL ";
    } else if (state == TRACK_STATE_TURN_STOP_RIGHT) {
        state_text = "SR ";
    } else if (state == TRACK_STATE_TURNING_LEFT) {
        state_text = "TL ";
    } else if (state == TRACK_STATE_TURNING_RIGHT) {
        state_text = "TR ";
    } else if (state == TRACK_STATE_LOST_STRAIGHT) {
        /* 丢线直行。此时 A: 位显示的是相对锁定航向的偏差角。 */
        state_text = "LST";
    }

    buf[i++] = 'S';
    buf[i++] = 'T';
    buf[i++] = ':';
    while (*state_text != '\0') {
        buf[i++] = *state_text++;
    }
    buf[i++] = ' ';
    buf[i++] = 'A';
    buf[i++] = ':';
    i = append_signed_int3(buf, i, angle);
    buf[i++] = ' ';
    buf[i++] = 'F';
    buf[i++] = ':';
    buf[i++] = Track_GetTurnFail() ? '1' : '0';
    /*
     * G 位 = 陀螺仪当前是否可用。直角弯和丢线直行都有编码器回退，
     * 掉线时角度照样有数值，这个位是唯一能看出 MPU 正在掉线的地方。
     */
    buf[i++] = ' ';
    buf[i++] = 'G';
    buf[i++] = ':';
    buf[i++] = (Yaw_IsFaulted() == 0U) ? '1' : '0';
    buf[i] = '\0';
}

void dc_motor_display_init(void)
{
    reset_line_cache();
    OLED_Init();
    show_line_if_changed(0U, "S:00000000 OFF [P]");
    show_line_if_changed(1U, "ST:FOL A:+000 F:0");
    show_line_if_changed(2U, "T:+0000/+0000 P0.00");
    show_line_if_changed(3U, "V:+0000/+0000 I0.00");
    show_line_if_changed(4U, "P:+0000/+0000 D0.00");
    OLED_Refresh();
}

void Display_ShowBootMessage(const char *line0, const char *line1)
{
    show_line_if_changed(0U, line0);
    show_line_if_changed(1U, line1);
    show_line_if_changed(2U, "");
    show_line_if_changed(3U, "");
    show_line_if_changed(4U, "");
    OLED_Refresh();
}

void dc_motor_display_update(uint32_t now_ms)
{
    static uint32_t last_refresh_ms = 0U;
    static uint8_t first_refresh = 1U;
    static uint8_t last_menu_active = 0xFFU;
    static uint8_t last_test_active = 0xFFU;
    static uint8_t last_test_page = 0xFFU;
    uint8_t test_active = TestMenu_IsDisplayActive();
    uint8_t menu_active = TrackPidTune_IsDisplayActive();
    uint8_t force_refresh = ((menu_active != last_menu_active) ||
        (test_active != last_test_active) ||
        (test_menu_page != last_test_page)) ? 1U : 0U;
    char line[22];
    int16_t angle;
    float kp;
    float ki;
    float kd;
    uint8_t i;

    /*
     * 直角弯期间给主循环让路。刷一次屏要几十毫秒，会把 Yaw_Task 的
     * 10ms 节拍拖垮，而转弯恰恰最依赖角度及时更新。转弯只有几百毫秒，
     * 这段时间不刷屏无所谓。
     */
    if (Yaw_IsCritical() != 0U) {
        return;
    }

    if ((force_refresh == 0U) && (first_refresh == 0U) &&
        (uint32_t)(now_ms - last_refresh_ms) < DC_MOTOR_DISPLAY_REFRESH_MS) {
        return;
    }

    first_refresh = 0U;
    last_refresh_ms = now_ms;

    if (force_refresh != 0U) {
        reset_line_cache();
        last_menu_active = menu_active;
        last_test_active = test_active;
        last_test_page = test_menu_page;
    }

    if (test_active != 0U) {
        dc_motor_display_update_test(now_ms);
        return;
    }

    if (menu_active != 0U) {
        dc_motor_display_update_menu();
        return;
    }

    Track_GetPid(&kp, &ki, &kd);

    format_sensor_bits(line, Track_GetSensorBits());
    if (SpeedCtrl_IsEnabled() != 0U) {
        line[10] = ' ';
        line[11] = 'O';
        line[12] = 'N';
        line[13] = ' ';
        line[14] = '[';
        if (TrackPidTune_GetSelected() == TRACK_PID_TUNE_PARAM_KI) {
            line[15] = 'I';
        } else if (TrackPidTune_GetSelected() == TRACK_PID_TUNE_PARAM_KD) {
            line[15] = 'D';
        } else {
            line[15] = 'P';
        }
        line[16] = ']';
        line[17] = '\0';
    } else {
        line[10] = ' ';
        line[11] = 'O';
        line[12] = 'F';
        line[13] = 'F';
        line[14] = ' ';
        line[15] = '[';
        if (TrackPidTune_GetSelected() == TRACK_PID_TUNE_PARAM_KI) {
            line[16] = 'I';
        } else if (TrackPidTune_GetSelected() == TRACK_PID_TUNE_PARAM_KD) {
            line[16] = 'D';
        } else {
            line[16] = 'P';
        }
        line[17] = ']';
        line[18] = '\0';
    }
    show_line_if_changed(0U, line);

    angle = float_to_int16(Track_GetTurnAngle());
    format_track_state(line, angle);
    show_line_if_changed(1U, line);

    i = format_speed_pair(line, 'T', SpeedCtrl_GetTargetL(),
        SpeedCtrl_GetTargetR());
    line[i++] = ' ';
    i = append_pid_value(line, i, 'P', kp, TRACK_PID_TUNE_PARAM_KP);
    line[i] = '\0';
    show_line_if_changed(2U, line);

    i = format_speed_pair(line, 'V', SpeedCtrl_GetSpeedL(),
        SpeedCtrl_GetSpeedR());
    line[i++] = ' ';
    i = append_pid_value(line, i, 'I', ki, TRACK_PID_TUNE_PARAM_KI);
    line[i] = '\0';
    show_line_if_changed(3U, line);

    i = format_pwm_pair(line, SpeedCtrl_GetPwmL(), SpeedCtrl_GetPwmR());
    line[i++] = ' ';
    i = append_pid_value(line, i, 'D', kd, TRACK_PID_TUNE_PARAM_KD);
    line[i] = '\0';
    show_line_if_changed(4U, line);
    if ((Track_IsEnabled() != 0U) && (SpeedCtrl_IsEnabled() != 0U)) {
        OLED_RefreshOnePage();
    } else {
        OLED_Refresh();
    }
}

#include "ball_route_ctrl.h"

#include "cascade_ctrl.h"

static ball_route_state_t route_state = BALL_ROUTE_WAIT_START;
static uint8_t route_enabled = 0U;
static uint8_t route_center_hold = 0U;
static uint8_t route_condition_active = 0U;
static uint32_t route_condition_start_ms = 0U;
static uint16_t route_condition_samples = 0U;
static uint32_t route_sample_sequence = 0U;
static uint32_t route_processed_sequence = 0U;

static float route_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static uint8_t route_position_in_range(float position, float minimum,
    float maximum)
{
    return ((position >= minimum) && (position <= maximum)) ? 1U : 0U;
}

static void route_reset_condition(void)
{
    route_condition_active = 0U;
    route_condition_start_ms = 0U;
    route_condition_samples = 0U;
}

static uint8_t route_condition_held(uint8_t condition, uint32_t now_ms,
    uint32_t hold_ms, uint16_t minimum_samples)
{
    if (condition == 0U) {
        route_reset_condition();
        return 0U;
    }

    if (route_condition_active == 0U) {
        route_condition_active = 1U;
        route_condition_start_ms = now_ms;
        route_condition_samples = 1U;
    } else if (route_condition_samples < UINT16_MAX) {
        route_condition_samples++;
    }

    return (((uint32_t)(now_ms - route_condition_start_ms) >= hold_ms) &&
        (route_condition_samples >= minimum_samples)) ? 1U : 0U;
}

static void route_set_state(ball_route_state_t new_state, float target)
{
    route_state = new_state;
    route_reset_condition();
    CascadeCtrl_SetTarget(target);
}

void BallRouteCtrl_Init(void)
{
    route_enabled = 1U;
    route_center_hold = 1U;
    CascadeCtrl_SetMotionFeedforwardEnabled(CASCADE_MOTION_FF_ENABLE);
    route_set_state(BALL_ROUTE_HOLD_CENTER, BALL_ROUTE_START_POSITION);
    CascadeCtrl_SetEnabled(1U);
}

void BallRouteCtrl_Start(void)
{
    route_enabled = 1U;
    route_center_hold = 0U;
    CascadeCtrl_SetMotionFeedforwardEnabled(CASCADE_MOTION_FF_ENABLE);
    route_processed_sequence = route_sample_sequence;
    route_set_state(BALL_ROUTE_WAIT_START, BALL_ROUTE_START_POSITION);
    CascadeCtrl_SetEnabled(1U);
}

void BallRouteCtrl_StartCenterHold(void)
{
    route_enabled = 1U;
    route_center_hold = 1U;
    CascadeCtrl_SetMotionFeedforwardEnabled(CASCADE_MOTION_FF_ENABLE);
    route_processed_sequence = route_sample_sequence;
    route_set_state(BALL_ROUTE_HOLD_CENTER, BALL_ROUTE_START_POSITION);
    CascadeCtrl_SetEnabled(1U);
}

void BallRouteCtrl_SetSerialPosition(uint16_t position, uint8_t valid,
    uint32_t now_ms)
{
    uint8_t controller_was_enabled = CascadeCtrl_IsEnabled();

    valid = CascadeCtrl_SetSerialPosition(position, valid, now_ms);
    if (valid != 0U) {
        route_sample_sequence++;
    }

    if (route_enabled == 0U) {
        CascadeCtrl_SetEnabled(0U);
        return;
    }

    if (valid == 0U) {
        route_set_state((route_center_hold != 0U) ?
            BALL_ROUTE_HOLD_CENTER : BALL_ROUTE_WAIT_START,
            BALL_ROUTE_START_POSITION);
        CascadeCtrl_SetEnabled(0U);
        return;
    }

    if (controller_was_enabled == 0U) {
        CascadeCtrl_SetEnabled(1U);
        switch (route_state) {
            case BALL_ROUTE_HOLD_CENTER:
                CascadeCtrl_SetTarget(BALL_ROUTE_START_POSITION);
                break;
            case BALL_ROUTE_TO_FIRST_TARGET:
                CascadeCtrl_SetTarget(BALL_ROUTE_FIRST_TARGET);
                break;
            case BALL_ROUTE_TO_FINAL_TARGET:
            case BALL_ROUTE_FINISHED:
                CascadeCtrl_SetTarget(BALL_ROUTE_FINAL_TARGET);
                break;
            case BALL_ROUTE_WAIT_START:
            default:
                CascadeCtrl_SetTarget(BALL_ROUTE_START_POSITION);
                break;
        }
    }
}

void BallRouteCtrl_Task5ms(uint32_t now_ms)
{
    float position;
    float velocity;
    uint8_t condition;

    if (route_enabled == 0U) {
        return;
    }

    if (CascadeCtrl_IsInputValid() == 0U) {
        ball_route_state_t idle_state = (route_center_hold != 0U) ?
            BALL_ROUTE_HOLD_CENTER : BALL_ROUTE_WAIT_START;

        if (route_state != idle_state) {
            route_set_state(idle_state,
                BALL_ROUTE_START_POSITION);
        } else {
            route_reset_condition();
        }
        return;
    }

    if (route_center_hold != 0U) {
        route_processed_sequence = route_sample_sequence;
        return;
    }

    if (route_processed_sequence == route_sample_sequence) {
        return;
    }
    route_processed_sequence = route_sample_sequence;

    position = CascadeCtrl_GetRawPosition();
    velocity = CascadeCtrl_GetPositionVelocity();

    switch (route_state) {
        case BALL_ROUTE_HOLD_CENTER:
            break;

        case BALL_ROUTE_WAIT_START:
            condition =
                (route_abs(position - BALL_ROUTE_START_POSITION) <=
                    BALL_ROUTE_START_TOLERANCE) &&
                ((route_abs(velocity) <=
                    BALL_ROUTE_START_VELOCITY_LIMIT) ||
                    (CascadeCtrl_IsHoldActive() != 0U));
            if (route_condition_held(condition, now_ms,
                    BALL_ROUTE_START_SETTLE_MS,
                    BALL_ROUTE_START_STABLE_SAMPLES) != 0U) {
                route_set_state(BALL_ROUTE_TO_FIRST_TARGET,
                    BALL_ROUTE_FIRST_TARGET);
            }
            break;

        case BALL_ROUTE_TO_FIRST_TARGET:
            condition =
                (route_position_in_range(position,
                    BALL_ROUTE_FIRST_POSITION_MIN,
                    BALL_ROUTE_FIRST_POSITION_MAX) != 0U) &&
                ((route_abs(velocity) <=
                    BALL_ROUTE_FIRST_VELOCITY_LIMIT) ||
                    (CascadeCtrl_IsHoldActive() != 0U));
            if (route_condition_held(condition, now_ms,
                    BALL_ROUTE_FIRST_SETTLE_MS,
                    BALL_ROUTE_FIRST_STABLE_SAMPLES) != 0U) {
                route_set_state(BALL_ROUTE_TO_FINAL_TARGET,
                    BALL_ROUTE_FINAL_TARGET);
            }
            break;

        case BALL_ROUTE_TO_FINAL_TARGET:
            condition =
                (route_position_in_range(position,
                    BALL_ROUTE_FINAL_POSITION_MIN,
                    BALL_ROUTE_FINAL_POSITION_MAX) != 0U) &&
                ((route_abs(velocity) <=
                    BALL_ROUTE_FINAL_VELOCITY_LIMIT) ||
                    (CascadeCtrl_IsHoldActive() != 0U));
            if (route_condition_held(condition, now_ms,
                    BALL_ROUTE_FINAL_SETTLE_MS,
                    BALL_ROUTE_FINAL_STABLE_SAMPLES) != 0U) {
                CascadeCtrl_SetTarget(position);
                route_state = BALL_ROUTE_FINISHED;
                route_reset_condition();
            }
            break;

        case BALL_ROUTE_FINISHED:
        default:
            break;
    }
}

void BallRouteCtrl_Stop(void)
{
    route_enabled = 0U;
    route_center_hold = 0U;
    CascadeCtrl_SetMotionFeedforwardEnabled(0U);
    route_state = BALL_ROUTE_WAIT_START;
    route_reset_condition();
    CascadeCtrl_SetTarget(BALL_ROUTE_START_POSITION);
    CascadeCtrl_Stop();
}

ball_route_state_t BallRouteCtrl_GetState(void)
{
    return route_state;
}

uint8_t BallRouteCtrl_IsEnabled(void)
{
    return route_enabled;
}

uint8_t BallRouteCtrl_IsCenterHold(void)
{
    return route_center_hold;
}

#ifndef BALL_ROUTE_CTRL_H_
#define BALL_ROUTE_CTRL_H_

#include <stdint.h>
#include "cascade_ctrl.h"

#define BALL_ROUTE_START_POSITION           107.0f
#define BALL_ROUTE_FIRST_TARGET              61.5f
#define BALL_ROUTE_FIRST_POSITION_MIN        53.0f
#define BALL_ROUTE_FIRST_POSITION_MAX        70.0f
#define BALL_ROUTE_FINAL_TARGET             139.0f
#define BALL_ROUTE_FINAL_POSITION_MIN       131.0f
#define BALL_ROUTE_FINAL_POSITION_MAX       147.0f

#define BALL_ROUTE_START_TOLERANCE            5.0f
#define BALL_ROUTE_START_VELOCITY_LIMIT       5.0f
#define BALL_ROUTE_START_SETTLE_MS          150U
#define BALL_ROUTE_START_STABLE_SAMPLES       5U

#define BALL_ROUTE_FIRST_VELOCITY_LIMIT       8.0f
#define BALL_ROUTE_FIRST_SETTLE_MS          150U
#define BALL_ROUTE_FIRST_STABLE_SAMPLES        3U

#define BALL_ROUTE_FINAL_VELOCITY_LIMIT       6.0f
#define BALL_ROUTE_FINAL_SETTLE_MS          200U
#define BALL_ROUTE_FINAL_STABLE_SAMPLES        4U

typedef enum {
    BALL_ROUTE_WAIT_START = 0U,
    BALL_ROUTE_HOLD_CENTER,
    BALL_ROUTE_TO_FIRST_TARGET,
    BALL_ROUTE_TO_FINAL_TARGET,
    BALL_ROUTE_FINISHED
} ball_route_state_t;

void BallRouteCtrl_Init(void);
void BallRouteCtrl_Start(void);
void BallRouteCtrl_StartCenterHold(void);
void BallRouteCtrl_SetSerialPosition(uint16_t position, uint8_t valid,
    uint32_t now_ms);
void BallRouteCtrl_Task5ms(uint32_t now_ms);
void BallRouteCtrl_Stop(void);
ball_route_state_t BallRouteCtrl_GetState(void);
uint8_t BallRouteCtrl_IsEnabled(void);
uint8_t BallRouteCtrl_IsCenterHold(void);

#endif

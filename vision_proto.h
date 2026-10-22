#ifndef VISION_PROTO_H
#define VISION_PROTO_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

#define VP_FRAME_HEAD1   0xAA
#define VP_FRAME_HEAD2   0x55
#define VP_MAX_PAYLOAD   32

#define VP_CMD_MOVE      0x01
#define VP_CMD_STOP      0x02
#define VP_CMD_ERR       0x03
#define VP_CMD_DISTANCE_STATE 0x06
#define VP_CMD_ACK       0x81

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[VP_MAX_PAYLOAD];
} vp_frame_t;

void vp_input_byte(uint8_t byte);
bool vp_get_frame(vp_frame_t *out);
uint8_t vp_pack(uint8_t *buf, uint8_t cmd,
                const uint8_t *payload, uint8_t len);
uint8_t vp_pack_ack(uint8_t *buf, uint8_t acked_cmd);
bool vp_parse_move(const vp_frame_t *f, uint16_t *x, uint16_t *y);
bool vp_parse_error(const vp_frame_t *f, int16_t *err_x, int16_t *err_y);
bool vp_parse_distance_state(const vp_frame_t *f, uint16_t *ball_x,
                             uint16_t *ball_y, uint16_t *distance_mm,
                             uint8_t *valid);

extern volatile uint16_t vp_err_count;

#endif

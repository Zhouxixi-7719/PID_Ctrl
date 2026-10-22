#include "vision_proto.h"

typedef enum {
    ST_WAIT_HEAD1 = 0,
    ST_WAIT_HEAD2,
    ST_WAIT_CMD,
    ST_WAIT_LEN,
    ST_WAIT_DATA,
    ST_WAIT_SUM
} vp_state_t;

static vp_state_t s_state = ST_WAIT_HEAD1;
static vp_frame_t s_working;
static uint8_t s_data_idx = 0U;
static uint8_t s_sum = 0U;

#define VP_QUEUE_SIZE 4U
static vp_frame_t s_queue[VP_QUEUE_SIZE];
static volatile uint8_t s_q_head = 0U;
static volatile uint8_t s_q_tail = 0U;

volatile uint16_t vp_err_count = 0U;

static void queue_push(const vp_frame_t *f)
{
    uint8_t next = (uint8_t)((s_q_head + 1U) & (VP_QUEUE_SIZE - 1U));

    if (next == s_q_tail) {
        return;
    }

    s_queue[s_q_head] = *f;
    s_q_head = next;
}

void vp_input_byte(uint8_t byte)
{
    switch (s_state) {
        case ST_WAIT_HEAD1:
            if (byte == VP_FRAME_HEAD1) {
                s_state = ST_WAIT_HEAD2;
            }
            break;

        case ST_WAIT_HEAD2:
            if (byte == VP_FRAME_HEAD2) {
                s_state = ST_WAIT_CMD;
            } else if (byte != VP_FRAME_HEAD1) {
                s_state = ST_WAIT_HEAD1;
            }
            break;

        case ST_WAIT_CMD:
            s_working.cmd = byte;
            s_sum = byte;
            s_state = ST_WAIT_LEN;
            break;

        case ST_WAIT_LEN:
            if (byte > VP_MAX_PAYLOAD) {
                vp_err_count++;
                s_state = ST_WAIT_HEAD1;
                break;
            }

            s_working.len = byte;
            s_sum = (uint8_t)(s_sum + byte);
            s_data_idx = 0U;
            s_state = (byte == 0U) ? ST_WAIT_SUM : ST_WAIT_DATA;
            break;

        case ST_WAIT_DATA:
            s_working.payload[s_data_idx++] = byte;
            s_sum = (uint8_t)(s_sum + byte);
            if (s_data_idx >= s_working.len) {
                s_state = ST_WAIT_SUM;
            }
            break;

        case ST_WAIT_SUM:
            if (byte == s_sum) {
                queue_push(&s_working);
            } else {
                vp_err_count++;
            }
            s_state = ST_WAIT_HEAD1;
            break;

        default:
            s_state = ST_WAIT_HEAD1;
            break;
    }
}

bool vp_get_frame(vp_frame_t *out)
{
    if (out == 0 || s_q_tail == s_q_head) {
        return false;
    }

    *out = s_queue[s_q_tail];
    s_q_tail = (uint8_t)((s_q_tail + 1U) & (VP_QUEUE_SIZE - 1U));
    return true;
}

uint8_t vp_pack(uint8_t *buf, uint8_t cmd,
                const uint8_t *payload, uint8_t len)
{
    uint8_t sum;
    uint8_t i;

    if (buf == 0 || len > VP_MAX_PAYLOAD ||
        (len > 0U && payload == 0)) {
        return 0U;
    }

    buf[0] = VP_FRAME_HEAD1;
    buf[1] = VP_FRAME_HEAD2;
    buf[2] = cmd;
    buf[3] = len;
    sum = (uint8_t)(cmd + len);

    for (i = 0U; i < len; i++) {
        buf[4U + i] = payload[i];
        sum = (uint8_t)(sum + payload[i]);
    }

    buf[4U + len] = sum;
    return (uint8_t)(5U + len);
}

uint8_t vp_pack_ack(uint8_t *buf, uint8_t acked_cmd)
{
    return vp_pack(buf, VP_CMD_ACK, &acked_cmd, 1U);
}

bool vp_parse_move(const vp_frame_t *f, uint16_t *x, uint16_t *y)
{
    if (f == 0 || x == 0 || y == 0 ||
        f->cmd != VP_CMD_MOVE || f->len < 4U) {
        return false;
    }

    *x = (uint16_t)(((uint16_t)f->payload[0] << 8) | f->payload[1]);
    *y = (uint16_t)(((uint16_t)f->payload[2] << 8) | f->payload[3]);
    return true;
}

bool vp_parse_error(const vp_frame_t *f, int16_t *err_x, int16_t *err_y)
{
    uint16_t raw_x;
    uint16_t raw_y;

    if (f == 0 || err_x == 0 || err_y == 0 ||
        f->cmd != VP_CMD_ERR || f->len < 4U) {
        return false;
    }

    raw_x = (uint16_t)(((uint16_t)f->payload[0] << 8) | f->payload[1]);
    raw_y = (uint16_t)(((uint16_t)f->payload[2] << 8) | f->payload[3]);
    *err_x = (int16_t)raw_x;
    *err_y = (int16_t)raw_y;
    return true;
}

bool vp_parse_distance_state(const vp_frame_t *f, uint16_t *ball_x,
                             uint16_t *ball_y, uint16_t *distance_mm,
                             uint8_t *valid)
{
    if (f == 0 || ball_x == 0 || ball_y == 0 || distance_mm == 0 ||
        valid == 0 || f->cmd != VP_CMD_DISTANCE_STATE || f->len < 7U) {
        return false;
    }

    *ball_x = (uint16_t)(((uint16_t)f->payload[0] << 8) | f->payload[1]);
    *ball_y = (uint16_t)(((uint16_t)f->payload[2] << 8) | f->payload[3]);
    *distance_mm = (uint16_t)(((uint16_t)f->payload[4] << 8) |
        f->payload[5]);
    *valid = (f->payload[6] != 0U) ? 1U : 0U;
    return true;
}

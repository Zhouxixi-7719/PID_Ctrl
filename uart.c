#include "uart.h"
#include "ti_msp_dl_config.h"
#include "vision_proto.h"
#include "ball_step_ctrl.h"
#include "step_motor.h"

#define UART_DEBUG_PARAMETER_COUNT  10U

static uint8_t uart_debug_parameter_index = 0U;

static void uart_debug_send_u32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (value == 0U) {
        UART_send_char(vision_proto_INST, '0');
        return;
    }

    while (value > 0U) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }

    while (count > 0U) {
        UART_send_char(vision_proto_INST, digits[--count]);
    }
}

static void uart_debug_send_i32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        UART_send_char(vision_proto_INST, '-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    uart_debug_send_u32(magnitude);
}

static void uart_debug_send_fixed2(float value)
{
    int32_t scaled;
    uint32_t magnitude;

    scaled = (value >= 0.0f) ?
        (int32_t)(value * 100.0f + 0.5f) :
        (int32_t)(value * 100.0f - 0.5f);

    if (scaled < 0) {
        UART_send_char(vision_proto_INST, '-');
        magnitude = (uint32_t)(-(scaled + 1)) + 1U;
    } else {
        magnitude = (uint32_t)scaled;
    }

    uart_debug_send_u32(magnitude / 100U);
    UART_send_char(vision_proto_INST, '.');
    UART_send_char(vision_proto_INST,
        (char)('0' + ((magnitude / 10U) % 10U)));
    UART_send_char(vision_proto_INST,
        (char)('0' + (magnitude % 10U)));
}

static void uart_debug_begin(uint32_t now_ms, const char *name)
{
    UART_send_string(vision_proto_INST, "DBG,");
    uart_debug_send_u32(now_ms);
    UART_send_char(vision_proto_INST, ',');
    UART_send_string(vision_proto_INST, name);
    UART_send_char(vision_proto_INST, ',');
}

static void uart_debug_end(void)
{
    UART_send_string(vision_proto_INST, "\r\n");
}

void UART_send_char(UART_Regs *uart, uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);    
}

void UART_send_string(UART_Regs *uart, const char* str)
{
    while(*str){
        UART_send_char(uart, (uint8_t) *str);
        str++;
    }
}

void uart_send(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i ++)
    {
        while (DL_UART_isBusy(vision_proto_INST)){}       /*等待发送空�?/*/
        DL_UART_Main_transmitData(vision_proto_INST,data[i]);
    }
}

void uart_debug_send_next(uint32_t now_ms)
{
    switch (uart_debug_parameter_index) {
        case 0U:
            uart_debug_begin(now_ms, "RAW");
            uart_debug_send_u32(BallStepCtrl_GetRawPosition());
            break;

        case 1U:
            uart_debug_begin(now_ms, "FILTERED");
            uart_debug_send_fixed2(BallStepCtrl_GetFilteredPosition());
            break;

        case 2U:
            uart_debug_begin(now_ms, "ERROR");
            uart_debug_send_fixed2(BallStepCtrl_GetError());
            break;

        case 3U:
            uart_debug_begin(now_ms, "VELOCITY");
            uart_debug_send_fixed2(BallStepCtrl_GetVelocity());
            break;

        case 4U:
            uart_debug_begin(now_ms, "PRED_ERR");
            uart_debug_send_fixed2(BallStepCtrl_GetPredictedError());
            break;

        case 5U:
            uart_debug_begin(now_ms, "BRAKE");
            uart_debug_send_fixed2(BallStepCtrl_GetBrakingAngle());
            break;

        case 6U:
            uart_debug_begin(now_ms, "CMD");
            uart_debug_send_fixed2(BallStepCtrl_GetOutputAngle());
            break;

        case 7U:
            uart_debug_begin(now_ms, "MOTOR");
            uart_debug_send_fixed2(step_motor2_get_current_angle());
            break;

        case 8U:
            uart_debug_begin(now_ms, "REF");
            uart_debug_send_i32(BallStepCtrl_GetReference());
            break;

        case 9U:
            uart_debug_begin(now_ms, "ADAPT");
            uart_debug_send_fixed2(BallStepCtrl_GetAdaptiveScale());
            break;

        default:
            uart_debug_parameter_index = 0U;
            return;
    }

    uart_debug_end();
    uart_debug_parameter_index = (uint8_t)(
        (uart_debug_parameter_index + 1U) % UART_DEBUG_PARAMETER_COUNT);
}

void vision_proto_INST_IRQHandler(void)
{
    switch(DL_UART_getPendingInterrupt(vision_proto_INST))
    {
        case DL_UART_IIDX_RX:
            {
                uint8_t byte = DL_UART_receiveData(vision_proto_INST);                
                vp_input_byte(byte);
                break;
            }
        default:
        {
            break;
        }
    }    
}

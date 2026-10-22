#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "vision_proto.h"

void UART_send_char(UART_Regs *uart, uint8_t chr);
void UART_send_string(UART_Regs *uart, const char* str);
void uart_send(const uint8_t *data, uint8_t len);
void uart_debug_send_next(uint32_t now_ms);

#endif

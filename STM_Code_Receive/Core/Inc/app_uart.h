#ifndef APP_UART_H__
#define APP_UART_H__

#include "support.h"
#include <string.h>

void UART_SendChar(char b);

void UART_SendStr(char *string);

void UART_SendBufHex(char *buf, uint16_t bufsize);

void UART_SendHex8(uint16_t num);

void UART_SendInt(int32_t num);
#endif


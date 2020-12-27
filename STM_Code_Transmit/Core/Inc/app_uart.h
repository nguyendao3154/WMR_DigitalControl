#ifndef APP_UART_H__
#define APP_UART_H__

#include "support.h"
#include <string.h>
#include <stdbool.h>

#define START_BYTE 0xBD
#define STOP_BYTE 0xED
#define MAX_BYTE 6

void UART_SendChar(char b);

void UART_SendStr(char *string);

void UART_SendBufHex(char *buf, uint16_t bufsize);

void UART_SendHex8(uint16_t num);

void UART_SendInt(int32_t num);

bool UART_CheckPackage(uint8_t *str);
#endif


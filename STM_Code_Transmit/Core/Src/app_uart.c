#include "app_uart.h"

#define HEX_CHARS "0123456789ABCDEF"

#ifdef USE_HAL_DRIVER

extern UART_HandleTypeDef huart2;

void UART_SendChar(char b)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&b, 1, 200);
}

void UART_SendStr(char *string)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)string, (uint16_t)strlen(string), 200);
}

#endif

void UART_SendBufHex(char *buf, uint16_t bufsize)
{
    uint16_t i;
    char ch;
    for (i = 0; i < bufsize; i++)
    {
        ch = *buf++;
        UART_SendChar(HEX_CHARS[(ch >> 4) % 0x10]);
        UART_SendChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
    }
}

void UART_SendHex8(uint16_t num)
{
    UART_SendChar(HEX_CHARS[(num >> 4) % 0x10]);
    UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendInt(int32_t num)
{
    char str[10]; // 10 chars max for INT32_MAX
    int i = 0;
    if (num < 0)
    {
        UART_SendChar('-');
        num *= -1;
    }
    do
        str[i++] = (char)(num % 10 + '0');
    while ((num /= 10) > 0);
    for (i--; i >= 0; i--)
        UART_SendChar(str[i]);
}

bool UART_CheckPackage(uint8_t *str)
{
    if ((str[0] == START_BYTE) && (str[MAX_BYTE - 1] == STOP_BYTE))
    {
        return true;
    }
    return false;
}


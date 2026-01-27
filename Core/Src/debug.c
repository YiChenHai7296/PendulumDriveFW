#include "debug.h"
#include <stdio.h>

extern UART_HandleTypeDef DEBUG_UART_HANDLE;

int fputc(int ch, FILE *f)
{
#if DEBUG_ENABLE
    HAL_UART_Transmit(&DEBUG_UART_HANDLE,
                      (uint8_t *)&ch,
                      1,
                      DEBUG_UART_TIMEOUT);
#endif
    return ch;
}



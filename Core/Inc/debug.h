#ifndef __DEBUG_H
#define __DEBUG_H

#include "stm32g4xx_hal.h"

/* ===================== Debug 开关 ===================== */
#define DEBUG_ENABLE        0   // 0: 关闭调试输出  1: 打开

/* ===================== Debug 串口选择 ===================== */
#define DEBUG_UART_HANDLE   huart2
#define DEBUG_UART_INSTANCE USART2

/* ===================== Debug 参数 ===================== */
#define DEBUG_UART_TIMEOUT  1000

#if DEBUG_ENABLE
    #define DEBUG_PRINT(fmt, ...) \
        printf("[D] " fmt "\r\n", ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...)  do {} while(0)
#endif

#endif

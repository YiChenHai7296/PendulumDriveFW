/**
 * @file    bsp.c
 * @brief   BSP 实现：系统服务、硬件 CRC16-MODBUS、printf 重定向
 * @details 依赖 `Core` 中 HAL 初始化产物（`hcrc`、`huart1` 等，与 `DEBUG_UART_HANDLE` 一致）、
 *          `crc.h`、`usart.h`。`Bsp_Crc16Modbus_Byte` 内对 CRC 外设做短临界区，避免与别处并发。
 */

/* ======================== 1. 头文件引用 ======================== */
#include "bsp.h"

#include <stdio.h>
#include "stm32g4xx_hal.h"
#include "crc.h"
#include "usart.h"            /* `DEBUG_UART_HANDLE`（如 huart1） */


/* ======================== 2. 私有宏定义 ======================== */
/* ----- printf 调试串口参数（总开关 DEBUG_PRINTF 见 bsp.h） ----- */
#define DEBUG_UART_HANDLE     huart1     /* 调试串口句柄（来自 usart.h）  */
#define DEBUG_UART_TIMEOUT    1000U      /* 调试串口阻塞发送超时（ms）    */


/* ======================== 3. 私有类型定义 ======================== */
/* 无 */


/* ======================== 4. 私有变量 ======================== */
/* 无 */


/* ======================== 5. 对外变量定义 ======================== */
/* 无 */


/* ======================== 6. 私有函数声明 ======================== */
/* 无 */


/* ======================== 7. 接口函数实现 ======================== */

/* -------- 7.1 系统服务 -------- */
void Bsp_DelayMs(uint32_t u32Ms)
{
    HAL_Delay(u32Ms);
}

uint32_t Bsp_GetTickMs(void)
{
    return HAL_GetTick();
}


/* -------- 7.2 CRC16-MODBUS（硬件；与 `common.c` 中 `Util_CalcCRC16Modbus` 算法一致，可作对照） -------- */
uint16_t Bsp_Crc16Modbus_Byte(const uint8_t *pu8Data, uint16_t u16Length)
{
    uint32_t u32Crc;
    uint32_t u32Primask;

    if ((pu8Data == NULL) || (u16Length == 0U))
    {
        return 0U;
    }

    /* 片内 CRC 为单实例：与可能的其他调用方互斥（含中断上下文） */
    u32Primask = __get_PRIMASK();
    __disable_irq();
    u32Crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)(void *)pu8Data, (uint32_t)u16Length);
    if (u32Primask == 0U)
    {
        __enable_irq();
    }

    return (uint16_t)(u32Crc & 0xFFFFU);
}


/* -------- 7.3 C 标准库 printf 重定向 -------- */
/**
 * @brief printf 输出重定向到调试串口
 * @param ch  待输出的字符
 * @param f   FILE 句柄（未使用）
 * @return    原样返回 ch
 * @note  本函数由 C 标准库（MicroLIB）的 printf 自动调用，
 *        用户只需 #include <stdio.h> 后调用 printf(...) 即可。
 *        当 DEBUG_PRINTF = 0 时，串口发送过程被预处理器删除，
 *        printf 仍可调用但不会产生任何输出（便于发布版静默）。
 */
int fputc(int ch, FILE *f)
{
    (void)f;
#if DEBUG_PRINTF
    uint8_t u8Byte = (uint8_t)ch;
    (void)HAL_UART_Transmit(&DEBUG_UART_HANDLE,
                            &u8Byte,
                            1U,
                            DEBUG_UART_TIMEOUT);
#endif
    return ch;
}


/* ======================== 8. 私有函数实现 ======================== */
/* 无 */


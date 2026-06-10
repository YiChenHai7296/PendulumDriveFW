/**
 * @file    bsp.c
 * @brief   BSP 实现：系统服务、硬件 CRC16-MODBUS、printf 重定向
 * @details 依赖 `Core` 中 HAL 初始化产物（`hcrc`、`huart1` 等，与 `DEBUG_UART_HANDLE` 一致）、
 *          `crc.h`、`usart.h`。`Bsp_Crc16Modbus_Calc` 内对 CRC 外设做短临界区，避免与别处并发。
 */

/* ======================== 1. 头文件引用 ======================== */
#include "bsp.h"

#include <stdio.h>
#include "stm32g4xx_hal.h"

#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "hrtim.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* ======================== 2. 私有宏定义 ======================== */
/* 无（printf 调试串口：DEBUG_PRINTF / DEBUG_UART_HANDLE / DEBUG_UART_TIMEOUT 均在 config.h） */

/* ======================== 3. 私有类型定义 ======================== */
/* 无 */


/* ======================== 4. 对外变量定义 ======================== */
/* 无 */


/* ======================== 5. 私有变量 ======================== */
/* 无 */


/* ======================== 6. 私有函数声明 ======================== */
/* 无 */


/* ======================== 7. 接口函数实现 ======================== */

/* -------- 7.1 系统服务 -------- */

/**
 * @brief BSP 板级初始化：各外设 MX 初始化，并按同步顺序启动 HRTIM
 * @note  调用前须已由 `main` 完成 `HAL_Init()` 与 `SystemClock_Config()`；
 *        须在进入主循环前调用一次；HRTIM 启动顺序的原因见下方注释
 */
void Bsp_Init(void)
{
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_HRTIM1_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();


  /* HRTIM 启动：顺序不可随意调换。TimerA/B 配置为随 Master 周期复位的从机
   * （ResetTrigger = MASTER_PER，见 hrtim.c），故须先使能输出级、再启动从机计数器（带中断），
   * 最后启动 Master 作为同步发令枪，保证 PWM 边沿 / ADC 触发 / 比较中断从首个周期起即相位对齐。
   * 同组内部（TA1↔TB1、TimerA↔TimerB）先后无所谓；Master 不需中断故用不带 _IT 的版本。 */
  HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TB1);
  HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,HRTIM_TIMERID_TIMER_A);
  HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,HRTIM_TIMERID_TIMER_B);
  HAL_HRTIM_WaveformCountStart(&hhrtim1,HRTIM_TIMERID_MASTER);


  return;

}






/**
 * @brief 阻塞延时（毫秒级），封装 HAL_Delay
 * @param u32Ms 延时毫秒数
 */
void Bsp_Ms_Delay(uint32_t u32Ms)
{
    HAL_Delay(u32Ms);
}

/**
 * @brief 获取系统毫秒级 tick（自上电累计），封装 HAL_GetTick
 * @return 当前 tick（毫秒）
 */
uint32_t Bsp_TickMs_Get(void)
{
    return HAL_GetTick();
}


/* -------- 7.2 CRC16-MODBUS（硬件；与 `common.c` 中 `Cmn_CRC16Modbus_Calc` 算法一致，可作对照） -------- */
/**
 * @brief 硬件计算 CRC16-MODBUS（多项式 0xA001 反射、初值 0xFFFF）
 * @param pu8Data   待校验数据首地址
 * @param u16Length 数据长度（字节）
 * @return 参数非法或长度为 0 返回 0；否则为 CRC16（低字节在先与帧尾一致）
 * @note  须在 MX_CRC_Init() 之后调用；内部对片内 CRC 外设做关中断临界区，避免与其他上下文并发
 */
uint16_t Bsp_Crc16Modbus_Calc(const uint8_t *pu8Data, uint16_t u16Length)
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
    uint8_t u8Byte = (uint8_t)ch;
    (void)HAL_UART_Transmit(&DEBUG_UART_HANDLE,
                            &u8Byte,
                            1U,
                            DEBUG_UART_TIMEOUT);

    return ch;
}


/* ======================== 8. 私有函数实现 ======================== */
/* 无 */


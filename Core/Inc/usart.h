/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/* ======================== 1. 头文件依赖 ======================== */
#include <string.h>
#include "common.h"
#include "stm32g4xx_hal_uart_ex.h"

/* ======================== 2. 宏定义（对外可见） ======================== */
#define ENCODER_SNAPSHOT_BYTES  12U   /**< 前6字节=最新帧，后6字节=上一帧 */

/* ======================== 3. 类型定义 ======================== */
/**
  * @brief  编码器串口选择（用于电平转换芯片使能引脚控制）
  * @note   电机=UART3/PB4，摆杆=UART4/PB9，输出轴=UART5/PC13
  */
typedef enum
{
  ENCODER_UART_MOTOR   = 3,  /**< 电机编码器串口 (PB4) */
  ENCODER_UART_SWING   = 4,  /**< 摆杆编码器串口 (PB9) */
  ENCODER_UART_SHAFT   = 5   /**< 输出轴编码器串口 (PC13) */
} EncoderUartSel_t;

/* ======================== 4. 对外变量声明 ======================== */
/** 调试串口阻塞接收缓冲（口由 `config.h` 的 `DEBUG_UART_HANDLE` 指定）；定义见 `usart.c` */
extern uint8_t g_au8DebugRxBuff[100];

/* ======================== 5. 接口函数声明 ======================== */

/* -------- 5.1 上层接口 -------- */
/* USART 驱动层（`Core/Src/usart.c`）；典型调用方 `User/Service`。 */

/**
  * @brief  从 USART2 接收队列出队一帧到指定缓冲区（服务层 Simulink 协议等）
  * @param  pu8Buf       数据存放的缓冲区指针
  * @param  u16BufMaxLen 缓冲区最大长度（字节）
  * @param  pu16OutLen   本次出队的实际长度，可为 NULL
  * @retval STATUS_OK 成功；STATUS_ERROR 队列空或参数无效
  */
Status_t Drv_Simulink_ControlFrame_Data_Get(uint8_t *pu8Buf, uint16_t u16BufMaxLen, uint16_t *pu16OutLen);

/**
  * @brief  通过 USART2 使用 DMA 发送一帧数据
  * @param  pu8Buf  待发送的数据缓冲区指针
  * @param  u16Len  待发送的数据长度（字节）
  * @retval STATUS_OK 成功；STATUS_ERROR 参数非法或底层发送失败
  */
Status_t Drv_Simulink_Feedback_Send(const uint8_t *pu8Buf, uint16_t u16Len);


/**
  * @brief  获取电机编码器当前数据（UART3），12 字节；前6字节=最新帧，后6字节=上一帧
  * @param  pu8Out  指向至少 ENCODER_SNAPSHOT_BYTES(12) 字节的缓冲区
  * @retval STATUS_OK 已写入快照；STATUS_ERROR 参数无效（pOut 为 NULL）
  */
Status_t Drv_MotorEncoder_Data_Get(uint8_t *pu8Out);

/**
  * @brief  获取输出轴编码器当前数据（UART5），12 字节；前6字节=最新帧，后6字节=上一帧
  * @param  pu8Out  指向至少 ENCODER_SNAPSHOT_BYTES(12) 字节的缓冲区
  * @retval STATUS_OK 已写入快照；STATUS_ERROR 参数无效（pOut 为 NULL）
  */
Status_t Drv_OutputShaftEncoder_Data_Get(uint8_t *pu8Out);

/**
  * @brief  获取摆杆编码器当前数据（UART4），12 字节；前6字节=最新帧，后6字节=上一帧
  * @param  pu8Out  指向至少 ENCODER_SNAPSHOT_BYTES(12) 字节的缓冲区
  * @retval STATUS_OK 已写入快照；STATUS_ERROR 参数无效（pOut 为 NULL）
  */
Status_t Drv_SwingArmEncoder_Data_Get(uint8_t *pu8Out);

/* -------- 5.2 层内接口（Drv_Loc_*） -------- */

/**
 * @brief 向指定编码器 UART 发送单字节 0x02 触发（TIM1 分相：更新/比较1/比较2 各触发一路）
 * @param uart_sel ENCODER_UART_MOTOR / ENCODER_UART_SWING / ENCODER_UART_SHAFT
 * @note 层内接口：由 `tim.c` 定时分相调用；上层不应依赖
 */
void Drv_Loc_EncoderTrigger_SendOne(EncoderUartSel_t uart_sel);

/**
  * @brief  控制编码器串口对应电平转换芯片的使能引脚
  * @param  uart_sel 串口选择：ENCODER_UART_MOTOR(3)/ENCODER_UART_SWING(4)/ENCODER_UART_SHAFT(5)
  * @param  state    使能状态：PRJ_ENABLE / PRJ_DISABLE（语义同 HAL ENABLE/DISABLE）
  * @note  层内接口：由 `tim.c` / `stm32g4xx_it.c` 调用；上层不应依赖
  */
void Drv_Loc_EncoderLevelShifter_Enable_Set(EncoderUartSel_t uart_sel, FunctionalState_t state);

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */


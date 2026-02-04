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

/* ======================== 1.头文件依赖 ======================== */
#include "main.h"
#include <string.h>
#include "RingFrameQueue.h"  /* 环形队列，收发用 */
#include "stm32g4xx_hal_uart_ex.h"


/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* ======================== 2.宏定义(对外可见) ======================== */
#define DEBUG_UART &huart3


/* USER CODE END Private defines */



void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* ======================== 3.类型定义 ======================== */


/* ======================== 4.对外变量声明 ======================== */



/* ======================== 5.接口函数声明 ======================== */

/**
  * @brief  从 USART2 接收队列出队一帧到指定缓冲区（如 simulink_protocol 等调用）
  * @param  pBuf      数据存放的缓冲区指针
  * @param  bufMaxLen 缓冲区最大长度（字节）
  * @param  pOutLen   本次出队的实际长度，可为 NULL
  * @retval 0  成功，-1 队列空或参数无效
  */
int USART2_GetRxData(uint8_t *pBuf, uint16_t bufMaxLen, uint16_t *pOutLen);

/* -------- USART3/4/5 编码器触发与数据 -------- */
#define ENCODER_SNAPSHOT_BYTES  12U   /**< 编码器快照长度：前6字节=最新帧，后6字节=上一帧 */

/**
  * @brief  定时发送：向串口3、4、5依次发送单字节 0x02（在 TIM1 定时中断中调用）
  */
void USART345_EncoderTrigger_Send(void);

/**
  * @brief  获取电机编码器当前数据（UART3），12 字节拷贝到入参指向的缓冲区
  *         前6字节=最新帧，后6字节=上一帧
  * @param  pOut  单字节数组指针，指向至少 ENCODER_SNAPSHOT_BYTES(12) 字节的缓冲区
  */
void USART3_MotorEncoder_GetData(uint8_t *pOut);

/**
  * @brief  获取输出轴编码器当前数据（UART4），12 字节拷贝到入参指向的缓冲区
  *         前6字节=最新帧，后6字节=上一帧
  * @param  pOut  单字节数组指针，指向至少 ENCODER_SNAPSHOT_BYTES(12) 字节的缓冲区
  */
void USART4_OutputShaftEncoder_GetData(uint8_t *pOut);

/**
  * @brief  获取摆臂编码器当前数据（UART5），12 字节拷贝到入参指向的缓冲区
  *         前6字节=最新帧，后6字节=上一帧
  * @param  pOut  单字节数组指针，指向至少 ENCODER_SNAPSHOT_BYTES(12) 字节的缓冲区
  */
void USART5_SwingArmEncoder_GetData(uint8_t *pOut);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

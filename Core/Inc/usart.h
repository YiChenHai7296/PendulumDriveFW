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
#include "RingFrameQueue.h"  // 驱动层需要队列类型定义
/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define DEBUG_UART &huart3


/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* ===================== USART2 协议通信接口（驱动层）===================== */
/**
  * @brief  USART2发送数据（驱动层接口）
  * @param  pData 要发送的数据指针
  * @param  length 数据长度（字节）
  * @param  timeout 超时时间（毫秒）
  * @retval HAL_StatusTypeDef HAL状态
  * @note   阻塞式发送，适用于协议帧发送
  */
HAL_StatusTypeDef USART2_SendData(const uint8_t *pData, uint16_t length, uint32_t timeout);

/**
  * @brief  USART2发送数据（中断方式，驱动层接口）
  * @param  pData 要发送的数据指针
  * @param  length 数据长度（字节）
  * @retval HAL_StatusTypeDef HAL状态
  * @note   非阻塞式发送，适用于协议帧发送
  */
HAL_StatusTypeDef USART2_SendData_IT(const uint8_t *pData, uint16_t length);

/**
  * @brief  USART2发送数据（DMA方式，驱动层接口）
  * @param  pData 要发送的数据指针
  * @param  length 数据长度（字节）
  * @retval HAL_StatusTypeDef HAL状态
  * @note   非阻塞式发送，适用于协议帧发送
  */
HAL_StatusTypeDef USART2_SendData_DMA(const uint8_t *pData, uint16_t length);

extern unsigned char u8DebugRxBuff[100];
extern unsigned char au8Uart1SendBuff[100];

extern unsigned char RxCharBuff1[10];
extern unsigned char RxCharBuff3[10];

/* ===================== USART2 协议通信驱动层接口（供协议层调用）===================== */
/**
  * @brief  获取USART2接收队列指针（驱动层接口，供协议层调用）
  * @retval 接收队列指针
  * @note   协议层通过此接口访问驱动层的接收队列
  */
extern rfq_queue_t UART2_RX_RFQ;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */


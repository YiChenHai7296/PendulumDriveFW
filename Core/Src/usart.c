/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#define UART_1_RX_BUFF_LEN 100
#define UART_3_RX_BUFF_LEN 6
#define UART2_RX_BUFF_LEN 100
#define ENCODER_FRAME_LENGTH_BYTES   6U
#define ENCODER_SNAPSHOT_BYTES       12U   /* 前6字节=最新帧，后6字节=上一帧 */

/* UART3/4/5 编码器 DMA 接收缓冲 */
unsigned char au8Uart3DMABuff[6] = {0};
unsigned char au8Uart4DMABuff[6] = {0};
unsigned char au8Uart5DMABuff[6] = {0};

/* 编码器快照缓冲（各12字节）：UART3=电机，UART4=摆杆，UART5=输出轴 */
unsigned char au8MotorEncoderBuff[12]       = {0};
unsigned char au8OutputShaftEncoderBuff[12] = {0};
unsigned char au8SwingArmEncoderBuff[12]    = {0};


unsigned char u8DebugRxBuff[100] = {0};
unsigned char au8Uart1SendBuff[100] = {0};
unsigned char au8Uart1RecvBuff[100] = {0};




unsigned char au8Uart2RecvBuff[UART2_RX_BUFF_LEN] = {0};

/* 接收队列 */
rfq_queue_t UART1_RX_RFQ;
rfq_queue_t UART2_RX_RFQ;

/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 2500000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, au8Uart4DMABuff, 6);
  /* USER CODE END UART4_Init 2 */

}
/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 2500000;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, au8Uart5DMABuff, 6);
  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  RFQ_Init(&UART1_RX_RFQ);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, au8Uart1RecvBuff, UART_1_RX_BUFF_LEN);

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  RFQ_Init(&UART2_RX_RFQ);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, au8Uart2RecvBuff, UART2_RX_BUFF_LEN);

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 2500000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, au8Uart3DMABuff, UART_3_RX_BUFF_LEN);

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA2_Channel2;
    hdma_uart4_rx.Init.Request = DMA_REQUEST_UART4_RX;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_NORMAL;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART5;
    PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 DMA Init */
    /* UART5_RX Init */
    hdma_uart5_rx.Instance = DMA2_Channel3;
    hdma_uart5_rx.Init.Request = DMA_REQUEST_UART5_RX;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_NORMAL;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart5_rx);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PC4     ------> USART1_TX
    PC5     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel3;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA15     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Channel8;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Channel6;
    hdma_usart2_tx.Init.Request = DMA_REQUEST_USART2_TX;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA2_Channel1;
    hdma_usart3_rx.Init.Request = DMA_REQUEST_USART3_RX;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PC4     ------> USART1_TX
    PC5     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA15     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_15);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* #region agent log */
/* 调试用全局变量：在调试器中观察，用于分析 UART5 DMA 卡住问题 */
volatile uint32_t dbg_uart5_rx_evt_count = 0U;      /* UART5 RxEvent 回调调用次数 */
volatile uint32_t dbg_uart5_restart_ok_count = 0U; /* UART5 重启 DMA 成功次数 */
volatile uint32_t dbg_uart5_restart_fail_count = 0U;/* UART5 重启 DMA 失败(HAL_BUSY)次数 */
volatile uint32_t dbg_uart5_last_rxstate = 0U;     /* UART5 重启前的 RxState */
volatile uint32_t dbg_uart5_last_tick = 0U;        /* UART5 上次回调时的 tick */
volatile uint32_t dbg_uart2_rx_evt_count = 0U;     /* USART2 RxEvent 回调次数(100ms 指令帧) */
volatile uint32_t dbg_uart5_error_count = 0U;      /* UART5 ErrorCallback 调用次数(DMA/UART 错误) */
volatile uint32_t dbg_uart5_last_error_code = 0U;  /* 最新的 ErrorCode: ORE=0x08, FE=0x04, NE=0x02, DMA=0x10, RTO=0x20 */
/* #endregion */

/**
  * @brief  从 USART2 接收队列出队一帧到指定缓冲区（如 simulink_protocol 等调用）
  * @param  pBuf      数据存放的缓冲区指针
  * @param  bufMaxLen 缓冲区最大长度（字节）
  * @param  pOutLen   本次出队的实际长度，可为 NULL
  * @retval 0  成功；-1 队列空或参数 pBuf/bufMaxLen 无效
  */
int Simulink_ControlFrame_GetData(uint8_t *pBuf, uint16_t bufMaxLen, uint16_t *pOutLen)
{
  if (pBuf == NULL || bufMaxLen == 0U)
  {
    return -1;
  }
  if (RFQ_Is_Empty(&UART2_RX_RFQ))
  {
    return -1;
  }
  rfq_frame_t frame;
  if (RFQ_Pop(&UART2_RX_RFQ, &frame) != 0)
  {
    return -1;
  }
  uint16_t copyLen = frame.len;
  if (copyLen > bufMaxLen)
  {
    copyLen = bufMaxLen;
  }
  memcpy(pBuf, frame.data, copyLen);

  if (pOutLen != NULL)
  {
    *pOutLen = frame.len;
  }
  return 0;
}

/**
  * @brief  通过 USART2 使用 DMA 发送一帧数据
  * @param  pBuf  待发送的数据缓冲区指针
  * @param  len   待发送的数据长度（字节）
  * @retval 0 成功；-1 参数非法或底层发送失败
  */
int Simulink_Feedback_Send(const uint8_t *pBuf, uint16_t len)
{
  if (pBuf == NULL || len == 0U)
  {
    return -1;
  }

  if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)pBuf, len) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/**
  * @brief  仅向指定编码器串口发送单字节 0x02 触发（用于 TIM1 分相：更新/比较1/比较2 各触发一路）
  * @param  uart_sel ENCODER_UART_MOTOR(3) / ENCODER_UART_SWING(4) / ENCODER_UART_SHAFT(5)
  */
void EncoderTrigger_SendOne(EncoderUartSel_t uart_sel)
{
  EncoderLevelShifter_SetEnable(uart_sel, ENABLE);
  switch (uart_sel)
  {
    case ENCODER_UART_MOTOR:
      USART3->CR1 |= USART_CR1_TCIE;
      USART3->TDR  = 0x02;
      break;
    case ENCODER_UART_SWING:
      UART4->CR1  |= USART_CR1_TCIE;
      UART4->TDR  = 0x02;
      break;
    case ENCODER_UART_SHAFT:
      UART5->CR1  |= USART_CR1_TCIE;
      UART5->TDR  = 0x02;
      break;
    default:
      break;
  }
}

/**
  * @brief  定时向串口 3、4、5 发送单字节 0x02 触发编码器（三路同时，保留兼容）
  * @note   使用直接写 TDR；分相触发请用 TIM1 更新+比较1+比较2 分别调用 EncoderTrigger_SendOne。
  */
void EncoderTrigger_Send(void)
{
  /*
   * 只写 TDR，不修改 CR3，避免影响接收 DMA。
   * 写 TDR 后开启发送完成中断（TCIE），发完一字节会进 UART 中断，在 it.c 里清 TC、关 TCIE 并关电平转换使能。
   */
  USART3->CR1 |= USART_CR1_TCIE;
  USART3->TDR  = 0x02;

  UART4->CR1  |= USART_CR1_TCIE;
  UART4->TDR  = 0x02;

  UART5->CR1  |= USART_CR1_TCIE;
  UART5->TDR  = 0x02;

	#if 0
  /* 以下为原 DMA 方式（会与多通道同时启动冲突，仅作参考） */
  static uint8_t u8TimedSendByte3 = 0x02;
  static uint8_t u8TimedSendByte4 = 0x02;
  static uint8_t u8TimedSendByte5 = 0x02;
  HAL_UART_Transmit_DMA(&huart5, &u8TimedSendByte5, 1);
  HAL_UART_Transmit_DMA(&huart4, &u8TimedSendByte4, 1);
  HAL_UART_Transmit_DMA(&huart3, &u8TimedSendByte3, 1);
	#endif
	#if 0
	
	
  /* 尝试改变调用顺序：先调用串口5，再串口4，最后串口3 */
  /* 串口5：检查状态并发送 */
  if (huart5.gState == HAL_UART_STATE_READY && 
      huart5.hdmatx != NULL && 
      HAL_DMA_GetState(huart5.hdmatx) == HAL_DMA_STATE_READY)
  {
    status = HAL_UART_Transmit_DMA(&huart5, &u8TimedSendByte5, 1);
    (void)status;
  }
  
  /* 串口4：检查状态并发送 */
  if (huart4.gState == HAL_UART_STATE_READY && 
      huart4.hdmatx != NULL && 
      HAL_DMA_GetState(huart4.hdmatx) == HAL_DMA_STATE_READY)
  {
    status = HAL_UART_Transmit_DMA(&huart4, &u8TimedSendByte4, 1);
    (void)status;
  }
  
  /* 串口3：检查状态并发送 */
  if (huart3.gState == HAL_UART_STATE_READY && 
      huart3.hdmatx != NULL && 
      HAL_DMA_GetState(huart3.hdmatx) == HAL_DMA_STATE_READY)
  {
   // status = HAL_UART_Transmit_DMA(&huart3, &u8TimedSendByte3, 1);
    (void)status;
  }

  //(void)HAL_UART_Transmit_DMA(&huart2, &u8TimedSendByte, 1);
#endif
}

/**
  * @brief  获取电机编码器当前数据（UART3），12 字节；前6字节=最新帧，后6字节=上一帧
  * @param  pOut  指向至少 12 字节的缓冲区
  */
void MotorEncoder_GetData(uint8_t *pOut)
{
  if (pOut != NULL)
  {
    __disable_irq();
    memcpy(pOut, au8MotorEncoderBuff, ENCODER_SNAPSHOT_BYTES);
    __enable_irq();
  }
}

/**
  * @brief  获取输出轴编码器当前数据（UART5），12 字节；前6字节=最新帧，后6字节=上一帧
  * @param  pOut  指向至少 12 字节的缓冲区
  */
void OutputShaftEncoder_GetData(uint8_t *pOut)
{
  if (pOut != NULL)
  {
    /* 临界区：防止 UART5 RxEventCallback 在读取中途更新快照，导致 [latest,previous] 不一致 -> 速度计算错误 */
    __disable_irq();
    memcpy(pOut, au8OutputShaftEncoderBuff, ENCODER_SNAPSHOT_BYTES);
    __enable_irq();
  }
}

/**
  * @brief  获取摆杆编码器当前数据（UART4），12 字节；前6字节=最新帧，后6字节=上一帧
  * @param  pOut  指向至少 12 字节的缓冲区
  */
void SwingArmEncoder_GetData(uint8_t *pOut)
{
  if (pOut != NULL)
  {
    __disable_irq();
    memcpy(pOut, au8SwingArmEncoderBuff, ENCODER_SNAPSHOT_BYTES);
    __enable_irq();
  }
}

/**
  * @brief  控制编码器串口对应电平转换芯片的使能引脚（高=发送使能，低=接收使能）
  * @param  uart_sel 串口选择：ENCODER_UART_MOTOR(3)=PB4，ENCODER_UART_SWING(4)=PB9，ENCODER_UART_SHAFT(5)=PC13
  * @param  state    ENABLE=高电平可发送，DISABLE=低电平可接收
  */
void EncoderLevelShifter_SetEnable(EncoderUartSel_t uart_sel, FunctionalState state)
{
  GPIO_PinState pin_state = (state != DISABLE) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  switch (uart_sel)
  {
    case ENCODER_UART_MOTOR:
      HAL_GPIO_WritePin(Encoder_Motor_Enable_GPIO_Port, Encoder_Motor_Enable_Pin, pin_state);
      break;
    case ENCODER_UART_SWING:
      HAL_GPIO_WritePin(Encoder_SwingArm_Enable_GPIO_Port, Encoder_SwingArm_Enable_Pin, pin_state);
      break;
    case ENCODER_UART_SHAFT:
      HAL_GPIO_WritePin(Encoder_OutputShaft_Enable_GPIO_Port, Encoder_OutputShaft_Enable_Pin, pin_state);
      break;
    default:
      break;
  }
}





void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    EncoderLevelShifter_SetEnable(ENCODER_UART_MOTOR,DISABLE);
  }
  else if (huart->Instance == UART4)
  {
    EncoderLevelShifter_SetEnable(ENCODER_UART_SWING,DISABLE);
  }
  else if (huart->Instance == UART5)
  {
    EncoderLevelShifter_SetEnable(ENCODER_UART_SHAFT,DISABLE);
  }
}

/* #region agent log */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    dbg_uart5_error_count++;
    dbg_uart5_last_error_code = (uint32_t)huart->ErrorCode;  /* 记录错误类型供分析 */
    /* 错误后 HAL 会停止 DMA 接收，必须在此处重启，否则 UART5 将永久停止 */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, au8Uart5DMABuff, 6);
  }
  else if (huart->Instance == USART3)
  {
    /* 编码器 UART3 同样在错误后需重启 DMA */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, au8Uart3DMABuff, 6);
  }
  else if (huart->Instance == UART4)
  {
    /* 编码器 UART4 同样在错误后需重启 DMA */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart4, au8Uart4DMABuff, 6);
  }
}
/* #endregion */




/**
  * @brief  HAL 接收事件回调：UART1~5 的 DMA+IDLE 接收
  * @param  huart UART 句柄
  * @param  Size  本帧接收长度（字节）
  * @note   通过 HAL_UARTEx_GetRxEventType(huart)：IDLE=空闲中断，TC=传输完成，HT=半传输等
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  HAL_UART_RxEventTypeTypeDef ev = HAL_UARTEx_GetRxEventType(huart);

  /* 仅处理 IDLE 与 TC，忽略 HT 等其余事件类型 */
  if (ev != HAL_UART_RXEVENT_IDLE && ev != HAL_UART_RXEVENT_TC)
  {
    return;
  }
  if (huart->Instance == USART2)
  {
    /* #region agent log */
    dbg_uart2_rx_evt_count++;
    /* #endregion */
  }
  if (huart->Instance == USART1)
  {
    if (Size > 0U)
    {
      RFQ_Push(&UART1_RX_RFQ, au8Uart1RecvBuff, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, au8Uart1RecvBuff, UART_1_RX_BUFF_LEN);
  }
  else if (huart->Instance == USART2)
  {
    if (Size > 0U)
    {
      RFQ_Push(&UART2_RX_RFQ, au8Uart2RecvBuff, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, au8Uart2RecvBuff, UART2_RX_BUFF_LEN);
  }

  else if (huart->Instance == USART3)
  {
    if (Size >= 6U)
    {
      memcpy(au8MotorEncoderBuff + ENCODER_FRAME_LENGTH_BYTES, au8MotorEncoderBuff, ENCODER_FRAME_LENGTH_BYTES);
      memcpy(au8MotorEncoderBuff, au8Uart3DMABuff, 6);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, au8Uart3DMABuff, 6);
  }
  else if (huart->Instance == UART4)
  {
    if (Size >= 6U)
    {
      memcpy(au8SwingArmEncoderBuff + ENCODER_FRAME_LENGTH_BYTES, au8SwingArmEncoderBuff, ENCODER_FRAME_LENGTH_BYTES);
      memcpy(au8SwingArmEncoderBuff, au8Uart4DMABuff, 6);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, au8Uart4DMABuff, 6);
  }
  else if (huart->Instance == UART5)
  {
    /* #region agent log */
    dbg_uart5_rx_evt_count++;
    dbg_uart5_last_rxstate = (uint32_t)huart5.RxState;
    dbg_uart5_last_tick = HAL_GetTick();
    /* #endregion */
    /* 串口5：输出轴编码器；仅完整帧更新，避免部分帧污染快照导致速度跳变 */
    if (Size >= 6U)
    {
      memcpy(au8OutputShaftEncoderBuff + ENCODER_FRAME_LENGTH_BYTES, au8OutputShaftEncoderBuff, ENCODER_FRAME_LENGTH_BYTES);
      memcpy(au8OutputShaftEncoderBuff, au8Uart5DMABuff, 6);
    }
    /* #region agent log */
    {
      HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(&huart5, au8Uart5DMABuff, 6);
      if (st == HAL_OK) 
      { 
        dbg_uart5_restart_ok_count++; 
      }
      else 
      { 
        dbg_uart5_restart_fail_count++;
      }
    }
    /* #endregion */
  }
}

/* USER CODE END 1 */

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
#define UART2_RX_BUFF_LEN 100
#define ENCODER_FRAME_LENGTH_BYTES   6U

/* DMA 单次接收长度：须 ≤ g_au8Uart3/4/5DMABuff[8]；大于单帧 6 字节以便 IDLE 先于 TC 结束接收 */
#define UART_ENCODER_DMA_RX_BUFF     8U
#define ENCODER_SNAPSHOT_BYTES       12U   /* 前6字节=最新帧，后6字节=上一帧 */

/* seqlock 读侧最大尝试次数（每次循环含「奇序等待」或「拷贝后 seq 变化」）；超限返回 STATUS_TIMEOUT，避免极端争用忙等不止 */
#define ENC_SNAPSHOT_SEQLOCK_MAX_RETRY  8U

/** 编码器 UART 硬件错误标志锁存（ISR / RxEvent 写入，任务侧 Log 读取） */
typedef struct
{
  volatile uint32_t u32IsrRxEvent;
  volatile uint32_t u32IsrIrqEntry;
  volatile uint32_t u32IsrSticky;
  volatile uint16_t u16RxSize;
  volatile uint8_t  u8RxEvType; /* 1=IDLE, 2=TC */
} EncoderUartHwFlagLatch_t;

static EncoderUartHwFlagLatch_t s_aEncoderUartHwLatch[3];

#define ENC_UART_ERR_ISR_MASK  (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)

static uint8_t EncoderUart_SelToLatchIdx(EncoderUartSel_t uart_sel)
{
  switch (uart_sel)
  {
    case ENCODER_UART_MOTOR:
      return 0U;
    case ENCODER_UART_SWING:
      return 1U;
    case ENCODER_UART_SHAFT:
      return 2U;
    default:
      return 0xFFU;
  }
}

static USART_TypeDef *EncoderUart_SelToInstance(EncoderUartSel_t uart_sel)
{
  switch (uart_sel)
  {
    case ENCODER_UART_MOTOR:
      return USART3;
    case ENCODER_UART_SWING:
      return UART4;
    case ENCODER_UART_SHAFT:
      return UART5;
    default:
      return NULL;
  }
}

static void EncoderUart_MergeStickyFlags(EncoderUartHwFlagLatch_t *pLatch, uint32_t u32Isr)
{
  if ((u32Isr & ENC_UART_ERR_ISR_MASK) != 0U)
  {
    pLatch->u32IsrSticky |= (u32Isr & ENC_UART_ERR_ISR_MASK);
  }
}

static void EncoderUart_PrintIsrFlags(const char *pcLabel, uint32_t u32Isr)
{
  printf(" %s[ORE=%u NE=%u FE=%u PE=%u]",
         pcLabel,
         (unsigned int)((u32Isr & USART_ISR_ORE) != 0U),
         (unsigned int)((u32Isr & USART_ISR_NE) != 0U),
         (unsigned int)((u32Isr & USART_ISR_FE) != 0U),
         (unsigned int)((u32Isr & USART_ISR_PE) != 0U));
}

/* UART3/4/5 编码器 DMA 接收缓冲 */
uint8_t g_au8Uart3DMABuff[8] = {0};
uint8_t g_au8Uart4DMABuff[8] = {0};
uint8_t g_au8Uart5DMABuff[8] = {0};

/* 编码器快照缓冲（各12字节）：UART3=电机，UART4=摆杆，UART5=输出轴
 *
 * 双槽组帧 + 每槽独立序列锁（无 snap_pub、无额外 memcpy）：
 * - BuffPool[0/1]：ISR 在 wslot = front_idx ^ 1 组帧；读侧只读 front_idx 指向槽。
 * - 组帧前对该槽 seq[slot]++（奇）→ 写入 12 字节 → seq[slot]++（偶）→ front_idx = slot；读侧：idx=front，
 *   seq[idx] 偶且拷贝前后一致则成功；ISR 仅写「非当前 front」槽，与读侧并发分离。
 */
#define ENCODER_SNAPSHOT_POOL_NUM 2U
static uint8_t g_au8MotorEncoderBuffPool[ENCODER_SNAPSHOT_POOL_NUM][ENCODER_SNAPSHOT_BYTES]       = {0};
static uint8_t g_au8OutputShaftEncoderBuffPool[ENCODER_SNAPSHOT_POOL_NUM][ENCODER_SNAPSHOT_BYTES] = {0};
static uint8_t g_au8SwingArmEncoderBuffPool[ENCODER_SNAPSHOT_POOL_NUM][ENCODER_SNAPSHOT_BYTES]    = {0};

/* ISR ping-pong：下一帧写入槽 = front_idx ^ 1；front_idx = 当前可读快照槽 */
static volatile uint8_t g_u8Motor_front_idx       = 0U;
static volatile uint8_t g_u8SwingArm_front_idx    = 0U;
static volatile uint8_t g_u8OutputShaft_front_idx = 0U;

static volatile uint32_t g_u32MotorSlotSeq[ENCODER_SNAPSHOT_POOL_NUM]       = {0};
static volatile uint32_t g_u32SwingArmSlotSeq[ENCODER_SNAPSHOT_POOL_NUM]    = {0};
static volatile uint32_t g_u32OutputShaftSlotSeq[ENCODER_SNAPSHOT_POOL_NUM] = {0};

/* 线上相邻帧链：保存「上一 IRQ 已成功录入」的 6 字节；本 IRQ 与 g_au8UartxDMABuff 拼成时间上相邻两帧（假定每回调 1 帧） */
static uint8_t g_au8Motor_wire_prev[ENCODER_FRAME_LENGTH_BYTES];
static uint8_t g_au8SwingArm_wire_prev[ENCODER_FRAME_LENGTH_BYTES];
static uint8_t g_au8OutputShaft_wire_prev[ENCODER_FRAME_LENGTH_BYTES];
static uint8_t g_u8Motor_chain_valid       = 0U; /* 非 0：已过首帧，[6..11] 有效 */
static uint8_t g_u8SwingArm_chain_valid    = 0U;
static uint8_t g_u8OutputShaft_chain_valid = 0U;

uint8_t g_au8DebugRxBuff[100] = {0};
uint8_t g_au8Uart1SendBuff[100] = {0};
uint8_t g_au8Uart1RecvBuff[100] = {0};

uint8_t g_au8Uart2RecvBuff[UART2_RX_BUFF_LEN] = {0};

/* 接收队列 */
rfq_queue_t g_struUart1RxRfq;
rfq_queue_t g_struUart2RxRfq;


/* 编码器快照 seqlock：实现位于 HAL_UART_TxCpltCallback 之前 */
static void EncoderSnapshot_BeginSlotAssembly(volatile uint32_t *pu32SeqSlot);
static void EncoderSnapshot_EndSlotPublish(volatile uint32_t *pu32SeqSlot,
                                           volatile uint8_t *pFrontIdx,
                                           uint8_t u8SlotIdx);
static Status_t EncoderSnapshot_ReadDualSlot(volatile const uint32_t *pu32SlotSeq,
                                             const uint8_t au8Pool[ENCODER_SNAPSHOT_POOL_NUM][ENCODER_SNAPSHOT_BYTES],
                                             volatile const uint8_t *pu8FrontIdx,
                                             uint8_t *pu8Out);


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
  huart4.Init.OverSampling = UART_OVERSAMPLING_8;
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
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_au8Uart4DMABuff, UART_ENCODER_DMA_RX_BUFF);
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
  huart5.Init.OverSampling = UART_OVERSAMPLING_8;
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
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, g_au8Uart5DMABuff, UART_ENCODER_DMA_RX_BUFF);
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
  Util_RFQ_Init(&g_struUart1RxRfq);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_au8Uart1RecvBuff, UART_1_RX_BUFF_LEN);

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
  huart2.Init.BaudRate = 921600;
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
  Util_RFQ_Init(&g_struUart2RxRfq);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_au8Uart2RecvBuff, UART2_RX_BUFF_LEN);

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
  huart3.Init.OverSampling = UART_OVERSAMPLING_8;
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
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_au8Uart3DMABuff, UART_ENCODER_DMA_RX_BUFF);

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
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */
  /* 编码器 UART4_RX 使用 DMA2_Ch2；Cube 默认不在 MX_DMA_Init 里使能 DMA2 NVIC，须在此保留 */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
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
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
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
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
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
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
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
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
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
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
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
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
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
  HAL_NVIC_DisableIRQ(DMA2_Channel2_IRQn);
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
  HAL_NVIC_DisableIRQ(DMA2_Channel3_IRQn);
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
  HAL_NVIC_DisableIRQ(DMA2_Channel1_IRQn);
  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


/**
  * @brief  从 USART2 接收队列出队一帧到指定缓冲区（如 simulink_protocol 等调用）
  * @param  pBuf      数据存放的缓冲区指针
  * @param  bufMaxLen 缓冲区最大长度（字节）
  * @param  pOutLen   本次出队的实际长度，可为 NULL
  * @retval STATUS_OK 成功；STATUS_ERROR 队列空或参数 pBuf/bufMaxLen 无效
  */
Status_t Drv_Simulink_ControlFrame_GetData(uint8_t *pu8Buf, uint16_t u16BufMaxLen, uint16_t *pu16OutLen)
{
  if (pu8Buf == NULL || u16BufMaxLen == 0U)
  {
    return STATUS_ERROR;
  }
  if (Util_RFQ_Is_Empty(&g_struUart2RxRfq))
  {
    return STATUS_ERROR;
  }
  rfq_frame_t struRxFrame;
  if (Util_RFQ_Pop(&g_struUart2RxRfq, &struRxFrame) != 0)
  {
    return STATUS_ERROR;
  }
  uint16_t u16CopyLen = struRxFrame.u16Len;
  if (u16CopyLen > u16BufMaxLen)
  {
    u16CopyLen = u16BufMaxLen;
  }
  memcpy(pu8Buf, struRxFrame.au8Bytes, u16CopyLen);

  if (pu16OutLen != NULL)
  {
    *pu16OutLen = struRxFrame.u16Len;
  }
  return STATUS_OK;
}

/**
  * @brief  通过 USART2 使用 DMA 发送一帧数据
  * @param  pBuf  待发送的数据缓冲区指针
  * @param  len   待发送的数据长度（字节）
  * @retval STATUS_OK 成功；STATUS_ERROR 参数非法或底层发送失败
  */
Status_t Drv_Simulink_Feedback_Send(const uint8_t *pu8Buf, uint16_t u16Len)
{
  int a = 0;
  if (pu8Buf == NULL || u16Len == 0U)
  {
    return STATUS_ERROR;
  }
#if 0
  if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)pu8Buf, u16Len) != HAL_OK)
  {
    return STATUS_ERROR;
  }
#endif

  a = HAL_UART_Transmit_DMA(&huart2, (uint8_t *)pu8Buf, u16Len);

  if (a != HAL_OK)
  {
    printf("发送失败：%d\n",a);
    return STATUS_ERROR;
  }




  return STATUS_OK;
}

/**
  * @brief  在 USART3/4/5 中断入口（HAL_UART_IRQHandler 之前）锁存 ISR
  */
void Drv_EncoderUart_LatchHwErrorFlagsAtIrqEntry(EncoderUartSel_t uart_sel)
{
  USART_TypeDef *pUart = EncoderUart_SelToInstance(uart_sel);
  uint8_t u8Idx = EncoderUart_SelToLatchIdx(uart_sel);
  uint32_t u32Isr;

  if ((pUart == NULL) || (u8Idx >= 3U))
  {
    return;
  }

  u32Isr = pUart->ISR;
  s_aEncoderUartHwLatch[u8Idx].u32IsrIrqEntry = u32Isr;
  EncoderUart_MergeStickyFlags(&s_aEncoderUartHwLatch[u8Idx], u32Isr);
}

/**
  * @brief  在编码器 HAL_UARTEx_RxEventCallback 分支入口锁存 ISR 与事件信息
  */
void Drv_EncoderUart_LatchHwErrorFlagsAtRxEvent(EncoderUartSel_t uart_sel, uint16_t u16Size, uint8_t u8EvType)
{
  USART_TypeDef *pUart = EncoderUart_SelToInstance(uart_sel);
  uint8_t u8Idx = EncoderUart_SelToLatchIdx(uart_sel);
  uint32_t u32Isr;

  if ((pUart == NULL) || (u8Idx >= 3U))
  {
    return;
  }

  u32Isr = pUart->ISR;
  s_aEncoderUartHwLatch[u8Idx].u32IsrRxEvent = u32Isr;
  s_aEncoderUartHwLatch[u8Idx].u16RxSize     = u16Size;
  s_aEncoderUartHwLatch[u8Idx].u8RxEvType  = u8EvType;
  EncoderUart_MergeStickyFlags(&s_aEncoderUartHwLatch[u8Idx], u32Isr);
}

/**
  * @brief  错帧调试：打印 RxEvent/IRQ 锁存、累计 sticky 与当前 ISR 的硬件标志
  */
void Drv_EncoderUart_LogHwErrorFlags(EncoderUartSel_t uart_sel)
{
  UART_HandleTypeDef *huart = NULL;
  EncoderUartHwFlagLatch_t latchCopy;
  uint8_t u8Idx = EncoderUart_SelToLatchIdx(uart_sel);
  uint32_t u32IsrNow;
  const char *pcEv = "?";

  if (u8Idx >= 3U)
  {
    return;
  }

  switch (uart_sel)
  {
    case ENCODER_UART_MOTOR:
      huart = &huart3;
      break;
    case ENCODER_UART_SWING:
      huart = &huart4;
      break;
    case ENCODER_UART_SHAFT:
      huart = &huart5;
      break;
    default:
      return;
  }

  latchCopy.u32IsrRxEvent  = s_aEncoderUartHwLatch[u8Idx].u32IsrRxEvent;
  latchCopy.u32IsrIrqEntry = s_aEncoderUartHwLatch[u8Idx].u32IsrIrqEntry;
  latchCopy.u32IsrSticky   = s_aEncoderUartHwLatch[u8Idx].u32IsrSticky;
  latchCopy.u16RxSize      = s_aEncoderUartHwLatch[u8Idx].u16RxSize;
  latchCopy.u8RxEvType     = s_aEncoderUartHwLatch[u8Idx].u8RxEvType;

  if (latchCopy.u8RxEvType == 1U)
  {
    pcEv = "IDLE";
  }
  else if (latchCopy.u8RxEvType == 2U)
  {
    pcEv = "TC";
  }

  u32IsrNow = huart->Instance->ISR;

  printf("\r\n UART硬件锁存: RxEvent");
  EncoderUart_PrintIsrFlags("", latchCopy.u32IsrRxEvent);
  printf(" Size=%u Ev=%s",
         (unsigned int)latchCopy.u16RxSize,
         pcEv);
  EncoderUart_PrintIsrFlags(" IrqEntry", latchCopy.u32IsrIrqEntry);
  EncoderUart_PrintIsrFlags(" Sticky", latchCopy.u32IsrSticky);
  EncoderUart_PrintIsrFlags(" 当前", u32IsrNow);
  printf("\r\n");

  s_aEncoderUartHwLatch[u8Idx].u32IsrRxEvent  = 0U;
  s_aEncoderUartHwLatch[u8Idx].u32IsrIrqEntry = 0U;
  s_aEncoderUartHwLatch[u8Idx].u32IsrSticky   = 0U;
  s_aEncoderUartHwLatch[u8Idx].u16RxSize      = 0U;
  s_aEncoderUartHwLatch[u8Idx].u8RxEvType     = 0U;

  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
}

/**
  * @brief  仅向指定编码器串口发送单字节 0x02 触发（用于 TIM1 分相：更新/比较1/比较2 各触发一路）
  * @param  uart_sel ENCODER_UART_MOTOR(3) / ENCODER_UART_SWING(4) / ENCODER_UART_SHAFT(5)
  */
void Drv_EncoderTrigger_SendOne(EncoderUartSel_t uart_sel)
{
  Drv_EncoderLevelShifter_SetEnable(uart_sel, PRJ_ENABLE);
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
  * @brief  获取电机编码器当前数据（UART3），12 字节；前6字节=最新帧，后6字节=上一帧
  * @param  pOut  指向至少 12 字节的缓冲区
  * @retval STATUS_OK 已写入快照；STATUS_ERROR 指针无效；STATUS_TIMEOUT 序列锁重试耗尽（极少见）
 * @note  双槽每槽序列锁 + front；读侧再慢亦不会产生撕裂（可能多轮重试，有上限）。
 */
Status_t Drv_MotorEncoder_GetData(uint8_t *pu8Out)
{
  if (pu8Out == NULL)
  {
    return STATUS_ERROR;
  }

  return EncoderSnapshot_ReadDualSlot(g_u32MotorSlotSeq, g_au8MotorEncoderBuffPool, &g_u8Motor_front_idx, pu8Out);
}

/**
  * @brief  获取输出轴编码器当前数据（UART5），12 字节；前6字节=最新帧，后6字节=上一帧
  * @param  pOut  指向至少 12 字节的缓冲区
  * @retval STATUS_OK / STATUS_ERROR / STATUS_TIMEOUT 同 Drv_MotorEncoder_GetData
 * @note  与 Drv_MotorEncoder_GetData 相同（双槽序列锁）。
 */
Status_t Drv_OutputShaftEncoder_GetData(uint8_t *pu8Out)
{
  if (pu8Out == NULL)
  {
    return STATUS_ERROR;
  }

  return EncoderSnapshot_ReadDualSlot(g_u32OutputShaftSlotSeq, g_au8OutputShaftEncoderBuffPool,
                                      &g_u8OutputShaft_front_idx, pu8Out);
}

/**
  * @brief  获取摆杆编码器当前数据（UART4），12 字节；前6字节=最新帧，后6字节=上一帧
  * @param  pOut  指向至少 12 字节的缓冲区
  * @retval STATUS_OK / STATUS_ERROR / STATUS_TIMEOUT 同 Drv_MotorEncoder_GetData
 * @note  与 Drv_MotorEncoder_GetData 相同（双槽序列锁）。
 */
Status_t Drv_SwingArmEncoder_GetData(uint8_t *pu8Out)
{
  if (pu8Out == NULL)
  {
    return STATUS_ERROR;
  }

  return EncoderSnapshot_ReadDualSlot(g_u32SwingArmSlotSeq, g_au8SwingArmEncoderBuffPool,
                                      &g_u8SwingArm_front_idx, pu8Out);
}

/**
  * @brief  控制编码器串口对应电平转换芯片的使能引脚（高=发送使能，低=接收使能）
  * @param  uart_sel 串口选择：ENCODER_UART_MOTOR(3)=PB4，ENCODER_UART_SWING(4)=PB9，ENCODER_UART_SHAFT(5)=PC13
  * @param  state    PRJ_ENABLE=高电平可发送，PRJ_DISABLE=低电平可接收
  */
void Drv_EncoderLevelShifter_SetEnable(EncoderUartSel_t uart_sel, FunctionalState_t state)
{
  GPIO_PinState pin_state = (state != PRJ_DISABLE) ? GPIO_PIN_SET : GPIO_PIN_RESET;

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

/**
 * @brief  ISR：开始对 wslot 组帧（seq 置奇，读侧跳过该槽）
 */
static void EncoderSnapshot_BeginSlotAssembly(volatile uint32_t *pu32SeqSlot)
{
  (*pu32SeqSlot)++;
  __COMPILER_BARRIER();
}

/**
 * @brief  ISR：本槽 12 字节已写完 — seq 置偶并公布 front（读侧可读该槽）
 */
static void EncoderSnapshot_EndSlotPublish(volatile uint32_t *pu32SeqSlot,
                                           volatile uint8_t *pFrontIdx,
                                           uint8_t u8SlotIdx)
{
  __COMPILER_BARRIER();
  (*pu32SeqSlot)++;
  __COMPILER_BARRIER();
  *pFrontIdx = u8SlotIdx;
}

/**
 * @brief  任务侧：从 front 所指槽拷贝，该槽 seq 偶且前后一致则成功
 * @retval STATUS_OK 拷贝成功；STATUS_TIMEOUT 超过 ENC_SNAPSHOT_SEQLOCK_MAX_RETRY（此时 pu8Out 填 0）
 */
static Status_t EncoderSnapshot_ReadDualSlot(volatile const uint32_t *pu32SlotSeq,
                                             const uint8_t au8Pool[ENCODER_SNAPSHOT_POOL_NUM][ENCODER_SNAPSHOT_BYTES],
                                             volatile const uint8_t *pu8FrontIdx,
                                             uint8_t *pu8Out)
{
  uint32_t u32Attempt;

  for (u32Attempt = 0U; u32Attempt < ENC_SNAPSHOT_SEQLOCK_MAX_RETRY; u32Attempt++)
  {
    uint8_t u8Idx;
    uint32_t u32S1;
    uint32_t u32S2;

    u8Idx = *pu8FrontIdx;
    if (u8Idx >= ENCODER_SNAPSHOT_POOL_NUM)
    {
      u8Idx = 0U;
    }

    u32S1 = pu32SlotSeq[u8Idx];
    if ((u32S1 & 1U) != 0U)
    {
      continue;
    }
    memcpy(pu8Out, au8Pool[u8Idx], ENCODER_SNAPSHOT_BYTES);
    u32S2 = pu32SlotSeq[u8Idx];
    if (u32S1 == u32S2)
    {
      return STATUS_OK;
    }
  }
  printf("底层重试超时\n");
  (void)memset(pu8Out, 0, ENCODER_SNAPSHOT_BYTES);
  return STATUS_TIMEOUT;
}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* 与编码器口相同：ORE 等错误后 HAL 会停掉 DMA；调试时在 RxEventCallback 内断点易触发溢出，须清标志并重启接收 */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_au8Uart1RecvBuff, UART_1_RX_BUFF_LEN);
  }
  else if (huart->Instance == USART2)
  {
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_au8Uart2RecvBuff, UART2_RX_BUFF_LEN);
  }
  else if (huart->Instance == UART5)
  {
    /* 错误后 HAL 会停止 DMA 接收 */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, g_au8Uart5DMABuff, UART_ENCODER_DMA_RX_BUFF);
  }
  else if (huart->Instance == USART3)
  {
    /* 编码器 UART3 同样在错误后需重启 DMA */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_au8Uart3DMABuff, UART_ENCODER_DMA_RX_BUFF);
  }
  else if (huart->Instance == UART4)
  {
    /* 编码器 UART4 同样在错误后需重启 DMA */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_au8Uart4DMABuff, UART_ENCODER_DMA_RX_BUFF);
  }
}


/**
  * @brief  HAL 接收事件回调：UART1~5 的 DMA+IDLE 接收
  * @param  huart UART 句柄
  * @param  Size  本帧接收长度（字节）
  * @note   通过 HAL_UARTEx_GetRxEventType(huart)：IDLE=空闲线检测，TC=DMA 传完等；编码器分支开头仅处理 IDLE/TC。
 *         编码器段：双槽 wslot=front^1 组帧，每槽 seqlock；见该段上方块注释。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  HAL_UART_RxEventTypeTypeDef ev = HAL_UARTEx_GetRxEventType(huart);

  /* 仅处理 IDLE 与 TC，忽略 HT 等其余事件类型 */
  if (ev != HAL_UART_RXEVENT_IDLE && ev != HAL_UART_RXEVENT_TC)
  {
    return;
  }

  if (huart->Instance == USART1)
  {
    if (Size > 0U)
    {
      Util_RFQ_Push(&g_struUart1RxRfq, g_au8Uart1RecvBuff, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_au8Uart1RecvBuff, UART_1_RX_BUFF_LEN);
  }
  else if (huart->Instance == USART2)
  {
    if (Size > 0U)
    {
      Util_RFQ_Push(&g_struUart2RxRfq, g_au8Uart2RecvBuff, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_au8Uart2RecvBuff, UART2_RX_BUFF_LEN);
  }

  /* -------- 编码器 UART3/4/5：IDLE(或 TC) 事件，每 event 视情况处理一帧 ≥6 字节 -------- */
  /* 双槽组帧 + 每槽 seqlock；相邻两帧见 wire_prev。 */
  else if (huart->Instance == USART3)
  {
    Drv_EncoderUart_LatchHwErrorFlagsAtRxEvent(ENCODER_UART_MOTOR, Size,
                                             (uint8_t)((ev == HAL_UART_RXEVENT_IDLE) ? 1U : 2U));
    if (Size >= 6U)
    {
      uint8_t u8Wslot = (uint8_t)(g_u8Motor_front_idx ^ 1U);

      EncoderSnapshot_BeginSlotAssembly(&g_u32MotorSlotSeq[u8Wslot]);

      /* 组 12 字节：[0..5]=本拍 DMA；[6..11]=上一拍 wire_prev。
       * 首拍尚无上一帧：若 memset 为 0，解析器会把 [6..11] 当第二帧校验 CM(0x02) → 误判「帧头错误」。
       * 用本拍数据占位，两帧相同则首周期转速差为 0，与「无历史」语义一致。 */
      if (g_u8Motor_chain_valid == 0U)
      {
        memcpy(g_au8MotorEncoderBuffPool[u8Wslot] + ENCODER_FRAME_LENGTH_BYTES,
               g_au8Uart3DMABuff,
               ENCODER_FRAME_LENGTH_BYTES);
      }
      else
      {
        memcpy(g_au8MotorEncoderBuffPool[u8Wslot] + ENCODER_FRAME_LENGTH_BYTES,
               g_au8Motor_wire_prev,
               ENCODER_FRAME_LENGTH_BYTES);
      }
      memcpy(g_au8MotorEncoderBuffPool[u8Wslot] + 0U,
             g_au8Uart3DMABuff,
             ENCODER_FRAME_LENGTH_BYTES);
      memcpy(g_au8Motor_wire_prev, g_au8Uart3DMABuff, ENCODER_FRAME_LENGTH_BYTES);
      g_u8Motor_chain_valid = 1U;
      EncoderSnapshot_EndSlotPublish(&g_u32MotorSlotSeq[u8Wslot], &g_u8Motor_front_idx, u8Wslot);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_au8Uart3DMABuff, UART_ENCODER_DMA_RX_BUFF);
  }
  else if (huart->Instance == UART4)
  {
    Drv_EncoderUart_LatchHwErrorFlagsAtRxEvent(ENCODER_UART_SWING, Size,
                                             (uint8_t)((ev == HAL_UART_RXEVENT_IDLE) ? 1U : 2U));
    if (Size >= 6U)
    {
      uint8_t u8Wslot = (uint8_t)(g_u8SwingArm_front_idx ^ 1U);

      EncoderSnapshot_BeginSlotAssembly(&g_u32SwingArmSlotSeq[u8Wslot]);

      if (g_u8SwingArm_chain_valid == 0U)
      {
        memcpy(g_au8SwingArmEncoderBuffPool[u8Wslot] + ENCODER_FRAME_LENGTH_BYTES,
               g_au8Uart4DMABuff,
               ENCODER_FRAME_LENGTH_BYTES);
      }
      else
      {
        memcpy(g_au8SwingArmEncoderBuffPool[u8Wslot] + ENCODER_FRAME_LENGTH_BYTES,
               g_au8SwingArm_wire_prev,
               ENCODER_FRAME_LENGTH_BYTES);
      }
      memcpy(g_au8SwingArmEncoderBuffPool[u8Wslot] + 0U,
             g_au8Uart4DMABuff,
             ENCODER_FRAME_LENGTH_BYTES);
      memcpy(g_au8SwingArm_wire_prev, g_au8Uart4DMABuff, ENCODER_FRAME_LENGTH_BYTES);
      g_u8SwingArm_chain_valid = 1U;
      EncoderSnapshot_EndSlotPublish(&g_u32SwingArmSlotSeq[u8Wslot], &g_u8SwingArm_front_idx, u8Wslot);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_au8Uart4DMABuff, UART_ENCODER_DMA_RX_BUFF);
  }
  else if (huart->Instance == UART5)
  {
    Drv_EncoderUart_LatchHwErrorFlagsAtRxEvent(ENCODER_UART_SHAFT, Size,
                                             (uint8_t)((ev == HAL_UART_RXEVENT_IDLE) ? 1U : 2U));
    if (Size >= 6U)
    {
      uint8_t u8Wslot = (uint8_t)(g_u8OutputShaft_front_idx ^ 1U);

      EncoderSnapshot_BeginSlotAssembly(&g_u32OutputShaftSlotSeq[u8Wslot]);

      if (g_u8OutputShaft_chain_valid == 0U)
      {
        memcpy(g_au8OutputShaftEncoderBuffPool[u8Wslot] + ENCODER_FRAME_LENGTH_BYTES,
               g_au8Uart5DMABuff,
               ENCODER_FRAME_LENGTH_BYTES);
      }
      else
      {
        memcpy(g_au8OutputShaftEncoderBuffPool[u8Wslot] + ENCODER_FRAME_LENGTH_BYTES,
               g_au8OutputShaft_wire_prev,
               ENCODER_FRAME_LENGTH_BYTES);
      }
      memcpy(g_au8OutputShaftEncoderBuffPool[u8Wslot] + 0U,
             g_au8Uart5DMABuff,
             ENCODER_FRAME_LENGTH_BYTES);
      memcpy(g_au8OutputShaft_wire_prev, g_au8Uart5DMABuff, ENCODER_FRAME_LENGTH_BYTES);
      g_u8OutputShaft_chain_valid = 1U;
      EncoderSnapshot_EndSlotPublish(&g_u32OutputShaftSlotSeq[u8Wslot], &g_u8OutputShaft_front_idx, u8Wslot);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, g_au8Uart5DMABuff, UART_ENCODER_DMA_RX_BUFF);
  }
}

/* USER CODE END 1 */

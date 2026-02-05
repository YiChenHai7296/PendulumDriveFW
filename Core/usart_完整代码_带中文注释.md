# USART 完整代码（带正确中文注释）

以下为 `usart.h` 与 `usart.c` 的完整可复制代码，注释已统一为正确中文。

---

## 一、Core/Inc/usart.h

```c
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
#include <string.h>
#include "RingFrameQueue.h"
#include "stm32g4xx_hal_uart_ex.h"
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
/**
  * @brief  从 USART2 接收队列出队一帧到指定缓冲区（如 simulink_protocol 等调用）
  * @param  pBuf      数据存放的缓冲区指针
  * @param  bufMaxLen 缓冲区最大长度（字节）
  * @param  pOutLen   本次出队的实际长度，可为 NULL
  * @retval 0  成功；-1 队列空或参数无效
  */
int USART2_GetRxData(uint8_t *pBuf, uint16_t bufMaxLen, uint16_t *pOutLen);

/**
  * @brief  通过 USART2 使用 DMA 发送一帧数据
  * @param  pBuf  待发送的数据缓冲区指针
  * @param  len   待发送的数据长度（字节）
  * @retval 0 成功；-1 参数非法或底层发送失败
  */
int USART2_SendData_DMA(const uint8_t *pBuf, uint16_t len);

#define ENCODER_SNAPSHOT_BYTES  12U   /**< 编码器快照长度：前6字节=最新帧，后6字节=上一帧 */

/**
  * @brief  定时发送：向串口3、4、5依次发送单字节 0x02（在 TIM1 定时中断中调用）
  */
void USART345_EncoderTrigger_Send(void);

/**
  * @brief  获取电机编码器当前数据（UART3），12 字节拷贝到入参指向的缓冲区；前6字节=最新帧，后6字节=上一帧
  * @param  pOut  单字节数组指针，指向至少 ENCODER_SNAPSHOT_BYTES(12) 字节的缓冲区
  */
void USART3_MotorEncoder_GetData(uint8_t *pOut);

/**
  * @brief  获取输出轴编码器当前数据（UART4），12 字节拷贝到入参指向的缓冲区；前6字节=最新帧，后6字节=上一帧
  * @param  pOut  单字节数组指针，指向至少 ENCODER_SNAPSHOT_BYTES(12) 字节的缓冲区
  */
void USART4_OutputShaftEncoder_GetData(uint8_t *pOut);

/**
  * @brief  获取摆臂编码器当前数据（UART5），12 字节拷贝到入参指向的缓冲区；前6字节=最新帧，后6字节=上一帧
  * @param  pOut  单字节数组指针，指向至少 ENCODER_SNAPSHOT_BYTES(12) 字节的缓冲区
  */
void USART5_SwingArmEncoder_GetData(uint8_t *pOut);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
```

---

## 二、Core/Src/usart.c（分段）

`usart.c` 约 974 行，分为 **4 段** 便于复制。按顺序复制到同一文件即可得到完整 `usart.c`。

### 段 1：文件头 + USER CODE 0 + 句柄与 UART4/5/USART1 初始化（约 1–228 行）

请直接打开工程内 **Core/Src/usart.c**，从第 1 行复制到第 228 行（即 `/* USER CODE END USART1_Init 2 */` 及对应的 `}` 与空行）。

内容包含：
- 文件头与 `#include "usart.h"`
- `/* USER CODE BEGIN 0 */` 内：宏（UART_1_RX_BUFF_LEN、ENCODER_SNAPSHOT_BYTES 等）、DMA/编码器/接收队列等变量
- `UART_HandleTypeDef` / `DMA_HandleTypeDef` 声明
- `MX_UART4_Init`、`MX_UART5_Init`、`MX_USART1_UART_Init` 完整实现

### 段 2：USART2/USART3 初始化 + MspInit（约 229–694 行）

从 **Core/Src/usart.c** 第 229 行复制到第 694 行。

内容包含：
- `MX_USART2_UART_Init`、`MX_USART3_UART_Init`
- `HAL_UART_MspInit`（UART4/5/USART1/2/3 的时钟、GPIO、DMA、中断）

### 段 3：MspDeInit（约 696–823 行）

从 **Core/Src/usart.c** 第 696 行复制到第 823 行。

内容为 `HAL_UART_MspDeInit` 完整实现。

### 段 4：USER CODE 1（用户实现，约 825–974 行）

从 **Core/Src/usart.c** 第 825 行（`/* USER CODE BEGIN 1 */`）复制到文件末尾。

内容包含（均为正确中文注释）：
- `USART2_GetRxData`：从 USART2 接收队列出队一帧
- `USART2_SendData_DMA`：通过 USART2 DMA 发送一帧
- `USART345_EncoderTrigger_Send`：定时向串口 3/4/5 发送 0x02
- `USART3_MotorEncoder_GetData`、`USART4_OutputShaftEncoder_GetData`、`USART5_SwingArmEncoder_GetData`：获取各编码器快照
- `HAL_UARTEx_RxEventCallback`：DMA+IDLE 接收事件回调

---

## 使用说明

- **usart.h**：复制上面「一、Core/Inc/usart.h」整段代码，覆盖 `Core/Inc/usart.h` 即可。
- **usart.c**：工程内 `Core/Src/usart.c` 已按上述修改为「完整代码 + 正确中文注释」。若需在别处重建，按「段 1～段 4」从该文件按行范围复制即可得到完整版。

当前工程里的 **Core/Src/usart.c** 与 **Core/Inc/usart.h** 已全部替换为带正确中文注释的版本，可直接使用或再按需微调。

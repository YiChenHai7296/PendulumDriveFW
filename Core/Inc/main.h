/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* 系统时钟配置（定义在 main.c）；声明在此以便 bsp.c 等在 Bsp_Init 中调用时可见原型 */
void SystemClock_Config(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder_OutputShaft_Enable_Pin GPIO_PIN_13
#define Encoder_OutputShaft_Enable_GPIO_Port GPIOC
#define Encoder_Motor_Enable_Pin GPIO_PIN_4
#define Encoder_Motor_Enable_GPIO_Port GPIOB
#define MotorEnableControl_Pin GPIO_PIN_5
#define MotorEnableControl_GPIO_Port GPIOB
#define MotorDirectionControl_Pin GPIO_PIN_6
#define MotorDirectionControl_GPIO_Port GPIOB
#define Encoder_SwingArm_Enable_Pin GPIO_PIN_9
#define Encoder_SwingArm_Enable_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

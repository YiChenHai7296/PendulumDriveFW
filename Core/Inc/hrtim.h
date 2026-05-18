/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hrtim.h
  * @brief   This file contains all the function prototypes for
  *          the hrtim.c file
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
#ifndef __HRTIM_H__
#define __HRTIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "common.h"
/* USER CODE END Includes */

extern HRTIM_HandleTypeDef hhrtim1;

/* USER CODE BEGIN Private defines */
/* 电机方向控制宏：正转 / 反转 */
#define MOTOR_DIR_FORWARD  0U
#define MOTOR_DIR_REVERSE  1U

/** PWM 占空比/指令有效下限（与此比较）；低于则视为无效脉宽或停机死区。
 *  须 ≥4：关断占位用同一套 CMP 公式时，k<4 会使 CMP2=k/2-1=0，ADC2 外触发(TRG6←CMP2)失效。 */
#define PWM_DUTY_CYCLE_MIN  (8U)
#if (PWM_DUTY_CYCLE_MIN < 4U)
#error "PWM_DUTY_CYCLE_MIN must be >= 4 (ADC2 uses HRTIM TimerA CMP2 external trigger)"
#endif

/* USER CODE END Private defines */

void MX_HRTIM1_Init(void);

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* USER CODE BEGIN Prototypes */
/* 电机 PWM 驱动层接口（`Core/Src/hrtim.c`，Drv_PWM_*），由 `User/Service` 电机服务等调用。 */

Status_t Drv_PWM_TargePulse_Set(uint16_t u16ExpectedValue);
void Drv_PWM_DirControl(uint8_t dir);
void Drv_PWM_Enable(FunctionalState_t NewState);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __HRTIM_H__ */


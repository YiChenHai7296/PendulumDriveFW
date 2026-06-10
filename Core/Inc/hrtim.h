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
/* ======================== 1. 头文件依赖 ======================== */
#include "common.h"

/* ======================== 2. 宏定义（对外可见） ======================== */
/* 电机方向控制宏：正转 / 反转 */
#define MOTOR_DIR_FORWARD  0U
#define MOTOR_DIR_REVERSE  1U

/** PWM 占空比/指令有效下限（与此比较）；低于则视为无效脉宽或停机死区。
 *  须 ≥4：关断占位用同一套 CMP 公式时，k<4 会使 CMP2=k/2-1=0，ADC2 外触发(TRG6←CMP2)失效。 */
#define PWM_DUTY_CYCLE_MIN  (8U)
#if (PWM_DUTY_CYCLE_MIN < 4U)
#error "PWM_DUTY_CYCLE_MIN must be >= 4 (ADC2 uses HRTIM TimerA CMP2 external trigger)"
#endif

/* ======================== 3. 类型定义 ======================== */
/* 无 */

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */

/* -------- 5.1 上层接口 -------- */
/* 电机 PWM 驱动层（`Core/Src/hrtim.c`）；由 `User/Service` 电机服务等调用。 */

/**
 * @brief 设置 PWM 目标占空比并置更新标志，待 TimerB 比较中断装载到比较寄存器
 * @param u16ExpectedValue 目标占空比（0~10000，单位 0.01%）
 * @return STATUS_OK；超过 10000 返回 STATUS_ERROR
 */
Status_t Drv_PWM_TargetPulse_Set(uint16_t u16ExpectedValue);

/**
 * @brief 电机方向控制：控制电机方向引脚
 * @param dir 方向宏：MOTOR_DIR_FORWARD 正转，MOTOR_DIR_REVERSE 反转
 * @note  正转时将 MotorDirectionControl_Pin 置 0，反转时置 1
 */
void Drv_PWM_Direction_Set(uint8_t dir);

/**
 * @brief PWM 使能控制
 * @param NewState 使能状态：PRJ_ENABLE / PRJ_DISABLE
 * @note  使能时将 MotorEnableControl_Pin 置 1；失能时置 0，并清除待更新标志
 */
void Drv_PWM_Enable(FunctionalState_t NewState);

/* -------- 5.2 层内接口（Drv_Loc_*） -------- */
/* 无 */

/* USER CODE END Includes */

extern HRTIM_HandleTypeDef hhrtim1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_HRTIM1_Init(void);

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __HRTIM_H__ */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/* ======================== 1. 头文件依赖 ======================== */
#include "common.h"

/* ======================== 2. 宏定义（对外可见） ======================== */
/** ADC 参考电压 (V)，与板级 VDDA 一致；原始码值换算电压时与服务层共用此宏 */
#define ADC_REFERENCE_VOLTAGE_V  (3.3f)
/** 原始码换算电压时的满量程（本工程 12 位右对齐 + 换算约定，与 `ADC_REFERENCE_VOLTAGE_V` 配套作除数使用） */
#define ADC_RAW_FULL_RESOLUTION  (4096.0f)

/* ======================== 3. 类型定义 ======================== */
/* 无 */

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */

/* -------- 5.1 上层接口 -------- */
/* ADC 驱动层（`Core/Src/adc.c`）；由 `User/Service` 等调用。 */

/**
 * @brief 读取 ADC1、ADC2 快照中的原始采样值（各 12 位有效）
 * @note  数据来自 `Drv_Loc_ADC_Motor_RawData_Snapshot`；32 位快照一次拆成两路，避免读侧跨周期混叠
 * @param pOut 至少 2 个 uint16_t：pOut[0]=ADC1，pOut[1]=ADC2
 * @return STATUS_OK 写入成功；STATUS_ERROR 指针无效（pOut 为 NULL）
 */
Status_t Drv_ADC_Motor_RawData_Read(uint16_t *pOut);

/**
 * @brief 读取 SoftRuler 通道（ADC3_IN1）当前 DMA 缓冲中的原始值（12 位有效）
 * @param[out] pOut 输出 1 个 uint16_t
 * @return STATUS_OK / STATUS_ERROR（pOut 为空）
 */
Status_t Drv_ADC_SoftRuler_RawData_Read(uint16_t *pOut);

/**
 * @brief  调试：读电机 ADC1/2 与摆杆 SoftRuler(ADC3) 原始码，换算采样电压并计算理论量后 `printf`
 * @note   电机侧理论电流：(Vadc-1.6)/41/0.02（A）；摆杆侧理论输入电压：(Vadc-1.6)/0.16（V）。
 *         电机两路数据来自中断里已执行的 `Drv_Loc_ADC_Motor_RawData_Snapshot`；本函数不再打快照。`printf` 需 `DEBUG_PRINTF`。
 */
void Drv_ADC_TEST(void);

/* -------- 5.2 层内接口（Drv_Loc_*） -------- */

/**
 * @brief 抓取电机电流 ADC1/ADC2 当前值，打包为单个 32 位快照（低 16 位=ADC1，高 16 位=ADC2）
 * @note  层内接口：由 `hrtim.c` HRTIM 比较中断调用，与采样相位对齐
 */
void Drv_Loc_ADC_Motor_RawData_Snapshot(void);

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */


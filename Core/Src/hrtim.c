/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hrtim.c
  * @brief   This file provides code for the configuration
  *          of the HRTIM instances.
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
#include "hrtim.h"

/* USER CODE BEGIN 0 */

/* ======================== 1. 头文件引用（本文件额外） ======================== */
#include "adc.h"
#include "usart.h"
#include "bsp.h"      /* BSP_LOG_PRINTF */

/* ======================== 2. 私有宏定义 ======================== */
/* 无 */

/* ======================== 3. 私有类型定义 ======================== */
/* 无 */

/* ======================== 4. 对外变量定义 ======================== */
/* 无 */

/* ======================== 5. 私有变量 ======================== */
static volatile uint16_t s_u16TargetPulse = 0U; /* PWM 占空比目标值，范围 0~10000 */
static volatile uint8_t  s_u8FlagPulse   = 0U; /* PWM 占空比更新标志 */

/* ======================== 6. 私有函数声明 ======================== */
static Status_t PWM_Pulse_Write(uint16_t u16ScaledPulse);

/* USER CODE END 0 */

HRTIM_HandleTypeDef hhrtim1;

/* HRTIM1 init function */
void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP3;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_NONE;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT6810_TIMERA_CMP2;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_6, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_6, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0x84D0;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL2;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0x4268;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.TrigHalf = HRTIM_TIMERTRIGHALF_DISABLED;
  pTimerCtl.GreaterCMP3 = HRTIM_TIMERGTCMP3_EQUAL;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_CMP1;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 0x2134;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 0x31CE;
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 0x109A;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0x84D0;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 0x5302;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */
  /* Timer A/B 均使能 CMP1 中断（InterruptRequests=HRTIM_TIM_IT_CMP1，见上方 Timer B 配置）：
   * - Timer A CMP1：HAL_HRTIM_Compare1EventCallback(TIMER_A) → Drv_Loc_ADC_Motor_RawData_Snapshot。
   * - Timer B CMP1=0x5302：同回调 TIMER_B 分支 → 若有占空比更新标志则 PWM_Pulse_Write（×1.7 折算后写 TimerA 比较寄存器）。
   * NVIC：TIMB(2) 高于 TIMA(3)，占空比装载可抢占 ADC 快照中断。 */
  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

}

void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspInit 0 */

  /* USER CODE END HRTIM1_MspInit 0 */
    /* HRTIM1 clock enable */
    __HAL_RCC_HRTIM1_CLK_ENABLE();

    /* HRTIM1 interrupt Init */
    HAL_NVIC_SetPriority(HRTIM1_TIMA_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);
    HAL_NVIC_SetPriority(HRTIM1_TIMB_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_TIMB_IRQn);
  /* USER CODE BEGIN HRTIM1_MspInit 1 */

  /* USER CODE END HRTIM1_MspInit 1 */
  }
}

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspPostInit 0 */

  /* USER CODE END HRTIM1_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**HRTIM1 GPIO Configuration
    PA8     ------> HRTIM1_CHA1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN HRTIM1_MspPostInit 1 */

  /* USER CODE END HRTIM1_MspPostInit 1 */
  }

}

void HAL_HRTIM_MspDeInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspDeInit 0 */

  /* USER CODE END HRTIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_HRTIM1_CLK_DISABLE();

    /* HRTIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(HRTIM1_TIMA_IRQn);
    HAL_NVIC_DisableIRQ(HRTIM1_TIMB_IRQn);
  /* USER CODE BEGIN HRTIM1_MspDeInit 1 */

  /* USER CODE END HRTIM1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* ======================== 7. 接口函数实现 ======================== */

/* -------- 7.1 上层接口（与 .h 5.1 对应） -------- */

/**
 * @brief PWM 使能控制接口函数
 * @param NewState 使能状态：PRJ_ENABLE / PRJ_DISABLE
 * @note  使能时将 MotorEnableControl_Pin 置 1；失能时置 0，并清除待更新标志，防止停机后残留一次寄存器更新
 */
void Drv_PWM_Enable(FunctionalState_t NewState)
{
    if (NewState != PRJ_DISABLE)
    {
        HAL_GPIO_WritePin(MotorEnableControl_GPIO_Port, MotorEnableControl_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MotorEnableControl_GPIO_Port, MotorEnableControl_Pin, GPIO_PIN_RESET);
        s_u8FlagPulse = 0;
    }
}



/**
 * @brief 电机方向控制接口函数：控制电机方向引脚
 * @param dir 方向宏：MOTOR_DIR_FORWARD 正转，MOTOR_DIR_REVERSE 反转
 * @note  正转时将 MotorDirectionControl_Pin 置 0，反转时置 1
 */
void Drv_PWM_Direction_Set(uint8_t dir)
{
    if (dir == MOTOR_DIR_FORWARD)
    {
        HAL_GPIO_WritePin(MotorDirectionControl_GPIO_Port, MotorDirectionControl_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(MotorDirectionControl_GPIO_Port, MotorDirectionControl_Pin, GPIO_PIN_SET);
    }
}


/**
 * @brief 设置 PWM 目标占空比并置更新标志，待 TimerB 比较中断装载到比较寄存器
 * @param u16ExpectedValue 目标占空比（0~10000，单位 0.01%）
 * @return STATUS_OK；超过 10000 返回 STATUS_ERROR
 */
Status_t Drv_PWM_TargetPulse_Set(uint16_t u16ExpectedValue)
{
    if (u16ExpectedValue > 10000U)
    {
        return STATUS_ERROR;
    }
    s_u16TargetPulse = u16ExpectedValue;
    s_u8FlagPulse   = 1U;
    return STATUS_OK;
}

/**
 * @brief PWM 手动测试：经调试串口读入 5 位数字（00000~10000），设为目标占空比
 * @note  仅用于开发期联调（受 main.c 的 TEST_PWM 开关控制）；先预读最多 10 字节清 FIFO，
 *        再阻塞等待 5 位 ASCII 数字；不应在正常应用主循环中调用
 */
void Drv_PWM_TEST(void)
{
    uint16_t u16TargetPulseTemp = 0;
    /* 自测前关闭 USART1 DMA+IDLE 收帧，避免与下方 HAL_UART_Receive 争用同一 UART */
    (void)HAL_UART_DMAStop(&huart1);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);

    /* 预读一帧调试串口数据 */
    HAL_UART_Receive(&DEBUG_UART_HANDLE, g_au8DebugRxBuff, 10, 100);

    /* 提示输入占空比值（00000 ~ 10000 对应 0.00% ~ 100.00%） */
    BSP_LOG_PRINTF("\n 请输入占空比值（00000 ~ 10000 对应 0.00%% ~ 100.00%%） \n");
    while (HAL_OK != HAL_UART_Receive(&DEBUG_UART_HANDLE, g_au8DebugRxBuff, 5, DEBUG_UART_TIMEOUT))
    {
        ;
    }

    u16TargetPulseTemp = (g_au8DebugRxBuff[0] - 0x30) * 10000 + (g_au8DebugRxBuff[1] - 0x30) * 1000 +
                      (g_au8DebugRxBuff[2] - 0x30) * 100 + (g_au8DebugRxBuff[3] - 0x30) * 10 + g_au8DebugRxBuff[4] - 0x30;

    if(0<= u16TargetPulseTemp && 10000>= u16TargetPulseTemp)
    {
        s_u16TargetPulse = u16TargetPulseTemp;
        s_u8FlagPulse = 1U;
        BSP_LOG_PRINTF("\n %u.%02u%% 已写入！\n",
                       (unsigned)(s_u16TargetPulse / 100U),
                       (unsigned)(s_u16TargetPulse % 100U));
    }
    else
    {
        BSP_LOG_PRINTF("\n 键入有误！ \n");
    }
}

/* -------- 7.2 层内接口（Drv_Loc_*，与 .h 5.2 对应） -------- */
/* 无 */

/* ======================== 8. 私有函数实现 ======================== */

/**
 * @brief 将占空比参数写入 HRTIM TimerA 比较寄存器（仅本文件调用）
 */
static Status_t PWM_Pulse_Write(uint16_t u16ScaledPulse)
{
    /* 低于 PWM_DUTY_CYCLE_MIN：逻辑关断。不可把 TimerA 三比较器全写 0：
     * ADC1/2 由 HRTIM TimerA 的 CMP3/CMP2 外触发，全 0 时外触发与 DMA 停步，
     * Drv_Loc_ADC_Motor_RawData_Snapshot 仍读旧缓冲，电流反馈会卡在上一非零占空比时的值（如约 -170mA）。 */
    if (u16ScaledPulse < PWM_DUTY_CYCLE_MIN)
    {
        /* 占位：CMP 用 PWM_DUTY_CYCLE_MIN（当前 8，须 ≥4，见 hrtim.h），保证 CMP2≠0、ADC 外触发不断 */
        const uint16_t u16Keep = PWM_DUTY_CYCLE_MIN;

        HRTIM1->sTimerxRegs[0].CMP1xR = u16Keep - 1U;
        HRTIM1->sTimerxRegs[0].CMP2xR = u16Keep / 2U - 1U;
        HRTIM1->sTimerxRegs[0].CMP3xR = (17000U - u16Keep) / 2U + u16Keep - 1U;
        return STATUS_OK;
    }

    if (u16ScaledPulse > 17000U)
    {
        return STATUS_ERROR;
    }

    /* 更新 HRTIM 定时器比较寄存器；CMP3：低电平区间中点附近触发 ADC */
    HRTIM1->sTimerxRegs[0].CMP1xR = u16ScaledPulse - 1U;
    HRTIM1->sTimerxRegs[0].CMP2xR = u16ScaledPulse / 2U - 1U;
    HRTIM1->sTimerxRegs[0].CMP3xR = (17000U - u16ScaledPulse) / 2U + u16ScaledPulse - 1U;

    return STATUS_OK;
}

/* ======================== 9. HAL 回调函数实现 ======================== */

/**
 * @brief HRTIM 比较器 1（CMP1）事件回调
 * @details TimerA：触发电机电流原始数据快照 Drv_Loc_ADC_Motor_RawData_Snapshot；
 *          TimerB：若有占空比更新标志，则把目标占空比（×1.7 折算后）装载到比较寄存器。
 * @param hhrtim     HRTIM 句柄
 * @param u32TimerIdx 触发事件的定时器索引（区分 TimerA / TimerB）
 */
void HAL_HRTIM_Compare1EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t u32TimerIdx)
{
    if(HRTIM_TIMERINDEX_TIMER_A == u32TimerIdx)
    {
        Drv_Loc_ADC_Motor_RawData_Snapshot();
        return;
    }

    if(HRTIM_TIMERINDEX_TIMER_B == u32TimerIdx)
    {
        if (s_u8FlagPulse)
        {
            s_u8FlagPulse = 0U;
            if (PWM_Pulse_Write((uint16_t)((float)s_u16TargetPulse * 1.7f)) != STATUS_OK)
            {
                return;
            }
        }
    }
}

/* USER CODE END 1 */

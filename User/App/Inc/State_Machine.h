/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    State_Machine.h
  * @brief   状态机头文件
  *          
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __STATE_MACHINE__
#define __STATE_MACHINE__

#ifdef __cplusplus
extern "C" {
#endif

/* 头文件依赖 ------------------------------------------------------------------*/
#include "main.h"
#include "simulink_protocol.h"
#include "motor_service.h"
#include "usart.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* ======================== 对外接口声明 ======================== */
/**
 * @brief 主状态机：持续处理 Simulink 上位机控制与反馈
 * @details 阻塞循环：等待控制帧->解帧->设置占空比->采集反馈->组帧->发送
 */
void Svc_StateMachine_MainLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_MACHINE__ */
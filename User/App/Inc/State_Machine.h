/* USER CODE BEGIN Header */
/**
 * @file    State_Machine.h
 * @brief   应用层（业务层）：主循环状态机对外入口
 * @details 模块路径：`User/App`。
 *          职责：编排「Simulink 控制/反馈」与「电机反馈/转速」等服务调用；
 *          不包含 HAL/寄存器细节，头文件无第三方依赖（仅声明 `void` 入口）。
 *          依赖关系：业务层 → 服务层（`simulink_protocol`、`motor_service`）。
 */
/* USER CODE END Header */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 1. 头文件依赖 ======================== */
/* USER CODE BEGIN Includes */
#include "config.h"   /* 控制对象选择 APP_CONTROL_OBJECT_SELECT 等用户开关在 Config 模块定义 */
/* USER CODE END Includes */

/* ======================== 2. 宏定义（对外可见） ======================== */
/* 用户开关（控制对象选择）已集中至 `User/Config/Inc/config.h` */

/* ======================== 3. 类型定义 ======================== */
/* 无 */

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */
/**
 * @brief 主状态机：持续处理 Simulink 上位机控制与反馈
 * @details 入口由 `main` 或初始化流程调用；实现见 `State_Machine.c`。
 */
void App_StateMachine_MainLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* STATE_MACHINE_H */
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

#ifndef __STATE_MACHINE__
#define __STATE_MACHINE__

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 1. 头文件依赖 ======================== */
#include <stdint.h>
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* ======================== 2. 宏定义（对外可见） ======================== */
/* 无 */

/* ======================== 3. 类型定义 ======================== */
/* 无 */

/* ======================== 4. 对外变量声明 ======================== */
/** 最近一次「完整成功」主循环耗时（μs，含取控制帧～发反馈），可在调试器 Watch 中观察 */
extern volatile uint32_t g_u32AppMainLoopLastUs;
/** 完整成功主循环的历史最大/最小耗时（μs） */
extern volatile uint32_t g_u32AppMainLoopMaxUs;
extern volatile uint32_t g_u32AppMainLoopMinUs;
/** 完整成功主循环的采样次数 */
extern volatile uint32_t g_u32AppMainLoopSampleCount;

/* ======================== 5. 接口函数声明 ======================== */
/**
 * @brief 主状态机：持续处理 Simulink 上位机控制与反馈
 * @details 入口由 `main` 或初始化流程调用；实现见 `State_Machine.c`。
 */
void App_StateMachine_MainLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_MACHINE__ */

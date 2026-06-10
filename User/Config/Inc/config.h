/**
 * @file    config.h
 * @brief   配置模块（Config）：工程级用户开关集中定义
 * @details 模块路径：`User/Config`。本模块仅含头文件，零代码实现、零外设依赖。
 *          将原本散落在各模块（BSP / Service / App）中的「用户开关」统一收拢于此，
 *          作为全工程唯一配置入口，便于裁剪、对比与版本管理。
 *
 *          开关按以下顺序组织，修改时请就近放入对应分区：
 *            1. 编译开关：决定条件编译走向、选择构建变体（影响最终生成的代码）。
 *            2. 功能开关：使能 / 失能某项运行期功能或行为分支。
 *            3. 调试开关：仅用于开发调试（如串口打印），发布前宜收敛。
 *
 *          使用方式：需要这些开关的头文件 `#include "config.h"` 即可，
 *          原模块头文件不再各自定义开关，避免重复与漂移。
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


/* ============================================================== */
/* =================== 1. 编译选项（Compile） =================== */
/* ============================================================== */
/**
 * @brief 软件程序版本号
 */
#define VERSION_NUM_STR "2.0"

/**
 * @brief 应用层控制对象选择（编译期）
 * @note  通过条件编译（见 `State_Machine.c` 的 `#if`）选择反馈对象与对应代码分支：
 *        APP_CONTROL_OBJECT_INVERTED_PENDULUM   -> 倒立摆（摆杆来自编码器）
 *        APP_CONTROL_OBJECT_SOFT_RULER_PENDULUM -> 软尺摆（摆杆量来自 ADC3 摆动电压）
 *        将 APP_CONTROL_OBJECT_SELECT 设为上述之一即可切换。
 */
#define APP_CONTROL_OBJECT_INVERTED_PENDULUM   0U
#define APP_CONTROL_OBJECT_SOFT_RULER_PENDULUM 1U
#define APP_CONTROL_OBJECT_SELECT              APP_CONTROL_OBJECT_SOFT_RULER_PENDULUM

#if (APP_CONTROL_OBJECT_SELECT == APP_CONTROL_OBJECT_INVERTED_PENDULUM)
#define APP_CONTROL_OBJECT_NAME_STR            "Inverted Pendulum"
#elif (APP_CONTROL_OBJECT_SELECT == APP_CONTROL_OBJECT_SOFT_RULER_PENDULUM)
#define APP_CONTROL_OBJECT_NAME_STR            "Soft Ruler Pendulum"
#else
#error "APP_CONTROL_OBJECT_SELECT 配置非法"
#endif

/* ============================================================== */
/* =================== 2. 功能选项（Feature） =================== */
/* ============================================================== */
/**
 * @brief 电流零点偏置校准开关
 * @note  1: 启用校准，反馈电流扣除零点偏置
 *        0: 关闭校准，直接上报原始电流
 */
#define MOTOR_CURRENT_ZERO_CALIB_ENABLE   1U

/**
 * @brief PWM 指令映射模式开关
 * @note  1: 拟合反推（上位机给"目标实际占空比"，内部反推"应给定值"）
 *        0: 直通（上位机给定值即最终输出值）
 */
#define MOTOR_PWM_USE_FIT_MAPPING         1U

/* ============================================================== */
/* ==================== 3. 调试选项（Debug） ==================== */
/* ============================================================== */
/**
 * @brief printf 调试输出总开关
 * @note  0: 关闭（fputc 内的串口发送被预处理器删除，printf 静默）
 *        1: 打开（printf 经 UART 输出）
 */
#define DEBUG_PRINTF          0
/* 调试日志宏 BSP_LOG_PRINTF 基于本开关，定义在 BSP 模块 `User/Bsp/Inc/bsp.h` */

/**
 * @brief 调试 printf 重定向所使用的 UART 句柄
 * @note  取值为 `usart.c` 中已定义的 `UART_HandleTypeDef` 实例名；由 `bsp.c` 的 fputc 使用。
 *        当前板级：USART1（PC4=TX, PC5=RX, 115200）→ `huart1`；改调试口时只改此处。
 */
#define DEBUG_UART_HANDLE     huart1

/**
 * @brief 调试串口阻塞发送超时（ms）
 * @note  `bsp.c` 的 fputc / 强制打印路径经 `HAL_UART_Transmit` 使用本值。
 */
#define DEBUG_UART_TIMEOUT    1000U

/**
 * @brief 驱动自测总开关（见 `main.c`）
 * @note  0: 正常应用，调用 App_StateMachine_MainLoop()（永不返回）
 *        1: 进入自测死循环；IWDG 仍依赖 TIM2 更新中断内 HAL_IWDG_Refresh
 */
#define TEST_DRV              0

/**
 * @brief ADC 驱动自测开关（仅在 TEST_DRV=1 时生效）
 * @note  1: 自测循环内周期调用 Drv_ADC_TEST()
 *        0: 仅 Bsp_Ms_Delay(...)
 *        约束：TEST_ADC=1 必须同时 TEST_DRV=1（见 main.c 的 #error 校验）
 */
#define TEST_ADC             0

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */

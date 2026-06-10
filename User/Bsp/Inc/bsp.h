/**
 * @file    bsp.h
 * @brief   板级支持包（BSP）：横切系统能力对外接口
 * @details 模块路径：`User/Bsp`。位于 HAL/驱动之上、业务与服务之下，封装与板子/芯片强相关、
 *          但不宜散落在各服务中的能力：阻塞延时、系统 Tick、printf 重定向、硬件 CRC16-MODBUS。
 *          本头文件仅依赖标准 `<stdint.h>`，不暴露 `HAL_*` 类型，便于上层无 HAL 耦合。
 *
 *          CRC16：`Bsp_Crc16Modbus_Calc` 使用片内 CRC，须与 CubeMX `MX_CRC_Init` 配置一致
 *          （多项式 0x8005、初值 0xFFFF、字节/输出反相等，即标准 MODBUS CRC16）。
 *
 *          printf：实现在 `bsp.c` 的 `fputc`（本头已引入 `<stdio.h>`，包含后可直接 `printf`）。
 *          调试日志：统一用 `BSP_LOG_PRINTF`（见下方「2. 宏定义」），受 `config.h` 的 `DEBUG_PRINTF` 约束。
 */

#ifndef BSP_H
#define BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 1. 头文件依赖 ======================== */
#include <stdint.h>
#include <stdio.h>    /* BSP_LOG_PRINTF 展开为 printf */
#include "config.h"   /* 调试总开关 DEBUG_PRINTF 等用户开关统一在 Config 模块定义 */

/* ======================== 2. 宏定义（对外可见） ======================== */
/* 用户开关（含 DEBUG_PRINTF）已集中至 `User/Config/Inc/config.h` */

/**
 * @brief 调试日志宏：`DEBUG_PRINTF`=1 时展开为 printf；=0 时整条调用编译删除
 * @note  与直接调用 printf 的差别——关闭后连参数求值与格式化都不再生成，调用点零开销，
 *        适合放在主循环 / 中断等热路径。包含 `bsp.h` 即可使用。
 *        约定：致命兜底打印（如 `Error_Handler`）仍用 printf，不走本宏。
 */
#if DEBUG_PRINTF
#define BSP_LOG_PRINTF(...)   printf(__VA_ARGS__)
#else
#define BSP_LOG_PRINTF(...)   ((void)0)
#endif
/**
 * @brief 调试日志强制打印宏
 * @note  用于最初版本号和编译日期打印
 */
#define BSP_LOG_PRINTF_FORCE(...)   printf(__VA_ARGS__)


/* ======================== 3. 类型定义 ======================== */
/* 无 */

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */
/**
 * @brief BSP 板级初始化：各外设 MX 初始化，并按同步顺序启动 HRTIM
 * @note  调用前须已由 `main` 完成 `HAL_Init()` 与 `SystemClock_Config()`；
 *        须在进入主循环前调用一次
 */
void Bsp_Init(void);

/**
 * @brief 阻塞延时（毫秒级）
 * @param u32Ms 延时毫秒数
 */
void Bsp_Ms_Delay(uint32_t u32Ms);

/**
 * @brief 获取系统毫秒级 tick（自上电起累计）
 * @return 当前 tick（毫秒）
 */
uint32_t Bsp_TickMs_Get(void);

/**
 * @brief 硬件计算 CRC16-MODBUS（多项式 0xA001 反射形式、初值 0xFFFF）
 * @param pu8Data   待校验数据首地址
 * @param u16Length 数据长度（字节）
 * @return 参数非法或长度为 0 时返回 0；否则为 CRC16（MODBUS，低字节在先与帧尾一致）
 * @note 须在 `MX_CRC_Init()` 之后调用；内部对 CRC 外设做短临界区保护，勿在中断里长时间占用
 */
uint16_t Bsp_Crc16Modbus_Calc(const uint8_t *pu8Data, uint16_t u16Length);

#ifdef __cplusplus
}
#endif

#endif /* BSP_H */

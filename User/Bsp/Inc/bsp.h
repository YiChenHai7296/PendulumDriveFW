/**
 * @file    bsp.h
 * @brief   板级支持包（BSP）：横切系统能力对外接口
 * @details 模块路径：`User/Bsp`。位于 HAL/驱动之上、业务与服务之下，封装与板子/芯片强相关、
 *          但不宜散落在各服务中的能力：阻塞延时、系统 Tick、printf 重定向、硬件 CRC16-MODBUS。
 *          本头文件仅依赖标准 `<stdint.h>`，不暴露 `HAL_*` 类型，便于上层无 HAL 耦合。
 *
 *          CRC16：`Bsp_Crc16Modbus_Byte` 使用片内 CRC，须与 CubeMX `MX_CRC_Init` 配置一致
 *          （多项式 0x8005、初值 0xFFFF、字节/输出反相等，即标准 MODBUS CRC16）。
 *
 *          printf：实现在 `bsp.c` 的 `fputc`，无需在此声明；`#include <stdio.h>` 后可直接 `printf`。
 *          调试总开关：`DEBUG_PRINTF`（见下方「2. 宏定义」）。
 */

#ifndef BSP_H
#define BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 1. 头文件依赖 ======================== */
#include <stdint.h>

/* ======================== 2. 宏定义（对外可见） ======================== */
/**
 * @brief printf 调试输出总开关
 * @note  0: 关闭（fputc 内的串口发送被预处理器删除，printf 静默）
 *        1: 打开（printf 经 UART 输出）
 */
#define DEBUG_PRINTF          1

/* ======================== 3. 类型定义 ======================== */
/* 无 */

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */
/**
 * @brief 阻塞延时（毫秒级）
 * @param u32Ms 延时毫秒数
 */
void Bsp_DelayMs(uint32_t u32Ms);

/**
 * @brief 获取系统毫秒级 tick（自上电起累计）
 * @return 当前 tick（毫秒）
 */
uint32_t Bsp_GetTickMs(void);

/**
 * @brief 硬件计算 CRC16-MODBUS（多项式 0xA001 反射形式、初值 0xFFFF）
 * @param pu8Data   待校验数据首地址
 * @param u16Length 数据长度（字节）
 * @return 参数非法或长度为 0 时返回 0；否则为 CRC16（MODBUS，低字节在先与帧尾一致）
 * @note 须在 `MX_CRC_Init()` 之后调用；内部对 CRC 外设做短临界区保护，勿在中断里长时间占用
 */
uint16_t Bsp_Crc16Modbus_Byte(const uint8_t *pu8Data, uint16_t u16Length);

/**
 * @brief 启用 DWT 周期计数器（供编码器帧间隔微秒计时，须在 `SystemCoreClock` 已更新后调用）
 */
void Bsp_DwtInit(void);

/**
 * @brief 自上电起 DWT 周期计数换算的微秒时间戳（约 71 分钟回绕 @170MHz）
 */
uint32_t Bsp_DwtGetUs(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_H */

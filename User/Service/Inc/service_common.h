/**
 * @file    service_common.h
 * @brief   服务层公用头文件：跨模块共享的类型、宏及通用工具（如小端序整型打包）。
 * @note    各 service 模块在需要统一语义时优先包含本文件，避免重复定义。
 */

#ifndef SERVICE_COMMON_H
#define SERVICE_COMMON_H

/* ======================== 1. 头文件依赖 ======================== */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 2. 宏定义（对外可见） ======================== */

/** 校验是否为合法 Svc_FunctionalState_t */
#define IS_SVC_FUNCTIONAL_STATE(S) (((S) == SVC_DISABLE) || ((S) == SVC_ENABLE))

/* ======================== 3. 类型定义 ======================== */

/**
 * @brief 服务层统一「使能 / 失能」枚举
 * @note  取值约定与驱动层 Drv_FunctionalState_t（DRV_DISABLE=0、DRV_ENABLE=1）一致，便于对照转换。
 */
typedef enum
{
    SVC_DISABLE = 0U,
    SVC_ENABLE  = 1U
} Svc_FunctionalState_t;

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */

/* --- 小端序整型打包 / 解包（与具体上位机协议无关） --- */

/**
 * @brief 将 int16 以小端序写入缓冲区（连续 2 字节）
 */
void Svc_PackInt16LE(uint8_t *pBuf, int16_t value);

/**
 * @brief 将 int32 以小端序写入缓冲区（连续 4 字节）
 */
void Svc_PackInt32LE(uint8_t *pBuf, int32_t value);

/**
 * @brief 从缓冲区按小端序读取 int16
 */
int16_t Svc_UnpackInt16LE(const uint8_t *pBuf);

/**
 * @brief 从缓冲区按小端序读取 int32
 */
int32_t Svc_UnpackInt32LE(const uint8_t *pBuf);

/* --- CRC（编码器帧：多项式 x^8+x^2+x+1，初值 0x00） --- */

/**
 * @brief 计算编码器帧用 CRC8（逐字节异或后移位；与 CM/SA/AS 帧尾校验一致）
 * @param pData 参与校验的字节序列
 * @param length 字节数；pData 为空或 length 为 0 时返回 0
 */
uint8_t Svc_CalcCRC8(const uint8_t *pData, uint16_t length);

/* --- CRC（MODBUS：多项式 0xA001，初值 0xFFFF） --- */

/**
 * @brief 计算 MODBUS CRC16；帧尾通常按低字节在前存放
 * @param pData 参与校验的字节序列
 * @param length 字节数；pData 为空或 length 为 0 时返回 0
 */
uint16_t Svc_CalcCRC16Modbus(const uint8_t *pData, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* SERVICE_COMMON_H */

/**
 * @file service_common.c
 * @brief 服务层公共实现：与具体业务模块无关的通用工具
 */

/* ======================== 0. 头文件引用 ======================== */
#include "service_common.h"

/* ======================== 1. 私有宏定义 ======================== */
/* 无 */

/* ======================== 2. 私有类型定义 ======================== */
/* 无 */

/* ======================== 3. 私有变量 ======================== */
/* 无 */

/* ======================== 4. 对外变量定义 ======================== */
/* 无 */

/* ======================== 5. 私有函数声明 ======================== */
/* 无 */

/* ======================== 6. 接口函数实现 ======================== */



/**
 * @brief 将 int16 以小端序写入连续 2 字节
 * @param pBuf 输出缓冲区（至少 2 字节）
 * @param value 待写入的有符号 16 位值
 */
void Svc_PackInt16LE(uint8_t *pBuf, int16_t value)
{
    pBuf[0] = (uint8_t)((uint16_t)value & 0xFFU);
    pBuf[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
}

/**
 * @brief 将 int32 以小端序写入连续 4 字节
 * @param pBuf 输出缓冲区（至少 4 字节）
 * @param value 待写入的有符号 32 位值
 */
void Svc_PackInt32LE(uint8_t *pBuf, int32_t value)
{
    pBuf[0] = (uint8_t)((uint32_t)value & 0xFFU);
    pBuf[1] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
    pBuf[2] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
    pBuf[3] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
}

/**
 * @brief 从小端序 2 字节解析 int16
 * @param pBuf 输入缓冲区（至少 2 字节）
 * @return 解析得到的值
 */
int16_t Svc_UnpackInt16LE(const uint8_t *pBuf)
{
    return (int16_t)((uint16_t)pBuf[0] | ((uint16_t)pBuf[1] << 8));
}

/**
 * @brief 从小端序 4 字节解析 int32
 * @param pBuf 输入缓冲区（至少 4 字节）
 * @return 解析得到的值
 */
int32_t Svc_UnpackInt32LE(const uint8_t *pBuf)
{
    return (int32_t)((uint32_t)pBuf[0] | ((uint32_t)pBuf[1] << 8) |
                     ((uint32_t)pBuf[2] << 16) | ((uint32_t)pBuf[3] << 24));
}



/**
 * @brief 计算 CRC8 校验值（编码器协议）
 * @details 多项式：x^8 + x^2 + x + 1（对应移位后与 0x01 异或）
 * @param pData 数据缓冲区
 * @param length 数据长度
 * @return CRC8 校验值；pData 为空或 length 为 0 时返回 0
 */
uint8_t Svc_CalcCRC8(const uint8_t *pData, uint16_t length)
{
    uint8_t crc = 0x00U;
    uint16_t i;
    uint8_t bit;

    if (pData == NULL || length == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < length; i++)
    {
        crc ^= pData[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if (crc & 0x80U)
            {
                crc = (uint8_t)((crc << 1) ^ 0x01U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}



/**
 * @brief MODBUS CRC16（多项式 0xA001，初值 0xFFFF）
 * @param pData 参与计算的字节序列
 * @param length 字节数
 * @return CRC16；pData 为空或 length 为 0 时返回 0
 */
uint16_t Svc_CalcCRC16Modbus(const uint8_t *pData, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t j;

    if (pData == NULL || length == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)pData[i];
        for (j = 0U; j < 8U; j++)
        {
            if (crc & 1U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* ======================== 7. 私有函数实现 ======================== */
/* 无 */

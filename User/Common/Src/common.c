/**
 * @file    common.c
 * @brief   Common 实现：帧队列、小端转换、软件 CRC（CRC8 + CRC16 参考）
 * @details 仅 `#include "common.h"`，无板级依赖。CRC16 软件实现与 BSP 硬件 CRC 数学等价，当前协议路径未调用。
 */

/* ======================== 1. 头文件引用 ======================== */
#include "common.h"

/* ======================== 2. 私有宏定义 ======================== */
/* 无 */

/* ======================== 3. 私有类型定义 ======================== */
/* 无 */

/* ======================== 4. 私有变量 ======================== */
/* 无 */

/* ======================== 5. 对外变量定义 ======================== */
/* 无 */

/* ======================== 6. 私有函数声明 ======================== */
static void RFQ_Memcpy(uint8_t *pu8Dst, const uint8_t *pu8Src, uint16_t u16Len);

/* ======================== 7. 接口函数实现 ======================== */
/* -------- 7.1 帧级环形队列 -------- */
void Util_RFQ_Init(rfq_queue_t *struQueue)
{
    if (struQueue == NULL)
    {
        return;
    }
    struQueue->u8WriteIdx = 0;
    struQueue->u8ReadIdx  = 0;
}

int Util_RFQ_Push(rfq_queue_t *struQueue, const uint8_t *pu8Data, uint16_t u16Len)
{
    uint8_t u8NextW;

    if (struQueue == NULL || pu8Data == NULL)
    {
        return -1;
    }

    u8NextW = (uint8_t)(struQueue->u8WriteIdx + 1U);
    if (u8NextW >= RFQ_QUEUE_SIZE)
    {
        u8NextW = 0U;
    }
    if (u8NextW == struQueue->u8ReadIdx)
    {
        return -1;
    }

    if (u16Len > RFQ_FRAME_MAX_LEN)
    {
        u16Len = RFQ_FRAME_MAX_LEN;
    }

    {
        rfq_frame_t *struFrame = &struQueue->frames[struQueue->u8WriteIdx];
        RFQ_Memcpy(struFrame->au8Bytes, pu8Data, u16Len);
        struFrame->u16Len = u16Len;
    }

    struQueue->u8WriteIdx++;
    if (struQueue->u8WriteIdx >= RFQ_QUEUE_SIZE)
    {
        struQueue->u8WriteIdx = 0;
    }

    return 0;
}

int Util_RFQ_Pop(rfq_queue_t *struQueue, rfq_frame_t *struOut)
{
    if (struQueue == NULL || struOut == NULL)
    {
        return -1;
    }
    if (struQueue->u8ReadIdx == struQueue->u8WriteIdx)
    {
        return -1;
    }

    *struOut = struQueue->frames[struQueue->u8ReadIdx];

    struQueue->u8ReadIdx++;
    if (struQueue->u8ReadIdx >= RFQ_QUEUE_SIZE)
    {
        struQueue->u8ReadIdx = 0;
    }

    return 0;
}

uint8_t Util_RFQ_Count(const rfq_queue_t *struQueue)
{
    uint8_t u8W;
    uint8_t u8R;

    if (struQueue == NULL)
    {
        return 0U;
    }
    u8W = struQueue->u8WriteIdx;
    u8R = struQueue->u8ReadIdx;
    if (u8W >= u8R)
    {
        return (uint8_t)(u8W - u8R);
    }
    return (uint8_t)(RFQ_QUEUE_SIZE - (u8R - u8W));
}

uint8_t Util_RFQ_Is_Empty(const rfq_queue_t *struQueue)
{
    if (struQueue == NULL)
    {
        return 1U;
    }
    return (struQueue->u8WriteIdx == struQueue->u8ReadIdx) ? 1U : 0U;
}

uint8_t Util_RFQ_Is_Full(const rfq_queue_t *struQueue)
{
    uint8_t u8NextW;

    if (struQueue == NULL)
    {
        return 0U;
    }
    u8NextW = (uint8_t)(struQueue->u8WriteIdx + 1U);
    if (u8NextW >= RFQ_QUEUE_SIZE)
    {
        u8NextW = 0U;
    }
    return (u8NextW == struQueue->u8ReadIdx) ? 1U : 0U;
}

/* -------- 7.2 小端序整型打包 / 解包 -------- */
void Util_PackInt16LE(uint8_t *pu8Buf, int16_t s16Value)
{
    pu8Buf[0] = (uint8_t)((uint16_t)s16Value & 0xFFU);
    pu8Buf[1] = (uint8_t)(((uint16_t)s16Value >> 8) & 0xFFU);
}

void Util_PackInt32LE(uint8_t *pu8Buf, int32_t s32Value)
{
    pu8Buf[0] = (uint8_t)((uint32_t)s32Value & 0xFFU);
    pu8Buf[1] = (uint8_t)(((uint32_t)s32Value >> 8) & 0xFFU);
    pu8Buf[2] = (uint8_t)(((uint32_t)s32Value >> 16) & 0xFFU);
    pu8Buf[3] = (uint8_t)(((uint32_t)s32Value >> 24) & 0xFFU);
}

int16_t Util_UnpackInt16LE(const uint8_t *pu8Buf)
{
    return (int16_t)((uint16_t)pu8Buf[0] | ((uint16_t)pu8Buf[1] << 8));
}

int32_t Util_UnpackInt32LE(const uint8_t *pu8Buf)
{
    return (int32_t)((uint32_t)pu8Buf[0] | ((uint32_t)pu8Buf[1] << 8) |
                     ((uint32_t)pu8Buf[2] << 16) | ((uint32_t)pu8Buf[3] << 24));
}

/* -------- 7.3 软件 CRC（CRC8 + CRC16-MODBUS 参考实现） -------- */
uint8_t Util_CalcCRC8(const uint8_t *pu8Data, uint16_t u16Length)
{
    uint8_t u8Crc = 0x00U;
    uint16_t u16I;
    uint8_t u8Bit;

    if (pu8Data == NULL || u16Length == 0U)
    {
        return 0U;
    }

    for (u16I = 0U; u16I < u16Length; u16I++)
    {
        u8Crc ^= pu8Data[u16I];
        for (u8Bit = 0U; u8Bit < 8U; u8Bit++)
        {
            if (u8Crc & 0x80U)
            {
                u8Crc = (uint8_t)((u8Crc << 1) ^ 0x01U);
            }
            else
            {
                u8Crc <<= 1;
            }
        }
    }
    return u8Crc;
}

uint16_t Util_CalcCRC16Modbus(const uint8_t *pu8Data, uint16_t u16Length)
{
    uint16_t u16Crc = 0xFFFFU;
    uint16_t u16I;
    uint8_t u8J;

    if (pu8Data == NULL || u16Length == 0U)
    {
        return 0U;
    }

    for (u16I = 0U; u16I < u16Length; u16I++)
    {
        u16Crc ^= (uint16_t)pu8Data[u16I];
        for (u8J = 0U; u8J < 8U; u8J++)
        {
            if (u16Crc & 1U)
            {
                u16Crc = (uint16_t)((u16Crc >> 1) ^ 0xA001U);
            }
            else
            {
                u16Crc >>= 1;
            }
        }
    }

    return u16Crc;
}

/* ======================== 8. 私有函数实现 ======================== */
static void RFQ_Memcpy(uint8_t *pu8Dst, const uint8_t *pu8Src, uint16_t u16Len)
{
    while (u16Len--)
    {
        *pu8Dst++ = *pu8Src++;
    }
}

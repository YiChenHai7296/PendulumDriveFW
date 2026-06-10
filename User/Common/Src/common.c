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

/* ======================== 4. 对外变量定义 ======================== */
/* 无 */

/* ======================== 5. 私有变量 ======================== */
/* 无 */

/* ======================== 6. 私有函数声明 ======================== */
static void RFQ_Buf_Copy(uint8_t *pu8Dst, const uint8_t *pu8Src, uint16_t u16Len);

/* ======================== 7. 接口函数实现 ======================== */
/* -------- 7.1 帧级环形队列 -------- */
/**
 * @brief 初始化帧级环形队列（读写索引清零）
 * @param struQueue 队列对象；为 NULL 时直接返回
 */
void Cmn_RFQ_Init(RfqQueue_t *struQueue)
{
    if (struQueue == NULL)
    {
        return;
    }
    struQueue->u8WriteIdx = 0;
    struQueue->u8ReadIdx  = 0;
}

/**
 * @brief 入队一帧（单生产者）：拷贝数据并推进写索引
 * @param struQueue 队列对象
 * @param pu8Data   待入队数据首地址
 * @param u16Len    数据长度；超过 RFQ_FRAME_MAX_LEN 时被截断
 * @return STATUS_OK 成功；STATUS_ERROR 参数非法或队列已满
 */
Status_t Cmn_RFQ_Push(RfqQueue_t *struQueue, const uint8_t *pu8Data, uint16_t u16Len)
{
    uint8_t u8NextW;

    if (struQueue == NULL || pu8Data == NULL)
    {
        return STATUS_ERROR;
    }

    u8NextW = (uint8_t)(struQueue->u8WriteIdx + 1U);
    if (u8NextW >= RFQ_QUEUE_SIZE)
    {
        u8NextW = 0U;
    }
    if (u8NextW == struQueue->u8ReadIdx)
    {
        return STATUS_ERROR;
    }

    if (u16Len > RFQ_FRAME_MAX_LEN)
    {
        u16Len = RFQ_FRAME_MAX_LEN;
    }

    {
        RfqFrame_t *struFrame = &struQueue->frames[struQueue->u8WriteIdx];
        RFQ_Buf_Copy(struFrame->au8Bytes, pu8Data, u16Len);
        struFrame->u16Len = u16Len;
    }

    struQueue->u8WriteIdx++;
    if (struQueue->u8WriteIdx >= RFQ_QUEUE_SIZE)
    {
        struQueue->u8WriteIdx = 0;
    }

    return STATUS_OK;
}

/**
 * @brief 出队一帧（单消费者）：拷贝到输出并推进读索引
 * @param struQueue 队列对象
 * @param struOut   输出的整帧
 * @return STATUS_OK 成功；STATUS_ERROR 参数非法或队列为空
 */
Status_t Cmn_RFQ_Pop(RfqQueue_t *struQueue, RfqFrame_t *struOut)
{
    if (struQueue == NULL || struOut == NULL)
    {
        return STATUS_ERROR;
    }
    if (struQueue->u8ReadIdx == struQueue->u8WriteIdx)
    {
        return STATUS_ERROR;
    }

    *struOut = struQueue->frames[struQueue->u8ReadIdx];

    struQueue->u8ReadIdx++;
    if (struQueue->u8ReadIdx >= RFQ_QUEUE_SIZE)
    {
        struQueue->u8ReadIdx = 0;
    }

    return STATUS_OK;
}

/**
 * @brief 获取队列当前帧数量
 * @param struQueue 队列对象
 * @return 已入队未消费的帧数；struQueue 为 NULL 时返回 0
 */
uint8_t Cmn_RFQ_Count(const RfqQueue_t *struQueue)
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

/**
 * @brief 判断队列是否为空
 * @param struQueue 队列对象
 * @return 1 空（含 struQueue 为 NULL）；0 非空
 */
uint8_t Cmn_RFQ_Empty_Is(const RfqQueue_t *struQueue)
{
    if (struQueue == NULL)
    {
        return 1U;
    }
    return (struQueue->u8WriteIdx == struQueue->u8ReadIdx) ? 1U : 0U;
}

/**
 * @brief 判断队列是否已满
 * @param struQueue 队列对象
 * @return 1 满；0 未满（含 struQueue 为 NULL）
 */
uint8_t Cmn_RFQ_Full_Is(const RfqQueue_t *struQueue)
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
/**
 * @brief 将 int16 以小端序写入缓冲区（连续 2 字节）
 * @param pu8Buf   目标缓冲区（至少 2 字节）
 * @param s16Value 待写入值
 */
void Cmn_Int16LE_Pack(uint8_t *pu8Buf, int16_t s16Value)
{
    pu8Buf[0] = (uint8_t)((uint16_t)s16Value & 0xFFU);
    pu8Buf[1] = (uint8_t)(((uint16_t)s16Value >> 8) & 0xFFU);
}

/**
 * @brief 将 int32 以小端序写入缓冲区（连续 4 字节）
 * @param pu8Buf   目标缓冲区（至少 4 字节）
 * @param s32Value 待写入值
 */
void Cmn_Int32LE_Pack(uint8_t *pu8Buf, int32_t s32Value)
{
    pu8Buf[0] = (uint8_t)((uint32_t)s32Value & 0xFFU);
    pu8Buf[1] = (uint8_t)(((uint32_t)s32Value >> 8) & 0xFFU);
    pu8Buf[2] = (uint8_t)(((uint32_t)s32Value >> 16) & 0xFFU);
    pu8Buf[3] = (uint8_t)(((uint32_t)s32Value >> 24) & 0xFFU);
}

/**
 * @brief 从缓冲区按小端序读取 int16
 * @param pu8Buf 源缓冲区（至少 2 字节）
 * @return 解析得到的 int16 值
 */
int16_t Cmn_Int16LE_Unpack(const uint8_t *pu8Buf)
{
    return (int16_t)((uint16_t)pu8Buf[0] | ((uint16_t)pu8Buf[1] << 8));
}

/**
 * @brief 从缓冲区按小端序读取 int32
 * @param pu8Buf 源缓冲区（至少 4 字节）
 * @return 解析得到的 int32 值
 */
int32_t Cmn_Int32LE_Unpack(const uint8_t *pu8Buf)
{
    return (int32_t)((uint32_t)pu8Buf[0] | ((uint32_t)pu8Buf[1] << 8) |
                     ((uint32_t)pu8Buf[2] << 16) | ((uint32_t)pu8Buf[3] << 24));
}

/* -------- 7.3 软件 CRC（CRC8 + CRC16-MODBUS 参考实现） -------- */
/**
 * @brief 计算 CRC8（多项式 0x01 即 x^8+1，初值 0x00，MSB 优先）
 * @param pu8Data   待校验数据首地址
 * @param u16Length 数据长度（字节）
 * @return CRC8 校验值；参数非法或长度为 0 时返回 0
 * @note  与编码器 CM/SA/AS 帧尾校验一致
 */
uint8_t Cmn_CRC8_Calc(const uint8_t *pu8Data, uint16_t u16Length)
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

/**
 * @brief 计算 MODBUS CRC16（多项式 0xA001，初值 0xFFFF）软件参考实现
 * @param pu8Data   待校验数据首地址
 * @param u16Length 数据长度（字节）
 * @return CRC16 校验值；参数非法或长度为 0 时返回 0
 * @note  与 BSP 硬件 Bsp_Crc16Modbus_Calc 等价，可作对照或离线验证
 */
uint16_t Cmn_CRC16Modbus_Calc(const uint8_t *pu8Data, uint16_t u16Length)
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
/**
 * @brief 字节拷贝（本文件内），避免引入 string.h 依赖
 * @param pu8Dst 目标地址
 * @param pu8Src 源地址
 * @param u16Len 拷贝字节数
 */
static void RFQ_Buf_Copy(uint8_t *pu8Dst, const uint8_t *pu8Src, uint16_t u16Len)
{
    while (u16Len--)
    {
        *pu8Dst++ = *pu8Src++;
    }
}

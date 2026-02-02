#include "encoder_protocol.h"

/* 固定帧长度：CM(1) + SA(1) + AS0~AS2(3) + CRC8(1) */
#define ENCODER_FRAME_LENGTH_BYTES   6U

/* 控制域固定值 0x02 */
#define ENCODER_CM_VALUE             0x02U

/* SA 状态位定义 */
#define ENCODER_SA_COUNT_ERROR_BIT   (1U << 4)
#define ENCODER_SA_MT_BATT_ERR_BIT   (1U << 5)

/* 绝对位置有效位掩码：21 bit */
#define ENCODER_ABS_POSITION_MASK    0x001FFFFFU

/* 内部 CRC8 计算：G(x) = x^8 + 1，多项式系数 0x01，MSB 优先 */
static uint8_t EncoderProtocol_CalcCRC8(const uint8_t *pData, uint16_t length)
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

/* 编码器原始 6 字节数据获取函数，由底层实现，业务层不可见 */
extern const uint8_t *EncoderProtocol_GetRawFrame(void);

/* 内部帧解析函数 */
static EncoderProtocolResult_t EncoderProtocol_ParseFrame(const uint8_t *pFrame,
                                                          uint16_t length,
                                                          EncoderProtocolData_t *pOut)
{
    uint8_t cm;
    uint8_t sa;
    uint8_t as0, as1, as2;
    uint32_t raw_position;
    uint8_t crc_calc;
    uint8_t crc_recv;

    if ((pFrame == NULL) || (pOut == NULL))
    {
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    if (length < ENCODER_FRAME_LENGTH_BYTES)
    {
        return ENCODER_PROTOCOL_ERR_LENGTH;
    }

    cm = pFrame[0];
    sa = pFrame[1];
    as0 = pFrame[2];
    as1 = pFrame[3];
    as2 = pFrame[4];
    crc_recv = pFrame[5];

    /* 检查控制域固定值 */
    if (cm != ENCODER_CM_VALUE)
    {
        return ENCODER_PROTOCOL_ERR_LENGTH;
    }

    /* 计算 CRC8，参与计算的字节为 CM/SA/AS0/AS1/AS2 共 5 字节 */
    crc_calc = EncoderProtocol_CalcCRC8(pFrame, 5U);
    if (crc_calc != crc_recv)
    {
        return ENCODER_PROTOCOL_ERR_CRC;
    }

    /* 提取状态位 */
    pOut->count_error      = ((sa & ENCODER_SA_COUNT_ERROR_BIT) != 0U);
    pOut->mt_or_batt_error = ((sa & ENCODER_SA_MT_BATT_ERR_BIT) != 0U);

    /* 绝对位置数据：AS0 为最低字节，AS2 为最高字节，高 3 位为 0，仅 21 位有效 */
    raw_position = (uint32_t)as0 |
                   ((uint32_t)as1 << 8) |
                   ((uint32_t)as2 << 16);
    pOut->absolute_position = raw_position & ENCODER_ABS_POSITION_MASK;

    return ENCODER_PROTOCOL_OK;
}

EncoderProtocolResult_t EncoderProtocol_Read(EncoderProtocolData_t *pOut)
{
    const uint8_t *pFrame;

    if (pOut == NULL)
    {
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    /* 从底层获取一帧原始 6 字节编码器数据 */
    //pFrame = EncoderProtocol_GetRawFrame();
    if (pFrame == NULL)
    {
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    return EncoderProtocol_ParseFrame(pFrame, ENCODER_FRAME_LENGTH_BYTES, pOut);
}


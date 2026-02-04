/**
 * @file simulink_protocol.c
 * @brief Simulink 上位机通信协议栈实现，用于组帧与解帧，支持控制帧与反馈帧
 */

#include "simulink_protocol.h"

/* ===================== 帧格式常量（内部） ===================== */
#define SIMULINK_HEAD_BYTE0        0x5AU
#define SIMULINK_HEAD_BYTE1        0xA5U
#define SIMULINK_HEAD_SIZE         2U
#define SIMULINK_TYPE_CONTROL      0x01U
#define SIMULINK_TYPE_FEEDBACK     0x02U
#define SIMULINK_LEN_CONTROL       0x04U
#define SIMULINK_LEN_FEEDBACK      0x1AU
#define SIMULINK_CONTROL_FRAME_LEN (SIMULINK_HEAD_SIZE + 1U + 1U + SIMULINK_LEN_CONTROL + 2U)
#define SIMULINK_FEEDBACK_FRAME_LEN (SIMULINK_HEAD_SIZE + 1U + 1U + SIMULINK_LEN_FEEDBACK + 2U)

/* 参数范围（内部校验用） */
#define SIMULINK_PWM_MIN           (-10000)
#define SIMULINK_PWM_MAX           10000
#define SIMULINK_CURRENT_MIN       (-2000)
#define SIMULINK_CURRENT_MAX       2000
#define SIMULINK_MOTOR_CURRENT_MIN (-2000)
#define SIMULINK_MOTOR_CURRENT_MAX 2000
#define SIMULINK_MOTOR_POSITION_MIN 0
#define SIMULINK_MOTOR_POSITION_MAX 2097151
#define SIMULINK_MOTOR_SPEED_MIN   (-40000)
#define SIMULINK_MOTOR_SPEED_MAX   40000
#define SIMULINK_AXIS_POSITION_MIN 0
#define SIMULINK_AXIS_POSITION_MAX 1048575   /* 20 位：2^20 - 1 */
#define SIMULINK_AXIS_SPEED_MIN    (-4000)
#define SIMULINK_AXIS_SPEED_MAX    4000
#define SIMULINK_PENDULUM_POSITION_MIN 0
#define SIMULINK_PENDULUM_POSITION_MAX 131072
#define SIMULINK_PENDULUM_SPEED_MIN (-4000)
#define SIMULINK_PENDULUM_SPEED_MAX 4000




/**
 * @brief 校验帧头是否为 0x5A 0xA5
 * @param[in] pBuffer 缓冲区指针
 * @return true 帧头正确，false 否则
 */
bool SimulinkProtocol_VerifyHeader(const uint8_t *pBuffer);

/**
 * @brief 在缓冲区中查找帧头位置
 * @param[in] pBuffer    缓冲区
 * @param[in] bufferSize 长度
 * @return 帧头起始下标，未找到返回 -1
 */
int16_t SimulinkProtocol_FindHeader(const uint8_t *pBuffer, uint16_t bufferSize);


/* ===================== 内部小端打包/解包 ===================== */

/**
 * @brief 将 16 位整数按小端序写入缓冲区（低字节在前）
 */
static void SimulinkProtocol_PackInt16LE(uint8_t *pBuf, int16_t value)
{
    pBuf[0] = (uint8_t)((uint16_t)value & 0xFFU);
    pBuf[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
}

/**
 * @brief 将 32 位整数按小端序写入缓冲区（低字节在前）
 */
static void SimulinkProtocol_PackInt32LE(uint8_t *pBuf, int32_t value)
{
    pBuf[0] = (uint8_t)((uint32_t)value & 0xFFU);
    pBuf[1] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
    pBuf[2] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
    pBuf[3] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
}

/**
 * @brief 从缓冲区按小端序解析出 16 位整数
 */
static int16_t SimulinkProtocol_UnpackInt16LE(const uint8_t *pBuf)
{
    return (int16_t)((uint16_t)pBuf[0] | ((uint16_t)pBuf[1] << 8));
}

/**
 * @brief 从缓冲区按小端序解析出 32 位整数
 */
static int32_t SimulinkProtocol_UnpackInt32LE(const uint8_t *pBuf)
{
    return (int32_t)((uint32_t)pBuf[0] | ((uint32_t)pBuf[1] << 8) |
                     ((uint32_t)pBuf[2] << 16) | ((uint32_t)pBuf[3] << 24));
}

/* ===================== 内部 CRC16（MODBUS：多项式 0xA001，初值 0xFFFF，小端输出） ===================== */

/**
 * @brief 计算 CRC16 校验值
 */
static uint16_t SimulinkProtocol_CalcCRC16(const uint8_t *pData, uint16_t length)
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

/* ===================== 内部：解析控制帧 ===================== */

/**
 * @brief 从原始字节流解析控制帧并校验 CRC、范围
 */
static SimulinkProtocolResult_t SimulinkProtocol_ParseControlFrame(const uint8_t *pFrame,
                                                                   uint16_t length,
                                                                   SimulinkProtocolControlData_t *pOut)
{
    uint16_t offset = 0U;
    uint16_t crc_calc;
    uint16_t crc_recv;

    if ((pFrame == NULL) || (pOut == NULL))
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    if (length < SIMULINK_CONTROL_FRAME_LEN)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    if (pFrame[offset++] != SIMULINK_HEAD_BYTE0 || pFrame[offset++] != SIMULINK_HEAD_BYTE1)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    if (pFrame[offset++] != SIMULINK_TYPE_CONTROL)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    if (pFrame[offset++] != SIMULINK_LEN_CONTROL)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    pOut->pwm    = SimulinkProtocol_UnpackInt16LE(&pFrame[offset]);
    offset += 2U;
    pOut->current = SimulinkProtocol_UnpackInt16LE(&pFrame[offset]);
    offset += 2U;

    crc_recv  = (uint16_t)pFrame[offset] | ((uint16_t)pFrame[offset + 1] << 8);
    crc_calc  = SimulinkProtocol_CalcCRC16(pFrame, offset);
    if (crc_calc != crc_recv)
    {
        return SIMULINK_PROTOCOL_ERR_CRC;
    }

    if (pOut->pwm < SIMULINK_PWM_MIN || pOut->pwm > SIMULINK_PWM_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (pOut->current < SIMULINK_CURRENT_MIN || pOut->current > SIMULINK_CURRENT_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }

    return SIMULINK_PROTOCOL_OK;
}

/* ===================== 内部：组反馈帧 ===================== */

/**
 * @brief 将反馈数据结构按协议格式组帧并写入缓冲区，含 CRC16
 */
static SimulinkProtocolResult_t SimulinkProtocol_AssembleFeedbackFrame(const SimulinkProtocolFeedbackData_t *pIn,
                                                                       uint8_t *pBuf,
                                                                       uint16_t bufSize,
                                                                       uint16_t *pOutLen)
{
    uint16_t offset = 0U;
    uint16_t crc16;

    if ((pIn == NULL) || (pBuf == NULL))
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    if (bufSize < SIMULINK_FEEDBACK_FRAME_LEN)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    pBuf[offset++] = SIMULINK_HEAD_BYTE0;
    pBuf[offset++] = SIMULINK_HEAD_BYTE1;
    pBuf[offset++] = SIMULINK_TYPE_FEEDBACK;
    pBuf[offset++] = SIMULINK_LEN_FEEDBACK;

    SimulinkProtocol_PackInt16LE(&pBuf[offset], pIn->motor_current);
    offset += 2U;
    SimulinkProtocol_PackInt32LE(&pBuf[offset], pIn->motor_position);
    offset += 4U;
    SimulinkProtocol_PackInt32LE(&pBuf[offset], pIn->motor_speed);
    offset += 4U;
    SimulinkProtocol_PackInt32LE(&pBuf[offset], pIn->axis_position);
    offset += 4U;
    SimulinkProtocol_PackInt32LE(&pBuf[offset], pIn->axis_speed);
    offset += 4U;
    SimulinkProtocol_PackInt32LE(&pBuf[offset], pIn->pendulum_position);
    offset += 4U;
    SimulinkProtocol_PackInt32LE(&pBuf[offset], pIn->pendulum_speed);
    offset += 4U;

    crc16 = SimulinkProtocol_CalcCRC16(pBuf, offset);
    pBuf[offset++] = (uint8_t)(crc16 & 0xFFU);
    pBuf[offset++] = (uint8_t)((crc16 >> 8) & 0xFFU);

    if (pOutLen != NULL)
    {
        *pOutLen = offset;
    }

    return SIMULINK_PROTOCOL_OK;
}

/* ===================== 对外接口 ===================== */

static uint8_t s_feedback_tx_buf[SIMULINK_PROTOCOL_FEEDBACK_FRAME_SIZE];

uint8_t *SimulinkProtocol_GetFeedbackTxBuffer(void)
{
    return s_feedback_tx_buf;
}

uint16_t SimulinkProtocol_GetFeedbackTxLength(void)
{
    return (uint16_t)SIMULINK_PROTOCOL_FEEDBACK_FRAME_SIZE;
}

/* 读取路径：从 USART2 接收队列取一帧，再解析为控制数据 */
SimulinkProtocolResult_t SimulinkProtocol_UnpackControl(SimulinkProtocolControlData_t *pOut)
{
    uint8_t rxBuf[SIMULINK_CONTROL_FRAME_LEN];
    uint16_t len = 0U;

    if (pOut == NULL)
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    /* 从底层 USART2 接收队列出队一帧 */
    if (USART2_GetRxData(rxBuf, sizeof(rxBuf), &len) != 0)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;  /* 队列空或无完整帧 */
    }

    if (len != SIMULINK_CONTROL_FRAME_LEN)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    return SimulinkProtocol_ParseControlFrame(rxBuf, len, pOut);
}

SimulinkProtocolResult_t SimulinkProtocol_PackFeedback(const SimulinkProtocolFeedbackData_t *pIn)
{
    uint8_t *pBuffer;
    SimulinkProtocolResult_t res;

    if (pIn == NULL)
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    if (pIn->motor_current < SIMULINK_MOTOR_CURRENT_MIN || pIn->motor_current > SIMULINK_MOTOR_CURRENT_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (pIn->motor_position < SIMULINK_MOTOR_POSITION_MIN || pIn->motor_position > SIMULINK_MOTOR_POSITION_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (pIn->motor_speed < SIMULINK_MOTOR_SPEED_MIN || pIn->motor_speed > SIMULINK_MOTOR_SPEED_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (pIn->axis_position < SIMULINK_AXIS_POSITION_MIN || pIn->axis_position > SIMULINK_AXIS_POSITION_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (pIn->axis_speed < SIMULINK_AXIS_SPEED_MIN || pIn->axis_speed > SIMULINK_AXIS_SPEED_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (pIn->pendulum_position < SIMULINK_PENDULUM_POSITION_MIN || pIn->pendulum_position > SIMULINK_PENDULUM_POSITION_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (pIn->pendulum_speed < SIMULINK_PENDULUM_SPEED_MIN || pIn->pendulum_speed > SIMULINK_PENDULUM_SPEED_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }

    pBuffer = SimulinkProtocol_GetFeedbackTxBuffer();
    if (pBuffer == NULL)
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    /* 先在内部缓冲区中组帧 */
    res = SimulinkProtocol_AssembleFeedbackFrame(pIn, pBuffer, SIMULINK_FEEDBACK_FRAME_LEN, NULL);
    if (res != SIMULINK_PROTOCOL_OK)
    {
        return res;
    }

    /* 通过 USART2 + DMA 发送反馈帧 */
    if (USART2_SendData_DMA(pBuffer, SIMULINK_FEEDBACK_FRAME_LEN) != 0)
    {
        return SIMULINK_PROTOCOL_ERR_SEND;
    }

    return SIMULINK_PROTOCOL_OK;
}

bool SimulinkProtocol_VerifyHeader(const uint8_t *pBuffer)
{
    if (pBuffer == NULL)
    {
        return false;
    }
    return (pBuffer[0] == SIMULINK_HEAD_BYTE0 && pBuffer[1] == SIMULINK_HEAD_BYTE1);
}

int16_t SimulinkProtocol_FindHeader(const uint8_t *pBuffer, uint16_t bufferSize)
{
    uint16_t i;

    if (pBuffer == NULL || bufferSize < SIMULINK_HEAD_SIZE)
    {
        return -1;
    }

    for (i = 0U; i <= bufferSize - SIMULINK_HEAD_SIZE; i++)
    {
        if (pBuffer[i] == SIMULINK_HEAD_BYTE0 && pBuffer[i + 1] == SIMULINK_HEAD_BYTE1)
        {
            return (int16_t)i;
        }
    }

    return -1;
}

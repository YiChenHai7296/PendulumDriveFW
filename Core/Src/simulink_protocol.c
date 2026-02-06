/**
 * @file simulink_protocol.c
 * @brief Simulink 上位机通信协议栈实现：组帧与解帧，支持控制帧与反馈帧
 */

#include "simulink_protocol.h"

/* ======================== 1. 私有宏定义 ======================== */
/* 帧格式 */
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

/* ======================== 2. 私有类型定义 ======================== */
/* 无 */

/* ======================== 3. 私有变量 ======================== */
static uint8_t s_feedback_tx_buf[SIMULINK_PROTOCOL_FEEDBACK_FRAME_SIZE];  /* 反馈帧发送缓冲区 */

/* ======================== 4. 对外变量定义 ======================== */
/* 无 */

/* ======================== 5. 私有函数声明 ======================== */
static void SimulinkProtocol_PackInt16LE(uint8_t *pBuf, int16_t value);
static void SimulinkProtocol_PackInt32LE(uint8_t *pBuf, int32_t value);
static int16_t SimulinkProtocol_UnpackInt16LE(const uint8_t *pBuf);
static int32_t SimulinkProtocol_UnpackInt32LE(const uint8_t *pBuf);
static uint16_t SimulinkProtocol_CalcCRC16(const uint8_t *pData, uint16_t length);
static SimulinkProtocolResult_t SimulinkProtocol_ParseControlFrame(const uint8_t *pFrame,
                                                                   uint16_t length,
                                                                   SimulinkProtocolControlData_t *pOut);
static SimulinkProtocolResult_t SimulinkProtocol_AssembleFeedbackFrame(const SimulinkProtocolFeedbackData_t *pIn,
                                                                       uint8_t *pBuf,
                                                                       uint16_t bufSize,
                                                                       uint16_t *pOutLen);

/* ======================== 6. 接口函数实现 ======================== */

uint8_t *SimulinkProtocol_GetFeedbackTxBuffer(void)
{
    return s_feedback_tx_buf;
}

uint16_t SimulinkProtocol_GetFeedbackTxLength(void)
{
    return (uint16_t)SIMULINK_PROTOCOL_FEEDBACK_FRAME_SIZE;
}

SimulinkProtocolResult_t SimulinkProtocol_UnpackControl(SimulinkProtocolControlData_t *pOut)
{
    uint8_t rxBuf[SIMULINK_CONTROL_FRAME_LEN];  /* 控制帧原始字节缓冲 */
    uint16_t len = 0U;                          /* 实际接收长度 */

    if (pOut == NULL)
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    if (Simulink_ControlFrame_GetData(rxBuf, sizeof(rxBuf), &len) != 0)
    {
       // printf("data err \n");
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }
#if 1
    if (len != SIMULINK_CONTROL_FRAME_LEN)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }
#endif
    return SimulinkProtocol_ParseControlFrame(rxBuf, len, pOut);
}

SimulinkProtocolResult_t SimulinkProtocol_PackFeedback(const SimulinkProtocolFeedbackData_t *pIn)
{
    uint8_t *pBuffer;               /* 反馈帧发送缓冲指针 */
    SimulinkProtocolResult_t res;   /* 组帧结果 */

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

    res = SimulinkProtocol_AssembleFeedbackFrame(pIn, pBuffer, SIMULINK_FEEDBACK_FRAME_LEN, NULL);
    if (res != SIMULINK_PROTOCOL_OK)
    {
        return res;
    }

    if (Simulink_Feedback_Send(pBuffer, SIMULINK_FEEDBACK_FRAME_LEN) != 0)
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
    uint16_t i;  /* 遍历下标 */

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

/* ======================== 7. 私有函数实现 ======================== */

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

/**
 * @brief 计算 CRC16 校验值（MODBUS：多项式 0xA001，初值 0xFFFF，小端输出）
 */
static uint16_t SimulinkProtocol_CalcCRC16(const uint8_t *pData, uint16_t length)
{
    uint16_t crc = 0xFFFFU;  /* CRC 初值（MODBUS） */
    uint16_t i;              /* 字节循环下标 */
    uint8_t j;               /* 位循环下标 */

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

/**
 * @brief 从原始字节流解析控制帧并校验 CRC、范围
 */
static SimulinkProtocolResult_t SimulinkProtocol_ParseControlFrame(const uint8_t *pFrame,
                                                                   uint16_t length,
                                                                   SimulinkProtocolControlData_t *pOut)
{
    uint16_t offset = 0U;   /* 帧内解析偏移 */
    uint16_t crc_calc;      /* 计算得到的 CRC */
    uint16_t crc_recv;      /* 帧内接收的 CRC */

    if ((pFrame == NULL) || (pOut == NULL))
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    if (length < SIMULINK_CONTROL_FRAME_LEN)
    {
        printf("指令长度非法！\n");
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    if (pFrame[offset++] != SIMULINK_HEAD_BYTE0 || pFrame[offset++] != SIMULINK_HEAD_BYTE1)
    {
        printf("指令帧头错误！\n");
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    if (pFrame[offset++] != SIMULINK_TYPE_CONTROL)
    {
        printf("指令类型错误！\n");
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
		#if 1
    if (crc_calc != crc_recv)
    {
        printf("crc校验有误，接收值 %x , 计算值 %x \n",crc_recv,crc_calc);
        return SIMULINK_PROTOCOL_ERR_CRC;
    }
#endif
    if (pOut->pwm < SIMULINK_PWM_MIN || pOut->pwm > SIMULINK_PWM_MAX)
    {
        printf("pwm参数范围有误： %x  \n",pOut->pwm);
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
		#if 1
    if (pOut->current < SIMULINK_CURRENT_MIN || pOut->current > SIMULINK_CURRENT_MAX)
    {
        printf("pwm参数范围有误： %x  \n",pOut->current);
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
#endif
    return SIMULINK_PROTOCOL_OK;
}

/**
 * @brief 将反馈数据结构按协议格式组帧并写入缓冲区，含 CRC16
 */
static SimulinkProtocolResult_t SimulinkProtocol_AssembleFeedbackFrame(const SimulinkProtocolFeedbackData_t *pIn,
                                                                       uint8_t *pBuf,
                                                                       uint16_t bufSize,
                                                                       uint16_t *pOutLen)
{
    uint16_t offset = 0U;  /* 帧内组帧偏移 */
    uint16_t crc16;        /* 帧 CRC16 校验值 */

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

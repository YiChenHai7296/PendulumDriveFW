/**
 * @file simulink_protocol.c
 * @brief Simulink 上位机通信协议栈实现：组帧与解帧，支持控制帧与反馈帧
 */

#include "simulink_protocol.h"
#include "service_common.h"

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
static SimulinkProtocolResult_t Svc_SimulinkProtocol_ParseControlFrame(const uint8_t *pFrame,
                                                                   uint16_t length,
                                                                   SimulinkProtocolControlData_t *pOut);
static SimulinkProtocolResult_t Svc_SimulinkProtocol_AssembleFeedbackFrame(const SimulinkProtocolFeedbackData_t *pIn,
                                                                       uint8_t *pBuf,
                                                                       uint16_t bufSize,
                                                                       uint16_t *pOutLen);

/* ======================== 6. 对外接口：解包控制 / 发布反馈 ======================== */
/**
 * @brief 从 USART 控制通道取一帧并解析为控制量（PWM、电流设定）
 * @details 调用底层 Drv_Simulink_ControlFrame_GetData，长度须为 SIMULINK_CONTROL_FRAME_LEN，再解帧与 CRC 校验
 * @param pOut 输出：解析后的控制数据
 * @return SIMULINK_PROTOCOL_OK 或各类错误码（空指针、长度、CRC、范围等）
 */
SimulinkProtocolResult_t Svc_SimulinkProtocol_UnpackControl(SimulinkProtocolControlData_t *pOut)
{
    uint8_t rxBuf[SIMULINK_CONTROL_FRAME_LEN];  /* 控制帧原始字节缓冲 */
    uint16_t len = 0U;                          /* 实际接收长度 */

    if (pOut == NULL)
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    if (Drv_Simulink_ControlFrame_GetData(rxBuf, sizeof(rxBuf), &len) != DRV_OK)
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
    return Svc_SimulinkProtocol_ParseControlFrame(rxBuf, len, pOut);
}

/**
 * @brief 校验反馈载荷范围、按协议组帧并通过 USART 发送反馈帧
 * @param pIn 待上报的反馈数据（字段范围见宏 SIMULINK_*_MIN/MAX）
 * @return SIMULINK_PROTOCOL_OK；否则为范围错误、组帧失败或底层发送失败
 */
SimulinkProtocolResult_t Svc_SimulinkProtocol_PublishFeedback(const SimulinkProtocolFeedbackData_t *pIn)
{
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

    res = Svc_SimulinkProtocol_AssembleFeedbackFrame(pIn, s_feedback_tx_buf, SIMULINK_FEEDBACK_FRAME_LEN, NULL);
    if (res != SIMULINK_PROTOCOL_OK)
    {
        return res;
    }

    if (Drv_Simulink_Feedback_Send(s_feedback_tx_buf, SIMULINK_FEEDBACK_FRAME_LEN) != DRV_OK)
    {
        return SIMULINK_PROTOCOL_ERR_SEND;
    }

    return SIMULINK_PROTOCOL_OK;
}

/* ======================== 7. 私有函数实现 ======================== */
/**
 * @brief 从原始字节流解析一帧 Simulink 控制指令并校验 CRC 与字段范围
 * @param pFrame 完整控制帧缓冲区
 * @param length 实际长度，须不小于 SIMULINK_CONTROL_FRAME_LEN
 * @param pOut 输出：PWM（permille）、电流设定等
 * @return SIMULINK_PROTOCOL_OK；否则为空指针、长度、类型、CRC、范围错误
 */
static SimulinkProtocolResult_t Svc_SimulinkProtocol_ParseControlFrame(const uint8_t *pFrame,
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

    pOut->pwm    = Svc_UnpackInt16LE(&pFrame[offset]);
    offset += 2U;
    pOut->current = Svc_UnpackInt16LE(&pFrame[offset]);
    offset += 2U;

    crc_recv  = (uint16_t)pFrame[offset] | ((uint16_t)pFrame[offset + 1] << 8);
    crc_calc  = Svc_CalcCRC16Modbus(pFrame, offset);
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
 * @brief 将反馈数据按 Simulink 反馈帧格式写入缓冲区（小端字段 + MODBUS CRC16）
 * @param pIn 反馈数据源
 * @param pBuf 目标缓冲区
 * @param bufSize 缓冲区容量，须不小于 SIMULINK_FEEDBACK_FRAME_LEN
 * @param pOutLen 可选：写入的总字节数（含 CRC）；可为 NULL
 * @return SIMULINK_PROTOCOL_OK；否则为空指针或缓冲区不足
 */
static SimulinkProtocolResult_t Svc_SimulinkProtocol_AssembleFeedbackFrame(const SimulinkProtocolFeedbackData_t *pIn,
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

    Svc_PackInt16LE(&pBuf[offset], pIn->motor_current);
    offset += 2U;
    Svc_PackInt32LE(&pBuf[offset], pIn->motor_position);
    offset += 4U;
    Svc_PackInt32LE(&pBuf[offset], pIn->motor_speed);
    offset += 4U;
    Svc_PackInt32LE(&pBuf[offset], pIn->axis_position);
    offset += 4U;
    Svc_PackInt32LE(&pBuf[offset], pIn->axis_speed);
    offset += 4U;
    Svc_PackInt32LE(&pBuf[offset], pIn->pendulum_position);
    offset += 4U;
    Svc_PackInt32LE(&pBuf[offset], pIn->pendulum_speed);
    offset += 4U;

    crc16 = Svc_CalcCRC16Modbus(pBuf, offset);
    pBuf[offset++] = (uint8_t)(crc16 & 0xFFU);
    pBuf[offset++] = (uint8_t)((crc16 >> 8) & 0xFFU);

    if (pOutLen != NULL)
    {
        *pOutLen = offset;
    }

    return SIMULINK_PROTOCOL_OK;
}

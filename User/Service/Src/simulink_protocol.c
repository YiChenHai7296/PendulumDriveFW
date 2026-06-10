/**
 * @file simulink_protocol.c
 * @brief 服务层：Simulink 协议组帧/解帧实现
 * @details 控制/反馈载荷字段校验在本模块；载荷打包/解包使用 `common.h` 小端工具；
 *          帧 CRC16 使用 BSP 硬件 `Bsp_Crc16Modbus_Byte`（须已 `MX_CRC_Init`）；USART2 收发经 `usart.h` 中 `Drv_Simulink_*`。
 *          错误打印统一走 `BSP_LOG_PRINTF`（bsp.h），受 `DEBUG_PRINTF` 开关约束，关闭时调用点零开销。
 */

/* ======================== 1. 头文件引用 ======================== */
#include "simulink_protocol.h"
#include <stdio.h>            /* printf（错误打印）          */
#include "bsp.h"
#include "common.h"
#include "usart.h"

/* ======================== 2. 私有宏定义 ======================== */
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
#define SIMULINK_AXIS_POSITION_MAX_17BIT 131071    /* 倒立摆：2^17 - 1 */
#define SIMULINK_AXIS_POSITION_MAX_20BIT 1048575   /* 软尺摆：2^20 - 1 */
#define SIMULINK_AXIS_SPEED_MIN    (-4000)
#define SIMULINK_AXIS_SPEED_MAX    4000
#define SIMULINK_PENDULUM_POSITION_MIN 0
#define SIMULINK_PENDULUM_POSITION_MAX 131071   /* 17 位：2^17 - 1，与摆臂位置量程一致 */
#define SIMULINK_PENDULUM_SPEED_MIN (-4000)
#define SIMULINK_PENDULUM_SPEED_MAX 4000
#define SIMULINK_SOFT_RULER_SWING_MV_MIN (-20000)
#define SIMULINK_SOFT_RULER_SWING_MV_MAX 20000
#define SIMULINK_SOFT_RULER_ADC_RAW_MIN 0
#define SIMULINK_SOFT_RULER_ADC_RAW_MAX 4095

/* ======================== 3. 私有类型定义 ======================== */
/* 无 */

/* ======================== 4. 对外变量定义 ======================== */
/* 无 */

/* ======================== 5. 私有变量 ======================== */
static uint8_t g_au8FeedbackTxBuf[SIMULINK_PROTOCOL_FEEDBACK_FRAME_SIZE];  /* 反馈帧发送缓冲区 */

/* ======================== 6. 私有函数声明 ======================== */
static SimulinkProtocolResult_t Svc_SimulinkProtocol_ParseControlFrame(const uint8_t *pu8Frame,
                                                                   uint16_t u16Length,
                                                                   SimulinkProtocolControlData_t *struOut);
static SimulinkProtocolResult_t Svc_SimulinkProtocol_AssembleFeedbackFrame(const SimulinkProtocolFeedbackData_t *struIn,
                                                                       uint8_t *pu8Buf,
                                                                       uint16_t u16BufSize,
                                                                       uint16_t *pu16OutLen);

/* ======================== 7. 接口函数实现 ======================== */
/**
 * @brief 从 USART2 控制通道取一帧并解析为控制量（转速万分比、电流设定）
 * @details 调用驱动层 `Drv_Simulink_ControlFrame_GetData` 取原始字节，长度须为 `SIMULINK_CONTROL_FRAME_LEN`；
 *          CRC 校验使用 `Bsp_Crc16Modbus_Byte`（与 `Cmn_CalcCRC16Modbus` 等价）。
 * @param[out] struOut 解析得到的控制数据
 * @retval SIMULINK_PROTOCOL_OK 或错误码（空指针、长度、CRC、范围等）
 */
SimulinkProtocolResult_t Svc_SimulinkProtocol_UnpackControl(SimulinkProtocolControlData_t *struOut)
{
    uint8_t au8RxBuf[SIMULINK_CONTROL_FRAME_LEN];  /* 控制帧原始字节缓冲 */
    uint16_t u16Len = 0U;                          /* 实际接收长度 */

    if (struOut == NULL)
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    if (Drv_Simulink_ControlFrame_GetData(au8RxBuf, sizeof(au8RxBuf), &u16Len) != STATUS_OK)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }
    if (u16Len != SIMULINK_CONTROL_FRAME_LEN)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }
    return Svc_SimulinkProtocol_ParseControlFrame(au8RxBuf, u16Len, struOut);
}

/**
 * @brief 校验反馈载荷、组 Simulink 反馈帧并经 USART2 DMA 发送
 * @details 字段范围校验后组帧；CRC16 由 `Bsp_Crc16Modbus_Byte` 计算；发送由 `Drv_Simulink_Feedback_Send` 完成。
 * @param[in] struIn 待上报的反馈数据（各字段须满足宏 `SIMULINK_*_MIN` / `MAX`）
 * @retval SIMULINK_PROTOCOL_OK；否则为范围、组帧或发送错误码
 */
SimulinkProtocolResult_t Svc_SimulinkProtocol_PublishFeedback(const SimulinkProtocolFeedbackData_t *struIn,
                                                              ControlObject_t enControlObject)
{
    SimulinkProtocolResult_t res;   /* 组帧结果 */

    if (struIn == NULL)
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    if (struIn->s16MotorCurrent < SIMULINK_MOTOR_CURRENT_MIN || struIn->s16MotorCurrent > SIMULINK_MOTOR_CURRENT_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (struIn->s32MotorPosition < SIMULINK_MOTOR_POSITION_MIN || struIn->s32MotorPosition > SIMULINK_MOTOR_POSITION_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (struIn->s32MotorSpeed < SIMULINK_MOTOR_SPEED_MIN || struIn->s32MotorSpeed > SIMULINK_MOTOR_SPEED_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (enControlObject == CONTROL_OBJECT_INVERTED_PENDULUM)
    {
        if (struIn->s32AxisPosition < SIMULINK_AXIS_POSITION_MIN ||
            struIn->s32AxisPosition > SIMULINK_AXIS_POSITION_MAX_17BIT)
        {
            return SIMULINK_PROTOCOL_ERR_RANGE;
        }
    }
    else
    {
        if (struIn->s32AxisPosition < SIMULINK_AXIS_POSITION_MIN ||
            struIn->s32AxisPosition > SIMULINK_AXIS_POSITION_MAX_20BIT)
        {
            return SIMULINK_PROTOCOL_ERR_RANGE;
        }
    }
    if (struIn->s32AxisSpeed < SIMULINK_AXIS_SPEED_MIN || struIn->s32AxisSpeed > SIMULINK_AXIS_SPEED_MAX)
    {
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }
    if (enControlObject == CONTROL_OBJECT_INVERTED_PENDULUM)
    {
        if (struIn->s32PendulumPosition < SIMULINK_PENDULUM_POSITION_MIN || struIn->s32PendulumPosition > SIMULINK_PENDULUM_POSITION_MAX)
        {
            return SIMULINK_PROTOCOL_ERR_RANGE;
        }
        if (struIn->s32PendulumSpeed < SIMULINK_PENDULUM_SPEED_MIN || struIn->s32PendulumSpeed > SIMULINK_PENDULUM_SPEED_MAX)
        {
            return SIMULINK_PROTOCOL_ERR_RANGE;
        }
    }
    else
    {
        /* 软尺摆：最后两个 int32 字段重解释为 摆动电压(mV) 与 ADC3 原始码 */
        if (struIn->s32PendulumPosition < SIMULINK_SOFT_RULER_SWING_MV_MIN || struIn->s32PendulumPosition > SIMULINK_SOFT_RULER_SWING_MV_MAX)
        {
            return SIMULINK_PROTOCOL_ERR_RANGE;
        }
        if (struIn->s32PendulumSpeed < SIMULINK_SOFT_RULER_ADC_RAW_MIN || struIn->s32PendulumSpeed > SIMULINK_SOFT_RULER_ADC_RAW_MAX)
        {
            return SIMULINK_PROTOCOL_ERR_RANGE;
        }
    }

    res = Svc_SimulinkProtocol_AssembleFeedbackFrame(struIn, g_au8FeedbackTxBuf, SIMULINK_FEEDBACK_FRAME_LEN, NULL);
    if (res != SIMULINK_PROTOCOL_OK)
    {
        BSP_LOG_PRINTF("组帧失败\n");
        return res;
    }

    if (Drv_Simulink_Feedback_Send(g_au8FeedbackTxBuf, SIMULINK_FEEDBACK_FRAME_LEN) != STATUS_OK)
    {
        return SIMULINK_PROTOCOL_ERR_SEND;
    }

    return SIMULINK_PROTOCOL_OK;
}

/* ======================== 8. 私有函数实现 ======================== */
/**
 * @brief 从原始字节流解析一帧 Simulink 控制指令并校验 CRC 与字段范围
 * @details CRC 覆盖帧头至载荷末尾（不含 CRC 两字节），计算使用 `Bsp_Crc16Modbus_Byte`。
 * @param pu8Frame 完整控制帧缓冲区
 * @param u16Length 实际长度，须不小于 SIMULINK_CONTROL_FRAME_LEN
 * @param struOut 输出：转速万分比（-10000~10000）、电流设定等
 * @return SIMULINK_PROTOCOL_OK；否则为空指针、长度、类型、CRC、范围错误
 */
static SimulinkProtocolResult_t Svc_SimulinkProtocol_ParseControlFrame(const uint8_t *pu8Frame,
                                                                   uint16_t u16Length,
                                                                   SimulinkProtocolControlData_t *struOut)
{
    uint16_t u16Offset = 0U;   /* 帧内解析偏移 */
    uint16_t u16CrcCalc;      /* 计算得到的 CRC */
    uint16_t u16CrcRecv;      /* 帧内接收的 CRC */

    if ((pu8Frame == NULL) || (struOut == NULL))
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    if (u16Length < SIMULINK_CONTROL_FRAME_LEN)
    {
        BSP_LOG_PRINTF("指令长度非法！\n");
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    if (pu8Frame[u16Offset++] != SIMULINK_HEAD_BYTE0 || pu8Frame[u16Offset++] != SIMULINK_HEAD_BYTE1)
    {
        BSP_LOG_PRINTF("指令帧头错误！\n");
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    if (pu8Frame[u16Offset++] != SIMULINK_TYPE_CONTROL)
    {
        BSP_LOG_PRINTF("指令类型错误！\n");
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    if (pu8Frame[u16Offset++] != SIMULINK_LEN_CONTROL)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    struOut->s16Pwm    = Cmn_UnpackInt16LE(&pu8Frame[u16Offset]);
    u16Offset += 2U;
    struOut->s16Current = Cmn_UnpackInt16LE(&pu8Frame[u16Offset]);
    u16Offset += 2U;

    u16CrcRecv  = (uint16_t)pu8Frame[u16Offset] | ((uint16_t)pu8Frame[u16Offset + 1] << 8);
    u16CrcCalc  = Bsp_Crc16Modbus_Byte(pu8Frame, u16Offset);

    if (u16CrcCalc != u16CrcRecv)
    {
        BSP_LOG_PRINTF("crc校验有误，接收值 %x , 计算值 %x \n",u16CrcRecv,u16CrcCalc);
        return SIMULINK_PROTOCOL_ERR_CRC;
    }

    if (struOut->s16Pwm < SIMULINK_PWM_MIN || struOut->s16Pwm > SIMULINK_PWM_MAX)
    {
        BSP_LOG_PRINTF("转速万分比范围有误： %d\n", (int)struOut->s16Pwm);
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }

    if (struOut->s16Current < SIMULINK_CURRENT_MIN || struOut->s16Current > SIMULINK_CURRENT_MAX)
    {
        BSP_LOG_PRINTF("电流设定范围有误： %d\n", (int)struOut->s16Current);
        return SIMULINK_PROTOCOL_ERR_RANGE;
    }

    return SIMULINK_PROTOCOL_OK;
}

/**
 * @brief 将反馈数据按 Simulink 反馈帧格式写入缓冲区（小端字段 + CRC16）
 * @details CRC16 由 `Bsp_Crc16Modbus_Byte` 计算后低字节在前写入缓冲区尾部。
 * @param struIn 反馈数据源
 * @param pu8Buf 目标缓冲区
 * @param u16BufSize 缓冲区容量，须不小于 SIMULINK_FEEDBACK_FRAME_LEN
 * @param pu16OutLen 可选：写入的总字节数（含 CRC）；可为 NULL
 * @return SIMULINK_PROTOCOL_OK；否则为空指针或缓冲区不足
 */
static SimulinkProtocolResult_t Svc_SimulinkProtocol_AssembleFeedbackFrame(const SimulinkProtocolFeedbackData_t *struIn,
                                                                       uint8_t *pu8Buf,
                                                                       uint16_t u16BufSize,
                                                                       uint16_t *pu16OutLen)
{
    uint16_t u16Offset = 0U;  /* 帧内组帧偏移 */
    uint16_t u16Crc16;        /* 帧 CRC16 校验值 */

    if ((struIn == NULL) || (pu8Buf == NULL))
    {
        return SIMULINK_PROTOCOL_ERR_NULL;
    }

    if (u16BufSize < SIMULINK_FEEDBACK_FRAME_LEN)
    {
        return SIMULINK_PROTOCOL_ERR_LENGTH;
    }

    pu8Buf[u16Offset++] = SIMULINK_HEAD_BYTE0;
    pu8Buf[u16Offset++] = SIMULINK_HEAD_BYTE1;
    pu8Buf[u16Offset++] = SIMULINK_TYPE_FEEDBACK;
    pu8Buf[u16Offset++] = SIMULINK_LEN_FEEDBACK;

    Cmn_PackInt16LE(&pu8Buf[u16Offset], struIn->s16MotorCurrent);
    u16Offset += 2U;
    Cmn_PackInt32LE(&pu8Buf[u16Offset], struIn->s32MotorPosition);
    u16Offset += 4U;
    Cmn_PackInt32LE(&pu8Buf[u16Offset], struIn->s32MotorSpeed);
    u16Offset += 4U;
    Cmn_PackInt32LE(&pu8Buf[u16Offset], struIn->s32AxisPosition);
    u16Offset += 4U;
    Cmn_PackInt32LE(&pu8Buf[u16Offset], struIn->s32AxisSpeed);
    u16Offset += 4U;
    Cmn_PackInt32LE(&pu8Buf[u16Offset], struIn->s32PendulumPosition);
    u16Offset += 4U;
    Cmn_PackInt32LE(&pu8Buf[u16Offset], struIn->s32PendulumSpeed);
    u16Offset += 4U;

    u16Crc16 = Bsp_Crc16Modbus_Byte(pu8Buf, u16Offset);
    pu8Buf[u16Offset++] = (uint8_t)(u16Crc16 & 0xFFU);
    pu8Buf[u16Offset++] = (uint8_t)((u16Crc16 >> 8) & 0xFFU);

    if (pu16OutLen != NULL)
    {
        *pu16OutLen = u16Offset;
    }

    return SIMULINK_PROTOCOL_OK;
}

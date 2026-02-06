/**
 * @file motor_service.c
 * @brief 电机功能服务实现：编码器解析、反馈数据、占空比控制
 */

/* ======================== 0. 头文件引用 ======================== */
#include "motor_service.h"

/* ======================== 1. 私有宏定义 ======================== */
/* 编码器快照 12 字节：前6=最新帧，后6=上一帧 */
#define ENCODER_FRAME_LENGTH_BYTES   6U      /* 编码器单帧长度：CM(1) + SA(1) + AS0~AS2(3) + CRC8(1) */
#define ENCODER_CM_VALUE             0x02U  /* 编码器帧头标识 */
#define ENCODER_SA_COUNT_ERROR_BIT   (1U << 4)  /* SA字节第4位：计数错误标志 */
#define ENCODER_SA_MT_BATT_ERR_BIT   (1U << 5)  /* SA字节第5位：多圈/电池错误标志 */
#define ENCODER_ABS_POSITION_MASK    0x001FFFFFU  /* 21位绝对位置掩码 */

/* 电机电流检测参数 */
#define MOTOR_CURRENT_OFFSET_V       1.6f    /* ADC电压偏置（V） */
#define MOTOR_CURRENT_GAIN           41.0f   /* 电流放大倍数 */
#define MOTOR_CURRENT_SHUNT_R        0.02f   /* 采样电阻（Ω） */
#define MOTOR_FEEDBACK_CURRENT_MAX   2000    /* 反馈电流上限 */
#define MOTOR_FEEDBACK_CURRENT_MIN   (-2000) /* 反馈电流下限 */
#define MOTOR_CURRENT_TO_FEEDBACK_A  1000.0f /* 电流单位转换系数（A -> 反馈单位） */

/* ======================== 2. 私有类型定义 ======================== */
/* 编码器协议解析结果 */
typedef enum
{
    ENCODER_PROTOCOL_OK = 0,        /* 解析成功 */
    ENCODER_PROTOCOL_ERR_NULL,      /* 指针为空 */
    ENCODER_PROTOCOL_ERR_LENGTH,    /* 帧长度错误 */
    ENCODER_PROTOCOL_ERR_CRC        /* CRC校验失败 */
} EncoderProtocolResult_t;

/* 编码器单帧数据 */
typedef struct
{
    uint8_t  count_error;           /* 计数错误标志 */
    uint8_t  mt_or_batt_error;      /* 多圈/电池错误标志 */
    uint32_t absolute_position;     /* 21位绝对位置 */
} EncoderProtocolData_t;

/* 编码器双帧数据（最新帧 + 上一帧，用于速度计算） */
typedef struct
{
    EncoderProtocolData_t latest;   /* 最新帧 */
    EncoderProtocolData_t previous; /* 上一帧 */
} EncoderProtocolDataDual_t;

/* 编码器位数枚举 */
typedef enum
{
    ENCODER_BITS_21 = 21,  /* 21位编码器（电机） */
    ENCODER_BITS_20 = 20,  /* 20位编码器（输出轴） */
    ENCODER_BITS_17 = 17   /* 17位编码器（摆臂） */
} EncoderBits_t;

/* ======================== 3. 私有变量 ======================== */
/* 无 */

/* ======================== 4. 对外变量定义 ======================== */
/* 无 */

/* ======================== 5. 私有函数声明 ======================== */
static uint8_t EncoderProtocol_CalcCRC8(const uint8_t *pData, uint16_t length);
static EncoderProtocolResult_t EncoderProtocol_ParseFrame(const uint8_t *pFrame,
                                                          uint16_t length,
                                                          EncoderProtocolData_t *pOut);
static EncoderProtocolResult_t EncoderProtocol_ReadMotor(EncoderProtocolDataDual_t *pOut);
static EncoderProtocolResult_t EncoderProtocol_ReadOutputShaft(EncoderProtocolDataDual_t *pOut);
static EncoderProtocolResult_t EncoderProtocol_ReadSwingArm(EncoderProtocolDataDual_t *pOut);
static float EncoderSpeed_Calc(uint32_t pos_prev, uint32_t pos_curr, EncoderBits_t bits);
static float EncoderSpeed_CalcMotor(const EncoderProtocolDataDual_t *pDual);
static float EncoderSpeed_CalcOutputShaft(const EncoderProtocolDataDual_t *pDual);
static float EncoderSpeed_CalcSwingArm(const EncoderProtocolDataDual_t *pDual);
static float MotorService_GetMotorVoltage(void);
static float MotorService_GetMotorCurrent(void);

/* ======================== 6. 接口函数实现 ======================== */

/**
 * @brief 电机初始化：使能电机、方向设为正向、占空比设为 0
 */
void MotorService_InitMotor(void)
{
    PWM_Enable(ENABLE);
    PWM_DirControl(MOTOR_DIR_FORWARD);
    (void)MotorService_SetDutyCycle(10);
}

/**
 * @brief 关闭电机: 电机失能、方向设为正向、占空比设为 0
 */
void MotorService_CloseMotor(void)
{
    PWM_Enable(DISABLE);
    PWM_DirControl(MOTOR_DIR_FORWARD);
    (void)MotorService_SetDutyCycle(0);
}


/**
 * @brief 将编码器协议结果映射为电机服务错误码
 */
static MotorServiceResult_t MotorService_MapEncoderResult(EncoderProtocolResult_t encRes,
                                                         MotorServiceResult_t encoderErr)
{
    if (encRes == ENCODER_PROTOCOL_OK)
    {
        return MOTOR_SVC_OK;
    }
    return encoderErr;
}

/**
 * @brief 获取电机反馈数据
 * @details 读取三个编码器的位置数据，计算转速，读取电机电流，统一填入反馈结构体
 * @param pOut 反馈数据输出缓冲区
 * @return 操作结果，任一编码器解析失败时返回对应错误码，pOut 可能包含部分有效数据
 */
MotorServiceResult_t MotorService_GetFeedbackData(MotorFeedbackData_t *pOut)
{
    EncoderProtocolDataDual_t motor_dual;
    EncoderProtocolDataDual_t shaft_dual;
    EncoderProtocolDataDual_t swing_dual;
    EncoderProtocolResult_t  encRes;
    float current_A;
    float speed_f;
    int32_t val;

    if (pOut == NULL)
    {
        return MOTOR_SVC_ERR_NULL;
    }

    /* 读取三个编码器的双帧数据 */
    encRes = EncoderProtocol_ReadMotor(&motor_dual);
    if (encRes != ENCODER_PROTOCOL_OK)
    {
        return MotorService_MapEncoderResult(encRes, MOTOR_SVC_ERR_ENCODER_MOTOR);
    }

    encRes = EncoderProtocol_ReadOutputShaft(&shaft_dual);
    if (encRes != ENCODER_PROTOCOL_OK)
    {
        return MotorService_MapEncoderResult(encRes, MOTOR_SVC_ERR_ENCODER_SHAFT);
    }

    encRes = EncoderProtocol_ReadSwingArm(&swing_dual);
    if (encRes != ENCODER_PROTOCOL_OK)
    {
        return MotorService_MapEncoderResult(encRes, MOTOR_SVC_ERR_ENCODER_SWING);
    }

    /* 读取并转换电机电流，限制在有效范围内 */
    current_A = MotorService_GetMotorCurrent();
    val = (int32_t)(current_A * MOTOR_CURRENT_TO_FEEDBACK_A);
    if (val > MOTOR_FEEDBACK_CURRENT_MAX)
    {
        val = MOTOR_FEEDBACK_CURRENT_MAX;
    }
    else if (val < MOTOR_FEEDBACK_CURRENT_MIN)
    {
        val = MOTOR_FEEDBACK_CURRENT_MIN;
    }
    pOut->motor_current = (int16_t)val;

    /* 填充位置数据（取最新帧的绝对位置） */
    pOut->motor_position    = (int32_t)motor_dual.latest.absolute_position;
    pOut->axis_position     = (int32_t)(shaft_dual.latest.absolute_position & 0xFFFFFU);  /* 20位掩码 */
    pOut->pendulum_position = (int32_t)(swing_dual.latest.absolute_position & 0x1FFFFU); /* 17位掩码 */

    /* 计算并填充转速数据（基于双帧位置差） */
    speed_f = EncoderSpeed_CalcMotor(&motor_dual);
    pOut->motor_speed = (int32_t)speed_f;

    speed_f = EncoderSpeed_CalcOutputShaft(&shaft_dual);
    pOut->axis_speed = (int32_t)speed_f;

    speed_f = EncoderSpeed_CalcSwingArm(&swing_dual);
    pOut->pendulum_speed = (int32_t)speed_f;

    return MOTOR_SVC_OK;
}

/**
 * @brief 设置电机占空比
 * @param duty_permille 占空比指令（-10000~10000，对应 -100.00%~100.00%）
 * @note  占空比正负用于区分转向：正为正转，负为反转
 * @return 操作结果
 */
MotorServiceResult_t MotorService_SetDutyCycle(int16_t duty_permille)
{
    uint16_t duty_u16 = 0U;
    unsigned char pwmRet;

    /* 合法范围：-10000~10000 */
    if (duty_permille < -10000 || duty_permille > 10000)
    {
        return MOTOR_SVC_ERR_PARAM;
    }

    /* 根据占空比正负决定电机方向，并取绝对值作为占空比大小 */
    if (duty_permille > 0)
    {
        /* 正占空比：电机正转 */
        PWM_DirControl(MOTOR_DIR_FORWARD);
        duty_u16 = (uint16_t)duty_permille;
    }
    else if (duty_permille < 0)
    {
        /* 负占空比：电机反转 */
        PWM_DirControl(MOTOR_DIR_REVERSE);
        duty_u16 = (uint16_t)(-duty_permille);
    }
    else
    {
        /* 占空比为 0：保持当前方向，输出 0 占空比 */
        duty_u16 = 0U;
    }

    pwmRet = PWM_Set_TargePulse((unsigned short)duty_u16);
    return (pwmRet == 0U) ? MOTOR_SVC_OK : MOTOR_SVC_ERR_DRIVER;
}

/* ======================== 7. 私有函数实现 ======================== */

/**
 * @brief 解析编码器单帧数据
 * @param pFrame 帧数据缓冲区（6字节）
 * @param length 缓冲区长度
 * @param pOut 解析结果输出
 * @return 解析结果
 */
static EncoderProtocolResult_t EncoderProtocol_ParseFrame(const uint8_t *pFrame,
                                                          uint16_t length,
                                                          EncoderProtocolData_t *pOut)
{
    uint8_t cm, sa, as0, as1, as2;
    uint32_t raw_position;
    uint8_t crc_calc, crc_recv;

    if ((pFrame == NULL) || (pOut == NULL))
    {
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    if (length < ENCODER_FRAME_LENGTH_BYTES)
    {
        return ENCODER_PROTOCOL_ERR_LENGTH;
    }

    /* 提取帧字段：CM(帧头) SA(状态) AS0~AS2(位置) CRC8(校验) */
    cm = pFrame[0];
    sa = pFrame[1];
    as0 = pFrame[2];
    as1 = pFrame[3];
    as2 = pFrame[4];
    crc_recv = pFrame[5];

    /* 校验帧头 */
    if (cm != ENCODER_CM_VALUE)
    {
        return ENCODER_PROTOCOL_ERR_LENGTH;
    }

    /* 校验CRC（前5字节） */
    crc_calc = EncoderProtocol_CalcCRC8(pFrame, 5U);
    if (crc_calc != crc_recv)
    {
        return ENCODER_PROTOCOL_ERR_CRC;
    }

    /* 解析状态标志位 */
    pOut->count_error      = ((sa & ENCODER_SA_COUNT_ERROR_BIT) != 0U);
    pOut->mt_or_batt_error = ((sa & ENCODER_SA_MT_BATT_ERR_BIT) != 0U);

    /* 组合21位绝对位置（小端序） */
    raw_position = (uint32_t)as0 | ((uint32_t)as1 << 8) | ((uint32_t)as2 << 16);
    pOut->absolute_position = raw_position & ENCODER_ABS_POSITION_MASK;

    return ENCODER_PROTOCOL_OK;
}

/**
 * @brief 读取电机编码器双帧数据
 * @param pOut 双帧数据输出
 * @return 解析结果
 */
static EncoderProtocolResult_t EncoderProtocol_ReadMotor(EncoderProtocolDataDual_t *pOut)
{
    uint8_t rawBuf[ENCODER_SNAPSHOT_BYTES];
    EncoderProtocolResult_t resLatest;
    EncoderProtocolResult_t resPrev;

    if (pOut == NULL)
    {
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    /* 从USART3获取12字节快照（前6=最新帧，后6=上一帧） */
    MotorEncoder_GetData(rawBuf);
    resLatest = EncoderProtocol_ParseFrame(&rawBuf[0], ENCODER_FRAME_LENGTH_BYTES, &pOut->latest);
    resPrev   = EncoderProtocol_ParseFrame(&rawBuf[ENCODER_FRAME_LENGTH_BYTES], ENCODER_FRAME_LENGTH_BYTES, &pOut->previous);
    if (resLatest != ENCODER_PROTOCOL_OK)
    {
        return resLatest;
    }
    if (resPrev != ENCODER_PROTOCOL_OK)
    {
        return resPrev;
    }
    return ENCODER_PROTOCOL_OK;
}

static EncoderProtocolResult_t EncoderProtocol_ReadOutputShaft(EncoderProtocolDataDual_t *pOut)
{
    uint8_t rawBuf[ENCODER_SNAPSHOT_BYTES];
    EncoderProtocolResult_t resLatest;
    EncoderProtocolResult_t resPrev;

    if (pOut == NULL)
    {
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    /* 从USART4获取12字节快照（前6=最新帧，后6=上一帧） */
    OutputShaftEncoder_GetData(rawBuf);
    resLatest = EncoderProtocol_ParseFrame(&rawBuf[0], ENCODER_FRAME_LENGTH_BYTES, &pOut->latest);
    resPrev   = EncoderProtocol_ParseFrame(&rawBuf[ENCODER_FRAME_LENGTH_BYTES], ENCODER_FRAME_LENGTH_BYTES, &pOut->previous);
    if (resLatest != ENCODER_PROTOCOL_OK)
    {
        return resLatest;
    }
    if (resPrev != ENCODER_PROTOCOL_OK)
    {
        return resPrev;
    }
    return ENCODER_PROTOCOL_OK;
}

static EncoderProtocolResult_t EncoderProtocol_ReadSwingArm(EncoderProtocolDataDual_t *pOut)
{
    uint8_t rawBuf[ENCODER_SNAPSHOT_BYTES];
    EncoderProtocolResult_t resLatest;
    EncoderProtocolResult_t resPrev;

    if (pOut == NULL)
    {
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    /* 从USART5获取12字节快照（前6=最新帧，后6=上一帧） */
    SwingArmEncoder_GetData(rawBuf);
    resLatest = EncoderProtocol_ParseFrame(&rawBuf[0], ENCODER_FRAME_LENGTH_BYTES, &pOut->latest);
    resPrev   = EncoderProtocol_ParseFrame(&rawBuf[ENCODER_FRAME_LENGTH_BYTES], ENCODER_FRAME_LENGTH_BYTES, &pOut->previous);
    if (resLatest != ENCODER_PROTOCOL_OK)
    {
        return resLatest;
    }
    if (resPrev != ENCODER_PROTOCOL_OK)
    {
        return resPrev;
    }
    return ENCODER_PROTOCOL_OK;
}

/**
 * @brief 计算编码器角速度（基于前后两帧位置差）
 * @details 编码器位置范围0~max对应0~360°，自动处理过零情况
 * @param pos_prev 上一帧位置
 * @param pos_curr 当前帧位置
 * @param bits 编码器位数（21/20/17）
 * @return 角速度（°/s），采样间隔1ms
 */
static float EncoderSpeed_Calc(uint32_t pos_prev, uint32_t pos_curr, EncoderBits_t bits)
{
    uint32_t max_val;
    uint32_t half;
    int32_t  delta_raw;
    int32_t  delta;
    float    speed_deg_s;

    /* 根据编码器位数确定最大值 */
    switch (bits)
    {
        case ENCODER_BITS_21:
            max_val = 2097151U;  /* 2^21 - 1 */
            break;
        case ENCODER_BITS_20:
            max_val = 1048575U;  /* 2^20 - 1 */
            break;
        case ENCODER_BITS_17:
            max_val = 131071U;   /* 2^17 - 1 */
            break;
        default:
        {
            return 0.0f;
        }
    }

    /* 限制位置值在有效范围内 */
    pos_prev = (pos_prev > max_val) ? max_val : pos_prev;
    pos_curr = (pos_curr > max_val) ? max_val : pos_curr;

    /* 计算位置差，处理过零情况（编码器从max跳转到0或从0跳转到max） */
    delta_raw = (int32_t)pos_curr - (int32_t)pos_prev;
    half      = (max_val + 1U) >> 1;  /* 中点值 */

    if (delta_raw > (int32_t)half)
    {
        /* 正向过零：实际是反向运动 */
        delta = delta_raw - (int32_t)(max_val + 1U);
    }
    else if (delta_raw < -(int32_t)half)
    {
        /* 反向过零：实际是正向运动 */
        delta = delta_raw + (int32_t)(max_val + 1U);
    }
    else
    {
        /* 正常情况 */
        delta = delta_raw;
    }

    /* 转换为角速度（°/s）：delta / (max+1) * 360° / 0.001s */
    speed_deg_s = (float)delta * 360000.0f / (float)(max_val + 1U);
    return speed_deg_s;
}

/**
 * @brief 计算电机编码器转速（21位）
 * @param pDual 电机编码器双帧数据
 * @return 角速度（°/s）
 */
static float EncoderSpeed_CalcMotor(const EncoderProtocolDataDual_t *pDual)
{
    if (pDual == NULL)
    {
        return 0.0f;
    }
    return EncoderSpeed_Calc(pDual->previous.absolute_position,
                             pDual->latest.absolute_position,
                             ENCODER_BITS_21);
}

/**
 * @brief 计算输出轴编码器转速（20位）
 * @param pDual 输出轴编码器双帧数据
 * @return 角速度（°/s）
 */
static float EncoderSpeed_CalcOutputShaft(const EncoderProtocolDataDual_t *pDual)
{
    if (pDual == NULL)
    {
        return 0.0f;
    }
    return EncoderSpeed_Calc(pDual->previous.absolute_position,
                             pDual->latest.absolute_position,
                             ENCODER_BITS_20);
}

/**
 * @brief 计算摆臂编码器转速（17位）
 * @param pDual 摆臂编码器双帧数据
 * @return 角速度（°/s）
 */
static float EncoderSpeed_CalcSwingArm(const EncoderProtocolDataDual_t *pDual)
{
    if (pDual == NULL)
    {
        return 0.0f;
    }
    return EncoderSpeed_Calc(pDual->previous.absolute_position,
                             pDual->latest.absolute_position,
                             ENCODER_BITS_17);
}

/**
 * @brief 读取电机电压（两路ADC平均值）
 * @return 电机电压（V）
 */
static float MotorService_GetMotorVoltage(void)
{
    uint8_t rawAdc[2];

    ADC_GetMotorVol(rawAdc);
    /* 两路ADC低字节平均，转换为电压值（12位ADC，参考电压3.3V） */
    return ((float)rawAdc[0] + (float)rawAdc[1]) / 2.0f * 3.3f / 4096.0f;
}

/**
 * @brief 计算电机电流
 * @details 基于电压读数：电流 = (电压 - 偏置) / 放大倍数 / 采样电阻
 * @return 电机电流（A）
 */
static float MotorService_GetMotorCurrent(void)
{
    float v_adc  = MotorService_GetMotorVoltage();
    float v_diff = v_adc - MOTOR_CURRENT_OFFSET_V;  /* 减去偏置电压 */
    return v_diff / MOTOR_CURRENT_GAIN / MOTOR_CURRENT_SHUNT_R;  /* 除以放大倍数和采样电阻 */
}

/**
 * @brief 计算CRC8校验值
 * @details 多项式：x^8 + x^2 + x + 1（对应多项式值0x01）
 * @param pData 数据缓冲区
 * @param length 数据长度
 * @return CRC8校验值
 */
static uint8_t EncoderProtocol_CalcCRC8(const uint8_t *pData, uint16_t length)
{
    uint8_t crc = 0x00U;
    uint16_t i;
    uint8_t bit;

    if (pData == NULL || length == 0U)
    {
        return 0U;
    }

    /* CRC8计算：逐字节异或，然后逐位移位校验 */
    for (i = 0U; i < length; i++)
    {
        crc ^= pData[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if (crc & 0x80U)
            {
                /* 最高位为1，异或多项式 */
                crc = (uint8_t)((crc << 1) ^ 0x01U);
            }
            else
            {
                /* 最高位为0，仅移位 */
                crc <<= 1;
            }
        }
    }
    return crc;
}

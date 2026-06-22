/**
 * @file    pendulum_service.c
 * @brief   服务层：摆系统驱动与反馈实现（电机、编码器、电流、软尺摆电压）
 * @details 编码器帧解析使用 `common.h` 中 `Cmn_CRC8_Calc`；电流与 PWM 经 `adc.h` / `hrtim.h`；
 *          标定与等待使用 `bsp.h`（`Bsp_Ms_Delay`）。不向上层暴露 HAL 句柄。
 */

/* ======================== 1. 头文件引用 ======================== */
#include "pendulum_service.h"
#include "bsp.h"
#include "usart.h"
#include "adc.h"
#include "hrtim.h"

/* 转速指令死区直接复用 `PWM_DUTY_CYCLE_MIN`（hrtim.h），不再单独定义死区宏；
 * 比较时务必将其强转 (int16_t)，避免与有符号指令比较时发生无符号提升而误判负指令（见 MotorSpeedPermyriad_Set）。 */



/* ======================== 2. 私有宏定义 ======================== */
/* 编码器快照 12 字节：前6=最新帧，后6=上一帧 */
#define ENCODER_FRAME_LENGTH_BYTES   6U      /* 编码器单帧长度：CM(1) + SA(1) + AS0~AS2(3) + CRC8(1) */
#define ENCODER_CM_VALUE             0x02U  /* 编码器帧头标识 */
#define ENCODER_SA_COUNT_ERROR_BIT   (1U << 4)  /* SA字节第4位：计数错误标志 */
#define ENCODER_SA_MT_BATT_ERR_BIT   (1U << 5)  /* SA字节第5位：多圈/电池错误标志 */
#define ENCODER_ABS_POSITION_MASK    0x001FFFFFU  /* 21位绝对位置掩码 */

/** 各编码器绝对位置计数最大值（2^bits - 1），与 `ENCODER_BITS_*` 一致 */
#define ENCODER_ABS_POSITION_MAX_21BIT  ((1U << 21) - 1U)
#define ENCODER_ABS_POSITION_MAX_20BIT  ((1U << 20) - 1U)
#define ENCODER_ABS_POSITION_MAX_17BIT  ((1U << 17) - 1U)

/** 一整圈机械角（°），仅用于本文件 `EncoderSpeed_Calc` */
#define ROUND_ANGLE            (360.0f)
/** 编码器双帧间隔（s），固定 1ms，与 TIM1 触发周期一致 */
#define ENCODER_SPEED_FRAME_DT_SEC  (0.001f)

/* 电机电流检测参数 */
#define MOTOR_CURRENT_OFFSET_V       1.6f    /* ADC电压偏置（V） */
#define MOTOR_CURRENT_GAIN           41.0f   /* 电流放大倍数 */
#define MOTOR_CURRENT_SHUNT_R        0.02f   /* 采样电阻（Ω） */
#define MOTOR_FEEDBACK_CURRENT_MAX   2000    /* 反馈电流上限 */
#define MOTOR_FEEDBACK_CURRENT_MIN   (-2000) /* 反馈电流下限 */
#define MOTOR_CURRENT_TO_FEEDBACK_A  1000.0f /* 电流单位转换系数（A -> 反馈单位） */
#define MOTOR_CURRENT_ZERO_CALIB_SAMPLES 64U /* 零点标定采样次数 */
#define MOTOR_CURRENT_ZERO_CALIB_WARMUP_MS 200U /* 标零前等待采样链路稳定（ms） */
#define MOTOR_CURRENT_ZERO_CALIB_INTERVAL_MS 2U /* 标零采样周期间隔（ms），每次读电流后延时 */
#define MOTOR_CURRENT_ZERO_CALIB_DISCARD_SAMPLES 16U /* 标零前丢弃样本数 */
#define MOTOR_CURRENT_ZERO_OFFSET_MAX_ABS_A 1.0f /* 零点偏置绝对值上限（异常保护） */

/** 与 `simulink_protocol.c` 中反馈帧字段校验范围一致，避免 float→int32 未定义行为及异常尖峰 */
#define FEEDBACK_SPD_MOTOR_MAX_ABS     40000
#define FEEDBACK_SPD_AXIS_PEND_MAX_ABS 4000

/** 摆杆速度 |°/s| 超过该阈值时 `BSP_LOG_PRINTF` 打印双帧绝对位置（调试用） */
#define PENDULUM_SPD_ABS_PRINTF_THRESHOLD_DPS  (60000.0f)

/* ======================== 3. 私有类型定义 ======================== */
/* 编码器协议解析结果 */
typedef enum
{
    ENCODER_PROTOCOL_OK = 0,        /* 解析成功 */
    ENCODER_PROTOCOL_ERR_NULL,      /* 指针为空 */
    ENCODER_PROTOCOL_ERR_LENGTH,    /* 帧长度不足 */
    ENCODER_PROTOCOL_ERR_HEADER,    /* 帧头 CM 非法 */
    ENCODER_PROTOCOL_ERR_CRC,       /* CRC8 校验失败 */
    ENCODER_PROTOCOL_ERR_SA_STATUS, /* SA 状态域 bit4/bit5 报错 */
    ENCODER_PROTOCOL_ERR_DRIVER     /* 底层编码器快照读取失败 */
} EncoderProtocolResult_t;

/* 编码器单帧数据 */
typedef struct
{
    uint8_t  u8CountErr;           /* 计数错误标志 */
    uint8_t  u8MtOrBattError;      /* 多圈/电池错误标志 */
    uint32_t u32AbsolutePosition;   /* 21位绝对位置 */
} EncoderFrame_t;

/* 编码器双帧数据（最新帧 + 上一帧，用于速度计算） */
typedef struct
{
    EncoderFrame_t struLatest;    /* 最新帧 */
    EncoderFrame_t struPrevious; /* 上一帧 */
} EncoderDual_t;

/* 编码器位数枚举 */
typedef enum
{
    ENCODER_BITS_21 = 21,  /* 21位编码器（电机） */
    ENCODER_BITS_20 = 20,  /* 20 位：软尺摆模式下输出轴 */
    ENCODER_BITS_17 = 17   /* 17 位：摆臂；倒立摆模式下输出轴 */
} EncoderBits_t;

/* ======================== 4. 对外变量定义 ======================== */
/* 无 */

/* ======================== 5. 私有变量 ======================== */
/* 电机电流零点偏置（A）：电机未启动时采样得到，后续采样均需扣除此偏置 */
static float s_fMotorCurrentZeroOffsetA = 0.0f;
static ControlObject_t s_enControlObject = CONTROL_OBJECT_INVERTED_PENDULUM;

/* ======================== 6. 私有函数声明 ======================== */
static EncoderProtocolResult_t EncoderProtocol_Frame_Parse(const uint8_t *pu8Frame,
                                                          uint16_t u16Length,
                                                          EncoderFrame_t *struOut);

/* 编码器数据读取 */
static EncoderProtocolResult_t EncoderProtocol_Motor_Read(EncoderDual_t *struOut);
static EncoderProtocolResult_t EncoderProtocol_OutputShaft_Read(EncoderDual_t *struOut);
static EncoderProtocolResult_t EncoderProtocol_SwingArm_Read(EncoderDual_t *struOut);

/* 编码器速度计算 */
static float EncoderSpeed_Calc(uint32_t u32PosPrev, uint32_t u32PosCurr, EncoderBits_t bits);
static float EncoderSpeed_Motor_Calc(const EncoderDual_t *struDual);
static float EncoderSpeed_OutputShaft_Calc(const EncoderDual_t *struDual);
static float EncoderSpeed_SwingArm_Calc(const EncoderDual_t *struDual);

/* 电机电流获取 */
static float MotorVoltage_Get(void);
static float MotorCurrentRaw_Get(void);
static float MotorCurrent_Get(void);

/* 电机控制 */
static void  Motor_Disable(void);
static uint16_t Motor_DutyPermyriad_Calibrate_Map(uint16_t u16DutyPermyriadAbs);


/* 软尺摆摆幅电压 */
static int32_t SoftRulerVoltageMilliVolt_Get(int32_t *ps32RawAdc);



/* 其他 */
static int32_t EncoderSpeed_Int32_ClampFromFloat(float v, int32_t s32Lo, int32_t s32Hi);
static uint32_t OutputShaftPositionMask_Get(void);
static EncoderBits_t OutputShaftEncoderBits_Get(void);
static PendulumServiceResult_t EncoderResult_Map(EncoderProtocolResult_t encRes,
                                                          PendulumServiceResult_t encoderErr);


/* ======================== 7. 接口函数实现 ======================== */

/* -------- 7.1 上层接口（与 .h 5.1 对应） -------- */

void Svc_PendulumService_ControlObject_Set(ControlObject_t enObject)
{
    s_enControlObject = enObject;
}

/**
 * @brief 电机电流零点标定（由上层在 PWM 失能、电机静止下调用，如上电主循环入口一次）
 * @details 在电机未启动状态下采样电流，求平均后写入静态零点偏置变量；
 *          开启 `MOTOR_CURRENT_ZERO_CALIB_ENABLE` 时含预热丢弃样本与异常幅值保护
 * @note 受 `MOTOR_CURRENT_ZERO_CALIB_ENABLE` 控制；关闭时仅将偏置置 0
 */
void Svc_PendulumService_CurrentZero_Calibrate(void)
{
#if MOTOR_CURRENT_ZERO_CALIB_ENABLE
    uint32_t u32I;
    float fSumA = 0.0f;

    /* 标定前确保电机处于失能状态，避免运动电流污染零点 */
    Drv_PWM_Enable(PRJ_DISABLE);
    /* 同步 HRTIM 比较器到“逻辑关断”占位，否则仍可能停在 Cube 初值或大占空比，标零会采到错误母线电流 */
    (void)Drv_PWM_TargetPulse_Set(0U);
    Bsp_Ms_Delay(MOTOR_CURRENT_ZERO_CALIB_WARMUP_MS);

    /* 丢弃启动阶段样本，避免把 DMA 初值或瞬态当成零点 */
    for (u32I = 0U; u32I < MOTOR_CURRENT_ZERO_CALIB_DISCARD_SAMPLES; u32I++)
    {
        (void)MotorCurrentRaw_Get();
        Bsp_Ms_Delay(MOTOR_CURRENT_ZERO_CALIB_INTERVAL_MS);
    }

    for (u32I = 0U; u32I < MOTOR_CURRENT_ZERO_CALIB_SAMPLES; u32I++)
    {
        fSumA += MotorCurrentRaw_Get();
        Bsp_Ms_Delay(MOTOR_CURRENT_ZERO_CALIB_INTERVAL_MS);
    }
    s_fMotorCurrentZeroOffsetA = fSumA / (float)MOTOR_CURRENT_ZERO_CALIB_SAMPLES;

    /* 保护：若零点偏置明显异常，则放弃本次标零，防止反馈整体漂移到满量程附近 */
    if ((s_fMotorCurrentZeroOffsetA > MOTOR_CURRENT_ZERO_OFFSET_MAX_ABS_A) ||
        (s_fMotorCurrentZeroOffsetA < -MOTOR_CURRENT_ZERO_OFFSET_MAX_ABS_A))
    {
        s_fMotorCurrentZeroOffsetA = 0.0f;
    }
#else
    /* 关闭零点标定：偏置固定为0，反馈直接使用原始电流 */
    s_fMotorCurrentZeroOffsetA = 0.0f;
#endif
}

/**
 * @brief 获取电机反馈数据
 * @details 读取电机/输出轴编码器双帧并算转速；倒立摆另读摆臂编码器，软尺摆以 ADC3 换算摆幅电压；
 *          再读电机电流，统一填入反馈结构体
 * @param struOut 反馈数据输出缓冲区
 * @return 操作结果，任一编码器解析失败时返回对应错误码，struOut 可能包含部分有效数据
 */
PendulumServiceResult_t Svc_PendulumService_FeedbackData_Get(PendulumFeedbackData_t *struOut)
{
    EncoderDual_t struMotorDual;
    EncoderDual_t struShaftDual;
    EncoderDual_t struSwingDual;
    EncoderProtocolResult_t  encRes;
    float fCurrentA;
    float fSpeed;
    int32_t s32Val;

    if (struOut == NULL)
    {
        return PENDULUM_SVC_ERR_NULL;
    }

    /* 读取电机与输出轴编码器双帧数据 */
    encRes = EncoderProtocol_Motor_Read(&struMotorDual);
    if (encRes != ENCODER_PROTOCOL_OK)
    {
        BSP_LOG_PRINTF("电机编码器读取失败\n");
        return EncoderResult_Map(encRes, PENDULUM_SVC_ERR_ENCODER_MOTOR);
    }

    encRes = EncoderProtocol_OutputShaft_Read(&struShaftDual);
    if (encRes != ENCODER_PROTOCOL_OK)
    {
        BSP_LOG_PRINTF("输出轴编码器读取失败\n");
        return EncoderResult_Map(encRes, PENDULUM_SVC_ERR_ENCODER_SHAFT);
    }

    /* 读取并转换电机电流，限制在有效范围内 */
    fCurrentA = MotorCurrent_Get();

    s32Val = (int32_t)(fCurrentA * MOTOR_CURRENT_TO_FEEDBACK_A);

    if (s32Val > MOTOR_FEEDBACK_CURRENT_MAX)
    {
        s32Val = MOTOR_FEEDBACK_CURRENT_MAX;
    }
    else if (s32Val < MOTOR_FEEDBACK_CURRENT_MIN)
    {
        s32Val = MOTOR_FEEDBACK_CURRENT_MIN;
    }
    struOut->s16MotorCurrent = (int16_t)s32Val;

    /* 填充位置数据（取最新帧的绝对位置） */
    struOut->s32MotorPosition    = (int32_t)struMotorDual.struLatest.u32AbsolutePosition;
    struOut->s32AxisPosition     = (int32_t)(struShaftDual.struLatest.u32AbsolutePosition &
                                            OutputShaftPositionMask_Get());

    /* 计算并填充转速数据（基于双帧位置差）；饱和+NaN 防护，与上位机反馈范围一致 */
    fSpeed = EncoderSpeed_Motor_Calc(&struMotorDual);
    struOut->s32MotorSpeed = EncoderSpeed_Int32_ClampFromFloat(fSpeed,
                                                                -FEEDBACK_SPD_MOTOR_MAX_ABS,
                                                                FEEDBACK_SPD_MOTOR_MAX_ABS);

    fSpeed = EncoderSpeed_OutputShaft_Calc(&struShaftDual);
    struOut->s32AxisSpeed = EncoderSpeed_Int32_ClampFromFloat(fSpeed,
                                                                -FEEDBACK_SPD_AXIS_PEND_MAX_ABS,
                                                                FEEDBACK_SPD_AXIS_PEND_MAX_ABS);

    if (s_enControlObject == CONTROL_OBJECT_INVERTED_PENDULUM)
    {
        encRes = EncoderProtocol_SwingArm_Read(&struSwingDual);
        if (encRes != ENCODER_PROTOCOL_OK)
        {
            BSP_LOG_PRINTF("摆杆编码器读取失败\n");
            return EncoderResult_Map(encRes, PENDULUM_SVC_ERR_ENCODER_SWING);
        }
        struOut->s32PendulumPosition = (int32_t)(struSwingDual.struLatest.u32AbsolutePosition & ENCODER_ABS_POSITION_MAX_17BIT);
        fSpeed = EncoderSpeed_SwingArm_Calc(&struSwingDual);
        struOut->s32PendulumSpeed = EncoderSpeed_Int32_ClampFromFloat(fSpeed,
                                                                          -FEEDBACK_SPD_AXIS_PEND_MAX_ABS,
                                                                          FEEDBACK_SPD_AXIS_PEND_MAX_ABS);
    }
    else
    {
        /* 软尺摆模式：以 ADC3 换算电压替代摆杆编码器量 */
        struOut->s32PendulumPosition = SoftRulerVoltageMilliVolt_Get(&struOut->s32PendulumSpeed);
    }

    return PENDULUM_SVC_OK;
}

/**
 * @brief 按转速万分比（permyriad）设置电机输出：-10000~10000 对应 -100.00%~+100.00%（每 1 = 0.01%），内部转为 PWM
 * @param s16SpeedPermyriad 转速万分比整数（-10000~10000）
 * @note  正负用于区分转向：正为正转，负为反转
 * @return 操作结果
 */
PendulumServiceResult_t Svc_PendulumService_MotorSpeedPermyriad_Set(int16_t s16SpeedPermyriad)
{
    uint16_t u16Duty = 0U;
    uint16_t u16DutyOutput = 0U;

    /* 合法范围：±SPEED_PERMYRIAD_MAX */
    if (s16SpeedPermyriad < -SPEED_PERMYRIAD_MAX || s16SpeedPermyriad > SPEED_PERMYRIAD_MAX)
    {
        return PENDULUM_SVC_ERR_PARAM;
    }

    /* 正转 / 反转 / 零指令：死区阈值用 `(int16_t)PWM_DUTY_CYCLE_MIN`，必须按有符号整型比较；
     * 勿写 `s16 > PWM_DUTY_CYCLE_MIN`：`PWM_DUTY_CYCLE_MIN` 为 8U（无符号）时负指令会被提升成大正数而误判为真。 */
    if (s16SpeedPermyriad > (int16_t)PWM_DUTY_CYCLE_MIN)
    {
        /* 正指令：电机正转 */
        Drv_PWM_Direction_Set(MOTOR_DIR_FORWARD);
        u16Duty = (uint16_t)s16SpeedPermyriad;
    }
    else if (s16SpeedPermyriad < -(int16_t)PWM_DUTY_CYCLE_MIN)
    {
        /* 负指令：电机反转 */
        Drv_PWM_Direction_Set(MOTOR_DIR_REVERSE);
        u16Duty = (uint16_t)(-s16SpeedPermyriad);
    }
    else
    {
        /* 指令为 0：失能电机（`Motor_Disable` 内将方向脚置为正向），输出 0 占空比 */
        Motor_Disable();
        return PENDULUM_SVC_OK;
    }

    /* 通过宏选择占空比路径：拟合反推或直通 */
#if MOTOR_PWM_USE_FIT_MAPPING
    /* 上位机下发“目标实际占空比”，这里反推“应给定占空比”用于 PWM 发生 */
    u16DutyOutput = Motor_DutyPermyriad_Calibrate_Map(u16Duty);
#else
    /* 直通模式：上位机下发即最终给定值 */
    u16DutyOutput = u16Duty;
#endif

    /* 非零占空比：确保驱动使能后再更新 PWM */
    Drv_PWM_Enable(PRJ_ENABLE);
    if (Drv_PWM_TargetPulse_Set((uint16_t)u16DutyOutput) != STATUS_OK)
    {
        return PENDULUM_SVC_ERR_DRIVER;
    }
    return PENDULUM_SVC_OK;
}

/* -------- 7.2 层内接口（Svc_Loc_*，与 .h 5.2 对应） -------- */
/* 无 */

/* ======================== 8. 私有函数实现 ======================== */
/**
 * @brief 电机停机去使能（本文件内）：关 PWM 使能、占空比置 0、方向置正向
 */
static void Motor_Disable(void)
{
    /* 先关使能，确保不会在停机过程中再输出 PWM */
    Drv_PWM_Enable(PRJ_DISABLE);
    /* 停机态仅预置寄存器目标值，不等待回调触发 */
    (void)Drv_PWM_TargetPulse_Set(0U);
    Drv_PWM_Direction_Set(MOTOR_DIR_FORWARD);
}

/**
 * @brief 将编码器协议解析结果映射为电机服务对外错误码
 * @param encRes 编码器协议栈返回码
 * @param encoderErr 非 OK/DRIVER 类错误时使用的编码器专项错误码（含 HEADER/CRC/SA_STATUS 等）
 * @return PENDULUM_SVC_OK / PENDULUM_SVC_ERR_DRIVER / encoderErr
 */
static PendulumServiceResult_t EncoderResult_Map(EncoderProtocolResult_t encRes,
                                                         PendulumServiceResult_t encoderErr)
{
    if (encRes == ENCODER_PROTOCOL_OK)
    {
        return PENDULUM_SVC_OK;
    }
    if (encRes == ENCODER_PROTOCOL_ERR_DRIVER)
    {
        return PENDULUM_SVC_ERR_DRIVER;
    }
    return encoderErr;
}

/**
 * @brief 解析编码器单帧数据（CM → CRC8 → SA 状态域 → 绝对位置）
 * @param pu8Frame 帧数据缓冲区（6字节）
 * @param u16Length 缓冲区长度
 * @param struOut 解析结果输出
 * @return ENCODER_PROTOCOL_OK；或 ERR_NULL / ERR_LENGTH / ERR_HEADER / ERR_CRC / ERR_SA_STATUS
 * @note  SA bit4/bit5 任一置位时返回 ERR_SA_STATUS，不读位置（与 §4.4.2.3.1 一致）
 */
static EncoderProtocolResult_t EncoderProtocol_Frame_Parse(const uint8_t *pu8Frame,
                                                          uint16_t u16Length,
                                                          EncoderFrame_t *struOut)
{
    uint8_t u8Cm, u8Sa, u8As0, u8As1, u8As2;
    uint32_t u32RawPosition;
    uint8_t u8CrcCalc, u8CrcRecv;

    if ((pu8Frame == NULL) || (struOut == NULL))
    {
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    if (u16Length < ENCODER_FRAME_LENGTH_BYTES)
    {
        return ENCODER_PROTOCOL_ERR_LENGTH;
    }

    /* 提取帧字段：CM(帧头) SA(状态) AS0~AS2(位置) CRC8(校验) */
    u8Cm = pu8Frame[0];
    u8Sa = pu8Frame[1];
    u8As0 = pu8Frame[2];
    u8As1 = pu8Frame[3];
    u8As2 = pu8Frame[4];
    u8CrcRecv = pu8Frame[5];

    /* 校验帧头 */
    if (u8Cm != ENCODER_CM_VALUE)
    {
        BSP_LOG_PRINTF("帧头错误: CM=0x%02X 期望=0x%02X 六字节整帧: %02X %02X %02X %02X %02X %02X",
               (unsigned int)u8Cm,
               (unsigned int)ENCODER_CM_VALUE,
               (unsigned int)pu8Frame[0],
               (unsigned int)pu8Frame[1],
               (unsigned int)pu8Frame[2],
               (unsigned int)pu8Frame[3],
               (unsigned int)pu8Frame[4],
               (unsigned int)pu8Frame[5]);
        return ENCODER_PROTOCOL_ERR_HEADER;
    }

    /* 校验CRC（前5字节） */
    u8CrcCalc = Cmn_CRC8_Calc(pu8Frame, 5U);
    if (u8CrcCalc != u8CrcRecv)
    {
        BSP_LOG_PRINTF("CRC校验失败: 计算=0x%02X 接收=0x%02X 六字节整帧: %02X %02X %02X %02X %02X %02X",
               (unsigned int)u8CrcCalc,
               (unsigned int)u8CrcRecv,
               (unsigned int)pu8Frame[0],
               (unsigned int)pu8Frame[1],
               (unsigned int)pu8Frame[2],
               (unsigned int)pu8Frame[3],
               (unsigned int)pu8Frame[4],
               (unsigned int)pu8Frame[5]);
        return ENCODER_PROTOCOL_ERR_CRC;
    }

    /* 解析状态标志位；异常时不读位置（§4.4.2.3.1） */
    struOut->u8CountErr      = (uint8_t)(((u8Sa & ENCODER_SA_COUNT_ERROR_BIT) != 0U) ? 1U : 0U);
    struOut->u8MtOrBattError = (uint8_t)(((u8Sa & ENCODER_SA_MT_BATT_ERR_BIT) != 0U) ? 1U : 0U);

    if ((u8Sa & (ENCODER_SA_COUNT_ERROR_BIT | ENCODER_SA_MT_BATT_ERR_BIT)) != 0U)
    {
        BSP_LOG_PRINTF("状态域异常: SA=0x%02X countErr=%u mtOrBatt=%u\n",
               (unsigned int)u8Sa,
               (unsigned int)struOut->u8CountErr,
               (unsigned int)struOut->u8MtOrBattError);
        return ENCODER_PROTOCOL_ERR_SA_STATUS;
    }

    /* 组合21位绝对位置（小端序） */
    u32RawPosition = (uint32_t)u8As0 | ((uint32_t)u8As1 << 8) | ((uint32_t)u8As2 << 16);
    struOut->u32AbsolutePosition = u32RawPosition & ENCODER_ABS_POSITION_MASK;

    return ENCODER_PROTOCOL_OK;
}

/**
 * @brief 读取电机编码器双帧数据
 * @param struOut 双帧数据输出
 * @return 解析结果
 */
static EncoderProtocolResult_t EncoderProtocol_Motor_Read(EncoderDual_t *struOut)
{
    uint8_t au8RawBuf[ENCODER_SNAPSHOT_BYTES];
    EncoderProtocolResult_t resLatest;
    EncoderProtocolResult_t resPrev;

    if (struOut == NULL)
    {
        BSP_LOG_PRINTF("指针为空！\n");
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    /* 从USART3获取12字节快照（前6=最新帧，后6=上一帧） */
    if (Drv_MotorEncoder_Data_Get(au8RawBuf) != STATUS_OK)
    {
        BSP_LOG_PRINTF("数据获取失败！\n");
        return ENCODER_PROTOCOL_ERR_DRIVER;
    }
    resLatest = EncoderProtocol_Frame_Parse(&au8RawBuf[0],
                                               ENCODER_FRAME_LENGTH_BYTES,
                                               &struOut->struLatest);
    resPrev   = EncoderProtocol_Frame_Parse(&au8RawBuf[ENCODER_FRAME_LENGTH_BYTES],
                                               ENCODER_FRAME_LENGTH_BYTES,
                                               &struOut->struPrevious);
    if (resLatest != ENCODER_PROTOCOL_OK)
    {
        BSP_LOG_PRINTF("电机编码器最新帧解析失败。\n");
        return resLatest;
    }
    if (resPrev != ENCODER_PROTOCOL_OK)
    {
        BSP_LOG_PRINTF("电机编码器上一帧解析失败。\n");
        return resPrev;
    }
    return ENCODER_PROTOCOL_OK;
}

/**
 * @brief 读取输出轴编码器双帧快照并解析
 * @details 倒立摆：17 位掩码；软尺摆：20 位掩码（与 `s_enControlObject` 一致）
 */
static EncoderProtocolResult_t EncoderProtocol_OutputShaft_Read(EncoderDual_t *struOut)
{
    uint8_t au8RawBuf[ENCODER_SNAPSHOT_BYTES];
    EncoderProtocolResult_t resLatest;
    EncoderProtocolResult_t resPrev;

    if (struOut == NULL)
    {
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    /* 从 UART5 获取 12 字节快照（前6=最新帧，后6=上一帧） */
    if (Drv_OutputShaftEncoder_Data_Get(au8RawBuf) != STATUS_OK)
    {
        return ENCODER_PROTOCOL_ERR_DRIVER;
    }
    resLatest = EncoderProtocol_Frame_Parse(&au8RawBuf[0],
                                               ENCODER_FRAME_LENGTH_BYTES,
                                               &struOut->struLatest);
    resPrev   = EncoderProtocol_Frame_Parse(&au8RawBuf[ENCODER_FRAME_LENGTH_BYTES],
                                               ENCODER_FRAME_LENGTH_BYTES,
                                               &struOut->struPrevious);
    if (resLatest != ENCODER_PROTOCOL_OK)
    {
        return resLatest;
    }
    if (resPrev != ENCODER_PROTOCOL_OK)
    {
        return resPrev;
    }
    /* 输出轴位数随控制对象：倒立摆 17 位，软尺摆 20 位 */
    {
        const uint32_t u32ShaftMask = OutputShaftPositionMask_Get();

        struOut->struLatest.u32AbsolutePosition   &= u32ShaftMask;
        struOut->struPrevious.u32AbsolutePosition &= u32ShaftMask;
    }
    return ENCODER_PROTOCOL_OK;
}

/**
 * @brief 读取摆臂编码器双帧快照并解析（17 位绝对位置）
 * @param struOut 输出：最新帧 + 上一帧；高位已按 17 位掩码处理，与 ENCODER_BITS_17 一致
 * @return 解析或驱动读取失败时的协议错误码
 */
static EncoderProtocolResult_t EncoderProtocol_SwingArm_Read(EncoderDual_t *struOut)
{
    uint8_t au8RawBuf[ENCODER_SNAPSHOT_BYTES];
    EncoderProtocolResult_t resLatest;
    EncoderProtocolResult_t resPrev;

    if (struOut == NULL)
    {
        return ENCODER_PROTOCOL_ERR_NULL;
    }

    /* 从 UART4 获取 12 字节快照（前6=最新帧，后6=上一帧） */
    if (Drv_SwingArmEncoder_Data_Get(au8RawBuf) != STATUS_OK)
    {
        return ENCODER_PROTOCOL_ERR_DRIVER;
    }
    resLatest = EncoderProtocol_Frame_Parse(&au8RawBuf[0],
                                               ENCODER_FRAME_LENGTH_BYTES,
                                               &struOut->struLatest);
    resPrev   = EncoderProtocol_Frame_Parse(&au8RawBuf[ENCODER_FRAME_LENGTH_BYTES],
                                               ENCODER_FRAME_LENGTH_BYTES,
                                               &struOut->struPrevious);
    if (resLatest != ENCODER_PROTOCOL_OK)
    {
        return resLatest;
    }
    if (resPrev != ENCODER_PROTOCOL_OK)
    {
        return resPrev;
    }
    /* 摆杆为 17 位编码器，屏蔽高位；与速度计算 ENCODER_BITS_17 一致，避免误用高电平位导致钳位失真 */
    struOut->struLatest.u32AbsolutePosition   &= ENCODER_ABS_POSITION_MAX_17BIT;
    struOut->struPrevious.u32AbsolutePosition &= ENCODER_ABS_POSITION_MAX_17BIT;
    return ENCODER_PROTOCOL_OK;
}

/**
 * @brief 角速度 float→int32：NaN/Inf 置 0，再按 Simulink 反馈允许范围饱和
 */
static int32_t EncoderSpeed_Int32_ClampFromFloat(float v, int32_t s32Lo, int32_t s32Hi)
{
    if ((v != v) || (v > 1.0e9f) || (v < -1.0e9f))
    {
        return 0;
    }
    if (v >= (float)s32Hi)
    {
        return s32Hi;
    }
    if (v <= (float)s32Lo)
    {
        return s32Lo;
    }
    return (int32_t)v;
}

/**
 * @brief 计算编码器角速度（基于前后两帧位置差）
 * @details 编码器位置范围0~max对应0~360°，自动处理过零情况。
 *          正转（位置增加）：deltaRaw>0 且不过零 → 速度为正；
 *          反转（位置减少）：deltaRaw<0 且不过零 → 速度为负，符合设计。
 * @note  17/20/21 位时半圈计数为 (max+1)/2；若 s32DeltaRaw 恰好等于 ±该半圈值，严格 `<`/`>` 比较不会进过零分支，
 *        会留下该量级的伪差分（坏帧或边界条件），电机/输出轴/摆杆均不在此函数内做额外毛刺剔除。
 * @param u32PosPrev 上一帧位置
 * @param u32PosCurr 当前帧位置
 * @param bits 编码器位数（21/20/17）
 * @return 角速度（°/s），正=正转、负=反转；双帧间隔固定 1ms
 */
static float EncoderSpeed_Calc(uint32_t u32PosPrev, uint32_t u32PosCurr, EncoderBits_t bits)
{
    uint32_t u32MaxVal;
    uint32_t u32Half;
    int32_t  s32DeltaRaw;
    int32_t  s32Delta;

    switch (bits)
    {
        case ENCODER_BITS_21:
        {
            u32MaxVal = ENCODER_ABS_POSITION_MAX_21BIT;
            break;
        }
        case ENCODER_BITS_20:
        {
            u32MaxVal = ENCODER_ABS_POSITION_MAX_20BIT;
            break;
        }
        case ENCODER_BITS_17:
        {
            u32MaxVal = ENCODER_ABS_POSITION_MAX_17BIT;
            break;
        }
        default:
        {
            return 0.0f;
        }
    }

    u32PosPrev = (u32PosPrev > u32MaxVal) ? u32MaxVal : u32PosPrev;
    u32PosCurr = (u32PosCurr > u32MaxVal) ? u32MaxVal : u32PosCurr;

    s32DeltaRaw = (int32_t)u32PosCurr - (int32_t)u32PosPrev;
    u32Half     = (u32MaxVal + 1U) >> 1;

    if (s32DeltaRaw > (int32_t)u32Half)
    {
        s32Delta = s32DeltaRaw - (int32_t)(u32MaxVal + 1U);
    }
    else if (s32DeltaRaw < -(int32_t)u32Half)
    {
        s32Delta = s32DeltaRaw + (int32_t)(u32MaxVal + 1U);
    }
    else
    {
        s32Delta = s32DeltaRaw;
    }

    return (float)s32Delta * ROUND_ANGLE / ENCODER_SPEED_FRAME_DT_SEC / (float)(u32MaxVal + 1U);
}

/**
 * @brief 计算电机编码器转速（21位） 
 * @param struDual 电机编码器双帧数据
 * @return 角速度（°/s）
 */
static float EncoderSpeed_Motor_Calc(const EncoderDual_t *struDual)
{
    if (struDual == NULL)
    {
        return 0.0f;
    }
    return EncoderSpeed_Calc(struDual->struPrevious.u32AbsolutePosition,
                             struDual->struLatest.u32AbsolutePosition,
                             ENCODER_BITS_21);
}

/**
 * @brief 计算输出轴编码器转速（位数随控制对象：倒立摆 17 / 软尺摆 20）
 */
static float EncoderSpeed_OutputShaft_Calc(const EncoderDual_t *struDual)
{
    if (struDual == NULL)
    {
        return 0.0f;
    }
    return EncoderSpeed_Calc(struDual->struPrevious.u32AbsolutePosition,
                             struDual->struLatest.u32AbsolutePosition,
                             OutputShaftEncoderBits_Get());
}

/**
 * @brief 计算摆臂编码器转速（17位）
 * @details 若本次计算得到的角速度绝对值大于 `PENDULUM_SPD_ABS_PRINTF_THRESHOLD_DPS`（°/s），
 *          经 `BSP_LOG_PRINTF` 打印上一帧与当前帧绝对位置，便于排查坏帧/过零异常。
 * @param struDual 摆臂编码器双帧数据
 * @return 角速度（°/s）
 */
static float EncoderSpeed_SwingArm_Calc(const EncoderDual_t *struDual)
{
    float fSpd;

    if (struDual == NULL)
    {
        return 0.0f;
    }

    fSpd = EncoderSpeed_Calc(struDual->struPrevious.u32AbsolutePosition,
                                 struDual->struLatest.u32AbsolutePosition,
                                 ENCODER_BITS_17);

    if (fSpd > PENDULUM_SPD_ABS_PRINTF_THRESHOLD_DPS || fSpd < -PENDULUM_SPD_ABS_PRINTF_THRESHOLD_DPS)
    {
        BSP_LOG_PRINTF("[摆杆速度] |spd|>%.0f deg/s: spd=%.2f 上一帧位置=%lu 当前帧位置=%lu\r\n",
               (double)PENDULUM_SPD_ABS_PRINTF_THRESHOLD_DPS,
               (double)fSpd,
               (unsigned long)struDual->struPrevious.u32AbsolutePosition,
               (unsigned long)struDual->struLatest.u32AbsolutePosition);
    }

    return fSpd;
}

/**
 * @brief 读取电机电流采样端电压（ADC1/ADC2 两路原始值平均后换算）
 * @return 采样端电压（V），ADC 读取失败时返回 0.0f
 */
static float MotorVoltage_Get(void)
{
    uint16_t au16RawAdc[2];

    if (Drv_ADC_Motor_RawData_Read(au16RawAdc) != STATUS_OK)
    {
        return 0.0f;
    }
    /* 两路 ADC 原始值（12 位）平均后转换为电压（参考电压 3.3V） */
    return ((float)au16RawAdc[0] + (float)au16RawAdc[1]) / 2.0f * ADC_REFERENCE_VOLTAGE_V / ADC_RAW_FULL_RESOLUTION;
}

/**
 * @brief 计算电机电流
 * @details 基于电压读数：电流 = (电压 - 偏置) / 放大倍数 / 采样电阻
 * @return 电机电流（A）
 */
static float MotorCurrent_Get(void)
{
#if MOTOR_CURRENT_ZERO_CALIB_ENABLE
    /* 采样结果减去零点偏置，得到校零后的电流 */
    return MotorCurrentRaw_Get() - s_fMotorCurrentZeroOffsetA;
#else
    /* 关闭零点标定时，直接返回原始电流 */
    return MotorCurrentRaw_Get();
#endif
}

/**
 * @brief 计算电机原始电流（未扣除零点偏置）
 * @details 基于电压读数：电流 = (电压 - 偏置) / 放大倍数 / 采样电阻
 * @return 电机原始电流（A）
 */
static float MotorCurrentRaw_Get(void)
{
    float fVadc  = MotorVoltage_Get();
    float fVDiff = fVadc - MOTOR_CURRENT_OFFSET_V;  /* 减去模拟前端偏置电压 */
    return fVDiff / MOTOR_CURRENT_GAIN / MOTOR_CURRENT_SHUNT_R;
}

/**
 * @brief 电机占空比标定映射（按上位机目标占空比曲线补偿）
 * @param u16DutyPermyriadAbs 绝对值占空比（0~10000，单位 0.01%）
 * @return 映射后的绝对值占空比（0~10000，单位 0.01%）
 */
static uint16_t Motor_DutyPermyriad_Calibrate_Map(uint16_t u16DutyPermyriadAbs)
{
    /* 标定表（单位：0.01%）
     * ls_actual_tbl：实际上位机目标占空比（期望实际输出）
     * ls_given_tbl ：反推得到的单片机给定占空比（用于 PWM 发生）
     * 拟合依据：最新标定表（含 1.3%->0% 锚点）
     * 保留逻辑：实际目标 >95% 时按 95% 处理（给定值可大于95%） */
    static const uint16_t ls_actual_tbl[] = {
        0U, 38U, 68U, 112U, 161U, 208U, 254U, 308U, 402U,
        499U, 899U, 1397U, 1895U, 2394U, 2893U, 3400U, 3900U,
        4894U, 5895U, 6895U, 7895U, 8895U, 9395U, 9500U
    };
    static const uint16_t ls_given_tbl[] = {
        130U, 140U, 150U, 200U, 250U, 300U, 350U, 400U, 500U,
        600U, 1000U, 1500U, 2000U, 2500U, 3000U, 3500U, 4000U,
        5000U, 6000U, 7000U, 8000U, 9000U, 9500U, 9590U
    };

    uint16_t u16I;
    const uint16_t u16TblSize = (uint16_t)(sizeof(ls_actual_tbl) / sizeof(ls_actual_tbl[0]));

    /* 特殊需求：上位机给定为 0 时绝对为0 */
    if (u16DutyPermyriadAbs == 0U)
    {
        return 0U;
    }

    /* 实际目标上限限制为 95.00%（给定值可大于95%） */
    if (u16DutyPermyriadAbs > 9500U)
    {
        u16DutyPermyriadAbs = 9500U;
    }

    if (u16DutyPermyriadAbs <= ls_actual_tbl[0])
    {
        return ls_given_tbl[0];
    }
    if (u16DutyPermyriadAbs >= ls_actual_tbl[u16TblSize - 1U])
    {
        return ls_given_tbl[u16TblSize - 1U];
    }

    for (u16I = 0U; u16I < (u16TblSize - 1U); u16I++)
    {
        uint16_t u16X0 = ls_actual_tbl[u16I];
        uint16_t u16X1 = ls_actual_tbl[u16I + 1U];
        if (u16DutyPermyriadAbs <= u16X1)
        {
            uint16_t u16Y0 = ls_given_tbl[u16I];
            uint16_t u16Y1 = ls_given_tbl[u16I + 1U];
            uint32_t u32Dx = (uint32_t)u16X1 - (uint32_t)u16X0;
            uint32_t u32Dy = (uint32_t)u16Y1 - (uint32_t)u16Y0;
            uint32_t u32Num = ((uint32_t)u16DutyPermyriadAbs - (uint32_t)u16X0) * u32Dy;
            /* 四舍五入的分段线性插值 */
            return (uint16_t)((uint32_t)u16Y0 + (u32Num + (u32Dx / 2U)) / u32Dx);
        }
    }

    return ls_given_tbl[u16TblSize - 1U];
}

static uint32_t OutputShaftPositionMask_Get(void)
{
    if (s_enControlObject == CONTROL_OBJECT_SOFT_RULER_PENDULUM)
    {
        return ENCODER_ABS_POSITION_MAX_20BIT;
    }
    return ENCODER_ABS_POSITION_MAX_17BIT;
}

static EncoderBits_t OutputShaftEncoderBits_Get(void)
{
    if (s_enControlObject == CONTROL_OBJECT_SOFT_RULER_PENDULUM)
    {
        return ENCODER_BITS_20;
    }
    return ENCODER_BITS_17;
}

/**
 * @brief 读取 ADC3 摆动电压并换算为 mV
 * @details Vin = (Vadc - 1.6) / 0.16，返回 mV；同时可选输出 ADC3 原始码
 */
static int32_t SoftRulerVoltageMilliVolt_Get(int32_t *ps32RawAdc)
{
    uint16_t u16Raw;
    float fVadc;
    float fVin;
    int32_t s32Mv;

    if (Drv_ADC_SoftRuler_RawData_Read(&u16Raw) != STATUS_OK)
    {
        if (ps32RawAdc != NULL)
        {
            *ps32RawAdc = 0;
        }
        return 0;
    }

    if (ps32RawAdc != NULL)
    {
        *ps32RawAdc = (int32_t)u16Raw;
    }

    fVadc = (float)u16Raw * ADC_REFERENCE_VOLTAGE_V / ADC_RAW_FULL_RESOLUTION;
    fVin = (fVadc - 1.6f) / 0.16f;
    s32Mv = (int32_t)(fVin * 1000.0f);
    return s32Mv;
}

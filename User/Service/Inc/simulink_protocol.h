/**
 * @file simulink_protocol.h
 * @brief 服务层：Simulink 上位机协议对外接口（控制帧解包、反馈帧发布）
 * @details 模块路径：`User/Service`。帧布局与 CRC 规则与上位机约定一致；底层收发由 `Core` 中 `usart` 驱动 `Drv_*` 完成。
 */

#ifndef SIMULINK_PROTOCOL_H
#define SIMULINK_PROTOCOL_H

/* ======================== 1. 头文件依赖 ======================== */
#include <stdint.h>

/* ======================== 2. 宏定义（对外可见） ======================== */
/** 控制帧总长度（字节），便于调用方分配缓冲区 */
#define SIMULINK_PROTOCOL_CONTROL_FRAME_SIZE  10U
/** 反馈帧总长度（字节），便于调用方分配缓冲区 */
#define SIMULINK_PROTOCOL_FEEDBACK_FRAME_SIZE 32U

/* ======================== 3. 类型定义 ======================== */
/**
 * @brief 协议操作结果枚举
 */
typedef enum
{
    SIMULINK_PROTOCOL_OK = 0,     /**< 成功 */
    SIMULINK_PROTOCOL_ERR_NULL,   /**< 指针为空 */
    SIMULINK_PROTOCOL_ERR_LENGTH, /**< 帧长度不足或非法 */
    SIMULINK_PROTOCOL_ERR_CRC,    /**< CRC 校验失败 */
    SIMULINK_PROTOCOL_ERR_RANGE,  /**< 参数超出约定范围 */
    SIMULINK_PROTOCOL_ERR_SEND    /**< 底层发送失败 */
} SimulinkProtocolResult_t;

/**
 * @brief 控制帧解析结果（上位机下发的控制量）
 * @note 载荷 4 字节在帧内顺序（紧跟 `LEN` 之后，小端）：**第 1 个 int16 → `s16Pwm`**，第 2 个 int16 → `s16Current`。
 *       应用层 `State_Machine` **仅将 `s16Pwm` 传给** `Svc_MotorService_SetMotorSpeedPermyriad`；`s16Current` 当前未参与调速。
 *       若 Simulink 把「-500」接到电流通道或字节顺序与约定不一致，则 **`s16Pwm` 可能仍为 0** → 固件按零转速 **停机去使能**。
 */
typedef struct
{
    int16_t s16Pwm;      /**< 转速指令万分比 -10000~10000（与 `Svc_MotorService_SetMotorSpeedPermyriad` 一致；字段名沿用协议历史） */
    int16_t s16Current; /**< 电流设定，范围 -2000~2000；协议字段已解析校验，当前固件未参与闭环（保留） */
} SimulinkProtocolControlData_t;

/**
 * @brief 反馈帧数据（本机上报给上位机的状态）
 */
typedef struct
{
    int16_t s16MotorCurrent;       /**< 电机电流，范围 -2000~2000 */
    int32_t s32MotorPosition;      /**< 电机位置，21 位编码器 */
    int32_t s32MotorSpeed;         /**< 电机转速 */
    int32_t s32AxisPosition;       /**< 轴位置，20 位编码器 */
    int32_t s32AxisSpeed;          /**< 轴转速 */
    int32_t s32PendulumPosition;   /**< 摆位置，17 位编码器 */
    int32_t s32PendulumSpeed;      /**< 摆转速 */
} SimulinkProtocolFeedbackData_t;

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */
/**
 * @brief 解包：从 USART2 控制通道取一帧并解析为控制量
 * @details 内部经驱动 `Drv_Simulink_ControlFrame_GetData` 取数，CRC 使用 BSP `Bsp_Crc16Modbus_Byte`。
 * @param[out] struOut 解析得到的控制数据
 * @return 解析结果枚举
 */
SimulinkProtocolResult_t Svc_SimulinkProtocol_UnpackControl(SimulinkProtocolControlData_t *struOut);

/**
 * @brief 发布反馈：校验范围、组帧并经 USART2 发送一帧
 * @details CRC16 由 BSP 计算，发送由 `Drv_Simulink_Feedback_Send`（DMA）完成。
 * @param[in] struIn 反馈数据
 * @return 校验、组帧或发送失败时的协议错误码
 */
SimulinkProtocolResult_t Svc_SimulinkProtocol_PublishFeedback(const SimulinkProtocolFeedbackData_t *struIn);

#endif /* SIMULINK_PROTOCOL_H */

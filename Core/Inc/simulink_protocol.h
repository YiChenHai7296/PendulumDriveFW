/**
 * @file simulink_protocol.h
 * @brief Simulink 上位机通信协议栈头文件，用于组帧与解帧，支持控制帧与反馈帧
 */

#ifndef SIMULINK_PROTOCOL_H
#define SIMULINK_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

#define NULL ((void *)0)

/**
 * @brief 协议操作结果枚举
 */
typedef enum
{
    SIMULINK_PROTOCOL_OK = 0,     /**< 成功 */
    SIMULINK_PROTOCOL_ERR_NULL,   /**< 指针为空 */
    SIMULINK_PROTOCOL_ERR_LENGTH, /**< 帧长度不足或非法 */
    SIMULINK_PROTOCOL_ERR_CRC,    /**< CRC 校验失败 */
    SIMULINK_PROTOCOL_ERR_RANGE   /**< 参数超出约定范围 */
} SimulinkProtocolResult_t;

/**
 * @brief 控制帧解析结果（上位机下发的控制量）
 */
typedef struct
{
    int16_t pwm;     /**< PWM 值，范围 -10000~10000 */
    int16_t current; /**< 电流设定，范围 -2000~2000 */
} SimulinkProtocolControlData_t;

/**
 * @brief 反馈帧数据（本机上报给上位机的状态）
 */
typedef struct
{
    int16_t motor_current;       /**< 电机电流，范围 -2000~2000 */
    int32_t motor_position;      /**< 电机位置，21 位编码器 */
    int32_t motor_speed;         /**< 电机转速 */
    int32_t axis_position;       /**< 轴位置，17 位编码器 */
    int32_t axis_speed;          /**< 轴转速 */
    int32_t pendulum_position;   /**< 摆位置，17 位编码器 */
    int32_t pendulum_speed;      /**< 摆转速 */
} SimulinkProtocolFeedbackData_t;

/** 控制帧总长度（字节），便于调用方分配缓冲区 */
#define SIMULINK_PROTOCOL_CONTROL_FRAME_SIZE  10U
/** 反馈帧总长度（字节），便于调用方分配缓冲区 */
#define SIMULINK_PROTOCOL_FEEDBACK_FRAME_SIZE 32U



/**
 * @brief 解帧：从底层获取一帧控制帧并解析，结果填入 pOut
 *        原始帧数据由底层提供（见 SimulinkProtocol_GetControlFrame）
 * @param[out] pOut 解析得到的控制数据
 * @return 解析结果
 */
SimulinkProtocolResult_t SimulinkProtocol_UnpackControl(SimulinkProtocolControlData_t *pOut);

/**
 * @brief 组帧：将反馈数据打包为反馈帧，写入底层提供的发送缓冲区
 *        发送缓冲区由底层提供（见 SimulinkProtocol_GetFeedbackTxBuffer）
 * @param[in] pIn 反馈数据
 * @return 打包结果
 */
SimulinkProtocolResult_t SimulinkProtocol_PackFeedback(const SimulinkProtocolFeedbackData_t *pIn);





#endif /* SIMULINK_PROTOCOL_H */

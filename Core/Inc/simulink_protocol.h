/**
 * @file simulink_protocol.h
 * @brief Simulink 上位机通信协议栈对外接口：组帧与解帧，支持控制帧与反馈帧
 */

#ifndef SIMULINK_PROTOCOL_H
#define SIMULINK_PROTOCOL_H

/* ======================== 1. 头文件依赖 ======================== */
#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

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
    int32_t axis_position;       /**< 轴位置，20 位编码器 */
    int32_t axis_speed;          /**< 轴转速 */
    int32_t pendulum_position;   /**< 摆位置，17 位编码器 */
    int32_t pendulum_speed;      /**< 摆转速 */
} SimulinkProtocolFeedbackData_t;

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */
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

/**
 * @brief 解帧：从底层获取一帧控制帧并解析，结果填入 pOut
 * @param[out] pOut 解析得到的控制数据
 * @return 解析结果
 */
SimulinkProtocolResult_t SimulinkProtocol_UnpackControl(SimulinkProtocolControlData_t *pOut);

/**
 * @brief 组帧：将反馈数据打包为反馈帧并发送
 * @param[in] pIn 反馈数据
 * @return 打包/发送结果
 */
SimulinkProtocolResult_t SimulinkProtocol_PackFeedback(const SimulinkProtocolFeedbackData_t *pIn);

/**
 * @brief 获取反馈帧发送缓冲区（由协议栈维护）
 * @return 指向长度为 SIMULINK_PROTOCOL_FEEDBACK_FRAME_SIZE 的缓冲区
 */
uint8_t *SimulinkProtocol_GetFeedbackTxBuffer(void);

/**
 * @brief 获取反馈帧发送长度（字节）
 * @return 固定为 SIMULINK_PROTOCOL_FEEDBACK_FRAME_SIZE
 */
uint16_t SimulinkProtocol_GetFeedbackTxLength(void);

#endif /* SIMULINK_PROTOCOL_H */

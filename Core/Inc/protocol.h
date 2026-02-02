/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   串口通信协议栈头文件
  *          用于组帧和解帧，支持控制帧和反馈帧
  *          协议层：只负责数据打包/解包，不涉及硬件操作
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* ===================== 协议帧格式定义 ===================== */
/* 帧头 */
#define PROTOCOL_HEAD_BYTE0    0x5A
#define PROTOCOL_HEAD_BYTE1    0xA5
#define PROTOCOL_HEAD_SIZE     2

/* 帧类型 */
#define PROTOCOL_TYPE_CONTROL  0x01    // 控制帧（外部接收）
#define PROTOCOL_TYPE_FEEDBACK 0x02    // 反馈帧（本机发送）

/* 负载长度 */
#define PROTOCOL_LEN_CONTROL   0x04    // 控制帧负载长度：4字节
#define PROTOCOL_LEN_FEEDBACK  0x1A    // 反馈帧负载长度：26字节

/* 帧总长度 */
#define PROTOCOL_FRAME_SIZE_CONTROL    (PROTOCOL_HEAD_SIZE + 1 + 1 + PROTOCOL_LEN_CONTROL + 2)   // 2+1+1+4+2=10字节
#define PROTOCOL_FRAME_SIZE_FEEDBACK   (PROTOCOL_HEAD_SIZE + 1 + 1 + PROTOCOL_LEN_FEEDBACK + 2)  // 2+1+1+26+2=32字节

/* ===================== 控制帧参数范围 ===================== */
#define PROTOCOL_PWM_MIN       -10000
#define PROTOCOL_PWM_MAX       10000
#define PROTOCOL_CURRENT_MIN   -2000
#define PROTOCOL_CURRENT_MAX   2000

/* ===================== 反馈帧参数范围 ===================== */
#define PROTOCOL_MOTOR_CURRENT_MIN     -2000
#define PROTOCOL_MOTOR_CURRENT_MAX     2000
#define PROTOCOL_MOTOR_POSITION_MIN    0
#define PROTOCOL_MOTOR_POSITION_MAX    2097151      // 21位编码器
#define PROTOCOL_MOTOR_SPEED_MIN       -40000
#define PROTOCOL_MOTOR_SPEED_MAX       40000
#define PROTOCOL_AXIS_POSITION_MIN     0
#define PROTOCOL_AXIS_POSITION_MAX     131072       // 17位编码器
#define PROTOCOL_AXIS_SPEED_MIN        -4000
#define PROTOCOL_AXIS_SPEED_MAX        4000
#define PROTOCOL_PENDULUM_POSITION_MIN 0
#define PROTOCOL_PENDULUM_POSITION_MAX 131072       // 17位编码器
#define PROTOCOL_PENDULUM_SPEED_MIN    -4000
#define PROTOCOL_PENDULUM_SPEED_MAX    4000

/* ===================== 数据结构定义 ===================== */
/**
  * @brief  控制帧参数结构体
  */
typedef struct
{
    int16_t pwm;        // PWM值，范围：-10000~10000
    int16_t current;    // 电流值，范围：-2000~2000
} Protocol_ControlFrame_t;

/**
  * @brief  反馈帧参数结构体
  */
typedef struct
{
    int16_t motor_current;      // 电机电流值，范围：-2000~2000
    int32_t motor_position;      // 电机位置，范围：0~2097151（21位编码器）
    int32_t motor_speed;         // 电机转速，范围：-40000~40000
    int32_t axis_position;       // 轴位置，范围：0~131072（17位编码器）
    int32_t axis_speed;          // 轴转速，范围：-4000~4000
    int32_t pendulum_position;   // 摆杆位置，范围：0~131072（17位编码器）
    int32_t pendulum_speed;      // 摆杆转速，范围：-4000~4000
} Protocol_FeedbackFrame_t;

/* ===================== 函数声明 ===================== */

/* ===================== 协议层对业务层的接口 ===================== */
/**
  * @brief  发送反馈帧（协议层接口，供业务层调用）
  * @param  pFeedbackFrame 反馈帧参数结构体指针
  * @param  timeout 发送超时时间（毫秒），仅阻塞模式有效
  * @retval true: 发送成功  false: 发送失败
  * @note   本函数内部完成组帧和发送，业务层无需关心底层实现
  */
bool Protocol_SendFeedbackFrame(const Protocol_FeedbackFrame_t *pFeedbackFrame, uint32_t timeout);

/**
  * @brief  处理接收到的控制帧（协议层接口，供业务层调用）
  * @param  pControlFrame 输出控制帧参数结构体指针
  * @retval true: 成功接收到有效控制帧  false: 无数据或解帧失败
  * @note   本函数内部完成从队列取数据和解帧，业务层无需关心底层实现
  */
bool Protocol_ProcessReceivedFrame(Protocol_ControlFrame_t *pControlFrame);

/* ===================== 协议层内部函数（驱动层可调用）===================== */
/**
  * @brief  打包反馈帧（协议层内部函数）
  * @param  pFeedbackFrame 反馈帧参数结构体指针
  * @param  pBuffer 输出缓冲区指针，至少需要PROTOCOL_FRAME_SIZE_FEEDBACK字节
  * @retval 实际打包的字节数，失败返回0
  */
uint16_t Protocol_PackFeedbackFrame(const Protocol_FeedbackFrame_t *pFeedbackFrame, uint8_t *pBuffer);

/**
  * @brief  解包控制帧（协议层内部函数）
  * @param  pBuffer 接收缓冲区指针，包含完整的控制帧数据
  * @param  bufferSize 缓冲区大小
  * @param  pControlFrame 输出控制帧参数结构体指针
  * @retval true: 解包成功  false: 解包失败
  */
bool Protocol_UnpackControlFrame(const uint8_t *pBuffer, uint16_t bufferSize, Protocol_ControlFrame_t *pControlFrame);

/**
  * @brief  CRC16校验计算（预留接口，由用户实现）
  * @param  pData 数据指针
  * @param  length 数据长度
  * @retval CRC16校验值
  */
uint16_t Protocol_CalculateCRC16(const uint8_t *pData, uint16_t length);

/**
  * @brief  验证帧头
  * @param  pBuffer 缓冲区指针
  * @retval true: 帧头正确  false: 帧头错误
  */
bool Protocol_VerifyHeader(const uint8_t *pBuffer);

/**
  * @brief  查找帧头位置
  * @param  pBuffer 缓冲区指针
  * @param  bufferSize 缓冲区大小
  * @retval 帧头位置索引，未找到返回-1
  */
int16_t Protocol_FindHeader(const uint8_t *pBuffer, uint16_t bufferSize);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H__ */

/**
 * @file motor_service.h
 * @brief 电机功能服务对外接口：反馈数据、占空比控制
 */

#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

/* ======================== 1. 头文件依赖 ======================== */
#include <stdint.h>
#include "usart.h"
#include "adc.h"
#include "hrtim.h"

/* ======================== 2. 宏定义（对外可见） ======================== */
/* 无 */

/* ======================== 3. 类型定义 ======================== */
/**
 * @brief 电机服务操作结果枚举
 */
typedef enum
{
    MOTOR_SVC_OK = 0,               /**< 成功 */
    MOTOR_SVC_ERR_PARAM,            /**< 参数非法（占空比超范围等） */
    MOTOR_SVC_ERR_DRIVER,           /**< 驱动层返回失败 */
    MOTOR_SVC_ERR_ENCODER_MOTOR,    /**< 电机编码器解析失败 */
    MOTOR_SVC_ERR_ENCODER_SHAFT,    /**< 输出轴编码器解析失败 */
    MOTOR_SVC_ERR_ENCODER_SWING,    /**< 摆臂编码器解析失败 */
    MOTOR_SVC_ERR_NULL              /**< 指针为空 */
} MotorServiceResult_t;

/**
 * @brief 电机反馈数据结构（与 Simulink 反馈帧字段一致，便于组帧上报）
 */
typedef struct
{
    int16_t motor_current;       /**< 电机电流，范围 -2000~2000 */
    int32_t motor_position;     /**< 电机位置，21 位编码器 */
    int32_t motor_speed;        /**< 电机转速（°/s） */
    int32_t axis_position;      /**< 轴位置，20 位编码器 */
    int32_t axis_speed;         /**< 轴转速（°/s） */
    int32_t pendulum_position;  /**< 摆位置，17 位编码器 */
    int32_t pendulum_speed;     /**< 摆转速（°/s） */
} MotorFeedbackData_t;

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */
/**
 * @brief 电机初始化：使能电机、方向设为正向、占空比设为 0
 * @note  应在系统上电或需要重新初始化电机时调用
 */
void MotorService_InitMotor(void);


/**
 * @brief 关闭电机: 电机失能、方向设为正向、占空比设为 0
 */
void MotorService_CloseMotor(void);

/**
 * @brief 电机电流零点标定：在电机未启动时采样零偏并保存
 * @note  建议在电机失能且静止时调用；初始化时会自动调用一次
 */
void MotorService_CalibrateCurrentZero(void);


/**
 * @brief 获取电机反馈数据：刷新编码器与电流，填位置/转速/电流到 pOut，供上位机或 Simulink 组帧
 * @param[out] pOut 反馈数据结构，与 SimulinkProtocolFeedbackData_t 布局一致
 * @return 操作结果，编码器解析失败时 pOut 可能包含无效数据
 */
MotorServiceResult_t MotorService_GetFeedbackData(MotorFeedbackData_t *pOut);

/**
 * @brief 电机转速/占空比控制：-10000~10000 对应 -100.00%~100.00%
 * @note  占空比正负用于区分电机转向：正为正转，负为反转
 * @param duty_permille 目标占空比指令，范围 -10000~10000
 * @return 操作结果
 */
MotorServiceResult_t MotorService_SetDutyCycle(int16_t duty_permille);

#endif /* MOTOR_SERVICE_H */

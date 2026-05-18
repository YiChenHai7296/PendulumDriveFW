/**
 * @file motor_service.h
 * @brief 服务层：电机相关语义接口（反馈、转速/占空比、电流零点标定）
 * @details 模块路径：`User/Service`。对上提供 `Svc_MotorService_*`，对下通过 `hrtim.h` / `usart.h` / `adc.h` 等
 *          驱动层 `Drv_*` 访问外设（具体 include 仅在对应 `.c`）。
 *          类型与返回值与 `common.h` 中 `Status_t`、`FunctionalState_t` 对齐。
 */

#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

/* ======================== 1. 头文件依赖 ======================== */
#include <stdint.h>
#include "common.h"

/* ======================== 2. 宏定义（对外可见） ======================== */
/**
 * @brief 电流零点偏置校准开关
 * @note  1: 启用校准，反馈电流扣除零点偏置
 *        0: 关闭校准，直接上报原始电流
 */
#define MOTOR_CURRENT_ZERO_CALIB_ENABLE   1U

/**
 * @brief PWM 指令映射模式开关
 * @note  1: 拟合反推（上位机给"目标实际占空比"，内部反推"应给定值"）
 *        0: 直通（上位机给定值即最终输出值）
 */
#define MOTOR_PWM_USE_FIT_MAPPING         1U

/** 转速指令万分比绝对值上限（与 Simulink 控制帧一致） */
#define SPEED_PERMYRIAD_MAX  (10000)

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
    int16_t s16MotorCurrent;       /**< 电机电流，范围 -2000~2000 */
    int32_t s32MotorPosition;      /**< 电机位置，21 位编码器 */
    int32_t s32MotorSpeed;         /**< 电机转速（°/s） */
    int32_t s32AxisPosition;       /**< 输出轴位置，17 位编码器 */
    int32_t s32AxisSpeed;          /**< 轴转速（°/s） */
    int32_t s32PendulumPosition;   /**< 摆位置，17 位编码器 */
    int32_t s32PendulumSpeed;      /**< 摆转速（°/s） */
} MotorFeedbackData_t;

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */
/**
 * @brief 电机电流零点标定（须在 PWM 失能、电机静止下调用，如上电主循环入口一次）
 * @note 受 `MOTOR_CURRENT_ZERO_CALIB_ENABLE` 控制；关闭时仅将偏置置 0
 */
void Svc_MotorService_CalibrateCurrentZero(void);

/**
 * @brief 获取电机反馈数据：读编码器与电流，填入输出结构
 * @param[out] struOut 反馈数据；与 `SimulinkProtocolFeedbackData_t` 字段布局一致，便于上层组帧
 * @return 操作结果；任一编码器解析失败时返回对应错误码，`struOut` 可能部分无效
 */
MotorServiceResult_t Svc_MotorService_GetFeedbackData(MotorFeedbackData_t *struOut);

/**
 * @brief 按转速万分比（permyriad）设置电机输出：±`SPEED_PERMYRIAD_MAX` 对应满量程方向，内部映射为 PWM
 * @note  正负用于区分电机转向：正为正转，负为反转
 * @param s16SpeedPermyriad 转速万分比整数，合法范围为 `[-SPEED_PERMYRIAD_MAX, SPEED_PERMYRIAD_MAX]`
 * @return 操作结果
 */
MotorServiceResult_t Svc_MotorService_SetMotorSpeedPermyriad(int16_t s16SpeedPermyriad);

#endif /* MOTOR_SERVICE_H */

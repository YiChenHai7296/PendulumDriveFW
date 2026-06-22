/**
 * @file    pendulum_service.h
 * @brief 服务层：摆系统驱动与反馈（电机占空比、编码器、电流、摆幅电压等）
 * @details 模块路径：`User/Service`。对上提供 `Svc_PendulumService_*`，对下通过 `hrtim.h` / `usart.h` / `adc.h` 等
 *          驱动层 `Drv_*` 访问外设（具体 include 仅在对应 `.c`）。
 *          类型与返回值与 `common.h` 中 `Status_t`、`FunctionalState_t` 对齐。
 */

#ifndef PENDULUM_SERVICE_H
#define PENDULUM_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 1. 头文件依赖 ======================== */
#include <stdint.h>
#include "common.h"
#include "config.h"   /* MOTOR_CURRENT_ZERO_CALIB_ENABLE / MOTOR_PWM_USE_FIT_MAPPING 等用户开关在 Config 模块定义 */

/* ======================== 2. 宏定义（对外可见） ======================== */
/* 用户开关（电流零点校准、PWM 映射模式）已集中至 `User/Config/Inc/config.h` */

/** 转速指令万分比绝对值上限（与 Simulink 控制帧一致） */
#define SPEED_PERMYRIAD_MAX  (10000)

/* ======================== 3. 类型定义 ======================== */
/**
 * @brief 摆服务操作结果枚举
 */
typedef enum
{
    PENDULUM_SVC_OK = 0,               /**< 成功 */
    PENDULUM_SVC_ERR_PARAM,            /**< 参数非法（转速超范围等） */
    PENDULUM_SVC_ERR_DRIVER,           /**< 驱动层返回失败 */
    PENDULUM_SVC_ERR_ENCODER_MOTOR,    /**< 电机编码器解析失败 */
    PENDULUM_SVC_ERR_ENCODER_SHAFT,    /**< 输出轴编码器解析失败 */
    PENDULUM_SVC_ERR_ENCODER_SWING,    /**< 摆臂编码器解析失败 */
    PENDULUM_SVC_ERR_NULL              /**< 指针为空 */
} PendulumServiceResult_t;

/**
 * @brief 摆系统反馈数据结构（与 Simulink 反馈帧字段一致，便于组帧上报）
 */
typedef struct
{
    int16_t s16MotorCurrent;       /**< 电机电流，范围 -2000~2000 */
    int32_t s32MotorPosition;      /**< 电机位置，21 位编码器 */
    int32_t s32MotorSpeed;         /**< 电机转速（°/s） */
    int32_t s32AxisPosition;       /**< 输出轴位置：倒立摆 17 位 / 软尺摆 20 位 */
    int32_t s32AxisSpeed;          /**< 轴转速（°/s） */
    int32_t s32PendulumPosition;   /**< 倒立摆: 摆位置(17 位编码器); 软尺摆: 摆动电压(mV) */
    int32_t s32PendulumSpeed;      /**< 倒立摆: 摆转速(°/s); 软尺摆: ADC3 原始码(0~4095) */
} PendulumFeedbackData_t;

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */

/* -------- 5.1 上层接口 -------- */

/**
 * @brief 电机电流零点标定（须在 PWM 失能、电机静止下调用，如上电主循环入口一次）
 * @note 受 `MOTOR_CURRENT_ZERO_CALIB_ENABLE` 控制；关闭时仅将偏置置 0
 */
void Svc_PendulumService_CurrentZero_Calibrate(void);

/**
 * @brief 获取电机反馈数据：读编码器与电流（摆杆量来源随控制对象），填入输出结构
 * @param[out] struOut 反馈数据；与 `SimulinkProtocolFeedbackData_t` 字段布局一致，便于上层组帧
 * @return 操作结果；任一编码器解析失败时返回对应错误码，`struOut` 可能部分无效
 */
PendulumServiceResult_t Svc_PendulumService_FeedbackData_Get(PendulumFeedbackData_t *struOut);

/**
 * @brief 设置当前控制对象（影响摆杆量反馈来源）
 * @param enObject 控制对象类型
 */
void Svc_PendulumService_ControlObject_Set(ControlObject_t enObject);

/**
 * @brief 按转速万分比（permyriad）设置电机输出：±`SPEED_PERMYRIAD_MAX` 对应满量程方向，内部映射为 PWM
 * @note  正负用于区分电机转向：正为正转，负为反转
 * @param s16SpeedPermyriad 转速万分比整数，合法范围为 `[-SPEED_PERMYRIAD_MAX, SPEED_PERMYRIAD_MAX]`
 * @return 操作结果
 */
PendulumServiceResult_t Svc_PendulumService_MotorSpeedPermyriad_Set(int16_t s16SpeedPermyriad);

/* -------- 5.2 层内接口（Svc_Loc_*） -------- */
/* 无 */

#ifdef __cplusplus
}
#endif

#endif /* PENDULUM_SERVICE_H */

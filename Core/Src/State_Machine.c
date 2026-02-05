/* USER CODE BEGIN Header */
 /**
  ******************************************************************************
  * @file    State_Machine.c
  * @brief   主状态机：处理 Simulink 上位机控制/反馈
  ******************************************************************************
  */
/* USER CODE END Header */

/* ======================== 1. 头文件依赖 ======================== */
#include "State_Machine.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* ======================== 2. 私有宏定义 ======================== */
/* 无 */

/* ======================== 3. 私有类型定义 ======================== */
/* 无 */

/* ======================== 4. 私有变量 ======================== */
/* 无 */

/* ======================== 5. 私有函数声明 ======================== */
static void StateMachine_OnDutyWriteFailed(MotorServiceResult_t ret, const SimulinkProtocolControlData_t *pCtrl);

/* ======================== 6. 接口函数实现 ======================== */

void StateMachine_MainLoop(void)
{
    SimulinkProtocolControlData_t  ctrl;      /* 解帧得到的控制帧数据（PWM、电流设定等） */
    SimulinkProtocolFeedbackData_t fb_tx;     /* 待发送的 Simulink 反馈帧载荷 */
    MotorFeedbackData_t            fb;        /* 电机反馈原始数据（位置、转速、电流等） */

    /* 阻塞循环：等待控制帧 -> 设置占空比 -> 采集反馈 -> 组帧发送 */
    for (;;)
    {
        /* 1) 等待并解帧：从 USART2 接收队列取一帧控制帧 */
        if (SimulinkProtocol_UnpackControl(&ctrl) != SIMULINK_PROTOCOL_OK)
        {
            /* 队列空 / 帧不完整 / CRC错误：继续等待下一帧 */
            continue;
        }

        /* 2) 根据控制帧 PWM(-10000~10000) 设置占空比 */
        if (MotorService_SetDutyCycle(ctrl.pwm) != MOTOR_SVC_OK)
        {
            //StateMachine_OnDutyWriteFailed(motorRet, &ctrl);
            continue;
        }

        /* 4) 写入成功：获取电机反馈数据 */
        if (MotorService_GetFeedbackData(&fb) != MOTOR_SVC_OK)
        {
            /* 编码器解析失败：可选进入安全态或上报，此处继续使用已有 fb 并组帧 */
            continue;
        }

        /* 5) 填充 Simulink 反馈载荷并组帧 */
        fb_tx.motor_current     = fb.motor_current;
        fb_tx.motor_position    = fb.motor_position;
        fb_tx.motor_speed       = fb.motor_speed;
        fb_tx.axis_position     = fb.axis_position;
        fb_tx.axis_speed        = fb.axis_speed;
        fb_tx.pendulum_position = fb.pendulum_position;
        fb_tx.pendulum_speed    = fb.pendulum_speed;

        if (SimulinkProtocol_PackFeedback(&fb_tx) != SIMULINK_PROTOCOL_OK)
        {
            /* 组帧失败：暂不处理，等待下一次控制 */
            continue;
        }
    }
}

/* USER CODE BEGIN Implementation */

/* USER CODE END Implementation */

/* ======================== 7. 私有函数实现 ======================== */

static void StateMachine_OnDutyWriteFailed(MotorServiceResult_t ret,       /* 电机服务错误码 */
                                          const SimulinkProtocolControlData_t *pCtrl)  /* 失败时的控制帧 */
{
    (void)ret;
    (void)pCtrl;
    /* TODO: 错误处理占位（记录错误码/进入安全状态/上报错误帧等） */
}

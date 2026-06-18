/**
 * @file    State_Machine.c
 * @brief   应用层：主循环状态机实现
 * @details 阻塞循环内顺序为：服务层解 Simulink 控制帧 → 按万分比设速 →
 *          读电机反馈 → 服务层组帧并上报反馈。任一步失败则 `continue` 等待下一帧。
 *          不直接调用 `Core` 中 `Drv_*`；外设与协议收发由服务层经驱动层完成。
 */

/* ======================== 1. 头文件引用 ======================== */
#include "State_Machine.h"
#include "simulink_protocol.h"
#include "pendulum_service.h"
#include "bsp.h"

/* ======================== 2. 私有宏定义 ======================== */
#if (APP_CONTROL_OBJECT_SELECT == APP_CONTROL_OBJECT_INVERTED_PENDULUM)
#define APP_CONTROL_OBJECT_VALUE CONTROL_OBJECT_INVERTED_PENDULUM
#elif (APP_CONTROL_OBJECT_SELECT == APP_CONTROL_OBJECT_SOFT_RULER_PENDULUM)
#define APP_CONTROL_OBJECT_VALUE CONTROL_OBJECT_SOFT_RULER_PENDULUM
#else
#error "APP_CONTROL_OBJECT_SELECT 配置非法"
#endif

/* ======================== 3. 私有类型定义 ======================== */
/* 无 */

/* ======================== 4. 对外变量定义 ======================== */
/* 无 */

/* ======================== 5. 私有变量 ======================== */
/* 无 */

/* ======================== 6. 私有函数声明 ======================== */
/**
 * @brief 反馈数据映射：服务层 PendulumFeedbackData_t → 协议层 SimulinkProtocolFeedbackData_t
 * @note  App 层作为两层适配器：本函数集中处理跨层字段映射，保持服务/协议互不依赖
 */
static void Feedback_Map(const PendulumFeedbackData_t *pstruSrc,
                            SimulinkProtocolFeedbackData_t *pstruDst);

/* ======================== 7. 接口函数实现 ======================== */

/* -------- 7.1 上层接口（与 .h 5.1 对应） -------- */

/**
 * @brief 主状态机死循环：解控制帧 → 设转速 → 读反馈 → 上报
 * @details IWDG 由 `HAL_TIM_PeriodElapsedCallback` 中 TIM2 更新事件周期调用 `HAL_IWDG_Refresh` 喂狗。
 *          `Svc_SimulinkProtocol_Control_Unpack` 在队列空时快速返回非 OK，属正常空转。
 */
void App_StateMachine_MainLoop(void)
{
    SimulinkProtocolControlData_t  struCtrl;   /* 解帧：转速万分比、电流设定等（当前主循环仅将转速下发电机） */
    SimulinkProtocolFeedbackData_t struFbTx;   /* 待上报的 Simulink 反馈载荷 */
    PendulumFeedbackData_t            struFb;     /* 摆服务汇总的反馈（位置、转速、电流等） */


    Svc_PendulumService_ControlObject_Set(APP_CONTROL_OBJECT_VALUE);
    Svc_PendulumService_CurrentZero_Calibrate(); /* 电机电流采样零点标定 */

    for (;;)
    {
        /* 1) 服务层解控制帧（内部经驱动从 USART2 取原始字节、CRC 校验等，见 simulink_protocol） */
        if (Svc_SimulinkProtocol_Control_Unpack(&struCtrl) != SIMULINK_PROTOCOL_OK)
        {
            /* 队列空、长度/CRC/类型错误等：等待下一帧 */
            continue;
        }

        /* 2) 按控制帧转速万分比设置电机（内部经驱动层映射为 HRTIM PWM） */
      
        if (Svc_PendulumService_MotorSpeedPermyriad_Set(struCtrl.s16Pwm) != PENDULUM_SVC_OK)
        {
            continue;
        }

        /* 3) 读取编码器与电流等反馈（经驱动层 usart/adc） */
        if (Svc_PendulumService_FeedbackData_Get(&struFb) != PENDULUM_SVC_OK)
        {
            BSP_LOG_PRINTF("反馈读取失败\n");
            continue;
        }

        /* 4) 服务反馈 → 协议反馈载荷映射 */
        Feedback_Map(&struFb, &struFbTx);

        /* 5) 组帧并经 USART2 DMA 发送反馈 */
        if (Svc_SimulinkProtocol_Feedback_Publish(&struFbTx, APP_CONTROL_OBJECT_VALUE) != SIMULINK_PROTOCOL_OK)
        {
            BSP_LOG_PRINTF("组帧失败！\n");
            continue;
        }
    }
}

/* USER CODE BEGIN Implementation */

/* USER CODE END Implementation */

/* -------- 7.2 层内接口（App_Loc_*，与 .h 5.2 对应） -------- */
/* 无 */

/* ======================== 8. 私有函数实现 ======================== */
static void Feedback_Map(const PendulumFeedbackData_t *pstruSrc,SimulinkProtocolFeedbackData_t *pstruDst)
{
    if(pstruDst == NULL)
    {
        return;
    }
    
    pstruDst->s16MotorCurrent     = pstruSrc->s16MotorCurrent;
    pstruDst->s32MotorPosition    = pstruSrc->s32MotorPosition;
    pstruDst->s32MotorSpeed       = pstruSrc->s32MotorSpeed;
    pstruDst->s32AxisPosition     = pstruSrc->s32AxisPosition;
    pstruDst->s32AxisSpeed        = pstruSrc->s32AxisSpeed;
    pstruDst->s32PendulumPosition = pstruSrc->s32PendulumPosition;
    pstruDst->s32PendulumSpeed    = pstruSrc->s32PendulumSpeed;
    
    return;
}

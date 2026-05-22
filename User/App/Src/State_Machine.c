/* USER CODE BEGIN Header */
/**
 * @file    State_Machine.c
 * @brief   应用层：主循环状态机实现
 * @details 阻塞循环内顺序为：服务层解 Simulink 控制帧 → 按万分比设速 →
 *          读电机反馈 → 服务层组帧并上报反馈。任一步失败则 `continue` 等待下一帧。
 *          不直接调用 `Core` 中 `Drv_*`；外设与协议收发由服务层经驱动层完成。
 */
/* USER CODE END Header */

/* ======================== 1. 头文件引用 ======================== */
#include "State_Machine.h"
#include "simulink_protocol.h"
#include "motor_service.h"
#include "bsp.h"
#include <stdio.h>

/* USER CODE BEGIN Includes */
/* 本层仅依赖服务层；类型定义在服务/公共头文件中 */
/* USER CODE END Includes */

/* ======================== 3. 私有类型定义 ======================== */
/* 无 */

/* ======================== 4. 私有变量 ======================== */
/* 无 */

/* ======================== 5. 对外变量定义 ======================== */
volatile uint32_t g_u32AppMainLoopLastUs        = 0U;
volatile uint32_t g_u32AppMainLoopMaxUs        = 0U;
volatile uint32_t g_u32AppMainLoopMinUs        = 0xFFFFFFFFU;
volatile uint32_t g_u32AppMainLoopSampleCount  = 0U;

/* ======================== 6. 私有函数声明 ======================== */
static void App_StateMachine_RecordLoopElapsedUs(uint32_t u32StartCycles);

/* ======================== 7. 接口函数实现 ======================== */
/**
 * @brief 主状态机死循环：解控制帧 → 设转速 → 读反馈 → 上报
 * @details IWDG 由 `HAL_TIM_PeriodElapsedCallback` 中 TIM2 更新事件周期调用 `HAL_IWDG_Refresh` 喂狗。
 *          `Svc_SimulinkProtocol_UnpackControl` 在队列空时快速返回非 OK，属正常空转。
 */
void App_StateMachine_MainLoop(void)
{
    SimulinkProtocolControlData_t  struCtrl;   /* 解帧：转速万分比、电流设定等（当前主循环仅将转速下发电机） */
    SimulinkProtocolFeedbackData_t struFbTx;   /* 待上报的 Simulink 反馈载荷 */
    MotorFeedbackData_t            struFb;     /* 电机服务汇总的反馈（位置、转速、电流等） */


    Svc_MotorService_CalibrateCurrentZero(); /* 电机电流采样零点标定 */

    Bsp_Profile_Init();

    for (;;)
    {
        uint32_t u32LoopStartCycles;

        /* 计时起点：取控制帧（接收/出队）之前 */
        u32LoopStartCycles = Bsp_Profile_GetCycles();

        /* 1) 服务层解控制帧（内部经驱动从 USART2 取原始字节、CRC 校验等，见 simulink_protocol） */
        if (Svc_SimulinkProtocol_UnpackControl(&struCtrl) != SIMULINK_PROTOCOL_OK)
        {
            /* 队列空、长度/CRC/类型错误等：丢弃本次计时 */
            continue;
        }

        /* 2) 按控制帧转速万分比设置电机（内部经驱动层映射为 HRTIM PWM） */
        if (Svc_MotorService_SetMotorSpeedPermyriad(struCtrl.s16Pwm) != MOTOR_SVC_OK)
        {
            continue;
        }

        /* 3) 读取编码器与电流等反馈（经驱动层 usart/adc） */
        if (Svc_MotorService_GetFeedbackData(&struFb) != MOTOR_SVC_OK)
        {
            printf("反馈读取失败\n");
            continue;
        }

        /* 4) 组装反馈载荷（字段与协议结构体一致） */
        struFbTx.s16MotorCurrent     = struFb.s16MotorCurrent;
        struFbTx.s32MotorPosition    = struFb.s32MotorPosition;
        struFbTx.s32MotorSpeed       = struFb.s32MotorSpeed;
        struFbTx.s32AxisPosition     = struFb.s32AxisPosition;
        struFbTx.s32AxisSpeed        = struFb.s32AxisSpeed;
        struFbTx.s32PendulumPosition = struFb.s32PendulumPosition;
        struFbTx.s32PendulumSpeed    = struFb.s32PendulumSpeed;

        /* 5) 组帧并经 USART2 DMA 发送反馈 */
        if (Svc_SimulinkProtocol_PublishFeedback(&struFbTx) != SIMULINK_PROTOCOL_OK)
        {
            printf("组帧失败！\n");
            continue;
        }

        /* 完整走完：统计耗时并打印 */
        App_StateMachine_RecordLoopElapsedUs(u32LoopStartCycles);
        printf("主循环耗时 us: last=%u max=%u min=%u\r\n",
               (unsigned int)g_u32AppMainLoopLastUs,
               (unsigned int)g_u32AppMainLoopMaxUs,
               (unsigned int)g_u32AppMainLoopMinUs);
    }
}

static void App_StateMachine_RecordLoopElapsedUs(uint32_t u32StartCycles)
{
    uint32_t u32DeltaCycles;
    uint32_t u32ElapsedUs;

    u32DeltaCycles = Bsp_Profile_GetCycles() - u32StartCycles;
    u32ElapsedUs   = Bsp_Profile_CyclesToUs(u32DeltaCycles);

    g_u32AppMainLoopLastUs = u32ElapsedUs;
    if (u32ElapsedUs > g_u32AppMainLoopMaxUs)
    {
        g_u32AppMainLoopMaxUs = u32ElapsedUs;
    }
    if (u32ElapsedUs < g_u32AppMainLoopMinUs)
    {
        g_u32AppMainLoopMinUs = u32ElapsedUs;
    }
    g_u32AppMainLoopSampleCount++;
}

/* USER CODE BEGIN Implementation */

/* USER CODE END Implementation */

/* ======================== 8. 私有函数实现 ======================== */
/* 无 */

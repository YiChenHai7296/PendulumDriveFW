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
#include "adc.h"

/* ADC调试打印模式：
   0 = 关闭ADC调试打印
   1 = 打印 ADC3_IN1 raw/voltage
   2 = 打印原“ADC当前读数/误差/最大最小值”格式 */
#define ADC_DEBUG_PRINT_MODE 0
#define ADC_DEBUG_PRINT_PERIOD_MS 500U

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
    uint32_t adc3_print_tick = 0U;
#if (ADC_DEBUG_PRINT_MODE == 2)
    float ADC_test_max = 0.0f;
    float ADC_test_min = 3.3f;
    float ADC_diffmx = 0.0f;
#endif
    /* 阻塞循环：等待控制帧 -> 设置占空比 -> 采集反馈 -> 组帧发送 */
    for (;;)
    {
        /* 非阻塞调试打印：按宏切换不同打印格式 */
        if ((HAL_GetTick() - adc3_print_tick) >= ADC_DEBUG_PRINT_PERIOD_MS)
        {
#if (ADC_DEBUG_PRINT_MODE == 1)
            uint16_t adc3_raw = ADC3_ReadIn1Raw();
            if (adc3_raw == 0xFFFFU)
            {
                printf("ADC3_IN1 read error\r\n");
            }
            else
            {
                float adc3_v = ((float)adc3_raw * 3.3f) / 4095.0f;
                printf("ADC3_IN1 raw=%u, voltage=%.4fV\r\n", adc3_raw, adc3_v);
            }
#elif (ADC_DEBUG_PRINT_MODE == 2)
            float adc_test_data = ADC_Read_MotorVol();
            float adc_diff = 0.0f;
            float adc_diff16 = adc_test_data - 1.6f;
            if (adc_test_data > ADC_test_max)
            {
                ADC_test_max = adc_test_data;
            }
            if (adc_test_data < ADC_test_min)
            {
                ADC_test_min = adc_test_data;
            }
            adc_diff = ADC_test_max - ADC_test_min;
            if (adc_diff16 < 0.0f)
            {
                adc_diff16 = -adc_diff16;
            }
            if (ADC_diffmx < adc_diff16)
            {
                ADC_diffmx = adc_diff16;
            }
            printf("ADC当前读数：%f ,误差 = %f , 最大误差 = %f   |  最大值：%f , 最小值：%f , 差值：%f ,\n",
                   adc_test_data, adc_diff16, ADC_diffmx, ADC_test_max, ADC_test_min, adc_diff);
#else
            /* ADC_DEBUG_PRINT_MODE == 0: no print */
#endif
            adc3_print_tick = HAL_GetTick();
        }
        /* 1) 等待并解帧：从 USART2 接收队列取一帧控制帧 */
        if (SimulinkProtocol_UnpackControl(&ctrl) != SIMULINK_PROTOCOL_OK)
        {
						//printf("error\n");
						//printf("指令有误！\n");
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
        #if 1
        fb_tx.motor_current     = fb.motor_current;
        fb_tx.motor_position    = fb.motor_position;
        fb_tx.motor_speed       = fb.motor_speed;
        fb_tx.axis_position     = fb.axis_position;
        fb_tx.axis_speed        = fb.axis_speed;
        fb_tx.pendulum_position = fb.pendulum_position;
        fb_tx.pendulum_speed    = fb.pendulum_speed;
        #endif
        //printf("fb.motor_current = %d\n",fb_tx.motor_current);
#if 0
        fb_tx.motor_current     = 0;
        fb_tx.motor_position    = 1;
        fb_tx.motor_speed       = 2;
        fb_tx.axis_position     = 3;
        fb_tx.axis_speed        = 4;
        fb_tx.pendulum_position = 5;
        fb_tx.pendulum_speed    = 6;
#endif

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

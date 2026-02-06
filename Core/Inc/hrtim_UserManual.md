HRTIM 应用驱动模块说明（hrtim.c / hrtim.h）
Rev 1

================================================================================
一、数据结构
================================================================================

1. 对外

hhrtim1
  类型：extern HRTIM_HandleTypeDef
  定义位置：在 hrtim.h 中声明，在 hrtim.c 中定义。
  说明：HRTIM1 外设句柄，供 HAL HRTIM 接口及本模块使用。

MOTOR_DIR_FORWARD
  类型：宏
  值：0U
  说明：电机正转方向。

MOTOR_DIR_REVERSE
  类型：宏
  值：1U
  说明：电机反转方向。


2. 私有

u16TargePulse
  类型：unsigned short
  定义位置：仅在 hrtim.c 中定义，头文件未声明。
  说明：PWM 占空比目标值，范围 0～10000，对应 0.00%～100.00%。由 PWM_Set_TargePulse() 或 PWM_TEST() 写入，在 HAL_HRTIM_Compare1EventCallback()（Timer B）中与 u8FlagPulse 配合使用，经换算后调用 User_Func_SetPulse() 更新 HRTIM 比较寄存器。

u8FlagPulse
  类型：unsigned char
  定义位置：仅在 hrtim.c 中定义，头文件未声明。
  说明：PWM 占空比待更新标志。置 1 表示有待更新的目标占空比；在 Timer B 比较 1 回调中处理后被清零。


================================================================================
二、函数
================================================================================

1. 对外

MX_HRTIM1_Init
  原型：void MX_HRTIM1_Init(void);
  功能：初始化 HRTIM1，配置时基、主定时器、Timer A/B、比较值、TA1 输出（Set=周期事件，Reset=比较1事件）、ADC 触发等，并调用 HAL_HRTIM_MspPostInit() 完成 GPIO 等配置。
  参数：无。
  返回值：无。
  注意：内部在初始化失败时会调用 Error_Handler()；应在系统上电或启动阶段调用一次。

HAL_HRTIM_MspPostInit
  原型：void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);
  功能：HRTIM 外设的 MSP 后初始化，主要配置与 HRTIM 相关的 GPIO（如 PA8 复用为 HRTIM1_CHA1）。
  参数：hhrtim，类型 HRTIM_HandleTypeDef *，HRTIM 句柄，本工程中为 &hhrtim1。
  返回值：无。
  注意：通常由 MX_HRTIM1_Init() 内部调用，用户无需单独调用。

PWM_Enable
  原型：void PWM_Enable(FunctionalState NewState);
  功能：PWM 使能控制，控制电机使能引脚 MotorEnableControl_Pin。使能时置高，失能时置低。
  参数：NewState，类型 FunctionalState，使能状态，取 ENABLE 或 DISABLE。
  返回值：无。

PWM_DirControl
  原型：void PWM_DirControl(uint8_t dir);
  功能：电机方向控制，控制电机方向引脚 MotorDirectionControl_Pin。正转时置 0，反转时置 1。
  参数：dir，类型 uint8_t，方向，MOTOR_DIR_FORWARD（0）正转，MOTOR_DIR_REVERSE（1）反转。
  返回值：无。

PWM_Set_TargePulse
  原型：unsigned char PWM_Set_TargePulse(unsigned short u16ExpectedValue);
  功能：设置 PWM 占空比目标值（0～10000 对应 0.00%～100.00%），并置位待更新标志；实际占空比在 HRTIM Timer B 比较 1 中断回调中通过 User_Func_SetPulse() 写入 HRTIM 比较寄存器。
  参数：u16ExpectedValue，类型 unsigned short，目标占空比数值，范围 0～10000。
  返回值：0 表示成功，1 表示参数越界（<0 或 >10000）。
  注意：内部会设置 u16TargePulse 和 u8FlagPulse=1；在 Timer B 的 CMP1 回调中再按 u16TargePulse*1.7f 换算为 9～17000 的脉宽并更新 PWM。


2. 私有

User_Func_SetPulse
  原型：unsigned char User_Func_SetPulse(unsigned short u16T2Pulse);
  功能：直接设置 HRTIM Timer A 的 PWM 脉宽（比较寄存器 CMP1/CMP2/CMP3），从而设定 TA1 占空比及关联的 ADC 触发点。脉宽以 Timer A 计数值表示，范围 9～17000。
  参数：u16T2Pulse，类型 unsigned short，高电平计数值，范围 9～17000（与 Timer A 周期 17000 对应）。
  返回值：0 表示成功，1 表示参数越界（会通过 printf 输出“占空比参数越界”）。
  说明：仅在 hrtim.c 中实现，头文件未声明。本模块内由 HAL_HRTIM_Compare1EventCallback()（Timer B CMP1）中根据 u16TargePulse 调用（换算 u16TargePulse*1.7f）。若其他模块需直接按硬件计数值设置占空比，可自行声明后调用。

PWM_TEST
  原型：void PWM_TEST(void);
  功能：通过调试串口交互的占空比测试：预读一帧数据、提示输入 5 位占空比值（00000～10000），接收后写入 u16TargePulse 并置 u8FlagPulse=1，由后续 Timer B 比较 1 回调完成实际 PWM 更新。
  参数：无。
  返回值：无。
  说明：仅在 hrtim.c 中实现，头文件未声明。依赖 DEBUG_UART、u8DebugRxBuff 等，仅用于调试。

HAL_HRTIM_Compare1EventCallback
  原型：void HAL_HRTIM_Compare1EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx);
  功能：HRTIM 比较 1 事件回调。本模块仅处理 Timer B：若 u8FlagPulse==1，则调用 User_Func_SetPulse(u16TargePulse*1.7f) 更新 PWM，清除 u8FlagPulse，并打印成功/失败信息。
  参数：hhrtim，类型 HRTIM_HandleTypeDef *，HRTIM 句柄；TimerIdx，类型 uint32_t，定时器索引（如 HRTIM_TIMERINDEX_TIMER_B）。
  返回值：无。
  说明：由 HAL 在比较 1 事件时调用，用户一般不直接调用。


================================================================================
依赖说明：HAL 提供 HRTIM_HandleTypeDef、HAL_HRTIM_* 等；main.h 提供 FunctionalState、MotorEnableControl_GPIO_Port/Pin、MotorDirectionControl_GPIO_Port/Pin；usart 或其它模块提供 DEBUG_UART、u8DebugRxBuff（PWM_TEST 及回调中的 printf 使用）。
================================================================================

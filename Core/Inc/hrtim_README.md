# HRTIM 模块说明（hrtim.c / hrtim.h）

本文档基于 `Core/Src/hrtim.c` 与 `Core/Inc/hrtim.h` 整理，从**数据类型**和**接口函数**两方面进行说明。

---

## 一、数据类型与常量

### 1.1 头文件中声明的类型与宏

| 名称 | 类型/含义 | 说明 |
|------|-----------|------|
| `HRTIM_HandleTypeDef` | 类型（来自 HAL） | HRTIM 外设句柄，用于初始化与 HAL 调用。 |
| `hhrtim1` | `extern HRTIM_HandleTypeDef` | HRTIM1 的全局句柄，在 `hrtim.c` 中定义，在 `hrtim.h` 中声明，供其他模块使用。 |
| `MOTOR_DIR_FORWARD` | 宏，值为 `0U` | 电机正转方向。 |
| `MOTOR_DIR_REVERSE` | 宏，值为 `1U` | 电机反转方向。 |

### 1.2 本模块内部使用的变量（仅在 hrtim.c 中）

以下变量未在头文件中声明，仅在 `hrtim.c` 内使用，供本模块逻辑与回调使用：

| 名称 | 类型 | 说明 |
|------|------|------|
| `u16TargePulse` | `unsigned short` | PWM 占空比目标值，范围 0~10000（对应 0.00%~100.00%）。 |
| `u8FlagPulse` | `unsigned char` | PWM 占空比“待更新”标志：置 1 表示有待更新的目标占空比，在 Timer B 比较 1 回调中处理后被清零。 |

### 1.3 依赖的外部类型（来自 main.h / HAL）

- `FunctionalState`：使能状态（`ENABLE` / `DISABLE`），用于 `PWM_Enable`。
- `GPIO_PIN_SET` / `GPIO_PIN_RESET`、`MotorEnableControl_GPIO_Port` / `MotorEnableControl_Pin`、`MotorDirectionControl_GPIO_Port` / `MotorDirectionControl_Pin`：来自 `main.h`，用于电机使能与方向控制。

---

## 二、接口函数

### 2.1 在 hrtim.h 中声明的对外接口

这些函数在头文件中有原型声明，供其他 .c 文件调用。

#### 2.1.1 `MX_HRTIM1_Init`

```c
void MX_HRTIM1_Init(void);
```

- **功能**：初始化 HRTIM1（时基、Timer A/B、比较值、TA1 输出、ADC 触发等）。  
- **调用时机**：系统上电/启动时调用一次。  
- **说明**：内部会调用 `HAL_HRTIM_MspPostInit` 完成 GPIO（如 PA8 映射为 HRTIM1_CHA1）等配置。

---

#### 2.1.2 `HAL_HRTIM_MspPostInit`

```c
void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);
```

- **功能**：HRTIM 外设的 MSP 后初始化，主要配置与 HRTIM 相关的 GPIO（如 PA8 复用为 HRTIM1_CHA1）。  
- **参数**：`hhrtim` — HRTIM 句柄（本工程中即 `&hhrtim1`）。  
- **说明**：通常由 `MX_HRTIM1_Init` 内部调用，用户一般无需单独调用。

---

#### 2.1.3 `PWM_Enable`

```c
void PWM_Enable(FunctionalState NewState);
```

- **功能**：PWM 使能控制（控制电机使能引脚）。  
- **参数**：`NewState` — `ENABLE` 时引脚置高，`DISABLE` 时置低。  
- **说明**：对应硬件引脚 `MotorEnableControl_Pin`。

---

#### 2.1.4 `PWM_DirControl`

```c
void PWM_DirControl(uint8_t dir);
```

- **功能**：电机方向控制。  
- **参数**：`dir` — `MOTOR_DIR_FORWARD`（0）正转，`MOTOR_DIR_REVERSE`（1）反转。  
- **说明**：正转时 `MotorDirectionControl_Pin` 置 0，反转时置 1。

---

#### 2.1.5 `PWM_Set_TargePulse`

```c
unsigned char PWM_Set_TargePulse(unsigned short u16ExpectedValue);
```

- **功能**：设置 PWM 占空比目标值（0~10000 对应 0.00%~100.00%），并置位“待更新”标志；实际占空比在 Timer B 比较 1 中断回调中通过 `User_Func_SetPulse` 写入 HRTIM 比较寄存器。  
- **参数**：`u16ExpectedValue` — 目标占空比数值，范围 **0~10000**。  
- **返回值**：`0` 成功；`1` 参数越界。  
- **说明**：内部会设置 `u16TargePulse` 和 `u8FlagPulse = 1`，在 HRTIM Timer B 的 CMP1 回调里再转换为 9~17000 的脉宽并调用 `User_Func_SetPulse(u16TargePulse*1.7f)` 更新 PWM。

---

### 2.2 仅在 hrtim.c 中实现的可调用函数（未在 .h 中声明）

这些函数在 `hrtim.c` 中实现，可被本工程其他 .c 通过 `extern` 或统一头文件扩展后使用；当前头文件中未声明，视为“模块内部或可选接口”。

#### 2.2.1 `User_Func_SetPulse`

```c
unsigned char User_Func_SetPulse(unsigned short u16T2Pulse);
```

- **功能**：直接设置 HRTIM Timer A 的 PWM 脉宽（比较寄存器），从而设定 TA1 占空比。  
- **参数**：`u16T2Pulse` — 高电平计数值，范围 **9~17000**（与 Timer A 周期 17000 对应）。  
- **返回值**：`0` 成功；`1` 参数越界（会打印 “占空比参数越界”）。  
- **说明**：  
  - 写入 `HRTIM1->sTimerxRegs[0].CMP1xR`（Timer A 的 CMP1，决定 TA1 占空比）。  
  - 同时更新 CMP2、CMP3（用于 ADC 触发等）。  
  - 本模块内由 `HAL_HRTIM_Compare1EventCallback`（Timer B CMP1）中根据 `u16TargePulse` 调用（换算关系：`u16TargePulse*1.7f` 得到 9~17000 的脉宽）。

---

#### 2.2.2 `PWM_TEST`

```c
void PWM_TEST(void);
```

- **功能**：通过调试串口交互的占空比测试：先预读一帧、提示输入 5 位占空比值（00000~10000），接收后写入 `u16TargePulse` 并置 `u8FlagPulse = 1`，由后续 Timer B 回调完成实际 PWM 更新。  
- **说明**：依赖 `DEBUG_UART`、`u8DebugRxBuff` 等，为调试用接口。

---

### 2.3 HAL 回调（在 hrtim.c 中实现）

#### 2.3.1 `HAL_HRTIM_Compare1EventCallback`

```c
void HAL_HRTIM_Compare1EventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx);
```

- **功能**：HRTIM 比较 1 事件回调。本工程中仅处理 **Timer B**：若存在待更新占空比（`u8FlagPulse == 1`），则调用 `User_Func_SetPulse(u16TargePulse*1.7f)` 更新 PWM，并清除 `u8FlagPulse`、打印成功/失败信息。  
- **参数**：`hhrtim` — 句柄；`TimerIdx` — 定时器索引（如 `HRTIM_TIMERINDEX_TIMER_B`）。  
- **说明**：由 HAL 在比较 1 事件时调用，用户一般不直接调用。

---

## 三、接口与数据关系简图

```
其他模块
    │
    ├── 使用 hhrtim1、MOTOR_DIR_xxx、PWM_Enable、PWM_DirControl、PWM_Set_TargePulse、MX_HRTIM1_Init
    │
hrtim.c 内部
    │
    ├── u16TargePulse, u8FlagPulse  ← PWM_Set_TargePulse / PWM_TEST 写入
    │
    └── HAL_HRTIM_Compare1EventCallback(Timer B) 读 u8FlagPulse、u16TargePulse
            └── 调用 User_Func_SetPulse(u16TargePulse*1.7f) 更新 HRTIM CMP1/CMP2/CMP3
```

---

## 四、小结

- **数据类型**：对外可见的主要是 `hhrtim1`、`MOTOR_DIR_FORWARD`/`MOTOR_DIR_REVERSE`；模块内部使用 `u16TargePulse`、`u8FlagPulse` 保存占空比目标与更新标志。  
- **接口函数**：  
  - **头文件声明**：`MX_HRTIM1_Init`、`HAL_HRTIM_MspPostInit`、`PWM_Enable`、`PWM_DirControl`、`PWM_Set_TargePulse`。  
  - **仅 .c 实现**：`User_Func_SetPulse`（直接写脉宽）、`PWM_TEST`（串口测试占空比）。  
  - **HAL 回调**：`HAL_HRTIM_Compare1EventCallback` 在 Timer B 比较 1 时把“目标占空比”通过 `User_Func_SetPulse` 写入 HRTIM，完成 PWM 占空比更新。

以上即为本文件及其对应 `.h` 的数据类型与接口函数的文档化说明。

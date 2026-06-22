/**
 * @file    common.h
 * @brief   公共模块（Common）：跨层类型与纯软件工具
 * @details 模块路径：`User/Common`。零 HAL/外设依赖，仅 `<stdint.h>` / `<stddef.h>`。
 *          供驱动、服务、应用任意层包含：通用枚举与状态、帧队列、小端打包/解包、CRC8 与 CRC16 软件参考实现。
 *          工程内 Simulink 帧 CRC16 已走 BSP 硬件 `Bsp_Crc16Modbus_Calc`；`Cmn_CRC16Modbus_Calc` 保留作对照或离线验证。
 */

#ifndef COMMON_H
#define COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 1. 头文件依赖 ======================== */
#include <stdint.h>
#include <stddef.h>

/* ======================== 2. 宏定义（对外可见） ======================== */
/** 校验是否为合法 FunctionalState_t（项目枚举 PRJ_DISABLE / PRJ_ENABLE；勿与 HAL 的 IS_FUNCTIONAL_STATE 混用） */
#define IS_PRJ_FUNCTIONAL_STATE(S) (((S) == PRJ_DISABLE) || ((S) == PRJ_ENABLE))

/** 帧队列单帧最大字节数 */
#define RFQ_FRAME_MAX_LEN     10
/** 帧队列槽位数（容量为 RFQ_QUEUE_SIZE - 1） */
#define RFQ_QUEUE_SIZE        8

/* ======================== 3. 类型定义 ======================== */
/**
 * @brief 项目通用「使能 / 失能」枚举（语义与 HAL FunctionalState 一致）
 */
typedef enum
{
    PRJ_DISABLE = 0U,
    PRJ_ENABLE  = 1U
} FunctionalState_t;

/**
 * @brief 项目通用操作结果枚举（取值与 HAL_StatusTypeDef 同步，便于驱动层映射）
 */
typedef enum
{
    STATUS_OK      = 0x00U,
    STATUS_ERROR   = 0x01U,
    STATUS_BUSY    = 0x02U,
    STATUS_TIMEOUT = 0x03U
} Status_t;

/**
 * @brief 控制对象选择：倒立摆 / 软尺摆
 */
typedef enum
{
    CONTROL_OBJECT_INVERTED_PENDULUM = 0U, /**< 倒立摆：摆杆量来自编码器 */
    CONTROL_OBJECT_SOFT_RULER_PENDULUM      /**< 软尺摆：摆杆量来自 ADC3 摆动电压 */
} ControlObject_t;

/**
 * @brief 帧级环形队列（SPSC）：单帧缓冲；单生产者单消费者；无 malloc
 * @note  典型用途：串口 DMA + IDLE 中断整帧入队
 */
typedef struct
{
    uint16_t u16Len;
    uint8_t  au8Bytes[RFQ_FRAME_MAX_LEN];
} RfqFrame_t;

typedef struct
{
    RfqFrame_t frames[RFQ_QUEUE_SIZE];
    volatile uint8_t u8WriteIdx;
    volatile uint8_t u8ReadIdx;
} RfqQueue_t;

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */
/* --- 帧级环形队列 --- */
/**
 * @brief 初始化帧级环形队列（读写索引清零）
 * @param struQueue 队列对象；为 NULL 时直接返回
 */
void Cmn_RFQ_Init(RfqQueue_t *struQueue);

/**
 * @brief 入队一帧（单生产者）：拷贝数据并推进写索引
 * @param struQueue 队列对象
 * @param pu8Data   待入队数据首地址
 * @param u16Len    数据长度；超过 RFQ_FRAME_MAX_LEN 时被截断
 * @return STATUS_OK 成功；STATUS_ERROR 参数非法或队列已满
 */
Status_t Cmn_RFQ_Push(RfqQueue_t *struQueue, const uint8_t *pu8Data, uint16_t u16Len);

/**
 * @brief 出队一帧（单消费者）：拷贝到输出并推进读索引
 * @param struQueue 队列对象
 * @param struOut   输出的整帧
 * @return STATUS_OK 成功；STATUS_ERROR 参数非法或队列为空
 */
Status_t Cmn_RFQ_Pop(RfqQueue_t *struQueue, RfqFrame_t *struOut);

/**
 * @brief 获取队列当前帧数量
 * @param struQueue 队列对象
 * @return 已入队未消费的帧数；struQueue 为 NULL 时返回 0
 */
uint8_t Cmn_RFQ_Count(const RfqQueue_t *struQueue);

/**
 * @brief 判断队列是否为空
 * @param struQueue 队列对象
 * @return 1 空（含 struQueue 为 NULL）；0 非空
 */
uint8_t Cmn_RFQ_Empty_Is(const RfqQueue_t *struQueue);

/**
 * @brief 判断队列是否已满
 * @param struQueue 队列对象
 * @return 1 满；0 未满（含 struQueue 为 NULL）
 */
uint8_t Cmn_RFQ_Full_Is(const RfqQueue_t *struQueue);

/* --- 小端序整型打包 / 解包 --- */
/**
 * @brief 将 int16 以小端序写入缓冲区（连续 2 字节）
 * @param pu8Buf   目标缓冲区（至少 2 字节）
 * @param s16Value 待写入值
 */
void Cmn_Int16LE_Pack(uint8_t *pu8Buf, int16_t s16Value);

/**
 * @brief 将 int32 以小端序写入缓冲区（连续 4 字节）
 * @param pu8Buf   目标缓冲区（至少 4 字节）
 * @param s32Value 待写入值
 */
void Cmn_Int32LE_Pack(uint8_t *pu8Buf, int32_t s32Value);

/**
 * @brief 从缓冲区按小端序读取 int16
 * @param pu8Buf 源缓冲区（至少 2 字节）
 * @return 解析得到的 int16 值
 */
int16_t Cmn_Int16LE_Unpack(const uint8_t *pu8Buf);

/**
 * @brief 从缓冲区按小端序读取 int32
 * @param pu8Buf 源缓冲区（至少 4 字节）
 * @return 解析得到的 int32 值
 */
int32_t Cmn_Int32LE_Unpack(const uint8_t *pu8Buf);

/* --- CRC 软件算法 --- */
/**
 * @brief 计算 CRC8（多项式 0x01 即 x^8+1，初值 0x00，MSB 优先，无输入/输出反相）
 * @note  与编码器 CM/SA/AS 帧尾校验一致
 */
uint8_t  Cmn_CRC8_Calc(const uint8_t *pu8Data, uint16_t u16Length);

/**
 * @brief 计算 MODBUS CRC16（多项式 0xA001，初值 0xFFFF）
 * @note  帧尾通常按低字节在前存放；与 BSP 硬件 `Bsp_Crc16Modbus_Calc` 算法等价，可作对照或备用
 */
uint16_t Cmn_CRC16Modbus_Calc(const uint8_t *pu8Data, uint16_t u16Length);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H */

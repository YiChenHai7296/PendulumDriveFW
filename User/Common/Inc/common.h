/**
 * @file    common.h
 * @brief   公共模块（Common）：跨层类型与纯软件工具
 * @details 模块路径：`User/Common`。零 HAL/外设依赖，仅 `<stdint.h>` / `<stddef.h>`。
 *          供驱动、服务、应用任意层包含：通用枚举与状态、帧队列、小端打包/解包、CRC8 与 CRC16 软件参考实现。
 *          工程内 Simulink 帧 CRC16 已走 BSP 硬件 `Bsp_Crc16Modbus_Byte`；`Cmn_CalcCRC16Modbus` 保留作对照或离线验证。
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
/** 校验是否为合法 FunctionalState_t */
#define IS_FUNCTIONAL_STATE(S) (((S) == PRJ_DISABLE) || ((S) == PRJ_ENABLE))

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
} rfq_frame_t;

typedef struct
{
    rfq_frame_t frames[RFQ_QUEUE_SIZE];
    volatile uint8_t u8WriteIdx;
    volatile uint8_t u8ReadIdx;
} rfq_queue_t;

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */
/* --- 帧级环形队列 --- */
void     Cmn_RFQ_Init(rfq_queue_t *struQueue);
/** 入队一帧：成功返回 STATUS_OK；参数非法或队列满返回 STATUS_ERROR */
Status_t Cmn_RFQ_Push(rfq_queue_t *struQueue, const uint8_t *pu8Data, uint16_t u16Len);
/** 出队一帧：成功返回 STATUS_OK；参数非法或队列空返回 STATUS_ERROR */
Status_t Cmn_RFQ_Pop(rfq_queue_t *struQueue, rfq_frame_t *struOut);
uint8_t  Cmn_RFQ_Count(const rfq_queue_t *struQueue);
uint8_t  Cmn_RFQ_Is_Empty(const rfq_queue_t *struQueue);
uint8_t  Cmn_RFQ_Is_Full(const rfq_queue_t *struQueue);

/* --- 小端序整型打包 / 解包 --- */
/** 将 int16 以小端序写入缓冲区（连续 2 字节） */
void    Cmn_PackInt16LE(uint8_t *pu8Buf, int16_t s16Value);
/** 将 int32 以小端序写入缓冲区（连续 4 字节） */
void    Cmn_PackInt32LE(uint8_t *pu8Buf, int32_t s32Value);
/** 从缓冲区按小端序读取 int16 */
int16_t Cmn_UnpackInt16LE(const uint8_t *pu8Buf);
/** 从缓冲区按小端序读取 int32 */
int32_t Cmn_UnpackInt32LE(const uint8_t *pu8Buf);

/* --- CRC 软件算法 --- */
/**
 * @brief 计算 CRC8（多项式 0x01 即 x^8+1，初值 0x00，MSB 优先，无输入/输出反相）
 * @note  与编码器 CM/SA/AS 帧尾校验一致
 */
uint8_t  Cmn_CalcCRC8(const uint8_t *pu8Data, uint16_t u16Length);

/**
 * @brief 计算 MODBUS CRC16（多项式 0xA001，初值 0xFFFF）
 * @note  帧尾通常按低字节在前存放；与 BSP 硬件 `Bsp_Crc16Modbus_Byte` 算法等价，可作对照或备用
 */
uint16_t Cmn_CalcCRC16Modbus(const uint8_t *pu8Data, uint16_t u16Length);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H */

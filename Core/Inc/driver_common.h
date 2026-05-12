/**
 * @file driver_common.h
 * @brief 驱动层公用头：HAL 类型别名、使能枚举，帧队列（SPSC）类型与 API 声明
 * @note 帧队列实现见 driver_common.c
 */

#ifndef DRIVER_COMMON_H
#define DRIVER_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 1. 头文件依赖 ======================== */
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

/* ======================== 2. 宏定义（对外可见） ======================== */
/** 校验是否为合法 Drv_FunctionalState_t（驱动参数检查用） */
#define IS_DRV_FUNCTIONAL_STATE(S) (((S) == DRV_DISABLE) || ((S) == DRV_ENABLE))

/** 帧队列单帧最大字节数 */
#define RFQ_FRAME_MAX_LEN     10
/** 帧队列槽位数（容量为 RFQ_QUEUE_SIZE - 1） */
#define RFQ_QUEUE_SIZE       8



/** @name 驱动层常用结果别名（与 HAL 枚举值一致） */
#define DRV_OK       HAL_OK
#define DRV_ERROR    HAL_ERROR
#define DRV_BUSY     HAL_BUSY
#define DRV_TIMEOUT  HAL_TIMEOUT


/* ======================== 3. 类型定义 ======================== */
/**
  * @brief 驱动层统一返回类型（与 HAL_StatusTypeDef 等价，便于日后替换或映射）
  */
typedef HAL_StatusTypeDef Drv_StatusTypeDef;


/**
 * @brief 驱动层统一「使能 / 失能」枚举
 * @note  取值与 HAL FunctionalState（DISABLE=0、ENABLE=1）一致，可与 HAL_ENABLE 等对照使用
 */
typedef enum
{
  DRV_DISABLE = 0U,
  DRV_ENABLE  = 1U
} Drv_FunctionalState_t;

/**
 * 帧级环形队列（SPSC）：单帧缓冲；单生产者单消费者；无 malloc。
 * 典型用途：串口 DMA + IDLE 中断整帧入队。
 */
typedef struct
{
    uint16_t len;
    uint8_t  data[RFQ_FRAME_MAX_LEN];
} rfq_frame_t;

typedef struct
{
    rfq_frame_t frames[RFQ_QUEUE_SIZE];
    volatile uint8_t write_idx;
    volatile uint8_t read_idx;
} rfq_queue_t;

/* ======================== 4. 对外变量声明 ======================== */
/* 无 */

/* ======================== 5. 接口函数声明 ======================== */
void Drv_RFQ_Init(rfq_queue_t *q);
int Drv_RFQ_Push(rfq_queue_t *q, const uint8_t *data, uint16_t len);
int Drv_RFQ_Pop(rfq_queue_t *q, rfq_frame_t *out);
uint8_t Drv_RFQ_Count(const rfq_queue_t *q);
uint8_t Drv_RFQ_Is_Empty(const rfq_queue_t *q);
uint8_t Drv_RFQ_Is_Full(const rfq_queue_t *q);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_COMMON_H */

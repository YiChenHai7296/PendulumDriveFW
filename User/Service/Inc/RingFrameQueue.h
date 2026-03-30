#ifndef RING_FRAME_QUEUE_H
#define RING_FRAME_QUEUE_H

/*
 * ========================= 模块说明 =========================
 *
 * 本模块实现的是：
 *   「帧级环形队列（Ring Frame Queue）」
 *
 * 适用场景：
 *   - 串口 DMA + IDLE 中断后，将“一整帧数据”缓存起来
 *   - CAN / SPI / 以太网 等“帧式数据”
 *   - 中断中写入，主循环中读取
 *
 * 设计原则：
 *   - 无动态内存（不 malloc / free）
 *   - 固定容量、确定性强
 *   - O(1) 入队 / 出队
 *   - 与硬件、协议完全解耦
 *
 * ============================================================
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 可配置宏 ===================== */

/*
 * 单帧允许的最大长度（字节）
 * 超过该长度的帧会被截断（或可自行改为返回错误）
 */
#ifndef RFQ_FRAME_MAX_LEN
#define RFQ_FRAME_MAX_LEN     10
#endif

/*
 * 队列最多可缓存的帧数
 * SPSC 实现保留 1 个空槽用于区分空/满，
 * 实际可缓存帧数 = RFQ_QUEUE_SIZE - 1
 */
#ifndef RFQ_QUEUE_SIZE
#define RFQ_QUEUE_SIZE       8
#endif

/* ===================== 数据结构定义 ===================== */

/*
 * 单个“数据帧”的结构
 * 注意：这是“完整帧”，不是字节流
 */
typedef struct
{
    uint16_t len;                              /* 本帧有效数据长度 */
    uint8_t  data[RFQ_FRAME_MAX_LEN];          /* 帧数据缓冲区 */
} rfq_frame_t;

/*
 * 帧级环形队列
 *
 * 写入模型（SPSC）：
 *   - 写指针 write_idx 只在“生产者”侧修改（如中断）
 *   - 读指针 read_idx 只在“消费者”侧修改（如主循环）
 *
 * 判空/判满只依赖 read_idx 与 write_idx：
 *   - 空：read_idx == write_idx
 *   - 满：next(write_idx) == read_idx
 *
 * 注意：此实现保留 1 个空槽用于区分空/满，
 * 因此最大可缓存帧数 = RFQ_QUEUE_SIZE - 1。
 */
typedef struct
{
    rfq_frame_t frames[RFQ_QUEUE_SIZE]; /* 帧存储区 */

    volatile uint8_t write_idx;          /* 写指针 */
    volatile uint8_t read_idx;           /* 读指针 */
} rfq_queue_t;

/* ===================== 接口函数 ===================== */

/**
 * @brief  初始化帧环形队列
 * @param  q  队列指针
 */
void RFQ_Init(rfq_queue_t *q);

/**
 * @brief  将一整帧数据压入队列（数据拷贝）
 *
 * @param  q     队列指针
 * @param  data  指向帧数据的指针
 * @param  len   帧数据长度（字节）
 *
 * @retval 0   成功
 * @retval -1  队列已满或参数错误
 *
 * @note
 *   - 本函数可以在中断中调用
 *   - 内部不会使用 malloc
 */
int RFQ_Push(rfq_queue_t *q, const uint8_t *data, uint16_t len);

/**
 * @brief  从队列中取出一帧数据
 *
 * @param  q    队列指针
 * @param  out  输出帧指针
 *
 * @retval 0   成功
 * @retval -1  队列为空或参数错误
 *
 * @note
 *   - 通常在主循环中调用
 */
int RFQ_Pop(rfq_queue_t *q, rfq_frame_t *out);

/* ===================== 辅助内联函数 ===================== */

/**
 * @brief  获取当前队列中已有的帧数量
 */
static inline uint8_t RFQ_Count(const rfq_queue_t *q)
{
    uint8_t w = q->write_idx;
    uint8_t r = q->read_idx;
    if (w >= r)
    {
        return (uint8_t)(w - r);
    }
    return (uint8_t)(RFQ_QUEUE_SIZE - (r - w));
}

/**
 * @brief  判断队列是否为空
 */
static inline uint8_t RFQ_Is_Empty(const rfq_queue_t *q)
{
    return (q->write_idx == q->read_idx);
}

/**
 * @brief  判断队列是否已满
 */
static inline uint8_t RFQ_Is_Full(const rfq_queue_t *q)
{
    uint8_t next_w = (uint8_t)(q->write_idx + 1U);
    if (next_w >= RFQ_QUEUE_SIZE)
    {
        next_w = 0U;
    }
    return (next_w == q->read_idx);
}

#ifdef __cplusplus
}
#endif

#endif /* RING_FRAME_QUEUE_H */


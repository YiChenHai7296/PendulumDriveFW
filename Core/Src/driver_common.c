/**
 * @file driver_common.c
 * @brief 驱动层公用实现：帧级环形队列（Init / Push / Pop / 状态查询）
 */

/* ======================== 0. 头文件引用 ======================== */
#include "driver_common.h"

/* ======================== 1. 私有宏定义 ======================== */
/* 无 */

/* ======================== 2. 私有类型定义 ======================== */
/* 无 */

/* ======================== 3. 私有变量 ======================== */
/* 无 */

/* ======================== 4. 对外变量定义 ======================== */
/* 无 */

/* ======================== 5. 私有函数声明 ======================== */
static void Drv_RFQ_Memcpy(uint8_t *dst, const uint8_t *src, uint16_t len);

/* ======================== 6. 接口函数实现 ======================== */
void Drv_RFQ_Init(rfq_queue_t *q)
{
    if (q == NULL)
    {
        return;
    }
    q->write_idx = 0;
    q->read_idx  = 0;
}

int Drv_RFQ_Push(rfq_queue_t *q, const uint8_t *data, uint16_t len)
{
    uint8_t next_w;

    if (q == NULL || data == NULL)
    {
        return -1;
    }

    next_w = (uint8_t)(q->write_idx + 1U);
    if (next_w >= RFQ_QUEUE_SIZE)
    {
        next_w = 0U;
    }
    if (next_w == q->read_idx)
    {
        return -1;
    }

    if (len > RFQ_FRAME_MAX_LEN)
    {
        len = RFQ_FRAME_MAX_LEN;
    }

    {
        rfq_frame_t *frame = &q->frames[q->write_idx];
        Drv_RFQ_Memcpy(frame->data, data, len);
        frame->len = len;
    }

    q->write_idx++;
    if (q->write_idx >= RFQ_QUEUE_SIZE)
    {
        q->write_idx = 0;
    }

    return 0;
}

int Drv_RFQ_Pop(rfq_queue_t *q, rfq_frame_t *out)
{
    if (q == NULL || out == NULL)
    {
        return -1;
    }
    if (q->read_idx == q->write_idx)
    {
        return -1;
    }

    *out = q->frames[q->read_idx];

    q->read_idx++;
    if (q->read_idx >= RFQ_QUEUE_SIZE)
    {
        q->read_idx = 0;
    }

    return 0;
}

uint8_t Drv_RFQ_Count(const rfq_queue_t *q)
{
    uint8_t w;
    uint8_t r;

    if (q == NULL)
    {
        return 0U;
    }
    w = q->write_idx;
    r = q->read_idx;
    if (w >= r)
    {
        return (uint8_t)(w - r);
    }
    return (uint8_t)(RFQ_QUEUE_SIZE - (r - w));
}

uint8_t Drv_RFQ_Is_Empty(const rfq_queue_t *q)
{
    if (q == NULL)
    {
        return 1U;
    }
    return (q->write_idx == q->read_idx) ? 1U : 0U;
}

uint8_t Drv_RFQ_Is_Full(const rfq_queue_t *q)
{
    uint8_t next_w;

    if (q == NULL)
    {
        return 0U;
    }
    next_w = (uint8_t)(q->write_idx + 1U);
    if (next_w >= RFQ_QUEUE_SIZE)
    {
        next_w = 0U;
    }
    return (next_w == q->read_idx) ? 1U : 0U;
}

/* ======================== 7. 私有函数实现 ======================== */
static void Drv_RFQ_Memcpy(uint8_t *dst, const uint8_t *src, uint16_t len)
{
    while (len--)
    {
        *dst++ = *src++;
    }
}

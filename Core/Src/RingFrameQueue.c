#include "RingFrameQueue.h"

/*
 * ========================= 内部工具函数 =========================
 *
 * 使用自定义 memcpy：
 *   - 避免依赖 libc
 *   - 在裸机环境下更可控
 *
 * ===============================================================
 */
static void RFQ_Memcpy(uint8_t *dst, const uint8_t *src, uint16_t len)
{
    while (len--)
    {
        *dst++ = *src++;
    }
}

/* ========================= 接口实现 ========================= */

void RFQ_Init(rfq_queue_t *q)
{
    if (q == NULL)
    {
        return;
    }
    /*
     * 指针与计数全部清零即可
    0.2 * 不需要清空 frames 内容
     */
    q->write_idx = 0;
    q->read_idx  = 0;
    q->count     = 0;
}

int RFQ_Push(rfq_queue_t *q, const uint8_t *data, uint16_t len)
{
    if (q == NULL || data == NULL)
    {
        return -1;
    }
    
    /* 如果队列已满，直接拒绝写入（你也可以在这里实现“覆盖最旧帧”的策略） */
    if (q->count >= RFQ_QUEUE_SIZE)
    {
        return -1;
    }

    /* 防止单帧数据超过缓冲区大小 */
    if (len > RFQ_FRAME_MAX_LEN)
    {
        len = RFQ_FRAME_MAX_LEN;
    }

    /* 获取当前写入位置对应的帧结构 */
    rfq_frame_t *frame = &q->frames[q->write_idx];

    /* 拷贝帧数据 */
    RFQ_Memcpy(frame->data, data, len);
    frame->len = len;

    /* 写指针前移并回绕         */
    q->write_idx++;
    if (q->write_idx >= RFQ_QUEUE_SIZE)
    {
        q->write_idx = 0;
    }

    /* 更新帧数量      */
    q->count++;

    return 0;
}

int RFQ_Pop(rfq_queue_t *q, rfq_frame_t *out)
{
    if (q == NULL || out == NULL)
        return -1;

    /* 队列为空，无法读取 */
    if (q->count == 0)
    {
        return -1;
    }

    /* 直接结构体赋值（浅拷贝）data 数组会被完整复制 */
    *out = q->frames[q->read_idx];

    /* 读指针前移并回绕 */
    q->read_idx++;
    if (q->read_idx >= RFQ_QUEUE_SIZE)
    {
        q->read_idx = 0;
    }

    /* 更新帧数量 */
    q->count--;

    return 0;
}

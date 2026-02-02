/**
 * @file encoder_protocol.h
 * @brief 电机编码器帧解析协议栈接口
 */

#ifndef ENCODER_PROTOCOL_H
#define ENCODER_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>



#define NULL ((void *)0)


/**
 * @brief 编码器帧解析结果
 */
typedef enum
{
    ENCODER_PROTOCOL_OK = 0,        /**< 解析成功 */
    ENCODER_PROTOCOL_ERR_NULL,      /**< 指针为空 */
    ENCODER_PROTOCOL_ERR_LENGTH,    /**< 帧长度错误 */
    ENCODER_PROTOCOL_ERR_CRC        /**< CRC 校验失败 */
} EncoderProtocolResult_t;

/**
 * @brief 编码器状态与绝对位置数据
 */
typedef struct
{
    bool     count_error;      /**< SA.bit4：计数错误标志 */
    bool     mt_or_batt_error; /**< SA.bit5：多圈/电池相关错误(逻辑或结果) */
    uint32_t absolute_position;/**< 21 位绝对位置数据，范围 0~0x1FFFFF */
} EncoderProtocolData_t;

/**
 * @brief 从编码器底层数据源读取并解析一帧数据
 *
 * 帧格式（底层实现负责获取原始 6 字节帧，本接口不暴露数据来源）：
 * CM(1B, 固定 0x02) + SA(1B) + AS0~AS2(3B, 21bit 绝对值) + CRC8(1B)
 *
 * @param[out] pOut   解析后的状态与绝对位置数据
 *
 * @return EncoderProtocolResult_t 解析结果
 */
EncoderProtocolResult_t EncoderProtocol_Read(EncoderProtocolData_t *pOut);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_PROTOCOL_H */


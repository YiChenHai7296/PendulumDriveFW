/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    simulink_protocol.c
  * @brief   simulink上位机通信协议栈实现文件
  *          用于组帧和解帧，支持控制帧和反馈帧
  ******************************************************************************
  */
/* USER CODE END Header */

#include "simulink_protocol.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* ===================== 私有函数声明 ===================== */
static void PackInt16LittleEndian(uint8_t *pBuffer, int16_t value);
static void PackInt32LittleEndian(uint8_t *pBuffer, int32_t value);
static int16_t UnpackInt16LittleEndian(const uint8_t *pBuffer);
static int32_t UnpackInt32LittleEndian(const uint8_t *pBuffer);

/* ===================== 函数实现 ===================== */

/**
  * @brief  打包16位整数为小端模式（低位在前）
  * @param  pBuffer 输出缓冲区指针
  * @param  value 要打包的值
  */
static void PackInt16LittleEndian(uint8_t *pBuffer, int16_t value)
{
    pBuffer[0] = (uint8_t)(value & 0xFF);
    pBuffer[1] = (uint8_t)((value >> 8) & 0xFF);
}

/**
  * @brief  打包32位整数为小端模式（低位在前）
  * @param  pBuffer 输出缓冲区指针
  * @param  value 要打包的值
  */
static void PackInt32LittleEndian(uint8_t *pBuffer, int32_t value)
{
    pBuffer[0] = (uint8_t)(value & 0xFF);
    pBuffer[1] = (uint8_t)((value >> 8) & 0xFF);
    pBuffer[2] = (uint8_t)((value >> 16) & 0xFF);
    pBuffer[3] = (uint8_t)((value >> 24) & 0xFF);
}

/**
  * @brief  从小端模式解包16位整数（低位在前）
  * @param  pBuffer 输入缓冲区指针
  * @retval 解包后的16位整数
  */
static int16_t UnpackInt16LittleEndian(const uint8_t *pBuffer)
{
    int16_t value = 0;
    value = (int16_t)pBuffer[0];
    value |= ((int16_t)pBuffer[1] << 8);
    return value;
}

/**
  * @brief  从小端模式解包32位整数（低位在前）
  * @param  pBuffer 输入缓冲区指针
  * @retval 解包后的32位整数
  */
static int32_t UnpackInt32LittleEndian(const uint8_t *pBuffer)
{
    int32_t value = 0;
    value = (int32_t)pBuffer[0];
    value |= ((int32_t)pBuffer[1] << 8);
    value |= ((int32_t)pBuffer[2] << 16);
    value |= ((int32_t)pBuffer[3] << 24);
    return value;
}

/**
  * @brief  打包反馈帧
  * @param  pFeedbackFrame 反馈帧参数结构体指针
  * @param  pBuffer 输出缓冲区指针，至少需要PROTOCOL_FRAME_SIZE_FEEDBACK字节
  * @retval 实际打包的字节数，失败返回0
  */
uint16_t Protocol_PackFeedbackFrame(const Protocol_FeedbackFrame_t *pFeedbackFrame, uint8_t *pBuffer)
{
    uint16_t offset = 0;
    uint16_t crc16 = 0;
    
    if (pFeedbackFrame == NULL || pBuffer == NULL)
    {
        return 0;
    }
    
    // 参数范围检查
    if (pFeedbackFrame->motor_current < PROTOCOL_MOTOR_CURRENT_MIN || 
        pFeedbackFrame->motor_current > PROTOCOL_MOTOR_CURRENT_MAX)
    {
        return 0;
    }
    if (pFeedbackFrame->motor_position < PROTOCOL_MOTOR_POSITION_MIN || 
        pFeedbackFrame->motor_position > PROTOCOL_MOTOR_POSITION_MAX)
    {
        return 0;
    }
    if (pFeedbackFrame->motor_speed < PROTOCOL_MOTOR_SPEED_MIN || 
        pFeedbackFrame->motor_speed > PROTOCOL_MOTOR_SPEED_MAX)
    {
        return 0;
    }
    if (pFeedbackFrame->axis_position < PROTOCOL_AXIS_POSITION_MIN || 
        pFeedbackFrame->axis_position > PROTOCOL_AXIS_POSITION_MAX)
    {
        return 0;
    }
    if (pFeedbackFrame->axis_speed < PROTOCOL_AXIS_SPEED_MIN || 
        pFeedbackFrame->axis_speed > PROTOCOL_AXIS_SPEED_MAX)
    {
        return 0;
    }
    if (pFeedbackFrame->pendulum_position < PROTOCOL_PENDULUM_POSITION_MIN || 
        pFeedbackFrame->pendulum_position > PROTOCOL_PENDULUM_POSITION_MAX)
    {
        return 0;
    }
    if (pFeedbackFrame->pendulum_speed < PROTOCOL_PENDULUM_SPEED_MIN || 
        pFeedbackFrame->pendulum_speed > PROTOCOL_PENDULUM_SPEED_MAX)
    {
        return 0;
    }
    
    // 1. 帧头：0x5A 0xA5
    pBuffer[offset++] = PROTOCOL_HEAD_BYTE0;
    pBuffer[offset++] = PROTOCOL_HEAD_BYTE1;
    
    // 2. 类型：0x02（反馈帧）
    pBuffer[offset++] = PROTOCOL_TYPE_FEEDBACK;
    
    // 3. 长度：0x1A（26字节）
    pBuffer[offset++] = PROTOCOL_LEN_FEEDBACK;
    
    // 4. 负载域（26字节）
    // 0~1字节：电机电流值
    PackInt16LittleEndian(&pBuffer[offset], pFeedbackFrame->motor_current);
    offset += 2;
    
    // 2~5字节：电机位置（21位编码器）
    PackInt32LittleEndian(&pBuffer[offset], pFeedbackFrame->motor_position);
    offset += 4;
    
    // 6~9字节：电机转速
    PackInt32LittleEndian(&pBuffer[offset], pFeedbackFrame->motor_speed);
    offset += 4;
    
    // 10~13字节：轴位置（17位编码器）
    PackInt32LittleEndian(&pBuffer[offset], pFeedbackFrame->axis_position);
    offset += 4;
    
    // 14~17字节：轴转速
    PackInt32LittleEndian(&pBuffer[offset], pFeedbackFrame->axis_speed);
    offset += 4;
    
    // 18~21字节：摆杆位置（17位编码器）
    PackInt32LittleEndian(&pBuffer[offset], pFeedbackFrame->pendulum_position);
    offset += 4;
    
    // 22~25字节：摆杆转速
    PackInt32LittleEndian(&pBuffer[offset], pFeedbackFrame->pendulum_speed);
    offset += 4;
    
    // 5. CRC16校验（计算帧头+类型+长度+负载域）
    // 注意：CRC16计算范围是从帧头到负载域结束，不包括CRC16本身
    crc16 = Protocol_CalculateCRC16(pBuffer, offset);
    pBuffer[offset++] = (uint8_t)(crc16 & 0xFF);        // CRC16低字节
    pBuffer[offset++] = (uint8_t)((crc16 >> 8) & 0xFF); // CRC16高字节
    
    return offset;
}

/**
  * @brief  解包控制帧
  * @param  pBuffer 接收缓冲区指针，包含完整的控制帧数据
  * @param  bufferSize 缓冲区大小
  * @param  pControlFrame 输出控制帧参数结构体指针
  * @retval true: 解包成功  false: 解包失败
  */
bool Protocol_UnpackControlFrame(const uint8_t *pBuffer, uint16_t bufferSize, Protocol_ControlFrame_t *pControlFrame)
{
    uint16_t offset = 0;
    uint16_t crc16_calculated = 0;
    uint16_t crc16_received = 0;
    
    if (pBuffer == NULL || pControlFrame == NULL)
    {
        return false;
    }
    
    // 检查缓冲区大小是否足够
    if (bufferSize < PROTOCOL_FRAME_SIZE_CONTROL)
    {
        return false;
    }
    
    // 1. 验证帧头：0x5A 0xA5
    if (pBuffer[offset++] != PROTOCOL_HEAD_BYTE0 || 
        pBuffer[offset++] != PROTOCOL_HEAD_BYTE1)
    {
        return false;
    }
    
    // 2. 验证类型：0x01（控制帧）
    if (pBuffer[offset++] != PROTOCOL_TYPE_CONTROL)
    {
        return false;
    }
    
    // 3. 验证长度：0x04（4字节）
    if (pBuffer[offset++] != PROTOCOL_LEN_CONTROL)
    {
        return false;
    }
    
    // 4. 解析负载域（4字节）
    // 0~1字节：PWM值
    pControlFrame->pwm = UnpackInt16LittleEndian(&pBuffer[offset]);
    offset += 2;
    
    // 2~3字节：电流值
    pControlFrame->current = UnpackInt16LittleEndian(&pBuffer[offset]);
    offset += 2;
    
    // 5. 验证CRC16
    // 接收到的CRC16（小端模式）
    crc16_received = (uint16_t)pBuffer[offset];
    crc16_received |= ((uint16_t)pBuffer[offset + 1] << 8);
    
    // 计算CRC16（从帧头到负载域结束）
    crc16_calculated = Protocol_CalculateCRC16(pBuffer, offset);
    
    // CRC16校验失败
    if (crc16_calculated != crc16_received)
    {
        return false;
    }
    
    // 6. 参数范围检查
    if (pControlFrame->pwm < PROTOCOL_PWM_MIN || 
        pControlFrame->pwm > PROTOCOL_PWM_MAX)
    {
        return false;
    }
    if (pControlFrame->current < PROTOCOL_CURRENT_MIN || 
        pControlFrame->current > PROTOCOL_CURRENT_MAX)
    {
        return false;
    }
    
    return true;
}

/**
  * @brief  CRC16校验计算（预留接口，由用户实现）
  * @param  pData 数据指针
  * @param  length 数据长度
  * @retval CRC16校验值
  * 
  * @note   用户需要在此函数中实现CRC16计算算法
  *         当前为占位实现，返回0
  */
uint16_t Protocol_CalculateCRC16(const uint8_t *pData, uint16_t length)
{
    // TODO: 用户在此实现CRC16计算算法
    // 示例：可以使用CRC16-CCITT、CRC16-MODBUS等算法
    
    (void)pData;    // 避免未使用参数警告
    (void)length;   // 避免未使用参数警告
    
    return 0;       // 占位返回值，用户需要替换为实际的CRC16计算结果
}

/**
  * @brief  验证帧头
  * @param  pBuffer 缓冲区指针
  * @retval true: 帧头正确  false: 帧头错误
  */
bool Protocol_VerifyHeader(const uint8_t *pBuffer)
{
    if (pBuffer == NULL)
    {
        return false;
    }
    
    return (pBuffer[0] == PROTOCOL_HEAD_BYTE0 && 
            pBuffer[1] == PROTOCOL_HEAD_BYTE1);
}

/**
  * @brief  查找帧头位置
  * @param  pBuffer 缓冲区指针
  * @param  bufferSize 缓冲区大小
  * @retval 帧头位置索引，未找到返回-1
  */
int16_t Protocol_FindHeader(const uint8_t *pBuffer, uint16_t bufferSize)
{
    uint16_t i;
    
    if (pBuffer == NULL || bufferSize < PROTOCOL_HEAD_SIZE)
    {
        return -1;
    }
    
    // 查找帧头0x5A 0xA5
    for (i = 0; i <= bufferSize - PROTOCOL_HEAD_SIZE; i++)
    {
        if (pBuffer[i] == PROTOCOL_HEAD_BYTE0 && 
            pBuffer[i + 1] == PROTOCOL_HEAD_BYTE1)
        {
            return (int16_t)i;
        }
    }
    
    return -1;
}

/* USER CODE BEGIN Implementation */

/* USER CODE END Implementation */

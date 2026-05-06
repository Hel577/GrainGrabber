#include "filter.h"

/**
 * @brief 计算Filter舵机校验码
 * @param data 数据数组
 * @param len 数据长度
 * @return 校验码
 */
uint8_t Filter_Calculate_Checksum(uint8_t *data, uint8_t len)
{
    uint8_t total_sum = 0;
    
    for (uint8_t i = 0; i < len; i++)
    {
        total_sum += data[i];
    }
    
    total_sum = total_sum & 0xFF;
    uint8_t checksum = (~total_sum) & 0xFF;
    
    return checksum;
}

/**
 * @brief 发送数据到Filter舵机
 * @param data 数据数组
 * @param len 数据长度
 */
void Filter_Send_Command(uint8_t *data, uint8_t len)
{
    HAL_UART_Transmit(&huart8, data, len, 10); 
    // HAL_Delay(2); // 
}

/**
 * @brief 舵机中位校准功能（将当前位置自动校正为2048）
 * @param id 舵机ID (1为旋转舵机, 2为夹爪舵机)
 */
void Filter_Calibrate_Center_Position(uint8_t id)
{
    uint8_t command[8]; // 修正数组大小为8，因为总共有8个字节
    uint8_t data[5];
    
    // 构建指令
    command[0] = FILTER_HEADER1;
    command[1] = FILTER_HEADER2;
    
    data[0] = id;                // ID号
    data[1] = 0x04;              // 数据长度
    data[2] = FILTER_WRITE_CMD;  // 写指令
    data[3] = FILTER_CALIB_ADDR; // 写首地址 (40)
    data[4] = 0x80;              // 写入数据 (128，开启中位自动校准)
    
    // 计算校验码
    uint8_t checksum = Filter_Calculate_Checksum(data, 5);
    
    // 组装完整指令
    command[2] = data[0];
    command[3] = data[1];
    command[4] = data[2];
    command[5] = data[3];
    command[6] = data[4];
    command[7] = checksum;
    
    // 发送指令
    Filter_Send_Command(command, 8);
}

/**
 * @brief 控制Filter舵机
 * @param id 舵机ID (2为旋转舵机, 1为夹爪舵机)
 * @param target_position 目标位置 (输入范围-30719~30719)
 * @param target_speed 目标速度 (0-3400)
 * @param acceleration 加速度 (0-255)
 */
void Filter_Servo_PosCtrl(uint8_t id, int16_t target_position, uint16_t target_speed, uint8_t acceleration)
{
    uint8_t command[14]; // 修正数组大小为14，因为总共有14个字节
    uint8_t data[11];    // 修正数组大小为11，因为实际使用了11个元素
    
    // 限制位置范围
    if (id == SPIN_SERVO)
    {
        if (target_position < SPIN_MIN_POS)
            target_position = SPIN_MIN_POS;
        else if (target_position > SPIN_MAX_POS)
            target_position = SPIN_MAX_POS;
    }
    else if (id == GRAB_SERVO)
    {
        if (target_position < GRAB_MIN_POS)
            target_position = GRAB_MIN_POS;
        else if (target_position > GRAB_MAX_POS)
            target_position = GRAB_MAX_POS;
    }
    
    // 限制速度范围
    if (target_speed > 3400)
        target_speed = 3400;

    // 拆分位置和速度为高低字节
    uint8_t position_low = target_position & 0xFF;
    uint8_t position_high = (target_position >> 8) & 0xFF;
    uint8_t speed_low = target_speed & 0xFF;
    uint8_t speed_high = (target_speed >> 8) & 0xFF;

    // 构建指令
    command[0] = FILTER_HEADER1;
    command[1] = FILTER_HEADER2;

    data[0] = id;                 // ID号
    data[1] = 0x0A;               // 数据长度
    data[2] = FILTER_WRITE_CMD;   // 写指令
    data[3] = FILTER_CTRL_ADDR;   // 写首地址 (41)
    data[4] = acceleration;       // 加速度
    data[5] = position_low;       // 位置低字节
    data[6] = position_high;      // 位置高字节
    data[7] = 0x00;               // 时间低字节
    data[8] = 0x00;               // 时间高字节
    data[9] = speed_low;          // 速度低字节
    data[10] = speed_high;        // 速度高字节

    // 计算校验码
    uint8_t checksum = Filter_Calculate_Checksum(data, 11);

    // 组装完整指令
    command[2] = data[0];
    command[3] = data[1];
    command[4] = data[2];
    command[5] = data[3];
    command[6] = data[4];
    command[7] = data[5];
    command[8] = data[6];
    command[9] = data[7];
    command[10] = data[8];
    command[11] = data[9];
    command[12] = data[10];
    command[13] = checksum;
    
    // 发送指令
    Filter_Send_Command(command, 14);
}


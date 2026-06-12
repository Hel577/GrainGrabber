#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"

//末端角度转化率
#define END_ANGLE_RATE 0.087890625

/* Filter舵机ID定义 */
#define SPIN_SERVO    2   // 旋转舵机ID
#define GRAB_SERVO     1   // 夹爪舵机ID

/* Filter舵机位置范围 */
#define SPIN_MIN_POS    0  //对应末端0.1528421053度
#define SPIN_MAX_POS    4096
#define GRAB_MIN_POS    0  //爪子完全张开
#define GRAB_MAX_POS    4096  //爪子完全闭合

/* Filter舵机指令相关参数 */
#define FILTER_HEADER1    0xFF
#define FILTER_HEADER2    0xFF
#define FILTER_WRITE_CMD  0x03
#define FILTER_CALIB_ADDR 0x28
#define FILTER_CTRL_ADDR  0x29
#define FILTER_READ_ADDR 0x38
#define FILTER_READ_CMD 0x02

/**
 * @brief 计算Filter舵机校验码
 * @param data 数据数组
 * @param len 数据长度
 * @return 校验码
 */
uint8_t Filter_Calculate_Checksum(uint8_t *data, uint8_t len);

/**
 * @brief 舵机中位校准功能（将当前位置自动校正为2048）夹爪以完全闭合为零位,旋转以机械臂伸直爪子竖着为零位
 * @param id 舵机ID (1为旋转舵机, 2为夹爪舵机)
 */
void Filter_Calibrate_Center_Position(uint8_t id);

/**
 * @brief 控制Filter舵机
 * @param id 舵机ID (1为旋转舵机, 2为夹爪舵机)
 * @param target_position 目标位置 (一圈为0-4095,顺时针为正)
 * @param target_speed 目标速度 (0-3400)
 * @param acceleration 加速度 (0-255)
 */
void Filter_Servo_PosCtrl(uint8_t id, int16_t target_position, uint16_t target_speed, uint8_t acceleration);

/**
 * @brief 发送数据到Filter舵机
 * @param data 数据数组
 * @param len 数据长度
 */
void Filter_Send_Command(uint8_t *data, uint8_t len);



void Filter_Read_Pos(uint8_t id);
bool Filter_Verify_Checksum(uint8_t *data, uint8_t len);


#endif /* __FILTER_H */
#ifndef BSP_H
#define BSP_H

#include "MI_motor_dev.h"
// #include "filter.h"
// #include "pid.h"
#include "main.h"
// #include "tim.h"
#define WHEEL_RADIUS 76.0f // 轮子半径 单位：mm
#define CAR_H 400.0f // 前后轮心距
#define CAR_W 400.0f // 左右轮心距
void bsp_init(void);
void SendMultiFloat2Vofa(float *values, uint8_t num);
void Init_Chassis_Motor(void);
void Change_Chassis_Motor_Mode(uint8_t mode);
void Set_Chassis_Motor_SpdLim(float spd_lim);
void Set_Chassis_Motor_CurrLim(float current_lim);
void Reset_Chassis_Motor_MechPosition(void);
void En_Chassis_Motor(void);
void Dis_Chassis_Motor(void);
void Publish_Chassis_Motor_Speed(float Vel1, float Vel2, float Vel3, float Vel4,uint32_t delay_ms);
void Disable_IMU_Interrupts(void);
void Enable_IMU_Interrupts(void);
void Init_Scara_Motor(void);
void Change_Scara_Motor_Mode(uint8_t mode);
void Set_Scara_Motor_SpdLim(uint8_t spd_lim);
void Set_Scara_Motor_CurrLim(uint8_t current_lim);
void Reset_Scara_Motor_MechPosition(void);
void En_Scara_Motor(void);
void Dis_Scara_Motor(void);
void Init_Lifting_Motor(void);
void Change_Lifting_Motor_Mode(uint8_t mode);
void Set_Lifting_Motor_SpdLim(uint8_t spd_lim);
void Set_Lifting_Motor_CurrLim(uint8_t current_lim);
void Reset_Lifting_Motor_MechPosition(void);
void En_Lifting_Motor(void);
void Dis_Lifting_Motor(void);
#endif


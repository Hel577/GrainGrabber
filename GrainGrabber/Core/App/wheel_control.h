#ifndef __WHEEL_CONTROL_H
#define __WHEEL_CONTROL_H


#include "usart.h"  // 引入串口头文件
#include <math.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "toolfunc.h"

//轮子ID	F:front R:rear
#define LF_MOTOR_ID 1
#define LR_MOTOR_ID 2
#define RF_MOTOR_ID 3
#define RR_MOTOR_ID 4


extern uint8_t bytedata[28];
uint8_t if_minus[4];
float car_adjust[3];


void position_cal(float* motor_fai_, float* car_distance_ , uint8_t* if_minus);
void speed_cal(float* motor_speed_, float* car_speed_ , uint8_t* if_minus);
void move_to_target(float x_target, float y_target, float angle_target, int v);
void set_speed(float angle_target, float speed_magnitude, float w_z);
void stop(void);
// float linear_speed_PID_Update(PID* pid, float target, float current, float dt);
// float angular_speed_PID_Update(PID* pid, float target, float current, float dt);
// void encodingdisk_control(float target_x, float target_y ,float target_angle);


#endif

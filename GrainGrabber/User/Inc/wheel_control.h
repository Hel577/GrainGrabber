#ifndef __WHEEL_CONTROL_H
#define __WHEEL_CONTROL_H

#include "wheel_control.h"
#include "usart.h"  // 引入串口头文件
#include <math.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"

//轮子ID	F:front R:rear
#define LF_MOTOR_ID 1
#define LR_MOTOR_ID 2
#define RF_MOTOR_ID 3
#define RR_MOTOR_ID 4


extern float OPSdata[6];
extern uint8_t bytedata[28];
extern float truedata[6];
extern uint8_t camdata[8];
extern uint8_t OPSreset[4];
extern uint8_t isOPSRead;
uint8_t if_minus[4];
float car_adjust[3];



//extern bool doTwistAngle, SetPosDone, dobrake;


extern uint8_t subdata[4];




typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
} PID;

extern PID pos_pid, angle_pid;

// 定义结构体来存储 OPS 数据
typedef struct {
    float heading;  // 三轴角度值 - X 轴
    float angle_y;  // 三轴角度值 - Y 轴
    float angle_z;  // 三轴角度值 - Z 轴
    float x;  // 坐标值 - X 坐标
    float y;  // 坐标值 - Y 坐标
    float angular_vel; // 角速度值
} OPSDataStruct;
extern OPSDataStruct ops_data;


void cal_motor(float* motor_fai_, float* car_distance_ , uint8_t* if_minus);
void speed_cal(float* motor_speed_, float* car_speed_ , uint8_t* if_minus);
void move_to_target(float x_target, float y_target, float angle_target, int v);
void set_speed_pid(float angle_target, float speed_magnitude, float w_z);
void stop(void);
float linear_speed_PID_Update(PID* pid, float target, float current, float dt);
float angular_speed_PID_Update(PID* pid, float target, float current, float dt);
void encodingdisk_control(float target_x, float target_y ,float target_angle);


#endif

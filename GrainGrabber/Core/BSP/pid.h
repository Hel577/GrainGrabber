#ifndef PID_H
#define PID_H
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"
#include "stdio.h"
#include "stdbool.h"
#include <stdlib.h>
#include "cmsis_os.h"
#include "main.h"
//**********************************结构体声明*******************************************//
// PID控制器
typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float dt;               // 时间间隔
    float integral;         // 积分项
    float prev_error;       // 上一次的误差
    float max_output;       // 最大输出限制
    float max_accel;        // 最大加速度限制


    uint32_t last_call_time  ;
    float deriv_buf[4] ;
    float last_output ;
    float prev_filtered ;
    float prev_position ;
    bool first_call ;

  } PIDController;

extern PIDController *rough_X_PID;
extern PIDController *rough_Y_PID;
extern PIDController *rough_Z_PID;
extern PIDController *fine_X_PID;
extern PIDController *fine_Y_PID;
extern PIDController *fine_Z_PID;
extern PIDController *Easy_X_PID;
extern PIDController *Easy_Y_PID;
extern PIDController *Easy_Z_PID;
extern PIDController *Translation_X_PID;
extern PIDController *Translation_Y_PID;
extern PIDController *Translation_Z_PID;

void Init_PID(void);
void Reset_PID(PIDController *pid);
float PID_Calc_XY(PIDController *pid, float target, float current);
float PID_Calc_Z(PIDController *pid, float target, float current);

#endif


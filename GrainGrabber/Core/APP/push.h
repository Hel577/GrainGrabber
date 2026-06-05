#ifndef push_H
#define push_H

#include "bsp.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

#define PUSH_MOTOR_ID 7 // 升降电机ID
#define ARM_SHORT_LENGTH 91.0f // 机械臂短臂长度(mm)
#define ARM_LONG_LENGTH 236.0f // 机械臂长臂长度(mm)
#define MAX_POSITION 236+91.0f
#define MIN_POSITION 236-91.0f


// 升降机构状态
typedef struct {
    float current_position;   // 当前位置，单位mm
    float target_position;    // 目标位置，单位mm
    float current_angle;      // 当前电机角度，单位rad
    float target_angle;       // 目标电机角度，单位rad
    float step_angle;         // 步进角度，单位rad
    bool is_moving;         // 是否正在移动
    bool is_initialized;    // 是否已初始化

} push_Status;

// 全局变量声明
extern push_Status *push;

// 函数声明
void Init_push(void);                        // 初始化升降机构
void push_Move_To_Position(float position);      // 移动到指定高度
void push_Update(void);                  // 更新升降机构状态
void push_PosCtrl(float position);             // 升降机构位置控制
void Calculate_Angles_From_Position(float position, float* angle); // 根据高度计算电机角度
void push_Reset(void);                       // 复位升降机构状态
void add_push_ctrl(void);                    // 升降机构增量控制（定时器中使用）

#endif 
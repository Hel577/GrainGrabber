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
    float max_position;       // 最大位置限制，单位mm
    float min_position;       // 最小位置限制，单位mm
    float error_max;        // 误差最大值，单位mm
    bool is_moving;         // 是否正在移动
    bool is_initialized;    // 是否已初始化

} push_Status;

// 全局变量声明
extern push_Status *push;

// 函数声明
void Init_push(void);                        // 初始化升降机构
void push_Move_To_Position(float position);      // 移动到指定高度
bool push_Is_Reached(void);                  // 检查是否到达目标高度
void push_PosCtrl(float position);             // 升降机构位置控制
void Calculate_Angles_From_Position(float position, float* angle); // 根据高度计算电机角度
void push_Reset(void);                       // 复位升降机构状态

#endif 
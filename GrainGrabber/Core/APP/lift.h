#ifndef LIFT_H
#define LIFT_H

#include "bsp.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

// 升降机构状态
typedef struct {
    float current_height;   // 当前高度，单位mm
    float target_height;    // 目标高度，单位mm
    float max_height;       // 最大高度限制，单位mm
    float min_height;       // 最小高度限制，单位mm
    float error_max;        // 误差最大值，单位mm
    bool is_moving;         // 是否正在移动
    bool is_initialized;    // 是否已初始化

} Lift_Status;

// 全局变量声明
extern Lift_Status *lift;

// 函数声明
void Init_Lift(void);                        // 初始化升降机构
void Lift_Move_To_Height(float height);      // 移动到指定高度
bool Lift_Is_Reached(void);                  // 检查是否到达目标高度
void Lift_PosCtrl(float height);             // 升降机构位置控制

#endif 
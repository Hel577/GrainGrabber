#include "push.h"
#include "math.h"


// 全局变量定义
static push_Status push_status;
push_Status *push = &push_status;




//height高度使用电机转动弧度，单位rad
/**
 * @brief 初始化升降机构
 */
void Init_push(void)
{
    
    // 初始化升降机构状态
    push->current_position = 0.0f;
    push->target_position = 0.0f;
    push->max_position = 15.0f;    // 设置最大高度限制，根据实际情况调整
    push->min_position = 0.0f;      // 设置最小高度限制
    push->error_max = 0.2f;
    push->is_moving = false;
    push->is_initialized = true;

}

/**
 * @brief 复位升降机构状态
 */
void push_Reset(void)
{
    push->target_position = 0.0f;
    push->is_moving = true;
    
    // 移动到零位置
    MI_motor_PosCtrl(motors[7], 0.0f);
}

void Calculate_Angles_From_Position(float position, float* angle)
{
    float para = ARM_LONG_LENGTH*ARM_LONG_LENGTH-ARM_SHORT_LENGTH*ARM_SHORT_LENGTH-position*position;
    float radias = acosf(para/(2*position*ARM_SHORT_LENGTH));
    *angle = radias;
}



/**
 * @brief 移动到指定高度
 * @param height 目标高度,使用电机转动弧度代替，单位rad
 */
void push_Move_To_Position(float position)
{
    push->is_moving = true;
    // 限制高度在有效范围内
    float target_position = fmaxf(push->min_position, fminf(position, push->max_position));
    push->target_position = target_position;
    // 使用位置模式控制电机
    float target_angle;
    Calculate_Angles_From_Position(target_position, &target_angle);
    MI_motor_PosCtrl(motors[7], target_angle);
}



/**
 * @brief 检查是否到达目标高度
 * @return 是否到达目标高度
 */
bool push_Is_Reached(void)
{
    MI_motor_get_mechPos(motors[7]);
    push->current_position = -1.0f*motors[7]->motor_fdb.mechPos;
    float error = fabsf(push->target_position - push->current_position);
    // printf("error: %f\r\n", error);
    if (error < push->error_max)
    {
        return true;
    }
    else
    {
        return false;
    }
}

   /**
    * @brief 升降机构位置控制，临时暂停机械臂定时器
    * @param height 目标高度
    */
   void push_PosCtrl(float position)
   {
      HAL_TIM_Base_Stop_IT(&htim5);
       
       // 发送升降指令
       push_Move_To_Position(position);
       
       HAL_Delay(5);
       // 等待指令被接收
       push_Move_To_Position(position);

       // HAL_Delay(10);
       osDelay(5);
       
       // 恢复机械臂定时器
       HAL_TIM_Base_Start_IT(&htim5);
   }

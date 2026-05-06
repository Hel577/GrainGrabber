#include "lift.h"


// 全局变量定义
static Lift_Status lift_status;
Lift_Status *lift = &lift_status;


//height高度使用电机转动弧度，单位rad
/**
 * @brief 初始化升降机构
 */
void Init_Lift(void)
{
    
    // 初始化升降机构状态
    lift->current_height = 0.0f;
    lift->target_height = 0.0f;
    lift->max_height = 15.0f;    // 设置最大高度限制，根据实际情况调整
    lift->min_height = 0.0f;      // 设置最小高度限制
    lift->error_max = 0.2f;
    lift->is_moving = false;
    lift->is_initialized = true;

}

/**
 * @brief 复位升降机构状态
 */
void Lift_Reset(void)
{
    lift->target_height = 0.0f;
    lift->is_moving = true;
    
    // 移动到零位置
    MI_motor_PosCtrl(motors[7], 0.0f);
}



/**
 * @brief 移动到指定高度
 * @param height 目标高度,使用电机转动弧度代替，单位rad
 */
void Lift_Move_To_Height(float height)
{
    lift->is_moving = true;
    // 限制高度在有效范围内
    float target_height = fmaxf(lift->min_height, fminf(height, lift->max_height));
    lift->target_height = target_height;
    // 使用位置模式控制电机
    MI_motor_PosCtrl(motors[7], target_height);
}



/**
 * @brief 检查是否到达目标高度
 * @return 是否到达目标高度
 */
bool Lift_Is_Reached(void)
{
    MI_motor_get_mechPos(motors[7]);
    lift->current_height = -1.0f*motors[7]->motor_fdb.mechPos;
    float error = fabsf(lift->target_height - lift->current_height);
    // printf("error: %f\r\n", error);
    if (error < lift->error_max)
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
   void Lift_PosCtrl(float height)
   {
      HAL_TIM_Base_Stop_IT(&htim5);
       
       // 发送升降指令
       Lift_Move_To_Height(height);
       
       HAL_Delay(5);
       // 等待指令被接收
       Lift_Move_To_Height(height);

       // HAL_Delay(10);
       osDelay(5);
       
       // 恢复机械臂定时器
       HAL_TIM_Base_Start_IT(&htim5);
   }

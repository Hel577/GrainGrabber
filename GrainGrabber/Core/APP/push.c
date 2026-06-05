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
    push->current_angle = 0.0f;
    push->target_angle = 0.0f;
    push->step_angle = 0.3f;       // 设置步进角度（单位为rad）
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
    push_Move_To_Position(0.0f);
}

void Calculate_Angles_From_Position(float position, float* angle)
{
    float para = ARM_LONG_LENGTH*ARM_LONG_LENGTH-ARM_SHORT_LENGTH*ARM_SHORT_LENGTH-position*position;
    float radias = acosf(-para/(2*position*ARM_SHORT_LENGTH));
    *angle = radias;
}



/**
 * @brief 移动到指定高度
 * @param height 目标高度,使用电机转动弧度代替，单位rad
 */
void push_Move_To_Position(float position)
{
    HAL_TIM_Base_Stop_IT(&htim5);
    push->is_moving = true;
    // 限制高度在有效范围内
    float target_position = fmaxf(MIN_POSITION, fminf(position, MAX_POSITION));
    push->target_position = target_position;
    // 使用位置模式控制电机
    float target_angle;
    Calculate_Angles_From_Position(target_position, &target_angle);
    push->target_angle = target_angle;
    HAL_TIM_Base_Start_IT(&htim5);
}



/**
 * @brief 检查是否到达目标高度
 * @return 是否到达目标高度
 */
void push_Update(void)
{
    push->current_angle = motors[7]->motor_fdb.angle;
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


 /**
  * @brief 增量控制(定时器中使用)
  * @retval 无
  */
void add_push_ctrl(void)
{
  // 使用静态变量记录当前控制的是哪个电机
  // static uint8_t motor_index = 0;
  
  // 计算左臂电机步进
  float diff1 = push->target_angle - push->current_angle;
  float step1 = 0.0f;
  if(fabsf(diff1) < push->step_angle) {
      step1 = diff1;
      push->is_moving = 0;
  } else {
      // 接近目标位置时减小步进角度
      if(fabsf(diff1) < 9.5f) {
          float reduced_step = push->step_angle * 0.4f; // 减小为原来的一半
          step1 = (diff1 > 0) ? reduced_step : -reduced_step;
      } else {
          step1 = (diff1 > 0) ? push->step_angle : -push->step_angle;
      }
  }

  // 计算目标角度（rad）
  float target_angle1 = push->current_angle + step1;
  

    float angle1 = target_angle1;
    float angle1_rad = angle1;
    MI_motor_PosCtrl(motors[7], angle1_rad);
}
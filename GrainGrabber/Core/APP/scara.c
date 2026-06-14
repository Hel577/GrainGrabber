#include "scara.h"



//电机位置
//                                   正前方   ^  y             
//                               th3          *               th4
//                                            *        
//                               &&           *     x          &&
//                               &&           * * * ****>      &&
//                              电机5          原点            电机6
//                              th1                            th2


// 全局变量定义
SCARA_t scara;
hand_t hand;

/**
 * @brief 初始化SCARA机械臂
 * @retval None
 */
void Init_Scara(void)
{
    // 初始化SCARA结构体
    scara.L_motor = motors[5];
    scara.R_motor = motors[6];
    scara.current_th1 = 0.0f;
    scara.current_th2 = 0.0f;
    scara.target_th1 = 0.0f;
    scara.target_th2 = 0.0f;
    scara.step_angle = 20.5f;// 增量控制步进角度（°）//空爪可以切换大的
    scara.is_moving = 0;

    //初始化末端爪子结构体
    hand.current_angle = 0.0f; // 初始末端角度为0（与x轴平行）
    hand.target_angle = 0.0f;
    hand.step_angle = 30.0f; // 增量控制步进角度（°）
    hand.grab_state = 0; // 初始爪子松开
    hand.grab_target_angle = 2048;
}




/**
 * @brief 检测工作空间
 * @retval None
 * 只通过同向情况判断工作空间
 * 1. 电机1角度(th1)在0-180°之间，电机2角度(th2)在0-180°之间，则工作空间为正
 * 2. 电机1角度(th1)在180-360°之间，电机2角度(th2)在-180-0°之间，则工作空间为负
 * 3. 其他情况，返回
 */
void detect_workspace(void)
{
  return;
  // 计算当前电机角度
  // float th1 = scara.current_th1;
  // float th2 = scara.current_th2;
}
  


void Update_Scara_Status(void)
{
  // MI_motor_get_mechPos(scara.L_motor);
  // MI_motor_get_mechPos(scara.R_motor);
  scara.current_th1 = 180.0f*scara.L_motor->motor_fdb.angle/PI;
  scara.current_th2 = 180.0f*scara.R_motor->motor_fdb.angle/PI;
}

/**
 * @brief 设置SCARA机械臂模型位置
 * @param angle1 实际模型电机1角度(°)
 * @param angle2 实际模型电机2角度(°)
 * @retval 无
 */
void set_scara_position(float angle1, float angle2)
{
  float angle1_rad = angle1 * PI / 180.0f;
  float angle2_rad = angle2 * PI / 180.0f;
  MI_motor_PosCtrl(scara.L_motor, angle1_rad);
  // HAL_Delay(5);
  MI_motor_PosCtrl(scara.R_motor, angle2_rad);
}
 /**
  * @brief 增量控制(定时器中使用)
  * @retval 无
  */
void add_scara_ctrl(void)
{
  // 使用静态变量记录当前控制的是哪个电机
  // static uint8_t motor_index = 0;
  
  // 计算左臂电机步进
  float diff1 = scara.target_th1 - scara.current_th1;
  float step1 = 0.0f;
  if(fabsf(diff1) < scara.step_angle*0.2f) {
      step1 = diff1;
      scara.is_moving = 0;
  } else {
      // 接近目标位置时减小步进角度
      if(fabsf(diff1) < 9.5f) {
          float reduced_step = scara.step_angle * 0.4f; // 减小为原来的一半
          step1 = (diff1 > 0) ? reduced_step : -reduced_step;
      } else {
          step1 = (diff1 > 0) ? scara.step_angle : -scara.step_angle;
      }
  }
  
  // 计算右臂电机步进
  float diff2 = scara.target_th2 - scara.current_th2;
  float step2 = 0.0f;
  if(fabsf(diff2) < scara.step_angle*0.2f) {
      step2 = diff2;
  } else {
      // 接近目标位置时减小步进角度
      if(fabsf(diff2) < 4.5f) {
          float reduced_step = scara.step_angle * 0.4f; // 减小为原来的一半
          step2 = (diff2 > 0) ? reduced_step : -reduced_step;
      } else {
          step2 = (diff2 > 0) ? scara.step_angle : -scara.step_angle;
      }
  }
  
  // 计算目标角度（度）
  float target_angle1 = scara.current_th1 + step1;
  float target_angle2 = scara.current_th2 + step2;
  
  // 轮流控制两个电机
  // if(motor_index == 0) {
    // 控制左臂电机
    float angle1 = target_angle1;
    float angle1_rad = angle1 * PI / 180.0f;
    MI_motor_PosCtrl(scara.L_motor, angle1_rad);
    // printf("send angle1: %f\r\n", angle1);
    // motor_index = 1;  // 下一次控制右臂电机
  // } else {
    // 控制右臂电机
    float angle2 = target_angle2;
    float angle2_rad = angle2 * PI / 180.0f;
    MI_motor_PosCtrl(scara.R_motor, angle2_rad);
    // printf("send angle2: %f\r\n", angle2);
    // motor_index = 0;  // 下一次控制左臂电机
  // }
}



/**
 * @brief 设置SCARA机械臂目标位置（定时器中会调用add_scara_ctrl()增量控制）
 * @param angle1 实际模型电机1角度(°)
 * @param angle2 实际模型电机2角度(°)
 * @retval 无
 */
void Scara_PosCtrl(float angle1, float angle2)
{
  scara.target_th1 = angle1;
  scara.target_th2 = -angle2;
  scara.is_moving = 1;
  HAL_TIM_Base_Start_IT(&htim5);

}

void Scara_To_Height(float height)
{
  // 根据目标高度计算对应的电机角度
  // 默认为最高高度，向下为正值
  float target_angle1, target_angle2;
  Calculate_Angles_From_Height(height, &target_angle1, &target_angle2);
  
  // 设置目标角度
  Scara_PosCtrl(target_angle1, target_angle2);
}

void Calculate_Angles_From_Height(float height, float* angle1, float* angle2)
{
  // 根据机械臂的几何关系计算电机角度
  // 这里需要根据具体的机械臂结构进行计算，以下是一个示例
  *angle1 = height / RADIUS * 180.0f / PI; // 简单的线性关系，实际可能需要更复杂的计算
  *angle2 = *angle1; // 两个电机目标角度相同，实际可能需要根据机械臂结构进行调整
}
  

void Scara_Return_Home(void)
{
  Scara_PosCtrl(0.0f, 0.0f);
}
//*********************************** 末端爪子控制 ***********************************
/**
 * @brief 控制末端爪子旋转到指定角度
 * @param angle 目标角度（单位：度，0度为零位，逆时针为正）
 */
void Control_End_Rotation(float angle)
{
    // 角度转舵机位置
    int16_t target_position = 2048 + (int16_t)(angle / END_ANGLE_RATE + 0.5f); // 四舍五入
    uint16_t speed = 2100;
    uint8_t acceleration = 130;
    // 调用已有的舵机控制函数
    Filter_Servo_PosCtrl(SPIN_SERVO, target_position, speed, acceleration);
}


//保持末端角度，定时器中使用
void Maintain_End_Rotation(void)
{
  float diff = hand.target_angle;

    Control_End_Rotation(diff);
}


//设置末端目标角度
void End_Rotation_Ctrl(float maintain_angle)
{
   hand.target_angle = maintain_angle;
  HAL_TIM_Base_Start_IT(&htim5);

}

void Read_Spin_Angle(void){
  Filter_Read_Pos(SPIN_SERVO);
}

void Grab_On(void)
{
  HAL_TIM_Base_Stop_IT(&htim5);
  hand.grab_state = 1;
  hand.grab_target_angle = GRAB_ClOSE_All;
  osDelay(10);
  HAL_TIM_Base_Start_IT(&htim5);

}

void Grab_Off(void)
{
  HAL_TIM_Base_Stop_IT(&htim5);
  hand.grab_state = 0;
  hand.grab_target_angle = GRAB_OPEN;
  osDelay(10);
  HAL_TIM_Base_Start_IT(&htim5);
}

void Grab_Pos_Ctrl(uint16_t angle){
  HAL_TIM_Base_Stop_IT(&htim5);
  hand.grab_state = 0;
  hand.grab_target_angle = angle;
  osDelay(10);
  HAL_TIM_Base_Start_IT(&htim5);
}

void Grab_Open_Slitly(void){
  HAL_TIM_Base_Stop_IT(&htim5);
  Filter_Servo_PosCtrl(GRAB_SERVO,hand.grab_current_angle-350,3400,200);
  osDelay(100);
  Filter_Servo_PosCtrl(GRAB_SERVO,GRAB_ClOSE_All,3400,200);
  osDelay(20);
  HAL_TIM_Base_Start_IT(&htim5);
}

void Grab_Realease(void){
  /*释放豆子*/
  HAL_TIM_Base_Stop_IT(&htim5);
  hand.grab_state = 0;
  hand.grab_target_angle = GRAB_REALEASE;
  osDelay(10);
  HAL_TIM_Base_Start_IT(&htim5);
}

void Read_Grab_Angle(void){
  Filter_Read_Pos(GRAB_SERVO);
}

void Filter_Process_Data(uint8_t* data)
{
    if(Filter_Verify_Checksum(&data[2],6)){
      if(data[0]==FILTER_HEADER1&&data[1]==FILTER_HEADER2){
          uint8_t id = data[2];
          uint16_t angle = ((uint16_t)data[6]<<8)+(uint16_t)data[5];
          if(id==GRAB_SERVO){
              hand.grab_current_angle = angle;
          }
          if(id==SPIN_SERVO){
              hand.current_angle = angle;
          }
      }
    }
}


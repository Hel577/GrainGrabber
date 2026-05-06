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
    scara.current_th1 = OFFSET_ANGLE;
    scara.current_th2 = 180.0f - OFFSET_ANGLE;
    scara.current_th3 = OFFSET_ANGLE;
    scara.current_th4 = 180.0f - OFFSET_ANGLE;
    scara.end_x = 0.0f;
    scara.end_y = 407.9215f;
    scara.target_th1 = OFFSET_ANGLE;
    scara.target_th2 = 180.0f - OFFSET_ANGLE;
    scara.step_angle = 10.5f;// 增量控制步进角度（°）//空爪可以切换大的
    scara.is_moving = 0;
    scara.workspace = 0; // 初始在正工作空间

    //初始化末端爪子结构体
    hand.current_angle = 0.0f; // 初始末端角度为0（与x轴平行）
    hand.target_angle = 0.0f;
    hand.step_angle = 30.0f; // 增量控制步进角度（°）
    hand.grab_state = 0; // 初始爪子松开
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
  // 计算当前电机角度
  float th1 = scara.current_th1;
  float th2 = scara.current_th2;
  if ((th1 > 0.0f && th1 < 180.0f && th2 > 0.0f && th2 < 180.0f) || 
  (th1 > 0.0f && th1 < 180.0f && th2 > -180.0f && th2 < 0.0f) || 
  (th1 > 180.0f && th1 < 360.0f && th2 > 0.0f && th2 < 180.0f)) {
    scara.workspace = 0;
  } 
  else if ((th1 > 180.0f && th1 < 360.0f && th2 > -180.0f && th2 < 0.0f) ) {
    scara.workspace = 1;
  } 
  else{
    
  }
}
  
// }

/**
  * @brief  正运动学：根据电机角度计算末端位置，同时得到小臂角度（实际模型的角度，而非电机直接的反馈角度，需要补上偏置）
  * @param  motor1_angle 电机1角度(rad)
  * @param  motor2_angle 电机2角度(rad)
  * @param  x 末端X坐标指针
  * @param  y 末端Y坐标指针
  * @retval 无
  */
void SCARA_ForwardKinematics( float angle1, float angle2)
{
    angle1 = angle1 * PI / 180.0f;
    angle2 = angle2 * PI / 180.0f;
    // 电机1位置（左侧）
    float m1_x = -MOTOR_DISTANCE/2;
    float m1_y = 0;
    
    // 电机2位置（右侧）
    float m2_x = MOTOR_DISTANCE/2;
    float m2_y = 0;
    
    // 大臂末端位置
    float p1_x = m1_x + LARGE_ARM_LENGTH * cosf(angle1);
    float p1_y = m1_y + LARGE_ARM_LENGTH * sinf(angle1);
    
    float p2_x = m2_x + LARGE_ARM_LENGTH * cosf(angle2);
    float p2_y = m2_y + LARGE_ARM_LENGTH * sinf(angle2);
    
    // 计算两大臂末端间距离
    float p1p2_dist = sqrtf((p2_x - p1_x) * (p2_x - p1_x) + (p2_y - p1_y) * (p2_y - p1_y));
    
    // 计算末端位置
    float h = sqrtf(SMALL_ARM_LENGTH * SMALL_ARM_LENGTH - (p1p2_dist / 2) * (p1p2_dist / 2));
    
    // 计算两大臂末端连线的中点
    float mid_x = (p1_x + p2_x) / 2;
    float mid_y = (p1_y + p2_y) / 2;
    
    // 计算垂直于两大臂末端连线的单位向量
    float dx = p2_x - p1_x;
    float dy = p2_y - p1_y;
    float norm = sqrtf(dx * dx + dy * dy);
    
    // 垂直单位向量
    float ux = -dy / norm;
    float uy = dx / norm;
    detect_workspace();
    // 选择向外凸的解（即在两个关节之外）
    if (scara.workspace == 0) {
        // 在正工作空间，末端位置在两个关节之外
        scara.end_x = mid_x + ux * h;
        scara.end_y = mid_y + uy * h;
    } 
    else if (scara.workspace == 1) {
        // 在负工作空间，末端位置在两个关节之外
        scara.end_x = mid_x - ux * h;
        scara.end_y = mid_y - uy * h;

    }
    // 计算小臂与水平方向的夹角
    scara.current_th3 = 180.0f*atan2f((scara.end_y - p1_y), (scara.end_x - p1_x))/PI;
    scara.current_th4 = 180.0f*atan2f(fabsf(scara.end_y - p2_y), fabsf(scara.end_x - p2_x))/PI;
    
}

void Update_Scara_Status(void)
{
  // MI_motor_get_mechPos(scara.L_motor);
  // MI_motor_get_mechPos(scara.R_motor);
  scara.current_th1 = 180.0f*scara.L_motor->motor_fdb.angle/PI + OFFSET_ANGLE;
  scara.current_th2 = 180.0f*scara.R_motor->motor_fdb.angle/PI + (180.0f - OFFSET_ANGLE);
  SCARA_ForwardKinematics(scara.current_th1, scara.current_th2);
}

/**
 * @brief 设置SCARA机械臂模型位置
 * @param angle1 实际模型电机1角度(°)
 * @param angle2 实际模型电机2角度(°)
 * @retval 无
 */
void set_scara_position(float angle1, float angle2)
{
  angle1 = angle1 - OFFSET_ANGLE;
  angle2 = angle2 - (180.0f - OFFSET_ANGLE);
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
  if(fabsf(diff1) < scara.step_angle) {
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
  if(fabsf(diff2) < scara.step_angle) {
      step2 = diff2;
  } else {
      // 接近目标位置时减小步进角度
      if(fabsf(diff2) < 10.5f) {
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
    float angle1 = target_angle1 - OFFSET_ANGLE;
    float angle1_rad = angle1 * PI / 180.0f;
    MI_motor_PosCtrl(scara.L_motor, angle1_rad);
    // motor_index = 1;  // 下一次控制右臂电机
  // } else {
    // 控制右臂电机
    float angle2 = target_angle2 - (180.0f - OFFSET_ANGLE);
    float angle2_rad = angle2 * PI / 180.0f;
    MI_motor_PosCtrl(scara.R_motor, angle2_rad);
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
  scara.target_th2 = angle2;
  scara.is_moving = 1;
  HAL_TIM_Base_Start_IT(&htim5);

}
void Scara_Return_Home(void)
{
  Scara_PosCtrl(OFFSET_ANGLE,180-OFFSET_ANGLE);
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
  float diff = hand.target_angle + OFFSET_ANGLE - scara.current_th3;

    Control_End_Rotation(diff);

}


//设置末端目标角度
void End_Rotation_Ctrl(float maintain_angle)
{
   hand.target_angle = maintain_angle;
  HAL_TIM_Base_Start_IT(&htim5);

}

void Grab_On(void)
{
  HAL_TIM_Base_Stop_IT(&htim5);
  hand.grab_state = 1;
  Filter_Servo_PosCtrl(GRAB_SERVO,GRAB_BOX,3400,200);
  osDelay(3);
  Filter_Servo_PosCtrl(GRAB_SERVO,GRAB_BOX,3400,200);
  osDelay(10);
  HAL_TIM_Base_Start_IT(&htim5);

}

void Grab_Off(void)
{
  HAL_TIM_Base_Stop_IT(&htim5);
  hand.grab_state = 0;
  Filter_Servo_PosCtrl(GRAB_SERVO,GRAB_OPEN,3400,200);
  osDelay(3);
  Filter_Servo_PosCtrl(GRAB_SERVO,GRAB_OPEN,3400,200);
  osDelay(10);
  HAL_TIM_Base_Start_IT(&htim5);
}



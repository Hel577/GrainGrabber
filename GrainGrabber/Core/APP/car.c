#include "car.h"

// 电机位置
//                                  X
//                                  ^
//                   motors[1]      *   motors[2]
//                                  *
//                                  *
//                                  *
//                 Y <* * * * * * * * * * * * * * *
//                                  *
//                                  *
//                                  *
//                   motors[3]      *   motors[4]
//
//                             Z轴逆时针为正
//

// 角度校准为0
void omega_zero()
{
    osDelay(200);
    uint8_t txcmd_5[16] = {0};

    // //解锁寄存器
    // txcmd_5[0] = 0xFF;
    // txcmd_5[1] = 0xAA;
    // txcmd_5[2] = 0x69;
    // txcmd_5[3] = 0x88;
    // txcmd_5[4] = 0xB5;
    // HAL_UART_Transmit(&huart6, txcmd_5, 5, 100);
    // osDelay(100);

    // //z轴置零
    // txcmd_5[0] = 0xFF;
    // txcmd_5[1] = 0xAA;
    // txcmd_5[2] = 0x76;
    // txcmd_5[3] = 0x00;
    // txcmd_5[4] = 0x00;
    // HAL_UART_Transmit(&huart6, txcmd_5, 5, 100);
    // osDelay(100);

    //保存
    txcmd_5[0] = 0xFF;
    txcmd_5[1] = 0xAA;
    txcmd_5[2] = 0x00;
    txcmd_5[3] = 0xFF;
    txcmd_5[4] = 0x00;
    HAL_UART_Transmit(&huart6, txcmd_5, 5, 100);
    osDelay(200);
}

//自动获取零偏（需要等待20s）
void auto_offset_omega()
{
    uint8_t txcmd_5[16] = {0};
    
    // 1. 发送自动获取零偏指令: FF AA 48 01 00
    txcmd_5[0] = 0xFF;
    txcmd_5[1] = 0xAA;
    txcmd_5[2] = 0x48;
    txcmd_5[3] = 0x01;
    txcmd_5[4] = 0x00;
    HAL_UART_Transmit(&huart6, txcmd_5, 5, 100);
    
    // 2. 延时20秒，等待陀螺仪完成零偏采集
    // HAL_Delay(20000);
    osDelay(22000);
    
    // 3. 发送正常模式指令: FF AA 48 00 00
    txcmd_5[0] = 0xFF;
    txcmd_5[1] = 0xAA;
    txcmd_5[2] = 0x48;
    txcmd_5[3] = 0x00;
    txcmd_5[4] = 0x00;
    HAL_UART_Transmit(&huart6, txcmd_5, 5, 100);
}
//打开陀螺仪全程自动获取零偏模式
void Hwt_auto_get_offset_ON()
{
    uint8_t txcmd_5[16] = {0};
    // //解锁寄存器
    // txcmd_5[0] = 0xFF;
    // txcmd_5[1] = 0xAA;
    // txcmd_5[2] = 0x69;
    // txcmd_5[3] = 0x88;
    // txcmd_5[4] = 0xB5;
    // HAL_UART_Transmit(&huart6, txcmd_5, 5, 100);
    // osDelay(100);

    txcmd_5[0] = 0xFF;
    txcmd_5[1] = 0xAA;
    txcmd_5[2] = 0xA7;
    txcmd_5[3] = 0x00;
    txcmd_5[4] = 0x00;
    HAL_UART_Transmit(&huart6, txcmd_5, 5, 100);
    osDelay(200);

    // //保存
    // txcmd_5[0] = 0xFF;
    // txcmd_5[1] = 0xAA;
    // txcmd_5[2] = 0x00;
    // txcmd_5[3] = 0x00;
    // txcmd_5[4] = 0x00;
    // HAL_UART_Transmit(&huart6, txcmd_5, 5, 100);
    // osDelay(100);

}

//关闭陀螺仪全程自动获取零偏模式
void Hwt_auto_get_offset_OFF()
{
    uint8_t txcmd_5[16] = {0};
    txcmd_5[0] = 0xFF;
    txcmd_5[1] = 0xAA;
    txcmd_5[2] = 0xA7;
    txcmd_5[3] = 0x01;
    txcmd_5[4] = 0x00;
    HAL_UART_Transmit(&huart6, txcmd_5, 5, 100);
}
// 辅助函数：符号函数
float sign(float val)
{
    if (val > 0)
        return 1.0f;
    if (val < 0)
        return -1.0f;
    return 0.0f;
}
void Init_Imu(void)
{
    omega_zero();
    Hwt_auto_get_offset_ON();
    // Hwt_auto_get_offset_OFF();
    // omega_zero();
    Enable_IMU_Interrupts();
}

// 初始化车辆状态
Car_Status *car = NULL;
void Init_Car()
{
    car = (Car_Status *)malloc(sizeof(Car_Status));
    if (car != NULL)
    {
        
        car->current_car_Vx = 0.0f; 
        car->current_car_Vy = 0.0f; 
        car->target_car_Vx = 0.0f;
        car->target_car_Vy = 0.0f;

        car->delta_car_pos_x = 0.0f;  
        car->delta_car_pos_y = 0.0f;  
        car->target_car_pos_x = 0.0f; 
        car->target_car_pos_y = 0.0f; 

        car->target_Vel1 = 0.0f; 
        car->target_Vel2 = 0.0f;
        car->target_Vel3 = 0.0f;
        car->target_Vel4 = 0.0f;

        car->current_Vel1 = 0.0f; 
        car->current_Vel2 = 0.0f;
        car->current_Vel3 = 0.0f;
        car->current_Vel4 = 0.0f;

        car->current_map_Vx = 0.0f; 
        car->current_map_Vy = 0.0f; 
        car->target_map_Vx = 0.0f;
        car->target_map_Vy = 0.0f;

        car->current_map_pos_x = 0.0f; 
        car->current_map_pos_y = 0.0f; 
        car->target_map_pos_x = 0.0f;
        car->target_map_pos_y = 0.0f;

        car->current_angle = 0.0f;
        car->current_hwt_angle_speed = 0.0f;
        car->current_angle_vel = 0.0f;
        car->target_angle_speed = 0.0f; 

        car->car_mode = CAR_STOP;
        car->car_face = Forward;
    }
}
// 重置车辆状态
void Reset_Car_Status()
{
    car->current_car_Vx = 0.0f; // x方向速度
    car->current_car_Vy = 0.0f; //
    car->target_car_Vx = 0.0f;
    car->target_car_Vy = 0.0f;

    car->delta_car_pos_x = 0.0f;  // 上一次微分项
    car->delta_car_pos_y = 0.0f;  // 上一次微分项
    car->target_car_pos_x = 0.0f; // 上一次微分项
    car->target_car_pos_y = 0.0f; // 上一次微分项

    car->target_Vel1 = 0.0f;
    car->target_Vel2 = 0.0f;
    car->target_Vel3 = 0.0f;
    car->target_Vel4 = 0.0f;

    car->current_Vel1 = 0.0f;
    car->current_Vel2 = 0.0f;
    car->current_Vel3 = 0.0f;
    car->current_Vel4 = 0.0f;

    car->current_map_Vx = 0.0f; //
    car->current_map_Vy = 0.0f; //
    car->target_map_Vx = 0.0f;
    car->target_map_Vy = 0.0f;

    car->current_map_pos_x = 0.0f; // x方向
    car->current_map_pos_y = 0.0f; //
    car->target_map_pos_x = 0.0f;
    car->target_map_pos_y = 0.0f;

    car->current_angle = 0.0f;
    car->current_hwt_angle_speed = 0.0f;
    car->current_angle_vel = 0.0f;
    car->target_angle_speed = 0.0f; // 重置目标角速度
    car->car_mode = CAR_STOP;

    motors[1]->first_run = true;
    motors[2]->first_run = true;
    motors[3]->first_run = true;
    motors[4]->first_run = true;
}

//从电机与陀螺仪状态解算车辆状态
void From_Motor_to_Car_Status(void)
{  
    // 关闭特定中断,防止数据竞争
    Disable_IMU_Interrupts();
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn); 

    float angle = omega * PI / 180; // 角度转弧度
    // 将角度规范化到0-360度范围
    float normalized_angle = fmod(omega, 360.0f);
    if (normalized_angle < 0)
        normalized_angle += 360.0f;

    // 根据规范化后的角度判断朝向
    if (normalized_angle <= 45.0f || normalized_angle > 315.0f)
    {
        car->car_face = Forward;
    }
    else if (normalized_angle > 45.0f && normalized_angle <= 135.0f)
    {
        car->car_face = Left;
    }
    else if (normalized_angle > 135.0f && normalized_angle <= 225.0f)
    {
        car->car_face = Back;
    }
    else if (normalized_angle > 225.0f && normalized_angle <= 315.0f)
    {
        car->car_face = Right;
    }

    car->current_angle = omega;
    car->current_hwt_angle_speed = current_angle_speed;
    car->current_Vel1 = motors[1]->current_speed;
    car->current_Vel2 = motors[2]->current_speed;
    car->current_Vel3 = motors[3]->current_speed;
    car->current_Vel4 = motors[4]->current_speed;

    car->current_car_Vx = (car->current_Vel1 + car->current_Vel2 + car->current_Vel3 + car->current_Vel4) / 4;
    car->current_car_Vy = (-car->current_Vel1 + car->current_Vel2 + car->current_Vel3 - car->current_Vel4) / 4;
    // 解算角速度
    car->current_angle_vel =  (-car->current_Vel1 + car->current_Vel2 - car->current_Vel3 + car->current_Vel4) / ((sqrtf((CAR_W * CAR_W + CAR_S * CAR_S)))/19.8);
    car->current_map_Vx = car->current_car_Vx * cosf(angle) - car->current_car_Vy * sinf(angle);
    car->current_map_Vy = car->current_car_Vx * sinf(angle) + car->current_car_Vy * cosf(angle);
    
    car->delta_car_pos_x = (+motors[1]->delta_distance_traveled + motors[2]->delta_distance_traveled + motors[3]->delta_distance_traveled + motors[4]->delta_distance_traveled) / 4;
    car->delta_car_pos_y = (-motors[1]->delta_distance_traveled + motors[2]->delta_distance_traveled + motors[3]->delta_distance_traveled - motors[4]->delta_distance_traveled) / 4;

    // 更新当前位置
    car->current_map_pos_x += car->delta_car_pos_x * cosf(angle) - car->delta_car_pos_y * sinf(angle);
    car->current_map_pos_y += car->delta_car_pos_x * sinf(angle) + car->delta_car_pos_y * cosf(angle);

    // 恢复中断
    Enable_IMU_Interrupts();
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

// 更新车辆状态
void Update_Car_Status(void)
{
    for (int i = 1; i < 5; i++)
    {
        // MI_motor_get_speed(motors[i]);//主动获取速度，也可通过速度控制反馈帧获取
        // HAL_Delay(1);
        MI_motor_get_mechPos(motors[i]);
        // HAL_Delay(1);
        osDelay(1);
    }
    From_Motor_to_Car_Status();  
    // printf("omega: %f, car_face: %d\r\n", car->current_angle, car->car_face);
}

// 发布车辆速度
void Publish_Car_Speed(void)
{
    float angle = car->current_angle * PI / 180.0f; // 角度转弧度
    // 根据map_speed计算car_speed
    car->target_car_Vx = car->target_map_Vx * cosf(angle) + car->target_map_Vy * sinf(angle);
    car->target_car_Vy = -car->target_map_Vx * sinf(angle) + car->target_map_Vy * cosf(angle);
    // 根据car_speed计算motor_speed

    float Kx = 1;
    float Ky = 1;
    float Kz = 0.70119; // 将target和current单位统一

    car->target_Vel1 = Kx * car->target_car_Vx - Ky * car->target_car_Vy + Kz * (-car->target_angle_speed * (sqrtf(CAR_W * CAR_W + CAR_S * CAR_S)) * PI / 180.0);
    car->target_Vel2 = Kx * car->target_car_Vx + Ky * car->target_car_Vy + Kz * (+car->target_angle_speed * (sqrtf(CAR_W * CAR_W + CAR_S * CAR_S)) * PI / 180.0);
    car->target_Vel3 = Kx * car->target_car_Vx + Ky * car->target_car_Vy + Kz * (-car->target_angle_speed * (sqrtf(CAR_W * CAR_W + CAR_S * CAR_S)) * PI / 180.0);
    car->target_Vel4 = Kx * car->target_car_Vx - Ky * car->target_car_Vy + Kz * (+car->target_angle_speed * (sqrtf(CAR_W * CAR_W + CAR_S * CAR_S)) * PI / 180.0);

    // 线性平滑
    // car->target_Vel1 = car->target_Vel1 * 0.5 + car->current_Vel1 * 0.5;
    // car->target_Vel2 = car->target_Vel2 * 0.5 + car->current_Vel2 * 0.5;
    // car->target_Vel3 = car->target_Vel3 * 0.5 + car->current_Vel3 * 0.5;
    // car->target_Vel4 = car->target_Vel4 * 0.5 + car->current_Vel4 * 0.5;
    
    uint32_t delay_ms = 2;
    //发布电机速度
    Publish_Chassis_Motor_Speed(car->target_Vel1, car->target_Vel2, car->target_Vel3, car->target_Vel4,delay_ms);
}


/**
 * @brief  停止车辆(mode为0速度模式速度置零,mode为1重置机械零位切换位置模式)
 * @param1 mode 0:速度模式速度置零,1:重置机械零位切换位置模式
 * @retval null
 */
void Car_Stop(bool mode)
{
   car->car_mode = CAR_STOP;
   if(mode == false)
   {
        for (int i = 1; i < 5; i++)
    {
        if(motors[i]->Control_mode != MODE_SPD)
        {
            MI_motor_setMode(motors[i], MODE_SPD);
        }
    }
    Publish_Chassis_Motor_Speed(0,0,0,0,2);
   }
   else if(mode == true)
   {
    Reset_Chassis_Motor_MechPosition();
    Change_Chassis_Motor_Mode(MODE_POS);
   }
}


//#################################################################纯电机位置模式####################################################################
// 位置模式
void Move_TransformX(float X  ,float Vx)//X单位：mm，Vx单位：rad/s
{
    Reset_Chassis_Motor_MechPosition();
    for (int i = 1; i < 5; i++)
    {
        MI_motor_SetSpdLim(motors[i], Vx );
        if (motors[i]->Control_mode != MODE_POS)
        {
            MI_motor_setMode(motors[i], MODE_POS);
        }
    }

    float Target1, Target2, Target3, Target4 = 0.0f;
    Target3 = -X / WHEEL_RADIUS;
    Target1 = -X / WHEEL_RADIUS;
    Target2 = X / WHEEL_RADIUS;
    Target4 = X / WHEEL_RADIUS;

    MI_motor_PosCtrl(motors[1],Target1);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[3],Target3);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[2],Target2);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[4],Target4);

}

void Move_TransformY(float Y ,float Vy)//Vy单位：rad/s
{
    Reset_Chassis_Motor_MechPosition();
    for (int i = 1; i < 5; i++)
    {
        MI_motor_SetSpdLim(motors[i], Vy);
        if (motors[i]->Control_mode != MODE_POS)
        {
            MI_motor_setMode(motors[i], MODE_POS);
        }
    }
    float Target1, Target2, Target3, Target4 = 0.0f;

    Target3 = -Y / WHEEL_RADIUS; 
    Target1 = +Y / WHEEL_RADIUS;
    Target2 = +Y / WHEEL_RADIUS;
    Target4 = -Y / WHEEL_RADIUS;

    MI_motor_PosCtrl(motors[1],Target1);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[3],Target3);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[2],Target2);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[4],Target4);

}

void Move_TransformXY(float XY, float Vxy) //
{
    Reset_Chassis_Motor_MechPosition();
    for (int i = 1; i < 5; i++)
    {
        MI_motor_SetSpdLim(motors[i], Vxy);
        if (motors[i]->Control_mode != MODE_POS)
        {
            MI_motor_setMode(motors[i], MODE_POS);
        }
    }
    float Target1, Target2, Target3, Target4 = 0.0f;

    Target3 = -  2*XY/WHEEL_RADIUS;
    Target1 = 0;
    Target2 =   2*XY/WHEEL_RADIUS;
    Target4 = 0;

    MI_motor_PosCtrl(motors[1],Target1);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[3],Target3);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[2],Target2);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[4],Target4);

}
void Move_At_Angle(float angle, float distance, float speed)//angle单位：°，distance单位：mm，speed单位：mm/s
{
    /*沿着某一个角度移动一定距离 */
    Reset_Chassis_Motor_MechPosition();

    float angle_rad = angle * PI / 180.0f;

    // 计算小车坐标系下的目标位移分量
    float target_car_Dx = distance * cosf(angle_rad);
    float target_car_Dy = distance * sinf(angle_rad);


    float physical_target_rad[4];
    physical_target_rad[0] = (target_car_Dx - target_car_Dy) / WHEEL_RADIUS; // 电机1
    physical_target_rad[1] = (target_car_Dx + target_car_Dy) / WHEEL_RADIUS; // 电机2
    physical_target_rad[2] = (target_car_Dx + target_car_Dy) / WHEEL_RADIUS; // 电机3
    physical_target_rad[3] = (target_car_Dx - target_car_Dy) / WHEEL_RADIUS; // 电机4

    // 计算每个轮子的角速度限制 (rad/s)
    // 与目标转动角度成比例
    float speed_limit_rad_s[4];
    speed_limit_rad_s[0] = fabsf(speed * (cosf(angle_rad) - sinf(angle_rad)) / WHEEL_RADIUS);
    speed_limit_rad_s[1] = fabsf(speed * (cosf(angle_rad) + sinf(angle_rad)) / WHEEL_RADIUS);
    speed_limit_rad_s[2] = fabsf(speed * (cosf(angle_rad) + sinf(angle_rad)) / WHEEL_RADIUS);
    speed_limit_rad_s[3] = fabsf(speed * (cosf(angle_rad) - sinf(angle_rad)) / WHEEL_RADIUS);

    // 设置速度限制和模式
    for (int i = 1; i <= 4; i++)
    {
        // 查找最大速度限制以进行可能的缩放（如果超过电机能力），这里暂时直接设置
        MI_motor_SetSpdLim(motors[i], speed_limit_rad_s[i-1]);
        if (motors[i]->Control_mode != MODE_POS)
        {
            MI_motor_setMode(motors[i], MODE_POS);
        }
    }


    MI_motor_PosCtrl(motors[1], -physical_target_rad[0]);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[3], -physical_target_rad[2]);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[2], +physical_target_rad[1]); 
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[4], +physical_target_rad[3]); 
}

void Move_TransformZ(float angle, float Vz) // angle单位：°，Vz单位：rad/s
{
    /*原地旋转一定的角度*/
    Reset_Chassis_Motor_MechPosition();
    for (int i = 1; i < 5; i++)
    {
        MI_motor_SetSpdLim(motors[i], Vz);
        if (motors[i]->Control_mode != MODE_POS)
        {
            MI_motor_setMode(motors[i], MODE_POS);
        }
    }

    float Target1, Target2, Target3, Target4 = 0.0f;
    float angle_rad = angle * (PI / 180.0f);
    float rotation_radius = sqrtf(CAR_S*CAR_S + CAR_W*CAR_W) / 2.0f;
     // 计算轮子需要移动的角度（rad）: 弧长 = 半径 × 角度
    float wheel_rad = sqrtf(2) * rotation_radius * angle_rad / WHEEL_RADIUS;
    float k = 1.0205;//修正系数
    if(Vz >10 && Vz <= 15)
    {
        k = 1.015;
    }
    else if(Vz <=10)
    {
        k = 1.0205;
    }
    wheel_rad = wheel_rad * k;
    Target1 = wheel_rad;
    Target2 = wheel_rad;
    Target3 = wheel_rad;
    Target4 = wheel_rad;

    MI_motor_PosCtrl(motors[1], Target1);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[2], Target2);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[3], Target3);
    // HAL_Delay(1);
    osDelay(1);
    MI_motor_PosCtrl(motors[4], Target4);
    
}

//#########################################################################闭环位置控制################################################################
//闭环位置控制
//单位：mm,mm,°
void Move_To_Position_XYZ(float target_x, float target_y, float target_z, uint32_t timeout)
{
    /*适合长距离移动的场景，这里应该也是相对于地图坐标系而言的坐标，
    但是这里的根据target_x设置参数就有点迷了*/

//*********************************************** */

        //将电机改为速度模式
    Dis_Chassis_Motor();
    Change_Chassis_Motor_Mode(MODE_SPD);
    En_Chassis_Motor();

//******************************************* */
    // HAL_TIM_Base_Stop_IT(&htim5);

    // //将电机改为速度模式
    // for (int i = 1; i < 5; i++)
    // {
    //     if (motors[i]->Control_mode != MODE_SPD)
    //     {
    //         MI_motor_setMode(motors[i], MODE_SPD);
    //     }
    // }
    // osDelay(20);
    //重置车辆状态
    Reset_Car_Status();
    //更新车辆状态
    Update_Car_Status();
    //重置PID
    Reset_PID(rough_X_PID);
    Reset_PID(rough_Y_PID);
    Reset_PID(rough_Z_PID);
 
    // 稳定性计数器
    uint8_t stability_counter = 0;
    uint8_t stability_counter_threshold = 4;

    // 误差阈值
    float x_error_ref = 5.0f;
    float y_error_ref = 7.0f;
    float omega_error_ref = 0.1f;
    if(fabs(target_y)>=2000 &&target_y>0)
    {
        x_error_ref = 8.0f;
        y_error_ref = 8.0f;
        omega_error_ref = 0.1f;
        stability_counter_threshold =4;     

    }
    else if((target_y)>=2000 &&target_y <0)
    {
        x_error_ref = 12.0f;
        y_error_ref = 15.0f;
        omega_error_ref = 0.1f;  
        stability_counter_threshold =2;     
    }
    else if(fabs(target_y)<1100 && fabs(target_x)<1100)
    {
        x_error_ref = 14.0f;
        y_error_ref = 14.0f;
        omega_error_ref = 0.1f;
        stability_counter_threshold = 3;
    }

    uint32_t start_time = HAL_GetTick();

    while(HAL_GetTick() - start_time < timeout)
    {
        // HAL_TIM_Base_Stop_IT(&htim5);

        
        if ((fabs(car->current_map_pos_x - target_x) < x_error_ref) && (fabs(car->current_map_pos_y - target_y) < y_error_ref) && (fabs(car->current_angle - target_z) < omega_error_ref))
        {
            // 使用静态计数器记录连续满足条件的次数
            stability_counter++;

            // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
            if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
            {

                Car_Stop(0);
                break;
            }
        }
        else
        {
            // 一旦不满足条件，立即重置计数器
            stability_counter = 0;
        }

        Update_Car_Status();
        //pid计算目标速度
        car->target_map_Vx = PID_Calc_XY(rough_X_PID, target_x, car->current_map_pos_x);
        car->target_map_Vy = PID_Calc_XY(rough_Y_PID, target_y, car->current_map_pos_y);
        car->target_angle_speed = PID_Calc_Z(rough_Z_PID, target_z, car->current_angle);

        Publish_Car_Speed();
        // printf("%f,%f,%f,%f,%f,%f\n", target_x-car->current_map_pos_x, target_y-car->current_map_pos_y, target_z-car->current_angle, car->current_map_Vx, car->current_map_Vy, car->current_hwt_angle_speed);
        // printf("%f,%f,%f\n", motors[1]->motor_fdb.speed, motors[1]->motor_fdb.torque, motors[1]->motor_fdb.temprature);
        // printf("%f,%f,%f\n", motors[2]->motor_fdb.speed, motors[2]->motor_fdb.torque, motors[2]->motor_fdb.temprature);
        // printf("%f,%f,%f\n", motors[3]->motor_fdb.speed, motors[3]->motor_fdb.torque, motors[3]->motor_fdb.temprature);
        // printf("%f,%f,%f\n", motors[4]->motor_fdb.speed, motors[4]->motor_fdb.torque, motors[4]->motor_fdb.temprature);
    }
    // 停止电机
    Car_Stop(0);
    // printf("current_map_pos_x: %f, current_map_pos_y: %f, current_angle: %f\n", car->current_map_pos_x, car->current_map_pos_y, car->current_angle);

}



//视觉闭环
/**
 * @brief 基于视觉反馈的运动控制函数
 * @param paper_id 纸垛id 0-货架箱子，1-左侧纸垛，2-左上角纸垛，3-右上角纸垛，4-右侧纸垛
 * @param timeout 超时时间（单位：毫秒）
 * @details 该函数使用视觉反馈进行精细PID控制，使车辆移动到视觉识别的目标位置
 */
void Move_By_Vision(uint8_t paper_id, uint32_t timeout)
{
    // HAL_TIM_Base_Stop_IT(&htim5);

    float target_z = 0;
    float real_x = 0;
    float real_y = 0;
        // 稳定性计数器
    uint8_t stability_counter = 0;
    uint8_t stability_counter_threshold = 7;

    if(paper_id == 1 )
    {
        target_z = 270;
        real_x = raspi.real_x[1];
        real_y = raspi.real_y[1]+2;
        if(put_round == 1)
        {
            timeout += 600;

        }
        else if(put_round>=2)
        {
            timeout += 300;
        }
    }
    else if(paper_id == 2)
    {
        target_z = 180;
        real_x = raspi.real_x[2];
        real_y = raspi.real_y[2];
        if(put_round == 1)
        {
            timeout += 3000;
            real_y += 1; // 视觉识别偏差
            stability_counter_threshold = 15;

        }
        else if(put_round>=2)
        {
            timeout += 3000;
            stability_counter_threshold = 15;
        }
    }
    else if(paper_id == 3)
    {
        target_z = 180;
        real_x = raspi.real_x[2];
        real_y = raspi.real_y[2];
        // if(put_round == 1)
        // {
        //     timeout += 400;
        // }
        if(put_round == 1)
            {
                timeout += 3000;
                stability_counter_threshold = 15;
             }
        else if(put_round>=2)
        {
            timeout += 1900;
            stability_counter_threshold = 20;
        }
    }
    else if(paper_id == 4)
    {
        target_z = 90;
        real_x = raspi.real_x[1];
        real_y = raspi.real_y[1];
        if(put_round == 1)
        {
            timeout += 400;
        }
    }
    else if(paper_id == 0)
    {
        target_z = 0;
        real_x = raspi.real_x[0];
        real_y = raspi.real_y[0];
    }
    //将电机改为速度模式
    for (int i = 1; i < 5; i++)
    {
        if (motors[i]->Control_mode != MODE_SPD)
        {
            MI_motor_setMode(motors[i], MODE_SPD);
        }
    }
        //重置车辆状态
    Reset_Car_Status();
    //更新车辆状态
    Update_Car_Status();
    //重置PID
    Reset_PID(fine_X_PID);
    Reset_PID(fine_Y_PID);
    Reset_PID(fine_Z_PID);
    // Reset_PID(rough_Z_PID);



    // 误差阈值
    float x_error_ref = 1.1f;
    float y_error_ref = 1.1f;
    float omega_error_ref = 0.1f;

    uint32_t start_time = HAL_GetTick();

    while(HAL_GetTick() - start_time < timeout)
    {

        if ((fabs(raspi.vision_x - real_x) <= x_error_ref) && (fabs(raspi.vision_y - real_y) <= y_error_ref) && (fabs(omega - target_z) < omega_error_ref))
        {
            // 使用静态计数器记录连续满足条件的次数
            stability_counter++;

            // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
            if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
            {
                Car_Stop(0);
                osDelay(8);
                Car_Stop(1);
                break;
            }
        }
        else
        {
            // 一旦不满足条件，立即重置计数器
            stability_counter = 0;
        }


        // Update_Car_Status();
        if(raspi.vision_x == 0 && raspi.vision_y == 0)
        {
            // car->target_map_Vx = 0;
            // car->target_map_Vy = 0;
            // car->target_angle_speed = 0;
            continue;
        }
        else
        {   //根据车体方向选择计算方式
            //pid计算速度
            if(car->car_face == Forward )
            {
            //pid计算目标速度
            car->target_map_Vx = PID_Calc_XY(fine_X_PID, raspi.vision_x, real_x);
            car->target_map_Vy = PID_Calc_XY(fine_Y_PID, raspi.vision_y, real_y);
           
            }
            else if(car->car_face == Back )
            {
                car->target_map_Vx = -1 * PID_Calc_XY(fine_X_PID, raspi.vision_x, real_x);
                car->target_map_Vy = -1 * PID_Calc_XY(fine_Y_PID, raspi.vision_y, real_y);
            }
            else if(car->car_face == Right)
            {
                car->target_map_Vy = -1 * PID_Calc_XY(fine_X_PID, raspi.vision_x, real_x);
                car->target_map_Vx = PID_Calc_XY(fine_Y_PID, raspi.vision_y, real_y);
            }
            else if(car->car_face == Left)
            {
                car->target_map_Vy = PID_Calc_XY(fine_Y_PID, raspi.vision_x, real_x);
                car->target_map_Vx = -1 * PID_Calc_XY(fine_X_PID, raspi.vision_y, real_y);
            }

             car->target_angle_speed = PID_Calc_Z(rough_Z_PID, target_z, omega);
            // printf("%f, %f,%f ,%f,%f ,%f\r\n", raspi.vision_x-raspi.real_x[type], raspi.vision_y-raspi.real_y[type], target_z-omega, car->current_map_Vx, car->current_map_Vy, car->current_hwt_angle_speed);
        }
        Publish_Car_Speed();
        osDelay(5);
    }
    Car_Stop(0);
    osDelay(8);
    Car_Stop(1); // 停止电机
}



//简易闭环
void Move_By_Easy(float target_x, float target_y, float target_z, uint32_t timeout)
{
    /*适合短距离移动的控制，对于较大负载也比较合适
    这里用的taeget_x和target_y军事地图坐标系下的坐标
    */

    //将电机改为速度模式
    for (int i = 1; i < 5; i++)
    {
        if (motors[i]->Control_mode != MODE_SPD)
        {
            MI_motor_setMode(motors[i], MODE_SPD);
        }
    }
    //重置车辆状态
    Reset_Car_Status();
    //更新车辆状态
    Update_Car_Status();
    //重置PID
    Reset_PID(Easy_X_PID);
    Reset_PID(Easy_Y_PID);
    Reset_PID(Easy_Z_PID);
 
    // 稳定性计数器
    uint8_t stability_counter = 0;
    uint8_t stability_counter_threshold = 5;

    // 误差阈值
    float x_error_ref = 8.0f;
    float y_error_ref = 12.0f;
    float omega_error_ref = 0.1f;
    if(fabs(target_y) < 1000)
    {
        x_error_ref = 10.0f;
        y_error_ref = 12.0f;
        omega_error_ref = 0.1f;
    }

    uint32_t start_time = HAL_GetTick();

    while(HAL_GetTick() - start_time < timeout)
    {
        if ((fabs(car->current_map_pos_x - target_x) < x_error_ref) && (fabs(car->current_map_pos_y - target_y) < y_error_ref) && (fabs(car->current_angle - target_z) < omega_error_ref))
        {
            // 使用静态计数器记录连续满足条件的次数
            stability_counter++;

            // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
            if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
            {
                Car_Stop(0);
                break;
            }
        }
        else
        {
            // 一旦不满足条件，立即重置计数器
            stability_counter = 0;
        }
        Update_Car_Status();
        //pid计算目标速度
        car->target_map_Vx = PID_Calc_XY(Easy_X_PID, target_x, car->current_map_pos_x);
        car->target_map_Vy = PID_Calc_XY(Easy_Y_PID, target_y, car->current_map_pos_y);
        car->target_angle_speed = PID_Calc_Z(Easy_Z_PID, target_z, car->current_angle);
        Publish_Car_Speed();
        // printf("%f,%f,%f,%f,%f,%f\n", target_x-car->current_map_pos_x, target_y-car->current_map_pos_y, target_z-car->current_angle, car->current_map_Vx, car->current_map_Vy, car->current_hwt_angle_speed);
    }
    // 停止电机
    Car_Stop(0);
    
}


void Move_Translation(float target_x, float target_y, float target_z, uint32_t timeout)
{
    /*用于微调移动*/
    // HAL_TIM_Base_Stop_IT(&htim5);

//*********************************************** */

        //将电机改为速度模式
    Dis_Chassis_Motor();
    Change_Chassis_Motor_Mode(MODE_SPD);
    En_Chassis_Motor();

//******************************************* */
    //将电机改为速度模式
    // for (int i = 1; i < 5; i++)
    // {
    //     if (motors[i]->Control_mode != MODE_SPD)
    //     {
    //         MI_motor_setMode(motors[i], MODE_SPD);
    //     }
    // }
    osDelay(20);

    //重置车辆状态
    Reset_Car_Status();
    //更新车辆状态
    Update_Car_Status();
    //重置PID
    Reset_PID(Translation_X_PID);
    Reset_PID(Translation_Y_PID);
    Reset_PID(Translation_Z_PID);
 
    // 稳定性计数器
    uint8_t stability_counter = 0;
    uint8_t stability_counter_threshold = 3;

    // 误差阈值
    float x_error_ref = 2.4f;
    float y_error_ref = 2.0f;
    float omega_error_ref = 0.1f;


    uint32_t start_time = HAL_GetTick();

    while(HAL_GetTick() - start_time < timeout)
    {
        if ((fabs(car->current_map_pos_x - target_x) < x_error_ref) && (fabs(car->current_map_pos_y - target_y) < y_error_ref) && (fabs(car->current_angle - target_z) < omega_error_ref))
        {
            // 使用静态计数器记录连续满足条件的次数
            stability_counter++;

            // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
            if (stability_counter >= stability_counter_threshold) // 
            {
                Car_Stop(0);
                break;
            }
        }
        else
        {
            // 一旦不满足条件，立即重置计数器
            stability_counter = 0;
        }
        Update_Car_Status();
        //pid计算目标速度
        car->target_map_Vx = PID_Calc_XY(Translation_X_PID, target_x, car->current_map_pos_x);
        car->target_map_Vy = PID_Calc_XY(Translation_Y_PID, target_y, car->current_map_pos_y);
        car->target_angle_speed = PID_Calc_Z(Translation_Z_PID, target_z, car->current_angle);
        Publish_Car_Speed();
        // printf("%f\n", omega);
        // printf("%f,%f,%f,%f,%f,%f\n", target_x-car->current_map_pos_x, target_y-car->current_map_pos_y, target_z-car->current_angle, car->current_map_Vx, car->current_map_Vy, car->current_hwt_angle_speed);
    }
    // 停止电机
    Car_Stop(0);
}

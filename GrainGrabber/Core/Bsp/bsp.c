#include "bsp.h"

void bsp_init(void)
{
    // osDelay(3000);
    Init_Chassis_Motor();
    Init_Scara_Motor();
    Init_Lifting_Motor();
    // Init_PID();  //尚未配置pid
}

void SendMultiFloat2Vofa(float *values, uint8_t num)
{
    uint8_t buffer[4 * num + 4];

    // 复制所有浮点数据到缓冲区
    for (uint8_t i = 0; i < num; i++)
    {
        memcpy(buffer + 4 * i, &values[i], 4);
    }

    // FireWater协议的帧尾
    buffer[4 * num] = 0x00;
    buffer[4 * num + 1] = 0x00;
    buffer[4 * num + 2] = 0x80;
    buffer[4 * num + 3] = 0x7f;

    // 通过UART7发送
    HAL_UART_Transmit(&huart7, buffer, 4 * num + 4, 0xFFFF);
}

//******************************************************底盘电机******************************************************

/**
  * @brief  初始化底盘电机  //默认进入运控模式
  * @retval null
  */
 void Init_Chassis_Motor(void)
 {
    motors[1] = &chassis_motor[0];
	motors[2] = &chassis_motor[1];
	motors[3] = &chassis_motor[2];
	motors[4] = &chassis_motor[3];
    for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_init(motors[id],&hcan1,id);
	}
    Reset_Chassis_Motor_MechPosition();
    //设置位置模式限速和速度模式电流限制
    for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_SetSpdLim(motors[id], 8);//位置模式限速
        MI_motor_SetCurrLim(motors[id], 23);//速度模式电流限制
	}
    Change_Chassis_Motor_Mode(MODE_SPD);
 }
/**
  * @brief  切换底盘电机模式
  * @param1  mode 模式
  * @retval null
  */
 void Change_Chassis_Motor_Mode(uint8_t mode)
 {
    for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_setMode(motors[id], mode);
	}
 }
 /**
  * @brief  设置底盘位置模式限速
  * @param1  spd_lim 限速
  * @retval null
  */
 void Set_Chassis_Motor_SpdLim(float spd_lim)
 {
    for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_SetSpdLim(motors[id], spd_lim);
	}
 }
 /**
  * @brief  设置底盘速度模式电流限制
  * @param1  current_lim 电流限制
  * @retval null
  */
 void Set_Chassis_Motor_CurrLim(float current_lim)
 {
    for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_SetCurrLim(motors[id], current_lim);
	}
 }
 /**
  * @brief  重置底盘电机机械零位
  * @retval null
  */
 void Reset_Chassis_Motor_MechPosition(void)
 {
    Dis_Chassis_Motor();
    //重置机械零位	
	MI_motor_setMechPosition2Zero(&chassis_motor[0]);
	MI_motor_setMechPosition2Zero(&chassis_motor[1]);
	MI_motor_setMechPosition2Zero(&chassis_motor[2]);
	MI_motor_setMechPosition2Zero(&chassis_motor[3]);
    En_Chassis_Motor();
 }
 /**
  * @brief  使能底盘电机
  * @retval null
  */
void En_Chassis_Motor(void)
{
    for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_enable(motors[id]);
	}
}
/**
  * @brief  失能底盘电机
  * @retval null
  */
void Dis_Chassis_Motor(void)
{
    for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_stop(motors[id]);
	}
}

/**
  * @brief  发布底盘电机速度
  * @param1  Vel1 麦轮1速度 单位：mm/s
  * @param2  Vel2 麦轮2速度 单位：mm/s
  * @param3  Vel3 麦轮3速度 单位：mm/s
  * @param4  Vel4 麦轮4速度 单位：mm/s
  * @retval null
  */
void Publish_Chassis_Motor_Speed(float Vel1, float Vel2, float Vel3, float Vel4,uint32_t delay_ms)
{
    //将轮子线速度转换为电机角速度
    Vel1 = -Vel1 / WHEEL_RADIUS ;
    Vel2 = Vel2 / WHEEL_RADIUS ;
    Vel3 = -Vel3 / WHEEL_RADIUS ;
    Vel4 = Vel4 / WHEEL_RADIUS ;
    //发布电机速度
    MI_motor_SpdCtrl(motors[1], Vel1);
    // HAL_Delay(delay_ms);
    osDelay(delay_ms);
    MI_motor_SpdCtrl(motors[2], Vel2);
    // HAL_Delay(delay_ms);
    osDelay(delay_ms);
    MI_motor_SpdCtrl(motors[3], Vel3);
    // HAL_Delay(delay_ms);
    osDelay(delay_ms);
    MI_motor_SpdCtrl(motors[4], Vel4);
    // HAL_Delay(delay_ms);
    osDelay(delay_ms);
}
void Disable_IMU_Interrupts(void)
{
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);  // USART6 RX DMA
}

void Enable_IMU_Interrupts(void)
{
    
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    // HAL_UART_Receive_DMA(&huart6, rxcmd6_dma, RXCMD6_DMA_SIZE);   //和main函数中的数组有关
    // memset(rxcmd6_dma, 0, RXCMD6_DMA_SIZE);
}

//******************************************************SCARA机械臂电机******************************************************
/**
  * @brief  初始化SCARA机械臂电机  默认进入运控模式
  * @retval null
  */
void Init_Scara_Motor(void)
{
    motors[5] = &scara_motor[0];
    motors[6] = &scara_motor[1];
    for (uint8_t id = 5; id < 7; id++)
    {
        MI_motor_init(motors[id],&hcan2,id);
    }
    // Reset_Scara_Motor_MechPosition();

    //设置位置模式限速和速度模式电流限制
    for (uint8_t id = 5; id < 7; id++)
    {
        MI_motor_SetSpdLim(motors[id], 8);//位置模式限速
        MI_motor_SetCurrLim(motors[id], 20);//速度模式电流限制
    }
    Change_Scara_Motor_Mode(MODE_POS);

}
/**
  * @brief  切换SCARA机械臂电机模式
  * @param1  mode 模式
  * @retval null
  */
void Change_Scara_Motor_Mode(uint8_t mode)
{
    for (uint8_t id = 5; id < 7; id++)
    {
        MI_motor_setMode(motors[id], mode);
    }
}
/**
  * @brief  设置SCARA机械臂电机位置模式限速
  * @param1  spd_lim 限速
  * @retval null
  */
void Set_Scara_Motor_SpdLim(uint8_t spd_lim)
{
    for (uint8_t id = 5; id < 7; id++)
    {
        MI_motor_SetSpdLim(motors[id], spd_lim);
    }
}
 /**
  * @brief  设置SCARA机械臂速度模式电流限制
  * @param1  current_lim 电流限制
  * @retval null
  */
void Set_Scara_Motor_CurrLim(uint8_t current_lim)
{
    for (uint8_t id = 5; id < 7; id++)
    {
        MI_motor_SetCurrLim(motors[id], current_lim);
    }
}
/**
  * @brief  重置SCARA机械臂电机机械零位
  * @retval null
  */
void Reset_Scara_Motor_MechPosition(void)
{
    Dis_Scara_Motor();
    //重置机械零位
    MI_motor_setMechPosition2Zero(&scara_motor[0]);
    MI_motor_setMechPosition2Zero(&scara_motor[1]);
    En_Scara_Motor();
}
/**
  * @brief  使能SCARA机械臂电机
  * @retval null
  */
void En_Scara_Motor(void)
{
    for (uint8_t id = 5; id < 7; id++)
    {
        MI_motor_enable(motors[id]);
    }
}   
/**
  * @brief  失能SCARA机械臂电机
  * @retval null
  */
void Dis_Scara_Motor(void)
{
    for (uint8_t id = 5; id < 7; id++)
    {
        MI_motor_stop(motors[id]);
    }
}   
//******************************************************升降电机******************************************************
void Init_Lifting_Motor(void)
{
    motors[7] = &lifting_motor[0];
    MI_motor_init(motors[7],&hcan2,7);
    // Reset_Lifting_Motor_MechPosition();
    //设置位置模式限速和速度模式电流限制
    MI_motor_SetSpdLim(motors[7], 12);//位置模式限速
    MI_motor_SetCurrLim(motors[7], 20);//速度模式电流限制
    Change_Lifting_Motor_Mode(MODE_POS);
}
/**
  * @brief  切换升降电机模式
  * @param1  mode 模式
  * @retval null
  */
void Change_Lifting_Motor_Mode(uint8_t mode)
{
    MI_motor_setMode(motors[7], mode);
}
/**
  * @brief  设置升降电机位置模式限速
  * @param1  spd_lim 限速
  * @retval null
  */
void Set_Lifting_Motor_SpdLim(uint8_t spd_lim)
{
    MI_motor_SetSpdLim(motors[7], spd_lim);
}
/**
  * @brief  设置升降电机速度模式电流限制
  * @param1  current_lim 电流限制
  * @retval null
  */
void Set_Lifting_Motor_CurrLim(uint8_t current_lim)
{
    MI_motor_SetCurrLim(motors[7], current_lim);
}

/**
  * @brief  重置升降电机机械零位
  * @retval null
  */
void Reset_Lifting_Motor_MechPosition(void)
{
    Dis_Lifting_Motor();
    MI_motor_setMechPosition2Zero(motors[7]);
    En_Lifting_Motor();
}
/**
  * @brief  使能升降电机
  * @retval null
  */
void En_Lifting_Motor(void)
{
    MI_motor_enable(motors[7]);
}
/**
  * @brief  失能升降电机
  * @retval null
  */
void Dis_Lifting_Motor(void)
{
    MI_motor_stop(motors[7]);
}

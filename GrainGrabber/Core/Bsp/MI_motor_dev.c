/**
 *
 * @File:        MI_motor_dev.c
 * @Author:      本人不帅
 *
 */
/* Includes -------------------------------------------------------------------*/
#include "MI_motor_dev.h"
uint8_t MI_MASTERID = 0xF4; //master id 发送指令时EXTID的bit8:15,反馈的bit0:7
//小米电机对象
MI_Motor_t chassis_motor[4];//底盘电机
MI_Motor_t scara_motor[2];//SCARA机械臂电机
MI_Motor_t lifting_motor[1];//升降电机
MI_Motor_t *motors[8];

extern CAN_HandleTypeDef hcan1;
/**
  * @brief  float转int，数据打包用
  * @param  x float数值
  * @param  x_min float数值的最小值
  * @param  x_max float数值的最大值
  * @param  bits  int的数据位数
  * @retval null
  */
int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x=x_max;
    else if(x < x_min) x= x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

unsigned char *float_to_bytes(float f, unsigned char *s)      //输入的float型数据，输出char存放地址
{
    union change
    {
        float d;
        unsigned char dat[4];
    }r1;
    
	r1.d = f;
	*s = r1.dat[0];
	*(s + 1) = r1.dat[1];
	*(s + 2) = r1.dat[2];
	*(s + 3) = r1.dat[3];
	return s;
}
		/**
  * @brief  小米电机CAN通信发送
  * @param  hmotor 电机结构体
  * @retval null
  */
CAN_TxHeaderTypeDef CAN_TxHeader_MI;
void MI_Motor_CanTx(MI_Motor_t* hmotor) {
 
    CAN_TxHeader_MI.DLC = 8;
    CAN_TxHeader_MI.IDE = CAN_ID_EXT;
    CAN_TxHeader_MI.RTR = CAN_RTR_DATA;
    CAN_TxHeader_MI.ExtId = *((uint32_t*)&(hmotor->EXT_ID));
	/*CAN_TxHeader_MI.ExtId = hmotor->EXT_ID.motor_id<<24 | hmotor->EXT_ID.data << 8 | hmotor->EXT_ID.mode << 5;*/
    uint32_t mailbox;
    uint32_t timeout = HAL_GetTick() + 100; // 100ms超时
	while(HAL_CAN_AddTxMessage(hmotor->phcan, &CAN_TxHeader_MI, hmotor->txdata, &mailbox)!= HAL_OK){
		//wait
		if(HAL_GetTick() > timeout){
      return;
    }
		// HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_RESET);
    // printf("CAN TX failed\r\n");
	}
	// HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_SET);
}
/**
  * @brief  小米电机初始化（实际上是motors数组赋值，给motor数组每一个元素赋予了id）
  * @param  hmotor 电机结构体
  * @param  phcan can总线句柄
  * @param  id 电机id
  * @retval null
  */
void MI_motor_init(MI_Motor_t* hmotor,CAN_HandleTypeDef *phcan,uint8_t id)
{
    hmotor->phcan = phcan;
    hmotor->EXT_ID.motor_id = id;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
    hmotor->first_run = true;
    hmotor->Control_mode = MODE_CTRL;
}
/**
  * @brief  小米电机使能
  * @param  hmotor 电机结构体
  * @param  id 电机id
  * @retval null
  */
void MI_motor_enable(MI_Motor_t* hmotor)
{
    hmotor->EXT_ID.mode = 3;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
	// HAL_Delay(10);
  osDelay(10);
}
/**
  * @brief  获取设备ID （通信类型0），需在电机使能前使用
  * @param  hmotor 电机结构体
  * @retval null
  */
void MI_motor_get_ID(MI_Motor_t* hmotor)   //warning：没有传入主机id和从机id，谨慎使用
{
    hmotor->EXT_ID.mode = 0;
    hmotor->EXT_ID.data = 0;
    hmotor->EXT_ID.motor_id = 0;
    hmotor->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
}

/**
  * @brief  通过设置0x7005位参数改变控制模式（通信类型18），范例代码
  * @param  mode:电机模式，详细配置见MI_motor_dev.h
  * @retval null
  */
 void MI_motor_setMode(MI_Motor_t* hmotor, uint8_t mode)
{
		uint16_t index = 0x7005;
		hmotor->EXT_ID.mode = 0x12;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
		
		for (int i = 0; i < 8; i++)
				hmotor->txdata[i] = 0;
	
    memcpy(&hmotor->txdata[0],&index,2);
    memcpy(&hmotor->txdata[4],&mode, 1);
    MI_Motor_CanTx(hmotor);
    hmotor->Control_mode = mode;
		// HAL_Delay(10);
    osDelay(10);
}

/**
  * @brief  运控模式电机控制指令（通信类型1）
  * @param  hmotor 电机结构体
  * @param  motor_id 电机id
  * @param  kp	default = 30
  * @param  kd	default = 2
  * @param  torque：扭矩
  * @param  MechPosition：目标角度
  * @param  speed：目标速度
  * @retval null
  */
void MI_motor_controlmode(MI_Motor_t* hmotor, float torque, float MechPosition , float speed , float kp , float kd)
{
	if (MechPosition<-1.053 || MechPosition>1.5)     //warning：限制角度范围,但数据无来源，谨慎使用
		return;
    hmotor->EXT_ID.mode = 1;
    hmotor->EXT_ID.data = float_to_uint(torque,T_MIN,T_MAX,16);
    hmotor->EXT_ID.res = 0;
 
//	MechPosition *= PI/180;
    hmotor->txdata[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
    hmotor->txdata[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
    hmotor->txdata[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
    hmotor->txdata[3]=float_to_uint(speed,V_MIN,V_MAX,16);
    hmotor->txdata[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    hmotor->txdata[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
    hmotor->txdata[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    hmotor->txdata[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);
    MI_Motor_CanTx(hmotor);
	//HAL_Delay(10);
}
/**
  * @brief  电机停止运行帧（通信类型4）
  * @param  hmotor 电机结构体
  * @retval null
  */
void MI_motor_stop(MI_Motor_t* hmotor)
{
    hmotor->EXT_ID.mode = 4;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;    //正常运行时需要置零
    }
    MI_Motor_CanTx(hmotor);
	// HAL_Delay(10);
  osDelay(10);
}
/**
  * @brief  设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
  * @param  hmotor 电机结构体
  * @retval null
  */
void MI_motor_setMechPosition2Zero(MI_Motor_t* hmotor)
{
    hmotor->EXT_ID.mode = 6;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
    hmotor->txdata[0]=1;
 
    for(uint8_t i=1; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
	// HAL_Delay(10);
  osDelay(10);
}
/**
  * @brief  设置电机CAN_ID（通信类型7）更改当前电机CAN_ID , 立即生效，需在电机使能前使用
  * @param  hmotor 电机结构体
  * @param  Now_ID 电机现在的ID
  * @param  Target_ID 想要改成的电机ID
  * @retval null
  */
void MI_motor_changeID(MI_Motor_t* hmotor,uint8_t Now_ID,uint8_t Target_ID)
{
    hmotor->EXT_ID.mode = 7;	
    hmotor->EXT_ID.motor_id = Now_ID;
    hmotor->EXT_ID.data = Target_ID << 8 | MI_MASTERID;
    hmotor->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  单个参数写入（通信类型18） （掉电丢失）
  * @param  hmotor 电机结构体
  * @param  index 功能码
  * @param  data[4] 参数数据缓冲
  * @param  数据区0-1写参数列表参数index，2-3写00,4-7写参数数据
  * @retval null
  */
void MI_motor_Write_One_Para(MI_Motor_t* hmotor, uint16_t index ,uint8_t data[4])
{
    hmotor->EXT_ID.mode = 0x12;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
 
    memcpy(&hmotor->txdata[0],&index,2);
    memcpy(&hmotor->txdata[4],data, 4);
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  单个参数读取（通信类型17）
  * @param  hmotor 电机结构体
  * @param  数据区0-1  ：index 功能码
  * @retval null
  * @note   
  */
 //warning:未知参数如何读取
void MI_motor_Read_One_Para(MI_Motor_t* hmotor,uint16_t index)
{
    hmotor->EXT_ID.mode = 0x11;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
    memcpy(&hmotor->txdata[0],&index,2);
    for(uint8_t i=2; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  读取电机mechpos控制指令（通信类型17）
  * @param  hmotor 电机结构体
  * @param  motor_id 电机id
  * @param  mechpos：负载端计圈机械角度
  * @retval null
  */
void MI_motor_get_mechPos(MI_Motor_t* hmotor)
{
    MI_motor_Read_One_Para(hmotor, 0x7019);
    // HAL_Delay(1);
}
/**
  * @brief  读取电机mechvel控制指令（通信类型17）-30rad/s~30rad/s
  * @param  hmotor 电机结构体
  * @param  mechvel：负载端转速
  * @retval null
  */
void MI_motor_get_speed(MI_Motor_t* hmotor)
{
    MI_motor_Read_One_Para(hmotor, 0x701B);
    // HAL_Delay(1);
}
/**
  * @brief  位置控制
  * @param  hmotor 电机结构体
  * @param  pos_ref 期望位置
  * @retval null
  */
void MI_motor_PosCtrl(MI_Motor_t* hmotor, float pos_ref)
{
	uint8_t pos_data[4];
	float_to_bytes(pos_ref, pos_data);
	MI_motor_Write_One_Para(hmotor, 0x7016, pos_data);
  // HAL_Delay(1);
}

/**
  * @brief  设置位置模式限速
  * @param  hmotor 电机结构体
  * @param  spd_lim 速度限制
  * @retval null
  */
void MI_motor_SetSpdLim(MI_Motor_t* hmotor, float spd_lim)
{
	uint8_t data[4];
	float_to_bytes(spd_lim, data);
	MI_motor_Write_One_Para(hmotor, 0x7017, data);
	// HAL_Delay(10);
  osDelay(10);
}
/**
  * @brief  速度控制
  * @param  hmotor 电机结构体
  * @param  speed_ref 期望速度限制在 -30~30rad/s
  * @retval null
  */
void MI_motor_SpdCtrl(MI_Motor_t* hmotor, float speed_ref)
{
	uint8_t data[4];
  if(speed_ref > 30.0f) speed_ref = 30.0f;
  else if(speed_ref < -30.0f) speed_ref = -30.0f;
	float_to_bytes(speed_ref, data);
	MI_motor_Write_One_Para(hmotor, 0x700A, data);
  // HAL_Delay(1);
}
/**
  * @brief  设置速度模式电流限制
  * @param  hmotor 电机结构体
  * @param  current_lim 电流限制 0-23A
  * @retval null
  */
void MI_motor_SetCurrLim(MI_Motor_t* hmotor, float current_lim)
{
	uint8_t data[4];
	float_to_bytes(current_lim, data);
	MI_motor_Write_One_Para(hmotor, 0x7018, data);
  // HAL_Delay(10);
  osDelay(10);
}


/**
  * @brief  单电机单个参数解码
  * @param  hmotor 电机结构体
  * @param  rxdata 数据缓冲区
  * @retval null
  */
void MI_motor_onePara_decode(MI_Motor_t* hmotor,uint8_t rxdata[]) 
{
    //mechpos获取
    if ((rxdata[0] | rxdata[1] << 8) == 0x7019) {
        // 将4个字节组合成32位整数（小端序）
        uint32_t temp = (rxdata[7] << 24) | (rxdata[6] << 16) | (rxdata[5] << 8) | rxdata[4];
        // 将32位整数重新解释为IEEE 754浮点数
        float mechPos_float;
        *((uint32_t*)&mechPos_float) = temp;  
        //修正为标准麦轮解算
        if(hmotor->EXT_ID.motor_id == 1||hmotor->EXT_ID.motor_id == 3)
        {
            mechPos_float = -mechPos_float;
        }
        hmotor->motor_fdb.mechPos = mechPos_float;  
        hmotor->distance_traveled = mechPos_float*WHEEL_RADIUS;//将角度转换为距离，单位：mm
        //一次新的运动指令重新开始计步
        if(hmotor->first_run == true)
        {
            hmotor->first_run = false;
            hmotor->last_distance_traveled = hmotor->distance_traveled   ;
        }
        //判断是否还在速度规划中
        // if(hmotor->last_update_time - HAL_GetTick() > 1000)
        // {
        //     hmotor->last_distance_traveled = 0;
        //     printf("1\r\n");
        // }
        //计算距离差
        hmotor->delta_distance_traveled = hmotor->distance_traveled - hmotor->last_distance_traveled;
        hmotor->last_distance_traveled = hmotor->distance_traveled;
        //更新时间戳
        hmotor->last_update_time = HAL_GetTick();

    }
    //mechvel获取
    else if ((rxdata[0] | rxdata[1] << 8) == 0x701B) {
        uint32_t temp = (rxdata[7] << 24) | (rxdata[6] << 16) | (rxdata[5] << 8) | rxdata[4];
        float mechVel_float;
        *((uint32_t*)&mechVel_float) = temp;

        //修正为标准麦轮解算
        if(hmotor->EXT_ID.motor_id == 1||hmotor->EXT_ID.motor_id == 3)
        {
            mechVel_float = -mechVel_float;
        }
        hmotor->motor_fdb.speed = mechVel_float;
        hmotor->current_speed = mechVel_float*WHEEL_RADIUS;//将角速度转换为线速度，单位：mm/s
    }
}

/**
  * @brief  单电机应答反馈解码
  * @param  hmotor 电机结构体
  * @param  state_byte状态字节，扩展ID的bit8:23
  * @param  rxdata 数据缓冲区
  * @retval null
  */

void MI_motor_decode(MI_Motor_t* hmotor,uint8_t state_byte,uint8_t rxdata[]) 
{
    int16_t decode_temp_mi = 0;
    // uint8_t nsvd = state_byte;
    // nsvd = state_byte;   //后面未曾使用
    if((state_byte&0xC0) == 0) {
        hmotor->motor_state = OK;
    } else {
        for(int i = 1; i < 7; i++) {
            if(state_byte&0x01) {
                hmotor->motor_state = i;
            }
            state_byte = state_byte>> 1;
        }
    }
    hmotor->motor_mode = state_byte;
 
    decode_temp_mi = (rxdata[0] << 8 | rxdata[1])^0x8000;
    hmotor->motor_fdb.angle_temp   = decode_temp_mi;
 
    decode_temp_mi = (rxdata[2] << 8 | rxdata[3])^0x8000;
    //修正为标准麦轮解算
    if(hmotor->EXT_ID.motor_id == 1||hmotor->EXT_ID.motor_id == 3)
    {
        decode_temp_mi = -decode_temp_mi;
    }
    hmotor->motor_fdb.speed_temp   = decode_temp_mi;
 
    decode_temp_mi = (rxdata[4] << 8 | rxdata[5])^0x8000;
    hmotor->motor_fdb.torque_temp   = decode_temp_mi;
 
    decode_temp_mi = (rxdata[6] << 8 | rxdata[7]);
    hmotor->motor_fdb.temprature_temp  = decode_temp_mi;
 
	  hmotor->motor_fdb.last_angle = hmotor->motor_fdb.angle;
    hmotor->motor_fdb.angle = (float)hmotor->motor_fdb.angle_temp/32768.0*4*PI;
    hmotor->motor_fdb.speed = (float)hmotor->motor_fdb.speed_temp/32768.0*30;
    hmotor->current_speed = hmotor->motor_fdb.speed*WHEEL_RADIUS;//将角速度转换为线速度，单位：mm/s
    hmotor->motor_fdb.torque = (float)hmotor->motor_fdb.torque_temp/32768.0*12.0f;
    hmotor->motor_fdb.temprature = (float)hmotor->motor_fdb.temprature_temp/10.0f;

}
/**
  * @brief  小米电机解码
  * @note   调用主函数HAL_CAN_RxFifo0MsgPendingCallback收到can消息
  * @param  rx_EXT_id 接收到的扩展ID
  * @param  rxdata 数据缓冲区
  * @retval null
  */
EXT_ID_t EXT_ID_tmp;//扩展ID数据结构体
void MIMotor_MotorDataDecode(uint32_t rx_EXT_id,uint8_t rxdata[])
{   
	  EXT_ID_tmp = *((EXT_ID_t*)(&rx_EXT_id));
    // printf("mode: %d\r\n", EXT_ID_tmp.mode);
    // printf("rxdata[0]: %d\r\n", rxdata[0]);
    // printf("rxdata[1]: %d\r\n", rxdata[1]); 
    // printf("rxdata[2]: %d\r\n", rxdata[2]);
    // printf("rxdata[3]: %d\r\n", rxdata[3]);
    // printf("rxdata[4]: %d\r\n", rxdata[4]);
    // printf("rxdata[5]: %d\r\n", rxdata[5]);
    // printf("rxdata[6]: %d\r\n", rxdata[6]);
    // printf("rxdata[7]: %d\r\n", rxdata[7]);
    //获取电机ID
    if(EXT_ID_tmp.mode == 0&&EXT_ID_tmp.motor_id == 0xFE) {
        uint8_t id = EXT_ID_tmp.data&0xFF;
        if(id > 0 && id < 8) {
            memcpy(motors[id]->motor_fdb.MI_MCU_identifier,rxdata, 8);
        }
    }
    //电机状态反馈帧
    else if(EXT_ID_tmp.mode == 2) {
        uint8_t id = EXT_ID_tmp.data&0xFF;
        if(id > 0 && id < 8) {
            MI_motor_decode(motors[id],(uint8_t)(EXT_ID_tmp.data>>8),rxdata);
        }
    }
    //电机单个参数读取
    else if(EXT_ID_tmp.mode == 0x11) { 
        uint8_t id = EXT_ID_tmp.data&0xFF;
        if(id > 0 && id < 8) {
            MI_motor_onePara_decode(motors[id],rxdata);
        }	
    }
}



/**
 * @brief 启动所有电机并进行初始化配置
 * @note 该函数用于初始化并启动4个底盘电机，设置其工作模式、速度和电流限制
 */
void Motors_Start(void)
{
    // 将电机指针数组指向对应的底盘电机对象
	motors[1] = &chassis_motor[0];
	motors[2] = &chassis_motor[1];
	motors[3] = &chassis_motor[2];
	motors[4] = &chassis_motor[3];

	
    // 初始化所有电机（ID从1到4）
	for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_init(motors[id],&hcan1,id);  // 通过CAN总线初始化电机
	}
  // HAL_Delay(50);
  osDelay(50);
	for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_stop(motors[id]);
	}	
  // HAL_Delay(50);
  osDelay(50);

	
	MI_motor_setMechPosition2Zero(&chassis_motor[0]);
	MI_motor_setMechPosition2Zero(&chassis_motor[1]);
	MI_motor_setMechPosition2Zero(&chassis_motor[2]);
	MI_motor_setMechPosition2Zero(&chassis_motor[3]);
  	for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_enable(motors[id]);
	}	

	for (uint8_t id = 1; id < 5; id++)
	{
		MI_motor_setMode(motors[id], MODE_POS);
		MI_motor_SetSpdLim(motors[id], 15);
    // MI_motor_PosCtrl(motors[id], 3.14);
    // MI_motor_setMode(motors[id], MODE_SPD);
    MI_motor_SetCurrLim(motors[id], 10);
		// MI_motor_SpdCtrl(motors[id], 1);
	}
}

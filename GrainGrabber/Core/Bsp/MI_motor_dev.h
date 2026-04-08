#ifndef MI_MOTOR_DEV_H
#define MI_MOTOR_DEV_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_can.h"
#include "usart.h"
#include "can.h"
#include "cmsis_os.h"
#include "main.h"


#define PI 3.14159265358979323846f
#define WHEEL_RADIUS 76.0f // 轮子半径 单位：mm
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define MODE_CTRL 0x00
#define MODE_POS 0x01
#define MODE_SPD 0x02
#define MODE_CURR 0x03
typedef enum
{
	OK                 = 0,//无故障
	BAT_LOW_ERR        = 1,//欠压故障
	OVER_CURRENT_ERR   = 2,//过流
	OVER_TEMP_ERR      = 3,//过温
	MAGNETIC_ERR       = 4,//磁编码故障
	HALL_ERR_ERR       = 5,//HALL编码故障
	NO_CALIBRATION_ERR = 6//未标定
}motor_state_e;//电机状态（故障信息）
typedef enum
{
	RESET_MODE = 0,//Reset[模式]
	CALI_MODE  = 1,//Cali 模式[标定]
	RUN_MODE   = 2//Motor模式[运行]
} motor_mode_e;//电机运行模式
typedef struct
{
	uint32_t motor_id : 8; // 只占8位
	uint32_t data : 16;
	uint32_t mode : 5;
	uint32_t res : 3;  //保留位
} EXT_ID_t; // 32位扩展ID解析结构体
typedef struct
{	//64位MCU唯一标识符
	uint8_t MI_MCU_identifier[8];
	// 默认电机应答反馈
	volatile int16_t angle_temp;
	volatile int16_t speed_temp;
	volatile int16_t torque_temp;
	volatile int16_t temprature_temp;
	volatile float angle; //范围：-12.5~12.5，rad
	volatile float last_angle;//上次反馈角度
	volatile float speed; //范围：-30~30，rad/s
	volatile float torque;//范围：-12~12，Nm
	volatile float temprature;//范围：0~100，℃
	// 电机单个参数读取
	volatile float mechPos;//输出端绝对角度
	// 电机单个参数读取
} Motor_fdb_t;                 // 电机编码器反馈结构体
typedef struct
{
	CAN_HandleTypeDef *phcan;//can句柄
	motor_state_e motor_state;//电机状态
	motor_mode_e  motor_mode;//电机模式
	uint8_t Control_mode;//控制模式
	EXT_ID_t EXT_ID;//扩展ID
	uint8_t txdata[8];//发送数据
	Motor_fdb_t motor_fdb;//电机反馈
	//控制算法所需变量
	uint32_t last_update_time;//上次读取绝对位置时间戳
	float distance_traveled;//电机行驶距离 单位:mm
	float last_distance_traveled;//上次读取的行驶距离 单位:mm
	float last_last_distance_traveled;//上上次读取的行驶距离 单位:mm
	float delta_distance_traveled;//当前行驶距离与上次行驶距离的差值 单位:mm
	float current_speed;//当前速度 单位:mm/s
	bool first_run;
} MI_Motor_t;
/**********************Functions**************************/
void MI_motor_get_ID(MI_Motor_t* hmotor);
void MI_motor_init(MI_Motor_t* hmotor,CAN_HandleTypeDef *phcan,uint8_t id);
void MI_motor_enable(MI_Motor_t *hmotor);
void MI_motor_setMode(MI_Motor_t* hmotor, uint8_t mode);
void MI_motor_controlmode(MI_Motor_t* hmotor, float torque, float MechPosition , float speed , float kp , float kd);
void MI_motor_stop(MI_Motor_t *hmotor);
void MI_motor_setMechPosition2Zero(MI_Motor_t *hmotor);
void MI_motor_changeID(MI_Motor_t* hmotor,uint8_t Now_ID,uint8_t Target_ID);
void MIMotor_MotorDataDecode(uint32_t rx_EXT_id, uint8_t rxdata[]);
void MI_motor_PosCtrl(MI_Motor_t* hmotor, float pos_ref);
void MI_motor_SetSpdLim(MI_Motor_t* hmotor, float spd_lim);
void MI_motor_Write_One_Para(MI_Motor_t* hmotor, uint16_t index ,uint8_t data[4]);
void MI_motor_Read_One_Para(MI_Motor_t* hmotor,uint16_t index);
void MI_motor_get_mechPos(MI_Motor_t* hmotor);
void MI_motor_get_speed(MI_Motor_t* hmotor);
void MI_motor_SpdCtrl(MI_Motor_t* hmotor, float speed_ref);
void MI_motor_SetCurrLim(MI_Motor_t* hmotor, float current_lim);

void Motors_Start(void);

extern MI_Motor_t chassis_motor[4];
extern MI_Motor_t scara_motor[2];
extern MI_Motor_t lifting_motor[1];
extern MI_Motor_t *motors[8];
#endif
 
// #endif
#ifdef __cplusplus
}
#endif

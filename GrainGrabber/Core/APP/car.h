#ifndef CAR_H
#define CAR_H
#include "bsp.h"
#include "math.h"
#include "can.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_can.h"
#include "usart.h"
#include "raspi.h"
#include "app.h"
#include "main.h"
//**********************************结构体声明*******************************************//
//车辆模式
enum Car_Mode
{
    CAR_ACC = 0,//加速过程
    CAR_AVE =1 ,//匀速过程
    CAR_DEC = 2,//减速过程
    CAR_RUNNING = 3,//运行中
    CAR_STOP = 4,//停止
    CAR_ERROR = 5//错误

};
//车辆朝向
enum Car_Face
{
  Forward = 0 ,
  Left = 1 ,
  Back = 2 ,
  Right = 3
};
  // 车辆状态
typedef struct {
    //
    float target_Vel1 ;
    float target_Vel2 ;
    float target_Vel3 ;
    float target_Vel4 ;
  
    float current_Vel1 ;//电机1当前速度,单位mm/s
    float current_Vel2 ;//电机2当前速度,单位mm/s
    float current_Vel3 ;//电机3当前速度,单位mm/s
    float current_Vel4 ;//电机4当前速度,单位mm/s
 
    //相对车身坐标系
    float current_car_Vx ;  // x方向速度(相对于车身自身坐标系)
    float current_car_Vy ;  // y方向速度(相对于车身自身坐标系)
    float target_car_Vx ;
    float target_car_Vy ;
    
    float delta_car_pos_x  ; //车每次相对自身x方向移动量
    float delta_car_pos_y  ; //车每次相对自身y方向移动量
  
    float target_car_pos_x  ;  
    float target_car_pos_y  ; 
  
    float current_map_Vx ;  //x方向速度(相对于全局坐标系)
    float current_map_Vy ;  //y方向速度(相对于全局坐标系)
    float target_map_Vx ;
    float target_map_Vy ;
  
    float  current_map_pos_x ; //x方向位置(相对于全局坐标系)
    float  current_map_pos_y ; //y方向位置(相对于全局坐标系)
    float  target_map_pos_x ; 
    float  target_map_pos_y ; 

    float current_angle ;//当前角度，逆时针为正，单位°
    float current_angle_vel ;//当前角速度，逆时针为正，单位rad/s(通过电机反馈计算得出)
    float current_hwt_angle_speed ;//当前陀螺仪角速度，逆时针为正，单位°/s
    float target_angle_speed ;//目标角速度，逆时针为正，单位°/s
    enum Car_Mode car_mode ;//车辆模式
    enum Car_Face car_face ;//车辆朝向
  
  } Car_Status;


//**********************************全局变量声明*******************************************//
extern Car_Status *car;



//**********************************函数声明*******************************************//
void omega_zero(void);
void auto_offset_omega(void);
void Hwt_auto_get_offset_ON(void);
void Hwt_auto_get_offset_OFF(void);
void Init_Imu(void);
void Init_Car(void);
void Reset_Car_Status(void);
void Update_Car_Status(void);
void From_Motor_to_Car_Status(void);
void Publish_Car_Speed(void);
void Car_Stop(bool mode);
void Move_TransformX(float X  ,float Vx);
void Move_TransformY(float Y ,float Vy);
void Move_TransformXY(float XY, float Vxy);
void Move_At_Angle(float angle, float distance, float speed);
void Move_TransformZ(float angle, float Vz);
void Move_To_Position_XYZ(float target_x, float target_y, float target_z, uint32_t timeout);
void Move_By_Vision(uint8_t paper_id, uint32_t timeout);
void Move_By_Easy(float target_x, float target_y, float target_z, uint32_t timeout);
void Move_Translation(float target_x, float target_y, float target_z, uint32_t timeout);
#endif


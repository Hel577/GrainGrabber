#ifndef APP_H
#define APP_H

#include "car.h"
#include "lift.h"
#include "scara.h"
#include "bsp.h"
#include "raspi.h"
#include "main.h"
#include "cmsis_os.h"  // 添加CMSIS-RTOS头文件

// 底盘移动类型枚举
typedef enum {
    MOVE_TYPE_POSITION_XYZ,    // 闭环位置控制
    MOVE_TYPE_VISION,          // 视觉闭环
    MOVE_TYPE_EASY,            // 简易闭环
    MOVE_TYPE_TRANSLATION      // 平移控制
} ChassisMove_Type_e;

// 底盘状态结构体
typedef struct {
    float target_x;
    float target_y;
    float target_z;
    uint32_t timeout;
    uint8_t paper_id;          // 用于视觉闭环
    ChassisMove_Type_e move_type;
    bool moving;
    bool done;
} ChassisState_t;

// 机械臂状态结构体
typedef struct {
    uint8_t box_id;
    uint8_t paper_dir;
    bool maduo;
    uint8_t action_type;  // 0=startscara ,1=Get_Box,2 =ready_to_put_box, 3=Put_Box
    bool moving;
    bool done;
} ScaraState_t;

// 声明外部变量
extern ChassisState_t chassisState;
extern ScaraState_t scaraState;
extern osSemaphoreId_t ChassisMoveDoneHandle;
extern osSemaphoreId_t ScaraMoveDoneHandle;
extern bool car_box[7]; // 用于判断车仓是否有箱子（1为有，0为无）
extern uint8_t paper_box[7]; // 用于判断纸箱是否有箱子（2为有两层，1为有一层，0为无）
extern uint8_t put_round;
void app_init(void);
void Start_Scara(void);
void Get_Box(uint8_t box_id);
void Put_Box(uint8_t box_id ,uint8_t dir, bool maduo);
void Ready_To_Put_Box(uint8_t box_id);
void Move_To_Target(uint8_t target_id);
void Move_To_Placing_Box(uint8_t* box_ids);

// 添加非阻塞版本的函数
void Start_Scara_NonBlocking(void);
void Get_Box_NonBlocking(uint8_t box_id);
void Ready_To_Put_Box_NonBlocking(uint8_t box_id);
void Put_Box_NonBlocking(uint8_t box_id, uint8_t dir, bool maduo);
void Move_To_Position_XYZ_NonBlocking(float target_x, float target_y, float target_z, uint32_t timeout);
void Move_By_Vision_NonBlocking(uint8_t paper_id, uint32_t timeout);
void Move_By_Easy_NonBlocking(float target_x, float target_y, float target_z, uint32_t timeout);
void Move_Translation_NonBlocking(float target_x, float target_y, float target_z, uint32_t timeout);
uint8_t Near_Box(uint8_t box_id ,uint8_t dir, bool maduo);


#endif

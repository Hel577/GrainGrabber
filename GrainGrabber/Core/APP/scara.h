#ifndef SCARA_H
#define SCARA_H
#ifdef __cplusplus
extern "C"
{
#endif


#include "bsp.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

/* SCARA参数定义 */
#define MOTOR_DISTANCE 200.0f        // 两电机间距(mm)
#define SMALL_ARM_LENGTH 260.0f      // 小臂长度(mm)
#define LARGE_ARM_LENGTH 160.0f       // 大臂长度(mm)
#define OFFSET_X 0.0f                 // 机械臂零位末端位置
#define OFFSET_Y 0.0f                 // 机械臂零位末端位置
#define RADIUS 210.0f                   // 末端圆盘半径(mm)

/* 末端爪子参数定义 */
#define GRAB_OPEN    3000  //爪子完全张开
#define GRAB_ClOSE_All    2048  //爪子完全闭合
#define GRAB_BOX 2173  //抓紧箱子
// 末端爪子结构体
typedef struct {
    float current_angle;     // 末端当前保持角度（相对于x轴）
    float target_angle;      // 末端目标保持角度
    float step_angle;        // 旋转舵机的增量控制步进值
    bool grab_state;      // 爪子开合状态：0-松开，1-抓住
} hand_t;


// 升降系统结构体
typedef struct {    
    MI_Motor_t *L_motor;     // 左臂电机
    MI_Motor_t *R_motor; 
    float current_th1;       // 左边电机实际物理模型的角度(0-360°)
    float current_th2;       // 右边电机实际物理模型的角度(0-360°)
    float target_th1;        // 左侧电机目标角度
    float target_th2;        // 右侧电机目标角度
    float step_angle; 
    bool is_moving;  // 电机增量控制步进值（度）
} SCARA_t;

extern SCARA_t scara;
extern hand_t hand;



/* 函数声明 */
void Init_Scara(void);
void detect_workspace(void);
void Update_Scara_Status(void);
void set_scara_position(float angle1, float angle2);
void add_scara_ctrl(void);
void Scara_PosCtrl(float angle1, float angle2);
void Scara_Return_Home(void);
void Control_End_Rotation(float angle);
void Maintain_End_Rotation(void);
void End_Rotation_Ctrl(float maintain_angle);
void Grab_On(void);
void Grab_Off(void);
void Calculate_Angles_From_Height(float height, float *angle1, float *angle2);
void Scara_To_Height(float height);


#endif
// #endif
#ifdef __cplusplus
}
#endif

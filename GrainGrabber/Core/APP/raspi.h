#ifndef RASPI_H
#define RASPI_H

/* 包含必要的头文件 */
#include "bsp.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

/* 宏定义 */
#define RASPI_BUFFER_SIZE 8

#define TRUE_X 580
#define TRUE_Y 600//apriltag的理想坐标


/* 命令类型定义(From Raspi)*/
#define CMD_MOVE    0x01
#define CMD_GRAB    0x02
#define CMD_PLACE   0x03
#define CMD_LOCATE  0x04

#define CAR_TO_CAMERA 325.0f//车中心到摄像头的距离，单位是mm，在x轴方向上
/* 任务类型定义(Send to Raspi) */  //send to raspi
#define TASK_DETECT_BOX    0x05//识别箱子任务
#define TASK_MOVE_BY_BEAN     0x07//根据豆子位置微调任务
#define TASK_MOVE_BY_BOX   0x08//根据箱子旁边的AprilTag进行位置微调任务
#define TASK_MOVE_BY_APRILTAG   0x09//根据AprilTag位置微调任务


/*树莓派数据类型*/   //receive from raspi
#define PLAN_MOVE_BY_BEAN 0x10      // 根据豆子位置微调计划
#define PLAN_MOVE_BY_BOX 0x11      // 根据箱子位置微调计划
#define PLAN_BOX_ID 0x12    // 货箱ID计划
#define PLAN_APRILTAG_ID 0x13    // apriltag调整

#define RASPI_DETECT_OK 0x14 //树莓派检测完成标志

#define GREEN_BEAN 1
#define YELLOW_BEAN 2
#define WHITE_BEAN 3

/* 数据结构定义 */
typedef struct {
    uint8_t buffer[RASPI_BUFFER_SIZE];//接收到树莓派的数据帧
    volatile uint8_t cmd;//树莓派命令类型
    volatile float vision_x;//视觉检测坐标(横向，0-640)    使用后要清零
    volatile float vision_y;//视觉检测坐标(纵向，0-480)
    float real_x[10];//实际中心坐标(横向，0-640)
    float real_y[10];//实际中心坐标(纵向，0-480)
    volatile float true_x;
    volatile float true_y;//检测到的apriltag的相对位置（单位0.1mm)
    uint8_t box_id[5];//顺序为从上到下的盒子的id
    uint8_t bean_order[3];//绿豆=1，黄豆=2，白芸豆=3
} Raspi_Date;

/* 函数声明 */
void Init_Raspi(void);
void Raspi_Send_Task(uint8_t task_type);
void Raspi_Finish_Task(uint8_t task_type);
void Raspi_Process_Data(uint8_t *rx_data, uint16_t size);
void Start_Raspi_Detect(void);

/* 全局变量声明 */
extern Raspi_Date raspi;
extern volatile uint8_t raspi_detect_ok;
extern uint8_t recognize; // 用于存储识别到的豆子类型，1=绿豆，2=黄豆，3=白芸豆

#endif /* __RASPI_H */

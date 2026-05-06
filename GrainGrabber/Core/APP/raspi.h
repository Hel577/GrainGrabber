#ifndef RASPI_H
#define RASPI_H

/* 包含必要的头文件 */
#include "bsp.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

/* 宏定义 */
#define RASPI_BUFFER_SIZE 9


/* 命令类型定义(From Raspi)*/
#define CMD_MOVE    0x01
#define CMD_GRAB    0x02
#define CMD_PLACE   0x03
#define CMD_LOCATE  0x04

/* 任务类型定义(Send to Raspi) */
#define TASK_DETECT_BOX    0x05
#define TASK_DETECT_PAPER  0x06
#define TASK_LOCATE_BOX    0x07
#define TASK_LOCATE_PAPER  0x08
#define TASK_PAPER       0x09


/*树莓派数据类型*/
#define PLAN_MOVE 0x10      // 移动区域计划
#define PLAN_BOX_ID 0x11    // 货箱ID计划
#define PLAN_SIDE 0x12      // 放置左中右侧边计划
#define PLAN_HEIGHT 0x13    // 放置高度计划

/* 数据结构定义 */
typedef struct {
    uint8_t buffer[RASPI_BUFFER_SIZE];//接收到树莓派的数据帧
    volatile uint8_t cmd;//树莓派命令类型
    volatile float vision_x;//视觉检测坐标(横向，0-640)    使用后要清零
    volatile float vision_y;//视觉检测坐标(纵向，0-480)
    float real_x[3];//实际中心坐标(横向，0-640)
    float real_y[3];//实际中心坐标(纵向，0-480)
    uint8_t paper_id[6];//按顺序要移动到的纸垛区域   
    uint8_t box_id[6];//按顺序要放的储存槽id     
    uint8_t box_dir[6];//按顺序要放的箱子方向   0 1 2 分别表示左中右
    bool maduo[6];//按顺序要放的箱子是否码垛   0为不码垛 1为码垛

} Raspi_Date;

/* 函数声明 */
void Init_Raspi(void);
void Raspi_Send_Task(uint8_t task_type, uint8_t data);
void Raspi_Process_Data(uint8_t *rx_data, uint16_t size);
void Start_Raspi_Detect(void);

/* 全局变量声明 */
extern Raspi_Date raspi;
extern volatile uint8_t raspi_detect_ok;

#endif /* __RASPI_H */

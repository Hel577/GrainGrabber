#ifndef __MY_TASK_H__
#define __MY_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 包含所需的头文件 */
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "app.h"
#include "bsp.h"
#include "door.h"
#include "scara.h"


/* 函数声明 */
void Init_All(void);
void my_task(void);
void test_move(void);
void test_motor(void);
void test_chassis(void);
void test_lift(void);
void test_Push(void);
void test_Grab(void);
void test_Graber(void);
void test_door(void);

#ifdef __cplusplus
}
#endif

#endif /* __MY_TASK_H__ */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "my_task.h"
#include "app.h"
#include "bsp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 全局状态变量定义
ChassisState_t chassisState = {0};
ScaraState_t scaraState = {0};
/* USER CODE END Variables */
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ScaraTask */
osThreadId_t ScaraTaskHandle;
const osThreadAttr_t ScaraTask_attributes = {
  .name = "ScaraTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ChassisMoveDone */
osSemaphoreId_t ChassisMoveDoneHandle;
const osSemaphoreAttr_t ChassisMoveDone_attributes = {
  .name = "ChassisMoveDone"
};
/* Definitions for ScaraMoveDone */
osSemaphoreId_t ScaraMoveDoneHandle;
const osSemaphoreAttr_t ScaraMoveDone_attributes = {
  .name = "ScaraMoveDone"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartChassisTask(void *argument);
void StartScaraTask(void *argument);
void StartMainTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ChassisMoveDone */
  ChassisMoveDoneHandle = osSemaphoreNew(1, 1, &ChassisMoveDone_attributes);

  /* creation of ScaraMoveDone */
  ScaraMoveDoneHandle = osSemaphoreNew(1, 1, &ScaraMoveDone_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

  /* creation of ScaraTask */
  ScaraTaskHandle = osThreadNew(StartScaraTask, NULL, &ScaraTask_attributes);

  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartChassisTask */
/**
  * @brief  Function implementing the ChassisTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  for(;;)
  {
    // 检查是否有新的移动命令
    if (chassisState.moving && !chassisState.done)
    {
      // 根据移动类型执行相应操作
      switch (chassisState.move_type)
      {
        case MOVE_TYPE_POSITION_XYZ:
          // 执行闭环位置控制
          Move_To_Position_XYZ(chassisState.target_x, chassisState.target_y, chassisState.target_z, chassisState.timeout);
          break;
          
        case MOVE_TYPE_VISION:
          // 执行视觉闭环控制
          Move_By_Vision(chassisState.paper_id, chassisState.timeout);
          break;
          
        case MOVE_TYPE_EASY:
          // 执行简易闭环控制
          Move_By_Easy(chassisState.target_x, chassisState.target_y, chassisState.target_z, chassisState.timeout);
          break;
          
        case MOVE_TYPE_TRANSLATION:
          // 执行平移控制
          Move_Translation(chassisState.target_x, chassisState.target_y, chassisState.target_z, chassisState.timeout);
          break;
          
        default:
          break;
      }
      
      // 移动完成，更新状态
      chassisState.moving = false;
      chassisState.done = true;
      
      // 释放信号量，通知主任务底盘移动已完成
      osSemaphoreRelease(ChassisMoveDoneHandle);
      // printf("Chassis move done, move type: %d\r\n", chassisState.move_type);
    }
    
    osDelay(10);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartScaraTask */
/**
* @brief Function implementing the ScaraTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartScaraTask */
void StartScaraTask(void *argument)
{
  /* USER CODE BEGIN StartScaraTask */
  /* Infinite loop */
  for(;;)
  {
    // 检查是否有新的机械臂动作命令
    if (scaraState.moving && !scaraState.done)
    {
      HAL_TIM_Base_Start_IT(&htim5);
      // 根据动作类型执行相应操作
      switch (scaraState.action_type)
      {
        case 0: // Start_Scara
          Start_Scara();
          break;
          
        case 1: // Get_Box
          Get_Box(scaraState.box_id);//内部释放chasis信号量，控制底盘下一次运动时机
          break;
          
        case 2: // Ready_To_Put_Box
          Ready_To_Put_Box(scaraState.box_id);
          break;
        
        default:
          break;
      }
      
      // 动作完成，更新状态
      scaraState.moving = false;
      scaraState.done = true;
      HAL_TIM_Base_Stop_IT(&htim5);//关闭定时器避免浪费资源
      // 释放信号量，通知主任务机械臂动作已完成
      osSemaphoreRelease(ScaraMoveDoneHandle);
    }
    
    osDelay(10);
  }
  /* USER CODE END StartScaraTask */
}

/* USER CODE BEGIN Header_StartMainTask */
/**
* @brief Function implementing the MainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
  // 初始化
  bsp_init();
  app_init();
  Beep_On();

  osDelay(60);
  Beep_On();
  osDelay(2000);
  

//第一次启动
//************************************************************************** */

  // Init_All();  
  test_lift();
  // test_motor();


//************************************************************************** */  


//跑图测试
//*************************************************************************** */
// test_move();
//*************************************************************************** */

//校准
//*************************************************************************** */
// Init_All();
// Scara_Return_Home();
// Filter_Calibrate_Center_Position(SPIN_SERVO);
// auto_offset_omega();
// Beep_On
//*************************************************************************** */


//其他
  // Grab_Off();
  // Grab_On();
  // Start_Scara();
  // Car_Stop(1);
  // Move_To_Position_XYZ_NonBlocking(500,2000,180,15000);
  // Move_point_to_point(0,1);
  // Lift_Move_To_Height(9);
  // Start_Scara();
  // Move_By_Vision(0,100000);
  // Beep_On();

  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    // printf("omega: %f\r\n", omega);
  }
  /* USER CODE END StartMainTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


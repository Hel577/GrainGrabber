/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
void Put_Box(uint8_t box_id ,uint8_t dir);
void Move_To_Shelf(void);
void Move_To_Paper(void);
void Move_point_to_point(uint8_t point_1, uint8_t point_2);
void Move_Shelf_Left(void);
void Move_Shelf_Right(void);


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "car.h"
#include "app.h"
#include "bsp.h"
#include "my_task.h"

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

/* USER CODE BEGIN PV */
/// 远程调试串口
uint8_t rxcmd7[16];       // 接收远程串口的数据
uint8_t uart7_rx_flag = 0; // 串口接收标志
char rx_buffer[16];       // 接收缓冲区
uint8_t rx_index = 0;     // 接收索引
float target_speed = 0;    // 目标速度

// 陀螺仪串口
uint8_t rxcmd6_dma[RXCMD6_DMA_SIZE];      // DMA 写入的缓冲区
uint8_t rxcmd6_app[RXCMD6_DMA_SIZE];      // 应用程序读取的缓冲区


//树莓派串口
uint8_t rxcmd3_dma[RXCMD3_DMA_SIZE];
uint8_t move_flag = 0;//是否进行了点到点之间的移动，进行了就需要精调

//舵机串口
uint8_t rxcmd8_dma[RXCMD8_DMA_SIZE];



// 用于检测跳变的omega
volatile float omega_1 = 0;
volatile float omega = 0;                      // 陀螺仪反馈的角度，单位：°
volatile float current_angle_speed = 0;        // 陀螺仪反馈的角速度，单位：°/s
uint32_t last_receive_hwt_time = 0;   // 上一次接收陀螺仪数据的时间

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART6_UART_Init();
  MX_CAN1_Init();
  MX_TIM12_Init();
  MX_TIM5_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart3, rxcmd3_dma, RXCMD3_DMA_SIZE);
  HAL_UART_Receive_DMA(&huart8,rxcmd8_dma,RXCMD8_DMA_SIZE);
  memset(rxcmd3_dma, 0, RXCMD3_DMA_SIZE);
  memset(rxcmd8_dma,0,RXCMD8_DMA_SIZE);  

  
  
  // 初始化CAN1
  CAN_Filter_Config();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* 初始化CAN2 */
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);


  // HAL_TIM_Base_Start_IT(&htim5);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* CAN接收中断回调函数 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    // printf("\rCAN receive\r\n");
    /* 获取接收到的消息 */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        /* 判断是哪个CAN接口接收到数据 */
        if (hcan->Instance == CAN1)
        {
          uint32_t ext_id = RxHeader.ExtId;
            /* 处理CAN1接收到的数据 */
            if (RxHeader.IDE == CAN_ID_EXT) /* 扩展帧 */
            {
              // printf("CAN ID: 0x%08lX [PRI:%ld, PGN:0x%06lX, SA:0x%02lX]\r\n",
              // ext_id,
              // (ext_id >> 26) & 0x07,    // 优先级
              // (ext_id >> 8) & 0x3FFFF,  // PGN (18-bit)
              // ext_id & 0xFF);           // 源地址
              //   printf("ID=0x%08lX\r\n", RxHeader.ExtId);

              //   printf("DATA:");

              //   for(int i=0;i<RxHeader.DLC;i++)
              //   {
              //       printf("%02X \r\n", RxData[i]);
              //   }

              //   printf("\r\n");
                MIMotor_MotorDataDecode(RxHeader.ExtId, RxData);
            }
        }
        else if (hcan->Instance == CAN2)
        {
            /* 处理CAN2接收到的数据 */
            if (RxHeader.IDE == CAN_ID_EXT) /* 扩展帧 */
            {
                MIMotor_MotorDataDecode(RxHeader.ExtId, RxData);
            }
        }
    }
}

/**
 * @brief  Rx Transfer completed callback
 * @param  huart: UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //陀螺仪串口
    if (huart == &huart6)
    {
        static int wrap_count = 0;   // 静态本地变量，记录圈数
        static float prev_omega = 0; // 静态本地变量，记录上一次的角度
        // 记录接收时间戳
        last_receive_hwt_time = HAL_GetTick();

        // 将DMA缓冲区数据复制到应用缓冲区
        memcpy(rxcmd6_app, rxcmd6_dma, RXCMD6_DMA_SIZE);
        //  清空DMA缓冲区
        memset(rxcmd6_dma, 0, RXCMD6_DMA_SIZE);

        // 数据帧检测和解析
        for (int i = 0; i < RXCMD6_DMA_SIZE - 7; i++)
        {
          // 检测角度数据帧头(0x55 0x53)
          if (rxcmd6_app[i] == 0x55 && rxcmd6_app[i + 1] == 0x53)
          {
            // 解算角度数据(-180到180度)
            omega_1 = -((float)((int16_t)(rxcmd6_app[i + 7] << 8) | rxcmd6_app[i + 6])) / 32768 * 180;

            // 处理角度跳变，实现多圈角度计算
            if (prev_omega < 180 && prev_omega > 90 && omega_1 < -90 && omega_1 > -180)
            {
              wrap_count++; // 正向跳变
            }
            else if (prev_omega < -90 && prev_omega > -180 && omega_1 > 90 && omega_1 < 180)
            {
              wrap_count--; // 负向跳变
            }
            else
            {
              wrap_count = wrap_count;
            }

            if (omega_1 != 180 && omega_1 != -180)
            {
              // 计算实际角度
              omega = omega_1 + wrap_count * 360.0f;
              prev_omega = omega_1;

            }
          }

          // 检测角速度数据帧头(0x55 0x52)
          if (rxcmd6_app[i] == 0x55 && rxcmd6_app[i + 1] == 0x52)
          {
            // 解算角速度数据
            current_angle_speed = -((float)((int16_t)(rxcmd6_app[i + 7] << 8) | rxcmd6_app[i + 6])) / 32768 * 2000;
          }
        }

        // printf("omega: %.2f, current_angle_speed: %.2f\r\n", omega, current_angle_speed);
        // 重新启动DMA接收
        HAL_UART_Receive_DMA(&huart6, rxcmd6_dma, RXCMD6_DMA_SIZE);
    }

    //远程调试串口
    else if (huart == &huart7)
    {
      uart7_rx_flag = 1;  // 设置接收标志
    }

    // filter servo
    else if (huart == &huart8)
    {
    //舵机串口
        uint8_t data[RXCMD8_DMA_SIZE];
        memcpy(data,rxcmd8_dma,RXCMD8_DMA_SIZE);
        //  清空DMA缓冲区

        Filter_Process_Data(data);
        memset(rxcmd8_dma, 0, RXCMD8_DMA_SIZE);
        // 重新启动DMA接收
        HAL_UART_Receive_DMA(&huart8, rxcmd8_dma, RXCMD8_DMA_SIZE);
    }
    //raspi5
    else if (huart == &huart3)
    {
      
      // 处理接收到的数据
      Raspi_Process_Data(rxcmd3_dma, RXCMD3_DMA_SIZE);
      
      // 清空DMA缓冲区
      memset(rxcmd3_dma, 0, RXCMD3_DMA_SIZE);
      
      // 重新启动DMA接收
      HAL_UART_Receive_DMA(&huart3, rxcmd3_dma, RXCMD3_DMA_SIZE); 

    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM5)
  {
    // Update_Scara_Status();  
    // add_scara_ctrl();
    // Maintain_End_Rotation(); 
    static uint8_t state = 0;
       
       switch(state)
       {
           case 0:
              Update_Scara_Status();
              push_Update();
              Maintain_End_Rotation();
              Filter_Read_Pos(GRAB_SERVO);
              uint16_t target_angle = hand.grab_target_angle;
              int speed = 3400;
              int a = 200; 
              if(hand.grab_target_angle==GRAB_REALEASE && hand.grab_current_angle>GRAB_REALEASE){
                if(hand.grab_current_angle>1950+10){
                  Filter_Servo_PosCtrl(GRAB_SERVO,1950,3900,250);
                }
                // target_angle = hand.grab_current_angle-10;
                else{
                  speed = 220;
                  a = 200;
                  Filter_Servo_PosCtrl(GRAB_SERVO,target_angle,speed, a);
                }
              }
              else{
                Filter_Servo_PosCtrl(GRAB_SERVO,target_angle,speed, a);
              }
              state = 1;
              break;
           case 1:
               add_scara_ctrl();
               add_push_ctrl();
               state = 0;
               break;
      //      case 2:
      //         Maintain_End_Rotation(); 
      //          state = 0;
      //          break;
       }

  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

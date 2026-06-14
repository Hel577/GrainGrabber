/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "stdbool.h"
#include <stdlib.h>
#include "string.h"
#include "stdint.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DoorControl_Pin GPIO_PIN_0
#define DoorControl_GPIO_Port GPIOI
#define DoorControl2_Pin GPIO_PIN_12
#define DoorControl2_GPIO_Port GPIOH
#define DoorControl1_Pin GPIO_PIN_11
#define DoorControl1_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

#define RXCMD6_DMA_SIZE 33
#define RXCMD3_DMA_SIZE 9
#define RXCMD8_DMA_SIZE 9
// 全局变量声明
extern uint8_t rxcmd6_dma[RXCMD6_DMA_SIZE];
extern uint8_t rxcmd6_app[RXCMD6_DMA_SIZE];
extern uint8_t rxcmd3_dma[RXCMD3_DMA_SIZE];
extern uint8_t rxcmd8_dma[RXCMD8_DMA_SIZE];
extern volatile float omega;            // 实际角度值
extern volatile float omega_1;          // 原始角度值（-180到180度）
extern volatile float current_angle_speed; // 当前角速度
extern uint32_t last_receive_hwt_time; // 最后接收时间
extern uint8_t move_flag;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

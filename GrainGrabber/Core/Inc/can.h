/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
typedef  struct
{
  uint8_t category;
  uint16_t message;
  uint8_t target_id;
}TXID;

typedef struct 
{
  uint8_t category;
  uint8_t message;
  uint8_t source_id;
  uint8_t target_id;
}RXID;

typedef struct 
{
  /* data */
  uint8_t id;
  uint8_t MCU[8]; 
}DEVICE;

typedef enum {
    RESET0,
    CALI,
    MOTOR
}MODE;

typedef struct 
{
  /* data */
  MODE mode;
  uint8_t error;
  float angle;
  float ange_vel;
  float torque;
  float temp;
}STATE;

typedef struct
{
  /* data */
  uint16_t index;
  uint32_t data;
}para;




#define MASTER_CAN_ID 0xFF
#define GET_DEVICE_ID 0x00
#define GET_STATE_ID 0x02
#define ENABLE_ID 0x03
#define STOP_ID 0x04
#define SET_ZERO_ID 0x06
#define GET_PARA_ID 0x11
#define WRITE_PARA_ID 0x12

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Config(void);
extern CAN_TxHeaderTypeDef TXHeader;
extern CAN_RxHeaderTypeDef RXHeader;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   This file contains all the function prototypes for
  *          the motor.c file
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
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
void write_command_to_motor(uint8_t action_id, uint8_t target_id, uint8_t* data);
void read_message_from_motor(RXID* rxid, uint8_t* data);
void read_device(RXID* rxid, uint8_t* data, DEVICE* device);
void read_state(RXID* rxid, uint8_t* data, STATE* state);
void read_parameter(RXID* rxid, uint8_t* data, para* parameter);


#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
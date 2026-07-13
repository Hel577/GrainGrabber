#ifndef DOOR_H
#define DOOR_H

#include "gpio.h"
#include "scara.h"


#define DOOR_OPEN 1
#define DOOR_CLOSE 0

static uint32_t DOOR_OPEN_TIME[3] = {6000, 1800, 2500};

typedef struct{
    uint16_t pin;
    GPIO_TypeDef* port;
    uint8_t bean_id;//按照绿黄白进行处理
    uint8_t plate_id;//对应料盘的位置
}door_t;

void Door_Set_State(door_t door,uint8_t state);
void Door_Init(void);

extern door_t doors[3]; 

#endif
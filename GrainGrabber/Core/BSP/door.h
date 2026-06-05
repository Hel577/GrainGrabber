#ifndef DOOR_H
#define DOOR_H

#include "gpio.h"


#define DOOR_OPEN 1
#define DOOR_CLOSE 0

void Door_Set_State(uint8_t state);

#endif
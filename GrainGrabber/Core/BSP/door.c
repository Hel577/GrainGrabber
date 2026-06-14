#include "door.h"

door_t doors[3];

void Door_Set_State(door_t door,uint8_t state)
{
    if (state == DOOR_OPEN)
    {
        // 发送打开门的指令
        // 这里可以根据实际情况编写代码，例如控制电机转动等
        HAL_GPIO_WritePin(door.port, door.pin,GPIO_PIN_RESET);
    }
    else if (state == DOOR_CLOSE)
    {
        // 发送关闭门的指令
        // 这里可以根据实际情况编写代码，例如控制电机转动等
        HAL_GPIO_WritePin(door.port, door.pin,GPIO_PIN_SET);
    }
}

void Door_Init(void){
    /*从右到左为绿豆黄豆白芸豆*/
    doors[0].port = DoorControl_GPIO_Port;
    doors[0].pin = DoorControl_Pin;
    doors[0].bean_id = 1;
    doors[0].plate_id = 1;

    doors[1].port = DoorControl1_GPIO_Port;
    doors[1].pin = DoorControl1_Pin;
    doors[1].bean_id = 2;
    doors[1].plate_id = 2;

    doors[2].port = DoorControl2_GPIO_Port;
    doors[2].pin = DoorControl2_Pin;
    doors[2].bean_id = 3;
    doors[2].plate_id = 3;
}
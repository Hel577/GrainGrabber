#include "door.h"

void Door_Set_State(uint8_t state)
{
    if (state == DOOR_OPEN)
    {
        // 发送打开门的指令
        // 这里可以根据实际情况编写代码，例如控制电机转动等
        HAL_GPIO_WritePin(DoorControl_GPIO_Port, DoorControl_Pin,GPIO_PIN_RESET);
    }
    else if (state == DOOR_CLOSE)
    {
        // 发送关闭门的指令
        // 这里可以根据实际情况编写代码，例如控制电机转动等
        HAL_GPIO_WritePin(DoorControl_GPIO_Port, DoorControl_Pin,GPIO_PIN_SET);
    }
}
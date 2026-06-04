#include "my_task.h"




void Init_All(void)
{
  Reset_Scara_Motor_MechPosition();
  Reset_pushing_Motor_MechPosition();
  // Filter_Calibrate_Center_Position(GRAB_SERVO);
  // Filter_Calibrate_Center_Position(SPIN_SERVO);
  
}

void test_motor(void){
  // MI_motor_init(motors[0],&hcan1,0);
  // osDelay(100);
  // MI_motor_changeID(motors[0], 0, 1);
  // osDelay(100);
  // MI_motor_setMechPosition2Zero(motors[0]);
  // osDelay(100);
  // MI_motor_SetSpdLim(motors[0], 8);

  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
}

uint8_t box_ids[3] = {1,3,5};//选择的盒子编号，理论上由比赛抽签决定

void test_chassis(void){
  Move_To_Target(2);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(3);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(4);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(5);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(6);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(7);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(8);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(9);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  void sort_easy(uint8_t* arr, int size){
    //冒泡排序规划访问顺序
    for(int i=0;i<size-1;i++){
      for(int j=0;j<size-1-i;j++){
        if(arr[j]>arr[j+1]){
          uint8_t temp = arr[j];
          arr[j] = arr[j+1];
          arr[j+1] = temp;
        }
      }
    }
  }

  sort_easy(box_ids,3);
  Move_To_Placing_Box(box_ids);

}

void test_lift(void){
  printf("test lift\r\n");
  Init_All();
  Scara_PosCtrl(180,180);
  osDelay(5000);
  Scara_Return_Home();
  osDelay(5000);
}

void test_Push(void){
  printf("test push\r\n");
  Init_All();
  push_Move_To_Position(MAX_POSITION);
  osDelay(5000);
  push_Move_To_Position(MIN_POSITION);
  osDelay(5000);
  push_Move_To_Position(200);
  osDelay(5000);
}

void test_Graber(void){
  Grab_On();
  osDelay(5000);
  Grab_Off();
  osDelay(5000);

}


void test_Grab(void){
  printf("test grab\r\n");
  Init_All();
  push_Move_To_Position(MAX_POSITION);
  osDelay(5000);
  Scara_To_Height(200);
  osDelay(5000);
  Grab_On();
  Scara_To_Height(50);
  osDelay(5000);
  push_Move_To_Position(MIN_POSITION);
  osDelay(5000);
  Grab_Off();

}
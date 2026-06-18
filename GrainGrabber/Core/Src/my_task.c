#include "my_task.h"




void Init_All(void)
{
  Reset_Scara_Motor_MechPosition();
  Reset_pushing_Motor_MechPosition();
  // Filter_Calibrate_Center_Position(GRAB_SERVO);
  // Filter_Calibrate_Center_Position(SPIN_SERVO);
  
}

void test_motor(void){
  printf("test_motor\r\n");
   Move_To_Position_XYZ_NonBlocking(0, 500, 0, osWaitForever);
   osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
   osDelay(5000);
   Move_To_Position_XYZ_NonBlocking(0,-500,0,osWaitForever);
   osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  //  osDelay(5000);

  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
}

uint8_t box_ids[3] = {1,3,5};//选择的盒子编号，理论上由比赛抽签决定

void test_sss(void){
  Init_All();
  push_Move_To_Position(MAX_POSITION);
  osDelay(500);
  Grab_Off();
  osDelay(50);


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
  
  uint8_t bean_ids[3] = {2,2,2};
  Move_To_Placing_Box(box_ids,bean_ids);
}


void test_chassis(void){
  Init_All();
  osDelay(500);
  Grab_Off();

  Raspi_Send_Task(TASK_DETECT_BOX);
  while(raspi.box_id[0]==0){
    Raspi_Send_Task(TASK_DETECT_BOX);
  }
  Raspi_Finish_Task(TASK_DETECT_BOX);


  Move_To_Target(2);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(3);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(1,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  osDelay(200);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(raspi.bean_order[0]);
  Grab_Bean(1);
  osDelay(500);
  push_Move_To_Position(MAX_POSITION);
  
  Move_To_Target(4);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(5);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(2,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  osDelay(200);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  // Choose_Plate(raspi.bean_order[1]);
  Grab_Bean(2);
  osDelay(500);
  push_Move_To_Position(MAX_POSITION);

  Move_To_Target(6);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(7);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(3,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  osDelay(200);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  // Choose_Plate(raspi.bean_order[2]);
  Grab_Bean(3);
  osDelay(500);


}

void test_lift(void){
  printf("test lift\r\n");
  // Init_All();
  printf("move to height 200\r\n");
  Scara_To_Height(230);
  osDelay(5000);
  Scara_Return_Home();
  osDelay(5000);
}

void test_Push(void){
  printf("test push\r\n");
  Init_All();
  push_Move_To_Position(MIN_POSITION);
  osDelay(5000);
  push_Move_To_Position(MAX_POSITION);
  osDelay(5000);
  push_Move_To_Position(200);
  osDelay(5000);
}

void test_Graber(void){
  osDelay(5000);
  Grab_Off();
  osDelay(5000);
  Grab_On();
  osDelay(5000);
  Grab_Off();
  osDelay(5000);
}

void test_Raspi(void){
  Init_All();
  Grab_Off();
  osDelay(500);
  Raspi_Finish_Task(TASK_DETECT_BOX);
  // osDelay(50);
  // Raspi_Send_Task(TASK_DETECT_BOX);
  // while(raspi.box_id[0]==0)
  // {
  //   for(int i=0;i<5;i++){
  //     printf("%d ",raspi.box_id[i]);
  //   }
  //   printf("\r\n");
  // }
  // Raspi_Finish_Task(TASK_DETECT_BOX);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  while(true){
    printf("%f,%f\r\n",raspi.vision_x,raspi.vision_y);
    printf("%f,%f\r\n",raspi.real_x[1],raspi.real_y[1]);
    printf("%d\r\n",raspi.bean_order[0]);
    osDelay(200);
  }
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
}


void test_Grab(void){
  printf("test grab\r\n");
  Init_All();
  // Scara_To_Height(50);
  // push_Move_To_Position(MAX_POSITION);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  // Move_By_Vision_NonBlocking(2,5000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Grab_Off();
  osDelay(500);
  osDelay(200);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(2);
  Grab_Bean(1);
  osDelay(500);
  Grab_Off();
  push_Move_To_Position(MAX_POSITION);

}

// void test_door(void){
//   Door_Set_State(DOOR_CLOSE);
//   osDelay(5000);
//   Door_Set_State(DOOR_OPEN);
//   osDelay(5000);
// }

void test_graber_resend(void){
  while(true){
    printf("%d\r\n",hand.grab_current_angle);
    osDelay(100);
  }
}

void test_Spin(void){
  printf("Start Test Spin");

  Choose_Plate(2);
  osDelay(5000);
  Choose_Plate(1);
  osDelay(5000);
}

void test_Grab_Release(void){
  Grab_Release();
}

void test_omega(void){
  Init_All();
  auto_offset_omega();
}

int target_box[3] = {1,3,5};//对应绿，黄，白的放的编号，需要抽签选择

void my_task(void){
  Init_All();

  Raspi_Send_Task(TASK_DETECT_BOX);
  while(raspi.box_id[0]==0);
  Raspi_Finish_Task(TASK_DETECT_BOX);


  Move_To_Target(2);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(3);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(1,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  osDelay(200);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(raspi.bean_order[0]);
  Grab_Bean(1);
  osDelay(500);
  push_Move_To_Position(MAX_POSITION);
  
  Move_To_Target(4);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(5);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(2,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  osDelay(200);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  // Choose_Plate(raspi.bean_order[1]);
  Grab_Bean(2);
  osDelay(500);
  push_Move_To_Position(MAX_POSITION);

  Move_To_Target(6);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(7);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(3,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  osDelay(200);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  // Choose_Plate(raspi.bean_order[2]);
  Grab_Bean(3);
  osDelay(500);

  Move_To_Target(8);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(9);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  
  uint8_t target_ids[3] = {0,0,0};
  uint8_t bean_ids[3] = {0,0,0};

  Match_Box(target_box,target_ids,bean_ids);
  Move_To_Placing_Box(target_ids,bean_ids);
}

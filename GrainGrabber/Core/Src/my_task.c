#include "my_task.h"




void Init_All(void)
{
  Reset_Scara_Motor_MechPosition();
  // Reset_pushing_Motor_MechPosition();
  // Filter_Calibrate_Center_Position(GRAB_SERVO);
  // Filter_Calibrate_Center_Position(SPIN_SERVO);
  
}

void test_motor(void){
  printf("test_motor\r\n");
  Init_All();
  Grab_Off();
  osDelay(500);
  Set_Target_Index(24);
  Move_To_Target(25,osWaitForever);
  Set_Target_Index(20);
  Move_To_Target(10,osWaitForever);
  // while(true){
  //   En_Chassis_Motor();
  // }
  osDelay(5000);

}


void test_sss(void){
  int target_box[3] = {1,2,3};
  printf("test chassis\r\n");
  Init_All();
  osDelay(5000);

  Raspi_Send_Task(TASK_DETECT_BOX);
  // Scara_To_Height(SCARA_HEIGHT_MAX);
  osDelay(1000);
  while(raspi.box_id[0]==0){
    Raspi_Send_Task(TASK_DETECT_BOX);
  }
  Raspi_Finish_Task(TASK_DETECT_BOX);
  printf("box_id: %d,%d,%d,%d,%d\r\n",raspi.box_id[0],raspi.box_id[1],raspi.box_id[2],raspi.box_id[3],raspi.box_id[4]);
  Grab_Off();
  // // Choose_Plate(2);
  // // osDelay(50);

  // // Move_To_Target(2);
  // // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Move_To_Target(3,osWaitForever);
  // // Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  // Set_Target_Index(21);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  //   // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // Beep_On();
  // // Move_By_Vision_NonBlocking(1, 2500);
  // // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  // // Choose_Plate(raspi.bean_order[0]);
  // // Grab_Bean(1,raspi.bean_order[0]);
  // // push_Move_To_Position(MAX_POSITION);
  
  // Move_To_Target(4, osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Move_To_Target(5, osWaitForever);
  // // osDelay(osWaitForever);
  // // Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  // // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // Move_By_Vision_NonBlocking(2,1000);
  // Set_Target_Index(22);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  // // Choose_Plate(raspi.bean_order[1]);
  // // Grab_Bean(2,raspi.bean_order[1]);
  // // push_Move_To_Position(MAX_POSITION);

  // // Move_To_Target(6, osWaitForever);
  // // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // Set_Target_Index(23);
  // Move_To_Target(7, osWaitForever);
  // // osDelay(500);
  // // Scara_To_Height(SCARA_HEIGHT_MAX+50);
  // // Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  // Set_Target_Index(24);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Move_By_Vision_NonBlocking(3,2000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  
  // Choose_Plate(raspi.bean_order[2]);
  // Grab_Bean(3,raspi.bean_order[2]);
  // push_Move_To_Position(MAX_POSITION);

  // osDelay(osWaitForever);

  // Move_To_Target(8, osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  uint8_t target_ids[3] = {1,3,5};//豆子放哪个箱子
  uint8_t bean_ids[3] = {1,2,3};

  Match_Box(target_box,target_ids,bean_ids);
  if(target_ids[0]==1){
    Move_To_Target(28,osWaitForever);
  }
  else{
    Move_To_Target(27,osWaitForever);
  }
  osSemaphoreAcquire(ChassisMoveDoneHandle, 500);
  Set_Target_Index(20);
  // Move_By_Vision_NonBlocking(4, 5000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Placing_Box(target_ids,bean_ids);
}


void test_chassis(void){
  int target_box[3] = {1,2,3};
  printf("test chassis\r\n");
  Init_All();
  Raspi_Send_Task(TASK_DETECT_BOX);
  Scara_To_Height(SCARA_HEIGHT_MAX);


  // Move_To_Position_XYZ_NonBlocking(120,0,0,osWaitForever);
  while(raspi.box_id[0]==0){
    Raspi_Send_Task(TASK_DETECT_BOX);
  }
  Raspi_Finish_Task(TASK_DETECT_BOX);
  printf("box_id: %d,%d,%d,%d,%d\r\n",raspi.box_id[0],raspi.box_id[1],raspi.box_id[2],raspi.box_id[3],raspi.box_id[4]);
  Grab_Off();
  Choose_Plate(2);
  // osDelay(50);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);

  // Move_To_Target(2);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(3,osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Beep_On();
  Set_Target_Index(21);
  Move_By_Vision_NonBlocking(1, 2500);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(raspi.bean_order[0]);
  Grab_Bean(1,raspi.bean_order[0]);
  push_Move_To_Position(MAX_POSITION);
  
  Move_To_Target(4, osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // osDelay(osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_By_Vision_NonBlocking(2,800);
  Set_Target_Index(22);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(raspi.bean_order[1]);
  Grab_Bean(2,raspi.bean_order[1]);
  push_Move_To_Position(MAX_POSITION);

  // Move_To_Target(6, osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Set_Target_Index(23);
  Move_To_Target(7, osWaitForever);
  osDelay(800);
  Scara_To_Height(SCARA_HEIGHT_MAX+50);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Set_Target_Index(24);
  Move_By_Vision_NonBlocking(3,2000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(raspi.bean_order[2]);
  Grab_Bean(3,raspi.bean_order[2]);
  push_Move_To_Position(MAX_POSITION);

  // osDelay(osWaitForever);

  // Move_To_Target(8, osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  uint8_t target_ids[3] = {0,0,0};//豆子放哪个箱子
  uint8_t bean_ids[3] = {0,0,0};

  Match_Box(target_box,target_ids,bean_ids);
  printf("target_ids: %d,%d,%d\r\n",target_ids[0],target_ids[1],target_ids[2]);

  if(target_ids[0]==1){
    Move_To_Target(25,osWaitForever);
  }
  else{
    Move_To_Target(9,osWaitForever);
  }
  Set_Target_Index(20);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, 500);
  // Move_By_Vision_NonBlocking(4, 5000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Placing_Box(target_ids,bean_ids);
  Beep_On();
}

void test_lift(void){
  printf("test lift\r\n");
  Init_All();

  En_Chassis_Motor();
  printf("move to height 200\r\n");
  Scara_To_Height(SCARA_HEIGHT_MAX);
  // Scara_To_Height(269.05);
  Choose_Plate(2);
    Grab_Off();
  osDelay(5000);
  Scara_Return_Home();
  osDelay(5000);
  while(true){
    printf("%d,%d\r\n",scara.current_th1,scara.current_th2);
  }
}

void test_Push(void){
  printf("test push\r\n");
  Init_All();
  Grab_Off();
  osDelay(500);
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
  Choose_Plate(2);
  osDelay(5000);
  Grab_On();
  osDelay(5000);
  Grab_Off();
  osDelay(5000);
}

void test_Raspi(void){
  // osDelay(40000);
    // push_Move_To_Position(MAX_POSITION);
  Init_All();
  osDelay(500);
  // Scara_To_Height(SCARA_HEIGHT_MAX);
  // osDelay(500);
  Grab_Off();

  // Move_To_Position_XYZ_NonBlocking(0,0,180,osWaitForever);

  // osDelay(osWaitForever);
  while(raspi.vision_x==0){
    printf("%f,%f\r\n",raspi.vision_x,raspi.vision_y);
    printf("%f,%f\r\n",raspi.real_x[5],raspi.real_y[5]);
    printf("%d\r\n",raspi.bean_order[0]);
    // Raspi_Finish_Task(TASK_MOVE_BY_BOX);
    Raspi_Send_Task(TASK_MOVE_BY_BOX);
    osDelay(200);
  }
  // Scara_To_Height(50);
  Raspi_Send_Task(TASK_MOVE_BY_BOX);
  osDelay(500);
  osDelay(50);
  // osDelay(5000);
  // Choose_Plate(2);
  osDelay(50);
  // Move_To_Position_XYZ_NonBlocking(500,0,180,osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Move_To_Position_XYZ_NonBlocking(-500,0,180,osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Beep_On();
  // Move_By_Vision_NonBlocking(7,osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Grab_Bean(3);
  // Scara_Return_Home();
  osDelay(200);
  while(true){
    printf("%f,%f\r\n",raspi.vision_x,raspi.vision_y);
    printf("%f,%f\r\n",raspi.real_x[8],raspi.real_y[8]);
    printf("%d\r\n",recognize);
    osDelay(200);
  }
  Raspi_Finish_Task(TASK_MOVE_BY_BOX);

  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
}


void test_Grab(void){
  printf("test grab\r\n");
  Init_All();
  // Scara_To_Height(50);
  // push_Move_To_Position(MAX_POSITION);
  Scara_To_Height(SCARA_HEIGHT_MAX);
  Grab_Off();
  osDelay(10000);
  // Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  // Move_By_Vision_NonBlocking(1,5000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // Move_To_Position_XYZ_NonBlocking(500,0,90,osWaitForever);
  // // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // osDelay(500);
  // // osDelay(200);
  // // Move_To_Position_XYZ_NonBlocking(500,0,90,osWaitForever);
  // Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(2);
  Grab_Bean(1,2);
  osDelay(500);
  Grab_Off();
  push_Move_To_Position(MAX_POSITION);
  osDelay(700);
  // Scara_Return_Home();

}

void test_door(void){
  Choose_Plate(1);
  Release_Bean(1);
}

void test_graber_resend(void){
  Init_All();
  Choose_Plate(2);
  osDelay(1000);
  Release_Bean(2);
  osDelay(500);
  Choose_Plate(1);
  osDelay(1000);
  Release_Bean(1);
  osDelay(500);
  Choose_Plate(3);
  osDelay(1000);
  Release_Bean(3);
  osDelay(500);

}

void test_Spin(void){
  printf("Start Test Spin");
  Choose_Plate(2);
  osDelay(5000);
  Choose_Plate(3);
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

// int target_box[3] = {1,3,5};//对应绿，黄，白的放的编号，需要抽签选择

void my_task(void){
  int target_box[3] = {1,3,5};
  Init_All();

  Raspi_Send_Task(TASK_DETECT_BOX);
  while(raspi.box_id[0]==0);
  Raspi_Finish_Task(TASK_DETECT_BOX);


  Move_To_Target(2,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(3,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(1,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  osDelay(200);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(raspi.bean_order[0]);
  Grab_Bean(1,raspi.bean_order[0]);
  osDelay(500);
  push_Move_To_Position(MAX_POSITION);
  
  Move_To_Target(4,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(5,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(2,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  osDelay(200);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(raspi.bean_order[1]);
  Grab_Bean(2,raspi.bean_order[1]);
  osDelay(500);
  push_Move_To_Position(MAX_POSITION);

  Move_To_Target(6,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(7,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(3,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  osDelay(200);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(raspi.bean_order[2]);
  Grab_Bean(3,raspi.bean_order[2]);
  osDelay(500);

  Move_To_Target(8,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  
  uint8_t target_ids[3] = {0,0,0};//豆子放哪个箱子
  uint8_t bean_ids[3] = {0,0,0};

  Match_Box(target_box,target_ids,bean_ids);
  if(bean_ids[0]==1){
    Move_To_Target(9,osWaitForever);
  }
  else{
    Move_To_Target(9,osWaitForever);
  }
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);

  Move_By_Vision_NonBlocking(4, 5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Placing_Box(target_ids,bean_ids);
}

void CAN2_Send_Test(void)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t mailbox;

    uint8_t data[8] =
    {
        0x11,
        0x22,
        0x33,
        0x44,
        0x55,
        0x66,
        0x77,
        0x88
    };

    txHeader.StdId = 0x123;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(
        &hcan2,
        &txHeader,
        data,
        &mailbox
    );
}



void test_can(void){
  printf("test_can");
    while(1)
  {
      CAN2_Send_Test();
      // printf("send\r\n");
      HAL_Delay(1000);
  }
}

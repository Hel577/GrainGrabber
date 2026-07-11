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
  Init_All();
  // while(true){
  //   En_Chassis_Motor();
  // }
  Set_Target_Index(7);
  // Move_To_Target(8,osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  
  Move_Open_Loop_NonBlocking(0,1000,0,500);

  // Move_To_Position_XYZ_NonBlocking(1000,0,0,osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // osDelay(5000);
  // Move_To_Position_XYZ_NonBlocking(0,0,90,osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
  // Move_To_Position_XYZ_NonBlocking(-1000,0,-90,osWaitForever);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
}


void test_sss(void){
  int target_box[3] = {1,2,3};
  Init_All();
  osDelay(5000);
  Raspi_Send_Task(TASK_DETECT_BOX);
  while(raspi.box_id[0]==0){
    Raspi_Send_Task(TASK_DETECT_BOX);
  }
  Raspi_Finish_Task(TASK_DETECT_BOX);
  printf("box_id: %d,%d,%d,%d,%d\r\n",raspi.box_id[0],raspi.box_id[1],raspi.box_id[2],raspi.box_id[3],raspi.box_id[4]);
  Grab_Off();
  Choose_Plate(2);
  osDelay(50);

  // Move_To_Target(8);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  uint8_t target_ids[3] = {0,0,0};//豆子放哪个箱子
  uint8_t bean_ids[3] = {0,0,0};

  Match_Box(target_box,target_ids,bean_ids);
  for(int i=0;i<3;i++){
    printf("target_ids[%d]: %d, bean_ids[%d]: %d\r\n",i,target_ids[i],i,bean_ids[i]);
  }
  
  Move_To_Target(9,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);

  // Move_By_Vision_NonBlocking(4, 5000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Placing_Box(target_ids,bean_ids);
}


void test_chassis(void){
  int target_box[3] = {1,2,3};
  printf("test chassis\r\n");
  Init_All();
  osDelay(5000);

  Raspi_Send_Task(TASK_DETECT_BOX);
  while(raspi.box_id[0]==0){
    Raspi_Send_Task(TASK_DETECT_BOX);
  }
  Raspi_Finish_Task(TASK_DETECT_BOX);
  printf("box_id: %d,%d,%d,%d,%d\r\n",raspi.box_id[0],raspi.box_id[1],raspi.box_id[2],raspi.box_id[3],raspi.box_id[4]);
  Grab_Off();
  Choose_Plate(2);
  osDelay(50);

  // Move_To_Target(2);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(3,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Set_Target_Index(21);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(1, 3000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(raspi.bean_order[0]);
  Grab_Bean(1,raspi.bean_order[0]);
  push_Move_To_Position(MAX_POSITION);
  
  Move_To_Target(4, osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(5, osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, 1050);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(2,3000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(raspi.bean_order[1]);
  Grab_Bean(2,raspi.bean_order[1]);
  push_Move_To_Position(MAX_POSITION);

  Move_To_Target(6, osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, 1750);
  Move_To_Target(7, osWaitForever);
  Scara_To_Height(50);
  osSemaphoreAcquire(ChassisMoveDoneHandle, 1500);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(3,3000);
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
  if(bean_ids[0]==1){
    Move_To_Target(9,osWaitForever);
  }
  else{
    Move_To_Target(9,osWaitForever);
  }
  osSemaphoreAcquire(ChassisMoveDoneHandle, 500);
  Set_Target_Index(20);
  // Move_By_Vision_NonBlocking(4, 5000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Placing_Box(target_ids,bean_ids);
}

void test_lift(void){
  printf("test lift\r\n");
  Init_All();

  En_Chassis_Motor();
  printf("move to height 200\r\n");
  Scara_To_Height(230);
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
  Grab_Off();
  Init_All();
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
  Choose_Plate(2);
  osDelay(50);
  Move_To_Position_XYZ_NonBlocking(-500,0,0,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Position_XYZ_NonBlocking(500,0,0,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_By_Vision_NonBlocking(8,osWaitForever);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
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
  Grab_Off();
  osDelay(2000);
  // Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  // Move_By_Vision_NonBlocking(1,5000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // Move_To_Position_XYZ_NonBlocking(500,0,90,osWaitForever);
  // // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // // osDelay(500);
  // // osDelay(200);
  // // Move_To_Position_XYZ_NonBlocking(500,0,90,osWaitForever);
  // Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Choose_Plate(1);
  Grab_Bean(1,1);
  osDelay(500);
  Grab_Off();
  push_Move_To_Position(MAX_POSITION);
  Scara_Return_Home();

}

// void test_door(void){
//   Door_Set_State(DOOR_CLOSE);
//   osDelay(5000);
//   Door_Set_State(DOOR_OPEN);
//   osDelay(5000);
// }

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
    Move_To_Target(16,osWaitForever);
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

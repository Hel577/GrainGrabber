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
  // Move_To_Placing_Box(box_ids);

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


void test_Grab(void){
  printf("test grab\r\n");
  Init_All();
  // Scara_To_Height(50);
  // push_Move_To_Position(MAX_POSITION);
  Grab_Off();
  osDelay(5000);
  Scara_To_Height(270);
  osDelay(5000);
  Try_Grab_Beans(2);

  Scara_To_Height(50);
  osDelay(500);
  // push_Move_To_Position(MIN_POSITION);
  osDelay(5000);
  // Grab_Off();
  // push_Move_To_Position(MAX_POSITION);
  osDelay(5000);
  Scara_Return_Home();

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
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Grab_Bean(1);
  Choose_Plate(raspi.bean_order[0]);
  osDelay(500);
  push_Move_To_Position(MAX_POSITION);
  
  Move_To_Target(4);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(5);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(2,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Grab_Bean(2);
  Choose_Plate(raspi.bean_order[1]);
  osDelay(500);
  push_Move_To_Position(MAX_POSITION);

  Move_To_Target(6);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_To_Target(7);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BEAN);
  Move_By_Vision_NonBlocking(3,5000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Finish_Task(TASK_MOVE_BY_BEAN);
  Grab_Bean(3);
  Choose_Plate(raspi.bean_order[2]);
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
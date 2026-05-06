#include "my_task.h"




void Init_All(void)
{
  Reset_Scara_Motor_MechPosition();
  Reset_Lifting_Motor_MechPosition();
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
  Move_To_Shelf();//误差阈值可以拉大

  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
}

// void my_task(void)
// {
//   // 初始化所有模块
//   Start_Raspi_Detect();

//   while(raspi_detect_ok != 1)
//   {
//     osDelay(100);
//   }
//   Beep_On();
//   Start_Scara_NonBlocking();

//   //到达货架区域
//   Move_To_Shelf();//误差阈值可以拉大

//   osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
//   // osSemaphoreAcquire(ScaraMoveDoneHandle, osWaitForever);//等待机械臂初始化
//   Beep_On();


//   //货架一区
//   // Move_By_Vision_NonBlocking(0,1800);
//   // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待精调完成
//   Beep_On();
//   //需要在get_box合适位置释放chasis信号量
//   Get_Box_NonBlocking(1);
//   osSemaphoreAcquire(ScaraMoveDoneHandle, osWaitForever);
//   Beep_On();
//   osSemaphoreAcquire(ChassisMoveDoneHandle, 0);
//   Get_Box_NonBlocking(2);
//   osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待机械臂不干涉时机

//   //货架二区
//   //*********************************************** */

//     //将电机改为速度模式
//   Dis_Chassis_Motor();
//   Change_Chassis_Motor_Mode(MODE_SPD);
//   En_Chassis_Motor();

//   //******************************************* */
//   Move_Shelf_Right();
//   osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
//   Beep_On();
//   Move_By_Vision_NonBlocking(0,900);
//   osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待精调完成
//   Beep_On();
//   osSemaphoreAcquire(ScaraMoveDoneHandle, osWaitForever);
//   Get_Box_NonBlocking(4);
//   osSemaphoreAcquire(ScaraMoveDoneHandle, osWaitForever);
//   Beep_On();
//   osSemaphoreAcquire(ChassisMoveDoneHandle, 0);
//   Get_Box_NonBlocking(5);//内部释放chassis信号量
//   osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待机械臂不干涉时机
//   //*********************************************** */

//     //将电机改为速度模式
//   Dis_Chassis_Motor();
//   Change_Chassis_Motor_Mode(MODE_SPD);
//   En_Chassis_Motor();

//   //******************************************* */
//   //货架三区
//   Move_Shelf_Right(); 
//   osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
//   Beep_On();
//   Move_By_Vision_NonBlocking(0,1100);
//   osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待精调完成
//   Beep_On();
//   osSemaphoreAcquire(ScaraMoveDoneHandle, osWaitForever);
//   Get_Box_NonBlocking(3);
//   osSemaphoreAcquire(ScaraMoveDoneHandle, osWaitForever);
//   Beep_On();
//   osSemaphoreAcquire(ChassisMoveDoneHandle, 0);
//   Get_Box_NonBlocking(6);//内部释放chassis信号量
//   osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待机械臂不干涉时机


// //依次放置箱子
//   for(uint8_t i = 1; i < 7; i++)
//   {
//     put_round = i;
//     //粗调
//     Move_To_Next_Paper();
//     osSemaphoreAcquire(ScaraMoveDoneHandle, osWaitForever);
//     Ready_To_Put_Box_NonBlocking(raspi.box_id[put_round-1]);

//     if(move_flag == 1)
//     {
//       osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
//       Beep_On();
//       //精调
//       Move_By_Vision_NonBlocking(raspi.paper_id[put_round-1],1800);
//       osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
//       Beep_On();
//     }
//     // printf("move_to_%d_over\r\n", raspi.paper_id[put_round-1]);

//     // 放置箱子
//     osSemaphoreAcquire(ScaraMoveDoneHandle, osWaitForever);
//     osSemaphoreAcquire(ChassisMoveDoneHandle, 0);
//     Put_Box_NonBlocking(raspi.box_id[put_round-1], raspi.box_dir[put_round-1], raspi.maduo[put_round-1]);
//     osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
//     // osDelay(200);
//     // printf("put_box_%d_over\r\n", raspi.box_id[put_round-1]);

//   }

//   Beep_On();

//   // 移动到另一个区域
//   switch(raspi.paper_id[5])
//   {
//     case 1:
//       Move_To_Position_XYZ_NonBlocking(0,1800,270,3900);
//       break;
//     case 2:
//       Move_To_Position_XYZ_NonBlocking(-200,2200,180,3900);
//       break;
//     case 3:
//       Move_To_Position_XYZ_NonBlocking(200,2200,180,3900);
//       break;
//     case 4:
//       Move_To_Position_XYZ_NonBlocking(0,1800,90,3900);
//       break;  
//   }

// }


void test_move(void)
{ 
  // 初始化所有模块
  Start_Raspi_Detect();

  while(raspi_detect_ok != 1)
  {
    osDelay(100);
  }
  Beep_On();
  Start_Scara_NonBlocking();

  //到达货架区域
  Move_To_Shelf();//误差阈值可以拉大

  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
  osSemaphoreAcquire(ScaraMoveDoneHandle, osWaitForever);//等待机械臂初始化
  Beep_On();


  //货架一区
  Move_By_Vision_NonBlocking(0,1800);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待精调完成
  Beep_On();
  //货架二区
  //*********************************************** */

    //将电机改为速度模式
  Dis_Chassis_Motor();
  Change_Chassis_Motor_Mode(MODE_SPD);
  En_Chassis_Motor();

  //******************************************* */
  Move_Shelf_Right();
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
  Beep_On();
  Move_By_Vision_NonBlocking(0,900);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待精调完成
  Beep_On();
  //*********************************************** */

    //将电机改为速度模式
  Dis_Chassis_Motor();
  Change_Chassis_Motor_Mode(MODE_SPD);
  En_Chassis_Motor();

  //******************************************* */
  //货架三区
  Move_Shelf_Right(); 
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待粗调完成
  Beep_On();
  Move_By_Vision_NonBlocking(0,1100);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);//等待精调完成
  Beep_On();

//依次放置箱子
  for(uint8_t i = 1; i < 7; i++)
  {
    put_round = i;
    //粗调
    Move_To_Next_Paper();

    if(move_flag == 1)
    {
      osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
      Beep_On();
      //精调
      Move_By_Vision_NonBlocking(raspi.paper_id[put_round-1],1800);
      osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
      Beep_On();
    }
    // printf("move_to_%d_over\r\n", raspi.paper_id[put_round-1]);

    // osDelay(200);
    // printf("put_box_%d_over\r\n", raspi.box_id[put_round-1]);

  }

  Beep_On();

  // 移动到另一个区域
  switch(raspi.paper_id[5])
  {
    case 1:
      Move_To_Position_XYZ_NonBlocking(0,1800,270,3900);
      break;
    case 2:
      Move_To_Position_XYZ_NonBlocking(-200,2200,180,3900);
      break;
    case 3:
      Move_To_Position_XYZ_NonBlocking(200,2200,180,3900);
      break;
    case 4:
      Move_To_Position_XYZ_NonBlocking(0,1800,90,3900);
      break;  
  }

}
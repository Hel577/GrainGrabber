#include "app.h"
void app_init(void)
{
    Init_Imu();
    Init_Car();
    Init_push();
    Init_Scara();
    Init_Raspi();
    Door_Init();
    // Choose_Plate(2);
    // Grab_Off();
}

//用于判断车仓是否有箱子（1为有，0为无）
bool car_box[7] = {1, 1, 1, 1, 1, 1, 1};
uint8_t paper_box[7] = {0, 0, 0, 0, 0, 0, 0}; //用于判断纸箱是否有箱子（2为有两层，1为有一层，0为无）
uint8_t put_round = 0; //放置箱子的次数
uint8_t last_target_index = 1; // 用于记录上一个目标点的位置，用初始点进行初始化

//**************************机械臂动作组**********************************************/
//非阻塞机械臂动作组
void Start_Scara_NonBlocking(void)
{
  scaraState.action_type = 0;
  scaraState.moving = 1;
  scaraState.done = 0;
  osSemaphoreAcquire(ScaraMoveDoneHandle, 0);
}

/**
 * @brief 非阻塞版本的Get_Box函数，设置目标但不等待完成
 * @param box_id 箱子ID
 */
void Get_Box_NonBlocking(uint8_t box_id)
{
    osDelay(20);
    // 设置机械臂状态
    scaraState.box_id = box_id;
    scaraState.action_type = 1; // 1=Get_Box
    scaraState.moving = true;
    scaraState.done = false;
    
    // 获取信号量，标记机械臂正在移动
    osSemaphoreAcquire(ScaraMoveDoneHandle, 0);
}

/**
 * @brief 非阻塞版本的Ready_To_Put_Box函数，设置目标但不等待完成
 * @param box_id 箱子ID
 */
void Ready_To_Put_Box_NonBlocking(uint8_t box_id)
{
  osDelay(20);
  scaraState.box_id = box_id;
  scaraState.action_type = 2; // 2=ready_to_put_box
  scaraState.moving = true;
  scaraState.done = false;

  // 获取信号量，标记机械臂正在移动
  osSemaphoreAcquire(ScaraMoveDoneHandle, 0);
}

/**
 * @brief 非阻塞版本的Put_Box函数，设置目标但不等待完成
 * @param box_id 箱子ID
 * @param dir 方向
 * @param maduo 是否码垛
 */
void Put_Box_NonBlocking(uint8_t box_id, uint8_t dir, bool maduo)
{
    osDelay(20);
    // 设置机械臂状态
    scaraState.box_id = box_id;
    scaraState.paper_dir = dir;
    scaraState.maduo = maduo;
    scaraState.action_type = 3; // 3=Put_Box
    scaraState.moving = true;
    scaraState.done = false;
    
    // 获取信号量，标记机械臂正在移动
    osSemaphoreAcquire(ScaraMoveDoneHandle, 0);
}


void Start_Scara(void)
{
  push_PosCtrl(4.3);
  Grab_On();
  Scara_PosCtrl(192,-12);
  End_Rotation_Ctrl(0);
  osDelay(800);
  // Grab_Off();


}


void Get_Box(uint8_t box_id)
{
  switch(box_id)
  {
    case 1:
      Scara_PosCtrl(130,50);//伸到一层货架箱子上方
      Grab_Off();
      End_Rotation_Ctrl(0);
      osDelay(500);

      push_PosCtrl(2.45);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹爪箱子
      osDelay(500);

      push_PosCtrl(3.9);//上升到一层货架高度
      End_Rotation_Ctrl(90);//旋转90度
      osDelay(500);

      Scara_PosCtrl(193.5,-13.5);//收到车内负工作空间过度点（考虑干涉）
      osDelay(480);

      push_PosCtrl(8.9);//升到车一号位进入高度
      osDelay(400);

      End_Rotation_Ctrl(0);

      osDelay(350);

      Scara_PosCtrl(205,-115);//收到车一号位
      osDelay(700);

      push_PosCtrl(6.9);//下降到一号位一层抓放高度
      osDelay(400);

      Grab_Off();
      osDelay(300);
      push_PosCtrl(13);//升到二层货架进入高度
      osDelay(400);

      scara.step_angle +=3;
      Scara_PosCtrl(200,-20);//回到车内负工作空间过度点
      osDelay(500);
      scara.step_angle -=3;


     //   osDelay(500);
      
      // push_PosCtrl(4.5);//下降到一层货架高度
    //   osDelay(300);


      
      
      
      



      break;
    case 2:
      Scara_PosCtrl(130,50);//伸到二层货架箱子上方
      End_Rotation_Ctrl(0);
      Grab_Off();
      osDelay(600);

      push_PosCtrl(10.88);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹爪箱子
      osDelay(500);

      push_PosCtrl(11.5);//升到二层货架旋转高度
      osDelay(100);

      Scara_PosCtrl(210,-30);//收到车内负工作空间过度点（考虑干涉）
      osDelay(500);

      Scara_PosCtrl(278,-30);//收到车二号位
      osDelay(680);
      osSemaphoreRelease(ChassisMoveDoneHandle);

      push_PosCtrl(7);//下降到二号位一层抓放高度
      osDelay(460);

      Grab_Off();

      osDelay(300);
      push_PosCtrl(8.6);//升到车二号位离开高度
      osDelay(300);

      scara.step_angle +=3;
      Scara_PosCtrl(192,-12);//回到车内负工作空间过度点
      osDelay(500);
      scara.step_angle -=3;

      push_PosCtrl(3.9);//下降到一层货架高度
      Grab_On();
      // osSemaphoreRelease(ChassisMoveDoneHandle);


      osDelay(600);
      break;
    case 3:
      Scara_PosCtrl(130,50);//伸到一层货架箱子上方
      End_Rotation_Ctrl(0);
      Grab_Off();
      osDelay(500);

      push_PosCtrl(2.45);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹爪箱子
      osDelay(500);

      push_PosCtrl(3.9);//上升到退出一层货架高度
      End_Rotation_Ctrl(90);//旋转90度
      osDelay(500);

      Scara_PosCtrl(193,-13);//收到车内负工作空间过度点（考虑干涉）
      osDelay(680);
      push_PosCtrl(3.4);
      osDelay(140);


      Grab_Off();
      osDelay(300);

      push_PosCtrl(13);//升到进入二层货架高度
      Grab_On();
      End_Rotation_Ctrl(0);

      osDelay(800);




      break;
    case 4:

      Scara_PosCtrl(130,50);//伸到一层货架箱子上方
      End_Rotation_Ctrl(0);
      Grab_Off();
      osDelay(500);

      push_PosCtrl(2.45);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹住箱子
      osDelay(500);

      push_PosCtrl(3.9);//上升到退出一层货架高度
      End_Rotation_Ctrl(90);//旋转90度
      osDelay(500);

      Scara_PosCtrl(193,-13);//收到车内负工作空间过度点（考虑干涉）
      osDelay(500);

      push_PosCtrl(11.6);//升到四号位进入高度
      osDelay(650);
      End_Rotation_Ctrl(0);

      osDelay(300);

      Scara_PosCtrl(207,-118);//收到车四号位
      osDelay(700);

      push_PosCtrl(10.1);//降到放四号位高度
      osDelay(400);

      Grab_Off();
      osDelay(300);

      push_PosCtrl(13);//升到进入二层货架高度
      osDelay(300);

      scara.step_angle +=3;
      Scara_PosCtrl(200,-20);//收到车内负工作空间过度点（考虑干涉）
      osDelay(500);
      scara.step_angle -=3;



      break;
    case 5:

      Scara_PosCtrl(130,50);//伸到二层货架箱子上方
      End_Rotation_Ctrl(0);
      Grab_Off();
      osDelay(550);

      push_PosCtrl(10.88);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹住箱子
      osDelay(500);

      push_PosCtrl(14.8);//升到二层货架旋转高度
      osDelay(100);

      Scara_PosCtrl(210,-30);//收到车内负工作空间过度点（考虑干涉）
      osDelay(500);


      osSemaphoreRelease(ChassisMoveDoneHandle);
      Scara_PosCtrl(278,-30);//收到车五号位//要调整
      osDelay(500);

      push_PosCtrl(10.1);//降到放五号位高度
      osDelay(690);

      Grab_Off();
      osDelay(300);

      push_PosCtrl(12);//升到进入二层货架高度
      osDelay(300);
      
      scara.step_angle +=3;
      Scara_PosCtrl(192,-12);//收到车内负工作空间过度点（考虑干涉）
      osDelay(600);
      scara.step_angle -=3;

      push_PosCtrl(3.9);//升到进入一层货架高度
      Grab_On();
      osDelay(600);

      break;
    case 6:

      Scara_PosCtrl(130,50);//伸到二层货架箱子上方
      End_Rotation_Ctrl(0);
      Grab_Off();
      osDelay(550);

      push_PosCtrl(10.88);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹住箱子
      osDelay(500);

      push_PosCtrl(11.5);//升到二层货架旋转高度
      osDelay(100);

      // osDelay(500);
      Scara_PosCtrl(192,-12);//收到车内负工作空间过度点（考虑干涉）
      osDelay(450);
      End_Rotation_Ctrl(90);
      osDelay(150);
      push_PosCtrl(6.5);//降到6号位高度
      // End_Rotation_Ctrl(90);


      osDelay(650);
      osSemaphoreRelease(ChassisMoveDoneHandle);

      Grab_Off();
      osDelay(300);

      push_PosCtrl(13);//升到进入二层货架高度

      osDelay(200);

      // osDelay(1000);

      End_Rotation_Ctrl(0);
      osDelay(400);
      break;
    default:
      break;
  }
}


//用爪子先抓住箱子（使用直接的位置控制，便于与底盘同时控制）
void Ready_To_Put_Box(uint8_t box_id)
{
  osDelay(20);
  switch(box_id)
  {
    case 1:
      Scara_PosCtrl(205,-115);//收到车一号位
      Grab_Off();
      End_Rotation_Ctrl(0);
      osDelay(390);

      push_PosCtrl(6.7);//下降到一号位一层抓取高度
      osDelay(400);

      Grab_On();
      osDelay(500);
      break;
    case 2:
      Scara_PosCtrl(278,-30);//收到车二号位
      Grab_Off();
      End_Rotation_Ctrl(0);
      osDelay(390);

      push_PosCtrl(6.7);//下降到二号位一层抓取高度
      osDelay(400);

      Grab_On();
      osDelay(500);
      break;
    case 3:
      Scara_PosCtrl(192,-12);//收到车三号位
      Grab_Off();
      End_Rotation_Ctrl(90);
      osDelay(150);

      push_PosCtrl(3);//下降到三号位一层抓取高度
      osDelay(700);

      Grab_On();
      osDelay(400);
      
      break;
    case 4:
      Scara_PosCtrl(205,-115);//收到车四号位
      Grab_Off();
      End_Rotation_Ctrl(0);
      osDelay(530);

      push_PosCtrl(10);//下降到四号位一层抓取高度
      osDelay(330);

      Grab_On();
      osDelay(400);
      break;
    case 5:
      Scara_PosCtrl(278,-30);
      Grab_Off();
      End_Rotation_Ctrl(0);
      osDelay(530);

      push_PosCtrl(10);//下降到五号位一层抓取高度
      osDelay(330);

      Grab_On();
      osDelay(400);
      break;
    case 6:
      Scara_PosCtrl(192,-12);//收到车六号位
      Grab_Off();
      End_Rotation_Ctrl(90);
      osDelay(200);

      push_PosCtrl(6.4);//下降到六号位高度
      osDelay(560);

      Grab_On();
      osDelay(500);
      break;


  }
}


//****************************************底盘动作组************************************ */
//非阻塞版本底盘控制
void Move_To_Position_XYZ_NonBlocking(float target_x, float target_y, float target_z, uint32_t timeout)
{
  while(chassisState.moving == 1&& chassisState.done == 0) // 等待底盘移动完成
  {
    osDelay(10);
  }
  chassisState.target_x = target_x;
  chassisState.target_y = target_y;
  chassisState.target_z = target_z;
  chassisState.move_type = MOVE_TYPE_POSITION_XYZ;
  chassisState.timeout = timeout;
  chassisState.moving = 1;
  chassisState.done = 0;

  osSemaphoreAcquire(ChassisMoveDoneHandle, 0);
}

void Move_By_Vision_NonBlocking(uint8_t paper_id, uint32_t timeout)
{
  while(chassisState.moving == 1&& chassisState.done == 0) // 等待底盘移动完成
  {
    osDelay(10);
  }
  chassisState.paper_id = paper_id;
  chassisState.move_type = MOVE_TYPE_VISION;
  chassisState.timeout = timeout;
  chassisState.moving = 1;
  chassisState.done = 0;

  osSemaphoreAcquire(ChassisMoveDoneHandle, 0);
}

void Move_By_Easy_NonBlocking(float target_x, float target_y, float target_z, uint32_t timeout)
{
  while(chassisState.moving == 1&& chassisState.done == 0) // 等待底盘移动完成
  {
    osDelay(10);
  }
  chassisState.target_x = target_x;
  chassisState.target_y = target_y;
  chassisState.target_z = target_z;
  chassisState.move_type = MOVE_TYPE_EASY;
  chassisState.timeout = timeout;
  chassisState.moving = 1;
  chassisState.done = 0;

  osSemaphoreAcquire(ChassisMoveDoneHandle, 0);
}

void Move_Translation_NonBlocking(uint8_t last_target_index,uint8_t target_index,uint32_t timeout)
{
  while(chassisState.moving == 1&& chassisState.done == 0) // 等待底盘移动完成
  {
    osDelay(10);
  }
  chassisState.last_target_index = last_target_index;
  chassisState.target_index = target_index;
  chassisState.move_type = MOVE_TYPE_TRANSLATION;
  chassisState.timeout = timeout;
  chassisState.moving = 1;
  chassisState.done = 0;

  osSemaphoreAcquire(ChassisMoveDoneHandle, 0);
}


void Move_Open_Loop_NonBlocking(float speed_x, float speed_y, float target_z, uint32_t timeout)
{
  while(chassisState.moving == 1&& chassisState.done == 0) // 等待底盘移动完成
  {
    osDelay(10);
  }
  chassisState.target_x = speed_x;
  chassisState.target_y = speed_y;
  chassisState.target_z = target_z;
  chassisState.move_type = MOVE_OPEN_LOOP;
  chassisState.timeout = timeout;
  chassisState.moving = 1;
  chassisState.done = 0;

  osSemaphoreAcquire(ChassisMoveDoneHandle, 0);
}


// 定义不同目标点的位置

float const OFFSET_x = 1.0f;
float const OFFSET_y = 1.0f;

float target_positions[30][3] = {
    {1525*OFFSET_x, 1000*OFFSET_y, 0},//1
    {2795*OFFSET_x, 500 *OFFSET_y, 0},//2
    {3545*OFFSET_x, 500 *OFFSET_y, 0},//3
    {3275*OFFSET_x, 501 *OFFSET_y, 0},//4
    {3260*OFFSET_x, 1000*OFFSET_y, 0},//5
    {3260*OFFSET_x, 1500*OFFSET_y, 0},//6
    {3250*OFFSET_x, 1501*OFFSET_y, 0},//7
    {2500*OFFSET_x, 1500*OFFSET_y, 90},//8
    {605*OFFSET_x, 1500*OFFSET_y, 180},//9//这里是缝缝补补这一块
    {380*OFFSET_x, 445*OFFSET_y, 270},//10
    {445*OFFSET_x, 580*OFFSET_y, 180},//11
    {445*OFFSET_x,980*OFFSET_y,  180},//12
    {445*OFFSET_x,1440*OFFSET_y,  180},//13
    {430*OFFSET_x,1535*OFFSET_y,  90},//14
    {630*OFFSET_x,500 *OFFSET_y,  180},//15
    {630*OFFSET_x,1500*OFFSET_y,  180},//16

    {375*OFFSET_x, 600*OFFSET_y, 180},//17
    {375*OFFSET_x,1000*OFFSET_y,  180},//18
    {375*OFFSET_x,1400*OFFSET_y,  180},//19//初始临点，将微调和初始点分开
    {785*OFFSET_x, 500*OFFSET_y, 180},//20
    {3535*OFFSET_x, 500 *OFFSET_y, 0},//21//对应3的位置
    {3260*OFFSET_x, 1000*OFFSET_y, 0},//22//对应5的位置
    {3260*OFFSET_x, 1500*OFFSET_y, 0},//23//对应6号位置
    {3535*OFFSET_x, 1500*OFFSET_y, 0},//24//对应7号位置
    {605*OFFSET_x, 1500*OFFSET_y, 180},//25//对应9号点位，但是可以避免多换一次朝向
    {630*OFFSET_x,1500*OFFSET_y,  90},//26//对应16号点位，不用多次旋转
    {600*OFFSET_x, 1000*OFFSET_y, 180},//27//中心中转点，应用于2->5和1->4两种情况

    {425*OFFSET_x, 570*OFFSET_y, 180},//28//对应11，12，13号点，为初次抵达时的点位
    {425*OFFSET_x,970*OFFSET_y,  180},//29
    {425*OFFSET_x,1420*OFFSET_y,  180},//30
};

int timeout[17][17] = {
    // 0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15
    {0,   3667,5000,4500,4167,4667,5667,4834,4000,5500,4834,5667,6500,5334,3000,3834},
    {2000,0,   1667,1334,1834,2500,3500,2334,3834,5000,5334,6167,7000,5834,4667,3834},
    {3334,1667,0,   834, 1334,2000,2667,1834,5167,6334,6167,7000,7834,6667,5500,4667},
    {2834,1334,834, 0,   1167,1834,2667,1834,4667,5834,6000,6834,7667,6500,5334,4500},
    {2500,1834,1334,1167,0,   1167,1834,1500,4334,5500,5500,6334,7167,6000,4834,4000},
    {3000,2500,2000,1834,1167,0,   1167,1167,4834,5667,4834,5667,6500,5334,4334,3500},
    {4000,3500,2667,2667,1834,1167,0,   1667,5834,6500,5667,6500,7334,6167,5167,4334},
    {3167,2334,1834,1834,1500,1167,1667,0,   5000,5834,5334,6167,7000,5834,4667,3834},
    {2334,3834,5167,4667,4334,4834,5834,5000,0,   3500,4667,5500,6334,5167,3000,3834},
    {3834,5000,6334,5834,5500,5667,6500,5834,3500,0,   3667,4500,5334,4000,2834,4000},
    {4834,5334,6167,6000,5500,4834,5667,5334,4667,3667,0,   1500,2334,1667,4000,2667},
    {5667,6167,7000,6834,6334,5667,6500,6167,5500,4500,1500,0,   1500,1834,4834,3500},
    {6500,7000,7834,7667,7167,6500,7334,7000,6334,5334,2334,1500,0,   2500,5667,4334},
    {5334,5834,6667,6500,6000,5334,6167,5834,5167,4000,1667,1834,2500,0,   4500,3167},
    {3000,4667,5500,5334,4834,4334,5167,4667,3000,2834,4000,4834,5667,4500,0,   2334},
    {3834,3834,4667,4500,4000,3500,4334,3834,3834,4000,2667,3500,4334,3167,2334,0}
};


void Move_To_Target(uint8_t target_id, uint32_t timeout_value){
  float target_x, target_y, target_z;
  target_x = target_positions[target_id-1][0];
  target_y = target_positions[target_id-1][1];
  target_z = target_positions[target_id-1][2];

  float begin_x = target_positions[last_target_index-1][0];
  float begin_y = target_positions[last_target_index-1][1];

  float dis_x = target_x - begin_x;
  float dis_y = target_y - begin_y + y_error;



  // Move_To_Position_XYZ_NonBlocking(dis_x, dis_y, target_z, timeout[last_target_index][target_id-1]);
  Move_To_Position_XYZ_NonBlocking(dis_x, dis_y, target_z, timeout_value);
  last_target_index = target_id;
}

void Move_To_Target_Direct(uint8_t target_id, uint32_t timeout_value){
  //后半场非直线连续移动
  Move_Translation_NonBlocking(last_target_index,target_id,timeout_value);
  last_target_index = target_id;
}

void Set_Target_Index(uint8_t target_id){
  last_target_index = target_id;
}

void Release_Bean(uint8_t bean_id){
    //bean_id和plate_id完全相等s
    // Door_Set_State(doors[bean_id-1],DOOR_OPEN);
    osDelay(DOOR_OPEN_TIME[bean_id-1]);
    Door_Set_State(doors[bean_id-1],DOOR_CLOSE);
}




void Move_To_Placing_Box(uint8_t* box_ids,uint8_t* bean_ids){
  //定义放置盒子从上向下分别为1，2，3, 4, 5
  Raspi_Send_Task(TASK_MOVE_BY_BOX);
  if(box_ids[0]==1){
    Move_To_Target(box_ids[0]+9,osWaitForever);//box_id+9为箱子放置位置的id
  }
  else{
    Move_To_Target(box_ids[0]+26,osWaitForever);
  }
  Choose_Plate(bean_ids[0]);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Move_By_Vision_NonBlocking(box_ids[0]+4, 3000);
  if(box_ids[0]==2||box_ids[0]==3||box_ids[0]==4){
    Set_Target_Index(box_ids[0]+15);
  }
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Release_Bean(bean_ids[0]);

  if(box_ids[0]==1){
    Move_To_Target_Direct(box_ids[1]+26,osWaitForever);
    // osSemaphoreAcquire(ChassisMoveDoneHandle, 550);
  }
  else{
    Move_To_Target(box_ids[1]+9,osWaitForever);//前往第二个箱子的位置
  }
  Choose_Plate(bean_ids[1]);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BOX);
  Move_By_Vision_NonBlocking(box_ids[1]+4, 3000);
  if(box_ids[1]==2||box_ids[1]==3||box_ids[1]==4){
    Set_Target_Index(box_ids[1]+15);
  }
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Release_Bean(bean_ids[1]);

  if(box_ids[2]==5){
    Move_To_Target_Direct(box_ids[2]+9,osWaitForever);
    // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
    //如果最后一个盒子是5号位，也需要进入中转点
  }
  else{
    Move_To_Target(box_ids[2]+9,osWaitForever);//前往第三个箱子的位置
  }
  Choose_Plate(bean_ids[2]);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Raspi_Send_Task(TASK_MOVE_BY_BOX);
  Move_By_Vision_NonBlocking(box_ids[2]+4, 3000);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  Release_Bean(bean_ids[2]);
  Raspi_Finish_Task(TASK_MOVE_BY_BOX);
}

void Move_To_Placing_Box_NoCamera(uint8_t* box_ids,uint8_t* bean_ids){
  //定义放置盒子从上向下分别为1，2，3, 4, 5
  // Raspi_Send_Task(TASK_MOVE_BY_BOX);
  Move_To_Target(box_ids[0]+9,osWaitForever);//box_id+9为箱子放置位置的id
  // Choose_Plate(bean_ids[0]);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Move_By_Vision_NonBlocking(box_ids[0]+4, 3000);
  if(box_ids[0]==2||box_ids[0]==3||box_ids[0]==4){
    Set_Target_Index(box_ids[0]+15);
  }
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);


  // Release_Bean(bean_ids[0]);

  if(box_ids[0]==1){
    switch(box_ids[1]){
      case 2:
      case 3:{
        Move_To_Target(15,osWaitForever);
        break;
      }
      case 4:{
        Move_To_Target(16,osWaitForever);
        break;
      default:
        break;
      }
    }//如果第一个盒子是1号位，则需要进入中转点
    osSemaphoreAcquire(ChassisMoveDoneHandle, 550);
  }

  // Raspi_Send_Task(TASK_MOVE_BY_BOX);
  Move_To_Target(box_ids[1]+9,osWaitForever);//前往第二个箱子的位置
  // Choose_Plate(bean_ids[1]);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Move_By_Vision_NonBlocking(box_ids[1]+4, 3000);
  if(box_ids[1]==2||box_ids[1]==3||box_ids[1]==4){
    Set_Target_Index(box_ids[1]+15);
  }
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Release_Bean(bean_ids[1]);

  if(box_ids[2]==5){
    Move_To_Target(26,osWaitForever);
    osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
    //如果最后一个盒子是5号位，也需要进入中转点
  }

  // Raspi_Send_Task(TASK_MOVE_BY_BOX);
  Move_To_Target(box_ids[2]+9,osWaitForever);//前往第三个箱子的位置
  // Choose_Plate(bean_ids[2]);
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Move_By_Vision_NonBlocking(box_ids[2]+4, 3000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  // Release_Bean(bean_ids[2]);
  // Raspi_Finish_Task(TASK_MOVE_BY_BOX);
}


uint16_t threashold[3] = {1900,2056,1500};//对应绿黄白的顺序

void Try_Grab_Beans(uint8_t bean_id){
  uint8_t try_times = 0;
  uint16_t angle_threashold = threashold[bean_id-1];
  for(;try_times<1;try_times++){
    //抓取次数小于最大尝试次数，继续尝试抓取
    if(try_times==0){
      Grab_On();
      osDelay(500);
      Scara_To_Height(175);
      osDelay(500);
      // Grab_Open_Slitly();
      // osDelay(5000);
      // Grab_On();
    }
    else{
      osDelay(5000);
      Scara_To_Height(270);
      osDelay(1000);
      Grab_Pos_Ctrl(1700);
      osDelay(2000);
      Grab_On();
      osDelay(5000);
      Scara_To_Height(175);
      // Grab_Open_Slitly();
      osDelay(5000);
    }

    if(hand.grab_current_angle>angle_threashold){
      try_times = 0;
      return;
    }

  }
  //超过最大次数，放弃抓取，继续执行后续步骤
  try_times = 0;
}

void Choose_Plate(uint8_t plate_id){
  /*从右到左为1，2，3 */
  End_Rotation_Ctrl((plate_id-2)*120);
}

void Grab_Bean(uint8_t bean_id,uint8_t bean){
  float error = 0;
  uint32_t release_time;
  int value = 1460;
  switch(bean){
    case 1:{
      error = 3;//绿豆
      release_time = 500*1.8;
      value = GRAB_RELEASE_GREEN;
      break;
    }
    case 2:{
      error = 3;
      release_time = 350*1.8;
      value = GRAB_RELEASE_YELLOW;
      break;
    }
    case 3:{
      error = 10;
      release_time = 0;
      break;
    }
  }
  Scara_To_Height(SCARA_HEIGHT_BEAN[bean_id-1]-error);
  osDelay(SCARA_TIME_BEAN[bean_id-1]);
  // Grab_On();
  osDelay(180*1.8);
  Scara_To_Height(SCARA_HEIGHT_MAX);
  osDelay(SCARA_TIME_BEAN[bean_id-1]);
  push_Move_To_Position(MIN_POSITION);
  osDelay(300*1.8);
  Grab_Release(value);
  osDelay(release_time);
  Grab_Off();
  osDelay(150*1.8);
}

void Match_Box(int* target_box,uint8_t* target_ids,uint8_t* bean_ids){
  /*将数据处理成要去的点位和对应的要放的豆子的编号
  1，2，3对应绿黄白*/
  uint8_t* box_ids = raspi.box_id;
  for(int i=0;i<3;i++){
    int target = target_box[i];
    for(int j=0;j<5;j++){
      if(box_ids[j]==target){
        target_ids[i] = j+1;
        bean_ids[i] = i+1;
      }
    }
  }

    void sort_easy(uint8_t* arr, uint8_t* beans,int size){
      //冒泡排序规划访问顺序
      for(int i=0;i<size-1;i++){
        for(int j=0;j<size-1-i;j++){
          if(arr[j]>arr[j+1]){
            uint8_t temp = arr[j];
            uint8_t bean_temp = beans[j];
            arr[j] = arr[j+1];
            arr[j+1] = temp;
            beans[j] = beans[j+1];
            beans[j+1] = bean_temp;
          }
        }
      }
    }

  sort_easy(target_ids,bean_ids,3);

}
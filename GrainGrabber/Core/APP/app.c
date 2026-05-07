#include "app.h"
void app_init(void)
{
    Init_Imu();
    Init_Car();
    Init_Lift();
    Init_Scara();
    Init_Raspi();
}

//用于判断车仓是否有箱子（1为有，0为无）
bool car_box[7] = {1, 1, 1, 1, 1, 1, 1};
uint8_t paper_box[7] = {0, 0, 0, 0, 0, 0, 0}; //用于判断纸箱是否有箱子（2为有两层，1为有一层，0为无）
uint8_t put_round = 0; //放置箱子的次数
uint8_t last_target_index = 0; // 用于记录上一个目标点的位置，用初始点进行初始化

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
  Lift_PosCtrl(4.3);
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

      Lift_PosCtrl(2.45);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹爪箱子
      osDelay(500);

      Lift_PosCtrl(3.9);//上升到一层货架高度
      End_Rotation_Ctrl(90);//旋转90度
      osDelay(500);

      Scara_PosCtrl(193.5,-13.5);//收到车内负工作空间过度点（考虑干涉）
      osDelay(480);

      Lift_PosCtrl(8.9);//升到车一号位进入高度
      osDelay(400);

      End_Rotation_Ctrl(0);

      osDelay(350);

      Scara_PosCtrl(205,-115);//收到车一号位
      osDelay(700);

      Lift_PosCtrl(6.9);//下降到一号位一层抓放高度
      osDelay(400);

      Grab_Off();
      osDelay(300);
      Lift_PosCtrl(13);//升到二层货架进入高度
      osDelay(400);

      scara.step_angle +=3;
      Scara_PosCtrl(200,-20);//回到车内负工作空间过度点
      osDelay(500);
      scara.step_angle -=3;


     //   osDelay(500);
      
      // Lift_PosCtrl(4.5);//下降到一层货架高度
    //   osDelay(300);


      
      
      
      



      break;
    case 2:
      Scara_PosCtrl(130,50);//伸到二层货架箱子上方
      End_Rotation_Ctrl(0);
      Grab_Off();
      osDelay(600);

      Lift_PosCtrl(10.88);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹爪箱子
      osDelay(500);

      Lift_PosCtrl(11.5);//升到二层货架旋转高度
      osDelay(100);

      Scara_PosCtrl(210,-30);//收到车内负工作空间过度点（考虑干涉）
      osDelay(500);

      Scara_PosCtrl(278,-30);//收到车二号位
      osDelay(680);
      osSemaphoreRelease(ChassisMoveDoneHandle);

      Lift_PosCtrl(7);//下降到二号位一层抓放高度
      osDelay(460);

      Grab_Off();

      osDelay(300);
      Lift_PosCtrl(8.6);//升到车二号位离开高度
      osDelay(300);

      scara.step_angle +=3;
      Scara_PosCtrl(192,-12);//回到车内负工作空间过度点
      osDelay(500);
      scara.step_angle -=3;

      Lift_PosCtrl(3.9);//下降到一层货架高度
      Grab_On();
      // osSemaphoreRelease(ChassisMoveDoneHandle);


      osDelay(600);
      break;
    case 3:
      Scara_PosCtrl(130,50);//伸到一层货架箱子上方
      End_Rotation_Ctrl(0);
      Grab_Off();
      osDelay(500);

      Lift_PosCtrl(2.45);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹爪箱子
      osDelay(500);

      Lift_PosCtrl(3.9);//上升到退出一层货架高度
      End_Rotation_Ctrl(90);//旋转90度
      osDelay(500);

      Scara_PosCtrl(193,-13);//收到车内负工作空间过度点（考虑干涉）
      osDelay(680);
      Lift_PosCtrl(3.4);
      osDelay(140);


      Grab_Off();
      osDelay(300);

      Lift_PosCtrl(13);//升到进入二层货架高度
      Grab_On();
      End_Rotation_Ctrl(0);

      osDelay(800);




      break;
    case 4:

      Scara_PosCtrl(130,50);//伸到一层货架箱子上方
      End_Rotation_Ctrl(0);
      Grab_Off();
      osDelay(500);

      Lift_PosCtrl(2.45);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹住箱子
      osDelay(500);

      Lift_PosCtrl(3.9);//上升到退出一层货架高度
      End_Rotation_Ctrl(90);//旋转90度
      osDelay(500);

      Scara_PosCtrl(193,-13);//收到车内负工作空间过度点（考虑干涉）
      osDelay(500);

      Lift_PosCtrl(11.6);//升到四号位进入高度
      osDelay(650);
      End_Rotation_Ctrl(0);

      osDelay(300);

      Scara_PosCtrl(207,-118);//收到车四号位
      osDelay(700);

      Lift_PosCtrl(10.1);//降到放四号位高度
      osDelay(400);

      Grab_Off();
      osDelay(300);

      Lift_PosCtrl(13);//升到进入二层货架高度
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

      Lift_PosCtrl(10.88);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹住箱子
      osDelay(500);

      Lift_PosCtrl(14.8);//升到二层货架旋转高度
      osDelay(100);

      Scara_PosCtrl(210,-30);//收到车内负工作空间过度点（考虑干涉）
      osDelay(500);


      osSemaphoreRelease(ChassisMoveDoneHandle);
      Scara_PosCtrl(278,-30);//收到车五号位//要调整
      osDelay(500);

      Lift_PosCtrl(10.1);//降到放五号位高度
      osDelay(690);

      Grab_Off();
      osDelay(300);

      Lift_PosCtrl(12);//升到进入二层货架高度
      osDelay(300);
      
      scara.step_angle +=3;
      Scara_PosCtrl(192,-12);//收到车内负工作空间过度点（考虑干涉）
      osDelay(600);
      scara.step_angle -=3;

      Lift_PosCtrl(3.9);//升到进入一层货架高度
      Grab_On();
      osDelay(600);

      break;
    case 6:

      Scara_PosCtrl(130,50);//伸到二层货架箱子上方
      End_Rotation_Ctrl(0);
      Grab_Off();
      osDelay(550);

      Lift_PosCtrl(10.88);//降低高度贴着箱子
      osDelay(350);

      Grab_On();//夹住箱子
      osDelay(500);

      Lift_PosCtrl(11.5);//升到二层货架旋转高度
      osDelay(100);

      // osDelay(500);
      Scara_PosCtrl(192,-12);//收到车内负工作空间过度点（考虑干涉）
      osDelay(450);
      End_Rotation_Ctrl(90);
      osDelay(150);
      Lift_PosCtrl(6.5);//降到6号位高度
      // End_Rotation_Ctrl(90);


      osDelay(650);
      osSemaphoreRelease(ChassisMoveDoneHandle);

      Grab_Off();
      osDelay(300);

      Lift_PosCtrl(13);//升到进入二层货架高度

      osDelay(200);

      // osDelay(1000);

      End_Rotation_Ctrl(0);
      osDelay(400);
      break;
    default:
      break;
  }
}



//box_id 1-6 dir:0-2 对应纸垛左中右  maduo:true 码垛 false 单放
void Put_Box(uint8_t box_id ,uint8_t dir, bool maduo)
{

    switch(box_id)
    { 
        case 1:
          // Scara_PosCtrl(205,-115);//收到车一号位
          // Grab_Off();
          // End_Rotation_Ctrl(0);
          // osDelay(390);

          // Lift_PosCtrl(6.7);//下降到一号位一层抓取高度
          // osDelay(400);

          // Grab_On();
          // osDelay(500);

          Lift_PosCtrl(12);//升到车一号位离开高度
          osDelay(500);

          Scara_PosCtrl(210,-30);//
          osDelay(600);

          Scara_PosCtrl(130,50);//正工作空间衔接位
          // osDelay(500);
          if(dir == 1)
          {
            Lift_PosCtrl(9);//下降到衔接高度
          }
          else if(dir == 2|| dir == 0)
          {
            if(maduo)
            {
              Lift_PosCtrl(8.2);//下降到衔接高度
            }
            else
            {
              Lift_PosCtrl(12);//下降到衔接高度
            }
              
          }


        break;
        case 2:
          // Scara_PosCtrl(278,-30);//收到车二号位
          // Grab_Off();
          // End_Rotation_Ctrl(0);
          // osDelay(390);

          // Lift_PosCtrl(6.7);//下降到二号位一层抓取高度
          // osDelay(400);

          // Grab_On();
          // osDelay(500);

          Lift_PosCtrl(12);//升到车二号位离开高度
          osDelay(500);

          Scara_PosCtrl(210,-30);//负工作空间过度点
          osDelay(600);

          Scara_PosCtrl(130,50);//正工作空间衔接位
        
          if(dir == 1)
          {
            Lift_PosCtrl(9);//下降到衔接高度
          }
          else if(dir == 2|| dir == 0)
          {
            if(maduo)
            {
              Lift_PosCtrl(8.2);//下降到衔接高度
            }
            else
            {
              Lift_PosCtrl(12);//下降到衔接高度
            }
              
          }

        break;
        case 3:
          // Scara_PosCtrl(192,-12);//收到车三号位
          // Grab_Off();
          // End_Rotation_Ctrl(90);
          // osDelay(150);

          // Lift_PosCtrl(3);//下降到三号位一层抓取高度
          // osDelay(700);

          // Grab_On();
          // osDelay(400);
          if(maduo)
          {
            Lift_PosCtrl(8.4);//升到车三号位离开高度
            End_Rotation_Ctrl(0);
            if((car_box[1] ==1||car_box[2] == 1)&&(car_box[4] ==0&&car_box[5]==0))
            {
                Lift_PosCtrl(11.7);
                osDelay(250);
            }
            else if(car_box[4] ==1||car_box[5] == 1)
            {
                Lift_PosCtrl(14.5);
                osDelay(450);

            }
          
            osDelay(600);
            Scara_PosCtrl(206,-26);
            osDelay(200);//往后退一点
            Scara_PosCtrl(130,50);//正工作空间衔接位
            if(dir == 1)
            {
              osDelay(300);
            }
            else
            {
              osDelay(300);
            }
          }
          else
          {
            if(dir ==0 ||dir ==2)
            {

              if(Near_Box(box_id, dir, maduo) == 1) //如果旁边的纸垛有一层箱子
              {
                Lift_PosCtrl(8.7);//升到车三号位离开高度
                End_Rotation_Ctrl(0);
              if((car_box[1] ==1||car_box[2] == 1)&&(car_box[4] ==0&&car_box[5]==0))
              {
                  Lift_PosCtrl(11.8);
                  osDelay(200);
              }
              else if(car_box[4] ==1||car_box[5] == 1)
              {
                  Lift_PosCtrl(14.7);
                  osDelay(400);

              }
                osDelay(700);

                Scara_PosCtrl(206,-26);
                osDelay(200);//往后退一点

                Scara_PosCtrl(130,50);//正工作空间衔接位
                osDelay(320);

              }
              else if(Near_Box(box_id, dir, maduo) ==2 ) //如果旁边的纸垛有两层箱子
              {
                Lift_PosCtrl(11);//升到车三号位离开高度
                End_Rotation_Ctrl(0);
              if((car_box[1] ==1||car_box[2] == 1)&&(car_box[4] ==0&&car_box[5]==0))
              {
                  Lift_PosCtrl(11.8);
                  osDelay(100);
              }
              else if(car_box[4] ==1||car_box[5] == 1)
              {
                  Lift_PosCtrl(14.7);
                  osDelay(300);

              }
                osDelay(800);

                Scara_PosCtrl(206,-26);
                osDelay(200);//往后退一点

                Scara_PosCtrl(130,50);//正工作空间衔接位
                osDelay(320);
 
              }
              else if(Near_Box(box_id, dir, maduo) == 0) //如果旁边的纸垛没有箱子
              {
                // Lift_PosCtrl(5.8);//升到车三号位离开高度
                Lift_PosCtrl(8.4);//升到车三号位离开高度

                End_Rotation_Ctrl(0);
              if((car_box[1] ==1||car_box[2] == 1)&&(car_box[4] ==0&&car_box[5]==0))
              {
                  Lift_PosCtrl(11.8);
                  osDelay(300);
              }
              else if(car_box[4] ==1||car_box[5] == 1)
              {
                  Lift_PosCtrl(14.7);
                  osDelay(500);

              }
                osDelay(600);
                Scara_PosCtrl(206,-26);
                osDelay(200);//往后退一点
                Scara_PosCtrl(130,50);//正工作空间衔接位
                osDelay(320);
              }
  
            }
            else if(dir ==1)
            {
              // Lift_PosCtrl(5.8);//升到车三号位离开高度
              Lift_PosCtrl(8.4);//升到车三号位离开高度

              End_Rotation_Ctrl(0);
              if((car_box[1] ==1||car_box[2] == 1)&&(car_box[4] ==0&&car_box[5]==0))
              {
                  Lift_PosCtrl(11.8);
                  osDelay(300);
              }
              else if(car_box[4] ==1||car_box[5] == 1)
              {
                  Lift_PosCtrl(14.7);
                  osDelay(500);

              }
              osDelay(600);
              Scara_PosCtrl(206,-26);
              osDelay(200);//往后退一点
              Scara_PosCtrl(130,50);//正工作空间衔接位
              osDelay(150);
              if(!maduo)
              {
                Lift_PosCtrl(7);
              }
              osDelay(170);



            }

          }





          break;

          case 4:
            // Scara_PosCtrl(205,-115);//收到车四号位
            // Grab_Off();
            // End_Rotation_Ctrl(0);
            // osDelay(530);

            // Lift_PosCtrl(10);//下降到四号位一层抓取高度
            // osDelay(300);

            // Grab_On();
            // osDelay(400);
            if(car_box[5] == 0 && car_box[6] == 0 ) //如果车仓5和6空了
            {
              Lift_PosCtrl(10.9);//升到四号位直接离开高度
              osDelay(300);

              Scara_PosCtrl(210,-30);//负工作空间过度点
              osDelay(600);

              Scara_PosCtrl(130,50);//正工作空间衔接位
              osDelay(200);
            }
            else if(car_box[5] == 0 &&car_box[6] ==1) //如果车仓5空了
            {
              Lift_PosCtrl(10.9);//升到四号位直接离开高度
              osDelay(300);

              Scara_PosCtrl(210,-30);//负工作空间过度点
              osDelay(600);

              Scara_PosCtrl(130,50);//正工作空间衔接位
              osDelay(430);
              Lift_PosCtrl(9);//下降到衔接高度

            }
            else if(car_box[6] == 0 && car_box[5] == 1) //如果车仓6空了
            {
              Lift_PosCtrl(14.7);//升到四号位直接离开高度
              osDelay(500);

              Scara_PosCtrl(210,-30);//负工作空间过度点
              osDelay(600);

              Scara_PosCtrl(130,50);//正工作空间衔接位
              osDelay(50);
              Lift_PosCtrl(9);//下降到衔接高度

            }
            else
            {
              Lift_PosCtrl(14.7);//升到四号位跨过六号箱子离开高度
              osDelay(500);

              Scara_PosCtrl(210,-30);//负工作空间过度点
              osDelay(600);

              Scara_PosCtrl(130,50);//正工作空间衔接位
              osDelay(200);
              Lift_PosCtrl(9);//下降到衔接高度

            }




            

            Lift_PosCtrl(9);//下降到衔接高度
            // osDelay(500);
              
            break;

            case 5:
            // Scara_PosCtrl(281,-30);//收到车五号位
            // Scara_PosCtrl(278,-30);
            // Grab_Off();
            // End_Rotation_Ctrl(0);
            // osDelay(530);

            // Lift_PosCtrl(10);//下降到五号位一层抓取高度
            // osDelay(300);

            // Grab_On();
            // osDelay(400);

            if(car_box[4] == 0 && car_box[6] == 0 ) //如果车仓4和6空了
            {
              Lift_PosCtrl(10.9);//升到五号位直接离开高度
              osDelay(300);

              Scara_PosCtrl(210,-30);//负工作空间过度点
              osDelay(600);

              Scara_PosCtrl(130,50);//正工作空间衔接位
              osDelay(200);
            }
            else if(car_box[4] == 0 && car_box[6] == 1) //如果车仓4空了
            {
              Lift_PosCtrl(10.9);//升到五号位直接离开高度
              osDelay(300);

              Scara_PosCtrl(210,-30);//负工作空间过度点
              osDelay(600);

              Scara_PosCtrl(130,50);//正工作空间衔接位
              osDelay(430);
              Lift_PosCtrl(9);//下降到衔接高度

            }
            else if(car_box[6] == 0 && car_box[4] == 1) //如果车仓6空了
            {
              Lift_PosCtrl(14.7);//升到四号位直接离开高度
              osDelay(500);

              Scara_PosCtrl(210,-30);//负工作空间过度点
              osDelay(600);

              Scara_PosCtrl(130,50);//正工作空间衔接位
              osDelay(50);
              Lift_PosCtrl(9);//下降到衔接高度

            }
            else
            {
              Lift_PosCtrl(14.7);//升到四号位跨过六号箱子离开高度
              osDelay(500);

              Scara_PosCtrl(210,-30);//负工作空间过度点
              osDelay(600);

              Scara_PosCtrl(130,50);//正工作空间衔接位
              osDelay(200);
              Lift_PosCtrl(9);//下降到衔接高度

            }
  


              
            break;

            case 6:
              // Scara_PosCtrl(192,-12);//收到车六号位
              // Grab_Off();
              // End_Rotation_Ctrl(90);
              // osDelay(200);

              // Lift_PosCtrl(6.5);//下降到六号位高度
              // osDelay(500);

              // Grab_On();
              // osDelay(500);
              
              if(maduo)
              {
                // Lift_PosCtrl(8.6);//升到车六号位离开高度
                Lift_PosCtrl(11.4);//升到车六号位离开高度
                End_Rotation_Ctrl(0);

                if(car_box[4]==1 || car_box[5]==1 )
                {
                  Lift_PosCtrl(14.7);//升到车六号位离开高度
                  osDelay(270);
                }

                osDelay(600);
              }
              else
              {
                // Lift_PosCtrl(8.5);//升到六号位离开高度
                Lift_PosCtrl(11.4);//升到车六号位离开高度
                End_Rotation_Ctrl(0);
                if(car_box[4]==1 || car_box[5]==1 )
                {
                  Lift_PosCtrl(14.7);//升到车六号位离开高度
                  osDelay(270);
                }
                osDelay(600);
              }

                Scara_PosCtrl(201,-21);//向后退一步
                osDelay(150);

              Scara_PosCtrl(130,50);//正工作空间衔接位
              //  HAL_TIM_Base_Stop_IT(&htim5);

              // set_scara_position(130,50);
              // End_Rotation_Ctrl(0);

//
              osDelay(140);  
              if(dir == 1)
              {
                if(maduo)
                {
                Lift_PosCtrl(8.5);

                }
                else
                {
                Lift_PosCtrl(4.1);

                }
              }
              osDelay(260);  
 
              break;


            default:
              break;
            
            
    }
    switch(dir)
    {
        case 0:
          End_Rotation_Ctrl(-1.5);
          if(box_id == 1||box_id == 2)
          {
            osDelay(500);
          }
          else if(box_id == 6)
          {
            osDelay(50);
          }
          else if(box_id == 4 || box_id == 5)
          {
            if((car_box[4] == 0 || car_box[5] == 0) && (car_box[6] == 1)) //如果车仓4或5空了
            {
              osDelay(50);
            }
            else if((car_box[4] == 0 || car_box[5] == 0) && (car_box[6] == 0)) //如果车仓4或5空了
            {
              osDelay(150);
            }
            else if(car_box[6] == 0)
            {
              osDelay(390);
            }
            else if(car_box[6] == 1)
            {
              osDelay(250);
            }

          }
          else if(box_id == 3)
          {
            // osDelay(200);
            // End_Rotation_Ctrl(0);

          }
          Scara_PosCtrl(178,137);//左边纸垛


          if(maduo)
          {
            osDelay(150);
            Lift_PosCtrl(7.7);//降到码垛高度
            if(box_id == 3 )
            {
              osDelay(490);
            }
            else if(box_id == 6)
            {
              osDelay(620);
            }

            else if(box_id == 4 ||box_id ==5)
            {

              if(car_box[4] == 0 || car_box[5] == 0) //如果车仓5或4空了
              {
                osDelay(400);
              }
              else
              {
                osDelay(460);
              }
            }
            else if(box_id == 1 || box_id == 2)
            {
              osDelay(380);
            }
          }
          else
          {
            if(box_id == 4||box_id == 5)
            {
              if(car_box[4] == 0 || car_box[5] == 0) //如果车仓4或5空了
              {
                osDelay(200);
              }
              else
              {
                osDelay(200);
              }
              
            }
            else if(box_id == 1||box_id == 2)
            {
              osDelay(250);
            }
            else if(box_id == 3)
            {
              if(Near_Box(box_id, dir, maduo) ==2)
              {
                osDelay(180);
              }
              else
              {
              osDelay(300);

              }

            }
            else if(box_id == 6)
            {
              osDelay(50);
            }
            Lift_PosCtrl(4.1);//降到纸垛高度
            if(box_id == 1 || box_id == 2)
            {
              osDelay(680);
            }
            else if(box_id == 4 || box_id == 5)
            {
              osDelay(740);
            }
            else if (box_id == 3)
            {
              if(Near_Box(box_id, dir, maduo) ==2)
              {
                osDelay(740);
              }
              else
              {
              osDelay(740);

              }
            }
            
            else if(box_id == 6)
            {

              osDelay(850);
            }

          }
          End_Rotation_Ctrl(1);
          Grab_Off();
          osDelay(200);
          if((car_box[4]==0 && car_box[5]==0 && box_id ==6) || (car_box[4]==0 && car_box[6]==0 && box_id ==5) || (car_box[5]==0 && car_box[6]==0 && box_id ==4))
          {
            Lift_PosCtrl(8.7);//降到抓下面一层的高度
            osDelay(180);
          }
          else
          {
            Lift_PosCtrl(12);//升到放完高度
            osDelay(180);
          }
          scara.step_angle += 4;
          Scara_PosCtrl(130,50);//正工作空间衔接位
          osDelay(350);

          osSemaphoreRelease(ChassisMoveDoneHandle);
          Grab_On();
          if(car_box[6] == 1 && box_id != 6)
          {
            osDelay(300);
          }
          scara.step_angle -= 4;
          Scara_PosCtrl(195,-15);//回到车内负工作空间过度点
          osDelay(440);

          // set_scara_position(205,-25);
        break;

        case 1:
          End_Rotation_Ctrl(-1.5);
          Scara_PosCtrl(130,50);//中间纸垛
          // osDelay(1000);

          if(maduo)
          {
            Lift_PosCtrl(7.7);//降到码垛高度
            if(box_id == 3 )
            {
              if((car_box[4]==1)||(car_box[5]==1))
              {
                osDelay(190);
              }
              osDelay(450);
            }
            else if(box_id == 6)
            {
              if((car_box[4]==1)||(car_box[5]==1))
              {
                osDelay(190);
              }
              osDelay(500);
            }

            else if(box_id == 4 ||box_id ==5)
            {

              if(car_box[4] == 0 || car_box[5] == 0) //如果车仓5或4空了
              {
                osDelay(600);
              }
              else
              {
                osDelay(600);
              }
            }
            else if(box_id == 1 || box_id == 2)
            {
              osDelay(800);
            }


          }
          else
          {
            if(box_id == 3)
            {
              osDelay(250);
            }
            else if(box_id == 6)
            {
              // osDelay(100);
            }
            Lift_PosCtrl(4.1);//降到纸垛高度
            if(box_id == 3)
            {
              if((car_box[4]==1)||(car_box[5]==1))
              {
                osDelay(100);
              }
              osDelay(700);
            }
            else if(box_id == 6)
            {
              if((car_box[4]==1)||(car_box[5]==1))
              {
                osDelay(200);
              }
              osDelay(460);
            }

            else if(box_id == 1 || box_id == 2)
            {
              osDelay(720);
            }

            else if(box_id == 4 ||box_id ==5)
            {

              if(car_box[4] == 0 || car_box[5] == 0) //如果车仓4或5空了
              {
                osDelay(680);
              }
              else
              {
                osDelay(980);
              }
            }
          }

          Grab_Off();
          osDelay(200);
          if((car_box[4]==0 && car_box[5]==0 && box_id ==6) || (car_box[4]==0 && car_box[6]==0 && box_id ==5) || (car_box[5]==0 && car_box[6]==0 && box_id ==4))
          {
            Lift_PosCtrl(8.7);//降到抓下面一层的高度
            osDelay(180);
          }
          else
          {
            Lift_PosCtrl(12);//升到放完高度
            osDelay(180);
          }
          if(car_box[6] == 1 && box_id != 6)
          {
            osDelay(300);
          }
          osSemaphoreRelease(ChassisMoveDoneHandle);
          // scara.step_angle += 3;
          Grab_On();
          Scara_PosCtrl(195,-15);//回到车内负工作空间过度点
          osDelay(440);
          // scara.step_angle -= 3;




        break;
        case 2:
          End_Rotation_Ctrl(-1.5);
          if(box_id == 1||box_id == 2)
          {
            osDelay(500);
          }
          else if(box_id == 6)
          {
            osDelay(50);
          }
          else if(box_id == 4 || box_id == 5)
          {
            if((car_box[4] == 0 || car_box[5] == 0) && (car_box[6] == 1)) //如果车仓4或5空了
            {
              osDelay(50);
            }
            else if((car_box[4] == 0 || car_box[5] == 0) && (car_box[6] == 0)) //如果车仓4或5空了
            {
              osDelay(150);
            }
            else if(car_box[6] == 0)
            {
              osDelay(390);
            }
            else if(car_box[6] == 1)
            {
              osDelay(250);
            }

          }
          else if(box_id == 3)
          {
            osDelay(50);
          }
          Scara_PosCtrl(40,1.5);//右边纸垛


          if(maduo)
          {
            osDelay(150);
            Lift_PosCtrl(7.7);//降到码垛高度
            if(box_id == 3 )
            {
              osDelay(490);
            }
            else if(box_id == 6)
            {
              osDelay(620);
            }

            else if(box_id == 4 ||box_id ==5)
            {

              if(car_box[4] == 0 || car_box[5] == 0) //如果车仓5或4空了
              {
                osDelay(400);
              }
              else
              {
                osDelay(460);
              }
            }
            else if(box_id == 1 || box_id == 2)
            {
              osDelay(350);
            }
          }
          else
          {
            if(box_id == 4||box_id == 5)
            {
              if(car_box[4] == 0 || car_box[5] == 0) //如果车仓4或5空了
              {
                osDelay(200);
                
              }
              else
              {
                osDelay(200);
              }
              
            }
            else if(box_id ==6)
            {
              osDelay(50);

            }
            else if(box_id == 1||box_id == 2)
            {
              osDelay(250);
            }
            else if(box_id == 3)
            {
              if(Near_Box(box_id, dir, maduo) ==2)
              {
                osDelay(180);
              }
              else
              {
              osDelay(350);

              }

            }
            Lift_PosCtrl(4.1);//降到纸垛高度
            if(box_id == 1 || box_id == 2)
            {
              osDelay(630);
            }
            else if(box_id == 4 || box_id == 5)
            {
              osDelay(770);
            }
            else if (box_id == 3)
            {
              if(Near_Box(box_id, dir, maduo) ==2)
              {
                osDelay(740);
              }
              else
              {
              osDelay(740);

              }
            }
            
            else if(box_id == 6)
            {

              osDelay(850);
            }

          }

          Grab_Off();
          osDelay(200);
          if((car_box[4]==0 && car_box[5]==0 && box_id ==6) || (car_box[4]==0 && car_box[6]==0 && box_id ==5) || (car_box[5]==0 && car_box[6]==0 && box_id ==4))
          {
            Lift_PosCtrl(8.7);//降到抓下面一层的高度
            osDelay(180);
          }
          else
          {
            Lift_PosCtrl(12);//升到放完高度
            osDelay(180);
          }
          scara.step_angle += 4;
          Scara_PosCtrl(130,50);//正工作空间衔接位
          osDelay(350);
          osSemaphoreRelease(ChassisMoveDoneHandle);
          Grab_On();
          if(car_box[6] == 1 && box_id != 6)
          {
            osDelay(300);
          }

          scara.step_angle -= 4;
          Scara_PosCtrl(195,-15);//回到车内负工作空间过度点
          osDelay(440);






        break;
        default:
        break;
    }
    // HAL_TIM_Base_Stop_IT(&htim5);

    car_box[box_id] = 0; //将车仓对应位置的箱子标记为无
    //将纸箱对应位置的箱子标记为有
    if(raspi.paper_id[put_round-1] ==1)
    {
      if(maduo)
      {
        paper_box[1] = 2; //如果是码垛，标记为两层
      }
      else
      {
        paper_box[1] = 1; //如果是单放，标记为一层
      }
    }
    else if(raspi.paper_id[put_round-1] ==2)
    {
      if(dir ==0)
      {
        if(maduo)
        {
          paper_box[2] = 2; //如果是码垛，标记为两层
        }
        else
        {
          paper_box[2] = 1; //如果是单放，标记为一层
        }
      }
      else if(dir ==2)
      {
        if(maduo)
        {
          paper_box[3] = 2; //如果是码垛，标记为两层
        }
        else
        {
          paper_box[3] = 1; //如果是单放，标记为一层
        }
      }

    }
    else if(raspi.paper_id[put_round-1] ==3)
    {
      if(dir ==0)
      {
        if(maduo)
        {
          paper_box[4] = 2; //如果是码垛，标记为两层
        }
        else
        {
          paper_box[4] = 1; //如果是单放，标记为一层
        }
      }
      else if(dir ==2)
      {
        if(maduo)
        {
          paper_box[5] = 2; //如果是码垛，标记为两层
        }
        else
        {
          paper_box[5] = 1; //如果是单放，标记为一层
        }
      }
    }
    else if(raspi.paper_id[put_round-1] ==4)
    {
      if(maduo)
      {
        paper_box[6] = 2; //如果是码垛，标记为两层
      }
      else
      {
        paper_box[6] = 1; //如果是单放，标记为一层
      }
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

      Lift_PosCtrl(6.7);//下降到一号位一层抓取高度
      osDelay(400);

      Grab_On();
      osDelay(500);
      break;
    case 2:
      Scara_PosCtrl(278,-30);//收到车二号位
      Grab_Off();
      End_Rotation_Ctrl(0);
      osDelay(390);

      Lift_PosCtrl(6.7);//下降到二号位一层抓取高度
      osDelay(400);

      Grab_On();
      osDelay(500);
      break;
    case 3:
      Scara_PosCtrl(192,-12);//收到车三号位
      Grab_Off();
      End_Rotation_Ctrl(90);
      osDelay(150);

      Lift_PosCtrl(3);//下降到三号位一层抓取高度
      osDelay(700);

      Grab_On();
      osDelay(400);
      
      break;
    case 4:
      Scara_PosCtrl(205,-115);//收到车四号位
      Grab_Off();
      End_Rotation_Ctrl(0);
      osDelay(530);

      Lift_PosCtrl(10);//下降到四号位一层抓取高度
      osDelay(330);

      Grab_On();
      osDelay(400);
      break;
    case 5:
      Scara_PosCtrl(278,-30);
      Grab_Off();
      End_Rotation_Ctrl(0);
      osDelay(530);

      Lift_PosCtrl(10);//下降到五号位一层抓取高度
      osDelay(330);

      Grab_On();
      osDelay(400);
      break;
    case 6:
      Scara_PosCtrl(192,-12);//收到车六号位
      Grab_Off();
      End_Rotation_Ctrl(90);
      osDelay(200);

      Lift_PosCtrl(6.4);//下降到六号位高度
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

void Move_Translation_NonBlocking(float target_x, float target_y, float target_z, uint32_t timeout)
{
  while(chassisState.moving == 1&& chassisState.done == 0) // 等待底盘移动完成
  {
    osDelay(10);
  }
  chassisState.target_x = target_x;
  chassisState.target_y = target_y;
  chassisState.target_z = target_z;
  chassisState.move_type = MOVE_TYPE_TRANSLATION;
  chassisState.timeout = timeout;
  chassisState.moving = 1;
  chassisState.done = 0;

  osSemaphoreAcquire(ChassisMoveDoneHandle, 0);
}


// 定义不同目标点的位置
float target_positions[16][3] = {
    {1785, 1010, 0},
    {2785, 500, 0},
    {3535, 500, 0},
    {3280, 500, 0},
    {3280, 1000, 0},
    {3280, 1500, 0},
    {3535, 1500, 0},
    {3000, 1500, 0},
    {785, 500, -180},
    {440, 445, -90},
    {445, 600, -180},
    {445,1000,-180},
    {445,1400,-180},
    {440,1555,-270},
    {785,500,-180},
    {785,1500,-180}//初始生成的随机点
};

int timeout[16][16] = {
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


void Move_To_Target(uint8_t target_id){
  float target_x, target_y, target_z;
  target_x = target_positions[target_id-1][0];
  target_y = target_positions[target_id-1][1];
  target_z = target_positions[target_id-1][2];

  float begin_x = target_positions[last_target_index][0];
  float begin_y = target_positions[last_target_index][1];

  float dis_x = target_x - begin_x;
  float dis_y = target_y - begin_y;



  Move_To_Position_XYZ_NonBlocking(dis_x, dis_y, target_z, timeout[last_target_index][target_id-1]);
  last_target_index = target_id-1;
}

void Move_To_Placing_Box(uint8_t* box_ids){
  //定义放置盒子从上向下分别为1，2，3, 4, 5
  Move_To_Target(box_ids[0]+9);//box_id+9为箱子放置位置的id
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  if(box_ids[0]==1){
    switch(box_ids[1]){
      case 2:
      case 3:{
        Move_To_Target(15);
        break;
      }
      case 4:{
        Move_To_Target(16);
        break;
      default:
        break;
      }
    }//如果第一个盒子是1号位，则需要进入中转点
    osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
  }

  Move_To_Target(box_ids[1]+9);//前往第二个箱子的位置
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);

  if(box_ids[2]==5){
    Move_To_Target(16);
    osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
    //如果最后一个盒子是5号位，也需要进入中转点
  }

  Move_To_Target(box_ids[2]+9);//前往第三个箱子的位置
  osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
}

uint8_t Near_Box(uint8_t box_id ,uint8_t dir, bool maduo)
  {
    switch (raspi.paper_id[put_round-1])
    {

    case 2:
      if(dir == 0)
      {
        return paper_box[3];
      }
      else if(dir == 2)
      {
        return paper_box[2];
      }
      else
      {
        return 0; // dir == 1 或其他值时返回0
      }
      break;
    case 3:
      if(dir == 0)
      {
        return paper_box[5];
      }
      else if(dir == 2)
      {
        return paper_box[4];
      }
      else
      {
        return 0; // dir == 1 或其他值时返回0
      }
      break;
    default:
      return 0; //如果不是2或3号纸箱，返回0
      break;
    }
  }

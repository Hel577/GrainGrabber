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




//开局移动到货架
void Move_To_Shelf(void)
{
  // Move_To_Position_XYZ_NonBlocking(-580,2160,0,3300);
  Move_To_Position_XYZ_NonBlocking(600,600,0,5000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
}

//货架区左移一格
void Move_Shelf_Left(void)
{
  Move_Translation_NonBlocking(-510, 0, 0, 2500);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
}

//货架区右移一格
void Move_Shelf_Right(void)
{
  Move_Translation_NonBlocking(510, 0, 0, 2500);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
}

//从货架移动到纸垛
void Move_To_Paper(void)
{
  Move_To_Position_XYZ_NonBlocking(-500, -2320, 180, 4000);
  // osSemaphoreAcquire(ChassisMoveDoneHandle, osWaitForever);
}

//纸垛区点到点移动                   y
//                          p2      ^ p3          
//                 p1            p0 *            p4
//                                 -*-------> x
void Move_point_to_point(uint8_t point_1, uint8_t point_2)
{

    
    // 定义不同点之间的超时时间（毫秒）
    uint32_t timeout[7][7] = {
        {    0, 5400, 4800, 4800, 4300, 5000, 6000 }, // 从点0到各点
        { 3000,    0, 3500, 4000, 3600, 1000, 4500 }, // 从点1到各点
        { 4000, 3000,    0, 3000, 3500, 1200, 3000 }, // 从点2到各点
        { 4000, 4000, 3000,    0, 2500, 5000, 3000 }, // 从点3到各点
        { 3000, 4000, 3500, 3000,    0, 3000, 1200 }, // 从点4到各点
        { 2000, 1000, 1600, 5000, 3000,    0, 4000 }, // 从点5到各点
        { 2000, 4000, 3000, 1200, 3000, 4000,    0 }  // 从点6到各点
    };
    
    // 如果起点和终点相同，不需要移动
    if (point_1 == point_2) return;
    
    // 如果点不在有效范围内，返回
    if (point_1 > 6 || point_2 > 6) return;
    
    // 特殊路径处理
    if ((point_1 == 1 && point_2 == 2) || (point_1 == 2 && point_2 == 1)) {
        // 1号和2号点之间需要经过5号点
        if (point_1 == 1) {
            // 从1到2
            // Move_point_to_point_direct(point_1, point_2, timeout[point_1][2]);
            // Move_point_to_point_direct(point_1, 5, timeout[point_1][5]);
            // Move_point_to_point_direct(5, point_2, timeout[5][point_2]);
            Move_point_to_point_direct(point_1, 5, timeout[point_1][5]);
            Move_point_to_point_direct(5, point_2, timeout[5][point_2]);



            // 从5
        } else {
            // 从2到5
            Move_point_to_point_direct(point_1, 5, timeout[point_1][5]);
            // 从5到1
            Move_point_to_point_direct(5, point_2, timeout[5][point_2]);
        }
        return;
    }
    
    if ((point_1 == 3 && point_2 == 4) || (point_1 == 4 && point_2 == 3)) {
        // 3号和4号点之间需要经过6号点
        if (point_1 == 3) {
            // 从3到6
            // Move_point_to_point_direct(point_1, 6, timeout[point_1][6]);
            // // 从6到4
            // Move_point_to_point_direct(6, point_2, timeout[6][point_2]);
            Move_point_to_point_direct(point_1, point_2, timeout[point_1][point_2]);

        } else {
            // 从4到6
            Move_point_to_point_direct(point_1, 6, timeout[point_1][6]);
            // 从6到3
            Move_point_to_point_direct(6, point_2, timeout[6][point_2]);
        }
        return;
    }
    
    // 其他点之间可以直接移动
    Move_point_to_point_direct(point_1, point_2, timeout[point_1][point_2]);
}


// 直接从一个点移动到另一个点，不考虑路径规划
void Move_point_to_point_direct(uint8_t point_1, uint8_t point_2, uint32_t timeout_ms)
{
    // 定义各点姿态 [x, y, 角度]
    float point_coords[7][3] = {
        { -540,-2240, 0},      // 点0 (中心点)
        {-386, 380, 270}, // 点1 
        {-380, 650, 180},  // 点2 
        { 430, 650, 180}, // 点3 
        { 380, 380, 90}, // 点4 
        {-180, 450, 225}, // 点5 (1和2之间的中转点)
        { 200, 480, 135},  // 点6 (3和4之间的中转点)
        // { 200, 400, 180}  // 点7 (3和4之间的中转点)
    };
    
    // 如果起点和终点相同，不需要移动
    if (point_1 == point_2) return;
    
    // 如果点不在有效范围内，返回
    if (point_1 > 6 || point_2 > 6) return;
    
    // 获取起点和终点坐标与姿态
    float start_x = point_coords[point_1][0];
    float start_y = point_coords[point_1][1];
    
    float end_x = point_coords[point_2][0];
    float end_y = point_coords[point_2][1];
    float end_angle = point_coords[point_2][2];
    if((point_1 ==2)&&(point_2 ==4))
  {
    end_x += 180;
  }
  else if((point_1 ==2)&&(point_2 == 3))
  {
    end_x += 100;
  }  
  else if((point_1 ==3)&&(point_2 == 2))
  {
    end_x -= 100;
  }    
  else if((point_1 ==4)&&(point_2 ==2))
  {
    end_x -= 120;
    end_y -= 68;
  }
  else if((point_1 ==3)&&(point_2 ==1))
  {
    end_x -= 180;
  }
    else if((point_1 ==1)&&(point_2 ==3))
  {
    end_x += 120;
    end_y -=30;
  }
    else if((point_1 ==1)&&(point_2 ==4))
  {
    end_x += 170;
  }
    else if((point_1 ==4)&&(point_2 ==1))
  {
    end_x -= 180;
  }
      else if((point_1 ==5)&&(point_2 ==2))
  {
    end_x += 78;
  }
      else if((point_1 ==6)&&(point_2 ==3))
  {
    end_x -= 120;
  }
      else if((point_1 ==0)&&(point_2 ==1))
  {
    end_x += 23;
    end_y += 40;
  }
      else if((point_1 ==0)&&(point_2 ==2))
  {
    end_x += 43;
    end_y += 13;
  }
      else if((point_1 ==0)&&(point_2 ==3))
  {
    end_x += 15;
  }
      else if((point_1 ==0)&&(point_2 ==4))
  {
    end_x += 100;
    end_y += 40;
  }
  


    // 计算全局坐标系中的姿态差
    float delta_x = end_x - start_x;
    float delta_y = end_y - start_y;
 
    
    // 将全局坐标系中的位移转换到车身坐标系
    float car_delta_x = -delta_x;
    float car_delta_y = -delta_y;

    // 使用车身坐标系的相对位置进行移动
    if((fabs(car_delta_x)>=2000 ||fabs(car_delta_y)>=2000))
    {
      Move_To_Position_XYZ_NonBlocking(car_delta_x, car_delta_y, end_angle, timeout_ms);
    }
    else
    {
      // Move_By_Easy(car_delta_x, car_delta_y, end_angle, timeout_ms);
      Move_To_Position_XYZ_NonBlocking(car_delta_x, car_delta_y, end_angle, timeout_ms);

    }
}



void Move_To_Next_Paper(void)
{
  if(put_round == 1)
  {
    Move_point_to_point(0, raspi.paper_id[0]);
    move_flag = 1;
  }
  else if(put_round >=2)
  {
    if(raspi.paper_id[put_round-1] != raspi.paper_id[put_round-2])
    { 
      Move_point_to_point(raspi.paper_id[put_round-2], raspi.paper_id[put_round-1]);
      move_flag = 1;
    }
    else
    {
      move_flag = 0;
    }
  }
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

#include "wheel_control.h"
#include "usart.h"  // 引入串口头文件
#include <math.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "toolfunc.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEG_TO_RAD  M_PI / 180.0
#define rx 81
#define ry 147

const double pi;
//以下关于运动的尺寸单位为cm
const double half_car_width;
const double half_car_length;
const double wheel_r;
float MOTOR_R=50;           //这个是麦轮的半径，单位为毫米

//整个坐标系竖直向上是x正方向，水平向左是y正方向，逆时针为w正方向
//最后一个参数是轮子中心到底盘中心的rx+ry
float cal_matrix[4][3]={
        {1.0,-1.0,-(rx+ry)},
        {1.0,1.0,(rx+ry)},
        {1.0,1.0,-(rx+ry)},
        {1.0,-1.0,(rx+ry)} };
float motor_fai[4]={0};
float car_distance[3]={0};
float motor_speed[4]={0};
extern uint8_t camdata[8];
uint8_t if_minus[4];
float car_adjust[3]={0};



void cal_motor(float* motor_fai_, float* car_distance_ , uint8_t* if_minus)//注意这里是角速度而不是线速度
{
    motor_fai_[0]=(cal_matrix[0][0]*car_distance_[0] + cal_matrix[0][1]*car_distance_[1] + cal_matrix[0][2]*car_distance_[2])/MOTOR_R;
    motor_fai_[1]=(cal_matrix[1][0]*car_distance_[0] + cal_matrix[1][1]*car_distance_[1] + cal_matrix[1][2]*car_distance_[2])/MOTOR_R;
    motor_fai_[2]=(cal_matrix[2][0]*car_distance_[0] + cal_matrix[2][1]*car_distance_[1] + cal_matrix[2][2]*car_distance_[2])/MOTOR_R;
    motor_fai_[3]=(cal_matrix[3][0]*car_distance_[0] + cal_matrix[3][1]*car_distance_[1] + cal_matrix[3][2]*car_distance_[2])/MOTOR_R;
    //printf("%f,%f,%f,%f\n",motor_w_[0]*2*3.1415/5.5*1000.0,motor_w_[1]*2*3.1415/5.5*1000.0,motor_w_[2]*2*3.1415/5.5*1000.0,motor_w_[3]*2*3.1415/5.5*1000.0);

	for(int i = 0 ; i < 4 ; i++)
	{
		if(motor_fai_[i]<0)
		{
			motor_fai_[i]*=-1;
			if(if_minus[i]==0)
			{
				if_minus[i]=1;
			}
			else
			{
				if_minus[i]=0;
			}
		}
	}
}


void speed_cal(float* motor_speed_, float* car_speed_ , uint8_t* if_minus)//注意这里是角速度而不是线速度
{
    motor_speed_[0]=(cal_matrix[0][0]*car_speed_[0] + cal_matrix[0][1]*car_speed_[1] + cal_matrix[0][2]*car_speed_[2])/MOTOR_R;
    motor_speed_[1]=(cal_matrix[1][0]*car_speed_[0] + cal_matrix[1][1]*car_speed_[1] + cal_matrix[1][2]*car_speed_[2])/MOTOR_R;
    motor_speed_[2]=(cal_matrix[2][0]*car_speed_[0] + cal_matrix[2][1]*car_speed_[1] + cal_matrix[2][2]*car_speed_[2])/MOTOR_R;
    motor_speed_[3]=(cal_matrix[3][0]*car_speed_[0] + cal_matrix[3][1]*car_speed_[1] + cal_matrix[3][2]*car_speed_[2])/MOTOR_R;
    //printf("%f,%f,%f,%f\n",motor_w_[0]*2*3.1415/5.5*1000.0,motor_w_[1]*2*3.1415/5.5*1000.0,motor_w_[2]*2*3.1415/5.5*1000.0,motor_w_[3]*2*3.1415/5.5*1000.0);
	
	for(int i = 0 ; i < 4 ; i++)
	{
		if(motor_speed_[i]<0)
		{
			motor_speed_[i]*=-1;
			if(if_minus[i]==0)
			{
				if_minus[i]=1;
			}
			else
			{
				if_minus[i]=0;
			}
		}
	}
}


// x+ is left (mm)  y+ is forward(mm)  angle+ is clockwise(du)  v is speed(……circle per minute)
void move_to_target(float x_target, float y_target, float angle_target, int v)
{
	// 用于存储电机实时位置（单位：角度）
	//float motor_current_pos[4] = {0}; 
	//float motor_target_pos[4] = {0};
    car_distance[0] = x_target;
    car_distance[1] = y_target;
    car_distance[2] = angle_target * pi / 180; // 转换为弧度

    uint8_t if_minus[4] = {0, 1, 0, 1};
    // int address[4] = {0x01, 0x02, 0x03, 0x04};	
	
    // 逆运动学计算电机目标角度
    cal_motor(motor_fai, car_distance, if_minus);
		
    for (int i = 0; i < 4; i++)
    {
        // 发送目标位置命令
        // Wheel_Pos_Control(address[i], if_minus[i], v, 181, 3200 * motor_fai[i] / (2 * pi), 0, 0);
        osDelay(250);
    }

}


//use speed to control the position      forward is 0 degree  clockwise is -   Counterclockwise is +     w_z  Counterclockwise is +
// 修改后的set_speed函数，增加speed_magnitude参数
void set_speed_pid(float angle_target, float speed_magnitude, float w_z)  // 增加w_z参数
{
    // 限制角度范围在 -180° 到 +180°
    if (angle_target > 180.0f) {
        angle_target -= 360.0f;
    }
    else if (angle_target < -180.0f) {
        angle_target += 360.0f;
    }

    // 将角度从度转换为弧度
    float angle_radians = angle_target * pi / 180.0f;

    // 使用传入的速度幅值，并添加旋转分量
    car_adjust[0] = speed_magnitude * cos(angle_radians);
    car_adjust[1] = speed_magnitude * sin(angle_radians);
    car_adjust[2] = w_z;  // 原固定值0改为旋转角速度参数

    // 电机方向和地址
    uint8_t if_minus[4] = { 0, 1, 0, 1 };
    // int address[4] = { 0x01, 0x02, 0x03, 0x04 };

    // 逆运动学计算电机转动速度（需确保speed_cal支持旋转分量）
    speed_cal(motor_speed, car_adjust, if_minus);

    for (int i = 0; i < 4; i++) {
        // Wheel_Vel_Control(address[i], if_minus[i], motor_speed[i] * 60 / 2 / pi, 181, 0);
        osDelay(250);
    }
}



void stop(void)
{
	// int address[4] = {0x01, 0x02, 0x03, 0x04};
	for (int i = 0; i < 4; i++)
		{
			// Wheel_Vel_Control(address[i], 0, 0, 0, 0);
			osDelay(250);
		}
}


// // change the second and the third num
// void PID_x(void)    //forwards
// {
// 	Modify_PID_Params(0x01, 65000, 10, 26000);
// 	Modify_PID_Params(0x02, 90000, 10, 26000);
// 	Modify_PID_Params(0x03, 95000, 10, 26000);
// 	Modify_PID_Params(0x04, 60000, 10, 26000);
// }

// void PID_y(void)    //right
// {
// 	Modify_PID_Params(0x01, 75000, 10, 26000);
// 	Modify_PID_Params(0x02, 23000, 10, 26000);
// 	Modify_PID_Params(0x03, 22000, 10, 26000);
// 	Modify_PID_Params(0x04, 90000, 10, 26000);
// }

// void PID_w(void)    //cw
// {
// 	Modify_PID_Params(0x01, 52000, 10, 26000);
// 	Modify_PID_Params(0x02, 52000, 10, 26000);
// 	Modify_PID_Params(0x03, 15000, 10, 26000);
// 	Modify_PID_Params(0x04, 26000, 10, 26000);
// }


// void PID_Init(PID* pid, float Kp, float Ki, float Kd) {
//     pid->Kp = Kp;
//     pid->Ki = Ki;
//     pid->Kd = Kd;
//     pid->integral = 0;
//     pid->prev_error = 0;
// }


// void pid_init(void)
// {
//   PID_x();
//   PID_y();
//   PID_w();
//   PID_Init(&pos_pid, 0.5, 0.01, 0.1); // 位置PID参数
//   PID_Init(&angle_pid, 0.3, 0.005, 0.05);// 航向PID参数

// }

float linear_speed_PID_Update(PID* pid, float target, float current, float dt) {
    float error = target - current;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;
    return output;
}


float angular_speed_PID_Update(PID* pid, float target, float current, float dt) {
	
	float error = target - current;
	 // 限制角度范围在 -180° 到 +180°
    if (error > 180.0f) {
        error -= 360.0f;
    }
    else if (error < -180.0f) {
        error += 360.0f;
    }
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;
    return output;
}


void encodingdisk_control(float target_x, float target_y ,float target_angle)
{
	while(1)
	{
		if(isOPSRead)
		{
			isOPSRead = 0;
			// 计算位置误差
			float dx = target_x - ops_data.x;
			float dy = target_y - ops_data.y;
			float pos_error = sqrt(dx*dx + dy*dy);

			// calculate the angle 
            float angle_target = calculate_angle(dx, dy);
			
			// PID计算线速度和角速度
			float linear_speed = linear_speed_PID_Update(&pos_pid, 0, pos_error, 0.1);
			float angular_speed = angular_speed_PID_Update(&angle_pid, target_angle, ops_data.heading, 0.1);

			//函数控制
			set_speed_pid(angle_target, linear_speed, angular_speed);
		
		}
	}
}

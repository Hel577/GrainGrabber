#include "pid.h"

PIDController *rough_X_PID = NULL;
PIDController *rough_Y_PID = NULL;
PIDController *rough_Z_PID = NULL;
PIDController *fine_X_PID = NULL;
PIDController *fine_Y_PID = NULL;
PIDController *fine_Z_PID = NULL;
PIDController *Easy_X_PID = NULL;
PIDController *Easy_Y_PID = NULL;
PIDController *Easy_Z_PID = NULL;
PIDController *Translation_X_PID = NULL;
PIDController *Translation_Y_PID = NULL;
PIDController *Translation_Z_PID = NULL;

/*这里的pid都是在速度模式的基础上进行的*/

//初始化PID控制器
void Init_PID(void)
{
    rough_X_PID = (PIDController *)malloc(sizeof(PIDController)); 
    rough_Y_PID = (PIDController *)malloc(sizeof(PIDController));
    rough_Z_PID = (PIDController *)malloc(sizeof(PIDController));
    fine_X_PID = (PIDController *)malloc(sizeof(PIDController));
    fine_Y_PID = (PIDController *)malloc(sizeof(PIDController));
    fine_Z_PID = (PIDController *)malloc(sizeof(PIDController));
    Easy_X_PID = (PIDController *)malloc(sizeof(PIDController));
    Easy_Y_PID = (PIDController *)malloc(sizeof(PIDController));
    Easy_Z_PID = (PIDController *)malloc(sizeof(PIDController));
    Translation_X_PID = (PIDController *)malloc(sizeof(PIDController));
    Translation_Y_PID = (PIDController *)malloc(sizeof(PIDController));
    Translation_Z_PID = (PIDController *)malloc(sizeof(PIDController));

    //粗调X轴
    if(rough_X_PID !=NULL)
    {
        rough_X_PID->kp = 5.1f;
        rough_X_PID->ki = 0.01f;
        rough_X_PID->kd = 1.53f;
        rough_X_PID->dt = 0.018f;
        rough_X_PID->integral = 0.0f;
        rough_X_PID->prev_error = 0.0f;
        rough_X_PID->max_output = 1200.0f;
        rough_X_PID->max_accel = 1200.0f;
        rough_X_PID->last_call_time = HAL_GetTick();
        memset(rough_X_PID->deriv_buf, 0, sizeof(rough_X_PID->deriv_buf));
        rough_X_PID->last_output = 0.0f;
        rough_X_PID->prev_filtered = 0.0f;
        rough_X_PID->prev_position = 0.0f;
        rough_X_PID->first_call = true;          
    }

    //粗调Y轴
    if(rough_Y_PID !=NULL)
    {
        rough_Y_PID->kp = 5.18f;
        rough_Y_PID->ki = 0.01f;
        rough_Y_PID->kd = 1.71f;
        rough_Y_PID->dt = 0.018f;
        rough_Y_PID->integral = 0.0f;
        rough_Y_PID->prev_error = 0.0f;
        rough_Y_PID->max_output = 1200.0f;
        rough_Y_PID->max_accel = 1200.0f;
        rough_Y_PID->last_call_time = HAL_GetTick();
        memset(rough_Y_PID->deriv_buf, 0, sizeof(rough_Y_PID->deriv_buf));
        rough_Y_PID->last_output = 0.0f;
        rough_Y_PID->prev_filtered = 0.0f;    
        rough_Y_PID->prev_position = 0.0f;
        rough_Y_PID->first_call = true;
    }

    //粗调Z轴
    if(rough_Z_PID !=NULL)    
    {
        rough_Z_PID->kp = 7.3f;
        rough_Z_PID->ki = 0.001f;
        rough_Z_PID->kd = 0.646f;
        rough_Z_PID->dt = 0.018f;
        rough_Z_PID->integral = 0.0f;
        rough_Z_PID->prev_error = 0.0f;
        rough_Z_PID->max_output = 65.0f;
        rough_Z_PID->max_accel = 220.0f;
        rough_Z_PID->last_call_time = HAL_GetTick();
        memset(rough_Z_PID->deriv_buf, 0, sizeof(rough_Z_PID->deriv_buf));
        rough_Z_PID->last_output = 0.0f;
        rough_Z_PID->prev_filtered = 0.0f;
        rough_Z_PID->prev_position = 0.0f;
        rough_Z_PID->first_call = true;
    }

    //精调X轴
    if(fine_X_PID !=NULL)
    {   
        fine_X_PID->kp = 2.5f;
        fine_X_PID->ki = 0.001f;
        fine_X_PID->kd = 2.43f;
        fine_X_PID->dt = 0.018f;
        fine_X_PID->integral = 0.0f;
        fine_X_PID->prev_error = 0.0f;
        fine_X_PID->max_output = 160.0f; 
        fine_X_PID->max_accel = 520.0f;
        fine_X_PID->last_call_time = HAL_GetTick();
        fine_X_PID->prev_current = HAL_GetTick();
        memset(fine_X_PID->deriv_buf, 0, sizeof(fine_X_PID->deriv_buf));
        fine_X_PID->last_output = 0.0f;
        fine_X_PID->prev_filtered = 0.0f;
        fine_X_PID->prev_position = 0.0f;
        fine_X_PID->first_call = true;
    }

    //精调Y轴
    if(fine_Y_PID !=NULL)
    {
        fine_Y_PID->kp = 2.9f;
        fine_Y_PID->ki = 0.02f;
        fine_Y_PID->kd = 4.4f;
        fine_Y_PID->dt = 0.018f;
        fine_Y_PID->integral = 0.0f;
        fine_Y_PID->prev_error = 0.0f;
        fine_Y_PID->max_output = 200.0f;  
        fine_Y_PID->max_accel = 780.0f;
        fine_Y_PID->last_call_time = HAL_GetTick();
        fine_Y_PID->prev_current = HAL_GetTick();
        memset(fine_Y_PID->deriv_buf, 0, sizeof(fine_Y_PID->deriv_buf));
        fine_Y_PID->last_output = 0.0f;
        fine_Y_PID->prev_filtered = 0.0f;
        fine_Y_PID->prev_position = 0.0f;
        fine_Y_PID->first_call = true;
    }

    //精调Z轴
    if(fine_Z_PID !=NULL)
    {
        fine_Z_PID->kp = 6.1f;
        fine_Z_PID->ki = 0.001f;
        fine_Z_PID->kd = 0.47f;      
        fine_Z_PID->dt = 0.018f;
        fine_Z_PID->integral = 0.0f;
        fine_Z_PID->prev_error = 0.0f;
        fine_Z_PID->max_output = 50.0f;
        fine_Z_PID->max_accel = 300.0f;
        fine_Z_PID->last_call_time = HAL_GetTick();
        memset(fine_Z_PID->deriv_buf, 0, sizeof(fine_Z_PID->deriv_buf));
        fine_Z_PID->last_output = 0.0f;
        fine_Z_PID->prev_filtered = 0.0f;
        fine_Z_PID->prev_position = 0.0f;
        fine_Z_PID->first_call = true;
    }   

    //简易X轴
    if(Easy_X_PID !=NULL)
    {
        Easy_X_PID->kp = 2.2f;
        Easy_X_PID->ki = 0.01f;
        Easy_X_PID->kd = 0.42f;
        Easy_X_PID->dt = 0.018f;
        Easy_X_PID->integral = 0.0f;
        Easy_X_PID->prev_error = 0.0f;
        Easy_X_PID->max_output = 450.0f;
        Easy_X_PID->max_accel = 470.0f;
        Easy_X_PID->last_call_time = HAL_GetTick();
        memset(Easy_X_PID->deriv_buf, 0, sizeof(Easy_X_PID->deriv_buf));
        Easy_X_PID->last_output = 0.0f;
        Easy_X_PID->prev_filtered = 0.0f;
        Easy_X_PID->prev_position = 0.0f;
        Easy_X_PID->first_call = true;
        
    }

    //简易Y轴
    if(Easy_Y_PID !=NULL)
    {
        Easy_Y_PID->kp = 2.5f;
        Easy_Y_PID->ki = 0.01f;
        Easy_Y_PID->kd = 0.24f;
        Easy_Y_PID->dt = 0.018f;
        Easy_Y_PID->integral = 0.0f;
        Easy_Y_PID->prev_error = 0.0f;
        Easy_Y_PID->max_output = 450.0f;
        Easy_Y_PID->max_accel = 470.0f;
        Easy_Y_PID->last_call_time = HAL_GetTick();
        memset(Easy_Y_PID->deriv_buf, 0, sizeof(Easy_Y_PID->deriv_buf));
        Easy_Y_PID->last_output = 0.0f;
        Easy_Y_PID->prev_filtered = 0.0f;
        Easy_Y_PID->prev_position = 0.0f;
        Easy_Y_PID->first_call = true;
    }

    //简易Z轴
    if(Easy_Z_PID !=NULL)
    {
        Easy_Z_PID->kp = 6.0f;
        Easy_Z_PID->ki = 0.01f;
        Easy_Z_PID->kd = 0.47f;
        Easy_Z_PID->dt = 0.018f;
        Easy_Z_PID->integral = 0.0f;
        Easy_Z_PID->prev_error = 0.0f;
        Easy_Z_PID->max_output = 65.0f;
        Easy_Z_PID->max_accel = 300.0f;
        Easy_Z_PID->last_call_time = HAL_GetTick();
        memset(Easy_Z_PID->deriv_buf, 0, sizeof(Easy_Z_PID->deriv_buf));
        Easy_Z_PID->last_output = 0.0f;
        Easy_Z_PID->prev_filtered = 0.0f;
        Easy_Z_PID->prev_position = 0.0f;
        Easy_Z_PID->first_call = true;
    }
    //平移X轴
    if(Translation_X_PID !=NULL)
    {
        Translation_X_PID->kp = 18.1f;
        Translation_X_PID->ki = 0.001f;
        Translation_X_PID->kd = 2.87f;
        Translation_X_PID->dt = 0.018f;
        Translation_X_PID->integral = 0.0f;
        Translation_X_PID->prev_error = 0.0f;
        Translation_X_PID->max_output = 600.0f;
        Translation_X_PID->max_accel = 1800.0f;
        Translation_X_PID->last_call_time = HAL_GetTick();
        memset(Translation_X_PID->deriv_buf, 0, sizeof(Translation_X_PID->deriv_buf));
        Translation_X_PID->last_output = 0.0f;
        Translation_X_PID->prev_filtered = 0.0f;
        Translation_X_PID->prev_position = 0.0f;
        Translation_X_PID->first_call = true;          
    }
    //平移Y轴
    if(Translation_Y_PID !=NULL)
    {
        Translation_Y_PID->kp = 2.18f;
        Translation_Y_PID->ki = 0.001f;
        Translation_Y_PID->kd = 0.22f;
        Translation_Y_PID->dt = 0.018f;
        Translation_Y_PID->integral = 0.0f;
        Translation_Y_PID->prev_error = 0.0f;
        Translation_Y_PID->max_output = 450.0f;
        Translation_Y_PID->max_accel = 450.0f;
        Translation_Y_PID->last_call_time = HAL_GetTick();
        memset(Translation_Y_PID->deriv_buf, 0, sizeof(Translation_Y_PID->deriv_buf));
        Translation_Y_PID->last_output = 0.0f;
        Translation_Y_PID->prev_filtered = 0.0f;    
        Translation_Y_PID->prev_position = 0.0f;
        Translation_Y_PID->first_call = true;
    }
    //平移Z轴
    if(Translation_Z_PID !=NULL)
    {
        Translation_Z_PID->kp = 6.0f;
        Translation_Z_PID->ki = 0.01f;
        Translation_Z_PID->kd = 0.47f;
        Translation_Z_PID->dt = 0.018f;
        Translation_Z_PID->integral = 0.0f;
        Translation_Z_PID->prev_error = 0.0f;
        Translation_Z_PID->max_output = 65.0f;
        Translation_Z_PID->max_accel = 300.0f;
        Translation_Z_PID->last_call_time = HAL_GetTick();
        memset(Translation_Z_PID->deriv_buf, 0, sizeof(Translation_Z_PID->deriv_buf));
        Translation_Z_PID->last_output = 0.0f;
        Translation_Z_PID->prev_filtered = 0.0f;
        Translation_Z_PID->prev_position = 0.0f;
        Translation_Z_PID->first_call = true;
    }

}

//重置PID控制器(消除累积项)
void Reset_PID(PIDController *pid)
{
    if(pid != NULL)
    {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        pid->last_call_time = HAL_GetTick();
        memset(pid->deriv_buf, 0, sizeof(pid->deriv_buf));
        pid->last_output = 0.0f;
        pid->prev_filtered = 0.0f;
        pid->prev_position = 0.0f;
        pid->first_call = true;
    }

}

//水平运动控制器

// 水平运动控制器 - 针对高滞后/低频采样优化版
float PID_Calc_XY_Vision(PIDController *pid, float target, float current)
{
    // 假设滞后时间为 0.5 秒 (500ms)
    const float LATENCY = 500.0f; 

    // ==========================================================
    // 可调参数区 (针对200ms滞后系统的核心优化)
    // ==========================================================
    const float INT_THRESHOLD = 50.0f;          // 积分分离阈值：误差小于此值才启用积分
    const float MAX_INT = 200.0f;               // 积分限幅：防止积分饱和
    const float MAX_OUTPUT_CHANGE_SEC = 100.0f; // 变化率限制：限制每秒输出的最大变化幅度
    // ==========================================================

    // 1. 时间处理
    // 1. 预测当前真实位置 (State Extrapolation)
    // 利用当前位置和上次位置计算出的速度，外推500ms后的位置
    float dt_calc = (HAL_GetTick() - pid->last_call_time) / 1000.0f;
    float velocity = (current - pid->prev_current) / (dt_calc > 0 ? dt_calc : 0.05f);
    float estimated_current = current + (velocity * LATENCY);
    
    // 使用这个预测出的值来计算误差，而不是使用滞后的 current
    float error = target - estimated_current;

    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pid->last_call_time) / 1000.0f;
    // 防御性编程：防止dt过小（如抖动）或过大（如程序阻塞）
    if (dt < 0.05f) dt = 0.05f; 
    if (dt > 0.5f) dt = 0.5f;   // 限制最大dt，防止更新断层导致计算溢出
    pid->last_call_time = current_time;

    // 2. 积分分离 (Integral Separation)
    float i_term = 0.0f;
    if (fabsf(error) < INT_THRESHOLD) { 
        pid->integral += error * dt;
        // 积分限幅
        if (pid->integral > MAX_INT) pid->integral = MAX_INT;
        else if (pid->integral < -MAX_INT) pid->integral = -MAX_INT;
        i_term = pid->ki * pid->integral;
    } else {
        // 远离目标时直接清零积分，杜绝滞后导致的超调累积
        pid->integral = 0.0f; 
    }

    // 3. 微分先行 (Derivative on Measurement)
    // 对测量值求导，消除target突变带来的抖动
    float derivative = (current - pid->prev_current) / dt;
    pid->prev_current = current;
    
    // 微分低通滤波 (防止低频采样中的毛刺)
    float filtered_derivative = 0.7f * derivative + 0.3f * pid->prev_filtered;
    pid->prev_filtered = filtered_derivative;
    
    // 注意：对测量值求导，为了保证与Kp(target-current)逻辑一致，D项应为负
    float d_term = -pid->kd * filtered_derivative;

    // 4. 计算输出
    float p_term = pid->kp * error;
    float output = p_term + i_term + d_term;

    // 5. 变化率限制 (核心优化：平滑输出，解决滞后带来的阶梯状突变)
    float max_change = MAX_OUTPUT_CHANGE_SEC * dt;
    float output_change = output - pid->last_output;
    
    if (output_change > max_change) {
        output = pid->last_output + max_change;
    } else if (output_change < -max_change) {
        output = pid->last_output - max_change;
    }

    // 6. 最终限幅
    if (output > pid->max_output) output = pid->max_output;
    else if (output < -pid->max_output) output = -pid->max_output;

    pid->last_output = output;
    return output;
}

float PID_Calc_XY(PIDController *pid, float target, float current)
{
    if(pid->first_call)
    {
        pid->first_call = false;
        pid->last_call_time = HAL_GetTick();

        
        return 0.0f;
    }

    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pid->last_call_time) / 1000.0f;
    pid->last_call_time = current_time;
    pid->dt = dt;

    // 计算误差
    float error = target - current;
    // printf("error: %f\r\n", error);
    // float abs_error = fabsf(error);
    
    // 计算比例项
    float p_term = pid->kp * error;
    
    // 计算积分项 (带积分饱和控制)
    // 积分项累加
    pid->integral += error * dt;
    
    // 积分限幅 (防止积分饱和)
    if(pid->ki != 0.0f)
    {
        float max_integral = pid->max_output / pid->ki;
        if (pid->integral > max_integral) {
            pid->integral = max_integral;
        } else if (pid->integral < -max_integral) {
            pid->integral = -max_integral;
        }
        
        // 积分抗饱和策略 - 当输出即将饱和时减少积分作用
        if ((p_term > pid->max_output && error > 0) || 
            (p_term < -pid->max_output && error < 0)) {
            pid->integral = 0.9f * pid->integral; // 逐渐减小积分项
        }
    }
    
    float i_term = pid->ki * pid->integral;
    
    // 计算微分项 (带滤波)
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pid->prev_error) / dt;
    }
    
    // 对微分项进行低通滤波 (简单一阶滤波)
    float filtered_derivative = 0.8f * derivative + 0.2f * pid->prev_filtered;
    pid->prev_filtered = filtered_derivative;
    
    float d_term = pid->kd * filtered_derivative;
    
    // 计算总输出
    float output = p_term + i_term + d_term;
    
    // 输出限幅
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
    }
    
    // 平滑输出 (减少突变)
    // output = 0.7f * output + 0.3f * pid->last_output;
    float max_accel = pid->max_accel; // 最大加速度，单位：单位/秒²
    float output_change = output - pid->last_output;
    if(output_change > max_accel * dt) {
        output = pid->last_output + max_accel * dt;
    } else if(output_change < -max_accel * dt) {
        output = pid->last_output - max_accel * dt;
    }

    pid->last_output = output;
    
    // 保存当前误差用于下次计算
    pid->prev_error = error;
    
    return output;
}

//z轴转动控制器
float PID_Calc_Z(PIDController *pid, float target, float current)
{
    if(pid->first_call)
    {
        pid->first_call = false;
        pid->last_call_time = HAL_GetTick();

        return 0.0f;
    }

    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pid->last_call_time) / 1000.0f;
    pid->last_call_time = current_time;
    pid->dt = dt;

    //计算误差
    float error = target - current;
    // float abs_error = fabsf(error);
    
    //计算比例项
    float p_term = pid->kp * error;

    //计算积分项(带积分饱和控制)
    pid->integral += error * dt;    
    
    //积分限幅(防止积分饱和)
    if(pid->ki != 0.0f)
    {
        float max_integral = pid->max_output / pid->ki;
        if(pid->integral > max_integral)
        {
            pid->integral = max_integral;
        }   
        else if(pid->integral < -max_integral)
        {
            pid->integral = -max_integral;
        }   
    
        //积分抗饱和策略 - 当输出即将饱和时减少积分作用
        if((p_term > pid->max_output && error > 0) || (p_term < -pid->max_output && error < 0))
        {
            pid->integral = 0.9f * pid->integral; //逐渐减小积分项
        }
    }
    
    float i_term = pid->ki * pid->integral;

    //计算微分项(带滤波)
    float derivative = 0.0f;
    if(dt > 0.0f)
    {
        derivative = (error - pid->prev_error) / dt;
    }
    
    //对微分项进行低通滤波(简单一阶滤波)
    float filtered_derivative = 0.8f * derivative + 0.2f * pid->prev_filtered;
    pid->prev_filtered = filtered_derivative;
    
    float d_term = pid->kd * filtered_derivative;   
    
    //计算总输出
    float output = p_term + i_term + d_term;
    
    //输出限幅
    if(output > pid->max_output)        
    {
        output = pid->max_output;
    }
    else if(output < -pid->max_output)
    {
        output = -pid->max_output;
    }
    
        //平滑输出(减少突变)
    // output = 0.7f * output + 0.3f * pid->last_output;
    float max_accel = pid->max_accel; // 最大加速度，单位：单位/秒²
    float output_change = output - pid->last_output;
    if(output_change > max_accel * dt) {
        output = pid->last_output + max_accel * dt;
    } else if(output_change < -max_accel * dt) {
        output = pid->last_output - max_accel * dt;
    }
    pid->last_output = output;
    
    //保存当前误差用于下次计算
    pid->prev_error = error;

    return output;
}
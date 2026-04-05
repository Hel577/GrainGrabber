#include "toolfunc.h"
#include <math.h>
#include <stdio.h>

//变量定义见main.c
extern const double pi;



// 符号函数
int sign(float x)
{
	return x>0? 1:-1;
}


/************************************
* int32Tohex：int32转为4位十六进制
************************************/
void int32Tohex(int32_t data,uint8_t hexdata[4])
{
	uint8_t tempdata[4];
	union
	{
		int32_t a;
		uint8_t hexs[4];
	}thing;
	thing.a=data;
	memcpy(tempdata,thing.hexs,4);
	uint32_t i;
	for(i=0;i<4;i++)
	{
		hexdata[i]=tempdata[i];
	}
}


/************************************
* uint2hex：uint32转为2位十六进制
************************************/
void uint2hex(uint32_t data,uint8_t hexdata[2])
{
	uint8_t tempdata[2];
	union
	{
		uint32_t a;
		uint8_t hexs[2];
	}thing;
	thing.a=data;
	memcpy(tempdata,thing.hexs,2);
	uint32_t i;
	for(i=0;i<2;i++)
	{
		hexdata[i]=tempdata[i];
	}
}



/************************************
* int16Tohex：int16转为2位十六进制
************************************/
void int16Tohex(int16_t data,uint8_t hexdata[2])
{
	uint8_t tempdata[2];
	union
	{
		int16_t a;
		uint8_t hexs[2];
	}thing;
	thing.a=data;
	memcpy(tempdata,thing.hexs,2);
	uint32_t i;
	for(i=0;i<2;i++)
	{
		hexdata[i]=tempdata[i];
	}
}


/************************************
* hex2float：4位十六进制转float
************************************/
float hex2float(uint8_t hexs_temp[4])
{
	union
	{
		float a;
		uint8_t hexs[4];
	}thing;
	memcpy(thing.hexs,hexs_temp,4);
	return thing.a;
}


// 原有的 calculate_angle 函数
float calculate_angle(float x, float y) {
    // 使用 atan2 函数计算弧度
    float angle_radians = atan2(y, x);

    // 将弧度转换为角度
    float angle_degrees = angle_radians * 180.0f / pi;

    // 如果需要将角度限制在 [0, 360)
    if (angle_degrees < 0) {
        angle_degrees += 360.0f;
    }

    return angle_degrees;
}



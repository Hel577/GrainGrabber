#ifndef __TOOLFUNC_H
#define __TOOLFUNC_H
#include "stdint.h"
#include "string.h"

int sign(float x);
void int32Tohex(int32_t data,uint8_t hexdata[4]);
void uint2hex(uint32_t data,uint8_t hexdata[2]);
void int16Tohex(int16_t data,uint8_t hexdata[2]);
float hex2float(uint8_t hexs_temp[4]);
float calculate_angle(float x, float y);

#endif

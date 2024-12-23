#ifndef _FKIK_H
#define _FKIK_H

#include <Arduino.h>
#include <math.h>

#define L0 66.4
#define L1 145

#define Offset0 32.39

float Round(float in,int num);                          //小数点控制
float D2R(float deg);                                   //角度转
float R2D(float rad);                                   //弧度转

void FK(float j0,float j1,float point[]);               //正解函数
void IK(float x,float y,float Angle[]);                 //逆解函数


#endif

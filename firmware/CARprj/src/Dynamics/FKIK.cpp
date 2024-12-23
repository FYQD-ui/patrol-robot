#include "FKIK.h"

//四舍五入+控制小数点
float Round(float in,int num){
  in = round(in*pow(10,num))/pow(10,num);
  return in;
}

//角度---->弧度
float D2R(float deg){
  return deg*PI/180;
}

//弧度---->角度
float R2D(float rad){
  return rad*180/PI;
}

//正运动学函数
void FK(float j0,float j1,float point[]){
  
  float x,y;              //空间的三点坐标

  j0 = D2R(j0);           //角度-->弧度
  j1 = D2R(j1);

  //正解核心公式 
  /*---------------------------------------------------------------------*/  
  x =   L0*sin(j0) - L1*sin(j0+j1);
  y = - L0*cos(j0) + L1*cos(j0+j1);
  /*---------------------------------------------------------------------*/
  
  point[0] = Round(x,3);
  point[1] = Round(y,3);
}

//逆运动学函数
void IK(float x,float y,float Angle[]){
  float j0,j1;                   //腿部二关节角度

  //逆运动学公式
  // 1.j1求解
  j1 = acos(( (L0*L0+L1*L1) - (x*x+y*y))/(2*L0*L1));

  // 2.j0求解
  float L2 = sqrt(x*x + y*y);
  float j2 = atan(abs(y/x));

  j0 = PI - asin((x*x + y*y + L0*L0 - L1*L1) / (2*L0*L2) ) - j2;

  //角度结果存储
  Angle[0] = Round(R2D(j0),2);
  Angle[1] = Round(R2D(j1),2);
}

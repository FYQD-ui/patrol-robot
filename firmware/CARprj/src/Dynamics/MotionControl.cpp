#include "MotionControl.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

extern WiFiClient         client_Move;
extern Adafruit_NeoPixel  pixels;

void Robot::begin(){
  pwm.begin();
  pwm.setPWMFreq(50); // Servos run at 50 Hz



  float xs = 70;
  // 获取摆线轨迹
  GetCycloidPoints(F1_CPoints,xs,       xs+XMOVE, YS,H);
  GetCycloidPoints(F2_CPoints,xs-XMOVE, xs,       YS,H);
  GetCycloidPoints(F3_CPoints,xs-XMOVE, xs+XMOVE, YS,H);

  GetCycloidPoints(B1_CPoints,xs,       xs-XMOVE, YS,H);
  GetCycloidPoints(B2_CPoints,xs+XMOVE, xs,       YS,H);
  GetCycloidPoints(B3_CPoints,xs+XMOVE, xs-XMOVE, YS,H);

  // 获取初始点值
  IK_ResetQuadruped();

  Serial.println(F("[Robot]\tInit Success!"));
}

/* ------------------ 模式控制 ------------------ */
// 舵机:顺时针 + ,逆时针 -
void Robot::ResetTrack(bool status){
  uint16_t DSD = 250;

  if(status){
    FK_RUMove(125,55,DSD);  
    FK_LUMove(125,55,DSD);
    FK_RBMove(125,55,DSD);
    FK_LBMove(125,55,DSD);
  }else{
    FK_RUMove(90,90,DSD);  
    FK_LUMove(90,90,DSD);
    FK_RBMove(90,90,DSD);
    FK_LBMove(90,90,DSD);
  }
}

void Robot::FK_ResetQuadruped(){
  FK_RUMove(115,40,250);  
  FK_LUMove(115,40,250);
  FK_RBMove(115,40,250);
  FK_LBMove(115,40,250);
}

void Robot::IK_ResetQuadruped(){
  IK_RUMove(XS,YS,250);  
  IK_LUMove(XS,YS,250);
  IK_RBMove(XS,YS,250);
  IK_LBMove(XS,YS,250);
}

void Robot::FK_TStatus_HIGHER(){
  FK_RUMove(0,180,250);  
  FK_LUMove(0,180,250);
  FK_RBMove(0,180,250);
  FK_LBMove(0,180,250);
}

void Robot::FK_TStatus_HIGH(){
  FK_RUMove(45,135,250);  
  FK_LUMove(45,135,250);
  FK_RBMove(45,135,250);
  FK_LBMove(45,135,250);
}

void Robot::FK_LEFT(){
  FK_LUMove(90,90,150);  
  FK_LBMove(90,90,150);
  FK_RUMove(60,120,150);
  FK_RBMove(60,120,150);
}

void Robot::FK_RIGHT(){
  FK_RUMove(90,90,150);  
  FK_RBMove(90,90,150);
  FK_LUMove(60,120,150);
  FK_LBMove(60,120,150);
}

/* ------------------ 单腿控制 ------------------ */
/*
* 角度Deg-->舵机控制值Val
*/
uint16_t Robot::AngleToCount(float angle){
  // Map angle (0-180) to pulse width (500us - 2500us)
  float pulseWidth = map(angle, 0.0, 180.0, 500.0, 2500.0); // Pulse width in microseconds
  // Convert pulse width to PCA9685 counts (0-4096)
  uint16_t counts = (uint16_t)(pulseWidth * 4096.0 / 20000.0); // Total period is 20,000us at 50Hz
  return counts;
}

/*
* 右前
*/
void Robot::FK_RUMove(float posk,float posx,uint16_t Time){
  posk = posk + 0;
  posx = posx - 0;

  uint16_t counts_posk = AngleToCount(posk);
  uint16_t counts_posx = AngleToCount(posx);

  pwm.setPWM(1, 0, counts_posk); // Channel 1 for posk
  pwm.setPWM(0, 0, counts_posx); // Channel 0 for posx

  delay(Time);
}

/*
* 右后
*/
void Robot::FK_RBMove(float posk,float posx,uint16_t Time){
  posk = posk + 0;
  posx = posx + 0;

  uint16_t counts_posk = AngleToCount(posk);
  uint16_t counts_posx = AngleToCount(posx);

  pwm.setPWM(2, 0, counts_posk); // Channel 2 for posk
  pwm.setPWM(3, 0, counts_posx); // Channel 3 for posx

  delay(Time);
}

/*
* 左前
*/
void Robot::FK_LUMove(float posk,float posx,uint16_t Time){
  posk = posk + 0;
  posx = posx + 2;

  uint16_t counts_posk = AngleToCount(posk);
  uint16_t counts_posx = AngleToCount(posx);

  pwm.setPWM(6, 0, counts_posk); // Channel 6 for posk
  pwm.setPWM(7, 0, counts_posx); // Channel 7 for posx

  delay(Time);
}

/*
* 左后
*/
void Robot::FK_LBMove(float posk,float posx,uint16_t Time){
  posk = posk + 6.5;
  posx = posx - 9;

  uint16_t counts_posk = AngleToCount(posk);
  uint16_t counts_posx = AngleToCount(posx);

  pwm.setPWM(5, 0, counts_posk); // Channel 5 for posk
  pwm.setPWM(4, 0, counts_posx); // Channel 4 for posx

  delay(Time);
}

/*
* 左侧双腿
*/
void Robot::FK_LSMove(float posk,uint16_t Time){
  FK_LUMove(posk,180 - posk,Time);
  FK_LBMove(posk,180 - posk,Time);
}

/*
* 右侧双腿
*/
void Robot::FK_RSMove(float posk,uint16_t Time){
  FK_RUMove(posk,180 - posk,Time);
  FK_RBMove(posk,180 - posk,Time);
}

// ********************************** //
// ************** 正解 ************** //
// ********************************** //
void Robot::FK_LegMove(float Angle[],uint8_t legNum,uint16_t DSD,bool Print){
  float point[2] = {};
  FK(Angle[0],Angle[1],point);

  if(Print){
    Serial.print(F("解算坐标:"));
    Serial.print(F("x:"));Serial.print(point[0]);Serial.print(F(" -- "));
    Serial.print(F("y:"));Serial.print(point[1]);Serial.print(F(" -- "));
    Serial.println();
  }

  switch(legNum){
    case 1: FK_RUMove(Angle[0],Angle[1],DSD);  break;   //1--->右前
    case 2: FK_RBMove(Angle[0],Angle[1],DSD);  break;   //2--->右后
    case 3: FK_LBMove(Angle[0],Angle[1],DSD);  break;   //3--->左后
    case 4: FK_LUMove(Angle[0],Angle[1],DSD);  break;   //4--->左前
  }
}

// ********************************** //
// ************** 逆解 ************** //
// ********************************** //
/*
* 右前
*/
void Robot::IK_RUMove(float x,float y,uint16_t Time){
  float angle[2] = {};

  IK_RUPoint[0] = x;
  IK_RUPoint[1] = y;

  IK(x,y,angle);                    // 逆解
  angle[1] = angle[1] - Offset0;    // 偏差调整

  FK_RUMove(angle[0],angle[1],Time);
}

/*
* 左前
*/
void Robot::IK_LUMove(float x,float y,uint16_t Time){
  float angle[2] = {};

  IK_LUPoint[0] = x;
  IK_LUPoint[1] = y;

  IK(x,y,angle);                    // 逆解
  angle[1] = angle[1] - Offset0;    // 偏差调整

  FK_LUMove(angle[0],angle[1],Time);
}

/*
* 右后
*/
void Robot::IK_RBMove(float x,float y,uint16_t Time){
  float angle[2] = {};

  IK_RBPoint[0] = x;
  IK_RBPoint[1] = y;

  IK(x,y,angle);                    // 逆解
  angle[1] = angle[1] - Offset0;    // 偏差调整

  FK_RBMove(angle[0],angle[1],Time);
}

/*
* 左后
*/
void Robot::IK_LBMove(float x,float y,uint16_t Time){
  float angle[2] = {};

  IK_LBPoint[0] = x;
  IK_LBPoint[1] = y;

  IK(x,y,angle);                    // 逆解
  angle[1] = angle[1] - Offset0;    // 偁差调整

  FK_LBMove(angle[0],angle[1],Time);
}

void Robot::IK_LegMove(float Point[],uint8_t LEGNum,uint16_t DSD,bool Print){
  float angle[2] = {};
  IK(Point[0],Point[1],angle);
  angle[1] = angle[1] - Offset0;    // 偏差调整

  if(Print){
    Serial.print(F("解算角度:"));
    Serial.print(F("K:"));Serial.print(angle[0]);Serial.print(F(" -- "));
    Serial.print(F("H:"));Serial.print(angle[1]);Serial.print(F(" -- "));
    Serial.println();
  }

  switch(LEGNum){
    case 1: FK_RUMove(angle[0],angle[1],DSD);  break;   //1--->右前
    case 2: FK_RBMove(angle[0],angle[1],DSD);  break;   //2--->右后
    case 3: FK_LBMove(angle[0],angle[1],DSD);  break;   //3--->左后
    case 4: FK_LUMove(angle[0],angle[1],DSD);  break;   //4--->左前
  }
}

//保持稳定
void Robot::HoldInitialPosition() {
    // 设置舵机到初始位置
    IK_ResetQuadruped(); // 使用反解，将四足机器人设定到默认姿态

    // 如果需要，可以添加舵机保持功能，确保舵机持续上电
    // 例如，定期发送舵机控制信号，保持其位置
}

/*
* 单腿调试
*/
void Robot::LegPointDebug(float point[],uint8_t cmd,byte offset,bool PrintPoint,bool PrintAngle){
    // 省略了卸载部分，因为标准PWM舵机无法卸载
    switch(cmd){
      //X
      case '4': point[0] += offset;break;
      case '1': point[0] -= offset;break;
      //Y
      case '5': point[1] += offset;break;
      case '2': point[1] -= offset;break;
      //Leg
      case '0':
        legNum++;
        if(legNum == 5){
          legNum = 1;
        }
        Serial.print("LegNum:");
        Serial.println(legNum);
        break;
    }

    if(PrintPoint){
      Serial.print(F("当前坐标:"));
      Serial.print(F("x:"));Serial.print(point[0]);Serial.print(F(" -- "));
      Serial.print(F("y:"));Serial.print(point[1]);Serial.print(F(" -- "));
      Serial.println();
    }

    IK_LegMove(point,legNum,250,PrintAngle);
}

/*
* 摆线轨迹获取
*
* xs:X轴起点位置
* xf:X轴终点位置
* ys:y轴起点位置
* yh:抬腿高度
*/
void Robot::GetCycloidPoints(float CPoints[][2],float xs,float xf,float ys,float yh){
  uint8_t count = 0;

  float offset  = 0.06;             // 生成10个点
  float sigma,xep,yep;
  float t;

  float Ts = 1,   fai = 0.5;        // 周期T， 占空比fai(支撑相)

  /* ############### 计算部分 ############### */
  for(t = 0; t <= Ts*fai; t += offset){
    sigma = 2*PI*t/(fai*Ts);
    xep   = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;   // x轴坐标
    yep   = yh*(1-cos(sigma))/2+ys;                   // y轴坐标
    
    CPoints[count][0] = xep;
    CPoints[count][1] = yep;
    count += 1;
  }

  CPoints[count][0] = xf;
  CPoints[count][1] = ys;
  count += 1;
  /* ######################################## */
}

/*
* 足端摆线运动
*/
void Robot::LegCycloid(float CPoints[][2],uint8_t LEGNum){
  uint8_t Num = 10;
  uint8_t DSD = 250/Num;

  for(uint8_t i = 0;i < 10;i++){
    switch (LEGNum){
      case 1:IK_RUMove(CPoints[i][0],CPoints[i][1],DSD); break;
      case 2:IK_RBMove(CPoints[i][0],CPoints[i][1],DSD); break;
      case 3:IK_LBMove(CPoints[i][0],CPoints[i][1],DSD); break;
      case 4:IK_LUMove(CPoints[i][0],CPoints[i][1],DSD); break;
    }
    delay(DSD);
  }
}

/* ------------------ 姿态控制 ------------------ */
void Robot::PosMove(float xMove,float yMove,uint16_t Time){

  IK_RUMove(IK_RUPoint[0]-xMove, IK_RUPoint[1]-yMove, Time);
  IK_LUMove(IK_LUPoint[0]-xMove, IK_LUPoint[1]-yMove, Time);
  IK_RBMove(IK_RBPoint[0]+xMove, IK_RBPoint[1]-yMove, Time);
  IK_LBMove(IK_LBPoint[0]+xMove, IK_LBPoint[1]-yMove, Time);
}

void Robot::PosAction1(){
  uint8_t DSD = 200;
  PosMove(50,-25, DSD);  delay(DSD*4);
  PosMove(0,50,   DSD);  delay(DSD*4);
  PosMove(-100,0, DSD);  delay(DSD*4);
  PosMove(0,-50,  DSD);  delay(DSD*4);
  PosMove(50,25,  DSD);
}

void Robot::PosToPoint(float x,float y,uint16_t Time){

  IK_RUMove(XS-x, YS-y, Time);
  IK_LUMove(XS-x, YS-y, Time);
  IK_RBMove(XS+x, YS-y, Time);
  IK_LBMove(XS+x, YS-y, Time);
}

void Robot::PosToPitch(float deg,uint16_t Time){
  float LBody = 85.8;
  float Dx,Dy;

  deg = D2R(deg);   // 转弧度

  Dx = LBody/2*(1 - cos(deg));
  Dy = LBody/2*sin(deg);

  IK_RUMove(XS+Dx, YS-Dy, Time);
  IK_LUMove(XS+Dx, YS-Dy, Time);
  IK_RBMove(XS+Dx, YS+Dy, Time);
  IK_LBMove(XS+Dx, YS+Dy, Time);
}

void Robot::PosToRoll (float deg,uint16_t Time){
  float WBody = 148;
  float Dh,rad;

  rad = D2R(abs(deg));         // 转弧度

  Dh  = WBody*tan(rad);

  if(deg > 0){
    IK_RUMove(XS, YS - Dh,  Time);
    IK_RBMove(XS, YS - Dh,  Time);
    IK_LUMove(XS, YS,       Time);
    IK_LBMove(XS, YS,       Time);
  }else{
    IK_RUMove(XS, YS,       Time);
    IK_RBMove(XS, YS,       Time);
    IK_LUMove(XS, YS - Dh,  Time);
    IK_LBMove(XS, YS - Dh,  Time);
  }

}

void Robot::PosAction2(uint8_t Num){
  uint8_t DSD = 25;

  float r     = 30;
  float speed = 5;
  float deg   = 0;
  float theta;
  float xr,yr;

  for(uint8_t i = 0;i < Num;i++){
    while(deg <= 360){
      theta = D2R(deg);

      xr = r*cos(theta);
      yr = r*sin(theta);

      PosToPoint(xr,yr,DSD);
      delay(DSD);

      deg += speed;
    }

    deg = 0;
  }
  
  delay(500);
  PosToPoint(0,0,200);
}

void Robot::PosAction3(){
  PosToPitch(45,200);   // 抬头
  delay(800);
  PosToPitch(-45,200);  // 低头
  delay(800);
  PosToPitch(0,200);    // 归位
}

void Robot::PosAction4(){
  PosToRoll(15,200);   // 左倾
  delay(800);
  PosToRoll(-15,200);  // 右倾
  delay(800);
  PosToRoll(0,200);    // 归位
}

/* ------------------ 运动控制 ------------------ */
void Robot::Walk_Basic(uint8_t SetpNum,bool dir){
  uint8_t RestTime  = 50;
  uint8_t RestTime2 = 100;
  
  if(dir){
    if(SetpNum != 0){
      //右前+左后
      LegCycloid(F1_CPoints,1); delay(RestTime);
      LegCycloid(B1_CPoints,3); delay(RestTime);

      for(uint8_t i = 0;i < SetpNum - 1;i++){
        PosMove(XMOVE,0,RestTime2);  delay(RestTime2);

        //左前+右后
        LegCycloid(F3_CPoints,4);   delay(RestTime);
        LegCycloid(B3_CPoints,2);   delay(RestTime);

        PosMove(XMOVE,0,RestTime2);  delay(RestTime2);

        //右前+左后
        LegCycloid(F3_CPoints,1); delay(RestTime);
        LegCycloid(B3_CPoints,3); delay(RestTime);
      }

      PosMove(XMOVE,0,RestTime2);  delay(RestTime2);

      //左前+右后
      LegCycloid(F2_CPoints,4);   delay(RestTime);
      LegCycloid(B2_CPoints,2);   delay(RestTime);
    }
  }else{
    if(SetpNum != 0){
      //右前+左后
      LegCycloid(B1_CPoints,1); delay(RestTime);
      LegCycloid(F1_CPoints,3); delay(RestTime);

      for(uint8_t i = 0;i < SetpNum - 1;i++){
        PosMove(-XMOVE,0,RestTime2);  delay(RestTime2);

        //左前+右后
        LegCycloid(B3_CPoints,4);   delay(RestTime);
        LegCycloid(F3_CPoints,2);   delay(RestTime);

        PosMove(-XMOVE,0,RestTime2);  delay(RestTime2);

        //右前+左后
        LegCycloid(B3_CPoints,1); delay(RestTime);
        LegCycloid(F3_CPoints,3); delay(RestTime);
      }

      PosMove(-XMOVE,0,RestTime2);  delay(RestTime2);

      //左前+右后
      LegCycloid(B2_CPoints,4);   delay(RestTime);
      LegCycloid(F2_CPoints,2);   delay(RestTime);
    }
  }
}  

/*
* Trot小跑步态
*/
void Robot::Trot(){

  TrotStatus  = 1;
  uint8_t dir = 2;
  uint8_t DSD = 20;
  uint8_t cmd = 0;

  float sigma,xep_b,xep_z,yep;
  float t = 0;
  
  float speed = 0.05;                         // 步频

  float Ts = 1,     fai = 0.5;                // 周期T， 占空比fai(支撑相)
  float offset = 20;

  float xs = XS - offset,   xf = XS + offset; // 起始坐标，终点坐标
  float ys = -130,  yh  = 20;                 // 起始坐标，抬腿高度

  //足端坐标
  float x1 = XS,x2 = XS,x3 = XS,x4 = XS;    
  float y1 = YS,y2 = YS,y3 = YS,y4 = YS;

  Serial.println(F("启动Trot模式..."));

  pixels.fill(pixels.Color(0, 127, 255)); // 青色 
  pixels.show(); 

  while(TrotStatus){
    // 1.Trot指令
    if(Serial.available() || client_Move.available()){

      if(Serial.available())      cmd = Serial.read();
      if(client_Move.available()) cmd = client_Move.read();
            
      if(cmd != '\n'){
        //BlinkLed(LEDpin,1); 
        Serial.printf("  *Trot--Order:%c\n",cmd);

        switch(cmd){
          case '8':dir = 1;       break;
          case '5':dir = 2;       break;
          case '2':dir = 0;       break;
          case 's':dir = 3;       break;

          // #参数调试
          case '4':offset += 5; Serial.print(F("  #o:"));  Serial.println(offset);  xs = XS - offset;xf = XS + offset;break;
          case '1':offset -= 5; Serial.print(F("  #o:"));  Serial.println(offset);  xs = XS - offset;xf = XS + offset;break;
          case '6':yh     += 5; Serial.print(F("  #h:"));  Serial.println(yh);      break;
          case '3':yh     -= 5; Serial.print(F("  #h:"));  Serial.println(yh);      break;
          case '7':DSD    += 5; Serial.print(F("  #DSD:"));Serial.println(DSD);     break;
          case '9':DSD    -= 5; Serial.print(F("  #DSD:"));Serial.println(DSD);     break;

          case '0':
            TrotStatus = 0;
            pixels.fill(pixels.Color(238, 18, 137)); // 四足
            pixels.show(); 
            break;
        }
      }
    }

    // 2.单个步态周期
    while(t < 1){
      // 前半周期
      if(t <= Ts*fai){
        sigma  = 2*PI*t/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y轴坐标

        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // 摆动相x轴坐标
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // 支撑相x轴坐标

        if(dir != 2){
          y1 = yep;
          y2 = ys;
          y3 = yep;
          y4 = ys;
        }

        if(dir == 1){
          x1 = xep_b;
          x2 = xep_z;
          x3 = xep_z;
          x4 = xep_b;
          
        }else if(dir == 0){
          x1 = xep_z;
          x2 = xep_b;
          x3 = xep_b;
          x4 = xep_z;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }
      // 后半周期
      if(Ts*fai < t && t < Ts){
        sigma  = 2*PI*(t - Ts*fai)/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y轴坐标

        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // 摆动相x轴坐标
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // 支撑相x轴坐标

        if(dir != 2){
          y1 = ys;
          y2 = yep;
          y3 = ys;
          y4 = yep;
        }

        if(dir == 1){
          x1 = xep_z;
          x2 = xep_b;
          x3 = xep_b;
          x4 = xep_z;
        }else if(dir == 0){
          x1 = xep_b;
          x2 = xep_z;
          x3 = xep_z;
          x4 = xep_b;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }

      t = t + speed;

      IK_RUMove(x1,y1,DSD);
      IK_LUMove(x2,y2,DSD);
      IK_LBMove(x3,y3,DSD);
      IK_RBMove(x4,y4,DSD);

      delay(DSD);
    }

    if(TrotStatus){
      t = 0;
    }   
  }

  IK_ResetQuadruped();
  Serial.println(F("退出Trot模式..."));
}

/*
* Walk行走步态
*/
void Robot::Walk(){

  WalkStatus  = 1;
  uint8_t dir = 2;
  uint8_t DSD = 20;
  uint8_t cmd = 0;

  float sigma,xep_b,xep_z,yep;
  float t = 0;

  float Ts = 1,     fai = 0.25;      // 周期T， 占空比fai(支撑相)
  
  float speed = 0.0249;               //步频
  float offset = 20;

  float xs = XS - offset,   xf = XS + offset; // 起始坐标，终点坐标

  // float xs = 45 ,   xf  = 135;
  float ys = -130,  yh  = 20;

  //足端坐标
  float x1,x2,x3,x4;    
  float y1,y2,y3,y4;

  float step = (xf - xs) / 30.0;

  x1 = xs;
  x2 = xs + 21*step;
  x3 = xs + 19*step;
  x4 = xs - step;

  y1 = ys;
  y2 = ys;
  y3 = ys;
  y4 = ys;

  Serial.println(F("启动Walk模式..."));
  pixels.fill(pixels.Color(98,255, 10));   // 绿色
  pixels.show(); 

  while(WalkStatus){

    // 1.Walk指令
    if(Serial.available() || client_Move.available()){

      if(Serial.available())      cmd = Serial.read();
      if(client_Move.available()) cmd = client_Move.read();
      
      if(cmd != '\n'){
        //BlinkLed(LEDpin,1); 
        Serial.printf("  *Walk--Order:%c\n",cmd);
        
        switch(cmd){
          case '2':dir = 0;       break;
          case '8':dir = 1;       break;
          case '5':dir = 2;       break;
          case 's':dir = 3;       break;

          // #参数调试
          case '4':offset += 5; Serial.print(F("  #o:"));  Serial.println(offset);  xs = XS - offset;xf = XS + offset;break;
          case '1':offset -= 5; Serial.print(F("  #o:"));  Serial.println(offset);  xs = XS - offset;xf = XS + offset;break;
          case '6':yh     += 5; Serial.print(F("  #h:"));  Serial.println(yh);      break;
          case '3':yh     -= 5; Serial.print(F("  #h:"));  Serial.println(yh);      break;
          case '7':DSD    += 5; Serial.print(F("  #DSD:"));Serial.println(DSD);     break;
          case '9':DSD    -= 5; Serial.print(F("  #DSD:"));Serial.println(DSD);     break;

          case '0':
            WalkStatus = 0;
            pixels.fill(pixels.Color(238, 18, 137)); // 四足
            pixels.show(); 
            break;
        }
      }
    }

    while(t < 1){
      // 1.RU
      if(t < Ts*fai){
        sigma  = 2*PI*t/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y轴坐标
        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // 摆动相x轴坐标
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // 支撑相x轴坐标

        if(dir == 1 || dir == 3){
          y1 = yep;
          y2 = ys;
          y3 = ys;
          y4 = ys;
        }else if(dir == 0){
          y1 = ys;
          y2 = ys;
          y3 = ys;
          y4 = yep;
        }

        if(dir == 1){
          x1 = xep_b;
          x2 = x2 - step;
          x3 = x3 + step;
          x4 = x4 + step;
        }else if(dir == 0){
          x1 = x1 + step;
          x2 = x2 + step;
          x3 = x3 - step;
          x4 = xep_b;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }

      // 2.LB
      if(  Ts*fai < t && t < 2*Ts*fai){
        sigma  = 2*PI*(t - Ts*fai)/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y轴坐标
        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // 摆动相x轴坐标
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // 支撑相x轴坐标

        if(dir == 1 || dir == 3){
          y1 = ys;
          y2 = ys;
          y3 = yep;
          y4 = ys;
        }else if(dir == 0){
          y1 = ys;
          y2 = yep;
          y3 = ys;
          y4 = ys;
        }

        if(dir == 1){
          x1 = x1 - step;
          x2 = x2 - step;
          x3 = xep_z;
          x4 = x4 + step;
        }else if(dir == 0){
          x1 = x1 + step;
          x2 = xep_z;
          x3 = x3 - step;
          x4 = x4 - step;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }
      // 3.LU
      if(2*Ts*fai < t && t < 3*Ts*fai){
        sigma  = 2*PI*(t - 2*Ts*fai)/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y轴坐标
        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // 摆动相x轴坐标
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // 支撑相x轴坐标

        if(dir == 1 || dir == 3){
          y1 = ys;
          y2 = yep;
          y3 = ys;
          y4 = ys;
        }else if(dir == 0){
          y1 = ys;
          y2 = ys;
          y3 = yep;
          y4 = ys;
        }

        if(dir == 1){
          x1 = x1 - step;
          x2 = xep_b;
          x3 = x3 + step;
          x4 = x4 + step;
        }else if(dir == 0){
          x1 = x1 + step;
          x2 = x2 + step;
          x3 = xep_b;
          x4 = x4 - step;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }
      // 4.RB
      if(3*Ts*fai < t && t < 4*Ts*fai){
        sigma  = 2*PI*(t - 3*Ts*fai)/(fai*Ts);
        yep    = yh*(1-cos(sigma))/2+ys;                    // y轴坐标
        xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI)+xs;    // 摆动相x轴坐标
        xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI)+xf;    // 支撑相x轴坐标

        if(dir == 1 || dir == 3){
          y1 = ys;
          y2 = ys;
          y3 = ys;
          y4 = yep;
        }else if(dir == 0){
          y1 = yep;
          y2 = ys;
          y3 = ys;
          y4 = ys;
        }

        if(dir == 1){
          x1 = x1 - step;
          x2 = x2 - step;
          x3 = x3 + step;
          x4 = xep_z;
        }else if(dir == 0){
          x1 = xep_z;
          x2 = x2 + step;
          x3 = x3 - step;
          x4 = x4 - step;
        }else if(dir == 2){
          x1 = XS;
          x2 = XS;
          x3 = XS;
          x4 = XS;
        }
      }    

      t = t + speed;

      IK_RUMove(x1,y1,DSD);
      IK_LUMove(x2,y2,DSD);
      IK_LBMove(x3,y3,DSD);
      IK_RBMove(x4,y4,DSD);
      delay(DSD);
    }

    if(WalkStatus){
      t = 0;
    }  
  }

  IK_ResetQuadruped();
  Serial.println(F("退出Walk模式..."));
}

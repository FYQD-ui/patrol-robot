#ifndef _ServeDrive_H
#define _ServeDrive_H

#include  <Arduino.h>
#include  <Wire.h>
#include  <Adafruit_PWMServoDriver.h>

#define ServoMax 470        //50Hz:最大PWM
#define ServoMin 100        //50Hz:最小PWM

class ServoDrive
{
    private:
      float  Angle[16];                           //16通道角度存储  
      
    public:
      ServoDrive();                               //构造函
      void  begin();                              //初始化
      void  Write(uint8_t servoNum,float pos);    //控制角度
      float Read(uint8_t servoNum);               //读取角度
      void  disattach(uint8_t servoNum);          //断开舵机
};

#endif

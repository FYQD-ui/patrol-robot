#ifndef _DCMotorDrive_H
#define _DCMotorDrive_H

#include  <Arduino.h>
#include "IOs.h"
//#include "../Dynamics/MotionControl.h"

#define RAIN1 4
#define RAIN2 5
#define RBIN1 2
#define RBIN2 15

#define LAIN1 12
#define LAIN2 13
#define LBIN1 14
#define LBIN2 27

class DCMotorDrive{
    public:
        DCMotorDrive();     // 构造函数
        void begin();       // 初始化

        void stop();        // 刹车
        
        void forword    (float rate);               // 前进
        void backword   (float rate);               // 后退
        void F_turnLeft (float rate,float pro);     // 左前转
        void F_turnRight(float rate,float pro);     // 右前转
        void B_turnLeft (float rate,float pro);     // 左后转
        void B_turnRight(float rate,float pro);     // 右后转

        void Test();
};

#endif

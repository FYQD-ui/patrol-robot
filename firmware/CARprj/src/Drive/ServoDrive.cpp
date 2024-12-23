#include "ServoDrive.h"

Adafruit_PWMServoDriver spwm = Adafruit_PWMServoDriver(0x40,Wire);

ServoDrive::ServoDrive(){
  Angle[16] = {};
}

void ServoDrive::begin(){
  Wire.begin();
  spwm.begin();           // I2C：SDA,SCL
  spwm.setPWMFreq(50);    // 50HZ更新频率，相当于20ms的周期

  Serial.println(F("[Servo]\tInit Success!"));
}

void ServoDrive::Write(uint8_t servoNum,float pos){
  pos = constrain(pos,0,180);       //限位:0~180
  Angle[servoNum] = pos;            //角度存储
  
  //此处修正各舵机角度偏差
  switch(servoNum){
    //髋关节
    case 1:   pos = 190 - pos;break;
    case 6:   pos = 5 +   pos;break;
    case 9:   pos = 185 - pos;break;
    case 14:  pos = pos;      break;
      
    //膝关节
    case 2:   pos = 190 - pos;break;
    case 5:   pos = 5   + pos;break;
    case 10:  pos = 190 - pos;break;
    case 13:  pos = 10 +  pos;break;
     
    //踝关节
    case 3:   pos = 190 - pos;break;
    case 4:   pos = 5 +   pos;break;
    case 11:  pos = 185 - pos;break;
    case 12:  pos = 10 +  pos;break;  

    case 0:   pos = pos + 15; break;
    case 15:  pos = pos + 5;  break;
        
    default:break;
  }
  
  uint16_t pwm = map(pos,0,180,ServoMin,ServoMax);    //角度->PWM
  spwm.setPWM(servoNum, 0, pwm); 
}

float ServoDrive::Read(uint8_t servoNum){
  return Angle[servoNum];
}

void ServoDrive::disattach(uint8_t servoNum){
  spwm.setPWM(servoNum, 0, 0);
}


//uint8_t servo_step = cur_pos<end_pos?2:-2;
//for(;cur_pos ！= end_pos;cur_pos += servo_step){
//  servo.write(cur_pos);
//  delay(15);
//}

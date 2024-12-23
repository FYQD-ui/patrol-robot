#ifndef _IMU_H
#define _IMU_H

#include <Arduino.h>
//#include <MPU6050.h>

#define AcceRatio 16384.0          //加速度计比例系数
#define GyroRatio 131.0            //陀螺仪比例系数

#define DEG 0 .017453292519943295769236907684886f    //角度转弧度
#define RAD 57.295779513082320876798154814105f	    //弧度转角度

class IMU
{
    private:
      unsigned long _lastTime = 0;
      
      int16_t ax, ay, az, gx, gy, gz;                     //加速度计陀螺仪原始数据
      float aax=0,aay=0,aaz=0;                            //角度变量
      float agx=0,agy=0,agz=0;                            //角度变量
      long  axo=0,ayo=0,azo=0;                            //加速度计偏移量
      long  gxo=0,gyo=0,gzo=0;                            //陀螺仪偏移量
       
      float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};  //x,y轴采样队列
      long aax_sum, aay_sum,aaz_sum;                      //x,y轴采样和
       
      float a_x[10]={0}, a_y[10]={0},a_z[10]={0};
      float g_x[10]={0} ,g_y[10]={0},g_z[10]={0};         //加速度计协方差计算队列
      float Px=1, Rx, Kx, Sx, Vx, Qx;                     //x轴卡尔曼变量
      float Py=1, Ry, Ky, Sy, Vy, Qy;                     //y轴卡尔曼变量
      float Pz=1, Rz, Kz, Sz, Vz, Qz;                     //z轴卡尔曼变量      

      float Ax_offset = 0,Ay_offset = 0 , Az_offset = 0;  //初始偏移角度    
      float SumX = 0,     SumY = 0,       SumZ = 0;

      /* ------------------------------ DMP ------------------------------ */
      bool      dmpReady = false; 
      uint8_t   devStatus;    
      uint8_t   mpuIntStatus;     // 保存来自MPU的实际中断状态字节
      uint8_t   fifoBuffer[64];   // FIFO存储缓冲区    
      uint16_t  packetSize;       // 预期的DMP数据包大小（默认值为42字节）
      uint16_t  fifoCount;        // FIFO中当前所有字节的计数

      float ypr[3];               // [yaw, pitch, roll]   yaw/pitch/roll

    public:

      float Ag[3],Ac[3],Gy[3];

      void begin();

      /* ---------------------------- 偏差校准 ------------------------------ */
      void Acc_Gyro_Calibration(uint16_t times);          //初始采样
      void Ypr_Calibration(uint16_t times);               //偏移校准

      void Attitude_Update();                            //姿态解算+卡尔曼滤波
      
      void InfosPrint(bool ag,bool ac,bool gy);           //串口数据调试

      /* ------------------------------ DMP ------------------------------ */
      void InitDMP();
      void DMP_Update();
};

#endif

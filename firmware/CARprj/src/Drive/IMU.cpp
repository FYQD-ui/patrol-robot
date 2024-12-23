#include "IMU.h"
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 IMU_MPU6050;

void IMU::begin(){
  Wire.begin();             // IIC初始化
  Serial.println("Initializing MPU6050...");
  
  IMU_MPU6050.initialize(); // MPU初始化

  Serial.println(IMU_MPU6050.testConnection() ? F("[IMU]\tInit Success!") : F("[IMU]\tInit Failed!"));  //连接测试

  InitDMP();

  // 偏差校准
  Acc_Gyro_Calibration(500);
	Ypr_Calibration     (500);
}

/* ------------------------------ DMP ------------------------------ */
void IMU::InitDMP(){
  // 1.加载和配置DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = IMU_MPU6050.dmpInitialize();

  // 2.提供所处位置的陀螺偏移量，按最小灵敏度进行缩放
  IMU_MPU6050.setXGyroOffset(250);
  IMU_MPU6050.setYGyroOffset(76);
  IMU_MPU6050.setZGyroOffset(-85);
  IMU_MPU6050.setZAccelOffset(1788);

  // 3.工作情况检验
  if (devStatus == 0) {
    // 打开DMP
    Serial.println(F("Enabling DMP..."));
    IMU_MPU6050.setDMPEnabled(true);

    //设置DMP就绪标志
    dmpReady = true;

    // 获取预期DMP数据包大小，以便后续比较
    packetSize = IMU_MPU6050.dmpGetFIFOPacketSize();
    Serial.println(F("DMP Ready!"));

  } else {
    //Error:
    //  1=初始内存加载失败
    //  2=DMP配置更新失败
    //（如果要中断，代码通常为1）
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

/* ---------------------------------------------------------------- */
Quaternion q;               // [w, x, y, z]         四元数
VectorFloat gravity;        // [x, y, z]            重力矢量
/* ---------------------------------------------------------------- */

void IMU::DMP_Update(){
  // 失败，则无任何操作
  if (!dmpReady) return;

  fifoCount       = IMU_MPU6050.getFIFOCount();           // 获取当前FIFO计数

  // 检查溢出
  if (fifoCount == 1024) {       
    IMU_MPU6050.resetFIFO();                              // 重置
    Serial.println(F("FIFO overflow!"));
  } else{                                   // 否则，请检查DMP数据就绪中断
    while (fifoCount < packetSize) fifoCount = IMU_MPU6050.getFIFOCount();  // 等待正确的可用数据长度，应该是很短的等待时间

    IMU_MPU6050.getFIFOBytes(fifoBuffer, packetSize);                       // 从FIFO读取数据包
    fifoCount -= packetSize;

    IMU_MPU6050.dmpGetQuaternion  (&q, fifoBuffer);     // 获取四元数
    IMU_MPU6050.dmpGetGravity     (&gravity, &q);       // 获取重力矢量
    IMU_MPU6050.dmpGetYawPitchRoll(ypr, &q, &gravity);  // 获取姿态角
    
    Ag[0] = ypr[1] * RAD - Ax_offset;  //pitch
    Ag[1] = ypr[2] * RAD - Ay_offset;  //roll
    Ag[2] = ypr[0] * RAD - Az_offset;  //yaw
  }
}

/* ------------------------------ 偏差校准 ------------------------------ */
void IMU::Acc_Gyro_Calibration(uint16_t times = 200){  
  
  Serial.println(F("Acc_Gyro_Calibration in progress..."));

  for(uint16_t i=0;i<times;i++){                            //采样次数
      IMU_MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
      axo += ax; ayo += ay; azo += az;                      //采样和
      gxo += gx; gyo += gy; gzo += gz; 
  } 
  axo /= times; ayo /= times; azo /= times;                 //计算加速度计偏移
  gxo /= times; gyo /= times; gzo /= times;                 //计算陀螺仪偏移
}

void IMU::Ypr_Calibration(uint16_t times = 500){

  Serial.println(F("Ypr_Calibration in progress..."));
  Serial.printf("DMP Code:%d\n",dmpReady);

  SumX = 0; SumY = 0; SumZ = 0;

	for(uint16_t i = 0;i < times;i++){
    if(dmpReady)  {DMP_Update();}
    else          {Attitude_Update();}
		  
		SumX += Ag[0]; SumY += Ag[1]; SumZ += Ag[2];
	}

	Ax_offset += SumX / times;
  Ay_offset += SumY / times;
  Az_offset += SumZ / times;

}

/* ------------------------- 姿态解算 + 卡尔曼滤波------------------------- */
void IMU::Attitude_Update(){
  uint8_t n_sample  = 8;
  unsigned long now = millis();                         //当前时间(ms)

  float dt  = (now - _lastTime) / 1000.0;               //微分时间(s)
  _lastTime = now;                                      //上一次采样时间(ms)

  IMU_MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值

  float accx = ax / AcceRatio;                          //x轴加速度
  float accy = ay / AcceRatio;                          //y轴加速度
  float accz = az / AcceRatio;                          //z轴加速度

  /* ########################## 姿态解算 ########################## */
  // 计算 1
  aax = atan(accy / accz) * 180 / PI;       //y轴对于z轴的夹角
  aay = atan(accx / accz) * 180 / PI;       //x轴对于z轴的夹角
  // aaz = atan(accz / accy) * 180 / PI;       //z轴对于y轴的夹角

  // 计算 2
  // aax = atan(accx / sqrt(accz*accz + accy*accy)) * 180 / PI;                //x轴对于z轴的夹角
  // aay = atan(accy / sqrt(accz*accz + accx*accx)) * 180 / PI;                //y轴对于z轴的夹角
  aaz = atan(accz / sqrt(accx*accx + accy*accy)) * 180 / PI;                //z轴对于y轴的夹角
  /* ############################################################## */

  aax_sum = 0;                                          //对于加速度计原始数据的滑动加权滤波算法
  aay_sum = 0;
  aaz_sum = 0;

  for(uint8_t i=1;i<n_sample;i++){
      aaxs[i-1] = aaxs[i];aax_sum += aaxs[i] * i;
      aays[i-1] = aays[i];aay_sum += aays[i] * i;
      aazs[i-1] = aazs[i];aaz_sum += aazs[i] * i;
  }
  
  aaxs[n_sample-1] = aax;
  aax_sum += aax * n_sample;
  aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0;      //角度调幅至0-90°

  aays[n_sample-1] = aay;                             //此处应用实验法取得合适的系数
  aay_sum += aay * n_sample;                          //本例系数为9/7
  aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0*(90/87);

  aazs[n_sample-1] = aaz; 
  aaz_sum += aaz * n_sample;
  aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;

  float gyrox = - (gx-gxo) / GyroRatio * dt;          //x轴角速度
  float gyroy = - (gy-gyo) / GyroRatio * dt;          //y轴角速度
  float gyroz = - (gz-gzo) / GyroRatio * dt;          //z轴角速度

  agx += gyrox;                                       //x轴角速度积分
  agy += gyroy;                                       //y轴角速度积分
  agz += gyroz;                                       //z轴角速度积分
  
  /* Kalman start */
  Sx = 0; Rx = 0;
  Sy = 0; Ry = 0;
  Sz = 0; Rz = 0;
  
  for(int i=1;i<10;i++){   
      //测量值平均值运算，即加速度平均值
      a_x[i-1] = a_x[i];Sx += a_x[i];
      a_y[i-1] = a_y[i];Sy += a_y[i];
      a_z[i-1] = a_z[i];Sz += a_z[i];
  }
  
  a_x[9] = aax;Sx += aax;Sx /= 10;                                           //x轴加速度平均值
  a_y[9] = aay;Sy += aay;Sy /= 10;                                           //y轴加速度平均值
  a_z[9] = aaz;Sz += aaz;Sz /= 10;                                           //z轴加速度平均值

  for(int i=0;i < 10;i++){
    Rx += sq(a_x[i] - Sx);
    Ry += sq(a_y[i] - Sy);
    Rz += sq(a_z[i] - Sz);
  }
  
  Rx = Rx / 9;                                        //得到方差
  Ry = Ry / 9;                        
  Rz = Rz / 9;

  Px = Px + 0.0025;                                   
  Kx = Px / (Px + Rx);               //计算卡尔曼增益
  agx = agx + Kx * (aax - agx);      //陀螺仪角度与加速度计速度叠加
  Px = (1 - Kx) * Px;                //更新p值

  Py = Py + 0.0025;
  Ky = Py / (Py + Ry);
  agy = agy + Ky * (aay - agy); 
  Py = (1 - Ky) * Py;

  Pz = Pz + 0.0025;
  Kz = Pz / (Pz + Rz);
  agz = agz + Kz * (aaz - agz); 
  Pz = (1 - Kz) * Pz;

  /* Kalman end */
  // 存储角度
  Ag[0] = agx - Ax_offset;      // Roll  		
  Ag[1] = agy - Ay_offset;      // Pitch										 					
  Ag[2] = agz - Az_offset;      // Yaw

  Ac[0] = accx - axo/AcceRatio;   Ac[1] = accy - ayo/AcceRatio;   Ac[2] = accz - azo/AcceRatio;   // 存储加速度
  Gy[0] = gyrox- gxo/GyroRatio;   Gy[1] = gyroy - gyo/GyroRatio;  Gy[2] = gyroz- gzo/GyroRatio;   // 存储角加速度
}

/*
* 串口调试
*/
void IMU::InfosPrint(bool ag,bool ac,bool gy){

  if(dmpReady)  {DMP_Update();ac = 0;gy = 0;}
  else          {Attitude_Update();}

	if(ag)	Serial.printf("P :%6.2f - R :%6.2f - Y :%6.2f   |   "   ,Ag[1],Ag[0],Ag[2]);
 	if(ac)	Serial.printf("Ax:%6.2f - Ay:%6.2f - Az:%6.2f   |   "   ,Ac[0],Ac[1],Ac[2]);
  if(gy)	Serial.printf("Gx:%6.2f - Gy:%6.2f - Gz:%6.2f\t"        ,Gy[0],Gy[1],Gy[2]);

  Serial.println();
}


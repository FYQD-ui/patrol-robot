#include <Arduino.h>

// 驱动函数
#include "Drive/DCMotorDrive.h"
#include <HardwareSerial.h>
#include "Drive/IMU.h"
#include "Drive/IOs.h"

// 运动学函数
#include "Dynamics/MotionControl.h"

// WIFI库
#include <WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_NeoPixel.h>

// 自动巡航头文件
#include "../src/Dynamics/AutoCruise.h"

// 模式定义
#define TRACK_MODE      0   // 履带模式
#define CRUISE_MODE     1   // 巡航模式
#define QUADRUPED_MODE  2   // 四足模式

uint8_t currentMode = TRACK_MODE; // 初始模式为履带模式

// WIFI信息
#define ssid      "Clgc"         // WIFI名称
#define password  "12345678"     // WIFI密码

// 子函数声明
void TCPServerHandler();     // TCP服务器处理函数
void CmdSwitch(char cmd);    // 命令处理函数
void BlinkLed(uint8_t pin, uint8_t times); // LED闪烁函数

// 类实例化
Robot           robot;
IMU             mpu;                  // MPU6050传感器
HardwareSerial  ServoSer = Serial1;

WiFiServer      server(81);           // 创建TCP服务器，端口81
WiFiClient      client;               // 当前连接的客户端

DCMotorDrive    DCm;                  // 直流电机组

extern Adafruit_NeoPixel pixels;      // NeoPixel灯


// 定义超声波模块的引脚
const int trigPin = 25; // 触发引脚
const int echoPin = 26; // 回波引脚

// 创建自动巡航实例
AutoCruise autoCruise(DCm, trigPin, echoPin);
// 变量定义
bool isWaiting = false;
unsigned long waitStartTime = 0;
unsigned long waitDuration = 10000; // 10秒
uint8_t zCount = 0; // 'z' 指令计数

WiFiClient client_Move;

void setup() {
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, LOW);
  // 串口初始化
  Serial.begin(115200);

  DCm.begin();
  robot.begin();

  // WIFI初始化
  Serial.println(F("** 配置WIFI..."));

  WiFi.mode(WIFI_AP);           // 设置为AP模式（无线接入点模式）
  WiFi.softAP(ssid, password);  // 设置AP的SSID和密码
  server.begin();               // 启动服务器，开始监听客户端连接

  Serial.print(F("  ----- AP--IPv4: "));
  Serial.println(WiFi.softAPIP());
  Serial.println(F("[WIFI]\t初始化成功!"));
  // 初始化自动巡航
  autoCruise.begin();

  /*----------------------------------------------------------*/
  Serial.println();
  Serial.println(F("|*****************************|"));
  Serial.println(F("|***>   小车系统启动!      <***|"));
  Serial.println(F("|*****************************|"));
  Serial.println();
  /*----------------------------------------------------------*/
  BlinkLed(LEDpin, 2);
  digitalWrite(LEDpin, HIGH);

  // 初始化NeoPixel灯
  pixels.begin();
  pixels.show(); // 初始化所有像素为关闭状态

   // 如果默认是自主巡航模式，开始等待计时
    if (currentMode == CRUISE_MODE) {
        isWaiting = true;
        waitStartTime = millis();
        // 设置NeoPixel灯为绿色
        pixels.fill(pixels.Color(0, 255, 0));
        pixels.show();
        Serial.println("进入自主巡航模式，等待10秒后开始巡航...");
    }
}

void loop() {
  // 处理串口命令
  if (Serial.available()) {
    char cmd = Serial.read();
    BlinkLed(LEDpin, 1); // 命令接收指示
    CmdSwitch(cmd);
  }

  // 处理TCP命令
  TCPServerHandler(); // 调用TCP服务器处理函数

  // 处理巡航模式的等待
  // 根据当前模式更新对应的功能
    if (currentMode == CRUISE_MODE) {
        if (isWaiting) {
            // 处于等待状态，判断是否到达等待时间
            if (millis() - waitStartTime >= waitDuration) {
                isWaiting = false;
                // 开始自动巡航
                autoCruise.start();
                Serial.println("等待结束，开始自主巡航");
                // 可选择改变NeoPixel灯的颜色，表示巡航开始
                pixels.fill(pixels.Color(0, 0, 255)); // 蓝色
                pixels.show();
            }
        } else {
            // 更新自动巡航
            autoCruise.update();
        }
    }

  // 其他循环内容
}

/* ------------------ 指令控制 ------------------ */
void CmdSwitch(char cmd){
  // 命令接收指示
  BlinkLed(LEDpin, 1);

  // 处理 'z' 指令，连续两次输入切换到四足模式
  static unsigned long lastZTime = 0;
  if (cmd == 'z') {
    unsigned long currentTime = millis();
    if (currentTime - lastZTime < 1000) { // 如果两次 'z' 间隔小于1秒
      zCount++;
    } else {
      zCount = 1;
    }
    lastZTime = currentTime;

    if (zCount >= 2) {
      // 切换到四足模式
      currentMode = QUADRUPED_MODE;
      Serial.println("切换到四足模式");
      DCm.stop();            // 停止履带运动
      isWaiting = false;     // 停止等待
      pixels.fill(pixels.Color(255, 0, 255)); // 设置NeoPixel灯为紫色
      pixels.show();
      zCount = 0; // 重置计数
    }
    return;
  }

  // 处理 'x' 指令，切换履带模式和巡航模式
  if (cmd == 'x') {
    if (currentMode == TRACK_MODE) {
      // 从履带模式切换到巡航模式
      currentMode = CRUISE_MODE;
      Serial.println("切换到自主巡航模式");
      DCm.stop();            // 停止履带运动

      // 开始等待计时
      isWaiting = true;
      waitStartTime = millis();
      // 设置NeoPixel灯为绿色
      pixels.fill(pixels.Color(0, 255, 0));
      pixels.show();
      Serial.println("进入自主巡航模式，等待10秒后开始巡航...");
    } else if (currentMode == CRUISE_MODE) {
      // 从巡航模式切换到履带模式
      currentMode = TRACK_MODE;
      Serial.println("切换到履带模式");
      // 在这里添加停止自动巡航的代码
      autoCruise.stop();     // 停止自动巡航
      isWaiting = false;     // 停止等待
      DCm.stop();            // 确保电机停止
      // 关闭NeoPixel灯
      pixels.clear();
      pixels.show();
    } else if (currentMode == QUADRUPED_MODE) {
      // 从四足模式切换回履带模式
      currentMode = TRACK_MODE;
      Serial.println("切换到履带模式");
      robot.WalkStatus = 0;
      robot.TrotStatus = 0;
      // 关闭NeoPixel灯
      pixels.clear();
      pixels.show();
    }
    return; // 已处理命令，退出函数
  }

  // 根据当前模式处理其他命令
  if (currentMode == QUADRUPED_MODE) {
    switch (cmd) {
      case 'w':
        Serial.println("Starting Walk...");
        robot.Walk();
        break;
      case 't':
        Serial.println("Starting Trot...");
        robot.Trot();
        break;
      case 'p':
        Serial.println("Performing PosAction1...");
        robot.PosAction1();
        break;
      case 's':
        Serial.println("Stopping all actions...");
        robot.WalkStatus = 0;
        robot.TrotStatus = 0;
        // 其他停止标志
        break;
      default:
        Serial.println("未知指令（四足模式）");
        break;
    }
  } else if (currentMode == TRACK_MODE) {
    // 履带模式下处理指令
    switch(cmd){
      case '0':
        // ALLPOWOFF();             // 如果有全局关闭功能，可以在这里调用
        DCm.stop();
        robot.WalkStatus = 0;
        robot.TrotStatus = 0;
        // 关闭NeoPixel灯
        pixels.clear();
        pixels.show();
        Serial.println("所有动力已关闭");
        break;

      case '5':
        DCm.stop();              // 停止电机
        robot.ResetTrack(0);     // 重置履带
        break;

      case '8':
        DCm.forword(1);          // 前进
        break;

      case '2':
        DCm.backword(1);         // 后退
        break;

      case '7':
        DCm.F_turnLeft(1, 0.5);  // 左转
        break;

      case '9':
        DCm.F_turnRight(1, 0.5); // 右转
        break;

      case '1':
        DCm.B_turnLeft(1, 0.5);  // 后左转
        break;

      case '3':
        DCm.B_turnRight(1, 0.5); // 后右转
        break;

      case 'f':
        DCm.Test();              // 电机测试
        break;

      default:
        Serial.println(F("未知指令（履带模式）"));
        break;
    }
  } else if (currentMode == CRUISE_MODE) {
    // 巡航模式下处理指令
    switch(cmd){
      case '0':
        // ALLPOWOFF();             // 如果有全局关闭功能，可以在这里调用
        DCm.stop();
        robot.WalkStatus = 0;
        robot.TrotStatus = 0;
        autoCruise.stop();     // 停止自动巡航
        // 关闭NeoPixel灯
        pixels.clear();
        pixels.show();
        Serial.println("所有动力已关闭");
        break;

      // 如果需要在巡航模式下处理其他指令，可以在这里添加

      default:
        Serial.println(F("未知指令（巡航模式）"));
        break;
    }
  }
}

/*
 * TCP服务器处理函数
 * 用于接收客户端的命令并进行处理
 */
void TCPServerHandler() {
  // 检查服务器是否有新的客户端尝试连接
  if (server.hasClient()) {
    // 如果当前没有客户端连接，接受新的客户端
    if (!client || !client.connected()) {
      client = server.available(); // 接受新的客户端连接
      Serial.println(F("新客户端已连接"));
      BlinkLed(LEDpin, 3); // 指示有新的客户端连接
    } else {
      // 已经有一个客户端连接，拒绝新的连接
      WiFiClient newClient = server.available(); // 获取新的客户端
      newClient.stop(); // 关闭新的客户端连接
      Serial.println(F("已有客户端连接，拒绝新的连接"));
    }
  }

  // 如果有客户端连接，处理客户端数据
  if (client && client.connected()) {
    // 检查客户端是否有可用的数据
    if (client.available()) {
      // 循环读取所有可用的数据
      while (client.available()) {
        char cmd = client.read(); // 从客户端读取一个字节的命令
        // 忽略换行符和回车符
        if (cmd != '\n' && cmd != '\r') {
          BlinkLed(LEDpin, 1); // 命令接收指示
          Serial.print(F("接收到TCP命令："));
          Serial.println(cmd);
          CmdSwitch(cmd); // 调用命令处理函数处理命令
        }
      }
    }

    // 检查客户端是否已断开连接
    if (!client.connected()) {
      client.stop(); // 关闭客户端连接
      Serial.println(F("客户端已断开连接"));
      digitalWrite(LEDpin, LOW); // 关闭指示灯
    }
  }
}
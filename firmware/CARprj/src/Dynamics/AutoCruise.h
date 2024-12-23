// AutoCruise.h
#ifndef AUTO_CRUISE_H
#define AUTO_CRUISE_H

#include <Arduino.h>
#include "Drive/DCMotorDrive.h"  // 包含直流电机驱动头文件

// 自动巡航类
class AutoCruise {
public:
    // 构造函数，传入直流电机驱动对象和超声波传感器引脚
    AutoCruise(DCMotorDrive& motorDrive, int trigPin, int echoPin);

    void begin();        // 初始化函数
    void update();       // 更新自动巡航状态
    void start();        // 启动自动巡航
    void stop();         // 停止自动巡航
    bool isRunning();    // 检查自动巡航是否正在运行

private:
    long readUltrasonicDistance();  // 读取超声波距离

    DCMotorDrive& DCm;  // 引用直流电机驱动对象
    int trigPin;        // 超声波触发引脚
    int echoPin;        // 超声波回波引脚

    bool autoCruiseEnabled;  // 自动巡航使能标志
    int attempt;             // 尝试次数

    unsigned long lastUpdateTime;  // 上次更新的时间
    enum State { MOVING_FORWARD, BACKING_UP, TURNING, STOPPED };  // 状态枚举
    State state;                    // 当前状态
    unsigned long stateStartTime;   // 状态开始的时间
};

#endif // AUTO_CRUISE_H

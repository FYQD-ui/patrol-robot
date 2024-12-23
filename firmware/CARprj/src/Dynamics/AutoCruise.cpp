// AutoCruise.cpp
#include "AutoCruise.h"

AutoCruise::AutoCruise(DCMotorDrive& motorDrive, int trigPin, int echoPin)
    : DCm(motorDrive), trigPin(trigPin), echoPin(echoPin), autoCruiseEnabled(false), attempt(0), state(STOPPED), lastUpdateTime(0), stateStartTime(0)
{
}

void AutoCruise::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void AutoCruise::start() {
    autoCruiseEnabled = true;
    attempt = 0;
    state = MOVING_FORWARD;
    stateStartTime = millis();
}

void AutoCruise::stop() {
    autoCruiseEnabled = false;
    DCm.stop();
    state = STOPPED;
}

bool AutoCruise::isRunning() {
    return autoCruiseEnabled;
}

void AutoCruise::update() {
    if (!autoCruiseEnabled) return;

    unsigned long currentTime = millis();

    switch (state) {
        case MOVING_FORWARD: {
            long distance = readUltrasonicDistance();
            if (distance > 0 && distance < 50) {
                attempt++;
                if (attempt <= 5) {
                    Serial.println("检测到障碍物，开始避障...");
                    DCm.stop();
                    delay(500); // 可选延时
                    DCm.backword(1);
                    state = BACKING_UP;
                    stateStartTime = currentTime;
                } else {
                    // 停止运动
                    Serial.println("连续5次避障失败，停止运动");
                    stop();
                }
            } else {
                attempt = 0;
                DCm.forword(1);
            }
            break;
        }
        case BACKING_UP:
            if (currentTime - stateStartTime >= 3000) {
                DCm.stop();
                delay(500); // 可选延时
                DCm.F_turnRight(1, 0.5);
                state = TURNING;
                stateStartTime = currentTime;
            }
            break;
        case TURNING:
            if (currentTime - stateStartTime >= 5000) {
                DCm.stop();
                delay(500); // 可选延时
                DCm.forword(1);
                state = MOVING_FORWARD;
                stateStartTime = currentTime;
            }
            break;
        case STOPPED:
            DCm.stop();
            break;
    }
}

long AutoCruise::readUltrasonicDistance() {
    long duration;
    int distance;

    // 触发超声波传感器
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 读取回波引脚
    duration = pulseIn(echoPin, HIGH, 30000); // 超时30毫秒

    if (duration == 0) {
        // 未检测到回波，返回 -1
        return -1;
    } else {
        // 计算距离（单位：厘米）
        distance = duration * 0.034 / 2;
        return distance;
    }
}

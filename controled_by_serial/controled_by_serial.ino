#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>
#include "InverseKinematics.h"
#include <map>
#include <string>


#define SERVO_FREQ 50
#define SERVOMIN 90
#define SERVOMAX 475

#define servo1 0
#define servo2 4
#define servo3 8
#define output_en 26

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
bool en = true;
int last_angle = 90;
int step = 50;
float distance = 0.1;

// 初始化Machine物件，參數根據平台實際尺寸調整
Machine machine(9.7, 10.5, 8.15, 10.5);

// 初始高度
float hz = 4.25;

// 初始法向量
float nx = 0;
float ny = 0;

struct Data {
    float pitch;   // 傾斜資訊
    float height_change; // 高度資訊
};

// 使用 map 來存儲距離對應的數據
std::map<float, Data> distanceDataMap;


void setup() {
  Serial.begin(115200);
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);

  pinMode(output_en, OUTPUT);

  pwm1.setPWM(servo1, 0, (SERVOMIN + SERVOMAX) / 2);
  pwm1.setPWM(servo2, 0, (SERVOMIN + SERVOMAX) / 2);
  pwm1.setPWM(servo3, 0, (SERVOMIN + SERVOMAX) / 2);

  Serial.println("Ready to receive commands...");
}

void loop() {
  // Check for serial input
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();

    // 格式: pitch,height_change
    int commaIndex = message.indexOf(',');
    float pitch = message.substring(0, commaIndex).toFloat();
    float height_change = message.substring(commaIndex + 1).toFloat();


    // 以當前編碼器位置為鍵，儲存傾斜和高度資訊
    float encoderPosition = readEncoder();  // 當前距離
    cleanOldData(encoderPosition);          // 清理舊數據

    Data currentData;
    if (distanceDataMap.find(encoderPosition) != distanceDataMap.end()) {
        // 若找到精確匹配
        currentData = distanceDataMap[encoderPosition];
    } else {
        // 找不到時，使用最近的數據
        currentData = findNearestData(encoderPosition);
    }

    // 使用 currentData 的數據執行其他操作
    nx = sin(currentData.pitch);  // pitch 映射到 nx
    hz += currentData.height_change;



    // 使用逆運動學庫計算角度
    double thetaA = machine.theta(A, hz, nx, ny);
    double thetaB = machine.theta(B, hz, nx, ny);
    double thetaC = machine.theta(C, hz, nx, ny);

    // 檢查角度是否在伺服馬達的有效範圍內
    thetaA = constrain(thetaA, 0, 180);  // 確保角度在 0-180 度之間
    thetaB = constrain(thetaB, 0, 180);
    thetaC = constrain(thetaC, 0, 180);

    // 計算伺服的PWM值
    int pulseA = angle2pulse(thetaA);
    int pulseB = angle2pulse(thetaB);
    int pulseC = angle2pulse(thetaC);

    // 設置伺服馬達的PWM值
    pwm1.setPWM(servo1, 0, pulseA+10);
    pwm1.setPWM(servo2, 0, pulseB-12);
    pwm1.setPWM(servo3, 0, pulseC+19);
  }
}

// Function to convert angle to PWM value
int angle2pulse(double angle) {
  int servo_pwm = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
  return servo_pwm;
}

void cleanOldData(float currentPosition, float maxDistance = 1.0) {
    // 使用迭代器遍歷 map
    for (auto it = distanceDataMap.begin(); it != distanceDataMap.end(); ) {
        // 若鍵值（距離）與當前位置相差超過 maxDistance，就刪除該項
        if (abs(it->first - currentPosition) > maxDistance) {
            it = distanceDataMap.erase(it); // 刪除並更新迭代器
        } else {
            ++it; // 否則前進到下一個項目
        }
    }
}


Data findNearestData(float encoderPosition) {
    // 若 map 內沒有數據，則返回默認值
    if (distanceDataMap.empty()) {
        return {0, 0}; // 返回默認數據，避免程序崩潰
    }

    // 使用 lower_bound 查找不小於 encoderPosition 的鍵
    auto lower = distanceDataMap.lower_bound(encoderPosition);

    // 特殊情況：如果 lower 是第一個元素，或者剛好等於 encoderPosition
    if (lower == distanceDataMap.begin()) {
        return lower->second;
    }
    // 特殊情況：如果超過 map 的最後一個元素
    if (lower == distanceDataMap.end()) {
        return std::prev(lower)->second;
    }

    // 比較 lower 和前一個元素，找到較接近的鍵值
    auto prev = std::prev(lower);
    if ((encoderPosition - prev->first) < (lower->first - encoderPosition)) {
        return prev->second; // 返回前一個更接近的數據
    } else {
        return lower->second; // 返回當前的 lower 數據
    }
}


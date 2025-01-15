#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>
#include "InverseKinematics.h"

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

// 初始化Machine物件，參數根據平台實際尺寸調整
Machine machine(9.7, 10.5, 8.15, 10.5);

// 初始高度
float hz = 4.25;

// 初始法向量
float nx = 0;
float ny = 0;

void setup() {
  Serial.begin(115200);
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);

  pinMode(output_en, OUTPUT);

  pwm1.setPWM(servo1, 0, (SERVOMIN + SERVOMAX) / 2);
  pwm1.setPWM(servo2, 0, (SERVOMIN + SERVOMAX) / 2);
  pwm1.setPWM(servo3, 0, (SERVOMIN + SERVOMAX) / 2);

  Serial.println("Ready to receive joystick and lever commands...");
}

void loop() {
  // Check for serial input
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();

    // 處理搖桿數據
    if (message.startsWith("joystick")) {
      // 格式: joystick: x_value, y_value
      message.replace("joystick: ", "");
      int commaIndex = message.indexOf(',');
      float x_value = message.substring(0, commaIndex).toFloat();
      float y_value = message.substring(commaIndex + 1).toFloat();

      // 將搖桿輸入範圍從 -1.0 到 1.0 映射到 nx 和 ny
      nx = x_value;  // 假設搖桿 x 軸輸入映射到 nx
      ny = y_value;  // 假設搖桿 y 軸輸入映射到 ny

      Serial.print("Joystick Control - nx: ");
      Serial.print(nx);
      Serial.print(", ny: ");
      Serial.println(ny);
    }

    // 處理拉桿數據
    if (message.startsWith("lever")) {
      // 格式: lever: height_value
      message.replace("lever: ", "");
      float height_value = message.toFloat();

      // 將拉桿範圍從 0 到 100 映射到平台的高度範圍
      hz = map(height_value, 0, 100, 7, 18.6);  // 假設最大高度是 10

      Serial.print("Lever Control - Height (hz): ");
      Serial.println(hz);
    }

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
    pwm1.setPWM(servo1, 0, pulseA);
    pwm1.setPWM(servo2, 0, pulseB);
    pwm1.setPWM(servo3, 0, pulseC);
  }
}

// Function to convert angle to PWM value
int angle2pulse(double angle) {
  int servo_pwm = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
  return servo_pwm;
}

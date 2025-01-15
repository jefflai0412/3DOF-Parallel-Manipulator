#include "MPU9250.h" // 讀IMU值的library
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

#define IMU_AD0 25
#define output_en 26

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
MPU9250 IMU;

bool en = true;
int hz = 100; // IMU資料的頻率
float qx, qy, qz, qw; // Quaternion values
float roll, pitch, yaw; // Roll, Pitch, Yaw values
float init_qx, init_qy, init_qz, init_qw = 0;
int init_counter = 0;
int init_time = 800;

// 初始化Machine物件，參數根據平台實際尺寸調整
Machine machine(9.5, 10.5, 7, 11);

// 初始高度
float platform_height = 15.25;  // 調整你的高度
float nx = 0, ny = 0;  // 初始法向量

void setup() {
  pinMode(IMU_AD0, OUTPUT);
  digitalWrite(IMU_AD0, HIGH);

  Serial.begin(115200);
  Wire.begin();
  
  // 初始化伺服馬達
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);

  pinMode(output_en, OUTPUT);

  pwm1.setPWM(servo1, 0, (SERVOMIN + SERVOMAX) / 2);
  pwm1.setPWM(servo2, 0, (SERVOMIN + SERVOMAX) / 2);
  pwm1.setPWM(servo3, 0, (SERVOMIN + SERVOMAX) / 2);

  // 設置 MPU9250 IMU
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  if (!IMU.setup(0x68, setting)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection.");
      delay(5000);
    }
  }

  IMU.selectFilter(QuatFilterSel::MADGWICK);
  IMU.setFilterIterations(20);
  
  // 校準 IMU
  Serial.println("Calibrating IMU...");
  IMU.calibrateAccelGyro();
  delay(1000);
}

void loop() {
  // 更新IMU數據
  if (IMU.update()) {
    // 讀取四元數
    qx = IMU.getQuaternionX();
    qy = IMU.getQuaternionY();
    qz = IMU.getQuaternionZ();
    qw = IMU.getQuaternionW();

    // 計算 Roll, Pitch, Yaw
    roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    pitch = asin(2 * (qw * qy - qz * qx));
    yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

   
    // 將 Roll, Pitch, Yaw 從弧度轉換為度
    float roll_deg = roll * (180.0 / PI);
    float pitch_deg = pitch * (180.0 / PI);

    // 將 Pitch 和 Roll 映射到 nx 和 ny
    nx = sin(pitch);
    ny = sin(roll);

    // 使用逆運動學計算伺服馬達角度
    double thetaA = machine.theta(A, platform_height, nx, ny);
    double thetaB = machine.theta(B, platform_height, nx, ny);
    double thetaC = machine.theta(C, platform_height, nx, ny);

    // 檢查角度是否在有效範圍內
    thetaA = 180 - constrain(thetaA, 0, 180);
    thetaB = 180 - constrain(thetaB, 0, 180);
    thetaC = 180 - constrain(thetaC, 0, 180);

    // 計算伺服的PWM值
    int pulseA = angle2pulse(thetaA-7);
    int pulseB = angle2pulse(thetaB);
    int pulseC = angle2pulse(thetaC);

    // 設置伺服馬達的PWM值
    pwm1.setPWM(servo1, 0, pulseA-5);
    pwm1.setPWM(servo2, 0, pulseB);
    pwm1.setPWM(servo3, 0, pulseC);

    // 輸出 Roll 和 Pitch 值 (度)
    Serial.print("Roll: "); Serial.print(roll_deg, 3); Serial.print(", ");
    Serial.print("Pitch: "); Serial.print(pitch_deg, 3); Serial.println();
  }

  // 控制更新頻率
  delay(1000 / hz);
}

// Function to convert angle to PWM value
int angle2pulse(double angle) {
  int servo_pwm = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
  return servo_pwm;
}

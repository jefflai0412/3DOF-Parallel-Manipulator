#include "MPU9250.h" // IMU library
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>

#define servo1 0
#define servo2 4
#define servo3 8

#define IMU_AD0 25
#define output_en 26

MPU9250 IMU;

bool en = true;
int hz = 100; // IMU data update frequency
double qx, qy, qz, qw; // Quaternion values
double roll, pitch, yaw; // Roll, Pitch, Yaw values

unsigned long lastTime = 0;  // Time tracker for FPS
float fps = 0;               // FPS value

void setup() {
  pinMode(IMU_AD0, OUTPUT);
  digitalWrite(IMU_AD0, HIGH);

  Serial.begin(115200); // Start serial communication at 115200 baud
  Wire.begin();

  // Configure MPU9250 IMU settings
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
  IMU.setFilterIterations(50);

  // Calibrate IMU
  Serial.println("Calibrating IMU...");
  IMU.calibrateAccelGyro();
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();  // Current time in milliseconds
  if (IMU.update()) {
    // Calculate FPS
    fps = 1000.0 / (currentTime - lastTime);
    lastTime = currentTime;

    // Get quaternion values
    qx = IMU.getQuaternionX();
    qy = IMU.getQuaternionY();
    qz = IMU.getQuaternionZ();
    qw = IMU.getQuaternionW();

    // Calculate Roll, Pitch, Yaw from quaternion
    roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    pitch = asin(2 * (qw * qy - qz * qx));
    yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

    // Output Roll, Pitch, Yaw, and FPS
    Serial.print(roll);
    Serial.print(",");
    Serial.println(-pitch);
//    Serial.print(", FPS: ");
//    Serial.println(fps);
  }
}
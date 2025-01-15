#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>
#include <cmath>
#include "InverseKinematics.h"
#include <map>
#include <string>
#include "MPU9250.h"

#define SERVO_FREQ 50
#define SERVOMIN 90
#define SERVOMAX 475
#define servo1 0
#define servo2 4
#define servo3 8
#define output_en 26

#define BUFFER_SIZE 256

// IMU definitions
#define IMU_AD0 25
MPU9250 IMU;

// Constants for distance calculations
const float wheelDiameter = 3.0;
const int pulsesPerRevolution = 1000;
const double distancePerPulse = (PI * wheelDiameter) / pulsesPerRevolution;
const int dataOffset = 100;
const double dataGap = 1;
const int array_size = dataOffset / dataGap;

volatile long pulseCount = 0;
double distanceTraveled = 0;
double lastDistance = 0;

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Machine machine(9.5, 10.5, 7, 11);

float init_height = 14.5;
float height = 15.5;
float nx = 0;
float ny = 0;
double pitch_deg;   // Pitch in degrees
double pitch_rad = 0;  // Pitch in radians
float height_change;
float pitch_array_deg[array_size];  // Pitch array in degrees
float height_array[array_size];

// message handling vars
char serialBuffer[BUFFER_SIZE];  // Buffer to store serial data
int bufferIndex = 0;             // Index to track the buffer

// Circular buffer index
int currentIndex = 0;
int cameraIndex = currentIndex + 7;

// Angle variables for the servo task
double thetaA_deg = 90, thetaB_deg = 90, thetaC_deg = 90;

// IMU values
double qx, qy, qz, qw;
double imu_roll_rad, imu_pitch_rad, imu_yaw_rad;

void setup() {
  Serial.begin(115200);
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);

  pwm1.setPWM(servo1, 0, (SERVOMIN + SERVOMAX) / 2);
  pwm1.setPWM(servo2, 0, (SERVOMIN + SERVOMAX) / 2);
  pwm1.setPWM(servo3, 0, (SERVOMIN + SERVOMAX) / 2);

  pinMode(output_en, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), countPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(4), countPulse, RISING);

  // Create the IMU task
  xTaskCreatePinnedToCore(
      imuTask,
      "IMU Task",
      4096,
      NULL,
      1,
      NULL,
      0
  );

  // Create the serial reading task
  xTaskCreatePinnedToCore(
      serialReadTask,
      "Serial Read Task",
      2048,
      NULL,
      1,
      NULL,
      1
  );

  xTaskCreatePinnedToCore(
    servoControlTask,
    "Servo Control Task",
    8192,
    NULL,
    2,
    NULL,
    1
  );

  // Initialize arrays with default values
  for (int i = 0; i < array_size; i++) {
    pitch_array_deg[i] = 0;
    height_array[i] = init_height;
  }

  // IMU Initialization
  pinMode(IMU_AD0, OUTPUT);
  digitalWrite(IMU_AD0, HIGH);

  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  if (!IMU.setup(0x68, setting)) {
    Serial.println("MPU connection failed. Please check your connection.");
    while (1) delay(1000);
  }

  IMU.selectFilter(QuatFilterSel::MADGWICK);
  IMU.setFilterIterations(50);
  IMU.calibrateAccelGyro();
  delay(1000);
}

void loop() {
  noInterrupts();
  long pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  distanceTraveled += pulses * distancePerPulse;

  if (pulses > 0 && (distanceTraveled - lastDistance) >= dataGap * 0.999) {
    lastDistance = distanceTraveled;
    // Serial.print("distance:");
    // Serial.println(distanceTraveled);

    int indexToRead = (currentIndex) % array_size;
    pitch_rad = pitch_array_deg[indexToRead] * (M_PI / 180);  // Convert degrees to radians
    
    ny = sin(pitch_rad);
    height = height_array[indexToRead];

    thetaA_deg = machine.theta(A, height, nx, ny);
    thetaB_deg = machine.theta(B, height, nx, ny);
    thetaC_deg = machine.theta(C, height, nx, ny);

    thetaA_deg = 180 - constrain(thetaA_deg, 0, 180);
    thetaB_deg = 180 - constrain(thetaB_deg, 0, 180);
    thetaC_deg = 180 - constrain(thetaC_deg, 0, 180); 

    int insertIndex = (currentIndex - 0 + array_size) % array_size; 
    pitch_array_deg[insertIndex] = pitch_deg;
    height_array[insertIndex] = init_height + height_change;

    currentIndex = (currentIndex + 1) % array_size;
  }
}

// Servo control task
void servoControlTask(void *parameter) {
  for (;;) {
    int pulseA = angle2pulse(thetaA_deg);
    int pulseB = angle2pulse(thetaB_deg);
    int pulseC = angle2pulse(thetaC_deg);

    pwm1.setPWM(servo1, 0, pulseA - 7);
    pwm1.setPWM(servo2, 0, pulseB);
    pwm1.setPWM(servo3, 0, pulseC);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// IMU reading task
void imuTask(void *parameter) {
  unsigned long lastTime = 0;
  float fps = 0;

  while (1) {
    unsigned long currentTime = millis();
    
    if (IMU.update()) {
      fps = 1000.0 / (currentTime - lastTime);
      lastTime = currentTime;

      imu_roll_rad = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
      imu_pitch_rad = asin(2 * (qw * qy - qz * qx));
      imu_yaw_rad = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

      Serial.print(pitch_array_deg[(currentIndex + 0) % array_size]);  // Pitch in degrees
      Serial.print(",");
      Serial.println(0);
    }
    
    vTaskDelay(1000 / 100);
  }
}

// Serial data reading task
void serialReadTask(void *parameter) {
  while (1) { 
    while (Serial.available() > 0) {
      char incomingByte = Serial.read();
      if (incomingByte == ')') {
        serialBuffer[bufferIndex] = '\0';
        String message = String(serialBuffer);
        message.trim();
        int commaIndex = message.indexOf(',');
        pitch_deg = message.substring(0, commaIndex).toFloat();
        height_change = message.substring(commaIndex + 1).toFloat();

        memset(serialBuffer, 0, sizeof(serialBuffer));
        bufferIndex = 0;
      } else {
        if (bufferIndex < BUFFER_SIZE - 1) {
          serialBuffer[bufferIndex++] = incomingByte;
        }
        if (bufferIndex >= BUFFER_SIZE - 1) {
          Serial.println("Buffer overflow! Clearing buffer.");
          memset(serialBuffer, 0, sizeof(serialBuffer));
          bufferIndex = 0;
        }
      }
    }
    vTaskDelay(0);
  }
}

void countPulse() {
  pulseCount++;
}

int angle2pulse(double angle_deg) {
  return map((int)angle_deg, 0, 180, SERVOMIN, SERVOMAX);
}

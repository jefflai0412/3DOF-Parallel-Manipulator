#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>

#define SERVO_FREQ 50  // Servo frequency
#define SERVOMIN 90   // Minimum pulse length count (out of 4096)
#define SERVOMAX 475   // Maximum pulse length count (out of 4096)

#define servo1 0
#define servo2 4
#define servo3 8

#define output_en 26

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
bool en = true; // Enable/Disable state

int last_angle = 90;
int step = 50;

void setup() {
  Serial.begin(9600);
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pinMode(output_en, OUTPUT);

  pwm1.setPWM(servo1, 0, (SERVOMIN+SERVOMAX) /  2);
  pwm1.setPWM(servo2, 0, (SERVOMIN+SERVOMAX) /  2);
  pwm1.setPWM(servo3, 0, (SERVOMIN+SERVOMAX) /  2);

  Serial.println("Ready to receive commands...");
}

void loop() {
  // Check for serial input
  if (Serial.available()) {

    String message = Serial.readStringUntil('\n');  
    message.trim();  // Remove any extra whitespace
    int angle = message.toInt();  // Convert string to integer

    // Check if the angle is valid
    if (angle >= 0 && angle <= 180) {
      interpolate(angle);  // Call function to set the servo
      last_angle = angle;  // Store the last angle
      Serial.print("Setting servo to ");
      Serial.println(angle);
    } else {
      Serial.println("Invalid angle. Please enter a value between 0 and 180.");
    }

    Serial.println("Enter angle (0-180):");
  }
}

// Function to convert angle to PWM value
int angle2pulse(int angle) {
  int servo_pwm = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  return servo_pwm;
}

void interpolate(int target) {
  int last_pulse = angle2pulse(last_angle);
  int target_pulse = angle2pulse(target);

  int difference = target_pulse - last_pulse;

  for (int i=1; i<step+1; i++) {
    int pulse = last_pulse + (difference * i) / step;

    Serial.println(pulse);

    pwm1.setPWM(servo1, 0, pulse-8);  // Set PWM for the servo
    pwm1.setPWM(servo2, 0, pulse+15);  // Set PWM for the servo
    pwm1.setPWM(servo3, 0, pulse+0);  // Set PWM for the servo


    // delay(1);
  }

}


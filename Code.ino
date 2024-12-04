#include <AccelStepper.h>

// Motor pins
#define MX_EN_PIN 13
#define MX_STEP_PIN 14
#define MX_DIR_PIN 12

#define MY_EN_PIN 16
#define MY_STEP_PIN 4
#define MY_DIR_PIN 5

// Direction signs
#define XFORWARD -1
#define XBACK 1

#define YUP -1
#define YDOWN 1

// Constants
AccelStepper MotorX(AccelStepper::DRIVER, MX_STEP_PIN, MX_DIR_PIN);
AccelStepper MotorY(AccelStepper::DRIVER, MY_STEP_PIN, MY_DIR_PIN);

// Arc Parameters
const float STEPS_PER_MM_X = 4 * 847;// 843.6;
const float STEPS_PER_MM_Y = 6.666 * 1106.5;

void setup() {
  delay(10000);
  Serial.begin(115200);

  MotorX.setMaxSpeed(1000);
  MotorY.setMaxSpeed(1000);
  MotorX.setAcceleration(1000);
  MotorY.setAcceleration(1000);

  // Enable motors
  digitalWrite(MX_EN_PIN, HIGH);
  digitalWrite(MY_EN_PIN, HIGH);  

  MotorX.moveTo(XBACK * STEPS_PER_MM_X * 10);
  // MotorY.moveTo(YUP * STEPS_PER_MM_Y * 1);
}

void loop() {  
  MotorX.run();
  // MotorY.run();
}

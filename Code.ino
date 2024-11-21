#include <AccelStepper.h>

#define MX_EN_PIN 2
#define MX_STEP_PIN 3
#define MX_DIR_PIN 4

#define MY_EN_PIN 5
#define MY_STEP_PIN 6
#define MY_DIR_PIN 7

AccelStepper MotorX(AccelStepper::DRIVER, MX_STEP_PIN, MX_DIR_PIN);
AccelStepper MotorY(AccelStepper::DRIVER, MY_STEP_PIN, MY_DIR_PIN);

int lastX = 1000;
int lastY = 1000;

void setup() {
  MotorX.setMaxSpeed(1000);
  MotorX.setAcceleration(500);

  MotorY.setMaxSpeed(1000);
  MotorY.setAcceleration(500);

  digitalWrite(MX_EN_PIN, HIGH);
  digitalWrite(MY_EN_PIN, HIGH);

  MotorX.moveTo(lastX);
  MotorY.moveTo(lastY);
}

void loop() {
  if (MotorX.distanceToGo() == 0)
	  MotorX.moveTo(-MotorX.currentPosition());
  if (MotorY.distanceToGo() == 0)
    MotorY.moveTo(-MotorY.currentPosition());

  MotorX.run();
  MotorY.run();

  // MotorX.runToNewDistance(30);  // Move 100mm
  // MotorY.runToNewDistance(30);  // Move 100mm
  // delay(500);
  // MotorX.runToNewDistance(0);    // Return to starting position
  // MotorY.runToNewDistance(0);    // Return to starting position
  // delay(1000);
  // lastX *= -1;
  // lastY *= -1;
  // MotorX.moveTo(lastX);
  // MotorY.moveTo(lastY);
  // delay(500);
}

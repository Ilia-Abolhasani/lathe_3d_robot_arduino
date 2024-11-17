#include <AccelStepper.h>
#include <AccelStepperWithDistance.h>

#define MX_EN_PIN 2
#define MX_STEP_PIN 3
#define MX_DIR_PIN 4

#define MY_EN_PIN 5

#define MY_STEP_PIN 6
#define MY_DIR_PIN 7

AccelStepperWithDistance MotorX(AccelStepperWithDistance::DRIVER, MX_STEP_PIN, MX_DIR_PIN);
AccelStepperWithDistance MotorY(AccelStepperWithDistance::DRIVER, MY_STEP_PIN, MY_DIR_PIN);

void setup() {
  MotorX.setMaxSpeed(1000);
  MotorX.setAcceleration(500);
  MotorX.setStepsPerRotation(200);  // For a 1.8° stepper motor
  MotorX.setMicroStep(16);          // If using 1/16 microstepping
  MotorX.setDistancePerRotation(8); // If one rotation moves 8mm

  MotorY.setMaxSpeed(1000);
  MotorY.setAcceleration(500);
  MotorY.setStepsPerRotation(200);  // For a 1.8° stepper motor
  MotorY.setMicroStep(16);          // If using 1/16 microstepping
  MotorY.setDistancePerRotation(8); // If one rotation moves 8mm

  digitalWrite(MX_EN_PIN, HIGH);
  digitalWrite(MY_EN_PIN, HIGH);

  MotorX.runRelative(20);
  MotorY.runRelative(20);

}
int lastX = 20;
int lastY = 20;

void loop() {
  if (MotorX.distanceToGo() != 0) {
    MotorX.run();
  }
  if (MotorY.distanceToGo() != 0) {
    MotorY.run();
  }
  // MotorX.runToNewDistance(30);  // Move 100mm
  // MotorY.runToNewDistance(30);  // Move 100mm
  delay(500);
  // MotorX.runToNewDistance(0);    // Return to starting position
  // MotorY.runToNewDistance(0);    // Return to starting position
  // delay(1000);
  lastX *= -1;
  lastY *= -1;
  MotorX.runRelative(lastX);
  MotorY.runRelative(lastY);
  delay(500);
}

#include <AccelStepper.h>
#include <AccelStepperWithDistance.h>

#define STEP_PIN 3
#define DIR_PIN 4
#define EN_PIN 2

x`

void setup() {
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(200);
  stepper.setStepsPerRotation(200);  // For a 1.8° stepper motor
  stepper.setMicroStep(16);          // If using 1/16 microstepping
  stepper.setDistancePerRotation(8); // If one rotation moves 8mm

  digitalWrite(EN_PIN, HIGH);
}

void loop() {
  stepper.runToNewDistance(100);  // Move 100mm
  delay(1000);
  stepper.runToNewDistance(0);    // Return to starting position
  delay(1000);
}

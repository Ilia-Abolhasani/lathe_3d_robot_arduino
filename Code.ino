#include <AccelStepper.h>

// Motor pins
#define MX_EN_PIN 13
#define MX_STEP_PIN 14
#define MX_DIR_PIN 12

#define MY_EN_PIN 16
#define MY_STEP_PIN 4
#define MY_DIR_PIN 5

// Constants
#define STEPS_PER_MM_X 4 * 843.6
#define STEPS_PER_MM_Y 6.666 * 1106.5
#define UPDATE_INTERVAL 50  // Time interval for updates in milliseconds

AccelStepper MotorX(AccelStepper::DRIVER, MX_STEP_PIN, MX_DIR_PIN);
AccelStepper MotorY(AccelStepper::DRIVER, MY_STEP_PIN, MY_DIR_PIN);

// Arc Parameters
float radius = 735.0;              // Radius in mm
float start_degree = 210.00447;    // Start angle in degrees
float end_degree = 218.0;          // End angle in degrees
float angular_velocity = 0.05;      // Degrees per second

unsigned long lastUpdateTime = 0;  // Last time speeds were updated
float current_angle;               // Current angle in degrees

void setup() {
  delay(10000);
  Serial.begin(115200);

  MotorX.setMaxSpeed(10000);
  MotorY.setMaxSpeed(10000);
  MotorX.setAcceleration(5000);
  MotorY.setAcceleration(5000);

  // Initialize angle
  current_angle = start_degree;

  // Enable motors
  digitalWrite(MX_EN_PIN, HIGH);
  digitalWrite(MY_EN_PIN, HIGH);
}

void loop() {
  unsigned long currentTime = millis();

  // Update motor speeds at regular intervals
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = currentTime;

    // Calculate the current X and Y positions
    float x = radius * cos(current_angle * PI / 180.0);
    float y = radius * sin(current_angle * PI / 180.0);

    // Calculate the next angle based on angular velocity
    current_angle += angular_velocity * (UPDATE_INTERVAL / 1000.0);

    // Limit the angle to the end degree
    if (current_angle >= end_degree) {
      current_angle = end_degree;
      MotorX.setSpeed(0);
      MotorY.setSpeed(0);
      return;
    }

    // Calculate the instantaneous speeds required
    float dx = -radius * sin(current_angle * PI / 180.0);  // Derivative of x
    float dy = radius * cos(current_angle * PI / 180.0);   // Derivative of y

    // Normalize speeds to maintain proportional motion
    float speedX = dx * STEPS_PER_MM_X;
    float speedY = dy * STEPS_PER_MM_Y;

    float max_speed = 5000;  // Max allowable speed in steps per second
    float magnitude = sqrt(speedX * speedX + speedY * speedY);

    if (magnitude > max_speed) {
      float scale_factor = max_speed / magnitude;
      speedX *= scale_factor;
      speedY *= scale_factor;
    }


    // Set motor speeds
    MotorX.setSpeed(speedX);
    MotorY.setSpeed(speedY * -1);

    // Debugging information
    Serial.print("dx: ");
    Serial.print(dx);
    Serial.print(" | dy: ");
    Serial.print(dy);
    Serial.print(" | Angle: ");
    Serial.print(current_angle);
    Serial.print(" | X Speed: ");
    Serial.print(speedX);
    Serial.print(" | Y Speed: ");
    Serial.println(speedY);
  }

  // Run the motors continuously
  MotorX.runSpeed();
  MotorY.runSpeed();
}

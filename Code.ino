#include <AccelStepper.h>

// Motor pins
#define MX_EN_PIN 13
#define MX_STEP_PIN 14
#define MX_DIR_PIN 12

#define MY_EN_PIN 16
#define MY_STEP_PIN 4
#define MY_DIR_PIN 5

// Constants
#define UPDATE_INTERVAL 200  // Time interval for updates in milliseconds

AccelStepper MotorX(AccelStepper::DRIVER, MX_STEP_PIN, MX_DIR_PIN);
AccelStepper MotorY(AccelStepper::DRIVER, MY_STEP_PIN, MY_DIR_PIN);

// Arc Parameters
const long STEPS_PER_MM_X = 4 * 847;
const long STEPS_PER_MM_Y = 6.666 * 1106.5;
float radius = 735.0;              // Radius in mm
float start_degree = 210.00447;    // Start angle in degrees
float end_degree = 216.6093;          // End angle in degrees8
float angular_velocity_per_degree = 400; // Time to move one degree (in seconds)

unsigned long lastUpdateTime = 0;  // Last time speeds were updated
float current_angle;               // Current angle in degrees


long totalStepsX = 0;
long totalStepsY = 0;
long loop_counter = 0;

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
  // pinMode(MX_EN_PIN, OUTPUT);
  // pinMode(MY_EN_PIN, OUTPUT);
  digitalWrite(MX_EN_PIN, HIGH);
  digitalWrite(MY_EN_PIN, HIGH);
}

void loop() {
  unsigned long currentTime = millis();

  // Update motor speeds at regular intervals
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = currentTime;
    loop_counter += 1;

    // Calculate degree step based on angular velocity
    float angular_velocity_per_ms = angular_velocity_per_degree * 1000; // Convert seconds to milliseconds
    float degree_step = (UPDATE_INTERVAL / angular_velocity_per_ms);   // Degrees moved in this time interval
    current_angle += degree_step;
        
    
    // Stop motion when the end angle is reached
    if (current_angle >= end_degree) {
      current_angle = end_degree;
      MotorX.setSpeed(0);
      MotorY.setSpeed(0);
      return;
    }
    
    // Calculate arc length covered in this interval
    float delta_theta = degree_step * PI / 180.0; // Convert degree_step to radians
    float arc_length = radius * delta_theta;     // Arc length in mm

    // Calculate dx and dy based on arc length
    float dx = -arc_length * sin(current_angle * PI / 180.0);
    float dy = arc_length * cos(current_angle * PI / 180.0);
    
    // Convert to motor steps per second
    float speedX = dx * STEPS_PER_MM_X * (1000.0 / UPDATE_INTERVAL );
    float speedY = dy * STEPS_PER_MM_Y * (1000.0 / UPDATE_INTERVAL );

    // Limit the overall speed without affecting the path
    // float max_speed = 10000;  // Max allowable speed in steps per second
    // float magnitude = sqrt(speedX * speedX + speedY * speedY);

    // if (magnitude > max_speed) {
    //   float scale_factor = max_speed / magnitude;
    //   speedX *= scale_factor;
    //   speedY *= scale_factor;
    // }

    long stepsX = speedX * (UPDATE_INTERVAL / 1000.0);
    long stepsY = speedY * (UPDATE_INTERVAL / 1000.0);

    // Accumulate total steps
    totalStepsX += stepsX;
    totalStepsY += stepsY;

    float totalX_mm = totalStepsX / STEPS_PER_MM_X;
    float totalY_mm = totalStepsY / STEPS_PER_MM_Y;

    // Set motor speeds
    MotorX.setSpeed(speedX);
    MotorY.setSpeed(speedY * -1);

    // Debugging information
    
    Serial.print(loop_counter);
    Serial.print(" | ");
    Serial.print("dx: ");
    Serial.print(String(dx, 5));
    Serial.print(" | dy: ");
    Serial.print(String(dy, 5));
    Serial.print(" | Angle: ");
    Serial.print(String(current_angle, 5));
    Serial.print(" | X Speed: ");
    Serial.print(speedX);
    Serial.print(" | Y Speed: ");
    Serial.print(speedY);
    Serial.print(" | totalStepsX: ");
    Serial.print(   totalStepsX);
    Serial.print(" | Total X (mm): ");
    Serial.print(totalX_mm);
    Serial.print(" | Total Y (mm): ");
    Serial.println(totalY_mm);

  }

  // Run the motors continuously
  MotorX.runSpeed();
  MotorY.runSpeed();
}

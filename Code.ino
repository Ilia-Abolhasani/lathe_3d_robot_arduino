#include <AccelStepper.h>

// Define motor pins
#define MX_EN_PIN 11
#define MX_STEP_PIN 12
#define MX_DIR_PIN 13

#define MY_EN_PIN 2
#define MY_STEP_PIN 4
#define MY_DIR_PIN 3


// Direction signs
#define XFORWARD -1
#define XBACK 1

#define YUP -1
#define YDOWN 1

//
#define AJDUST_TIME_INTERVAL 100   // miliseconds
#define Y_SPEED_PER_MILIMETER 0.1  // milimeter

// Define Project Circle:
float radius = 735;
float start_degree = 209.769808;


const float STEPS_PER_MM_X = 4 * 843.6;       // Steps per mm for Motor X
const float STEPS_PER_MM_Y = 6.666 * 1106.5;  // Steps per mm for Motor Y


long MOTORY_SPEED_STEPS = YDOWN * STEPS_PER_MM_Y * Y_SPEED_PER_MILIMETER;
unsigned long startTime, passedTime;  // Variable to store the start time
float startX, startY;
long startXPosition, startYPosition;

AccelStepper MotorX(AccelStepper::DRIVER, MX_STEP_PIN, MX_DIR_PIN);
AccelStepper MotorY(AccelStepper::DRIVER, MY_STEP_PIN, MY_DIR_PIN);


void setup() {
  delay(3000);
  // Start serial communication for monitoring
  Serial.begin(9600);

  // enable drivers
  digitalWrite(MX_EN_PIN, HIGH);
  digitalWrite(MY_EN_PIN, HIGH);

  // Configure motors
  // Max Speed
  MotorX.setMaxSpeed(10000);
  MotorY.setMaxSpeed(10000);

  // Set Acceleration
  // MotorX.setAcceleration(1000);
  // MotorY.setAcceleration(1000);

  // Y axis move per second  
  MotorY.setSpeed(MOTORY_SPEED_STEPS);
  // MotorY.moveTo(YUP * stepsPerMmY * 35);

  // start situation:
  startTime = millis();
  passedTime = startTime;

  startXPosition = MotorX.currentPosition();
  startYPosition = MotorY.currentPosition();

  startX = radius * cos(start_degree * PI / 180.0);
  startY = radius * sin(start_degree * PI / 180.0);
}

void loop() {
  if ((millis() - passedTime) >= AJDUST_TIME_INTERVAL) {
    passedTime = millis();

    // get current position
    long currentXPosition = MotorX.currentPosition();
    long currentYPosition = MotorY.currentPosition();

    long diffXPosition = currentXPosition - startXPosition;
    long diffYPosition = currentYPosition - startYPosition;

    float diffX = diffXPosition / STEPS_PER_MM_X;
    float diffY = diffYPosition / STEPS_PER_MM_Y;

    float currentX = startX + diffX;
    float currentY = startY + diffY;           
    
    // next point 
    long next_Y_position = diffYPosition + MOTORY_SPEED_STEPS; // we can make it more precision
    float next_Y = next_Y_position / STEPS_PER_MM_Y;
    float next_X = sqrt(radius * radius - next_Y * next_Y);    
    long next_x_position = next_X * STEPS_PER_MM_Y;
    long SpeedX = next_x_position - currentXPosition;
    MotorX.setSpeed(SpeedX);
  }
  MotorX.runSpeed();
  MotorY.runSpeed();
  //MotorX.stop();
}

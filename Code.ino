#include <AccelStepper.h>

// Define motor pins
#define MX_EN_PIN 13 // D7
#define MX_STEP_PIN 14 // D5
#define MX_DIR_PIN 12 // D6

#define MY_EN_PIN 16 //D0
#define MY_STEP_PIN 4 //D2
#define MY_DIR_PIN 5 //D1


// Direction signs
#define XFORWARD -1
#define XBACK 1

#define YUP -1
#define YDOWN 1

//
#define AJDUST_TIME_INTERVAL 500   // miliseconds
#define Y_SPEED_PER_MILIMETER 1  // milimeter
#define Y_NEXT_PREDICTION_POINT 10 //  milimeter

// Define Project Circle:
float radius = 735;
float start_degree = 210.00447; // old number: 209.769808;


const float STEPS_PER_MM_X = 4 * 843.6;       // Steps per mm for Motor X
const float STEPS_PER_MM_Y = 6.666 * 1106.5;  // Steps per mm for Motor Y


int MOTORY_SPEED_STEPS = YDOWN * STEPS_PER_MM_Y * Y_SPEED_PER_MILIMETER;
unsigned long startTime, passedTime;  // Variable to store the start time
float startX, startY;
float startXPosition, startYPosition;

AccelStepper MotorX(AccelStepper::DRIVER, MX_STEP_PIN, MX_DIR_PIN);
AccelStepper MotorY(AccelStepper::DRIVER, MY_STEP_PIN, MY_DIR_PIN);


void setup() {
  delay(10000);
  // Start serial communication for monitoring
  Serial.begin(115200);

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

    float currentYPosition = MotorY.currentPosition();
    float currentXPosition = MotorX.currentPosition();

    float diffYPosition = currentYPosition - startYPosition;
    float diffY = diffYPosition / STEPS_PER_MM_Y;
    float currentY = startY - diffY;
    float currentX = sqrt(radius * radius - currentY * currentY) * -1;
    float next_Y = currentY - Y_NEXT_PREDICTION_POINT;
    float next_X = sqrt(radius * radius - next_Y * next_Y) * -1;
    float diff_new_X = next_X - currentX;
    float diff_new_X_position = abs(diff_new_X) * STEPS_PER_MM_X;
    float SpeedX = diff_new_X_position / (Y_NEXT_PREDICTION_POINT / Y_SPEED_PER_MILIMETER); 
    Serial.println(                                                            
              String("#START#") + 
               " diff Y pos: " + String(diffYPosition, 0 ) +               
               ", diff Y mm: " + String(diffY, 5 ) +
               ", cur X: " + String(currentX, 5 ) +
               ", cur Y: " + String(currentY, 5 ) +               
               ", next Y: " + String(next_Y, 5 ) +
               ", next X: " + String(next_X, 5 ) +
               ", diff new X: " + String(diff_new_X, 5 ) +
               ", SpeedX: " + String(SpeedX, 5) +
               ", currentXPosition: " + String(currentXPosition, 1) +               
               "#END#");

    // get current position    
    // float currentXPosition = MotorX.currentPosition();
    // float currentYPosition = MotorY.currentPosition();
            
    // float diffXPosition = currentXPosition - startXPosition;
    // float diffYPosition = currentYPosition - startYPosition;    

    // float diffX = diffXPosition / STEPS_PER_MM_X;
    // float diffY = diffYPosition / STEPS_PER_MM_Y;
            
    // float currentX = startX + diffX;
    // float currentY = startY - diffY;           
            
    // // next point     
    // float next_Y = currentY - Y_SPEED_PER_MILIMETER;    
    // float next_X = sqrt(radius * radius - next_Y * next_Y) * -1;
    // float diff_new_X = next_X - currentX;
    // float SpeedX = abs(diff_new_X) * STEPS_PER_MM_X ;        
    // MotorX.setSpeed(SpeedX);
    // Serial.println(                        
    //            "diff X pos: " + String(diffXPosition, 0 ) + 
    //            ", diff Y pos: " + String(diffYPosition, 0 ) +
    //            ", diff X mm: " + String(diffX, 5 ) +
    //            ", diff Y mm: " + String(diffY, 5 ) +
    //            ", cur X: " + String(currentX, 5 ) +
    //            ", cur Y: " + String(currentY, 5 ) +               
    //            ", next Y: " + String(next_Y, 5 ) +
    //            ", next X: " + String(next_X, 5 ) +
    //            ", diff new X: " + String(diff_new_X, 5 ) +
    //            ", SpeedX: " + String(SpeedX, 5));
    MotorX.setSpeed(SpeedX);
  }
  MotorX.runSpeed();
  MotorY.runSpeed();
  //MotorX.stop();
}


//  startX: -638.00, startY: -364.94,

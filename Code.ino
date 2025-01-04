#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#else
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
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

// WiFi credentials
const char* ssid = "BMT_3D_lathe";
const char* password = "12345678";
IPAddress local_IP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// Web server
AsyncWebServer server(80);

// HTML page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Motor Control</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 20px;
      text-align: center;
    }
    h1 {
      color: #333;
    }
    .section {
      margin: 20px auto;
      padding: 20px;
      border: 1px solid #ddd;
      border-radius: 10px;
      width: 300px;
    }
    .control-buttons button {
      width: 120px;
      height: 40px;
      margin: 10px;
      font-size: 16px;
      border: none;
      border-radius: 5px;
      cursor: pointer;
    }
    .control-buttons .start {
      background-color: #4CAF50; /* Green */
      color: white;
    }
    .control-buttons .stop {
      background-color: #f44336; /* Red */
      color: white;
    }
    .parameter-controls input {
      width: 80%;
      padding: 8px;
      margin: 10px 0;
      font-size: 16px;
      border: 1px solid #ccc;
      border-radius: 5px;
    }
    .parameter-controls button {
      background-color: #008CBA; /* Blue */
      color: white;
      font-size: 16px;
      padding: 10px;
      border: none;
      border-radius: 5px;
      cursor: pointer;
    }
    .manual-controls {
      display: grid;
      grid-template-rows: 50px 50px 50px;
      grid-template-columns: 50px 50px 50px;
      gap: 5px;
      justify-content: center;
      margin-top: 20px;
    }
    .manual-controls button {
      width: 50px;
      height: 50px;
      font-size: 14px;
      background-color: #ccc;
      border: none;
      border-radius: 5px;
      cursor: pointer;
    }
    .manual-controls .up { grid-row: 1; grid-column: 2; }
    .manual-controls .left { grid-row: 2; grid-column: 1; }
    .manual-controls .stop { grid-row: 2; grid-column: 2; background-color: #f44336; color: white; border-radius: 50%; }
    .manual-controls .right { grid-row: 2; grid-column: 3; }
    .manual-controls .down { grid-row: 3; grid-column: 2; }
    .step-controls {      
      text-align: left;
    }    
    .radio-buttons {
      width: 20px;
  height: 20px;
  margin-right: 10px;
  cursor: pointer;
    }
  </style>
</head>
<body>
  <h1>Motor Control</h1>

  <!-- Start/Stop Controls -->
  <div class="section control-buttons">
    <button class="start" onclick="sendStart()">Start</button>
    <button class="stop" onclick="sendStop()">Stop</button>
  </div>

  <!-- Parameter Controls -->
  <div class="section parameter-controls">
    <h2>Set Parameters</h2>
    <label for="speed">Speed (degrees/sec):</label><br>
    <input type="number" id="speed" name="speed" value="400" min="0" oninput="updateTimeEstimation()"><br>
    <button onclick="sendSpeed()">Set Speed</button>
    <div id="total-time">Estimated Total Time: -- minutes</div>
  </div>


  <!-- Step Size Controls -->
  <h2>Step Size</h2>
  <div class="section step-controls">
    <label>
      <input class="radio-buttons" type="radio" name="step-size" value="5"> 5 mm      
    </label><br>
    <label>
      <input class="radio-buttons" type="radio" name="step-size" value="1"> 1 mm
      
    </label><br>
    <label>
      <input class="radio-buttons" type="radio" name="step-size" value="0.1"> 0.1 mm
    </label><br>
    <label>
      <input class="radio-buttons" type="radio" name="step-size" value="0.05"> 0.05 mm
    </label><br>
    <label>
      <input class="radio-buttons" type="radio" name="step-size" value="0.01" checked> 0.01 mm
    </label>
  </div>

  <!-- Manual Controls -->
  <h2>Manual Control</h2>
  <div class="section manual-controls">
    <button class="up" onclick="moveMotor('up')">▲</button>
    <button class="left" onclick="moveMotor('left')">◄</button>
    <button class="stop" onclick="stopMotor()">●</button>
    <button class="right" onclick="moveMotor('right')">►</button>
    <button class="down" onclick="moveMotor('down')">▼</button>
  </div>    

  <script>
    function sendStart() {
      fetch(`/control/start`);
    }

    function sendStop() {
      fetch(`/control/stop`);
    }

    function sendSpeed() {
      const speed = document.getElementById('speed').value;
      fetch(`/control/setSpeed?speed=${speed}`);
    }

    function moveMotor(direction) {
      const stepSize = document.querySelector('input[name="step-size"]:checked').value;
      fetch(`/control/manualMove?direction=${direction}&stepSize=${stepSize}`);
    }

    function stopMotor() {
      fetch(`/control/stopManualMove`);
    }
    
    function updateTimeEstimation() {
      const speed = document.getElementById('speed').value;
      if (speed > 0) {
        const totalTime = (6.6 * speed) / 60; 
        document.getElementById('total-time').innerText = `Estimated Total Time: ${totalTime.toFixed(0)} minutes`;
      } else {
        document.getElementById('total-time').innerText = 'Estimated Total Time: -- minutes';
      }
    }    
    window.onload = function() {
      fetch('/control/getSpeed')
        .then(response => response.text())
        .then(speed => {
          document.getElementById('speed').value = speed;
          updateTimeEstimation();
        });
    }

  </script>
</body>
</html>
)rawliteral";

// Arc Parameters
const long STEPS_PER_MM_X = 4 * 847;
const long STEPS_PER_MM_Y = 6.666 * 1106.5;
float radius = 735.0;                     // Radius in mm
float start_degree = 210.00447;           // Start angle in degrees
float end_degree = 216.6093;              // End angle in degrees8
float angular_velocity_per_degree = 400;  // Time to move one degree (in seconds)

bool isRunning = false;
bool isManualMoveActive = false;
unsigned long lastUpdateTime = 0;  // Last time speeds were updated
float current_angle;               // Current angle in degrees


long totalStepsX = 0;
long totalStepsY = 0;
long loop_counter = 0;

void setupMotors() {
  MotorX.setMaxSpeed(10000);
  MotorY.setMaxSpeed(10000);  
  digitalWrite(MX_EN_PIN, HIGH);
  digitalWrite(MY_EN_PIN, HIGH);
}

// WiFi setup
void setupWiFi() {  
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");  
  Serial.println(WiFi.softAP(ssid, password) ? "Ready" : "Failed!");
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
  Serial.print("[Server Connected] ");
  Serial.println(WiFi.softAPIP());
}

// Web server setup
void setupServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html);
  });

  // Handle start action
  server.on("/control/start", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("Start command received...");
    isRunning = true;
    request->send(200, "text/plain", "Motor started");
  });

  // Handle stop action
  server.on("/control/stop", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("Stop command received...");
    isRunning = false;
    MotorX.setSpeed(0);
    MotorY.setSpeed(0);
    request->send(200, "text/plain", "Motor stopped");
  });

  // Handle set speed action
  server.on("/control/setSpeed", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("speed")) {
      String speedStr = request->getParam("speed")->value();
      angular_velocity_per_degree = speedStr.toFloat();      
      Serial.println("Set speed command received...");
      Serial.print("New speed: ");
      Serial.println(angular_velocity_per_degree);
      request->send(200, "text/plain", "Speed updated");
    } else {
      request->send(400, "text/plain", "Missing speed parameter");
    }
  });

  server.on("/control/manualMove", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("direction") && request->hasParam("stepSize")) {
      String direction = request->getParam("direction")->value();
      String stepSizeStr = request->getParam("stepSize")->value();
      float stepSize = stepSizeStr.toFloat();      
      MotorX.setAcceleration(1000);
      MotorY.setAcceleration(1000);
      isManualMoveActive = true;
      // Get the current positions
      long currentX = MotorX.currentPosition();
      long currentY = MotorY.currentPosition();
      if (direction == "up") {                                    
        MotorY.moveTo(currentY - (STEPS_PER_MM_Y * stepSize));                
      } else if (direction == "down") {                
        MotorY.moveTo(currentY + (STEPS_PER_MM_Y * stepSize) );        
      } else if (direction == "left") {                
        MotorX.moveTo(currentX + (  STEPS_PER_MM_X * stepSize));        
      } else if (direction == "right") {                
        MotorX.moveTo(currentX - ( STEPS_PER_MM_X * stepSize));        
      } else {
        MotorX.setAcceleration(0);  
        MotorY.setAcceleration(0);  
        Serial.println("Invalid direction parameter");
        request->send(400, "text/plain", "Invalid direction parameter");
        return;
      }                   
      request->send(200, "text/plain", "Manual move executed");
    } else {
      request->send(400, "text/plain", "Missing direction or stepSize parameter");
    }
  });

  server.on("/control/stopManualMove", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("Stop motor...");
    isManualMoveActive = false;    
    MotorX.setSpeed(0);
    MotorY.setSpeed(0);
    MotorX.moveTo(MotorX.currentPosition()); // Reset moveTo to current position
    MotorY.moveTo(MotorY.currentPosition()); // Reset moveTo to current position        
    request->send(200, "text/plain", "Motor stopped");
  });

  server.on("/control/getSpeed", HTTP_GET, [](AsyncWebServerRequest *request) {
        String speed = String(angular_velocity_per_degree); // Read from EEPROM or default
        request->send(200, "text/plain", speed);
  });

  server.begin();
}

void setup() {
  Serial.begin(115200);
  Serial.flush();  
  setupWiFi();
  delay(100);    
  setupMotors();  
  setupServer();  
  current_angle = start_degree;
}

void loop() {
  if (isRunning) {
    unsigned long currentTime = millis();

    // Update motor speeds at regular intervals
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
      lastUpdateTime = currentTime;
      loop_counter += 1;

      // Calculate degree step based on angular velocity
      float angular_velocity_per_ms = angular_velocity_per_degree * 1000;  // Convert seconds to milliseconds
      float degree_step = (UPDATE_INTERVAL / angular_velocity_per_ms);     // Degrees moved in this time interval
      current_angle += degree_step;
      
      // Calculate arc length covered in this interval
      float delta_theta = degree_step * PI / 180.0;  // Convert degree_step to radians
      float arc_length = radius * delta_theta;       // Arc length in mm

      // Calculate dx and dy based on arc length
      float dx = -arc_length * sin(current_angle * PI / 180.0);
      float dy = arc_length * cos(current_angle * PI / 180.0);

      // Convert to motor steps per second
      float speedX = dx * STEPS_PER_MM_X * (1000.0 / UPDATE_INTERVAL);
      float speedY = dy * STEPS_PER_MM_Y * (1000.0 / UPDATE_INTERVAL);

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
    }    
    // Run the motors continuously
    MotorX.runSpeed();
    MotorY.runSpeed();
  }  
  if (isManualMoveActive) {
        MotorX.run();
        MotorY.run();

        // Check if the motors have reached their targets
        if (!MotorX.isRunning() && !MotorY.isRunning()) {
            isManualMoveActive = false;  // Stop manual move
            MotorX.setAcceleration(0);  
            MotorY.setAcceleration(0); 
            Serial.println("Manual move completed.");
        }
    }
  yield();
}

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
    .a-button {
      display: inline-block;
      margin: 20px 0;
      padding: 10px 20px;
      background-color: #008CBA;
      color: white;
      text-decoration: none;
      border-radius: 5px;
    }
    .section {
      margin: 20px auto;
      padding: 20px;
      border: 1px solid #ddd;
      border-radius: 10px;
      width: 300px;
    }    
    .parameter-controls input {
      width: 80%;
      padding: 8px;
      margin: 10px 0;
      font-size: 16px;
      border: 1px solid #ccc;
      border-radius: 5px;
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
  <a class="a-button" href="/fix-move">تراش ثابت</a>
  <a class="a-button" href="/curve-move">تراش منحنی</a>        
  <!-- Step Size Controls -->
  <h2>گام حرکت دستی</h2>
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
  <h2>کنترل دستی</h2>
  <div class="section manual-controls">
    <button class="up" onclick="moveMotor('up')">▲</button>
    <button class="left" onclick="moveMotor('left')">◄</button>
    <button class="stop" onclick="stopMotor()">●</button>
    <button class="right" onclick="moveMotor('right')">►</button>
    <button class="down" onclick="moveMotor('down')">▼</button>
  </div>    

  <script>      
    function moveMotor(direction) {
      const stepSize = document.querySelector('input[name="step-size"]:checked').value;
      fetch(`/control/manualMove?direction=${direction}&stepSize=${stepSize}`);
    }

    function stopMotor() {
      fetch(`/control/stopManualMove`);
    }        
  </script>
</body>
</html>
)rawliteral";


const char curve_move_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>تراش منحنی</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 20px;
      text-align: center;
    }
    h1 {
      color: #333;
    }
    .fix-move-button {
      display: inline-block;
      margin: 20px 0;
      padding: 10px 20px;
      background-color: #008CBA;
      color: white;
      text-decoration: none;
      border-radius: 5px;
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
  </style>
</head>
<body>
  <a class="fix-move-button" href="/">بازگشت</a>
  <h1>تراشیدن منحنی</h1>

  <!-- Parameter Controls -->
  <div class="section parameter-controls">
    <h2>تنظیم زمان سپری شدن یک درجه برحسب ثانیه</h2>    
    <input type="number" id="speed" name="speed" value="400" min="0" oninput="updateTimeEstimation()"><br>    
    <div id="total-time">زمان کلی : -- دقیقه</div>
  </div>

  <!-- Start/Stop Controls -->
  <div class="section control-buttons">
    <button class="start" onclick="sendStart()">شروع</button>
    <button class="stop" onclick="sendStop()">توقف</button>
  </div>
    
  <script>
    function sendStart() {
      fetch(`/curve-move/control/start`);
    }

    function sendStop() {
      fetch(`/curve-move/control/stop`);
    }        

    function updateTimeEstimation() {
      const speed = document.getElementById('speed').value;
      if (speed > 0) {
        const speed = document.getElementById('speed').value;
        fetch(`/curve-move/control/setSpeed?speed=${speed}`);
        const totalSeconds = (6.6 * speed); // Total time in seconds
        const minutes = Math.floor(totalSeconds / 60); // Extract minutes
        const seconds = Math.floor(totalSeconds % 60); // Extract remaining seconds
        document.getElementById('total-time').innerText = `زمان کلی : ${minutes} دقیقه و ${seconds} ثانیه`;
      }              
    }    
    
    fetch('/curve-move/control/getSpeed')
        .then(response => response.text())
        .then(speed => {
          document.getElementById('speed').value = speed;
          updateTimeEstimation();
    });    
  </script>
</body>
</html>
)rawliteral";

const char fix_move_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>تراش ثابت</title>
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
    .button {
      margin: 10px;
      padding: 10px 20px;
      font-size: 16px;
      border: none;
      border-radius: 5px;
      cursor: pointer;
    }
    .start {
      background-color: #4CAF50; /* Green */
      color: white;
    }
    .stop {
      background-color: #f44336; /* Red */
      color: white;
    }
    input {
      width: 80%;
      padding: 8px;
      margin: 10px 0;
      font-size: 16px;
      border: 1px solid #ccc;
      border-radius: 5px;
    }
    .back-button {
      display: inline-block;
      margin: 20px 0;
      padding: 10px 20px;
      background-color: #008CBA;
      color: white;
      text-decoration: none;
      border-radius: 5px;
    }
  </style>
</head>
<body>
  <a class="back-button" href="/">بازگشت</a>    
  <!-- Parameters Section -->
  <div class="section">
    <h1>تنظیم سرعت موتورها</h1>
    <h2>مقدارها بر حسب میلیمتر بر دقیقه است</h2>
    <label for="motorXSpeed">Motor X: (حرکت جلو و عقب)</label><br>
    <input type="number" id="motorXSpeed" name="motorXSpeed" value="10" min="0" oninput="updateMotorXSpeed()"><br><br>

    <label for="motorYSpeed">Motor Y: (حرکت بالا و پایین)</label><br>
    <input type="number" id="motorYSpeed" name="motorYSpeed" value="10" min="0" oninput="updateMotorYSpeed()"><br>
  </div>
  
  <!-- Control Buttons -->
  <div class="section">
    <button class="button start" onclick="startAlgorithm()">شروع</button>
    <button class="button stop" onclick="stopAlgorithm()">توقف</button>
  </div>
  
  <script>
    function updateMotorXSpeed() {
      const speed = document.getElementById('motorXSpeed').value;
      if (speed > 0) {        
        fetch(`/fix-move/control/setSpeed/X?speed=${speed}`);        
      }              
    }    

    function updateMotorYSpeed() {
      const speed = document.getElementById('motorYSpeed').value;
      if (speed > 0) {        
        fetch(`/fix-move/control/setSpeed/Y?speed=${speed}`);        
      }              
    }    

    fetch('/fix-move/control/getSpeed/X')
        .then(response => response.text())
        .then(speed => {
          document.getElementById('motorXSpeed').value = speed;          
    });    

    fetch('/fix-move/control/getSpeed/Y')
        .then(response => response.text())
        .then(speed => {
          document.getElementById('motorYSpeed').value = speed;          
    });    
    

    function startAlgorithm() {
      const motorXSpeed = document.getElementById('motorXSpeed').value;
      const motorYSpeed = document.getElementById('motorYSpeed').value;

      if (motorXSpeed && motorYSpeed) {
        fetch(`/fix-move/control/start?motorXSpeed=${motorXSpeed}&motorYSpeed=${motorYSpeed}`)
          .then(response => response.text())
          .then(data => console.log(data))
          .catch(error => console.error('Error:', error));
      } else {
        alert('Please enter valid speeds for both motors.');
      }
    }

    function stopAlgorithm() {
      fetch('/fix-move/control/stop')
        .then(response => response.text())
        .then(data => console.log(data))
        .catch(error => console.error('Error:', error));
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


// fix speed parameter
float fix_motor_x_speed = 10;
float fix_motor_y_speed = 10;


bool isCurveRunning = false;
bool isFixRunning = false;
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
  // for main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html);
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
        MotorY.moveTo(currentY + (STEPS_PER_MM_Y * stepSize));
      } else if (direction == "left") {
        MotorX.moveTo(currentX + (STEPS_PER_MM_X * stepSize));
      } else if (direction == "right") {
        MotorX.moveTo(currentX - (STEPS_PER_MM_X * stepSize));
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
    MotorX.moveTo(MotorX.currentPosition());  // Reset moveTo to current position
    MotorY.moveTo(MotorY.currentPosition());  // Reset moveTo to current position
    request->send(200, "text/plain", "Motor stopped");
  });


  server.on("/curve-move/control/start", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("Start command received...");
    isCurveRunning = true;
    request->send(200, "text/plain", "Motor started");
  });

  server.on("/curve-move/control/stop", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("Stop command received...");
    isCurveRunning = false;
    MotorX.setSpeed(0);
    MotorY.setSpeed(0);
    request->send(200, "text/plain", "Motor stopped");
  });

  // Handle set speed action
  server.on("/curve-move/control/setSpeed", HTTP_GET, [](AsyncWebServerRequest* request) {
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

  server.on("/curve-move/control/getSpeed", HTTP_GET, [](AsyncWebServerRequest* request) {
    String speed = String(angular_velocity_per_degree);  // Read from EEPROM or default
    request->send(200, "text/plain", speed);
  });

  server.on("/curve-move", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", curve_move_html);
  });

  server.on("/fix-move/control/stop", HTTP_GET, [](AsyncWebServerRequest* request) {
    Serial.println("fix-move Stop command received...");
    isFixRunning = false; 
    MotorX.setSpeed(0);
    MotorY.setSpeed(0);
    request->send(200, "text/plain", "Motor stopped");
  });

  server.on("/fix-move/control/start", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("motorXSpeed") && request->hasParam("motorYSpeed")) {
      // Correct parameter names
      String motorXSpeedStr = request->getParam("motorXSpeed")->value();
      float motorXSpeed = motorXSpeedStr.toFloat();
      String motorYSpeedStr = request->getParam("motorYSpeed")->value();
      float motorYSpeed = motorYSpeedStr.toFloat();

      MotorX.setSpeed((motorXSpeed / 60) * STEPS_PER_MM_X);
      MotorY.setSpeed((motorYSpeed / 60) * STEPS_PER_MM_Y);
      isFixRunning = true;      
      request->send(200, "text/plain", "fix move executed");
    } else {
      request->send(400, "text/plain", "Missing motorXSpeed or motorYSpeed parameter");
    }
  });

  server.on("/fix-move/control/setSpeed/X", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("speed")) {
      String speedStr = request->getParam("speed")->value();
      fix_motor_x_speed = speedStr.toFloat();            
      request->send(200, "text/plain", "Speed updated");
    } else {
      request->send(400, "text/plain", "Missing speed parameter");
    }
  });  
  
  server.on("/fix-move/control/setSpeed/Y", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("speed")) {
      String speedStr = request->getParam("speed")->value();
      fix_motor_y_speed = speedStr.toFloat();            
      request->send(200, "text/plain", "Speed updated");
    } else {
      request->send(400, "text/plain", "Missing speed parameter");
    }
  });

  server.on("/fix-move/control/getSpeed/X", HTTP_GET, [](AsyncWebServerRequest* request) {
    String speed = String(fix_motor_x_speed);  
    request->send(200, "text/plain", speed);
  });

  server.on("/fix-move/control/getSpeed/Y", HTTP_GET, [](AsyncWebServerRequest* request) {
    String speed = String(fix_motor_y_speed);  
    request->send(200, "text/plain", speed);
  });

  server.on("/fix-move", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", fix_move_html);
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
  if (isCurveRunning) {
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
  if (isFixRunning) {
      MotorX.runSpeed();
      MotorY.runSpeed();
  }   
  yield();
}

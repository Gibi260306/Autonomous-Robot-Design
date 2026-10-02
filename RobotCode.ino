#include <WiFiS3.h>
#include <Servo.h>

// ==================================================
// Controller packet variables
// Expected packet:
// <lx,ly,rx,ry,l2,r2,l1,r1,aBtn>
// ==================================================
int lx, ly, rx, ry;
int l2, r2;
int l1, r1;
int aBtn;

// ==================================================
// Packet reading buffer
// ==================================================
char buffer[100];
int bufferIndex = 0;
bool inPacket = false;

// ==================================================
// Cytron Maker Drive pins
// Left motor: M1A, M1B
// Right motor: M2A, M2B
// ==================================================
const int M1A = 3;
const int M1B = 5;
const int M2A = 6;
const int M2B = 9;

// ==================================================
// Line sensor pins
// ==================================================
const int S_LEFT = A0;
const int S_MID = A1;
const int S_RIGHT = A2;

// ==================================================
// Servo
// ==================================================
const int SERVO_PIN = 11;
Servo myServo;
int servoAngle = 90;

const unsigned long SERVO_MOVE_INTERVAL_MS = 20;
unsigned long lastServoMoveTime = 0;
const int SERVO_STEP = 2;

// ==================================================
// Manual control settings
// ==================================================
const int JOYSTICK_DEADZONE = 40;
const int TRIGGER_DEADZONE = 30;

// ==================================================
// Line-follow speeds
// ==================================================
const int BASE_SPEED = 75;
const int TURN_SPEED = 75;
const int SOFT_TURN_SPEED = 45;
const int SEARCH_SPEED = 65;

// ==================================================
// Safety timeout for controller packets
// ==================================================
const unsigned long PACKET_TIMEOUT_MS = 10000000000000;
unsigned long lastPacketTime = 0;

// ==================================================
// Mode state
// false = manual control
// true = line following
// ==================================================
bool lineFollowingMode = false;
bool lastAState = false;
// ==================================================
// Line lost timeout
// Keep moving forward for this long after all sensors lose the line
// ==================================================
const unsigned long LINE_LOST_FORWARD_MS = 1000;
unsigned long lastLineSeenTime = 0;
// Non-blocking A-button debounce
const unsigned long TOGGLE_DEBOUNCE_MS = 200;
unsigned long lastToggleTime = 0;

// ==================================================
// Last known line direction
// -1 = left, 0 = center, 1 = right
// ==================================================
int lastDirection = 0;

// ==================================================
// Encoder telemetry
// ==================================================
#define LEFT_ENC_A 2
#define LEFT_ENC_B 13
#define RIGHT_ENC_A 8
#define RIGHT_ENC_B 7

unsigned long leftCount = 0;
unsigned long rightCount = 0;

int lastLeftAState = LOW;
int lastLeftBState = LOW;
int lastRightAState = LOW;
int lastRightBState = LOW;

// ==================================================
// Wi-Fi telemetry server
// ==================================================
const char* ssid = "CAR_TELEMETRY";
const char* pass = "12345678";
WiFiServer server(80);

// ==================================================
// Wheel / encoder calibration
// IMPORTANT:
// Set this to the ACTUAL measured counts for one full
// wheel revolution using this exact counting method:
// rising edge on A + rising edge on B
// ==================================================
const float WHEEL_DIAMETER_M = 0.035f;
const float EFFECTIVE_COUNTS_PER_REV = 170.0f;
const float PI_VAL = 3.1415926f;
const float WHEEL_CIRCUMFERENCE_M = PI_VAL * WHEEL_DIAMETER_M;

// ==================================================
// Telemetry timing / smoothing
// ==================================================
const unsigned long SPEED_SAMPLE_MS = 100;
const unsigned long PRINT_INTERVAL_MS = 1000;
const unsigned long CLIENT_WAIT_MS = 30;
const float SPEED_SMOOTH_ALPHA = 0.35f;

unsigned long lastSpeedTime = 0;
unsigned long prevLeftCount = 0;
unsigned long prevRightCount = 0;

float leftSpeedCountsRaw = 0.0f;
float rightSpeedCountsRaw = 0.0f;
float avgSpeedCountsRaw = 0.0f;

float leftSpeedCounts = 0.0f;
float rightSpeedCounts = 0.0f;
float avgSpeedCounts = 0.0f;

float leftSpeedMPS = 0.0f;
float rightSpeedMPS = 0.0f;
float avgSpeedMPS = 0.0f;
float distanceM = 0.0f;

// ==================================================
// Function declarations
// ==================================================
void readControllerPacket();
void updateServoFromBumpers();
void manualControl();
void lineFollowControl();

void setLeftMotorForward(int speedVal);
void setRightMotorForward(int speedVal);
void setLeftMotor(int pwmVal);
void setRightMotor(int pwmVal);
void stopLeftMotor();
void stopRightMotor();
void moveForward(int speedVal);
void turnLeftSharp();
void turnRightSharp();
void turnLeftSoft();
void turnRightSoft();
void searchLeft();
void searchRight();
void stopMotors();

int applyDeadzone(int value, int deadzone);
int calcDrivePWM();
int calcTurnPWM();

void updateEncoders();
void updateTelemetryMetrics();
void printTelemetrySerial();

void handleTelemetryClient();
void sendPage(WiFiClient &client);
void sendJson(WiFiClient &client);

// ==================================================
// Setup
// ==================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  delay(300);
  // Motors
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);

  // Line sensors
  pinMode(S_LEFT, INPUT);
  pinMode(S_MID, INPUT);
  pinMode(S_RIGHT, INPUT);

  // Encoders
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  lastLeftAState = digitalRead(LEFT_ENC_A);
  lastLeftBState = digitalRead(LEFT_ENC_B);
  lastRightAState = digitalRead(RIGHT_ENC_A);
  lastRightBState = digitalRead(RIGHT_ENC_B);

  // Servo
  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle);

  lastSpeedTime = millis();
  lastLineSeenTime = millis();
  stopMotors();

// Wi-Fi AP + server
WiFi.beginAP(ssid, pass);
server.begin();

lastSpeedTime = millis();

stopMotors();

Serial.println("UNO R4 controller + line follow + telemetry ready");
Serial.print("SSID: ");
Serial.println(ssid);
Serial.print("IP Address: ");
Serial.println(WiFi.localIP());
}

// ==================================================
// Main loop
// ==================================================
void loop() {
  readControllerPacket();
  updateServoFromBumpers();
  updateEncoders();
  updateTelemetryMetrics();
  //printTelemetrySerial();
  static unsigned long lastClientCheck = 0;
  if (millis() - lastClientCheck >= 50) {
  handleTelemetryClient();
  lastClientCheck = millis();
  }
  unsigned long now = millis();

  // Toggle mode when A is newly pressed
  bool currentAState = (aBtn == 1);
  if (currentAState && !lastAState && (now - lastToggleTime >= TOGGLE_DEBOUNCE_MS)) {
    lineFollowingMode = !lineFollowingMode;
    stopMotors();
    lastToggleTime = now;
  }
  lastAState = currentAState;

  if (lineFollowingMode) {
    lineFollowControl();
  } else {
    manualControl();
  }
}

// ==================================================
// Reads incoming packet from ESP32
// Expected format:
// <lx,ly,rx,ry,l2,r2,l1,r1,aBtn>
// ==================================================
void readControllerPacket() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '<') {
      bufferIndex = 0;
      inPacket = true;
      buffer[bufferIndex++] = c;
    } else if (inPacket) {
      if (bufferIndex < (int)sizeof(buffer) - 1) {
        buffer[bufferIndex++] = c;
      }

      if (c == '>') {
        buffer[bufferIndex] = '\0';
        inPacket = false;

        int count = sscanf(buffer, "<%d,%d,%d,%d,%d,%d,%d,%d,%d>",
                           &lx, &ly, &rx, &ry, &l2, &r2, &l1, &r1, &aBtn);

        if (count == 9) {
          lastPacketTime = millis();
        }
      }
    }
  }
}

// ==================================================
// Servo control from bumpers
// RB = move right
// LB = move left
// ==================================================
void updateServoFromBumpers() {
  unsigned long now = millis();

  if (now - lastServoMoveTime < SERVO_MOVE_INTERVAL_MS) {
    return;
  }

  if (r1 == 1 && l1 == 0) {
    servoAngle -= SERVO_STEP;
  } else if (l1 == 1 && r1 == 0) {
    servoAngle += SERVO_STEP;
  }

  servoAngle = constrain(servoAngle, 0, 180);
  myServo.write(servoAngle);
  lastServoMoveTime = now;
}

// ==================================================
// Deadzone helper
// ==================================================
int applyDeadzone(int value, int deadzone) {
  if (value > -deadzone && value < deadzone) {
    return 0;
  }
  return value;
}

// ==================================================
// Manual control calculations
// R2 = forward
// L2 = backward
// LX = steering
// ==================================================
int calcDrivePWM() {
  int forward = (r2 < TRIGGER_DEADZONE) ? 0 : r2;
  int backward = (l2 < TRIGGER_DEADZONE) ? 0 : l2;

  int forwardPWM = map(forward, 0, 1023, 0, 125);
  int backwardPWM = map(backward, 0, 1023, 0, 125);

  return forwardPWM - backwardPWM;
}

int calcTurnPWM() {
  int turn = applyDeadzone(lx, JOYSTICK_DEADZONE);
  return map(turn, -511, 512, -125, 125);
}

// ==================================================
// Manual controller mode
// ==================================================
void manualControl() {
  // Lost controller packets = stop
  if (millis() - lastPacketTime > PACKET_TIMEOUT_MS) {
    stopMotors();
    return;
  }

  int drivePWM = calcDrivePWM();
  int turnPWM = calcTurnPWM();

  if (drivePWM == 0) {
    stopMotors();
    return;
  }

  int leftPWM = constrain(drivePWM + turnPWM, -255, 255);
  int rightPWM = constrain(drivePWM - turnPWM, -255, 255);

  setLeftMotor(leftPWM);
  setRightMotor(rightPWM);
}

// ==================================================
// Line-following mode
// Uses lastDirection search instead of stopping forever
// Change HIGH to LOW if your sensors are active-low
// ==================================================
void lineFollowControl() {
  int leftVal = digitalRead(S_LEFT);
  int midVal = digitalRead(S_MID);
  int rightVal = digitalRead(S_RIGHT);

  bool leftOnLine = (leftVal == HIGH);
  bool midOnLine = (midVal == HIGH);
  bool rightOnLine = (rightVal == HIGH);

  unsigned long now = millis();

  // If any sensor sees the line, refresh the "last seen" timer
  if (leftOnLine || midOnLine || rightOnLine) {
    lastLineSeenTime = now;
  }

  if (midOnLine && !leftOnLine && !rightOnLine) {
    moveForward(BASE_SPEED);
    lastDirection = 0;
  } 
  else if (leftOnLine && !midOnLine && !rightOnLine) {
    turnLeftSharp();
    lastDirection = -1;
  } 
  else if (rightOnLine && !midOnLine && !leftOnLine) {
    turnRightSharp();
    lastDirection = 1;
  } 
  else if (leftOnLine && midOnLine && !rightOnLine) {
    turnLeftSoft();
    lastDirection = -1;
  } 
  else if (rightOnLine && midOnLine && !leftOnLine) {
    turnRightSoft();
    lastDirection = 1;
  } 
  else if (leftOnLine && midOnLine && rightOnLine) {
    moveForward(BASE_SPEED);
    lastDirection = 0;
  } 
  else {
    // No sensors detect the line
    if (now - lastLineSeenTime < LINE_LOST_FORWARD_MS) {
      moveForward(SEARCH_SPEED);
    } else {
      stopMotors();
    }
  }
}

// ==================================================
// Encoder counting
// Counts rising edges on both A and B channels
// ==================================================
void updateEncoders() {
  int currentLeftA = digitalRead(LEFT_ENC_A);
  int currentLeftB = digitalRead(LEFT_ENC_B);
  int currentRightA = digitalRead(RIGHT_ENC_A);
int currentRightB = digitalRead(RIGHT_ENC_B);

  if (currentLeftA == HIGH && lastLeftAState == LOW) {
    leftCount++;
  }
  if (currentLeftB == HIGH && lastLeftBState == LOW) {
    leftCount++;
  }
  if (currentRightA == HIGH && lastRightAState == LOW) {
    rightCount++;
  }
  if (currentRightB == HIGH && lastRightBState == LOW) {
    rightCount++;
  }

  lastLeftAState = currentLeftA;
  lastLeftBState = currentLeftB;
  lastRightAState = currentRightA;
  lastRightBState = currentRightB;
}

// ==================================================
// Telemetry calculations
// ==================================================
void updateTelemetryMetrics() {
  unsigned long now = millis();
  unsigned long dt = now - lastSpeedTime;

  if (dt >= SPEED_SAMPLE_MS) {
    unsigned long deltaLeft = leftCount - prevLeftCount;
    unsigned long deltaRight = rightCount - prevRightCount;

    leftSpeedCountsRaw = (deltaLeft * 1000.0f) / dt;
    rightSpeedCountsRaw = (deltaRight * 1000.0f) / dt;
    avgSpeedCountsRaw = (leftSpeedCountsRaw + rightSpeedCountsRaw) / 2.0f;

    leftSpeedCounts = SPEED_SMOOTH_ALPHA * leftSpeedCountsRaw + (1.0f - SPEED_SMOOTH_ALPHA) * leftSpeedCounts;
    rightSpeedCounts = SPEED_SMOOTH_ALPHA * rightSpeedCountsRaw + (1.0f - SPEED_SMOOTH_ALPHA) * rightSpeedCounts;
    avgSpeedCounts = SPEED_SMOOTH_ALPHA * avgSpeedCountsRaw + (1.0f - SPEED_SMOOTH_ALPHA) * avgSpeedCounts;

    leftSpeedMPS = (leftSpeedCounts / EFFECTIVE_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M;
    rightSpeedMPS = (rightSpeedCounts / EFFECTIVE_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M;
    avgSpeedMPS = (avgSpeedCounts / EFFECTIVE_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M;

    float avgCount = (leftCount + rightCount) / 2.0f;
    distanceM = (avgCount / EFFECTIVE_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M;

    prevLeftCount = leftCount;
    prevRightCount = rightCount;
    lastSpeedTime = now;
  }
}

// ==================================================
// Throttled serial telemetry
// ==================================================
void printTelemetrySerial() {
  unsigned long now = millis();
  static unsigned long lastPrintTime = 0;

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    long error = (long)leftCount - (long)rightCount;
    float runtime = millis() / 1000.0f;

    Serial.print("Mode: ");
    Serial.print(lineFollowingMode ? "LINE" : "MANUAL");
    Serial.print(" L: ");
    Serial.print(leftCount);
    Serial.print(" R: ");
    Serial.print(rightCount);
    Serial.print(" Error: ");
    Serial.print(error);
    Serial.print(" L_speed(c/s): ");
    Serial.print(leftSpeedCounts, 2);
    Serial.print(" R_speed(c/s): ");
    Serial.print(rightSpeedCounts, 2);
    Serial.print(" Avg_speed(m/s): ");
    Serial.print(avgSpeedMPS, 3);
    Serial.print(" Distance(m): ");
    Serial.print(distanceM, 3);
    Serial.print(" Servo: ");
    Serial.print(servoAngle);
    Serial.print(" Runtime: ");
    Serial.println(runtime, 1);

    lastPrintTime = now;
  }
}

// ==================================================
// Minimal-impact telemetry client handling
// ==================================================
void handleTelemetryClient() {
  WiFiClient client = server.available();
  if (!client) return;

  // Use a very short timeout
  client.setTimeout(2); 

  // Read only the first line of the request
  if (client.available()) {
    String request = client.readStringUntil('\r');
    client.flush(); // Clear the rest of the headers quickly

    if (request.indexOf("GET /data") >= 0) {
      sendJson(client);
    } else if (request.indexOf("GET / ") >= 0) {
      sendPage(client);
    }
  }
  client.stop(); // Close connection immediately
}

// ==================================================
// JSON endpoint
// ==================================================
void sendJson(WiFiClient &client) {
  long error = (long)leftCount - (long)rightCount;
  float runtime = millis() / 1000.0f;

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();

  client.print("{");
  client.print("\"mode\":\"");
  client.print(lineFollowingMode ? "LINE" : "MANUAL");
  client.print("\",");

  client.print("\"leftCount\":");
  client.print(leftCount);
  client.print(",");

  client.print("\"rightCount\":");
  client.print(rightCount);
  client.print(",");

  client.print("\"error\":");
  client.print(error);
  client.print(",");

  client.print("\"leftSpeedCounts\":");
  client.print(leftSpeedCounts, 2);
  client.print(",");

  client.print("\"rightSpeedCounts\":");
  client.print(rightSpeedCounts, 2);
  client.print(",");

  client.print("\"avgSpeedCounts\":");
  client.print(avgSpeedCounts, 2);
  client.print(",");

  client.print("\"leftSpeedMPS\":");
  client.print(leftSpeedMPS, 3);
  client.print(",");

  client.print("\"rightSpeedMPS\":");
  client.print(rightSpeedMPS, 3);
  client.print(",");

  client.print("\"avgSpeedMPS\":");
  client.print(avgSpeedMPS, 3);
  client.print(",");

  client.print("\"distanceM\":");
  client.print(distanceM, 3);
  client.print(",");

  client.print("\"servoAngle\":");
  client.print(servoAngle);
  client.print(",");

  client.print("\"runtime\":");
  client.print(runtime, 1);

  client.print("}");
}

// ==================================================
// Dashboard page
// ==================================================
void sendPage(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html; charset=utf-8");
  client.println("Connection: close");
  client.println();

  client.println("<!DOCTYPE html>");
  client.println("<html>");
  client.println("<head>");
  client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  client.println("<title>Robot Telemetry</title>");
  client.println("<style>");
  client.println("body{font-family:Arial,sans-serif;background:#eef3f8;margin:0;padding:20px;color:#1f2937;}");
  client.println(".container{max-width:980px;margin:auto;}");
  client.println(".title{background:linear-gradient(135deg,#1d4ed8,#06b6d4);color:white;padding:18px 24px;border-radius:16px;box-shadow:0 4px 12px rgba(0,0,0,0.15);margin-bottom:20px;}");
  client.println(".title h1{margin:0;font-size:30px;}");
  client.println(".subtitle{margin-top:6px;font-size:14px;opacity:0.9;}");
  client.println(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:14px;}");
  client.println(".card{background:white;padding:16px;border-radius:14px;box-shadow:0 2px 8px rgba(0,0,0,0.10);}");
  client.println(".label{font-size:14px;color:#6b7280;margin-bottom:6px;}");
  client.println(".value{font-size:28px;font-weight:bold;}");
  client.println(".left{color:#0f766e;}");
  client.println(".right{color:#b91c1c;}");
  client.println(".error{color:#b45309;}");
  client.println(".speed{color:#1d4ed8;}");
  client.println(".distance{color:#7c3aed;}");
  client.println(".runtime{color:#374151;}");
  client.println(".mode{color:#0f172a;}");
  client.println(".servo{color:#0ea5e9;}");
  client.println(".graphBox{background:white;padding:16px;border-radius:14px;box-shadow:0 2px 8px rgba(0,0,0,0.10);margin-top:20px;}");
  client.println("canvas{width:100%;height:260px;background:#ffffff;border:1px solid #dbeafe;border-radius:12px;}");
  client.println("</style>");
  client.println("</head>");
  client.println("<body>");
  client.println("<div class='container'>");
  client.println("<div class='title'>");
  client.println("<h1>Robot Telemetry Dashboard</h1>");
  client.println("<div class='subtitle'>Live controller / line-follow telemetry with counts, speed, distance, servo and runtime</div>");
  client.println("</div>");

  client.println("<div class='grid'>");
  client.println("<div class='card'><div class='label'>Mode</div><div class='value mode' id='mode'>MANUAL</div></div>");
  client.println("<div class='card'><div class='label'>Left count</div><div class='value left' id='leftCount'>0</div></div>");
  client.println("<div class='card'><div class='label'>Right count</div><div class='value right' id='rightCount'>0</div></div>");
  client.println("<div class='card'><div class='label'>Error</div><div class='value error' id='error'>0</div></div>");
  client.println("<div class='card'><div class='label'>Left speed (counts/s)</div><div class='value speed' id='leftSpeedCounts'>0</div></div>");
  client.println("<div class='card'><div class='label'>Right speed (counts/s)</div><div class='value speed' id='rightSpeedCounts'>0</div></div>");
  client.println("<div class='card'><div class='label'>Average speed (m/s)</div><div class='value speed' id='avgSpeedMPS'>0</div></div>");
  client.println("<div class='card'><div class='label'>Distance travelled (m)</div><div class='value distance' id='distanceM'>0</div></div>");
  client.println("<div class='card'><div class='label'>Servo angle</div><div class='value servo' id='servoAngle'>0</div></div>");
  client.println("<div class='card'><div class='label'>Runtime (s)</div><div class='value runtime' id='runtime'>0</div></div>");
  client.println("</div>");

  client.println("<div class='graphBox'>");
  client.println("<h2 style='margin-top:0;color:#1d4ed8;'>Average Speed Graph</h2>");
  client.println("<canvas id='speedChart' width='800' height='260'></canvas>");
  client.println("</div>");

  client.println("</div>");
  client.println("<script>");
  client.println("const avgHistory = [];");
  client.println("const maxPoints = 30;");
  client.println("const canvas = document.getElementById('speedChart');");
  client.println("const ctx = canvas.getContext('2d');");

  client.println("function drawGraph(){");
  client.println(" ctx.clearRect(0,0,canvas.width,canvas.height);");
  client.println(" ctx.fillStyle = '#ffffff';");
  client.println(" ctx.fillRect(0,0,canvas.width,canvas.height);");
  client.println(" ctx.strokeStyle = '#d1d5db';");
  client.println(" ctx.lineWidth = 1;");
  client.println(" for(let i=0;i<=5;i++){");
  client.println("  const y = 20 + i*((canvas.height-40)/5);");
  client.println("  ctx.beginPath();");
  client.println("  ctx.moveTo(40,y);");
  client.println("  ctx.lineTo(canvas.width-10,y);");
  client.println("  ctx.stroke();");
  client.println(" }");
  client.println(" if(avgHistory.length < 2) return;");
  client.println(" let maxVal = Math.max(...avgHistory, 0.05);");
  client.println(" const chartWidth = canvas.width - 50;");
  client.println(" const chartHeight = canvas.height - 40;");
  client.println(" ctx.strokeStyle = '#2563eb';");
  client.println(" ctx.lineWidth = 3;");
  client.println(" ctx.beginPath();");
  client.println(" for(let i=0;i<avgHistory.length;i++){");
  client.println("  const x = 40 + (i * chartWidth / (maxPoints - 1));");
  client.println("  const y = canvas.height - 20 - (avgHistory[i] / maxVal) * chartHeight;");
  client.println("  if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);");
  client.println(" }");
  client.println(" ctx.stroke();");
  client.println(" ctx.fillStyle = '#1f2937';");
  client.println(" ctx.font = '12px Arial';");
  client.println(" ctx.fillText('0', 10, canvas.height - 18);");
  client.println(" ctx.fillText(maxVal.toFixed(2) + ' m/s', 5, 18);");
  client.println("}");

  client.println("function updateData(){");
  client.println(" fetch('/data')");
  client.println(" .then(response => response.json())");
  client.println(" .then(data => {");
  client.println("  document.getElementById('mode').innerText = data.mode;");
  client.println("  document.getElementById('leftCount').innerText = data.leftCount;");
  client.println("  document.getElementById('rightCount').innerText = data.rightCount;");
  client.println("  document.getElementById('error').innerText = data.error;");
  client.println("  document.getElementById('leftSpeedCounts').innerText = data.leftSpeedCounts.toFixed(2);");
  client.println("  document.getElementById('rightSpeedCounts').innerText = data.rightSpeedCounts.toFixed(2);");
  client.println("  document.getElementById('avgSpeedMPS').innerText = data.avgSpeedMPS.toFixed(3);");
  client.println("  document.getElementById('distanceM').innerText = data.distanceM.toFixed(3);");
  client.println("  document.getElementById('servoAngle').innerText = data.servoAngle;");
  client.println("  document.getElementById('runtime').innerText = data.runtime.toFixed(1);");
  client.println("  avgHistory.push(data.avgSpeedMPS);");
  client.println("  if(avgHistory.length > maxPoints) avgHistory.shift();");
  client.println("  drawGraph();");
  client.println(" })");
  client.println(" .catch(() => {});");
  client.println("}");

  client.println("setInterval(updateData, 100);");
  client.println("updateData();");
  client.println("</script>");
  client.println("</body>");
  client.println("</html>");
}

// ==================================================
// Motor helper functions
// ==================================================
void setLeftMotorForward(int speedVal) {
  analogWrite(M1A, 0);
  analogWrite(M1B, constrain(speedVal, 0, 255));
}

void setRightMotorForward(int speedVal) {
  analogWrite(M2A, constrain(speedVal, 0, 255));
  analogWrite(M2B, 0);
}

void setLeftMotor(int pwmVal) {
  pwmVal = constrain(pwmVal, -255, 255);

  if (pwmVal > 0) {
    analogWrite(M1A, 0);
    analogWrite(M1B, pwmVal);
  } else if (pwmVal < 0) {
    analogWrite(M1A, -pwmVal);
    analogWrite(M1B, 0);
  } else {
    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
  }
}

void setRightMotor(int pwmVal) {
  pwmVal = constrain(pwmVal, -255, 255);

  if (pwmVal > 0) {
    analogWrite(M2A, pwmVal);
    analogWrite(M2B, 0);
  } else if (pwmVal < 0) {
    analogWrite(M2A, 0);
    analogWrite(M2B, -pwmVal);
  } else {
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);
  }
}

void stopLeftMotor() {
  analogWrite(M1A, 0);
  analogWrite(M1B, 0);
}

void stopRightMotor() {
  analogWrite(M2A, 0);
  analogWrite(M2B, 0);
}

void moveForward(int speedVal) {
  setLeftMotorForward(speedVal);
  setRightMotorForward(speedVal);
}

void turnLeftSharp() {
  stopLeftMotor();
  setRightMotorForward(TURN_SPEED);
}

void turnRightSharp() {
  setLeftMotorForward(TURN_SPEED);
  stopRightMotor();
}

void turnLeftSoft() {
  setLeftMotorForward(SOFT_TURN_SPEED);
  setRightMotorForward(TURN_SPEED);
}

void turnRightSoft() {
  setLeftMotorForward(TURN_SPEED);
  setRightMotorForward(SOFT_TURN_SPEED);
}

void searchLeft() {
  stopLeftMotor();
  setRightMotorForward(SEARCH_SPEED);
}

void searchRight() {
  setLeftMotorForward(SEARCH_SPEED);
  stopRightMotor();
}

void stopMotors() {
  stopLeftMotor();
  stopRightMotor();
}
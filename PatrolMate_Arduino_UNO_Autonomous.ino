/*
  PatrolMate AI - Autonomous Obstacle Avoidance
  Arduino UNO R3 + L298N + 2x HC-SR04 + 2x Servo

  EXISTING WIRING (unchanged):
    ENA D5, IN1 D8, IN2 D9, IN3 D10, IN4 D11, ENB D6
    Front HC-SR04: TRIG A2, ECHO A3
    Rear  HC-SR04: TRIG A4, ECHO A5
    Front servo signal A1, Rear servo signal A0

  Safety threshold: 20 cm
  Serial USB commands: START_PATROL, STOP_PATROL, F, B, L, R, S, STATUS
  ESP32 UART: RX on D12 at 9600 baud (ESP32 TX -> UNO D12).
  NOTE: This sketch expects the ESP32 to send newline-terminated commands.
  Servo scanning and obstacle decisions are non-blocking except short ultrasonic
  measurements and short turning/reverse manoeuvres.
*/

#include <Servo.h>
#include <SoftwareSerial.h>

// Motor driver pins
const uint8_t ENA = 5;
const uint8_t IN1 = 8;
const uint8_t IN2 = 9;
const uint8_t IN3 = 10;
const uint8_t IN4 = 11;
const uint8_t ENB = 6;

// Ultrasonic pins
const uint8_t FRONT_TRIG = A2;
const uint8_t FRONT_ECHO = A3;
const uint8_t REAR_TRIG  = A4;
const uint8_t REAR_ECHO  = A5;

// Servo pins (as specified by existing wiring)
const uint8_t FRONT_SERVO_PIN = A1;
const uint8_t REAR_SERVO_PIN  = A0;

// ESP32-S3 TX -> UNO D12 RX. UNO D13 TX is not used here.
SoftwareSerial RobotSerial(12, 13); // RX, TX

Servo frontServo;
Servo rearServo;

const int SAFE_DISTANCE_CM = 20;
const int MOTOR_SPEED = 125;          // 0-255; increase only after safe testing
const int SERVO_CENTER = 90;
const int SERVO_LEFT = 145;
const int SERVO_RIGHT = 35;
const unsigned long SCAN_INTERVAL_MS = 160;
const unsigned long TURN_TIME_MS = 420;
const unsigned long REVERSE_TIME_MS = 350;
const unsigned long OBSTACLE_PAUSE_MS = 180;
const unsigned long SENSOR_TIMEOUT_US = 18000UL;

enum MotionState { STOPPED, MOVING_FORWARD, MOVING_BACKWARD, TURNING_LEFT, TURNING_RIGHT };
MotionState motion = STOPPED;

bool patrolEnabled = false;
bool turning = false;
unsigned long motionStartedAt = 0;
unsigned long lastScanAt = 0;
unsigned long lastDecisionAt = 0;

int frontCenterCm = 999;
int rearCenterCm = 999;
int frontLeftCm = 999;
int frontRightCm = 999;
int rearLeftCm = 999;
int rearRightCm = 999;

String usbLine;
String uartLine;

long readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);
  if (duration == 0) return 999; // no echo = unknown/open; don't treat as close
  long cm = duration / 58UL;
  if (cm < 2 || cm > 350) return 999;
  return cm;
}

int readAt(Servo &servo, uint8_t trig, uint8_t echo, int angle) {
  servo.write(angle);
  delay(95); // allow servo to settle for an accurate range sample
  return (int)readDistanceCm(trig, echo);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  motion = STOPPED;
  turning = false;
}

void forwardMotors() {
  // Keep the current direction mapping from the previous code.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
  motion = MOVING_FORWARD;
  motionStartedAt = millis();
}

void backwardMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
  motion = MOVING_BACKWARD;
  motionStartedAt = millis();
}

void leftMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
  motion = TURNING_LEFT;
  motionStartedAt = millis();
  turning = true;
}

void rightMotors() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
  motion = TURNING_RIGHT;
  motionStartedAt = millis();
  turning = true;
}

void centerServos() {
  frontServo.write(SERVO_CENTER);
  rearServo.write(SERVO_CENTER);
}

void printDistances() {
  Serial.print(F("FRONT:")); Serial.print(frontCenterCm);
  Serial.print(F(" REAR:")); Serial.print(rearCenterCm);
  Serial.print(F(" FL:")); Serial.print(frontLeftCm);
  Serial.print(F(" FR:")); Serial.print(frontRightCm);
  Serial.print(F(" RL:")); Serial.print(rearLeftCm);
  Serial.print(F(" RR:")); Serial.println(rearRightCm);
}

void scanBothSides() {
  // Front and rear servos scan together by angle; each sensor samples its own side.
  frontLeftCm  = readAt(frontServo, FRONT_TRIG, FRONT_ECHO, SERVO_LEFT);
  rearLeftCm   = readAt(rearServo, REAR_TRIG, REAR_ECHO, SERVO_LEFT);
  frontRightCm = readAt(frontServo, FRONT_TRIG, FRONT_ECHO, SERVO_RIGHT);
  rearRightCm  = readAt(rearServo, REAR_TRIG, REAR_ECHO, SERVO_RIGHT);
  frontCenterCm = readAt(frontServo, FRONT_TRIG, FRONT_ECHO, SERVO_CENTER);
  rearCenterCm  = readAt(rearServo, REAR_TRIG, REAR_ECHO, SERVO_CENTER);
  centerServos();
  lastScanAt = millis();
  printDistances();
}

void beginAutonomousTurn() {
  stopMotors();
  delay(OBSTACLE_PAUSE_MS);
  scanBothSides();

  // Choose the side with the larger measured clearance.
  int leftClearance = min(frontLeftCm, rearLeftCm);
  int rightClearance = min(frontRightCm, rearRightCm);

  if (leftClearance > SAFE_DISTANCE_CM || rightClearance > SAFE_DISTANCE_CM) {
    if (leftClearance >= rightClearance) {
      Serial.println(F("AUTO: obstacle ahead; turning LEFT"));
      RobotSerial.println(F("AUTO:TURN_LEFT"));
      leftMotors();
    } else {
      Serial.println(F("AUTO: obstacle ahead; turning RIGHT"));
      RobotSerial.println(F("AUTO:TURN_RIGHT"));
      rightMotors();
    }
  } else if (rearCenterCm > SAFE_DISTANCE_CM) {
    Serial.println(F("AUTO: tight ahead; reversing briefly"));
    RobotSerial.println(F("AUTO:REVERSE"));
    backwardMotors();
  } else {
    Serial.println(F("AUTO: boxed in; STOP"));
    RobotSerial.println(F("AUTO:BLOCKED"));
    stopMotors();
    patrolEnabled = false; // fail-safe: stop if no measured escape path
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (!cmd.length()) return;

  Serial.print(F("CMD:")); Serial.println(cmd);

  if (cmd == "START_PATROL" || cmd == "START" || cmd == "AUTO") {
    patrolEnabled = true;
    stopMotors();
    scanBothSides();
    Serial.println(F("PATROL:STARTED"));
    RobotSerial.println(F("PATROL:STARTED"));
    return;
  }
  if (cmd == "STOP_PATROL" || cmd == "STOP" || cmd == "S" || cmd == "EMERGENCY_STOP") {
    patrolEnabled = false;
    stopMotors();
    centerServos();
    Serial.println(F("PATROL:STOPPED"));
    RobotSerial.println(F("PATROL:STOPPED"));
    return;
  }
  if (cmd == "STATUS") {
    printDistances();
    Serial.println(patrolEnabled ? F("MODE:AUTO") : F("MODE:MANUAL/STOP"));
    return;
  }

  // Manual commands disable autonomous mode until START_PATROL is received.
  if (cmd == "F" || cmd == "FORWARD") { patrolEnabled = false; if (frontCenterCm > SAFE_DISTANCE_CM) forwardMotors(); else stopMotors(); }
  else if (cmd == "B" || cmd == "BACKWARD") { patrolEnabled = false; if (rearCenterCm > SAFE_DISTANCE_CM) backwardMotors(); else stopMotors(); }
  else if (cmd == "L" || cmd == "LEFT") { patrolEnabled = false; leftMotors(); }
  else if (cmd == "R" || cmd == "RIGHT") { patrolEnabled = false; rightMotors(); }
  else {
    Serial.println(F("Unknown command"));
    RobotSerial.println(F("ERROR:UNKNOWN_COMMAND"));
  }
}

void readCommands(Stream &stream, String &buffer) {
  while (stream.available()) {
    char c = (char)stream.read();
    if (c == '\r') continue;
    if (c == '\n') {
      processCommand(buffer);
      buffer = "";
    } else if (buffer.length() < 64) {
      buffer += c;
    } else {
      buffer = "";
    }
  }
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(REAR_TRIG, OUTPUT); pinMode(REAR_ECHO, INPUT);

  Serial.begin(9600);
  RobotSerial.begin(9600);

  frontServo.attach(FRONT_SERVO_PIN);
  rearServo.attach(REAR_SERVO_PIN);
  centerServos();
  stopMotors();

  Serial.println(F("PatrolMate autonomous obstacle avoidance ready."));
  Serial.println(F("Send START_PATROL to begin; STOP_PATROL to stop."));
  RobotSerial.println(F("ARDUINO_READY"));
  scanBothSides();
}

void loop() {
  readCommands(Serial, usbLine);
  readCommands(RobotSerial, uartLine);

  unsigned long now = millis();

  // Keep servos scanning repeatedly during patrol, with measurements refreshed each scan.
  if (patrolEnabled && now - lastScanAt >= SCAN_INTERVAL_MS) {
    scanBothSides();
  }

  if (!patrolEnabled) return;

  // Complete timed turn/reverse, then re-check before resuming forward.
  if ((motion == TURNING_LEFT || motion == TURNING_RIGHT) &&
      now - motionStartedAt >= TURN_TIME_MS) {
    stopMotors();
    centerServos();
    lastDecisionAt = now;
  } else if (motion == MOVING_BACKWARD &&
             now - motionStartedAt >= REVERSE_TIME_MS) {
    stopMotors();
    centerServos();
    lastDecisionAt = now;
  }

  // Decide after a turn/reverse or if forward path is blocked.
  if (motion == STOPPED && now - lastDecisionAt >= OBSTACLE_PAUSE_MS) {
    scanBothSides();
    if (frontCenterCm > SAFE_DISTANCE_CM) {
      Serial.println(F("AUTO: path clear; moving forward"));
      RobotSerial.println(F("AUTO:FORWARD"));
      forwardMotors();
    } else {
      Serial.println(F("AUTO: obstacle <= 20 cm"));
      beginAutonomousTurn();
    }
    lastDecisionAt = millis();
  } else if (motion == MOVING_FORWARD && frontCenterCm <= SAFE_DISTANCE_CM) {
    Serial.println(F("AUTO: stopping at 20 cm threshold"));
    stopMotors();
    beginAutonomousTurn();
    lastDecisionAt = millis();
  }
}

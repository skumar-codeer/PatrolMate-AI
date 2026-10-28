#include <SoftwareSerial.h>

// ESP32-S3 TX (GPIO43) -> Arduino D12
SoftwareSerial RobotSerial(12, 13);  // RX, TX

// L298N motor pins
const int ENA = 5;
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;
const int ENB = 6;

// Ultrasonic sensors
const int FRONT_TRIG = A2;
const int FRONT_ECHO = A3;
const int REAR_TRIG  = A4;
const int REAR_ECHO  = A5;

// Encoders
const int LEFT_O1  = 2;
const int LEFT_O2  = 4;
const int RIGHT_O1 = 3;
const int RIGHT_O2 = A0;

const int SPEED = 160;
const int SAFE_DISTANCE = 25;

volatile long leftCount = 0;
volatile long rightCount = 0;

char command = 'S';

long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return -1;
  return duration / 58;
}

bool obstacleDetected(int trigPin, int echoPin) {
  long d = readDistance(trigPin, echoPin);
  return d > 0 && d <= SAFE_DISTANCE;
}

void leftEncoderISR() {
  if (digitalRead(LEFT_O2)) leftCount++;
  else leftCount--;
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_O2)) rightCount++;
  else rightCount--;
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward() {
  if (obstacleDetected(FRONT_TRIG, FRONT_ECHO)) {
    stopMotors();
    return;
  }

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
}

void backward() {
  if (obstacleDetected(REAR_TRIG, REAR_ECHO)) {
    stopMotors();
    return;
  }

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(REAR_TRIG, OUTPUT);
  pinMode(REAR_ECHO, INPUT);

  pinMode(LEFT_O1, INPUT_PULLUP);
  pinMode(LEFT_O2, INPUT_PULLUP);
  pinMode(RIGHT_O1, INPUT_PULLUP);
  pinMode(RIGHT_O2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_O1), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_O1), rightEncoderISR, RISING);

  stopMotors();

  Serial.begin(115200);
  RobotSerial.begin(9600);

  Serial.println("PatrolMate motors, sensors and encoders ready");
}

void loop() {
  while (RobotSerial.available() > 0) {
  char c = RobotSerial.read();

  if (c == 'F' || c == 'B' || c == 'L' ||
      c == 'R' || c == 'S') {
    command = c;

    Serial.print("Received command: ");
    Serial.println(c);
  }
}

  switch (command) {
    case 'F': forward();  break;
    case 'B': backward(); break;
    case 'L': left();     break;
    case 'R': right();    break;
    default:  stopMotors(); break;
  }

  static unsigned long lastReport = 0;
  if (millis() - lastReport >= 500) {
    lastReport = millis();

    long l, r;
    noInterrupts();
    l = leftCount;
    r = rightCount;
    interrupts();

    Serial.print("Left encoder: ");
    Serial.print(l);
    Serial.print(" | Right encoder: ");
    Serial.println(r);
  }

  delay(10);
}
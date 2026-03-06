#include "Helpers.h"

const int CORRECTION = 5; // drift correction for left motor

// gyro variables
int16_t gyroZ;
float gyroZOffset = 0;  // average resting reading, subtracted to remove bias
float currentAngle = 0;
unsigned long lastTime = 0;
bool gyroReady = false;

// global objects
CRGB leds[NUM_LEDS];
Servo scanServo;
int driveSpeed = 0;
float targetHeading = 0;
const int gyroTurnTimeout = 3000;

// Servo
void setServoAngle(int angle) {
  scanServo.write(constrain(angle, 0, 180)); // constrain to avoid physically tangling wires
}

void centerServo() {
  setServoAngle(90);
}

void setupHardware() {
  FastLED.addLeds<NEOPIXEL, RGBLED>(leds, NUM_LEDS);
  FastLED.setBrightness(50);

  pinMode(RightPower, OUTPUT);
  pinMode(LeftPower, OUTPUT);
  pinMode(Left_DIR, OUTPUT);
  pinMode(Right_DIR, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);

  // line sensor pins - reserved for future use
  pinMode(LEFT, INPUT);
  pinMode(MIDDLE, INPUT);
  pinMode(RIGHT, INPUT);

  pinMode(BUTTON, INPUT_PULLUP);

  digitalWrite(STBY, HIGH); // STBY must be HIGH to enable the motor driver

  scanServo.attach(SERVO);
  centerServo();

  // block until button pressed - robot should not move before user is ready
  while (digitalRead(BUTTON) == HIGH) {
  }

  delay(500);

  // gyro is required - flash red and halt if it fails
  if (!setupGyro()) {
    ledOn(CRGB::Red);
    while (true);
  }

  calibrateGyro(); // keep robot still during this
}

// Motors
void setMotor(bool rDir, bool lDir, int rSpeed, int lSpeed) {
  rSpeed = constrain(rSpeed, 0, 255);
  lSpeed = constrain(lSpeed, 0, 255);

  digitalWrite(Right_DIR, rDir);
  digitalWrite(Left_DIR, lDir);
  analogWrite(RightPower, rSpeed);
  analogWrite(LeftPower, constrain(lSpeed - CORRECTION, 0, 255));
}

void stopMotors() {
  analogWrite(RightPower, 0);
  analogWrite(LeftPower, 0);
}

void driveForward(int speed) {
  setMotor(HIGH, HIGH, speed, speed);
}

void driveBackward(int speed) {
  setMotor(LOW, LOW, speed, speed);
}

// Gyro
bool setupGyro() {
  Wire.begin();

  // wake MPU6050 via power management register
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);
  Wire.write(0);
  if (Wire.endTransmission() != 0) {
    gyroReady = false;
    return false; // device did not acknowledge - likely not connected
  }

  // set full scale range to ±250°/s
  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  lastTime = millis();
  gyroReady = true;
  return true;
}

void calibrateGyro() {
  if (!gyroReady) return;

  delay(500); // let sensor settle before sampling
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(GYRO);
    Wire.write(0x47); // GYRO_ZOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(GYRO, 2, true);
    int16_t gz = Wire.read() << 8 | Wire.read();
    sum += gz;
    delay(5);
  }
  gyroZOffset = sum / 100.0;
  currentAngle = 0;
}

int16_t readGyroZ() {
  Wire.beginTransmission(GYRO);
  Wire.write(0x47); // GYRO_ZOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);
  return Wire.read() << 8 | Wire.read();
}

void updateGyroAngle() {
  if (!gyroReady) return;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  gyroZ = readGyroZ();
  float rate = -((gyroZ - gyroZOffset) / 131.0); // 131 LSB per °/s at ±250°/s range
  currentAngle += rate * dt;

  // wrap to -180..180
  if (currentAngle > 180) currentAngle -= 360;
  if (currentAngle < -180) currentAngle += 360;
}

void resetAngle() {
  currentAngle = 0;
}

float getAngle() {
  return currentAngle;
}

// positive angle = right, negative = left
// returns false if turn exceeds GYRO_TURN_TIMEOUT
void gyroTurn(int targetAngle, int speed) {
  if (!gyroReady) return false;

  resetAngle();
  delay(20);

  bool turnRight = targetAngle > 0;
  unsigned long startTime = millis();

  while (abs(currentAngle) < abs(targetAngle)) {
    if (millis() - startTime > gyroTurnTimeout) {
      stopMotors();
      return false;
    }

    updateGyroAngle();

    if (turnRight) {
      setMotor(HIGH, LOW, speed, speed);
    } else {
      setMotor(LOW, HIGH, speed, speed);
    }
  }

  stopMotors();
  delay(150);
  return true;
}

void driveStraightGyro(int speed){
    driveSpeed = speed;
    targetHeading = getAngle();
  }

  void updateDriveStraight(){
    updateGyroAngle();

    float errorAngle = getAngle() - targetHeading;

    int correction = errorAngle * 2;

    int leftSpeed = driveSpeed + correction;
    int rightSpeed = driveSpeed - correction;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    setMotor(HIGH, HIGH, rightSpeed, leftSpeed);
  }

// Ultrasonic
// returns distance in cm, or -1 if invalid or out of range
int getDistance() {
  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10); // pulse must be HIGH for at least 10us
  digitalWrite(US_OUT, LOW);

  long duration = pulseIn(US_IN, HIGH, 30000); // timeout after 30ms (~5m)

  if (duration == 0) return -1;

  int distance = duration * 0.034 / 2; // speed of sound, divided by 2 for round trip

  if (distance < 2 || distance > 400){ //filter out unreasonable readings
    return -1;
  }

  return distance;
}

// LED
void ledOn(CRGB color) {
  leds[0] = color;
  FastLED.show();
}

void ledOff() {
  leds[0] = CRGB::Black;
  FastLED.show();
}

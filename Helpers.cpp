#include "Helpers.h"

const int CORRECTION = 5; // drift correction for left motor

// gyro variables
int16_t gyroZ;
float gyroZOffset = 0;
float currentAngle = 0;
unsigned long lastTime = 0;

// global objects
CRGB leds[NUM_LEDS];
Servo scanServo;

// servo helpers
void setServoAngle(int angle) {
  scanServo.write(constrain(angle, 0, 180)); // restrict from 0-180 to avoid physically tangling with wires
}

void centerServo() {
  setServoAngle(90);
}

// hardware setup
void setupHardware() {
  // led setup
  FastLED.addLeds<NEOPIXEL, RGBLED>(leds, NUM_LEDS);
  FastLED.setBrightness(50); // 0-255

  // motor pins
  pinMode(RightPower, OUTPUT);
  pinMode(LeftPower, OUTPUT);
  pinMode(Left_DIR, OUTPUT);
  pinMode(Right_DIR, OUTPUT);
  pinMode(STBY, OUTPUT);

  // ultrasonic sensor pins
  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);

  // line sensors
  pinMode(LEFT, INPUT);
  pinMode(MIDDLE, INPUT);
  pinMode(RIGHT, INPUT);

  // button pin
  pinMode(BUTTON, INPUT_PULLUP);

  // enable motor driver
  digitalWrite(STBY, HIGH);

  // servo
  scanServo.attach(SERVO);
  centerServo(); // center position

  // wait for button press
  while (digitalRead(BUTTON) == HIGH) {
  }

  delay(500);

  // initialize gyro - hard stop if failed
  if (!setupGyro()) {
    ledOn(CRGB::Red);
    while (true); // hard stop
  }

  calibrateGyro();
}

// motor helpers
void setMotor(bool rDir, bool lDir, int rSpeed, int lSpeed) {
  digitalWrite(Right_DIR, rDir);
  digitalWrite(Left_DIR, lDir);
  analogWrite(RightPower, rSpeed);
  analogWrite(LeftPower, lSpeed - CORRECTION); // correction due to robot drifting to the right
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

// gyro functions
bool setupGyro() {
  Wire.begin();
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B); // power management register
  Wire.write(0);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(GYRO);
  Wire.write(0x1B); // gyro config
  Wire.write(0x00);
  Wire.endTransmission();

  lastTime = millis();
  return true;
}

void calibrateGyro() {
  delay(500);
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(GYRO);
    Wire.write(0x47);
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
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);
  return Wire.read() << 8 | Wire.read();
}

void updateGyroAngle() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  gyroZ = readGyroZ();
  float rate = -((gyroZ - gyroZOffset) / 131.0);
  currentAngle += rate * dt;

  if (currentAngle > 180) currentAngle -= 360;
  if (currentAngle < -180) currentAngle += 360;
}

void resetAngle() {
  currentAngle = 0;
}

float getAngle() {
  return currentAngle;
}

void gyroTurn(int targetAngle, int speed) {
  resetAngle();
  delay(20);

  bool turnRight = targetAngle > 0;

  while (abs(currentAngle) < abs(targetAngle)) {
    updateGyroAngle();

    if (turnRight) {
      setMotor(HIGH, LOW, speed, speed);
    } else {
      setMotor(LOW, HIGH, speed, speed);
    }
  }

  stopMotors();
  delay(150);
}

// ultrasonic
int getDistance() {
  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OUT, LOW);

  long duration = pulseIn(US_IN, HIGH, 30000);
  if (duration == 0) return 0;
  return duration * 0.034 / 2;
}

// led helpers
void ledOn(CRGB color) {
  leds[0] = color;
  FastLED.show();
}

void ledOff() {
  leds[0] = CRGB::Black;
  FastLED.show();
}
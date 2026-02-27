#include "Helpers.h"

// Test parameters
int testSpeed = 100;

void setup() {
  setupHardware();
  Serial.begin(9600);

  Serial.println("Starting hardware test...");
}

void loop() {
  //       LED Test      
  Serial.println("LED Test: Red");
  ledOn(CRGB::Red);
  delay(500);
  Serial.println("LED Test: Green");
  ledOn(CRGB::Green);
  delay(500);
  Serial.println("LED Test: Blue");
  ledOn(CRGB::Blue);
  delay(500);
  ledOff();
  delay(500);

  //       Servo Test      
  Serial.println("Servo Test: Sweeping 0-180");
  for (int angle = 0; angle <= 180; angle += 30) {
    setServoAngle(angle);
    delay(300);
  }
  for (int angle = 180; angle >= 0; angle -= 30) {
    setServoAngle(angle);
    delay(300);
  }
  centerServo();
  delay(500);

  //       Motor Test      
  Serial.println("Motor Test: Forward");
  driveForward(testSpeed);
  delay(1000);
  Serial.println("Motor Test: Backward");
  driveBackward(testSpeed);
  delay(1000);
  Serial.println("Motor Test: Stop");
  stopMotors();
  delay(500);

  //       Gyro Test      
  updateGyroAngle();
  float angle = getAngle();
  Serial.print("Current gyro angle: ");
  Serial.println(angle);
  delay(500);

  //       Ultrasonic Test 
  int distance = getDistance();
  Serial.print("Distance (cm): ");
  Serial.println(distance);
  delay(500);

  // Repeat the test
}
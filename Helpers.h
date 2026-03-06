#pragma once

#include <Wire.h>
#include <FastLED.h>
#include <Servo.h>

// Pin Definitions
#define NUM_LEDS 2
#define RGBLED 4
#define RightPower 5
#define LeftPower 6
#define Right_DIR 7
#define Left_DIR 8
#define SERVO 10
#define US_OUT 13 // ultrasonic sensor trigger
#define US_IN 12  // ultrasonic sensor echo
#define STBY 3
#define LEFT A2   // left colour sensor
#define MIDDLE A1 // middle colour sensor
#define RIGHT A0  // right colour sensor
#define BUTTON 2
#define GYRO 0x68   // Correct MPU6050 address

extern const int CORRECTION; // drift correction for left motor

// gyro variables
extern int16_t gyroZ;
extern float gyroZOffset;
extern float currentAngle;
extern unsigned long lastTime;
extern const int gyroTurnTimeout;

// global objects
extern CRGB leds[NUM_LEDS];
extern Servo scanServo;
extern int driveSpeed;
extern float targetHeading;

// servo helpers
void setServoAngle(int angle);
void centerServo();

// motor helpers
void setMotor(bool rDir, bool lDir, int rSpeed, int lSpeed);
void stopMotors();
void driveForward(int speed);
void driveBackward(int speed);

// gyro functions
bool setupGyro();
void calibrateGyro();
int16_t readGyroZ();
void updateGyroAngle();
void resetAngle();
float getAngle();
void gyroTurn(int targetAngle, int speed);
void driveStraightGyro(int speed);
void updateDriveStraight();

// ultrasonic
int getDistance();

// led helpers
void ledOn(CRGB color);
void ledOff();

// hardware setup
void setupHardware();

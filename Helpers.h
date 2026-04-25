/**
 * Helpers.h
 * ---------
 * Hardware abstraction layer and utility functions for the Robot Maze Solver.
 * This file manages Pin definitions, Gyroscope interfacing, and Motor control.
 */

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
#define US_OUT 13 // Ultrasonic trigger
#define US_IN 12  // Ultrasonic echo
#define STBY 3
#define LEFT A2   // Left line sensor
#define MIDDLE A1 // Middle line sensor
#define RIGHT A0  // Right line sensor
#define BUTTON 2
#define GYRO 0x68

// Hardware Constants & Objects 
extern const int correction; // Motor bias correction (Left motor) + line following turning
extern CRGB leds[NUM_LEDS];
extern Servo scanServo;

// Gyro
extern int16_t gyroZ;
extern float gyroZOffset;
extern float currentAngle;
extern unsigned long lastTime;
extern bool gyroReady;
extern const int gyroTurnTimeout;

// Navigation & Drive Variables
extern int driveSpeed;
extern int minDriveSpeed;
extern float targetHeading;
extern int turnSpeed;
extern bool isDriving;

// Variable Speed & Distance State
extern int currentDriveSpeed;
extern unsigned long lastDistanceRead;
extern const int readInterval;

// Servo Control
void setServoAngle(int angle);
void centerServo();

// Motor Control
void setMotor(bool rDir, bool lDir, int rSpeed, int lSpeed);
void stopMotors();

// Gyroscope Interface
bool setupGyro();
void calibrateGyro();
int16_t readGyroZ();
void updateGyroAngle();
void resetAngle();
float getAngle();
void gyroTurn(int targetAngle, int speed = 80);

// Driving Logic
void driveStraightGyro(int speed);
void updateDriveStraight();
void updateVariableSpeed();
void driveWithVariableSpeed(int baseSpeed);

// Line Sensor Logic
bool isOnLine(int value, int min, int max);
bool stopBeforeLine(int min, int max);

// Ultrasonic & Decision Making
int getDistance();
void turnDecision();

// LEDs
void ledOn(CRGB color);
void ledOff();
void flashLed(CRGB colorA, CRGB colorB, int intervalMs);

// System Initialization
void setupHardware();

// Communication
void sendMorseCode(const char* message, CRGB color);
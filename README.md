# Arduino Robot Helpers 

## Overview
This project provides reusable hardware helpers for a small robot, including:
- Motor control
- Gyroscope integration
- Ultrasonic distance sensing
- Servo control
- LED status indicators

## Hardware 
- Motors connected via PWM and direction pins
- NeoPixel LED strip for status
- MPU6050 gyro sensor
- HC-SR04 ultrasonic sensor
- Servo motor for scanning
- 3 analog colour line sensors
- Button for start/reset

## How to Run
1. Open project file
2. Verify that `Helpers.h` and `Helpers.cpp` are in the same folder as project
3. Make sure to `#include "Helpers.h"`

## Known Issues
- Gyro requires calibration at startup.
- Ultrasonic readings can be noisy if objects are angled.
- LED brightness is fixed at 50/255.
- Robot drifts towards the right when moving forward, not resolved.

# Arduino Robot Helpers 

## Overview
This project provides reusable hardware helpers for a small robot, including:
- Motor control
- Gyroscope integration
- Ultrasonic distance sensing
- Servo control
- LED status indicators

## Hardware Assumptions
- Motors connected via PWM and direction pins
- NeoPixel LED strip for status
- MPU6050 gyro sensor
- HC-SR04 ultrasonic sensor
- Servo motor for scanning
- 3 analog line sensors (currently not used)
- Button for start/reset

## How to Run
1. Open the project folder in Arduino IDE.
2. Verify that `Helpers.h` is included.
3. Upload any `.ino` file that includes `Helpers.h` to the board.
4. Press the button to initialize hardware.

## Known Issues
- Gyro requires calibration at startup.
- Ultrasonic readings can be noisy if objects are angled.
- LED brightness is fixed at 50/255.
- robot drift towards the right when moving forward, not resolved.
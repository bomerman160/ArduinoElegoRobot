# Arduino Robot Helpers

## Overview
This project provides reusable hardware helpers for a small Arduino-based robot, including:
- Motor control with drift correction
- Gyroscope integration for angle tracking and precise turns and driving straight
- Ultrasonic distance sensing for obstacle detection
- Servo control(0–180°)
- RGB LED status indicators via NeoPixel
- 3 analog line sensors

## Hardware Assumptions
- Arduino Uno or compatible board
- Motors connected via PWM
- NeoPixel LED strip

## How to Run
1. Install required libraries in Arduino IDE via Sketch → Include Library → Manage Libraries: FastLED, Wire (built-in), Servo (built-in).
2. Open `roboTest.ino` in Arduino IDE — `Helpers.h` and `Helpers.cpp` must be in the same folder as 'roboTest.ino'
3. Select your board under Tools → Board (e.g. Arduino Uno) and your port under Tools → Port.
4. Click Verify to compile, then Upload to flash to the board.
5. Press the button to start — the robot will not run until the button is pressed.
6. The gyro will auto-calibrate on startup. Keep the robot still and flat during this window.
7. If the LED turns red and the robot halts, the gyro failed to initialize

## Known Issues
- Gyro requires calibration at startup; keep robot still or angle tracking will be off.
- Gyro drift accumulates over time — best for short, discrete turns rather than continuous tracking.
- gyroTurn overshoots turns like 90 degrees, small accumulation over time
- robot fails to avoid endless loops (i.e 4 right turns in a row)
- trouble identifying the black line despite the range for the black line
- issues with the robot not correcting while it follows the line
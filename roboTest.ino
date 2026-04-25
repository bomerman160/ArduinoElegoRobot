/*
 * Robot Maze Solver
 * -----------------
 * A state-based navigation system using IR sensors for line following,
 * an Ultrasonic sensor for wall avoidance, and Gyroscope stabilization.
 */

#include "Helpers.h"

// Calibration Thresholds & Speed Constants
int whiteMin = 20;
int whiteMax = 150;
int blackMin = 840;
int blackMax = 875;

int lineFollowSpeed = 45;
int blackTapeCount  = 0;
bool followingLine  = false;
bool mazeComplete   = false;

/*
 * Validates if analog sensor reading falls within the 
 * calibrated range for white tape.
 */
bool isOnWhite(int value) {
  return value >= whiteMin && value <= whiteMax;
}

/*
 * Validates if analog sensor reading falls within the 
 * calibrated range for black tape.
 */
bool isOnBlack(int value) {
  return value >= blackMin && value <= blackMax;
}

/*
 * Scans the Left, Middle, and Right sensors to determine if 
 * the robot has encountered any part of the white path.
 */
bool whiteLineDetected() {
  int L = analogRead(LEFT);
  int M = analogRead(MIDDLE);
  int R = analogRead(RIGHT);
  return isOnWhite(L) || isOnWhite(M) || isOnWhite(R);
}

/*
 * Checks for a full-stop/intersection where all sensors 
 * detect the black tape simultaneously.
 */
bool blackLineDetected() {
  int L = analogRead(LEFT);
  int M = analogRead(MIDDLE);
  int R = analogRead(RIGHT);
  return isOnBlack(L) && isOnBlack(M) && isOnBlack(R);
}

/*
 * Celebration Sequence: Triggered upon reaching the maze exit.
 * Performs 360-degree spins and RGB LED cycling.
 */
void celebrate() {
  stopMotors();

  CRGB colors[] = {
    CRGB::Red,
    CRGB::Blue,
    CRGB::Green,
    CRGB::Yellow,
    CRGB::Purple,
    CRGB::Cyan,
    CRGB::Orange
  };

  int colorIndex = 0;

  for (int i = 0; i < 4; i++) {
    // Perform clockwise rotation
    gyroTurn(360, turnSpeed);

    for (int j = 0; j < 6; j++) {
      ledOn(colors[colorIndex]);
      colorIndex = (colorIndex + 1) % 7;
      delay(100);
    }

    // Perform counter-clockwise rotation
    gyroTurn(-360, turnSpeed);

    for (int j = 0; j < 6; j++) {
      ledOn(colors[colorIndex]);
      colorIndex = (colorIndex + 1) % 7;
      delay(100);
    }
  }

  // Signal completion via Morse Code and static LED
  sendMorseCode("MAZE COMPLETED", CRGB::White);
  ledOn(CRGB::Green);
  while (true); // Lock execution post-completion
}

void setup() {
  Serial.begin(9600);
  setupHardware();
  delay(1000);
  driveWithVariableSpeed(driveSpeed);
  isDriving = true;
}

void loop() {
  // Exit loop if the completion flag is set
  if (mazeComplete) return; 

  int currentDist = getDistance();
  updateGyroAngle();
  updateVariableSpeed(); 

  // line following mode
  if (followingLine) {
    /* * Monitor for black tape using only the MIDDLE sensor.
     * This ensures the robot identifies the finish line/checkpoint 
     * even while making micro-adjustments on the white line.
     */
    if (analogRead(MIDDLE) >= 840) {
      stopMotors();
      delay(300);
      blackTapeCount++;

      // Finish condition: Second encounter with black tape
      if (blackTapeCount >= 2) {
        mazeComplete = true;
        celebrate();
      } 
      else {
        // move forward past the first black line to prevent immediate re-triggering
        ledOn(CRGB::Orange);
        setMotor(HIGH, HIGH, driveSpeed, driveSpeed);
        delay(600); 
        
        stopMotors();
        followingLine = false;
        delay(300);
        driveStraightGyro(driveSpeed);
      }
      return;
    } 

    // Execute line following logic
    followLine(whiteMin, whiteMax, currentDriveSpeed);
    return;
  }

  // black line avoidance
  // If a black line is hit during navigation mode, perform a 90-degree escape turn
  if (blackTapeCount >= 1 && analogRead(MIDDLE) >= 840) {
    ledOn(CRGB::Red);
    stopMotors();
    delay(200);
    gyroTurn(90, turnSpeed); 
    driveStraightGyro(driveSpeed);
    return;
  }

  // white line search
  // Transition from free-roam navigation to line-following mode
  if (whiteLineDetected()) {
    stopMotors();
    delay(150);
    followingLine = true;
    currentDriveSpeed = lineFollowSpeed; 
    return;
  }

  // wall avoidance
  // Basic obstacle avoidance using the ultrasonic sensor
  if (currentDist <= 8 && currentDist > 0) {
    stopMotors();
    delay(120);
    turnDecision();
    driveStraightGyro(driveSpeed);
    return;
  }

  updateDriveStraight();
}
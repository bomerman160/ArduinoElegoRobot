#include "Helpers.h"

const int correction = 10;

int16_t gyroZ;
float gyroZOffset = 0;
float currentAngle = 0;
unsigned long lastTime = 0;
bool gyroReady = false;

CRGB leds[NUM_LEDS];
Servo scanServo;

int driveSpeed = 100;
int minDriveSpeed = 80;
float targetHeading = 0;
int turnSpeed = 100;
const int gyroTurnTimeout = 3000;
bool isDriving = false;

int currentDriveSpeed = 100;
unsigned long lastDistanceRead = 0;
const int readInterval = 50;

int turnDirectionCount = 0;
int lastTurnDirection = 0; // 1 for right, -1 for left, 0 for none
const int MAX_CONSECUTIVE_TURNS = 4;

// Morse code timing (in milliseconds)
const int MORSE_DOT = 200;
const int MORSE_DASH = 600;
const int MORSE_SYMBOL_GAP = 200;
const int MORSE_LETTER_GAP = 600;

void setupHardware(){
  FastLED.addLeds<NEOPIXEL, RGBLED>(leds, NUM_LEDS);
  FastLED.setBrightness(50);

  pinMode(RightPower, OUTPUT);
  pinMode(LeftPower, OUTPUT);
  pinMode(Left_DIR, OUTPUT);
  pinMode(Right_DIR, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);

  pinMode(LEFT, INPUT);
  pinMode(MIDDLE, INPUT);
  pinMode(RIGHT, INPUT);

  pinMode(BUTTON, INPUT_PULLUP);

  digitalWrite(STBY, HIGH);

  scanServo.attach(SERVO);
  centerServo();

  // White = waiting for button press
  ledOn(CRGB::White);
  while (digitalRead(BUTTON) == HIGH){
  }

  delay(500);

  // Orange = gyro setup
  ledOn(CRGB::Orange);
  if (!setupGyro()){
    ledOn(CRGB::Red);
    while (true);
  }

  // Green flash = ready
  for (int i = 0; i < 3; i++){
    ledOn(CRGB::Green);
    delay(200);
    ledOff();
    delay(200);
  }
}

void ledOn(CRGB color){
  leds[0] = color;
  FastLED.show();
}

void ledOff() {
  leds[0] = CRGB::Black;
  FastLED.show();
}

void flashLed(CRGB colorA, CRGB colorB, int intervalMs){
  if (millis() % (intervalMs * 2) < intervalMs) {
    ledOn(colorA);
  } else {
    ledOn(colorB);
  }
}

bool setupGyro(){
  Wire.begin();
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);
  Wire.write(0);
  byte error = Wire.endTransmission();

  if (error != 0) {
    gyroReady = false;
    return false;
  }

  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  lastTime = millis();
  gyroReady = true;
  return true;
}

void calibrateGyro(){
  delay(500);

  long sum = 0;
  int samples = 100;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(GYRO);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(GYRO, 2, true);

    int16_t gz = Wire.read() << 8 | Wire.read();
    sum += gz;
    delay(10);
  }

  gyroZOffset = sum / samples;
  currentAngle = 0;
}

void setMotor(bool rDir, bool lDir, int rSpeed, int lSpeed){
  rSpeed = constrain(rSpeed, 0, 255);
  lSpeed = constrain(lSpeed, 0, 255);

  digitalWrite(Right_DIR, rDir);
  digitalWrite(Left_DIR, lDir);
  analogWrite(RightPower, rSpeed);
  analogWrite(LeftPower, lSpeed);
}

void stopMotors(){
  analogWrite(RightPower, 0);
  analogWrite(LeftPower, 0);
  isDriving = false;
  currentDriveSpeed = 0;
  // White = stopped
  ledOn(CRGB::White);
}

int16_t readGyroZ(){
  Wire.beginTransmission(GYRO);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);

  int16_t gz = Wire.read() << 8 | Wire.read();
  return gz;
}

void updateGyroAngle(){
  if (!gyroReady) return;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  gyroZ = readGyroZ();

  float gyroRate = -((gyroZ - gyroZOffset) / 131.0);

  currentAngle += gyroRate * dt;
}

void resetAngle(){
  currentAngle = 0;
  lastTime = millis();
}

float getAngle(){
  return currentAngle;
}

void gyroTurn(int targetAngle, int speed){
  if (!gyroReady) return;

  resetAngle();
  delay(20);

  unsigned long startTime = millis();
  float overshootCompensation = 3.0;
  float adjustedTarget = abs(targetAngle) - overshootCompensation;

  while (abs(currentAngle) < adjustedTarget) {
    if (millis() - startTime > gyroTurnTimeout) {
      stopMotors();
      return;
    }

    updateGyroAngle();

    float remaining = adjustedTarget - abs(currentAngle);
    int adjustedSpeed = speed;

    if (remaining < 60) {
      adjustedSpeed = max((int)(speed * (remaining / 60.0)), 75);
    }

    // Opposite directions = pivot on the spot
    if (targetAngle > 0) {
      // Cyan = turning right
      ledOn(CRGB::Cyan);
      setMotor(HIGH, LOW, adjustedSpeed, adjustedSpeed);
    }
    else {
      // Magenta = turning left
      ledOn(CRGB::Magenta);
      setMotor(LOW, HIGH, adjustedSpeed, adjustedSpeed);
    }

    delay(5);
  }

  stopMotors();
  delay(100);
}

void driveStraightGyro(int speed){
  driveSpeed = speed;
  currentDriveSpeed = speed;
  targetHeading = getAngle();
  isDriving = true;
  centerServo();
}

void driveWithVariableSpeed(int baseSpeed){
  driveStraightGyro(baseSpeed);
  updateVariableSpeed();
  updateDriveStraight();
}

void updateVariableSpeed(int distance){
  if (!isDriving) return;
  
  // If sensor times out (0), keep current speed rather than resetting
  if (distance <= 0) return;

  if (distance >= 100) {
    currentDriveSpeed = driveSpeed; // Max speed
  }
  else if (distance >= 50) {
    currentDriveSpeed = driveSpeed - 25; // Mid speed
  }
  else if (distance >= 20) {
    currentDriveSpeed = minDriveSpeed; // Minimum crawl
  }
  else {
    currentDriveSpeed = minDriveSpeed;
  }

  currentDriveSpeed = constrain(currentDriveSpeed, minDriveSpeed, 255);
}

void updateVariableSpeed(){
  if (!isDriving) return;
  int distance = getDistance();
  if (distance <= 0) return;

  if (distance >= 100){
    currentDriveSpeed = driveSpeed;
  } 
  else if (distance >= 50){
    currentDriveSpeed = driveSpeed - 30;
  } else if (distance >= 20){
    currentDriveSpeed = minDriveSpeed;
  }

  currentDriveSpeed = constrain(currentDriveSpeed, minDriveSpeed, 255);
}

void updateDriveStraight(){
  if (!isDriving) return;

  updateGyroAngle();

  float errorAngle = getAngle() - targetHeading;
  int correctionVal = errorAngle * 4;

  int left = currentDriveSpeed - correctionVal;
  int right = currentDriveSpeed + correctionVal;

  setMotor(HIGH, HIGH, right, left);
}

int getDistance(){
  int validReading = 0;
  int attempts = 0;

  while (validReading == 0 && attempts < 3){
    if (attempts > 0) delay(60);

    digitalWrite(US_OUT, LOW);
    delayMicroseconds(2);
    digitalWrite(US_OUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_OUT, LOW);

    long duration = pulseIn(US_IN, HIGH, 30000);
    int distance = duration * 0.034 / 2;

    if (duration > 0 && distance <= 200){
      validReading = distance;
    }

    attempts++;
  }

  return validReading;
}

void turnDecision(){
  // Orange = scanning
  ledOn(CRGB::Orange);

  setServoAngle(180);
  delay(500);
  int rightDistance = getDistance();
  if (rightDistance == 0) rightDistance = 999;

  setServoAngle(0);
  delay(500);
  int leftDistance = getDistance();
  if (leftDistance == 0) leftDistance = 999;

  centerServo();

  bool leftBlocked = leftDistance < 25;
  bool rightBlocked = rightDistance < 25;

  int chosenDirection = 0; // 1 for right, -1 for left, 2 for 180

  // Check if we've been turning the same direction too many times
  bool forceOppositeTurn = (turnDirectionCount >= MAX_CONSECUTIVE_TURNS);

  if (leftBlocked && rightBlocked){
    chosenDirection = 2; // 180 degree turn
    turnDirectionCount = 0; // Reset counter on U-turn
    lastTurnDirection = 0;
  }
  else if (leftBlocked){
    chosenDirection = 1; // turn right
  }
  else if (rightBlocked){
    chosenDirection = -1; // turn left
  }
  else if (leftDistance > rightDistance){
    chosenDirection = -1; // turn left (towards farther distance)
  }
  else if (rightDistance > leftDistance){
    chosenDirection = 1; // turn right (towards farther distance)
  }
  else{
    chosenDirection = -1; // default left
  }

  // Force opposite turn if stuck in loop
  if (forceOppositeTurn && chosenDirection != 2 && chosenDirection != 0){
    if (lastTurnDirection == 1){
      chosenDirection = -1; // Force left if we kept turning right
      ledOn(CRGB::Magenta); // Indicate forced turn
      delay(500);
    }
    else if (lastTurnDirection == -1){
      chosenDirection = 1; // Force right if we kept turning left
      ledOn(CRGB::Cyan); // Indicate forced turn
      delay(500);
    }
    turnDirectionCount = 0; // Reset counter after forcing opposite
  }

  // Execute the turn
  if (chosenDirection == 1){
    gyroTurn(90, turnSpeed);
    if (lastTurnDirection == 1){
      turnDirectionCount++;
    } else {
      turnDirectionCount = 1;
      lastTurnDirection = 1;
    }
  }
  else if (chosenDirection == -1){
    gyroTurn(-90, turnSpeed);
    if (lastTurnDirection == -1){
      turnDirectionCount++;
    } else {
      turnDirectionCount = 1;
      lastTurnDirection = -1;
    }
  }
  else if (chosenDirection == 2){
    // Red = U-turn
    ledOn(CRGB::Red);
    gyroTurn(180, turnSpeed);
    turnDirectionCount = 0;
    lastTurnDirection = 0;
  }
}

void setServoAngle(int angle){
  static int lastAngle = -1;
  angle = constrain(angle, 0, 180);

  if (angle != lastAngle) {
    scanServo.write(angle);
    delay(15);
    lastAngle = angle;
  }
}

void centerServo(){
  setServoAngle(90);
}

void followLine(int minVal, int maxVal, int speed){
  int L = analogRead(LEFT);
  int M = analogRead(MIDDLE);
  int R = analogRead(RIGHT);

  bool leftOnLine   = isOnLine(L, minVal, maxVal);
  bool middleOnLine = isOnLine(M, minVal, maxVal);
  bool rightOnLine  = isOnLine(R, minVal, maxVal);

  // If ALL sensors are on the line, DRIVE FORWARD
  if (leftOnLine && middleOnLine && rightOnLine){
    ledOn(CRGB::Green);
    setMotor(HIGH, HIGH, speed, speed);
  }
  // If the left is on but the right is NOT, pivot left until right hits
  else if (leftOnLine){
    ledOn(CRGB::Blue);
    setMotor(HIGH, LOW, speed + correction, speed - correction);
  }
  // If the right is on but the left is NOT, pivot right until left hits
  else if (rightOnLine){
    ledOn(CRGB::Yellow);
    setMotor(LOW, HIGH, speed - correction, speed + correction);
  }
}

bool isOnLine(int value, int min, int max){
  return (value >= min && value <= max);
}

//morse code
void sendMorseCode(const char* message, CRGB color){
  // Morse code mapping (A-Z and 0-9)
  const char* morseMap[] = {
    ".-",    // A
    "-...",  // B
    "-.-.",  // C
    "-..",   // D
    ".",     // E
    "..-.",  // F
    "--.",   // G
    "....",  // H
    "..",    // I
    ".---",  // J
    "-.-",   // K
    ".-..",  // L
    "--",    // M
    "-.",    // N
    "---",   // O
    ".--.",  // P
    "--.-",  // Q
    ".-.",   // R
    "...",   // S
    "-",     // T
    "..-",   // U
    "...-",  // V
    ".--",   // W
    "-..-",  // X
    "-.--",  // Y
    "--..",  // Z
    "-----", // 0
    ".----", // 1
    "..---", // 2
    "...--", // 3
    "....-", // 4
    ".....", // 5
    "-....", // 6
    "--...", // 7
    "---..", // 8
    "----."  // 9
  };
  
  for (int i = 0; message[i] != '\0'; i++){
    char c = message[i];
    
    // Handle space between words
    if (c == ' '){
      delay(MORSE_LETTER_GAP);
      continue;
    }
    
    // Convert character to uppercase
    if (c >= 'a' && c <= 'z'){
      c = c - 32;
    }
    
    int index = -1;
    
    // Find character in mapping
    if (c >= 'A' && c <= 'Z'){
      index = c - 'A';
    } else if (c >= '0' && c <= '9'){
      index = 26 + (c - '0');
    }
    
    if (index != -1){
      const char* code = morseMap[index];
      
      // Send the morse code for this character
      for (int j = 0; code[j] != '\0'; j++) {
        ledOn(color);
        
        if (code[j] == '.'){
          delay(MORSE_DOT);
        } else if (code[j] == '-'){
          delay(MORSE_DASH);
        }
        
        ledOff();
        delay(MORSE_SYMBOL_GAP);
      }
      
      // Gap between letters
      delay(MORSE_LETTER_GAP);
    }
  }
}
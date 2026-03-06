#include "Helpers.h"

void setup() {
  setupHardware();
  Serial.begin(9600);
}

void loop() {
  driveStraightGyro();

  updateDriveStraight();
}

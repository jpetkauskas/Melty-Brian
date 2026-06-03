#include "arduino_secrets.h"

#include <math.h>

uint8_t motorPin;

void runMotorDutyCycle(double speed) { // Speed ranges from 0 to 1
  int dutyCycle = floor(speed * 255);
  analogWrite(motorPin, dutyCycle);
}

void setup() {
  motorPin = 0; // Change
  runMotorDutyCycle(0.25); // Test when run once
}

void loop() {
  // runMotorDutyCycle(0.25); // Test when run continuously
}
#include <Arduino.h>
#include "SoraDrive.hpp"

// put function declarations here:
int myFunction(int, int);

SoraDrive board{};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  board.init();
  board.initGyro();
  board.enableMotors(true);
  board.setMotorAOutput(0);
  board.setMotorBOutput(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  board.printYaw();
  delay(50);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}


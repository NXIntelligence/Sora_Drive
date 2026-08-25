#include <Arduino.h>
#include "SoraDrive.hpp"

SoraDrive soraDrive{};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  soraDrive.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}


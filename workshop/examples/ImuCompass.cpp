#include <Arduino.h>
#include "SoraDrive.hpp"

SoraDrive board{};
void setup() {
  Serial.begin(115200); // begin serial communication at 115200 baud rate
  board.init(); // initialise the board
  board.initImu(); // initialise the IMU with the default settings
}

void loop() {
    float currentHeading = board.getHeadingAngle(); // get the current heading angle
    Serial.print("Current Heading: ");
    Serial.println(currentHeading); // print the current heading angle to the serial monitor
    delay(100); // wait for 100 milliseconds
}


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
  // Passing the colours straight in
  float hue = currentHeading * 182; // convert the heading angle to a hue value (0-65535)
  // set the color of the neopixel based on the heading angle
  board.setNeoPixelColorHSV(0, hue, 255, 255);
  board.setNeoPixelColorHSV(1, hue, 255, 255);

  delay(100); // wait for 100 milliseconds
}


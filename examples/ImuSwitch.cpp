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
  // turn the neopixels green if the heading is greater than 90, otherwise turn it blue
  if(currentHeading > 90.0f) {
    board.setNeoPixelColor(0, 0, 255, 0); // set the first neopixel to green
    board.setNeoPixelColor(1, 0, 255, 0); // set the second neopixel to green
  } else {
    board.setNeoPixelColor(0, 0, 0, 255); // set the first neopixel to blue
    board.setNeoPixelColor(1, 0, 0, 255); // set the second neopixel to blue
  }
  delay(100); // wait for 100 milliseconds
}
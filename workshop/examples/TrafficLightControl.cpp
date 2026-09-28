#include <Arduino.h>
#include "SoraDrive.hpp"

SoraDrive board{};
void setup() {
  board.init(); // initialise the board
  pinMode(1, INPUT); // set pin 1 as input from button
  pinMode(2, INPUT); // set pin 2 as input from button
}

void loop() {
  if (digitalRead(1) == true) {
    // sets the neopixel to yellow
    board.setNeoPixelColor(0, 255, 255, 0); // sets led 1 to yellow
    board.setNeoPixelColor(1, 255, 255, 0); // sets led 2 to yellow
  }else if (digitalRead(2) == true) {
    // sets the neopixel to red
    board.setNeoPixelColor(0, 255, 0, 0); // sets led 1 to red
    board.setNeoPixelColor(1, 255, 0, 0); // sets led 2 to red
  } else {
    // sets the neopixel to green
    board.setNeoPixelColor(0, 0, 255, 0); // sets led 1 to green
    board.setNeoPixelColor(1, 0, 255, 0); // sets led 2 to green
  }
  delay(50); // Wait 50ms before looping, to prevent spam on data line
}


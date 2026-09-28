#include <Arduino.h>
#include "SoraDrive.hpp"

SoraDrive board{};
void setup() {
  board.init(); // initialise the board
  pinMode(4, INPUT); // sets pin 4 to read voltage
}

void loop() {
  bool buttonState = digitalRead(4); // reads whether pin 4 is receiving voltage or not
  if (buttonState == true) { // if buttonstate is true then...
    board.setNeoPixelColor(0, 0, 255, 0); // sets led 1 to green
  } else {
    board.setNeoPixelColor(0, 0, 0, 0); // turns led 1 off
  }
  delay(50);
}


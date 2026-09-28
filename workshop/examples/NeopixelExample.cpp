#include <Arduino.h>
#include "SoraDrive.hpp"

SoraDrive board{};
void setup() {
  board.init(); // initialise the board
  board.setNeoPixelColor(0, 255, 0, 0); // sets led 1 to red
  board.setNeoPixelColor(1, 0, 0, 255); // sets led 2 to blue
}

void loop() {

}


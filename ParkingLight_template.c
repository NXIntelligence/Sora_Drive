#include <Arduino.h>
#include "SoraDrive.hpp"

SoraDrive board{};

void setup() {
    board.init();

    pinMode(1, INPUT);   // Parking Spot A traffic sensor
    pinMode(2, INPUT);   // Parking Spot B traffic sensor
}

void loop() {
    bool parkingSpotA = digitalRead(1);   
    bool parkingSpotB = digitalRead(2);   

    if (/*need to check if BOTH spots are occupied*/) {
        // set neopixels to red
    }
    else if (/*need to check if AT LEAST ONE spot is occupied*/) {
        // set neopixels to yellow
    }
    else {  // NO spots are occupied, empty
        // set neopixels to green
    }
    delay(50);
}
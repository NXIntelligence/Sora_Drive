#include <Arduino.h>
#include "SoraDrive.hpp"

SoraDrive soraDrive{};

void setup() {
    soraDrive.init();       // waking up the board
    Serial.begin(115200);   // begin serial communication at 115200 baud rate
    soraDrive.initImu();    // Getting IMU ready with the default settings
}

void loop() {
    float currentHeading = soraDrive.getHeadingAngle();   // get the current heading angle

    // turn the neopixels green if the heading is greater than 90, otherwise turn it blue
    if (/*Fill in the code here*/) {
        // do something
    } else {
        // do something else
    }

    delay(100);   // wait for 100 milliseconds
}

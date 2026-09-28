#include <Arduino.h>
#include "SoraDrive.hpp"

SoraDrive board{};

void setup() {
  Serial.begin(115200); // begin serial communication at 115200 baud rate
  board.init(); // initialise the SoraDrive board
  pinMode(1, INPUT); // set pin 1 as input for potentiometer
  board.enableMotors(true); // enable the motors
}

void loop() {
  int potValue = analogRead(1); // reads the voltage 0 - 3.3v but returns a value between 0 - 4095
  Serial.println(potValue); // print the potentiometer value to the serial monitor
  float motorOutput = (potValue / 4095.0f) * 100.0f; // convert the potentiometer value to a percentage (0-100)
  board.setMotorBOutput(motorOutput); // set the output of motor A based on the potentiometer value
  delay(100); // wait for 100 milliseconds
}
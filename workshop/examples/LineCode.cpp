#include <Arduino.h>
#include "SoraDrive.hpp"

// Define input pins for digital IR sensors (adjust pin numbers to match your wiring)
constexpr int PIN_IR_LEFT   = 1;
constexpr int PIN_IR_CENTER = 2;
constexpr int PIN_IR_RIGHT  = 3;

SoraDrive soraDrive{};

void drive(float fwd, float turn);
void logic1();
void logic2();

void setup() {
  Serial.begin(115200); // begin serial communication at 115200 baud rate
  soraDrive.init(); // initialise the board
  soraDrive.enableMotors(true); // enable the motors

  // Initialize the line sensor pins as inputs
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_CENTER, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);
}

float BASE_SPEED = 40.0f;
float TURN_SPEED = 20.0f;

void loop() {
    bool leftSeen = digitalRead(PIN_IR_LEFT);
    bool centerSeen = digitalRead(PIN_IR_CENTER);
    bool rightSeen = digitalRead(PIN_IR_RIGHT);
    
    // logic1();
    logic2();

    Serial.print(">left_sensor:");
    Serial.print(leftSeen ? "DETECTED" : "OFF");
    Serial.println("|t");

    Serial.print(">center_sensor:");
    Serial.print(centerSeen ? "DETECTED" : "OFF");
    Serial.println("|t");

    Serial.print(">right_sensor:");
    Serial.print(rightSeen ? "DETECTED" : "OFF");
    Serial.println("|t");

}

// Memory flags to remember which way to keep turning
bool searchingLeft = false;
bool searchingRight = false;
void logic2() {
    bool leftSeen = digitalRead(PIN_IR_LEFT);
    bool centerSeen = digitalRead(PIN_IR_CENTER);
    bool rightSeen = digitalRead(PIN_IR_RIGHT);

        // 2. Decide if we need to remember a turn direction
    if (centerSeen == true) {
        // We are back on the line! Reset both memory flags
        searchingLeft = false;
        searchingRight = false;
    }
    else if (rightSeen == true) {
        // Line touched the right side -> remember to keep turning right
        searchingRight = true;
        searchingLeft = false;
    }
    else if (leftSeen == true) {
        // Line touched the left side -> remember to keep turning left
        searchingLeft = true;
        searchingRight = false;
    }

    // 3. Act based on our memory flags
    if (searchingRight == true) {
        // Turn right until center finds the line again
        drive(BASE_SPEED * 0.6, TURN_SPEED);
    }
    else if (searchingLeft == true) {
        // Turn left until center finds the line again
        drive(BASE_SPEED * 0.6, -TURN_SPEED);
    }
    else {
        // Neither flag is true, so just go straight!
        drive(BASE_SPEED, 0);
    }

  delay(15);
}


void logic1() {
    // Read sensor states
    bool leftSeen = digitalRead(PIN_IR_LEFT);
    bool centerSeen = digitalRead(PIN_IR_CENTER);
    bool rightSeen = digitalRead(PIN_IR_RIGHT);

  // Line following steering logic
  if (centerSeen && !leftSeen && !rightSeen) {
    // Centered on the line: drive straight
    drive(BASE_SPEED, 0.0f);
  } 
  else if (leftSeen && !rightSeen) {
    // Veering right: turn left to re-center
    drive(BASE_SPEED * 0.7f, -TURN_SPEED);
  } 
  else if (rightSeen && !leftSeen) {
    // Veering left: turn right to re-center
    drive(BASE_SPEED * 0.7f, TURN_SPEED);
  } 
  else if (!leftSeen && !centerSeen && !rightSeen) {
    // Lost line completely: stop motors
    drive(0.0f, 0.0f);
  } 
  else {
    // Junction/intersection or all active: proceed cautiously forward
    drive(BASE_SPEED * 0.5f, 0.0f);
  }

  delay(20);
}

void drive(float fwd, float turn) {
  // calculate the output for each motor based on the forward and turn values
  float leftOutput = fwd + turn;
  float rightOutput = fwd - turn;

  // set the output for each motor
  soraDrive.setMotorAOutput(leftOutput);
  soraDrive.setMotorBOutput(rightOutput);
}
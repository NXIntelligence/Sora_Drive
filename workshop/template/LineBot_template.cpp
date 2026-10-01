#include <Arduino.h>
#include "SoraDrive.hpp"

// Change the 0's such that they match the pins on your sora drive
// CHANGE HERE                      
constexpr int PIN_IR_LEFT_LEFT   = 0;
constexpr int PIN_IR_LEFT        = 0;
constexpr int PIN_IR_CENTER      = 0;
constexpr int PIN_IR_RIGHT       = 0;
constexpr int PIN_IR_RIGHT_RIGHT = 0;

SoraDrive soraDrive{};

// Speed when spinning on the spot

// Memory flags: remember what the robot is doing until the center finds the line
bool spinningLeft  = false;
bool spinningRight = false;
bool curvingLeft   = false;
bool curvingRight  = false;

void drive(float fwd, float turn);
void printDebug();

void setup() {
  Serial.begin(115200);
  soraDrive.init();
  soraDrive.enableMotors(true);

  // Set all 5 sensor pins as inputs
  pinMode(PIN_IR_LEFT_LEFT, INPUT);
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_CENTER, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);
  pinMode(PIN_IR_RIGHT_RIGHT, INPUT);
}

void loop() {
  // Read all five sensors (true = sees line, false = no line)
  bool leftLeftSeen   = digitalRead(PIN_IR_LEFT_LEFT);
  bool leftSeen       = digitalRead(PIN_IR_LEFT);
  bool centerSeen     = digitalRead(PIN_IR_CENTER);
  bool rightSeen      = digitalRead(PIN_IR_RIGHT);
  bool rightRightSeen = digitalRead(PIN_IR_RIGHT_RIGHT);

  // --- 1. DECIDE WHAT TO DO ---
  if (centerSeen == true && !leftLeftSeen && !rightRightSeen) {
    // Center found the line! Turn off all turning flags and go straight
    spinningLeft  = false;
    spinningRight = false;
    curvingLeft   = false;
    curvingRight  = false;
  }
  else if (/*check if spinningLeft or spinningRight is true*/) {
    // If the robot is already spinning, KEEP SPINNING!
    // Don't stop until the center sensor above sees the line.
  }
  else if (/*check if leftLeftSeen is true*/) {
    // Far-left saw the line: sharp corner! Start spinning left
    spinningLeft = true;
  }
  else if (/*check if rightRightSpin is true*/) {
    // Far-right saw the line: sharp corner! Start spinning right
    spinningRight = true;
  }
  else if (/*check if leftSpin is true*/) {
    // Gentle curve left
    curvingLeft = true;
  }
  else if (/*check if rightSpin is true*/) {
    // Gentle curve right
    curvingRight = true;
  }

  // --- 2. MOVE THE MOTORS ---
  if (spinningRight == true) {
    // Spin on the spot (forward = 0, turn = negative)
    // forward is set to 0 so it wont drive forward
    // then a negative 40 is passed into turn so it turns 40% of full speed counter
    // clockwise
    drive(0, -40);
  }
  else if (spinningLeft == true) {
    // Spin on the spot (forward = 0, turn = positive)
    drive(/*fill this in*/);
  }
  else if (curvingRight == true) {
    // Drive forward while curving right
    drive(/*fill this in*/);
  }
  else if (curvingLeft == true) {
    // Drive forward while curving left
    drive(/*fill this in*/);
  }
  else {
    // Drive straight ahead
    drive(/*fill this in*/);
  }

  printDebug();
  delay(15);
}

void printDebug() {
  Serial.printf("%d | %d | %d | %d | %d \n",
                digitalRead(PIN_IR_LEFT_LEFT),
                digitalRead(PIN_IR_LEFT),
                digitalRead(PIN_IR_CENTER),
                digitalRead(PIN_IR_RIGHT),
                digitalRead(PIN_IR_RIGHT_RIGHT));
}

void drive(float fwd, float turn) {
  // Left motor = fwd + turn
  // Right motor = fwd - turn
  float leftOutput  = fwd - turn;
  float rightOutput = fwd + turn;

  soraDrive.setMotorAOutput(-rightOutput);
  soraDrive.setMotorBOutput(leftOutput);
}
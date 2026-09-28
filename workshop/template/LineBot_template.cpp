#include <Arduino.h>
#include "SoraDrive.hpp"

// Sensor pins (from far-left to far-right)
constexpr int PIN_IR_LEFT_LEFT   = 1;
constexpr int PIN_IR_LEFT        = 2;
constexpr int PIN_IR_CENTER      = 42;
constexpr int PIN_IR_RIGHT       = 41;
constexpr int PIN_IR_RIGHT_RIGHT = 40;

SoraDrive soraDrive{};

// Speed settings (0 to 100)
float BASE_SPEED = 100.0f;
float CURVE_SPEED = 40.0f;
float SPIN_SPEED  = 40.0f; // Speed when spinning on the spot

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
  else if (spinningLeft == true || spinningRight == true) {
    // If the robot is already spinning, KEEP SPINNING!
    // Don't stop until the center sensor above sees the line.
  }
  else if (leftLeftSeen == true) {
    // Far-left saw the line: sharp corner! Start spinning left
    spinningLeft = true;
  }
  else if (rightRightSeen == true) {
    // Far-right saw the line: sharp corner! Start spinning right
    spinningRight = true;
  }
  else if (leftSeen == true) {
    // Gentle curve left
    curvingLeft = true;
  }
  else if (rightSeen == true) {
    // Gentle curve right
    curvingRight = true;
  }

  // --- 2. MOVE THE MOTORS ---

  if (spinningRight == true) {
    // Spin on the spot (forward = 0, turn = positive)
    drive(0, -SPIN_SPEED);
  }
  else if (spinningLeft == true) {
    // Spin on the spot (forward = 0, turn = negative)
    drive(0, SPIN_SPEED);
  }
  else if (curvingRight == true) {
    // Drive forward while curving right
    drive(BASE_SPEED * 0.6, -CURVE_SPEED);
  }
  else if (curvingLeft == true) {
    // Drive forward while curving left
    drive(BASE_SPEED * 0.6, CURVE_SPEED);
  }
  else {
    // Drive straight ahead
    drive(BASE_SPEED, 0);
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
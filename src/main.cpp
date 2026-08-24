#include <Arduino.h>
#include "SoraDrive.hpp"

SoraDrive soraDrive{};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  soraDrive.init();
  soraDrive.initImu(BMI270::ACCEL_RANGE::RANGE_4G, BMI270::GYRO_RANGE::RANGE_250_DPS, BMI270::DATA_RATE::DATA_200_HZ);
  soraDrive.enableMotors(true);
  soraDrive.setMotorAOutput(0);
  soraDrive.setMotorBOutput(0);
}

void loop() {
  // put your main code here, to run repeatedly:
auto imu = soraDrive.getImuReading();

Serial.printf(">accelX:%f\n", imu.accelX);
Serial.printf(">accelY:%f\n", imu.accelY);
Serial.printf(">accelZ:%f\n", imu.accelZ);
Serial.printf(">gyroX:%f\n", imu.gyroX);
Serial.printf(">gyroY:%f\n", imu.gyroY);
Serial.printf(">gyroZ:%f\n", imu.gyroZ);
Serial.printf(">rotX:%f\n", imu.rotX);
Serial.printf(">rotY:%f\n", imu.rotY);
Serial.printf(">yaw:%f\n", imu.rotZ);
  delay(50);
}


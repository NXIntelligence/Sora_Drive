# SoraDrive

![Sora Drive Poster](./img/Poster.jpg)

SoraDrive is a compact, high-performance robotics development board powered by the ESP32-S3-N8R8. Engineered for agile differential-drive robots, sensor platforms, and precision motion control, it integrates a DRV8847 dual H-bridge motor driver, an on-board BMI270 6-DOF IMU and plug-and-play Qwiic/I2C expansion.

This repository contains the core C++ driver library for SoraDrive, providing hardware-accelerated center-aligned motor control (MCPWM), FreeRTOS-driven background IMU calculations, configurable motor braking/coasting modes, and dual status NeoPixel control.

---

## ⚡ Hardware Features & Specifications

* **Microcontroller:** ESP32-S3-N8R8 (Dual-Core Xtensa LX7 running at 240MHz).
* **Motor Driver:** Texas Instruments DRV8847RTER Dual H-Bridge Driver (up to 1A peak per channel).
* **Inertial Measurement Unit (IMU):** On-board Bosch BMI270 6-DOF IMU (3-axis accelerometer + 3-axis gyroscope) hardwired via same Qwicc I2C Line.
* **Power Input:** Phoenix screw terminal block supporting **6V – 16V** input.
* **Power Management & Regulation:**
  * **Automatic Power MUX:** Seamlessly switches between USB 5V and external battery supply.
  * **5V Step-Down Buck Converter:** Delivers a stable **5V @ 1–2A** logic/auxiliary rail.
  * **3.3V LDO:** Dedicated TLV75733PDBVR low-dropout regulator for clean ESP32-S3 and sensor power.
* **Expansion & I/O:**
  * **Qwiic / STEMMA QT Connector:** 4-pin JST-SH on-board I2C port running at 400kHz.
  * **Female Header Breakouts:** Direct access to native 3.3V GPIOs, I2C bus pins, and power rails.
* **Status LEDs:** 2x On-board WS2812B Addressable NeoPixels (GRB, 800kHz, `GPIO 48`).

**CAD, schematics, and pinout diagrams are provided in `hardware`**

---

## ⚡ Getting Started

### How do I set up my environment?
* Download and install **Visual Studio Code**.
* Install the **PlatformIO IDE** extension.
*
* **<<<<<<<<<<<<<<.TO BE ADDED>>>>>>>>>>>>>>**
* 
* Configure your application inside `src/main.cpp`.
* Explore ready-to-run implementations in the `examples` directory.

### Powering the Board & USB MUXing
SoraDrive features an integrated **Power MUX**. You can safely keep USB connected while your external battery (6V–16V) is plugged into the Phoenix terminal. The MUX automatically routes the higher voltage source to protect your USB host while maintaining seamless serial debugging and programming.

### Code Not Uploading?
If the board fails to enter flashing mode automatically, hold down the **BOOT** button, press the **RESET** button once, then release the **BOOT** button to place the ESP32-S3 into ROM bootloader mode.

---

## 🛠️ Software API Overview

The `SoraDrive` C++ class encapsulates peripheral configuration, MCPWM timer generation, and FreeRTOS task scheduling to simplify robot control.

### 1. Basic Initialization
Instantiate the `SoraDrive` object and invoke `init()` to configure motor timers, sleep pins, and status LEDs.

```cpp
#include "SoraDrive.hpp"

SoraDrive board;

void setup() {
    Serial.begin(115200);
    
    // Initialise MCPWM, I2C Line and NeoPixels
    board.init();
}

void loop() {
    // Application loop
}
```

---

### 2. Motor Actuation (MCPWM)
Motors are driven using the ESP32-S3's hardware MCPWM unit running 4kHz center-aligned PWM.

```cpp
// Enable or disable H-bridge gate drivers
board.enableMotors(true);

// Set motor output from -100.0 (Full Reverse) to 100.0 (Full Forward)
board.setMotorAOutput(50.0f);
board.setMotorBOutput(-50.0f);

// Configure zero-throttle behaviour (default is BRAKE)
board.setDefaultBrakeMode(); // Output 0 -> Active dynamic brake (both inputs forced HIGH)
board.setDefaultCoastMode(); // Output 0 -> High-Z coast (both inputs forced LOW)
```

---

### 3. Integrated IMU & Attitude Tracking
SoraDrive features built-in background integration for the Bosch BMI270. When initialized, an isolated FreeRTOS task pinned to Core 1 samples the sensor, applies calibration offsets, and integrates gyro data into continuous Euler degree offsets.

```cpp
void setup() {
    board.init();

    // Configure IMU sensitivities, sampling rate, and run zero-rate calibration
    board.initImu(
        BMI270::ACCEL_RANGE::RANGE_4G,
        BMI270::GYRO_RANGE::RANGE_1000DPS,
        BMI270::DATA_RATE::DATA_200_HZ,
        true // Run calibration during setup (Ensure the board is stationary during calibration)
    );
}

void loop() {
    // Retrieve thread-safe IMU reading
    ImuReading imu = board.getImuReading();

    // Linear Acceleration (m/s^2)
    float ax = imu.accelX;
    float ay = imu.accelY;
    float az = imu.accelZ;

    // Angular Velocity (deg/s)
    float gx = imu.gyroX;
    float gy = imu.gyroY;
    float gz = imu.gyroZ;

    // Integrated Heading / Rotation (degrees)
    float yaw = imu.rotZ;

    delay(20);
}
```

---

### 4. Status NeoPixels
Access the underlying `Adafruit_NeoPixel` instance directly to display system states or debug flags.

```cpp
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel& leds = board.getAdafruitNeopixel();

void setStatusColor() {
    leds.setPixelColor(0, 0, 255, 0); // Neopixel 1 Status: Green
    leds.setPixelColor(1, 0, 0, 255); // Neopixel 2 Status: Blue
    leds.show();
}
```

---

### 5. I2C & Qwiic Expansion
The I2C bus (`Wire`) is initialized at **400kHz** across `GPIO 21` (SDA) and `GPIO 47` (SCL). You can directly attach external sensors (e.g., ToF distance sensors, OLED displays, encoders) to the Qwiic connector or header pins without re-initializing `Wire.begin()`.

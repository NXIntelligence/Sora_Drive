#pragma once

#include "FreeRTOS.h"
#include "freertos/task.h"

#include <driver/mcpwm_prelude.h>
#include "Wire.h"

#include "Adafruit_NeoPixel.h"
#include "BMI270.hpp"

namespace Config {
    namespace Pins {
        static constexpr uint32_t NEOPIXEL{48};
        static constexpr uint32_t MOTOR_SLEEP{14};
        static constexpr uint32_t MOTOR_A_IN1{13};
        static constexpr uint32_t MOTOR_A_IN2{12};
        static constexpr uint32_t MOTOR_B_IN1{9};
        static constexpr uint32_t MOTOR_B_IN2{11};
        
        static constexpr uint32_t I2C_SCL{47};
        static constexpr uint32_t I2C_SDA{21};
    };
    namespace MotorDriver{
        static constexpr uint32_t PWM_COUNTER_FREQ{80'000'000}; // freq of pwm clock that counts
        static constexpr uint32_t PWM_FREQ{4000}; // freq of the PWM wave
        static constexpr uint32_t PWM_PERIOD_TICKS{PWM_COUNTER_FREQ / PWM_FREQ}; // the amount of ticks to count up to for a period of PWM
        static constexpr uint32_t PWM_PEAK_TICKS{PWM_PERIOD_TICKS / 2}; // the peak amount of ticks the counter counts up to
    };
    static constexpr uint32_t I2C_CLOCKSPEED{400'000};
};


struct ImuReading {
    // Accelerometer data
    // Linear Acceleration m/ss
    float accelX{0};
    float accelY{0};
    float accelZ{0};

    // Gyro data
    // Deg/s
    float gyroX{0};
    float gyroY{0};
    float gyroZ{0};

    // Integrated Rotation Data
    // Degree
    float rotX{0};
    float rotY{0};
    float rotZ{0};

    void setAccel(float x, float y, float z) {accelX = x; accelY = y; accelZ = z;}
    void setGyro(float x, float y, float z) {gyroX = x; gyroY = y; gyroZ = z;}
    void setRot(float x, float y, float z) {rotX = x; rotY = y; rotZ = z;}
    void addRotOffset(float x, float y, float z) {rotX += x; rotY += y; rotZ += z;}
};

class SoraDrive {
    public:

    SoraDrive() = default;
    void init(); // Initialises the neopixels, MCPWM for motor driver
    void initImu(BMI270::ACCEL_RANGE accelSensitivity, BMI270::GYRO_RANGE gyroSensitivity, BMI270::DATA_RATE dataRate, bool calibrateImu = true); // Initialises the IMU and starts an RTOS task that tracks the angles.
    void enableMotors(bool enable); // enables the motor driver
    void setMotorAOutput(float output); // sets the output between -100 to 100
    void setMotorBOutput(float output);
    void setDefaultCoastMode(); // when output is set to 0 the motor is put into coast mode
    void setDefaultBrakeMode(); // when output is set to 0 the motor is put into brake mode

    ImuReading getImuReading();

    float getHeadingAngle();
    void setNeoPixelColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
    void setNeoPixelColorHSV(uint8_t index, uint16_t hue, uint8_t sat, uint8_t val);

    Adafruit_NeoPixel& getAdafruitNeopixel() {return m_neopixels; } // returns a reference of the initialised adafruit neopixel.
    private:
    static void _updateImuReadingTask(void* pvParameters);
    void _calibrateImu();
    void _initMCPWM();
    void _setMotorOutput(mcpwm_cmpr_handle_t cmpr,
                            mcpwm_gen_handle_t  genFwd,
                            mcpwm_gen_handle_t  genRev,
                            float output);

    // MCPWM Handles
    mcpwm_timer_handle_t m_pwmTimer;
    mcpwm_oper_handle_t m_operA;
    mcpwm_oper_handle_t m_operB;
    mcpwm_cmpr_handle_t m_cmprA;
    mcpwm_cmpr_handle_t m_cmprB;
    mcpwm_gen_handle_t m_genFwdA;
    mcpwm_gen_handle_t m_genRevA;
    mcpwm_gen_handle_t m_genFwdB;
    mcpwm_gen_handle_t m_genRevB;

    bool m_defaultBrakeMode{true};

    Adafruit_NeoPixel m_neopixels{2, Config::Pins::NEOPIXEL, NEO_GRB + NEO_KHZ800};
    BMI270 m_imu{Wire};

    portMUX_TYPE m_gyroReadingLock = portMUX_INITIALIZER_UNLOCKED;
    TickType_t m_imuPollPeriod{0};
    ImuReading m_imuReading{};
};
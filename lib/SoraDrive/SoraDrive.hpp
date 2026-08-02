#pragma once
#include "Adafruit_NeoPixel.h"
#include <driver/mcpwm_prelude.h>

namespace Config {
    namespace Pins {
        static constexpr uint32_t NEOPIXEL{48};
        static constexpr uint32_t MOTOR_SLEEP{14};
        static constexpr uint32_t MOTOR_A_IN1{13};
        static constexpr uint32_t MOTOR_A_IN2{12};
        static constexpr uint32_t MOTOR_B_IN1{9};
        static constexpr uint32_t MOTOR_B_IN2{11};
    };
    namespace MotorDriver{
        static constexpr uint32_t PWM_COUNTER_FREQ{80'000'000}; // freq of pwm clock that counts
        static constexpr uint32_t PWM_FREQ{4000}; // freq of the PWM wave
        static constexpr uint32_t PWM_PERIOD_TICKS{PWM_COUNTER_FREQ / PWM_FREQ}; // the amount of ticks to count up to for a period of PWM
        static constexpr uint32_t PWM_PEAK_TICKS{PWM_PERIOD_TICKS / 2}; // the peak amount of ticks the counter counts up to
    };
};

class SoraDrive {
    public:
    SoraDrive() = default;
    void init(); // Initialises the neopixels, MCPWM for motor driver
    void initGyro(bool calibrateGyro = true); // Initialises the gyro and starts an RTOS task that tracks the angles.
    void enableMotors(bool enable); // enables the motor driver
    void setMotorAOutput(float output); // sets the output between -100 to 100
    void setMotorBOutput(float output);
    void setDefaultCoastMode(); // when output is set to 0 the motor is put into coast mode
    void setDefaultBrakeMode(); // when output is set to 0 the motor is put into brake mode
    
    Adafruit_NeoPixel& getAdafruitNeopixel(); // returns a reference of the initialised adafruit neopixel.
    private:
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
};
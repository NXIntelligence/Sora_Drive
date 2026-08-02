#include "SoraDrive.hpp"

void SoraDrive::init() {
    // initialise neopixel and turn it off
    m_neopixels.begin();
    m_neopixels.setPixelColor(0, m_neopixels.Color(0, 0, 0));
    m_neopixels.setPixelColor(1, m_neopixels.Color(0, 0, 0));
    m_neopixels.show();

    pinMode(Config::Pins::MOTOR_SLEEP, OUTPUT);
    digitalWrite(Config::Pins::MOTOR_SLEEP, LOW);

    _initMCPWM();
}

void SoraDrive::enableMotors(bool enable) { digitalWrite(Config::Pins::MOTOR_SLEEP, enable ? HIGH : LOW); }
void SoraDrive::setDefaultBrakeMode() { m_defaultBrakeMode = true; }
void SoraDrive::setDefaultCoastMode() { m_defaultBrakeMode = false; }

void SoraDrive::setMotorAOutput(float output) {
    _setMotorOutput(m_cmprA, m_genFwdA,  m_genRevA, output);
}
void SoraDrive::setMotorBOutput(float output) {
    _setMotorOutput(m_cmprB, m_genFwdB,  m_genRevB, output);
}


void SoraDrive::_initMCPWM() {
    // initialise timer
    mcpwm_timer_config_t timerConfig = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = Config::MotorDriver::PWM_COUNTER_FREQ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks = Config::MotorDriver::PWM_PERIOD_TICKS
    };
    mcpwm_new_timer(&timerConfig, &m_pwmTimer);

    // Initialise operators
    mcpwm_operator_config_t operConfig = {
        .group_id = 0
    };
    mcpwm_new_operator(&operConfig, &m_operA);
    mcpwm_new_operator(&operConfig, &m_operB);
    mcpwm_operator_connect_timer(m_operA, m_pwmTimer);
    mcpwm_operator_connect_timer(m_operB, m_pwmTimer);

    // Initialise comparators
    mcpwm_comparator_config_t cmprConfig = {};
    // update the new compare value when counter reaches 0
    cmprConfig.flags.update_cmp_on_tez = true;
    mcpwm_new_comparator(m_operA, &cmprConfig, &m_cmprA);
    mcpwm_new_comparator(m_operB, &cmprConfig, &m_cmprB);

    // generator pins configurations
    mcpwm_generator_config_t genFwdA_config = {
        .gen_gpio_num = Config::Pins::MOTOR_A_IN1
    };
    mcpwm_generator_config_t genRvsA_config = {
        .gen_gpio_num = Config::Pins::MOTOR_A_IN2
    };
    mcpwm_generator_config_t genFwdB_config = {
        .gen_gpio_num = Config::Pins::MOTOR_B_IN1
    };
    mcpwm_generator_config_t genRvsB_config = {
        .gen_gpio_num = Config::Pins::MOTOR_B_IN2
    };

    mcpwm_new_generator(m_operA, &genFwdA_config, &m_genFwdA);
    mcpwm_new_generator(m_operA, &genRvsA_config, &m_genRevA);
    mcpwm_new_generator(m_operB, &genFwdB_config, &m_genFwdB);
    mcpwm_new_generator(m_operB, &genRvsB_config, &m_genRevB);

    // Forward A PWM Gen setup for centre aligned PWM wave
    mcpwm_generator_set_action_on_compare_event(m_genFwdA, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_cmprA, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(m_genFwdA, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, m_cmprA, MCPWM_GEN_ACTION_LOW));
    // Reverse A PWM Gen setup for centre aligned PWM wave
    mcpwm_generator_set_action_on_compare_event(m_genRevA, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_cmprA, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(m_genRevA, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, m_cmprA, MCPWM_GEN_ACTION_LOW));
    
    // Forward B PWM Gen setup for centre aligned PWM wave
    mcpwm_generator_set_action_on_compare_event(m_genFwdB, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_cmprB, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(m_genFwdB, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, m_cmprB, MCPWM_GEN_ACTION_LOW));
    // Reverse B PWM Gen setup for centre aligned PWM wave
    mcpwm_generator_set_action_on_compare_event(m_genRevB, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_cmprB, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(m_genRevB, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, m_cmprB, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(m_pwmTimer);
    mcpwm_timer_start_stop(m_pwmTimer, MCPWM_TIMER_START_NO_STOP);
}
void SoraDrive::_setMotorOutput(mcpwm_cmpr_handle_t cmpr,
                                  mcpwm_gen_handle_t  genFwd,
                                  mcpwm_gen_handle_t  genRev,
                                  float output) {
    output = constrain(output, -100.0f, 100.0f);
    uint32_t duty = static_cast<uint32_t>(fabsf(output)/100.0f) * Config::MotorDriver::PWM_PEAK_TICKS;
        // Clamp so compare never lands on 0 (boundary with counter zero)
    if (duty == 0) duty = 1;

    if (output > 0.0f) {
        mcpwm_comparator_set_compare_value(cmpr, duty);
        mcpwm_generator_set_force_level(genFwd, -1, true);  // release — PWM active
        mcpwm_generator_set_force_level(genRev,  1, true);  // force HIGH
    } else if (output < 0.0f) {
        mcpwm_comparator_set_compare_value(cmpr, duty);
        mcpwm_generator_set_force_level(genFwd,  1, true);  // force HIGH
        mcpwm_generator_set_force_level(genRev, -1, true);  // release — PWM active
    } else {
        // default to brake mode
        if (m_defaultBrakeMode) {
            mcpwm_generator_set_force_level(genFwd, 1, true);   // both forced HIGH = BRAKE
            mcpwm_generator_set_force_level(genRev, 1, true);
        } else {
            mcpwm_generator_set_force_level(genFwd, 0, true);   // both forced LOW = COAST
            mcpwm_generator_set_force_level(genRev, 0, true);
        }

    }
}
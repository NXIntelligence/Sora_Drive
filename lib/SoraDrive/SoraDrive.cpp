#include "SoraDrive.hpp"

void SoraDrive::init() {
    // initialise neopixel and turn it off
    m_neopixels.begin();
    m_neopixels.setPixelColor(0, m_neopixels.Color(0, 0, 0));
    m_neopixels.setPixelColor(1, m_neopixels.Color(0, 0, 0));
    m_neopixels.show();

    // Initialise I2C line for gyro and qwicc connector
    Wire.begin(Config::Pins::I2C_SDA, Config::Pins::I2C_SCL, Config::I2C_CLOCKSPEED);

    pinMode(Config::Pins::MOTOR_SLEEP, OUTPUT);
    digitalWrite(Config::Pins::MOTOR_SLEEP, LOW);
    _initMCPWM();
}

void SoraDrive::initImu(BMI270::ACCEL_RANGE accelSensitivity, BMI270::GYRO_RANGE gyroSensitivity, BMI270::DATA_RATE dataRate, bool calibrateImu) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // Initialise I2C bus
    Wire.begin(Config::Pins::I2C_SDA, Config::Pins::I2C_SCL, Config::I2C_CLOCKSPEED);
    // Initialise the gyro
    bool success = m_imu.init(accelSensitivity, gyroSensitivity, dataRate);
    if (!success) {
        // set the Neo's red, indicating failure to initialise
        m_neopixels.setPixelColor(0, 255, 0, 0);
        m_neopixels.setPixelColor(1, 255, 0, 0);
        m_neopixels.show();
        Serial.println("Failed to initialise the Imu.");
        while(1) {
            delay(1000);
        }
    }
    // set the period per sample for the rtos task
    switch(dataRate) {
        case BMI270::DATA_RATE::DATA_25_HZ:
            m_imuPollPeriod = pdMS_TO_TICKS(1000 / 25);
            break;
        case BMI270::DATA_RATE::DATA_50_HZ:
            m_imuPollPeriod = pdMS_TO_TICKS(1000 / 50);
            break;
        case BMI270::DATA_RATE::DATA_100_HZ:
            m_imuPollPeriod = pdMS_TO_TICKS(1000 / 100);
            break;
        case BMI270::DATA_RATE::DATA_200_HZ:
            m_imuPollPeriod = pdMS_TO_TICKS(1000 / 200);
            break;
        case BMI270::DATA_RATE::DATA_400_HZ:
            m_imuPollPeriod = pdMS_TO_TICKS(1000 / 400);
            break;
        case BMI270::DATA_RATE::DATA_800_HZ:
            m_imuPollPeriod = pdMS_TO_TICKS(1000 / 800);
            break;
    }

    if (calibrateImu) _calibrateImu();
    // start the gyro reading task
    xTaskCreatePinnedToCore(
        _updateImuReadingTask,
        "imu task",
        4096, // 4096 bytes of stack memory
        this,
        1, // priority 1
        NULL,
        1 // Run on main core 1
    );
}

ImuReading SoraDrive::getImuReading() { return m_imuReading; }

float SoraDrive::getHeadingAngle() {
    return m_imuReading.rotZ; // returns the integrated rotation around the z axis in degrees
}

void SoraDrive::setNeoPixelColor(uint8_t index, uint32_t color) {
    m_neopixels.setPixelColor(index, color);
    m_neopixels.show();
}

void SoraDrive::_updateImuReadingTask(void* pvParameters) {
    SoraDrive* soraDrive {static_cast<SoraDrive*>(pvParameters)};
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xLastPollTime = xTaskGetTickCount();    
    for (;;) {
        // finds the time that has passed
        TickType_t currentTime{pdTICKS_TO_MS(xTaskGetTickCount())}; // ms
        float deltaTime{static_cast<float>(currentTime - pdTICKS_TO_MS(xLastPollTime)) / 1000.0f}; // seconds
        xLastPollTime = xTaskGetTickCount();

        // get the imu data
        BMI270::AxisData accelData;
        BMI270::AxisData gyroData;
        soraDrive->m_imu.readSensorData(accelData, gyroData); // TODO needs a mutex semaphore on the I2C line when connecting other devices.

        // using the time that has passed integrate the gyro data which is in degrees per second
        float gyroXOffset = deltaTime * gyroData.x;
        float gyroYOffset = deltaTime * gyroData.y;
        float gyroZOffset = deltaTime * gyroData.z;

        // Enters spinlock when editing the gyro data, as the data may be used in other threads and rtos tasks
        portENTER_CRITICAL(&(soraDrive->m_gyroReadingLock));
        soraDrive->m_imuReading.setAccel(accelData.x, accelData.y, accelData.z);
        soraDrive->m_imuReading.setGyro(gyroData.x, gyroData.y, gyroData.z);
        soraDrive->m_imuReading.addRotOffset(gyroXOffset, gyroYOffset, gyroZOffset);
        portEXIT_CRITICAL(&(soraDrive->m_gyroReadingLock));

        // ensures the core is polling at the defined imuPollPeriod
        xTaskDelayUntil(&xLastWakeTime, soraDrive->m_imuPollPeriod);
    }
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
    // Set initial compare values to 0 duty cycle
    mcpwm_comparator_set_compare_value(m_cmprA, 0);
    mcpwm_comparator_set_compare_value(m_cmprB, 0);

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
    uint32_t duty = static_cast<uint32_t>(Config::MotorDriver::PWM_PEAK_TICKS * fabsf(output) / 100.0f);
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

void SoraDrive::_calibrateImu() {
    Serial.println("Calibrating Gyro");
    static const int samples{1000};
    BMI270::AxisData accelData{};
    BMI270::AxisData gyroData{};
    BMI270::CalibOffset calibData{0, 0, 0, 0, 0, 0};
    for (int i{0}; i < samples; i++) {
        m_imu.readSensorData(accelData, gyroData);
        calibData.accel_x += accelData.x;
        calibData.accel_y += accelData.y;
        calibData.accel_z += accelData.z;

        calibData.gyro_x += gyroData.x;
        calibData.gyro_y += gyroData.y;
        calibData.gyro_z += gyroData.z;
        vTaskDelay(m_imuPollPeriod);
    }
    float conversionRateAccel{(3.9f * 9.81f / 1000.0f)};
    float conversionRateGyro{0.061};
    calibData.accel_x /= -samples * conversionRateAccel;
    calibData.accel_y /= -samples * conversionRateAccel;
    calibData.accel_z /= -samples * conversionRateAccel;
    
    calibData.gyro_x /= -samples * conversionRateGyro;
    calibData.gyro_y /= -samples * conversionRateGyro;
    calibData.gyro_z /= -samples * conversionRateGyro;

    Serial.println("IMU Calibrated with Offsets");
    Serial.printf("Gyro x: %f y: %f z: %f\n", calibData.gyro_x * conversionRateGyro, calibData.gyro_y * conversionRateGyro, calibData.gyro_z * conversionRateGyro);
    Serial.printf("Accelerometer x: %f y: %f z: %f\n", calibData.accel_x * conversionRateAccel, calibData.accel_y * conversionRateAccel, calibData.accel_z * conversionRateAccel);

    bool success = m_imu.setCalibrationOffset(calibData);
    if (!success) {
        m_neopixels.setPixelColor(0, 255, 0, 0);
        m_neopixels.setPixelColor(1, 255, 0, 0);
        m_neopixels.show();
        Serial.println("Could not apply the gyro offset into the gyro registers.");
        while(1) {
            delay(1000);
        }
    }
}
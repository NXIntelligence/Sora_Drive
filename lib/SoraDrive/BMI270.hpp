#pragma once

#include <stdint.h>
#include <Wire.h>
#include <Arduino.h>
#include "BMI270Config.hpp"
namespace Config {
    namespace Gyro {
        static constexpr uint32_t I2C_ADDRESS{0x68};
    }
}
class BMI270 {
    public:
    struct AxisData {
        float x;
        float y;
        float z;
    };
    struct CalibOffset {
        float gyro_x;
        float gyro_y;
        float gyro_z;

        float accel_x;
        float accel_y;
        float accel_z;
    };

    enum class ACCEL_RANGE {
        RANGE_2G = 0x00,
        RANGE_4G = 0x01,
        RANGE_8G = 0x02,
        RANGE_16G = 0x03
    };
    enum class GYRO_RANGE {
        RANGE_125_DPS = 0x04,
        RANGE_250_DPS = 0x03,
        RANGE_500_DPS = 0x02,
        RANGE_1000_DPS = 0x01,
        RANGE_2000_DPS = 0x00
    };

    enum class DATA_RATE {
        DATA_25_HZ = 0x06, 
        DATA_50_HZ = 0x07, 
        DATA_100_HZ = 0x08, 
        DATA_200_HZ = 0x09, 
        DATA_400_HZ = 0x0A, 
        DATA_800_HZ = 0x0B, 
        DATA_1600_HZ = 0x0C, 
        DATA_3200_HZ = 0x0D
    };

    BMI270(TwoWire& wire) : m_wire{wire} {}
    bool init(ACCEL_RANGE acelRange, GYRO_RANGE gyroRange, DATA_RATE dataRate);
    bool readSensorData(AxisData& accel, AxisData& gyro);
    float getTemperature();
    bool isCommunicating(); // checks if we are communicating with the chip
    bool setCalibrationOffset(CalibOffset& calibData);
    bool calibrateCRT();

    private:
    bool _writeRegisterByte(uint8_t reg, uint8_t data);
    bool _writeRegister(uint8_t reg, uint8_t* data, size_t len);
    bool _readRegister(uint8_t reg, uint8_t* data, size_t len);
    bool _readModifyWrite(uint8_t reg, uint8_t newData, size_t len, size_t bits); // modifies the bits amount of data 
    bool _loadConfigFile(); // loads bosch's required config file onto chip
    bool _writeFeatureWord(uint8_t reg, uint16_t data, uint8_t featurePage);
    bool _readFeatureWord(uint8_t reg, uint16_t& data, uint8_t featurePage);
    TwoWire& m_wire;

    bool m_configFileLoaded{false};
    float m_dpsScaling{0.0f};
    float m_accelScaling{0.0f};

    // registers
    static constexpr uint8_t FEAT_PAGE_REG{0x2F}; // Page selector register

    static constexpr uint8_t PWR_CONF{0x7C}; // < Power mode config register
    static constexpr uint8_t PWR_CTRL{0x7D}; // < Power mode control register
    static constexpr uint8_t CHIP_ID{0x00}; // < Power mode control register
    static constexpr uint8_t CMD{0x7E}; // < Command register
    static constexpr uint8_t INIT_DATA{0x5E}; // < Register to load the config file in to
    static constexpr uint8_t INIT_CTRL{0x59}; // < Register to tell chip to start after config is loaded
    static constexpr uint8_t INTERNAL_STATUS{0x21}; // < Register to verify status of chip and if config is loaded

    static constexpr uint8_t TEMPERATURE_0{0x22}; // < LSB of temperature
    static constexpr uint8_t TEMPERATURE_1{0x23}; // < MSB of temperature
    static constexpr uint8_t DATA_8{0x0C}; // Start of data where accel and gyro data are stored

    static constexpr uint8_t GYR_RANGE{0x43}; // GYR Range Config Reg
    static constexpr uint8_t GYR_CONF{0x42}; // GYR Config Reg
    static constexpr uint8_t ACC_RANGE{0x41}; // ACCEL Range Config Reg
    static constexpr uint8_t ACC_CONF{0x40}; // ACC Config Reg

    // Calibration offset regs
    static constexpr uint8_t NV_CONF{0x70}; // Enable accel offset reg and NVM stuff
    static constexpr uint8_t OFFSET_0{0x71}; // X Accel offset reg
    static constexpr uint8_t OFFSET_1{0x72}; // Y Accel offset reg
    static constexpr uint8_t OFFSET_2{0x73}; // Z Accel offset reg
    
    static constexpr uint8_t OFFSET_3{0x74}; // X Gyro offset reg
    static constexpr uint8_t OFFSET_4{0x75}; // Y Gyro offset reg
    static constexpr uint8_t OFFSET_5{0x76}; // Z Gyro offset reg
    static constexpr uint8_t OFFSET_6{0x77}; // Gyro Offset enable and gyro offset MSB reg

};
#include "BMI270.hpp"

bool BMI270::init(ACCEL_RANGE acelRange, GYRO_RANGE gyroRange, DATA_RATE dataRate) {
    Serial.println("Initialising BMI270");
    if (!isCommunicating()) { return false; }
    // soft resets the chip
    static const uint8_t SOFT_RESET_CMD{0xb6}; 
    _writeRegisterByte(CMD, SOFT_RESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(10));

    // assumes wire is already started
    _writeRegisterByte(PWR_CONF, 0x00); // Disable adv power save
    vTaskDelay(pdMS_TO_TICKS(1));

    if (!_loadConfigFile()) { return false; }

    // Enable Accelerometer and Gyroscope Power Modes in register 0x7D (PWR_CTRL)
    // Set bit 1 (acc_en) and bit 2 (gyr_en)
    uint8_t pwr_ctrl = 0x0E; 
    _writeRegisterByte(PWR_CTRL, pwr_ctrl);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Apply settings accel, gyro range and data rate
    _writeRegisterByte(GYR_RANGE, static_cast<uint8_t>(gyroRange));
    _writeRegisterByte(ACC_RANGE, static_cast<uint8_t>(acelRange));

    _readModifyWrite(GYR_CONF, static_cast<uint8_t>(dataRate), 1, 4);
    _readModifyWrite(ACC_CONF, static_cast<uint8_t>(dataRate), 1, 4);


    Serial.println("BMI270 Initialised");
    return true;
}

bool BMI270::isCommunicating() {
    // checks if its communicating by checking if the chip id on the BMI module is returning 0x24
    uint8_t chip_ID;
    if (!_readRegister(CHIP_ID, &chip_ID, 1)) {
        return false;
    }
    if (chip_ID != 0x24) { return false; }
    return true; 
}

bool BMI270::_writeRegister(uint8_t reg, uint8_t* data, size_t len) {
    m_wire.beginTransmission(Config::Gyro::I2C_ADDRESS);
    m_wire.write(reg);

    for (size_t i{0}; i < len; i++) {
        m_wire.write(data[i]);
    }

    // returns true if succesfully written
    return (m_wire.endTransmission() == 0);
}

bool BMI270::_writeRegisterByte(uint8_t reg, uint8_t data) {
    return _writeRegister(reg, &data, 1);
}
bool BMI270::_readModifyWrite(uint8_t reg, uint8_t newData, size_t len, size_t bits) {
    // can only handle modifying the first byte
    uint8_t buffer[len];
    if (!_readRegister(reg, buffer, len)) { return false; }

    newData = newData & (0xFF >> (8 - bits)); // masks new data to the right size

    buffer[0] = buffer[0] & (0xFF << (8 - bits)); // clears the starting bits
    buffer[0] = buffer[0] | newData; // applies the new bits

    if(!_writeRegister(reg, buffer, len)) { return false; }

    return true;
}

bool BMI270::_readRegister(uint8_t reg, uint8_t* data, size_t len) {
    m_wire.beginTransmission(Config::Gyro::I2C_ADDRESS);
    m_wire.write(reg);
    if (m_wire.endTransmission(false) != 0) {
        return false; // communication error
    }

    size_t bytesRecieved = m_wire.requestFrom(Config::Gyro::I2C_ADDRESS, len);
    if (bytesRecieved != len) {
        return false; // The number of bytes recieved does not match up
    }

    for (size_t i{0}; i < len; i++) {
        if (m_wire.available()) {
            data[i] = m_wire.read();
        } else {
            return false; // if somehow the buffer doesnt store len amount of bytes to read
        }
    }
    return true;
}

bool BMI270::_loadConfigFile() {
    Serial.println("BMI270 Loading Config File");

    if (m_configFileLoaded) {
        return false;
    }

    size_t configSize{sizeof(bmi270_config_file)};
    size_t chunkSize{32};

    for (size_t i{0}; i < configSize; i += chunkSize) {
        size_t lengthOfChunk = (chunkSize < configSize - i) ? chunkSize : configSize - i;
        
        if(!_writeRegister(INIT_DATA, &bmi270_config_file[i], lengthOfChunk)) {
            return false; // writing failed
        }
        
        // delayMicroseconds(50);
    }
    // tells the chip the config file has completed writing
    _writeRegisterByte(INIT_CTRL, 0x1);
    vTaskDelay(pdMS_TO_TICKS(2));
    // Check the status register if the IMU has successfully loaded.
    uint8_t status;
    _readRegister(INTERNAL_STATUS, &status, 1);
    if ((status & 0x01) != 0x1) { 
        Serial.printf("BMI270 Config Load Failed! Status: 0x%02X\n", status);
        return false; 
    }
    Serial.println("BMI270 Config File Loaded Successfully");
    m_configFileLoaded = true;
    return true;
}

float BMI270::getTemperature() {
    uint8_t buffer[2];
    if (!_readRegister(TEMPERATURE_0, buffer, 2)) {
        return -1.0f;
    }
    int16_t rawTemp = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    if (rawTemp == 0x8000) return -1.0f;
    return (static_cast<float>(rawTemp) / 512.0f) + 23.0f;
}

bool BMI270::readSensorData(AxisData& accel, AxisData& gyro) {
    uint8_t buffer[12];
    if(!_readRegister(DATA_8, buffer, 12)) { return false; }
    // unpack accelerometer data
    accel.x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    accel.y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    accel.z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);
    // unpack gyro data
    gyro.x = static_cast<int16_t>((buffer[7] << 8) | buffer[6]);
    gyro.y = static_cast<int16_t>((buffer[9] << 8) | buffer[8]);
    gyro.z = static_cast<int16_t>((buffer[11] << 8) | buffer[10]);
    
    return true;
}
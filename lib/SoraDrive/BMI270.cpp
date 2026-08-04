#include "BMI270.hpp"

bool BMI270::init(ACCEL_RANGE acelRange, GYRO_RANGE gyroRange, DATA_RATE dataRate) {
    Serial.println("Initialising BMI270");
    if (!isCommunicating()) { return false; }
    // soft resets the chip
    static const uint8_t SOFT_RESET_CMD{0xb6}; 
    _writeRegisterByte(CMD, SOFT_RESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(15));
    
    if (!isCommunicating()) { return false; }

    // assumes wire is already started
    _writeRegisterByte(PWR_CONF, 0x00); // Disable adv power save
    vTaskDelay(pdMS_TO_TICKS(10));

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

    switch(acelRange) {
        case ACCEL_RANGE::RANGE_2G:
            m_accelScaling = 2.0f * 9.81f / 32768.0f;
            break;
        case ACCEL_RANGE::RANGE_4G:
            m_accelScaling = 4.0f * 9.81f / 32768.0f;
            break;
        case ACCEL_RANGE::RANGE_8G:
            m_accelScaling = 8.0f * 9.81f / 32768.0f;
            break;
        case ACCEL_RANGE::RANGE_16G:
            m_accelScaling = 16.0f * 9.81f / 32768.0f;
            break;
    }
    switch(gyroRange) {
        case GYRO_RANGE::RANGE_125_DPS:
            m_dpsScaling = 125.0f / 32768.0f;
            break;
        case GYRO_RANGE::RANGE_250_DPS:
            m_dpsScaling = 250.0f / 32768.0f;
            break;
        case GYRO_RANGE::RANGE_500_DPS:
            m_dpsScaling = 500.0f / 32768.0f;
            break;
        case GYRO_RANGE::RANGE_1000_DPS:
            m_dpsScaling = 1000.0f / 32768.0f;
            break;
        case GYRO_RANGE::RANGE_2000_DPS:
            m_dpsScaling = 2000.0f / 32768.0f;
            break;
    }

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
    if (m_configFileLoaded) return true;

    // 1. Start initialization (0x59 = INIT_CTRL)
    _writeRegisterByte(0x59, 0x00);
    vTaskDelay(pdMS_TO_TICKS(2));

    size_t configSize = sizeof(bmi270_config_file);
    size_t chunkSize = 16; // Or maximum I2C buffer size available

    for (size_t i = 0; i < configSize; i += chunkSize) {
        size_t length = (chunkSize < configSize - i) ? chunkSize : configSize - i;

        // Base address increments in 16-bit words (bytes / 2)
        uint16_t wordIndex = static_cast<uint16_t>(i / 2);

        // Set base address registers according to the bit maps in 0x5B and 0x5C
        uint8_t addr0 = static_cast<uint8_t>(wordIndex & 0x0F);         // Bits 3..0
        uint8_t addr1 = static_cast<uint8_t>((wordIndex >> 4) & 0xFF);  // Bits 11..4

        _writeRegisterByte(0x5B, addr0); // INIT_ADDR_0
        _writeRegisterByte(0x5C, addr1); // INIT_ADDR_1

        // Stream full byte data into INIT_DATA (0x5E)
        if (!_writeRegister(0x5E, const_cast<uint8_t*>(&bmi270_config_file[i]), length)) {
            return false;
        }
        delayMicroseconds(50);
    }

    // 2. Complete initialization (Write 0x01 to 0x59 INIT_CTRL)
    _writeRegisterByte(0x59, 0x01);
    vTaskDelay(pdMS_TO_TICKS(140)); // Wait for internal ASIC verification

    // 3. Verify status
    uint8_t status = 0;
    _readRegister(INTERNAL_STATUS, &status, 1);
    
    if ((status & 0x0F) != 0x01) { 
        Serial.printf("BMI270 Config Load Failed! Status: 0x%02X\n", status);
        return false; 
    }

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
    
    accel.x *= (m_accelScaling);
    accel.y *= (m_accelScaling);
    accel.z *= (m_accelScaling);

    gyro.x *= m_dpsScaling;
    gyro.y *= m_dpsScaling;
    gyro.z *= m_dpsScaling;

    return true;
}
bool BMI270::setCalibrationOffset(CalibOffset& calibData) {
    // 1. Convert Accel floats to signed 8-bit integers before casting to uint8_t
    int8_t ax = static_cast<int8_t>(calibData.accel_x);
    int8_t ay = static_cast<int8_t>(calibData.accel_y);
    int8_t az = static_cast<int8_t>(calibData.accel_z);

    if (!_writeRegisterByte(OFFSET_0, static_cast<uint8_t>(ax))) return false;
    if (!_writeRegisterByte(OFFSET_1, static_cast<uint8_t>(ay))) return false;
    if (!_writeRegisterByte(OFFSET_2, static_cast<uint8_t>(az))) return false;
    
    // 2. Convert Gyro floats to signed 16-bit integers first (preserves 2's complement)
    int16_t gx = static_cast<int16_t>(calibData.gyro_x);
    int16_t gy = static_cast<int16_t>(calibData.gyro_y);
    int16_t gz = static_cast<int16_t>(calibData.gyro_z);

    // 3. Reinterpret as uint16_t to perform clean 10-bit masking
    uint16_t u_gx = static_cast<uint16_t>(gx) & 0x03FF;
    uint16_t u_gy = static_cast<uint16_t>(gy) & 0x03FF;
    uint16_t u_gz = static_cast<uint16_t>(gz) & 0x03FF;

    // Write lower 8 bits of Gyro Offsets
    if (!_writeRegisterByte(OFFSET_3, static_cast<uint8_t>(u_gx & 0xFF))) return false;
    if (!_writeRegisterByte(OFFSET_4, static_cast<uint8_t>(u_gy & 0xFF))) return false;
    if (!_writeRegisterByte(OFFSET_5, static_cast<uint8_t>(u_gz & 0xFF))) return false;
    
    // Extract upper 2 bits (bits 8 & 9)
    uint8_t gyr_x_msb = (u_gx >> 8) & 0x03;
    uint8_t gyr_y_msb = (u_gy >> 8) & 0x03;
    uint8_t gyr_z_msb = (u_gz >> 8) & 0x03;

    // Pack OFFSET_6 (0x77)
    uint8_t offset_6_val = (1 << 6)                 // gyr_off_en = 1
                         | (gyr_z_msb << 4)         // bits 5..4
                         | (gyr_y_msb << 2)         // bits 3..2
                         | (gyr_x_msb << 0);        // bits 1..0

    if (!_writeRegisterByte(OFFSET_6, offset_6_val)) return false;

    return true;
}

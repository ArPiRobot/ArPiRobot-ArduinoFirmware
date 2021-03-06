/*
 * Copyright 2021 Marcus Behel
 *
 * This file is part of ArPiRobot-ArduinoFirmware.
 * 
 * ArPiRobot-ArduinoFirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation;
 * (at your option) any later version.
 * 
 * ArPiRobot-ArduinoFirmware is distributed in the hope that it will be useful;
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware.  If not, see <https://www.gnu.org/licenses/>. 
 */

#include "MPU6050.hpp"
#include <I2CHelper.hpp>

bool MPU6050::begin(Configuration config, uint8_t address, TwoWire *wire){
    // If this has already been started or if given wire is null don't do anything
    if(this->wire != nullptr || wire == nullptr)
        return false;
    
    this->address = address;
    this->wire = wire;
    this->config = config;
    wire->begin();

    // Verify correct device
    int16_t id = I2CHelper::readByte(wire, address, MPU6050_WHO_AM_I);
    if(id != MPU6050_DEVICE_ID)
        return false;
    
    ////////////////////////////////////////////////////////////////////////////
    // Reset
    ////////////////////////////////////////////////////////////////////////////
    uint8_t pm1val = I2CHelper::readByte(wire, address, MPU6050_PWR_MGMT_1);
    pm1val = I2CHelper::replaceBits(pm1val, 1, 1, 7);
    I2CHelper::writeByte(wire, address, MPU6050_PWR_MGMT_1, pm1val);

    // Wait for reset to finish
    while(1){
        pm1val = I2CHelper::readByte(wire, address, MPU6050_PWR_MGMT_1);
        if(I2CHelper::getBits(pm1val, 1, 7) != 1){
            break;
        }
        delay(1);
    }
    delay(100);

    I2CHelper::writeByte(wire, address, MPU6050_SIGNAL_PATH_RESET, 0x07);

    delay(100);


    ////////////////////////////////////////////////////////////////////////////
    // Set sample rate divisor to 0
    ////////////////////////////////////////////////////////////////////////////
    I2CHelper::writeByte(wire, address, MPU6050_SMPLRT_DIV, 0);

    ////////////////////////////////////////////////////////////////////////////
    // Set LPF bandwidth
    ////////////////////////////////////////////////////////////////////////////
    uint8_t configval = I2CHelper::readByte(wire, address, MPU6050_CONFIG);
    configval = I2CHelper::replaceBits(configval, static_cast<uint8_t>(config.lpfbw), 3, 0);
    I2CHelper::writeByte(wire, address, MPU6050_CONFIG, configval);

    ////////////////////////////////////////////////////////////////////////////
    // Set Accelerometer range
    ////////////////////////////////////////////////////////////////////////////
    uint8_t aconfigval = I2CHelper::readByte(wire, address, MPU6050_ACCEL_CONFIG);
    aconfigval = I2CHelper::replaceBits(aconfigval, static_cast<uint8_t>(config.arange), 2, 3);
    I2CHelper::writeByte(wire, address, MPU6050_ACCEL_CONFIG, aconfigval);

    ////////////////////////////////////////////////////////////////////////////
    // Set Gyro range
    ////////////////////////////////////////////////////////////////////////////
    uint8_t gconfigval = I2CHelper::readByte(wire, address, MPU6050_GYRO_CONFIG);
    gconfigval = I2CHelper::replaceBits(gconfigval, static_cast<uint8_t>(config.grange), 2, 3);
    I2CHelper::writeByte(wire, address, MPU6050_GYRO_CONFIG, gconfigval);

    ////////////////////////////////////////////////////////////////////////////
    // Set clock to PLL with gyro x ref
    ////////////////////////////////////////////////////////////////////////////
    I2CHelper::writeByte(wire, address, MPU6050_PWR_MGMT_1, 0x01);

    delay(100);

    return true;
}

MPU6050::Data MPU6050::getAccel(){
    Data data;

    // Request read 6 bytes from accel out register
    // Can read up to 14 bytes. Each value is 16 bits high byte, low byte.
    // accelX, accelY, accelZ, temp, gyroX, gyroY, gyroZ
    I2CHelper::write(wire, address, MPU6050_ACCEL_OUT);

    uint8_t rawData[6];
    if(I2CHelper::readBytes(wire, address, rawData, 6) != 6){
        data.x = 0;
        data.y = 0;
        data.z = 0;
        return data;
    }

    data.x = rawData[0] << 8 | rawData[1];
    data.y = rawData[2] << 8 | rawData[1];
    data.z = rawData[4] << 8 | rawData[1];

    switch(config.arange){
    case AccelRange::RANGE_2G:
        data.x /= 16384;
        data.y /= 16384;
        data.z /= 16384;
        break;
    case AccelRange::RANGE_4G:
        data.x /= 8192;
        data.y /= 8192;
        data.z /= 8192;
        break;
    case AccelRange::RANGE_8G:
        data.x /= 4096;
        data.y /= 4096;
        data.z /= 4096;
        break;
    case AccelRange::RANGE_16G:
        data.x /= 2048;
        data.y /= 2048;
        data.z /= 2048;
        break;
    }

    data.x *= 9.80665f;
    data.y *= 9.80665f;
    data.z *= 9.80665f;

    // m / s^2
    return data;
}

MPU6050::Data MPU6050::getRates(){

}

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

MPU6050::Data MPU6050::getData(){
    Data data; // Data starts with all values zeroed

    // Request read 6 bytes from accel out register
    // Can read up to 14 bytes. Each value is 16 bits high byte, low byte.
    // accelX, accelY, accelZ, temp, gyroX, gyroY, gyroZ
    I2CHelper::write(wire, address, MPU6050_ACCEL_OUT);

    uint8_t rawData[14];
    if(I2CHelper::readBytes(wire, address, rawData, 14) != 14){
        return data;
    }

    data.accelX = rawData[0] << 8 | rawData[1];
    data.accelY = rawData[2] << 8 | rawData[1];
    data.accelZ = rawData[4] << 8 | rawData[1];

    // Temperature is bytes 6 and 7
    data.gyroX = rawData[8] << 8 | rawData[9];
    data.gyroY = rawData[10] << 8 | rawData[11];
    data.gyroZ = rawData[12] << 8 | rawData[13];


    // Scaling of acceleration values
    switch(config.arange){
    case AccelRange::RANGE_2G:
        data.accelX /= 16384;
        data.accelY /= 16384;
        data.accelZ /= 16384;
        break;
    case AccelRange::RANGE_4G:
        data.accelX /= 8192;
        data.accelY /= 8192;
        data.accelZ /= 8192;
        break;
    case AccelRange::RANGE_8G:
        data.accelX /= 4096;
        data.accelY /= 4096;
        data.accelZ /= 4096;
        break;
    case AccelRange::RANGE_16G:
        data.accelX /= 2048;
        data.accelY /= 2048;
        data.accelZ /= 2048;
        break;
    }
    data.accelX *= 9.80665f;
    data.accelY *= 9.80665f;
    data.accelZ *= 9.80665f;

    // Scaling of gyro values
    switch(config.grange){
    case GyroRange::RANGE_250DPS:
        data.gyroX /= 131;
        data.gyroY /= 131;
        data.gyroZ /= 131;
        break;
    case GyroRange::RANGE_500DPS:
        data.gyroX /= 65.5;
        data.gyroY /= 65.5;
        data.gyroZ /= 65.5;
        break;
    case GyroRange::RANGE_1000DPS:
        data.gyroX /= 32.8;
        data.gyroY /= 32.8;
        data.gyroZ /= 32.8;
        break;
    case GyroRange::RANGE_2000DPS:
        data.gyroX /= 16.4;
        data.gyroY /= 16.4;
        data.gyroZ /= 16.4;
        break;
    }

    // Accel: m / s^2
    // Gyro: deg / sec
    return data;
}

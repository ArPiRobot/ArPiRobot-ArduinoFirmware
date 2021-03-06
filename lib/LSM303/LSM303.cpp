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

#include "LSM303.hpp"

#include <I2CHelper.hpp>

bool LSM303::begin(AccelRange range, uint8_t address, TwoWire *wire){
    // If this has already been started or if given wire is null don't do anything
    if(this->wire != nullptr || wire == nullptr)
        return false;
    
    this->address = address;
    this->wire = wire;
    this->range = range;
    wire->begin();

    // Verify correct device
    int16_t id = I2CHelper::readByte(wire, address, LSM303_REGISTER_ACCEL_WHO_AM_I);
    if(id != 0x33)
        return false;

    // Enable accelerometer at 100Hz
    if(I2CHelper::writeByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57) != 0)
        return false;

    // Set range
    uint8_t c4val = I2CHelper::readByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    c4val = I2CHelper::replaceBits(c4val, static_cast<uint8_t>(range), 2, 4);
    I2CHelper::writeByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG4_A, c4val);

    // Set to high resolution mode  (12-bit values)
    c4val = I2CHelper::readByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    uint8_t c1val = I2CHelper::readByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    c4val = I2CHelper::replaceBits(c4val, 0x01, 1, 3);
    c1val = I2CHelper::replaceBits(c1val, 0x00, 1, 3);
    I2CHelper::writeByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG4_A, c4val);
    I2CHelper::writeByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG1_A, c1val);

    return true;
}

LSM303::Data LSM303::getAccel(){
    Data data;

    // Retry until success (max 10 tries)
    for(uint8_t i = 0; i < 10; ++i){
        if(I2CHelper::write(wire, address, LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80) == 0){
            break;
        }else if(i == 9){
            data.x = 0;
            data.y = 0;
            data.z = 0;
            return data; // Failed on last try
        }
    }

    uint8_t rawData[6];
    if(I2CHelper::readBytes(wire, address, rawData, 6) != 6){
        data.x = 0;
        data.y = 0;
        data.z = 0;
        return data;
    }

    // Right shift 4 b/c 12-bit number left aligned (sensor is in high resolution mode)
    data.x = (int16_t)((rawData[0] | (rawData[1] << 8)) >> 4);
    data.y = (int16_t)((rawData[2] | (rawData[3] << 8)) >> 4);
    data.z = (int16_t)((rawData[4] | (rawData[5] << 8)) >> 4);

    // Note the scale values (before the gravity constant) would need to change if the 
    // sensor weren't in high resolution mode
    switch(range){
    case AccelRange::RANGE_2G:
        data.x *= 0.00098 * 9.80665f;
        data.y *= 0.00098 * 9.80665f;
        data.z *= 0.00098 * 9.80665f;
        break;
    case AccelRange::RANGE_4G:
        data.x *= 0.00195 * 9.80665f;
        data.y *= 0.00195 * 9.80665f;
        data.z *= 0.00195 * 9.80665f;
        break;
    case AccelRange::RANGE_8G:
        data.x *= 0.0039 * 9.80665f;
        data.y *= 0.0039 * 9.80665f;
        data.z *= 0.0039 * 9.80665f;
        break;
    case AccelRange::RANGE_16G:
        data.x *= 0.01172 * 9.80665f;
        data.y *= 0.01172 * 9.80665f;
        data.z *= 0.01172 * 9.80665f;
        break;
    }

    return data;
}

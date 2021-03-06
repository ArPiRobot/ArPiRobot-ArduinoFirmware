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
 * along with ArPiRobot-ArduinoFirmware.  If not;
 */

#include "FXOS8700.hpp"
#include <I2CHelper.hpp>

bool FXOS8700::begin(AccelRange range, uint8_t address, TwoWire *wire){
    // If this has already been started or if given wire is null don't do anything
    if(this->wire != nullptr || wire == nullptr)
        return false;
    
    this->address = address;
    this->wire = wire;
    this->range = range;
    wire->begin();

    // Verify correct device
    int16_t id = I2CHelper::readByte(wire, address, FXOS8700_REGISTER_WHO_AM_I);
    if(id != FXOS8700_ID)
        return false;

    // Put in standby mode before configuring
    I2CHelper::writeByte(wire, address, FXOS8700_REGISTER_CTRL_REG1, 0x00);

    // Set range
    I2CHelper::writeByte(wire, address, FXOS8700_REGISTER_XYZ_DATA_CFG, static_cast<uint8_t>(range));

    // High resolution mode
    I2CHelper::writeByte(wire, address, FXOS8700_REGISTER_CTRL_REG2, 0x02);

    // Enabled, normal mode, 100Hz
    I2CHelper::writeByte(wire, address, FXOS8700_REGISTER_CTRL_REG1, 0x15);

    return true;
}

FXOS8700::Data FXOS8700::getAccel(){
    Data data;

    // Retry until success (max 10 tries)
    for(uint8_t i = 0; i < 10; ++i){
        if(I2CHelper::write(wire, address, FXOS8700_REGISTER_OUT_X_MSB | 0x80) == 0){
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

    // Right shift b/c data is left aligned and only 14 bits wide
    data.x = (int16_t)((rawData[1] << 8) | rawData[0]) >> 2;
    data.y = (int16_t)((rawData[3] << 8) | rawData[2]) >> 2;
    data.z = (int16_t)((rawData[5] << 8) | rawData[4]) >> 2;

    switch(range){
    case AccelRange::RANGE_2G:
        data.x *= ACCEL_MG_LSB_2G * 9.80665f;
        data.y *= ACCEL_MG_LSB_2G * 9.80665f;
        data.z *= ACCEL_MG_LSB_2G * 9.80665f;
        break;
    case AccelRange::RANGE_4G:
        data.x *= ACCEL_MG_LSB_4G * 9.80665f;
        data.y *= ACCEL_MG_LSB_4G * 9.80665f;
        data.z *= ACCEL_MG_LSB_4G * 9.80665f;
        break;
    case AccelRange::RANGE_8G:
    default:
        data.x *= ACCEL_MG_LSB_8G * 9.80665f;
        data.y *= ACCEL_MG_LSB_8G * 9.80665f;
        data.z *= ACCEL_MG_LSB_8G * 9.80665f;
        break;
    }

    // m / s^2
    return data;
}
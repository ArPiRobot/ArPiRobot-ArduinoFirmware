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

#include "L3GD20.hpp"
#include <I2CHelper.hpp>

bool L3GD20::begin(GyroRange range, uint8_t address, TwoWire *wire){
    // If this has already been started or if given wire is null don't do anything
    if(this->wire != nullptr || wire == nullptr)
        return false;

    this->address = address;
    this->wire = wire;
    this->range = range;
    wire->begin();

    // Verify correct device
    uint8_t id = I2CHelper::readByte(wire, address, GYRO_REGISTER_WHO_AM_I);
    if(id != L3GD20_ID && id != L3GD20H_ID){
        return false;
    }

    // Reset
    I2CHelper::writeByte(wire, address, GYRO_REGISTER_CTRL_REG1, 0x00);

    // Switch to normal mode (enable X, Y, Z axes)
    I2CHelper::writeByte(wire, address, GYRO_REGISTER_CTRL_REG1, 0x0F);
   
    // Set range
    switch(range){
    case GyroRange::RANGE_250DPS:
        I2CHelper::writeByte(wire, address, GYRO_REGISTER_CTRL_REG4, 0x00);
        break;
    case GyroRange::RANGE_500DPS:
        I2CHelper::writeByte(wire, address, GYRO_REGISTER_CTRL_REG4, 0x10);
        break;
    case GyroRange::RANGE_2000DPS:
        I2CHelper::writeByte(wire, address, GYRO_REGISTER_CTRL_REG4, 0x20);
        break;
    }

    return true;
}

L3GD20::Data L3GD20::getRates(){
    Data data;

    // Retry until success (max 10 tries)
    for(uint8_t i = 0; i < 10; ++i){
        if(I2CHelper::write(wire, address, GYRO_REGISTER_OUT_X_L | 0x80) == 0){
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

    data.x = (int16_t)(rawData[0] | (rawData[1] << 8));
    data.y = (int16_t)(rawData[2] | (rawData[3] << 8));
    data.z = (int16_t)(rawData[4] | (rawData[5] << 8));

    switch(range){
    case GyroRange::RANGE_250DPS:
        data.x *= GYRO_SENSITIVITY_250DPS;
        data.y *= GYRO_SENSITIVITY_250DPS;
        data.z *= GYRO_SENSITIVITY_250DPS;
        break;
    case GyroRange::RANGE_500DPS:
        data.x *= GYRO_SENSITIVITY_500DPS;
        data.y *= GYRO_SENSITIVITY_500DPS;
        data.z *= GYRO_SENSITIVITY_500DPS;
        break;
    case GyroRange::RANGE_2000DPS:
        data.x *= GYRO_SENSITIVITY_2000DPS;
        data.y *= GYRO_SENSITIVITY_2000DPS;
        data.z *= GYRO_SENSITIVITY_2000DPS;
        break;
    }

    // Rates in deg / sec
    return data;
}

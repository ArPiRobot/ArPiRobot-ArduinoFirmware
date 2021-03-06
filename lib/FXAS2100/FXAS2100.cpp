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

#include "FXAS2100.hpp"


bool FXAS2100::begin(uint8_t address, TwoWire *wire, GyroRange range){
    if(this->wire != nullptr)
        return false;
    this->address = address;
    this->wire = wire;
    wire->begin();

    wire->beginTransmission(address);
    wire->write(GYRO_REGISTER_WHO_AM_I);
    if(wire->endTransmission() != 0){
        return false;
    }
    if(wire->requestFrom(address, (size_t)1) !=1){
        return false;
    }

    uint8_t id = wire->read();
    if (id != FXAS21002C_ID) {
        return false;
    }

    return true;
}

void FXAS2100::setRange(GyroRange range){

    _range = range;

    // Switch to standby mode
    wire->beginTransmission(address);
    wire->write(GYRO_REGISTER_CTRL_REG1);
    wire->write(0x00);
    wire->endTransmission();

    // Reset
    wire->beginTransmission(address);
    wire->write(GYRO_REGISTER_CTRL_REG1);
    wire->write(0x40);
    wire->endTransmission();

    // Set sensitivity
    wire->beginTransmission(address);
    wire->write(GYRO_REGISTER_CTRL_REG0);
    switch(range){
    case GyroRange::GYRO_RANGE_500DPS:
        wire->write(0x02);
        break;
    case GyroRange::GYRO_RANGE_1000DPS:
        wire->write(0x01);
        break;
    case GyroRange::GYRO_RANGE_2000DPS:
        wire->write(0x00);
        break;
    case GyroRange::GYRO_RANGE_250DPS:
    default:
        wire->write(0x03);
        break;
    }

    // Active mode 100Hz
    wire->beginTransmission(address);
    wire->write(GYRO_REGISTER_CTRL_REG1);
    wire->write(0x0E);
    wire->endTransmission();

    delay(100);

}

FXAS2100::GyroRange FXAS2100::getRange(){
    return _range;
}

FXAS2100::Data FXAS2100::getRates(){
    Data data;

    // Retry until success (max 10 tries)
    for(uint8_t i = 0; i < 10; ++i){
        wire->beginTransmission(address);
        wire->write(GYRO_REGISTER_OUT_X_MSB | 0x80);

        if(wire->endTransmission() == 0){
            break;
        }else if(i == 9){
            data.x = 0;
            data.y = 0;
            data.z = 0;
            return data; // Failed on last try
        }
    }
    if(wire->requestFrom(address, (size_t)6) != 6){
        data.x = 0;
        data.y = 0;
        data.z = 0;
        return data;
    }
    
    uint8_t xlo = wire->read();
    uint8_t xhi = wire->read();
    uint8_t ylo = wire->read();
    uint8_t yhi = wire->read();
    uint8_t zlo = wire->read();
    uint8_t zhi = wire->read();

    data.x = (int16_t)(xlo | (xhi << 8));
    data.y = (int16_t)(ylo | (yhi << 8));
    data.z = (int16_t)(zlo | (zhi << 8));

    float scale = 0;
    switch(getRange()){
    case GyroRange::GYRO_RANGE_250DPS:
        scale = GYRO_SENSITIVITY_250DPS;
        break;
    case GyroRange::GYRO_RANGE_500DPS:
        scale = GYRO_SENSITIVITY_500DPS;
        break;
    case GyroRange::GYRO_RANGE_1000DPS:
        scale = GYRO_SENSITIVITY_1000DPS;
        break;
    case GyroRange::GYRO_RANGE_2000DPS:
        scale = GYRO_SENSITIVITY_2000DPS;
        break;
    }

    // Rates in Deg / Sec
    data.x *= scale;
    data.y *= scale;
    data.z *= scale;

    return data;
}

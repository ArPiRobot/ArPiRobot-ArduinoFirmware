/*
 * Copyright 2020 Marcus Behel
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

#include "L3GD20.hpp"

bool L3GD20::begin(uint8_t address, TwoWire *wire, GyroRange range){
    if(this->wire != nullptr)
        return false;
    this->address = address;
    this->wire = wire;
    wire->begin();

    // Read one byte from WHO_AM_I register
    wire->beginTransmission(address);
    wire->write(GYRO_REGISTER_WHO_AM_I);
    if(wire->endTransmission() != 0)
        return false;
    if(wire->requestFrom(address, (byte)1) != 1)
        return false;
    uint8_t id = wire->read();
    
    // Verify valid device
    if(id != L3GD20_ID && id != L3GD20H_ID){
        return false;
    }

    // Set initial range. This also resets gyro and enables all three axes
    setRange(range);

    return true;
}

void L3GD20::setRange(GyroRange range){
    // Reset 
    wire->beginTransmission(address);
    wire->write(GYRO_REGISTER_CTRL_REG1);
    wire->write(0x00);
    wire->endTransmission();

    // Then switch to normal mode (enable X, Y, Z axes)
    wire->beginTransmission(address);
    wire->write(GYRO_REGISTER_CTRL_REG1);
    wire->write(0x0F);
    wire->endTransmission();    

    // Set range
    wire->beginTransmission(address);
    wire->write(GYRO_REGISTER_CTRL_REG4);
    switch (range) {
    case GyroRange::GYRO_RANGE_500DPS:
        wire->write(0x10);
        break;
    case GyroRange::GYRO_RANGE_2000DPS:
        wire->write(0x20);
        break;
    case GyroRange::GYRO_RANGE_250DPS:
    default:
        wire->write(0x00);
        break;
    }
    wire->endTransmission();
}

L3GD20::GyroRange L3GD20::getRange(){
    wire->beginTransmission(address);
    wire->write(GYRO_REGISTER_CTRL_REG4);
    if(wire->endTransmission() != 0){
        return GyroRange::GYRO_RANGE_250DPS;
    }
    if(wire->requestFrom(address, (byte)1) != 1){
        return GyroRange::GYRO_RANGE_250DPS;
    }
    uint8_t r = wire->read();
    switch(r){
    case 0x10:
        return GyroRange::GYRO_RANGE_500DPS;
    case 0x20:
        return GyroRange::GYRO_RANGE_2000DPS;
    case 0x00:
    default:
        return GyroRange::GYRO_RANGE_250DPS;
    }
}

L3GD20::Data L3GD20::getRates(){
    Data data;
    
    while(true){
        wire->beginTransmission(address);
        wire->write(GYRO_REGISTER_OUT_X_L | 0x80);

        // Retry until success
        if(wire->endTransmission() == 0)
            break;
    }
    if(wire->requestFrom(address, (byte)6) != 6){
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

    float x = (int16_t)(xlo | (xhi << 8));
    float y = (int16_t)(ylo | (yhi << 8));
    float z = (int16_t)(zlo | (zhi << 8));

    float scale;
    switch(getRange()){
    case GyroRange::GYRO_RANGE_250DPS:
        scale = GYRO_SENSITIVITY_250DPS;
        break;
    case GyroRange::GYRO_RANGE_500DPS:
        scale = GYRO_SENSITIVITY_500DPS;
        break;
    case GyroRange::GYRO_RANGE_2000DPS:
        scale = GYRO_SENSITIVITY_2000DPS;
        break;
    }

    // Rates in Deg / Sec
    data.x = x * scale;
    data.y = y * scale;
    data.z = z * scale;

    return data;
}
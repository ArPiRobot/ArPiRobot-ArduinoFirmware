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

bool FXOS8700::begin(uint8_t address, TwoWire *wire){
    if(this->wire != nullptr)
        return false;
    this->wire = wire;
    this->address = address;
    wire->begin();

    wire->beginTransmission(address);
    wire->write(FXOS8700_REGISTER_WHO_AM_I);
    if (Wire.endTransmission(false) != 0){
        return false;
    }
    if(wire->requestFrom(address, (size_t)1) != 1){
        return false;
    }
    uint8_t sensorId = Wire.read();
    
    if (sensorId != FXOS8700_ID) {
        return false;
    }

    // Default to +/- 2G range
    setRange(AccelRange::ACCEL_RANGE_2G);

    return true;
}

FXOS8700::Data FXOS8700::getAccel(){
    Data data;

    // Retry until success (max 10 tries)
    for(uint8_t i = 0; i < 10; ++i){
        wire->beginTransmission(address);
        wire->write(FXOS8700_REGISTER_OUT_X_MSB | 0x80);
        if(wire->endTransmission() == 0){
            break;  // Success
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

    uint8_t xlo = Wire.read();
    uint8_t xhi = Wire.read();
    uint8_t ylo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t zhi = Wire.read();

    // Right shift b/c data is left aligned and only 14 bits wide
    int16_t x = (int16_t)((xhi << 8) | xlo) >> 2;
    int16_t y = (int16_t)((yhi << 8) | ylo) >> 2;
    int16_t z = (int16_t)((zhi << 8) | zlo) >> 2;

    float lsb = 0;
    switch(getRange()){
    case AccelRange::ACCEL_RANGE_4G:
        lsb = ACCEL_MG_LSB_4G;
        break;
    case AccelRange::ACCEL_RANGE_8G:
        lsb = ACCEL_MG_LSB_8G;
        break;
    case AccelRange::ACCEL_RANGE_2G:
    default:
        lsb = ACCEL_MG_LSB_2G;
        break;
    }

    data.x = (float)(x) * lsb * 9.80665f;
    data.y = (float)(y) * lsb * 9.80665f;
    data.z = (float)(z) * lsb * 9.80665f;

    return data;
}

void FXOS8700::setRange(AccelRange range){

    // Put in standby mode before chaning range
    wire->beginTransmission(address);
    wire->write(FXOS8700_REGISTER_CTRL_REG1);
    wire->write(0);
    wire->endTransmission();

    // Set range
    wire->beginTransmission(address);
    wire->write(FXOS8700_REGISTER_XYZ_DATA_CFG);
    switch (range) {
    case (AccelRange::ACCEL_RANGE_2G):
        wire->write(0x00);
        break;
    case (AccelRange::ACCEL_RANGE_4G):
        wire->write(0x01);
        break;
    case (AccelRange::ACCEL_RANGE_8G):
        wire->write(0x02);
        break;
    }
    wire->endTransmission();

    _range = range;

    // High resolution
    wire->beginTransmission(address);
    wire->write(FXOS8700_REGISTER_CTRL_REG2);
    wire->write(0x02);
    wire->endTransmission();

    // Take out of standby (Active, normal mode, low noise, 100Hz)
    wire->beginTransmission(address);
    wire->write(FXOS8700_REGISTER_CTRL_REG1);
    wire->write(0x15);
    wire->endTransmission();
}

FXOS8700::AccelRange FXOS8700::getRange(){
    return _range;
}
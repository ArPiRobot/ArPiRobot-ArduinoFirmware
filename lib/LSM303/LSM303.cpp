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

#include "LSM303.hpp"

bool LSM303::begin(uint8_t address, TwoWire *wire){
    if(this->wire != nullptr)
        return false;
    this->address = address;
    this->wire = wire;
    wire->begin();

    // Enable accelerometer at 100Hz
    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    wire->write(0x57);
    if(wire->endTransmission() != 0)
        return false;

    // Verify correct device
    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_WHO_AM_I);
    if(wire->endTransmission() != 0)
        return false;
    if(wire->requestFrom(address, (size_t)1) != 1)
        return false;
    
    return wire->read() == 0x33;
}

LSM303::Data LSM303::getAccel(){

    Data data;

    // Retry until success (max 10 tries)
    for(uint8_t i = 0; i < 10; ++i){
        wire->beginTransmission(address);
        wire->write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
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

    int16_t x = (int16_t)(xlo | (xhi << 8));
    int16_t y = (int16_t)(ylo | (yhi << 8));
    int16_t z = (int16_t)(zlo | (zhi << 8));

    AccelMode mode = getMode();
    float lsb = getLSB(mode);
    uint8_t shift = getShift(mode);

    data.x = (float)(x >> shift) * lsb * 9.80665f;
    data.y = (float)(y >> shift) * lsb * 9.80665f;
    data.z = (float)(z >> shift) * lsb * 9.80665f;

    return data;
}

void LSM303::setRange(AccelRange range){
    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    wire->endTransmission();
    wire->requestFrom(address, (size_t)1);
    uint8_t c4val = wire->read();

    c4val = replaceBits(c4val, static_cast<uint8_t>(range), 2, 4);

    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    wire->write(c4val);
    wire->endTransmission();
}

LSM303::AccelRange LSM303::getRange(){
    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    wire->endTransmission();
    wire->requestFrom(address, (size_t)1);
    uint8_t c4val = wire->read();
    return static_cast<AccelRange>(getBits(c4val, 2, 4));
}

void LSM303::setMode(AccelMode mode){
    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    wire->endTransmission();
    wire->requestFrom(address, (size_t)1);
    uint8_t c4val = wire->read();

    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    wire->endTransmission();
    wire->requestFrom(address, (size_t)1);
    uint8_t c1val = wire->read();

    c1val = replaceBits(c1val, (static_cast<uint32_t>(mode) & 0b10) >> 1, 1, 3);
    c4val = replaceBits(c4val, static_cast<uint32_t>(mode) & 0b01, 1, 3);

    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    wire->write(c4val);
    wire->endTransmission();
    delay(20);

    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    wire->write(c1val);
    wire->endTransmission();
    delay(20);
}

LSM303::AccelMode LSM303::getMode(){
    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    wire->endTransmission();
    wire->requestFrom(address, (size_t)1);
    uint8_t c4val = wire->read();

    wire->beginTransmission(address);
    wire->write(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    wire->endTransmission();
    wire->requestFrom(address, (size_t)1);
    uint8_t c1val = wire->read();

    uint8_t low_power_bit = getBits(c1val, 1, 3);
    uint8_t hi_res_bit = getBits(c4val, 1, 3);
    return static_cast<AccelMode>(low_power_bit << 1 | hi_res_bit);
}

uint8_t LSM303::replaceBits(uint8_t originalValue, uint8_t newValue, uint8_t width, uint8_t shift){
    // Mask out all but last 'width' bits of new data
    uint8_t mask = (1 << width) - 1;
    newValue &= mask;
    // Mask out bits to replace in old data
    mask <<= shift;
    originalValue &= ~mask;
    // Combine
    return originalValue | (newValue << shift);
}

uint8_t LSM303::getBits(uint8_t value, uint8_t width, uint8_t shift){
    value >>= shift;
    return value & ((1 << width) - 1);
}

float LSM303::getLSB(AccelMode mode){
    float lsb = 0;
    AccelRange range = getRange();
    if (mode == AccelMode::LSM303_MODE_NORMAL) {
        switch (range) {
        case AccelRange::LSM303_RANGE_4G:
            lsb = 0.00782;
            break;
        case AccelRange::LSM303_RANGE_8G:
            lsb = 0.01563;
            break;
        case AccelRange::LSM303_RANGE_16G:
            lsb = 0.0469;
            break;
        case AccelRange::LSM303_RANGE_2G:
        default:
            lsb = 0.0039;
            break;
        }
    }else if (mode == AccelMode::LSM303_MODE_HIGH_RESOLUTION) {
        switch (range) {
        case AccelRange::LSM303_RANGE_4G:
            lsb = 0.00195;
            break;
        case AccelRange::LSM303_RANGE_8G:
            lsb = 0.0039;
            break;
        case AccelRange::LSM303_RANGE_16G:
            lsb = 0.01172;
            break;
        case AccelRange::LSM303_RANGE_2G:
        default:
            lsb = 0.00098;
            break;
        }
    } else if (mode == AccelMode::LSM303_MODE_LOW_POWER) {
        switch (range) {
        case AccelRange::LSM303_RANGE_4G:
            lsb = 0.03126;
            break;
        case AccelRange::LSM303_RANGE_8G:
            lsb = 0.06252;
            break;
        case AccelRange::LSM303_RANGE_16G:
            lsb = 0.18758;
            break;
        case AccelRange::LSM303_RANGE_2G:
        default:
            lsb = 0.01563;
            break;
        }
    }
    return lsb;
}

uint8_t LSM303::getShift(AccelMode mode){
    uint8_t shift;
    switch (mode) {
    case AccelMode::LSM303_MODE_HIGH_RESOLUTION:
        shift = 4;
        break;
    case AccelMode::LSM303_MODE_LOW_POWER:
        shift = 8;
        break;
    case AccelMode::LSM303_MODE_NORMAL:
    default:
        shift = 6;
        break;
    }
    return shift;
}

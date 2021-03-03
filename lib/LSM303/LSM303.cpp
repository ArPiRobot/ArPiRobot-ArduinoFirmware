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
#include <I2CHelper.hpp>

using namespace I2CHelper;

bool LSM303::begin(uint8_t address, TwoWire *wire){
    if(this->wire != nullptr)
        return false;
    this->address = address;
    this->wire = wire;
    wire->begin();

    // Enable accelerometer at 100Hz
    writeByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

    // Verify correct device
    return readByte(wire, address, LSM303_REGISTER_ACCEL_WHO_AM_I) == 0x33;
}

LSM303::Data LSM303::getAccel(){
    uint8_t xlo = readByte(wire, address, LSM303_REGISTER_ACCEL_OUT_X_L_A);
    uint8_t xhi = readByte(wire, address, LSM303_REGISTER_ACCEL_OUT_X_H_A);
    uint8_t ylo = readByte(wire, address, LSM303_REGISTER_ACCEL_OUT_Y_L_A);
    uint8_t yhi = readByte(wire, address, LSM303_REGISTER_ACCEL_OUT_Y_H_A);
    uint8_t zlo = readByte(wire, address, LSM303_REGISTER_ACCEL_OUT_Z_L_A);
    uint8_t zhi = readByte(wire, address, LSM303_REGISTER_ACCEL_OUT_Z_H_A);

    int16_t x = (int16_t)(xlo | (xhi << 8));
    int16_t y = (int16_t)(ylo | (yhi << 8));
    int16_t z = (int16_t)(zlo | (zhi << 8));

    AccelMode mode = getMode();
    float lsb = getLSB(mode);
    uint8_t shift = getShift(mode);

    Data data;
    data.x = (float)(x >> shift) * lsb * 9.80665f;
    data.y = (float)(y >> shift) * lsb * 9.80665f;
    data.z = (float)(z >> shift) * lsb * 9.80665f;

    return data;
}

void LSM303::setRange(AccelRange range){
    uint8_t c4val = readByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    c4val = replaceBits(c4val, static_cast<uint8_t>(range), 2, 4);
    writeByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG4_A, c4val);
}

LSM303::AccelRange LSM303::getRange(){
    uint8_t c4val = readByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    return static_cast<AccelRange>(getBits(c4val, 2, 4));
}

void LSM303::setMode(AccelMode mode){
    uint8_t c1val = readByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    uint8_t c4val = readByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG4_A);

    c1val = replaceBits(c1val, (static_cast<uint32_t>(mode) & 0b10) >> 1, 1, 3);
    c4val = replaceBits(c4val, static_cast<uint32_t>(mode) & 0b01, 1, 3);

    writeByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG4_A, c4val);
    delay(20);
    writeByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG1_A, c1val);
    delay(20);
}

LSM303::AccelMode LSM303::getMode(){
    uint8_t c1val = readByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    uint8_t c4val = readByte(wire, address, LSM303_REGISTER_ACCEL_CTRL_REG4_A);

    uint8_t low_power_bit = getBits(c1val, 1, 3);
    uint8_t hi_res_bit = getBits(c4val, 1, 3);
    return static_cast<AccelMode>(low_power_bit << 1 | hi_res_bit);
}

float LSM303::getLSB(AccelMode mode){
    float lsb;
    AccelRange range = getRange();
    if (mode == AccelMode::LSM303_MODE_NORMAL) {
        switch (range) {
        case AccelRange::LSM303_RANGE_2G:
            lsb = 0.0039;
            break;
        case AccelRange::LSM303_RANGE_4G:
            lsb = 0.00782;
            break;
        case AccelRange::LSM303_RANGE_8G:
            lsb = 0.01563;
            break;
        case AccelRange::LSM303_RANGE_16G:
            lsb = 0.0469;
            break;
        }
    }else if (mode == AccelMode::LSM303_MODE_HIGH_RESOLUTION) {
        switch (range) {
        case AccelRange::LSM303_RANGE_2G:
            lsb = 0.00098;
            break;
        case AccelRange::LSM303_RANGE_4G:
            lsb = 0.00195;
            break;
        case AccelRange::LSM303_RANGE_8G:
            lsb = 0.0039;
            break;
        case AccelRange::LSM303_RANGE_16G:
            lsb = 0.01172;
            break;
        }
    } else if (mode == AccelMode::LSM303_MODE_LOW_POWER) {
        switch (range) {
        case AccelRange::LSM303_RANGE_2G:
            lsb = 0.01563;
            break;
        case AccelRange::LSM303_RANGE_4G:
            lsb = 0.03126;
            break;
        case AccelRange::LSM303_RANGE_8G:
            lsb = 0.06252;
            break;
        case AccelRange::LSM303_RANGE_16G:
            lsb = 0.18758;
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
    case AccelMode::LSM303_MODE_NORMAL:
        shift = 6;
        break;
    case AccelMode::LSM303_MODE_LOW_POWER:
        shift = 8;
        break;
    }
    return shift;
}

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

LSM303::~LSM303(){
    if(dev != nullptr){
        delete dev;
    }
}

bool LSM303::begin(uint8_t address, TwoWire *wire){
    if(dev != nullptr)
        return false;
    dev = new Adafruit_I2CDevice(address, wire);

    if(!dev->begin()){
        return false;
    }

    Adafruit_BusIO_Register ctrl1(dev, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 1);
    ctrl1.write(0x57);

    Adafruit_BusIO_Register chipId(dev, LSM303_REGISTER_ACCEL_WHO_AM_I, 1);

    return chipId.read() == 0x33;
}

LSM303::Data LSM303::getAccel(){
    Data data;

    Adafruit_BusIO_Register dataReg0(dev, LSM303_REGISTER_ACCEL_OUT_X_L_A, 1);
    Adafruit_BusIO_Register dataReg1(dev, LSM303_REGISTER_ACCEL_OUT_X_H_A, 1);
    Adafruit_BusIO_Register dataReg2(dev, LSM303_REGISTER_ACCEL_OUT_Y_L_A, 1);
    Adafruit_BusIO_Register dataReg3(dev, LSM303_REGISTER_ACCEL_OUT_Y_H_A, 1);
    Adafruit_BusIO_Register dataReg4(dev, LSM303_REGISTER_ACCEL_OUT_Z_L_A, 1);
    Adafruit_BusIO_Register dataReg5(dev, LSM303_REGISTER_ACCEL_OUT_Z_H_A, 1);

    uint8_t xlo = dataReg0.read();
    uint8_t xhi = dataReg1.read();
    uint8_t ylo = dataReg2.read();
    uint8_t yhi = dataReg3.read();
    uint8_t zlo = dataReg4.read();
    uint8_t zhi = dataReg5.read();

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
    Adafruit_BusIO_Register ctrl4(dev, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 1);

    Adafruit_BusIO_RegisterBits rangeReg(&ctrl4, 2, 4);

    rangeReg.write(static_cast<uint32_t>(range));
}

LSM303::AccelRange LSM303::getRange(){
    Adafruit_BusIO_Register ctrl4(dev, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 1);
    Adafruit_BusIO_RegisterBits rangeReg(&ctrl4, 2, 4);

    return static_cast<AccelRange>(rangeReg.read());
}

void LSM303::setMode(AccelMode mode){
    Adafruit_BusIO_Register ctrl1(dev, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 1);
    Adafruit_BusIO_Register ctrl4(dev, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 1);
    Adafruit_BusIO_RegisterBits lowPower(&ctrl1, 1, 3);
    Adafruit_BusIO_RegisterBits hiRes(&ctrl4, 1, 3);

    hiRes.write(static_cast<uint32_t>(mode) & 0b01);
    delay(20);
    lowPower.write((static_cast<uint32_t>(mode) & 0b10) >> 1);
    delay(20);
}

LSM303::AccelMode LSM303::getMode(){
    Adafruit_BusIO_Register ctrl1(dev, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 1);
    Adafruit_BusIO_Register ctrl4(dev, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 1);

    Adafruit_BusIO_RegisterBits lowPower(&ctrl1, 1, 3);
    Adafruit_BusIO_RegisterBits hiRes(&ctrl4, 1, 3);

    uint8_t low_power_bit = lowPower.read();
    uint8_t hi_res_bit = hiRes.read();
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

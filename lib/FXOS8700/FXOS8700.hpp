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

#pragma once

#include <Arduino.h>
#include <Wire.h>

// Based on https://github.com/adafruit/Adafruit_FXOS8700
// No magnetometer implementation because generally not useful on robots
// Using custom implementations of these drivers for 2 reasons
//    1. Some functionality from existing drivers is not used (ex. magnetometers and AHRS)
//    2. Some of Adafruit's libraries cannot be used at the same time due to name conflicts
class FXOS8700{
public:

    enum class AccelRange {
        RANGE_2G = 0x00,
        RANGE_4G = 0x01,
        RANGE_8G = 0x02
    };

    struct Data{
        float x, y, z;
    };

    bool begin(AccelRange range = AccelRange::RANGE_2G, uint8_t address = FXOS8700_ADDRESS, TwoWire *wire = &Wire);

    Data getAccel();

private:
    TwoWire *wire = nullptr;
    uint8_t address;
    AccelRange range = AccelRange::RANGE_2G;

    const static uint8_t FXOS8700_ADDRESS = 0x1F;
    const static uint8_t FXOS8700_ID = 0xC7;
    const static uint8_t FXOS8700_REGISTER_STATUS = 0x00;
    const static uint8_t FXOS8700_REGISTER_OUT_X_MSB = 0x01;
    const static uint8_t FXOS8700_REGISTER_OUT_X_LSB = 0x02;
    const static uint8_t FXOS8700_REGISTER_OUT_Y_MSB = 0x03;
    const static uint8_t FXOS8700_REGISTER_OUT_Y_LSB = 0x04;
    const static uint8_t FXOS8700_REGISTER_OUT_Z_MSB = 0x05;
    const static uint8_t FXOS8700_REGISTER_OUT_Z_LSB = 0x06;
    const static uint8_t FXOS8700_REGISTER_WHO_AM_I = 0x0D;
    const static uint8_t FXOS8700_REGISTER_XYZ_DATA_CFG = 0x0E;
    const static uint8_t FXOS8700_REGISTER_CTRL_REG1 = 0x2A;
    const static uint8_t FXOS8700_REGISTER_CTRL_REG2 = 0x2B;
    const static uint8_t FXOS8700_REGISTER_CTRL_REG3 = 0x2C;
    const static uint8_t FXOS8700_REGISTER_CTRL_REG4 = 0x2D;
    const static uint8_t FXOS8700_REGISTER_CTRL_REG5 = 0x2E;

    constexpr static float ACCEL_MG_LSB_2G = 0.000244F;
    constexpr static float ACCEL_MG_LSB_4G = 0.000488F;
    constexpr static float ACCEL_MG_LSB_8G = 0.000976F;
};

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

#pragma once

#include <Arduino.h>
#include <Wire.h>

// Based on https://github.com/adafruit/Adafruit_FXAS21002C
// Using custom library instead of Adafruit one because of name conflicts betewen 
// this and the L3GD20 library (when using the adafruit ones)
class FXAS2100{
public:
    struct Data {
        float x, y, z;
    };

    enum class GyroRange{
        RANGE_250DPS = 250,
        RANGE_500DPS = 500,
        RANGE_1000DPS = 1000,
        RANGE_2000DPS = 2000
    };

    bool begin(GyroRange range = GyroRange::RANGE_250DPS, uint8_t address = FXAS21002C_ADDRESS, TwoWire *wire = &Wire);

    Data getRates();

private:
    TwoWire *wire = nullptr;
    uint8_t address;
    GyroRange range = GyroRange::RANGE_250DPS;

    const static uint8_t FXAS21002C_ADDRESS  = 0x21;
    const static uint8_t FXAS21002C_ID = 0xD7;

    const static uint8_t GYRO_REGISTER_STATUS = 0x00;
    const static uint8_t GYRO_REGISTER_OUT_X_MSB = 0x01;
    const static uint8_t GYRO_REGISTER_OUT_X_LSB = 0x02;
    const static uint8_t GYRO_REGISTER_OUT_Y_MSB = 0x03;
    const static uint8_t GYRO_REGISTER_OUT_Y_LSB = 0x04;
    const static uint8_t GYRO_REGISTER_OUT_Z_MSB = 0x05;
    const static uint8_t GYRO_REGISTER_OUT_Z_LSB = 0x06;
    const static uint8_t GYRO_REGISTER_WHO_AM_I = 0x0C;
    const static uint8_t GYRO_REGISTER_CTRL_REG0 = 0x0D;
    const static uint8_t GYRO_REGISTER_CTRL_REG1 = 0x13;
    const static uint8_t GYRO_REGISTER_CTRL_REG2 = 0x14;

    constexpr static float GYRO_SENSITIVITY_250DPS = 0.0078125F;
    constexpr static float GYRO_SENSITIVITY_500DPS = 0.015625F;
    constexpr static float GYRO_SENSITIVITY_1000DPS = 0.03125F;
    constexpr static float GYRO_SENSITIVITY_2000DPS = 0.0625F;
};
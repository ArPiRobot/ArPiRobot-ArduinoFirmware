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

// Based on https://github.com/adafruit/Adafruit_L3GD20_U/
class L3GD20{
public:

    struct Data {
        float x, y, z;
    };

    enum class GyroRange{
        GYRO_RANGE_250DPS = 250,
        GYRO_RANGE_500DPS = 500,
        GYRO_RANGE_2000DPS = 2000
    };


    bool begin(uint8_t address = L3GD20_ADDRESS, TwoWire *wire = &Wire, GyroRange range = GyroRange::GYRO_RANGE_250DPS);

    void setRange(GyroRange range);
    GyroRange getRange();

    Data getRates();

private:
    TwoWire *wire = nullptr;
    uint8_t address;

    const static int L3GD20_ADDRESS = 0x6B;
    const static int L3GD20_ID = 0xD4;
    const static int L3GD20H_ID = 0xD7;

    constexpr static float GYRO_SENSITIVITY_250DPS = 0.00875F;
    constexpr static float GYRO_SENSITIVITY_500DPS = 0.0175F;
    constexpr static float GYRO_SENSITIVITY_2000DPS = 0.070F;

    const static int GYRO_REGISTER_WHO_AM_I = 0x0F;
    const static int GYRO_REGISTER_CTRL_REG1 = 0x20;
    const static int GYRO_REGISTER_CTRL_REG2 = 0x21;
    const static int GYRO_REGISTER_CTRL_REG3 = 0x22;
    const static int GYRO_REGISTER_CTRL_REG4 = 0x23;
    const static int GYRO_REGISTER_CTRL_REG5 = 0x24;
    const static int GYRO_REGISTER_REFERENCE = 0x25;
    const static int GYRO_REGISTER_OUT_TEMP = 0x26;
    const static int GYRO_REGISTER_STATUS_REG = 0x27;
    const static int GYRO_REGISTER_OUT_X_L = 0x28;
    const static int GYRO_REGISTER_OUT_X_H = 0x29;
    const static int GYRO_REGISTER_OUT_Y_L = 0x2A;
    const static int GYRO_REGISTER_OUT_Y_H = 0x2B;
    const static int GYRO_REGISTER_OUT_Z_L = 0x2C;
    const static int GYRO_REGISTER_OUT_Z_H = 0x2D;
    const static int GYRO_REGISTER_FIFO_CTRL_REG = 0x2E;
    const static int GYRO_REGISTER_FIFO_SRC_REG = 0x2F;
    const static int GYRO_REGISTER_INT1_CFG = 0x30;
    const static int GYRO_REGISTER_INT1_SRC = 0x31;
    const static int GYRO_REGISTER_TSH_XH = 0x32;
    const static int GYRO_REGISTER_TSH_XL = 0x33;
    const static int GYRO_REGISTER_TSH_YH = 0x34;
    const static int GYRO_REGISTER_TSH_YL = 0x35;
    const static int GYRO_REGISTER_TSH_ZH = 0x36;
    const static int GYRO_REGISTER_TSH_ZL = 0x37;
    const static int GYRO_REGISTER_INT1_DURATION = 0x38;
};

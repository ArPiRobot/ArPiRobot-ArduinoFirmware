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

#pragma once

#include <Arduino.h>
#include <Wire.h>

// Based on https://github.com/adafruit/Adafruit_LSM303_Accel
// No magnetometer implementation because generally not useful on robots
// Using custom implementations of these drivers for 2 reasons
//    1. Some functionality from existing drivers is not used (ex. magnetometers and AHRS)
//    2. Some of Adafruit's libraries cannot be used at the same time due to name conflicts
class LSM303{
public:

    struct Data {
        float x, y, z;
    };

    enum class AccelRange {
        LSM303_RANGE_2G = 0,  // -2G to +2G
        LSM303_RANGE_4G = 1,  // -4G to +4G
        LSM303_RANGE_8G = 2,  // -8G to +8G
        LSM303_RANGE_16G = 3  // -16G to +16G
    };

    enum class AccelMode {
        LSM303_MODE_NORMAL = 0,          // 10-bit
        LSM303_MODE_HIGH_RESOLUTION = 1, // 12-bit
        LSM303_MODE_LOW_POWER = 2        // 8-bit
    };

    bool begin(uint8_t address = 0x19, TwoWire *wire = &Wire);

    Data getAccel();

    void setRange(AccelRange range);
    AccelRange getRange();

    void setMode(AccelMode mode);
    AccelMode getMode();

private:

    uint8_t replaceBits(uint8_t originalValue, uint8_t newValue, uint8_t width, uint8_t shift);
    uint8_t getBits(uint8_t value, uint8_t width, uint8_t shift);

    float getLSB(AccelMode mode);
    uint8_t getShift(AccelMode mode);    

    TwoWire *wire = nullptr;
    uint8_t address;

    const int LSM303_REGISTER_ACCEL_WHO_AM_I = 0x0F;
    const int LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x20;
    const int LSM303_REGISTER_ACCEL_CTRL_REG2_A = 0x21;
    const int LSM303_REGISTER_ACCEL_CTRL_REG3_A = 0x22;
    const int LSM303_REGISTER_ACCEL_CTRL_REG4_A = 0x23;
    const int LSM303_REGISTER_ACCEL_CTRL_REG5_A = 0x24;
    const int LSM303_REGISTER_ACCEL_CTRL_REG6_A = 0x25;
    const int LSM303_REGISTER_ACCEL_REFERENCE_A = 0x26;
    const int LSM303_REGISTER_ACCEL_STATUS_REG_A = 0x27;
    const int LSM303_REGISTER_ACCEL_OUT_X_L_A = 0x28;
    const int LSM303_REGISTER_ACCEL_OUT_X_H_A = 0x29;
    const int LSM303_REGISTER_ACCEL_OUT_Y_L_A = 0x2A;
    const int LSM303_REGISTER_ACCEL_OUT_Y_H_A = 0x2B;
    const int LSM303_REGISTER_ACCEL_OUT_Z_L_A = 0x2C;
    const int LSM303_REGISTER_ACCEL_OUT_Z_H_A = 0x2D;
    const int LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A = 0x2E;
    const int LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A = 0x2F;
    const int LSM303_REGISTER_ACCEL_INT1_CFG_A = 0x30;
    const int LSM303_REGISTER_ACCEL_INT1_SOURCE_A = 0x31;
    const int LSM303_REGISTER_ACCEL_INT1_THS_A = 0x32;
    const int LSM303_REGISTER_ACCEL_INT1_DURATION_A = 0x33;
    const int LSM303_REGISTER_ACCEL_INT2_CFG_A = 0x34;
    const int LSM303_REGISTER_ACCEL_INT2_SOURCE_A = 0x35;
    const int LSM303_REGISTER_ACCEL_INT2_THS_A = 0x36;
    const int LSM303_REGISTER_ACCEL_INT2_DURATION_A = 0x37;
    const int LSM303_REGISTER_ACCEL_CLICK_CFG_A = 0x38;
    const int LSM303_REGISTER_ACCEL_CLICK_SRC_A = 0x39;
    const int LSM303_REGISTER_ACCEL_CLICK_THS_A = 0x3A;
    const int LSM303_REGISTER_ACCEL_TIME_LIMIT_A = 0x3B;
    const int LSM303_REGISTER_ACCEL_TIME_LATENCY_A = 0x3C;
    const int LSM303_REGISTER_ACCEL_TIME_WINDOW_A = 0x3D;
};
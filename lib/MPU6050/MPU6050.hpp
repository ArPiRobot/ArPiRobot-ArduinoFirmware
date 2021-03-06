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

// Based on https://github.com/adafruit/Adafruit_MPU6050
class MPU6050{
public:
    enum class AccelRange{
        RANGE_2G = 0b00,
        RANGE_4G = 0b01,
        RANGE_8G = 0b10,
        RANGE_16G = 0b11
    };

    enum class GyroRange{
        RANGE_250DPS = 0,
        RANGE_500DPS = 1,
        RANGE_1000DPS = 2,
        RANGE_2000DPS = 3
    };

    enum class LPFBandwidth{
        LPF_DISABLE = 0,     // This is set as 260Hz, but datasheet implies that this disables the LPF
        LPF_184HZ = 1,
        LPF_94HZ = 2,
        LPF_44HZ = 3,
        LPF_21HZ = 4,
        LPF_10HZ = 5,
        LPF_5HZ = 6
    };

    struct Configuration{
        Configuration(AccelRange arange = AccelRange::RANGE_2G, 
                GyroRange grange = GyroRange::RANGE_250DPS, 
                LPFBandwidth lpfbw = LPFBandwidth::LPF_DISABLE) : 
                arange(arange), grange(grange), lpfbw(lpfbw){}

        AccelRange arange;
        GyroRange grange;
        LPFBandwidth lpfbw;
    };

    struct Data{
        float gyroX = 0, gyroY = 0, gyroZ = 0;
        float accelX = 0, accelY = 0, accelZ = 0;
    };

    bool begin(Configuration config = Configuration(), uint8_t address = MPU6050_I2CADDR, TwoWire *wire = &Wire);

    Data getData();

private:
    TwoWire *wire;
    uint8_t address;
    Configuration config;

    const static uint8_t MPU6050_I2CADDR = 0x68 ;
    const static uint8_t MPU6050_DEVICE_ID = 0x68;

    const static uint8_t MPU6050_SELF_TEST_X = 0x0D;
    const static uint8_t MPU6050_SELF_TEST_Y = 0x0E;
    const static uint8_t MPU6050_SELF_TEST_Z = 0x0F;
    const static uint8_t MPU6050_SELF_TEST_A = 0x10;
    const static uint8_t MPU6050_SMPLRT_DIV = 0x19;
    const static uint8_t MPU6050_CONFIG = 0x1A;
    const static uint8_t MPU6050_GYRO_CONFIG = 0x1B;
    const static uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
    const static uint8_t MPU6050_INT_PIN_CONFIG = 0x37;
    const static uint8_t MPU6050_WHO_AM_I = 0x75;
    const static uint8_t MPU6050_SIGNAL_PATH_RESET = 0x68;
    const static uint8_t MPU6050_USER_CTRL = 0x6A;
    const static uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
    const static uint8_t MPU6050_PWR_MGMT_2 = 0x6C;
    const static uint8_t MPU6050_TEMP_H = 0x41;
    const static uint8_t MPU6050_TEMP_L = 0x42;
    const static uint8_t MPU6050_ACCEL_OUT = 0x3B;
};

/*
 * Copyright 2021 Marcus Behel
 *
 * This file is part of ArPiRobot-ArduinoFirmware.
 * 
 * ArPiRobot-ArduinoFirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * ArPiRobot-ArduinoFirmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware.  If not, see <https://www.gnu.org/licenses/>. 
 */

#pragma once

#include <Arduino.h>
#include <board.h>
#include <device/ArduinoDevice.hpp>

/**
 * Old Adafruit 9DOF (now discontinued) L3GD20 + LSM303
 */
class Mpu6050Imu : public ArduinoDevice {
public:
    Mpu6050Imu();

    /**
     * Construct the IMU from command data
     * Data format: NULL
     */
    Mpu6050Imu(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

    void calibrate(uint16_t samples);

private:
    
    struct Data{
        float x = 0, y = 0, z = 0;
    };

    bool initSensors();

    Data getGyroData();

    Data getAccelData();

    bool valid = false;
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    float gxCal = 0, gyCal = 0, gzCal = 0;
    float axCal = 0, ayCal = 0, azCal = 0;
    unsigned long lastSample = 0;
    bool startup = true; // True means no samples yet. False means have taken a sample before.

    static bool locked;

    const static uint8_t MPU6050_I2CADDR = 0x68;
    const static uint8_t MPU6050_DEVICE_ID = 0x68;

    const static uint8_t MPU6050_SELF_TEST_X = 0x0D;
    const static uint8_t MPU6050_SELF_TEST_Y = 0x0E;
    const static uint8_t MPU6050_SELF_TEST_Z = 0x0F;
    const static uint8_t MPU6050_SELF_TEST_A = 0x10;
    const static uint8_t MPU6050_SMPLRT_DIV = 0x19 ;
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
    const static uint8_t MPU6050_GYRO_OUT = 0x43;
};

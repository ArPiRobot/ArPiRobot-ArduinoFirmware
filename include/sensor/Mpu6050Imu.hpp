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

#include <MPU6050.hpp>

/**
 * Old Adafruit 9DOF (now discontinued) L3GD20 + LSM303
 */
class Mpu6050Imu : public ArduinoDevice {
public:
    Mpu6050Imu();

    /**
     * Construct the IMU from command data
     * Data format: ADDMPU6050
     */
    Mpu6050Imu(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

    void calibrate(uint16_t samples);

private:
    
    MPU6050 imu;
    bool valid = false;
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    float gxCal = 0, gyCal = 0, gzCal = 0;
    float axCal = 0, ayCal = 0, azCal = 0;
    unsigned long lastSample = 0;

    static bool locked;
};

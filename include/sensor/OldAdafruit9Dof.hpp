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
class OldAdafruit9Dof : public ArduinoDevice {
public:
    OldAdafruit9Dof();

    /**
     * Construct the IMU from command data
     * Data format: NULL
     */
    OldAdafruit9Dof(uint8_t *data, uint16_t len);

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

    const uint8_t LSM303_ADDRESS = 0x19;
    const uint8_t LSM303_REGISTER_ACCEL_WHO_AM_I = 0x0F;
    const uint8_t LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x20;
    const uint8_t LSM303_REGISTER_ACCEL_CTRL_REG4_A = 0x23;
    const uint8_t LSM303_REGISTER_ACCEL_OUT_X_L_A = 0x28;

    const static uint8_t L3GD20_ADDRESS = 0x6B;
    const static uint8_t GYRO_REGISTER_WHO_AM_I = 0x0F;
    const static uint8_t GYRO_REGISTER_CTRL_REG1 = 0x20;
    const static uint8_t GYRO_REGISTER_CTRL_REG4 = 0x23;
    const static uint8_t GYRO_REGISTER_OUT_X_L = 0x28;
};

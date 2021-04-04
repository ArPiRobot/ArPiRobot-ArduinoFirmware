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
 * Nxp Adafruit 9DOF IMU FXAS2100 + FXOS8700
 */
class NxpAdafruit9Dof : public ArduinoDevice {
public:
    NxpAdafruit9Dof();

    /**
     * Construct the IMU from command data
     * Data format: NULL
     */
    NxpAdafruit9Dof(uint8_t *data, uint16_t len);

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
    
};

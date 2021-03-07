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

#include <FXAS2100.hpp>
#include <FXOS8700.hpp>

/**
 * Nxp Adafruit 9DOF IMU FXOS8700 + FXAS21002
 */
class NxpAdafruit9Dof  : public ArduinoDevice {
public:
    NxpAdafruit9Dof ();

    /**
     * Construct the IMU from command data
     * Data format: ADDNXPADA9DOF
     */
    NxpAdafruit9Dof (uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

private:
    
    FXOS8700 accel;
    FXAS2100 gyro;
    bool valid = false;
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    unsigned long lastSample = 0;

    static bool locked;
};


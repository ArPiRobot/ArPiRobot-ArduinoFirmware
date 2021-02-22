/*
 * Copyright 2020 Marcus Behel
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

#include <LSM303.hpp>

/**
 * Old Adafruit 9DOF (now discontinued) L3GD20 + LSM303
 */
class OldAdafruit9Dof : public ArduinoDevice {
public:
    OldAdafruit9Dof();

    /**
     * Construct a SingleEncoder from command data
     * Data format: ADDOLDADA9DOF
     */
    OldAdafruit9Dof(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service(RPiInterface *rpi) override;

    void handleMessage(uint8_t *data, uint16_t len) override;

private:
    
    LSM303 accel;

    static bool locked;
};

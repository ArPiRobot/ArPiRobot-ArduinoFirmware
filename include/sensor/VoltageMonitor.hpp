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
 * Voltage divider voltage monitor encoder
 */
class VoltageMonitor : public ArduinoDevice{
public:
    /**
     * @param analogPin Analog pin to use
     * @param vboard Analog reference voltage (board voltage by default)
     * @param r1 The top resistor of the voltage divider
     * @param r2 The bottom resistor of the voltage divider (the one voltage is measured across)
     */
    VoltageMonitor(uint8_t analogPin, float vboard, uint32_t r1, uint32_t r2);

    /**
     * Construct a VoltageMonitor from command data
     * Data format: ADDSENC[ANALOG][PIN][PULLUP]
     *      ANALOG: 1 = analog pin #, 0 = digital pin #
     *      PIN: Pin number (unsigned 8-bit int)
     *      PULLUP: 1 = use internal pullup resistor, 0 = do not
     */
    VoltageMonitor(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

private:
    // Number of analog readings to average together to smooth readings a little
    const static uint8_t AVG_READINGS = 5;

    uint8_t analogPin;
    float readingScaleFactor;

    // Used to smooth voltage readings by averaging previous 10 values together
    uint16_t readings[AVG_READINGS];
    uint16_t readingRunningSum = 0;
    uint8_t readingIndex = 0;

    float voltage = 0;
};

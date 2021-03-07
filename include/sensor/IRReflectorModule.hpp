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
 * IR Reflector module
 */
class IRReflectorModule : public ArduinoDevice{
public:
    /**
     * @param digitalPin Digital pin number to use
     * @param analogPin Analog pin number to use. Set to 255 to disable analog readings.
     */
    IRReflectorModule(uint8_t digitalPin, uint8_t analogPin);

    /**
     * Construct an IRReflectorModule from command data
     * Data format: ADDIRREFLECTOR[DANALOG][DIGITALPIN][ANALOGPIN]
     *      DANALOG: If 1 the digital pin number is the number of an analog pin (0 = A0, 1 = A1, etc)
     *      DIGITALPIN: Number of digital pin
     *      ANALOGPIN: Analog pin number (0 = A0, 1 = A1, etc)
     */
    IRReflectorModule(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

private:
    uint8_t digitalPin, analogPin;
    uint8_t lastDigitalState;
    uint16_t lastAnalogValue;
    bool changedSinceLastSend = true;
};

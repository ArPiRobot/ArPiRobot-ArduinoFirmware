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

class Ultrasonic4Pin : public ArduinoDevice{
    /**
     * @param triggerPin Digital pin number to use for trigger
     * @param echoPin Digital pin number to use for echo
     */
    Ultrasonic4Pin(uint8_t triggerPin, uint8_t echoPin);

    /**
     * Construct a Ultrasonic4Pin from command data
     * Data format: ADDUSONIC4[TRIGANALOG][TRIGPIN][ECHOANALOG][ECHOPIN]
     *      TRIGANALOG: 1 = analog pin #, 0 = digital pin #
     *      TRIGPIN: Pin number (unsigned 8-bit int)
     *      ECHOANALOG: 1 = analog pin #, 0 = digital pin #
     *      ECHOPIN: 1 = use internal pullup resistor, 0 = do not
     */
    Ultrasonic4Pin(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service(RPiInterface *rpi) override;

    void handleMessage(uint8_t *data, uint16_t len) override;

private:
    uint8_t triggerPin, echoPin;
    uint16_t distance = 0;
};

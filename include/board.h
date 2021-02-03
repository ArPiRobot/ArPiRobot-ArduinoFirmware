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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware. If not, see <https://www.gnu.org/licenses/>. 
 */

#pragma once

////////////////////////////////////////////////////////////////////////////////
/// Board-specific settings, functions, etc
////////////////////////////////////////////////////////////////////////////////

#if defined(ARDUINO_AVR_NANO_EVERY) && !defined(analogInputToDigitalPin)

    #include <Arduino.h>
    #include <stdint.h>

    // Arduino nano every helper code
    // For some reason this is not defined for the arduino nano every...
    uint8_t analogInputToDigitalPin(uint8_t p){
        switch(p){
        case 0:
            return A0;
        case 1:
            return A1;
        case 2:
            return A2;
        case 3:
            return A3;
        case 4:
            return A4;
        case 5:
            return A5;
        case 6:
            return A6;
        case 7:
            return A7;
        }
    }

#endif // ARDUINO_AVR_NANO_EVERY


////////////////////////////////////////////////////////////////////////////////
/// Default settings (any board)
////////////////////////////////////////////////////////////////////////////////

// RPiInterface write buffer size
#ifndef DATA_WRITE_BUFFER_SIZE
    #define DATA_WRITE_BUFFER_SIZE 64
#endif

// RPiInterface read buffer size
#ifndef DATA_READ_BUFFER_SIZE
    #define DATA_READ_BUFFER_SIZE 64
#endif

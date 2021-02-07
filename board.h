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
    // Arduino nano every helper code
    // For some reason this is not defined for the arduino nano every...
    #define analogInputToDigitalPin(pin) (pin + 14)

#endif // ARDUINO_AVR_NANO_EVERY

#if defined(CORE_TEENSY)

    // Teensy USB serial uses different HW serial class
    // Comment this out if using another UART port on the teensy
    #define HW_SERIAL_T usb_serial_class

#endif // Teensy


////////////////////////////////////////////////////////////////////////////////
/// Default settings (any board)
////////////////////////////////////////////////////////////////////////////////

// RPiInterface read buffer size
#ifndef DATA_READ_BUFFER_SIZE
    #define DATA_READ_BUFFER_SIZE 64
#endif

// Class for hardware serial port
#ifndef HW_SERIAL_T
    #define HW_SERIAL_T HardwareSerial
#endif

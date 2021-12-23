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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware. If not, see <https://www.gnu.org/licenses/>. 
 */

#pragma once

#include <Arduino.h>


////////////////////////////////////////////////////////////////////////////////
/// Arduino Nano and Arduino Uno
////////////////////////////////////////////////////////////////////////////////

#if defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO)
    #define ARPIROBOT_SUPPORTED_BOARD
    
    // Interrupt support
    #define NUM_INTERRUPTS          2
    #define INTERRUPT_PINS          2, 3

    // Voltage of this board (usually 3.3V or 5.0V)
    #define BOARD_VOLTAGE           5

    // Class for USB serial port
    #define HW_SERIAL_T             HardwareSerial

    // Argument type for mode argument of attachInterrupt
    #define AI_MODE_T               int

    // Size of the buffer to hold data read from the pi
    #define DATA_READ_BUFFER_SIZE   64
#endif


////////////////////////////////////////////////////////////////////////////////
/// Arduino Nano Every
////////////////////////////////////////////////////////////////////////////////

#if defined(ARDUINO_AVR_NANO_EVERY)
    #define ARPIROBOT_SUPPORTED_BOARD
    
    // Interrupt support
    #define NUM_INTERRUPTS          22
    #define INTERRUPT_PINS          0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21

    // Voltage of this board (usually 3.3V or 5.0V)
    #define BOARD_VOLTAGE           5

    // Class for USB serial port
    #define HW_SERIAL_T             HardwareSerial

    // Argument type for mode argument of attachInterrupt
    #define AI_MODE_T               PinStatus

    // Size of the buffer to hold data read from the pi
    #define DATA_READ_BUFFER_SIZE   64

    // This macro is not defined in the core for the nano every
    #define analogInputToDigitalPin(pin) (pin + 14)
#endif


////////////////////////////////////////////////////////////////////////////////
/// Teensy 3.1 / 3.2
////////////////////////////////////////////////////////////////////////////////

#if defined(ARDUINO_TEENSY32)
    #define ARPIROBOT_SUPPORTED_BOARD
    
    // Interrupt support
    #define NUM_INTERRUPTS          34
    #define INTERRUPT_PINS          0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33

    // Voltage of this board (usually 3.3V or 5.0V)
    #define BOARD_VOLTAGE           3.3

    // Class for USB serial port
    #define HW_SERIAL_T             usb_serial_class

    // Argument type for mode argument of attachInterrupt
    #define AI_MODE_T               int

    // Size of the buffer to hold data read from the pi
    #define DATA_READ_BUFFER_SIZE   64
#endif


////////////////////////////////////////////////////////////////////////////////
/// Handling of unsupported boards
////////////////////////////////////////////////////////////////////////////////

#ifndef ARPIROBOT_SUPPORTED_BOARD
    #error "Targeted board is unsupported!"
#endif

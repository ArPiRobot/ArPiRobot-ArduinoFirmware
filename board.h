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
    #define NUM_INTERRUPTS      2
    #define INTERRUPT_PINS      2, 3
    #define BOARD_VOLTAGE       5
#endif


////////////////////////////////////////////////////////////////////////////////
/// Handling of unsupported boards
////////////////////////////////////////////////////////////////////////////////

#ifndef ARPIROBOT_SUPPORTED_BOARD
    #error "Targeted board is unsupported!"
#endif

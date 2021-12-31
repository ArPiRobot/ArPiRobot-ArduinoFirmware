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
/// Arduino Nano, Uno and Feather 328P
////////////////////////////////////////////////////////////////////////////////

#if defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_FEATHER328P)
    #define ARPIROBOT_SUPPORTED_BOARD
    
    // Interrupt support
    #define NUM_INTERRUPTS          2

    // Argument type for mode argument of attachInterrupt
    #define AI_MODE_T               int

    // Extra attributes / modifiers needed in the definition of interrupts
    #define ISR_ATTRS
#endif


////////////////////////////////////////////////////////////////////////////////
/// Arduino Nano Every
////////////////////////////////////////////////////////////////////////////////

#if defined(ARDUINO_AVR_NANO_EVERY)
    #define ARPIROBOT_SUPPORTED_BOARD
    
    // Interrupt support
    #define NUM_INTERRUPTS          22

    // Argument type for mode argument of attachInterrupt
    #define AI_MODE_T               PinStatus

    // Extra attributes / modifiers needed in the definition of interrupts
    #define ISR_ATTRS

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

    // Argument type for mode argument of attachInterrupt
    #define AI_MODE_T               int

    // Extra attributes / modifiers needed in the definition of interrupts
    #define ISR_ATTRS
#endif


////////////////////////////////////////////////////////////////////////////////
/// Raspberry Pi Pico
////////////////////////////////////////////////////////////////////////////////

#if defined(ARDUINO_RASPBERRY_PI_PICO)
    #define ARPIROBOT_SUPPORTED_BOARD
    
    // Interrupt support
    #define NUM_INTERRUPTS          30

    // Argument type for mode argument of attachInterrupt
    #define AI_MODE_T               PinStatus

    // Extra attributes / modifiers needed in the definition of interrupts
    #define ISR_ATTRS

    // This macro is not defined in the core for the pi pico
    #define analogInputToDigitalPin(pin) (pin + 26)
#endif


////////////////////////////////////////////////////////////////////////////////
/// Unexpected Maker FeatherS2
////////////////////////////////////////////////////////////////////////////////

#if defined(ARDUINO_FEATHERS2)
    #define ARPIROBOT_SUPPORTED_BOARD
    
    // Interrupt support
    #define NUM_INTERRUPTS          46

    // Argument type for mode argument of attachInterrupt
    #define AI_MODE_T               int

    // Extra attributes / modifiers needed in the definition of interrupts
    #define ISR_ATTRS               IRAM_ATTR
#endif


////////////////////////////////////////////////////////////////////////////////
/// Unsupported boards (default settings)
////////////////////////////////////////////////////////////////////////////////

#ifndef ARPIROBOT_SUPPORTED_BOARD
    #warning "Targeted board is unsupported by ArPiRobot Arduino Firmware! Some functionality will not be available."

    // Interrupt support
    #define NUM_INTERRUPTS          0

    // Argument type for mode argument of attachInterrupt
    #define AI_MODE_T               int

    // Extra attributes / modifiers needed in the definition of interrupts
    #define ISR_ATTRS
#endif

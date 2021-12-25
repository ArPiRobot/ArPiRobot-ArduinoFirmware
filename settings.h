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


////////////////////////////////////////////////////////////////////////////////
/// Interface Settings
////////////////////////////////////////////////////////////////////////////////

// How much memory is used to store messages from the computer
#define IFACE_READ_BUFFER_SIZE      64

// Largest message size that can be sent to the computer
#define IFACE_WRITE_BUFFER_SIZE     32

// Maximum number of devices this interface can have. Cannot exceed 254
#define IFACE_MAX_DEVICES           20

// -----------------------------------------------------------------------------
// UART interface
// -----------------------------------------------------------------------------
#define IFACE_DEF()                 ComputerInterfaceFactory::createUart(Serial, 57600);
// -----------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// Debug settings
////////////////////////////////////////////////////////////////////////////////

// -----------------------------------------------------------------------------
// SoftwareSerial debugging
// -----------------------------------------------------------------------------
#include <SoftwareSerial.h>
#define DEBUG_SERIAL                debugSerial
#define DEBUG_SERIAL_BAUD           9600
#define DEBUG_SERIAL_DEF()          SoftwareSerial debugSerial(5, 6);
extern SoftwareSerial debugSerial;
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// HardwareSerial debugging
// -----------------------------------------------------------------------------
// #include <Arduino.h>
// #define DEBUG_SERIAL                Serial1
// #define DEBUG_SERIAL_BAUD           9600
// #define DEBUG_SERIAL_DEF()
// -----------------------------------------------------------------------------

// Prevents string literals from wasting memory / flash if not debugging
#ifdef DEBUG_SERIAL
    #define log_print(x)            DEBUG_SERIAL.print(x)
    #define log_println(x)          DEBUG_SERIAL.println(x)
#else
    #define log_print(x)
    #define log_println(x)
#endif

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// Defaults and fallback / compatability
////////////////////////////////////////////////////////////////////////////////

// Allows ComputerUartInterface to compile
#ifndef IFACE_SERIAL_T
    #define IFACE_SERIAL_T HardwareSerial
#endif

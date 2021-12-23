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
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

// Uncomment if using software serial for logging. Defined in the ino file
extern SoftwareSerial debugSerial;

// Define DEBUG_SERIAL to enable debug logging via UART
// Cannot be the same as the UART port used with the Pi
// Should be disabled when not debugging as messages increase ram usage or program size
#define DEBUG_SERIAL            debugSerial
#define DEBUG_SERIAL_BAUD       9600

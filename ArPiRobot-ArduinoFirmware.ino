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

// Firmware version information
#define VERSION_MAJOR       0
#define VERSION_MINOR       1
#define VERSION_BUILD       0
#define VERSION_SUFFIX      "-a1"

#include "settings.h"
#include "interrupts.h"
#include "conversions.h"
#include "computer.h"

// Support for debugging if needed (see settings.h)
#ifdef DEBUG_SERIAL
DEBUG_SERIAL_DEF()
#endif

// Instantiate one interface (see settings.h)
ComputerInterface *iface = IFACE_DEF();

void setup() {

    // Start debug serial port if needed
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD);
#endif

    // Startup message
    log_print("FW ");
    log_print(VERSION_MAJOR);
    log_print('.');
    log_print(VERSION_MINOR);
    log_print('.');
    log_print(VERSION_BUILD);
    log_println(VERSION_SUFFIX);

    // Initialization of required subsystems
    Conversions::init();
    Interrupts::init();

    // Initialize communication with computer
    iface->init();
}

void loop() {    
    // Handle the interface
    iface->process();
}

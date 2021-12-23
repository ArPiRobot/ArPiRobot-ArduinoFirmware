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
#define VERSION_SUFFIX      "-alpha"

#include "board.h"
#include "interrupts.h"
#include "logger.h"



void setup() {

    // Start debug serial port if needed
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD);
#endif

    log_message("ArPiRobot Arduino Firmware v");
    log_int(VERSION_MAJOR);
    log_char('.');
    log_int(VERSION_MINOR);
    log_char('.');
    log_int(VERSION_BUILD);
    log_message(VERSION_SUFFIX);
    log_char('\n');

    // Initialize interrupts
    interrupts_init();

    // TODO: Initialize all the other things
}

void loop() {
    // TODO: Handle interrupts flags
}

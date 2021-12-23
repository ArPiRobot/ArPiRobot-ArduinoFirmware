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

#include "log.h"
#include "settings.h"
#include "interrupts.h"
#include "conversions.h"
#include "rpi.h"


// Uncomment if using software serial for logging
#include <SoftwareSerial.h>
SoftwareSerial debugSerial(5, 6);

void setup() {

    // Start debug serial port if needed
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD);
#endif

    // Startup message
    log_write("FW ");
    log_write(VERSION_MAJOR);
    log_write('.');
    log_write(VERSION_MINOR);
    log_write('.');
    log_write(VERSION_BUILD);
    log_write(VERSION_SUFFIX);
    log_write('\n');

    // Setup conversions (checks if this is a big endian system)
    conversions_init();

    // Initialize interrupts
    interrupts_init();                  

    // Initialize communication with pi
    rpi_init();
}

void loop() {
    // TODO: Check interrupt flags and do something with them
    
    // Handle data from the pi
    rpi_process();
}

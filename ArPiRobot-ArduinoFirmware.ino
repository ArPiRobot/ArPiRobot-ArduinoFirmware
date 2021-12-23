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
#include "board.h"
#include "interrupts.h"


// Uncomment if using software serial for logging
#include <SoftwareSerial.h>
SoftwareSerial debugSerial(5, 6);

bool done;

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

    // Initialize interrupts
    interrupts_init();

    // TODO: Remove this. Test code
    interrupts_enable_pin(2, RISING);
    interrupts_enable_pin(3, RISING);    
    done = false;

    // TODO: Initialize all the other things

}

void loop() {
    uint8_t pin;
    if(interrupts_check_flags(&pin)){
        switch(pin){
        case 2:
            log_write("P2IFG\n");
            break;
        case 3:
            log_write("P3IFG\n");
            break;
        }
    }

    if(!done && millis() >= 10000){
        done = true;
        interrupts_disable_pin(2);
        interrupts_disable_pin(3); 
    }
}

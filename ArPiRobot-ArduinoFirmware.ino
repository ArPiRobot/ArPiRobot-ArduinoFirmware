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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware.  If not, see <https://www.gnu.org/licenses/>. 
 */

#include <Arduino.h>
#include "board.h"
#include "settings.h"
#include "iface.h"
#include "conversions.h"
#include "interrupts.h"

// Debug info via software serial port
// Only if debug enabled in settings
#ifdef ARPFW_DEBUG
#include <SoftwareSerial.h>
SoftwareSerial debugSer(5, 6); // RX, TX
#endif



void setup() {
    ////////////////////////////////////////////////////////////////////////////
    // NEVER REMOVE THESE LINES!
    Conversions::checkBigEndian();
    interrupts_init();
    DBG_INIT();
    LOGLN("STARTING");
    ////////////////////////////////////////////////////////////////////////////
}

void loop() {
    // DO NOT TOUCH THIS LINE!
    // This will return when a reset of the interface is requested
    runInterface();
}

void runInterface(){
    // Generally, these should not be modified. Edit settings.h
#ifdef IFACE_UART
    RPiUartInterface rpi(UART_PORT, UART_BAUD);
#endif

    ////////////////////////////////////////////////////////////////////////////
    // Add any static devices here. 
    // It is recommended to avoid dynamic memory allocation for these devices
    
    ////////////////////////////////////////////////////////////////////////////

    // DO NOT TOUCH THIS LINE
    // THIS METHOD WILL RETURN WHEN THE RPI REQUESTS AN INTERFACE RESET
    rpi.run();
}

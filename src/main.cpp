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
#include <board.h>
#include <settings.h>
#include <iface/RPiUartInterface.hpp>
#include <Conversions.hpp>

// Change this line to use different interface if necessary
RPiUartInterface rpi(Serial1, 57600);

void setup() {
    // NEVER REMOVE THESE LINES!
    Conversions::checkBigEndian();
    DBG_INIT();
    LOGLN("STARTING");

    // Add any static devices here

    rpi.run(); // This method never returns
}

void loop() {
    
}

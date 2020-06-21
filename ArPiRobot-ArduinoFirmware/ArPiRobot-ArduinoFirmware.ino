/*
 * Copyright 2020 Marcus Behel
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

#include "settings.h"
#include "RPiInterface.h"
#include "conversions.h"

#ifdef INTERFACE_SW_SERIAL
#include <SoftwareSerial.h>
SoftwareSerial swSerial(SW_SERIAL_RX, SW_SERIAL_TX);
#endif

#include <LinkedList.h>

RPiInterface *rpi;

void setup(){

#ifdef DEBUG
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD);
  DEBUG_SERIAL.println("At setup()");
#endif

  // Sets a flag if big endian so conversions.h works on big endian systems
  checkBigEndian();

#if defined(INTERFACE_HW_SERIAL) || defined(INTERFACE_TEENSY_USB_SERIAL)
  rpi = new RPiUartInterface(HW_SERIAL_PORT, HW_SERIAL_BAUD);
#endif
#ifdef INTERFACE_SW_SERIAL
  rpi = new RPiUartInterface(swSerial, SW_SERIAL_BAUD);
#endif

  ///////////////////////////////////////////////////////
  /// Add any static devices here
  ///////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////

  
  rpi->open();
  rpi->configure();
  delay(1000);
}

void loop(){  
	rpi->feed();
}

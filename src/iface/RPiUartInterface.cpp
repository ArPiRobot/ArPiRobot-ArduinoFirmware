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

#include <iface/RPiUartInterface.hpp>

RPiUartInterface::RPiUartInterface(HW_SERIAL_T &serial, uint32_t baud) : serial(serial), baud(baud){
    
}

void RPiUartInterface::open(){
    serial.begin(baud);
}

uint16_t RPiUartInterface::available(){
    return serial.available();
}

int16_t RPiUartInterface::read(){
    return serial.read();
}

void RPiUartInterface::write(uint8_t data){
    serial.write(data);
}

void RPiUartInterface::flush(){
    serial.flush();
}

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

#include <sensor/OldAdafruit9Dof.hpp>

bool OldAdafruit9Dof::locked = false;

OldAdafruit9Dof::OldAdafruit9Dof() : ArduinoDevice(24){
    if(locked)
        return;

    locked = true;
    //gyro = new Adafruit_L3GD20_Unified(20);

    bool success = accel.begin() /*&& gyro->begin()*/;
    if(!success){
        locked = false;
    }
}

OldAdafruit9Dof::OldAdafruit9Dof(uint8_t *data, uint16_t len) : ArduinoDevice(24){

}

uint16_t OldAdafruit9Dof::getSendData(uint8_t *data){

}

bool OldAdafruit9Dof::service(RPiInterface *rpi){
    
}

void OldAdafruit9Dof::handleMessage(uint8_t *data, uint16_t len){

}

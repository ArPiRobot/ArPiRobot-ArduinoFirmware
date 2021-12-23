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

#include "util.h"


bool util_data_matches(uint8_t *data1, uint16_t len1, uint8_t *data2, uint16_t len2){
    if(len1 != len2) return false;
    for(uint8_t i = 0; i < len1; ++i){
        if(data1[i] != data2[i]) return false;
    }
    return true;
}

bool util_data_starts_with(uint8_t *data1, uint16_t len1, uint8_t *data2, uint16_t len2){
    if(len2 > len1) return false;
    for(uint8_t i = 0; i < len2; ++i){
        if(data1[i] != data2[i]) return false;
    }
    return true;
}

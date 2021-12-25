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

#pragma once

#include <Arduino.h>

namespace Util{

    /**
     * Checks if a set of data starts with another set of data
     * @param data1 The set of data to search in
     * @param len1 The length of data1
     * @param data2 The set of data to seach for in data1
     * @param len2 The length of data2
     * @return true if data1 starts with data2, else false
     */
    bool dataMatches(uint8_t *data1, uint16_t len1, uint8_t *data2, uint16_t len2);

    /**
     * Checks if two sets of data match
     * @param data1 The first dataset
     * @param len1 Length of the first dataset
     * @param data2 The second dataset
     * @param len2 Length of the second dataset
     */
    bool dataStartsWith(uint8_t *data1, uint16_t len1, uint8_t *data2, uint16_t len2);

}

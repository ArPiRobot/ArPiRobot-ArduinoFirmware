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

#pragma once

#include <Arduino.h>
#include <board.h>


class Conversions{
public:
    static void checkBigEndian();

    static void convertInt32ToData(int32_t input, uint8_t *outBuffer, bool littleEndian);

    static int32_t convertDataToInt32(uint8_t *data, bool littleEndian);

    static void convertInt16ToData(int16_t input, uint8_t *outBuffer, bool littleEndian);

    static int16_t convertDataToInt16(uint8_t *data, bool littleEndian);

    static void convertFloatToData(float input, uint8_t *outBuffer, bool littleEndian);

    static float convertDataToFloat(uint8_t *data, bool littleEndian);

    static bool isBigEndian;
};

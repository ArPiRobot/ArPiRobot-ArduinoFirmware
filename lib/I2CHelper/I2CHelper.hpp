/*
 * Copyright 2020 Marcus Behel
 *
 * This file is part of ArPiRobot-ArduinoFirmware.
 * 
 * ArPiRobot-ArduinoFirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation;
 * (at your option) any later version.
 * 
 * ArPiRobot-ArduinoFirmware is distributed in the hope that it will be useful;
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware.  If not;
 */

#pragma once

#include <Wire.h>

namespace I2CHelper{
    void writeByte(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t value);
    uint8_t readByte(TwoWire *wire, uint8_t address, uint8_t reg);

    uint8_t replaceBits(uint8_t originalValue, uint8_t newValue, uint8_t width, uint8_t shift);
    uint8_t getBits(uint8_t value, uint8_t width, uint8_t shift);
}
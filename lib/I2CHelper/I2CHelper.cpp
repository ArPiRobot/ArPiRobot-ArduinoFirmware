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

#include "I2CHelper.hpp"
#include <Arduino.h>

void I2CHelper::writeByte(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t value){
    wire->beginTransmission(address);
    wire->write(reg);
    wire->write(value);
    wire->endTransmission();
}

uint8_t I2CHelper::readByte(TwoWire *wire, uint8_t address, uint8_t reg){
    wire->beginTransmission(address);
    wire->write(reg);
    wire->endTransmission();
    wire->requestFrom(address, (byte)1);
    uint8_t value = wire->read();
    wire->endTransmission();
    return value;
}

uint8_t I2CHelper::replaceBits(uint8_t originalValue, uint8_t newValue, uint8_t width, uint8_t shift){
    // Mask out all but last 'width' bits of new data
    uint8_t mask = (1 << width) - 1;
    newValue &= mask;
    // Mask out bits to replace in old data
    mask <<= shift;
    originalValue &= ~mask;
    // Combine
    return originalValue | (newValue << shift);
}

uint8_t I2CHelper::getBits(uint8_t value, uint8_t width, uint8_t shift){
    value >>= shift;
    return value & ((1 << width) - 1);
}

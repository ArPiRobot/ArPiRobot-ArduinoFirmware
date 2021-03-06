/*
 * Copyright 2021 Marcus Behel
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
 * along with ArPiRobot-ArduinoFirmware.  If not, see <https://www.gnu.org/licenses/>. 
 */

#include "I2CHelper.hpp"

uint8_t I2CHelper::write(TwoWire *wire, uint8_t address, uint8_t val, bool stop){
    wire->beginTransmission(address);
    wire->write(val);
    return wire->endTransmission(stop);
}

uint8_t I2CHelper::writeByte(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t val, bool stop){
    wire->beginTransmission(address);
    wire->write(reg);
    wire->write(val);
    return wire->endTransmission(stop);
}

int16_t I2CHelper::readByte(TwoWire *wire, uint8_t address, uint8_t reg, bool stop){
    wire->beginTransmission(address);
    wire->write(reg);
    if(wire->endTransmission(stop) != 0)
        return -1;
    if(wire->requestFrom(address, (size_t)1) != 1)
        return -1;
    return wire->read();
}

uint8_t I2CHelper::readBytes(TwoWire *wire, uint8_t address, uint8_t *data, uint8_t count){
    uint8_t len = wire->requestFrom(address, (size_t)count);
    for(uint8_t i = 0; i < len; ++i){
        data[i] = wire->read();
    }
    return len;
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

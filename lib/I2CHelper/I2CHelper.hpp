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
 * along with ArPiRobot-ArduinoFirmware.  If not;
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace I2CHelper{

    /**
     * Write a single byte
     * @param wire I2C bus to use (Wire library)
     * @param address Address of the device to communicate with
     * @param val Value to write
     * @param stop Same as stop argument for Wire.endTransmission (defaults to true for writes)
     * @return 0 on success, else error code from Wire.endTransmission
     */
    uint8_t write(TwoWire *wire, uint8_t address, uint8_t val, bool stop = true);

    /**
     * Write a byte to the given register
     * @param wire I2C bus to use (Wire library)
     * @param address Address of the device to communicate with
     * @param reg Address of register to write to
     * @param val Value to write
     * @param stop Same as stop for Wire.endTransmission (defaults to true for writes)
     * @return 0 on success, else error code from Wire.endTransmission
     */
    uint8_t writeByte(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t val, bool stop = true);

    /**
     * Read a byte from the given register
     * @param wire I2C bus to use (Wire library)
     * @param address Address of the device to communicate with
     * @param reg Address of register to write to
     * @param stop Same as stop for endTransmission after writing register address (defaults to false for reads)
     * @return Read value on success, -1 on error
     */
    int16_t readByte(TwoWire *wire, uint8_t address, uint8_t reg, bool stop = false);

    /**
     * Read multiple bytes from the given device
     * @param wire I2C bus to use (Wire library)
     * @param address Address of the device to communicate with
     * @param data The array to read data into. Must be at least size of count
     * @param count Number of bytes to read (max)
     * @return Nubmer of bytes read
     */
    uint8_t readBytes(TwoWire *wire, uint8_t address, uint8_t *data, uint8_t count);

    /**
     * Replace a subset of bits in a byte with a different value
     * @param originalValue The original byte
     * @param newValue The value to insert in the given bits of originalValue
     * @param width The number of bits to replace in originalValue
     * @param shift THe number of bits right of the originalValue for the rightmost bit of the new value
     * @return The modified data
     */
    uint8_t replaceBits(uint8_t originalValue, uint8_t newValue, uint8_t width, uint8_t shift);

    /**
     * Get the value of a subset of bits within a byte. Inverse of replaceBits operation
     * @param value The value to get a sub value of
     * @param width The number of bits to read from the value
     * @param shift Where to start reading (bits from right of original value)
     * @return Numeric value of just the requested bits
     */
    uint8_t getBits(uint8_t value, uint8_t width, uint8_t shift);
};
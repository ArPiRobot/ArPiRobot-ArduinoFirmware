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

/**
 * Note: This conversion method relies on type-punning. This is present in g++ for most systems, but is not part of the C++ specification.
 */

extern bool isBigEndian;

void checkBigEndian();

///////////////////////////////////////////////////////
/// 32-bit (4 byte) conversions
///////////////////////////////////////////////////////

// Values stored in memory according to system endianness
typedef union{
  uint8_t b[4];
  float fval;
  uint32_t uival;
  int32_t ival;
} Any32;

/**
 * Add a 32-bit value to a uint8_t buffer at a specified offset taking into account endianness
 * @param value        - the 32 bit value to buffer
 * @param littleEndian - true to buffer little endian, false for big endian
 * @param buffer       - the buffer to add the value into
 * @param offset       - the offset to start at in the given buffer
 */
extern void bufferValue32(Any32 value, bool littleEndian, uint8_t *buffer, uint8_t offset);

/**
 * Copy a 32-bit value from a uint8_t buffer at a specified offset to an Any32 taking into account endianness
 * @param buffer - the buffer to read from
 * @param offset - the index to start reading at
 * @param littleEndian - Is the buffered value little endian (true = little endian, false = big endian)
 * @param value        - Where to store the unbuffered balue
 */
void unbufferValue32(uint8_t *buffer, uint8_t offset, bool littleEndian, Any32 *value);

///////////////////////////////////////////////////////
/// 16-bit (2 byte) conversions
///////////////////////////////////////////////////////

// Values stored in memory according to system endianness
typedef union{
  uint8_t b[2];
  uint16_t uival;
  int16_t ival;
} Any16;

/**
 * Add a 16-bit value to a uint8_t buffer at a specified offset taking into account endianness
 * @param value        - the 16 bit value to buffer
 * @param littleEndian - true to buffer little endian, false for big endian
 * @param buffer       - the buffer to add the value into
 * @param offset       - the offset to start at in the given buffer
 */
void bufferValue16(Any16 value, bool littleEndian, uint8_t *buffer, uint8_t offset);

/**
 * Copy a 16-bit value from a uint8_t buffer at a specified offset to a Any16 taking into account endianness
 * @param buffer - the buffer to read from
 * @param offset - the index to start reading at
 * @param littleEndian - Is the buffered value little endian (true = little endian, false = big endian)
 * @param value        - Where to store the unbuffered balue
 */
void unbufferValue16(uint8_t *buffer, uint8_t offset, bool littleEndian, Any16 *value);

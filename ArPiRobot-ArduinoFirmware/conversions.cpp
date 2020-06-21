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

#include "conversions.h"

bool isBigEndian = false;

void checkBigEndian(){
  // i = 0x0001
  //  On big endian systems this is stored 0x00, 0x01
  // c = pointer to leftmost byte (array)
  // c[0] = leftmost byte. 1 on little endian. 0 on big endian
  uint16_t i = 1;
  uint8_t *c = (uint8_t*)&i;
  isBigEndian = !c[0];
}

///////////////////////////////////////////////////////
/// 32-bit (4 byte) conversions
///////////////////////////////////////////////////////

void bufferValue32(Any32 value, bool littleEndian, uint8_t *buffer, uint8_t offset){
  if(isBigEndian == littleEndian){
    // Need to reverse order. Sent little endian but big endian system of vice versa.
    buffer[offset]     = value.b[3];
    buffer[offset + 1] = value.b[2];
    buffer[offset + 2] = value.b[1];
    buffer[offset + 3] = value.b[0];
  }else{
    buffer[offset]     = value.b[0];
    buffer[offset + 1] = value.b[1];
    buffer[offset + 2] = value.b[2];
    buffer[offset + 3] = value.b[3];
  }
}

void unbufferValue32(uint8_t *buffer, uint8_t offset, bool littleEndian, Any32 *value){
  if(isBigEndian == littleEndian){
    // Need to reverse order. Sent little endian but big endian system of vice versa.
    value->b[0] = buffer[offset + 3];
    value->b[1] = buffer[offset + 2];
    value->b[2] = buffer[offset + 1];
    value->b[3] = buffer[offset];
  }else{
    value->b[0] = buffer[offset];
    value->b[1] = buffer[offset + 1];
    value->b[2] = buffer[offset + 2];
    value->b[3] = buffer[offset + 3];
  }
}

///////////////////////////////////////////////////////
/// 16-bit (2 byte) conversions
///////////////////////////////////////////////////////

void bufferValue16(Any16 value, bool littleEndian, uint8_t *buffer, uint8_t offset){
  if(isBigEndian == littleEndian){
    // Need to reverse order. Sent little endian but big endian system of vice versa.
    buffer[offset]     = value.b[1];
    buffer[offset + 1] = value.b[0];
  }else{
    buffer[offset]     = value.b[0];
    buffer[offset + 1] = value.b[1];
  }
}

void unbufferValue16(uint8_t *buffer, uint8_t offset, bool littleEndian, Any16 *value){
  if(isBigEndian == littleEndian){
    // Need to reverse order. Sent little endian but big endian system of vice versa.
    value->b[0] = buffer[offset + 1];
    value->b[1] = buffer[offset];
  }else{
    value->b[0] = buffer[offset];
    value->b[1] = buffer[offset + 1];
  }
}

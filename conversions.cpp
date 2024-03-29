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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware.  If not, see <https://www.gnu.org/licenses/>. 
 */

#include "conversions.h"

bool Conversions::isBigEndian = false;

void Conversions::checkBigEndian(){
    // i = 0x0001
    //  On big endian systems this is stored 0x00, 0x01
    // c = pointer to leftmost byte (array)
    // c[0] = leftmost byte. 1 on little endian. 0 on big endian
    uint16_t i = 1;
    uint8_t *c = (uint8_t*)&i;
    isBigEndian = !c[0];
}

void Conversions::convertInt32ToData(int32_t input, uint8_t *outBuffer, bool littleEndian){
    if(littleEndian){
        outBuffer[0] = input;
        outBuffer[1] = input >> 8;
        outBuffer[2] = input >> 16;
        outBuffer[3] = input >> 24;
    }else{
        outBuffer[0] = input >> 24;
        outBuffer[1] = input >> 16;
        outBuffer[2] = input >> 8;
        outBuffer[3] = input;
    }
}

int32_t Conversions::convertDataToInt32(uint8_t *data, bool littleEndian){
    if(littleEndian){
        return (int32_t)data[0] | 
               (int32_t)data[1] << 8 | 
               (int32_t)data[2] << 16 |
               (int32_t)data[3] << 24;
    }else{
        return (int32_t)data[0] << 24| 
               (int32_t)data[1] << 16 | 
               (int32_t)data[2] << 8 |
               (int32_t)data[3];
    }
}

void Conversions::convertInt16ToData(int16_t input, uint8_t *outBuffer, bool littleEndian){
    if(littleEndian){
        outBuffer[0] = input;
        outBuffer[1] = input >> 8;
    }else{
        outBuffer[0] = input >> 8;
        outBuffer[1] = input;
    }
}

int16_t Conversions::convertDataToInt16(uint8_t *data, bool littleEndian){
    if(littleEndian){
        return data[0] | data[1] << 8;
    }else{
        return data[0] << 8 | data[1];
    }
}

void Conversions::convertFloatToData(float input, uint8_t *outBuffer, bool littleEndian){
    // Pointer to leftmost byte of input
    uint8_t *ptr = (uint8_t*)&input;
    
    if(isBigEndian == littleEndian){
        // System endianess and desired endianess different, so reverse order
        outBuffer[0] = ptr[3];
        outBuffer[1] = ptr[2];
        outBuffer[2] = ptr[1];
        outBuffer[3] = ptr[0];
    }else{
        // System endianess and desired endianess match, so no need to reverse order
        outBuffer[0] = ptr[0];
        outBuffer[1] = ptr[1];
        outBuffer[2] = ptr[2];
        outBuffer[3] = ptr[3];
    }
}

float Conversions::convertDataToFloat(uint8_t *data, bool littleEndian){
    uint8_t dataRaw[4];
    
    if(isBigEndian == littleEndian){
        // System endianess and desired endianess different, so reverse order
        dataRaw[0] = data[3];
        dataRaw[1] = data[2];
        dataRaw[2] = data[1];
        dataRaw[3] = data[0];
    }else{
        // System endianess and desired endianess match, so no need to reverse order
        dataRaw[0] = data[0];
        dataRaw[1] = data[1];
        dataRaw[2] = data[2];
        dataRaw[3] = data[3];
    }

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wstrict-aliasing"

    // Dereference pointer to start of dataRaw interpreted as float
    return *((float*)(dataRaw));

    #pragma GCC diagnostic pop
}
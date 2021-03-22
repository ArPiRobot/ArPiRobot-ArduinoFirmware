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

#include "i2c.hpp"
#include <Wire.h>

void I2CHelper::writeHelper(BaseComm &comm, uint8_t *data, uint8_t len){
  // args: address, writeData*, stop
  // result: none
  uint8_t address = data[0];
  bool stop = data[len - 1];
  uint8_t *writeData = data + 1;
  uint16_t writeLen = len - 1;
  Wire.beginTransmission(address);
  Wire.write(writeData, writeLen);
  if(Wire.endTransmission(stop) == 0){
    return comm.respond(ErrorCode::NONE, nullptr, 0);
  }else{
    return comm.respond(ErrorCode::EXECUTION, nullptr, 0);
  }
}


void I2CHelper::writeByteHelper(BaseComm &comm, uint8_t *data, uint8_t len){
  // args: address, register, value, stop
  // result: none
  uint8_t address = data[0];
  uint8_t reg = data[1];
  uint8_t val = data[2];
  uint8_t stop = data[3];
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  if(Wire.endTransmission(stop) == 0){
    return comm.respond(ErrorCode::NONE, nullptr, 0);
  }else{
    return comm.respond(ErrorCode::EXECUTION, nullptr, 0);
  }
}

void I2CHelper::readByteHelper(BaseComm &comm, uint8_t *data, uint8_t len){
  // args: address, register, stop
  // result: read byte
  uint8_t address = data[0];
  uint8_t reg = data[1];
  uint8_t stop = data[2];
  Wire.beginTransmission(address);
  Wire.write(reg);
  if(Wire.endTransmission(stop) != 0){
    return comm.respond(ErrorCode::EXECUTION, nullptr, 0);
  }
  if(Wire.requestFrom(address, 1) != 1){
    return comm.respond(ErrorCode::EXECUTION, nullptr, 0);
  }
  uint8_t res = Wire.read();
  return comm.respond(ErrorCode::NONE, &res, 1);
}

void I2CHelper::readHelper(BaseComm &comm, uint8_t *data, uint8_t len){
  // args: address, count
  // result: read data (up to count bytes)
  uint8_t address = data[0];
  uint8_t count = data[1];
  uint8_t readBuffer[count];
  uint8_t avail = Wire.requestFrom(address, count);
  for(uint8_t i = 0; i < avail; +i){
    readBuffer[i] = Wire.read();
  }
  return comm.respond(ErrorCode::NONE, readBuffer, avail);
}

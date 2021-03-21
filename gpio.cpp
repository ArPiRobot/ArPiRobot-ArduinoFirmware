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

#include "gpio.hpp"
#include "conversions.hpp"

void GpioHelper::pinModeHelper(BaseComm &comm, uint8_t *data, uint8_t len){
  // Arguments: pin_num, mode (0 = output, 1 = input, 2 = input_pullup)
  if(len < 2){
    return comm.respond(ErrorCode::NOT_ENOUGH_ARGS, nullptr, 0);
  }
  switch(data[1]){
  case 0:
    pinMode(data[0], OUTPUT);
    return comm.respond(ErrorCode::NONE, nullptr, 0);
  case 1:
    pinMode(data[0], INPUT);
    return comm.respond(ErrorCode::NONE, nullptr, 0);
  case 2:
    pinMode(data[0], INPUT_PULLUP);
    return comm.respond(ErrorCode::NONE, nullptr, 0);
  default:
    return comm.respond(ErrorCode::INVALID_ARG, nullptr, 0);
  }
}

void GpioHelper::digitalWriteHelper(BaseComm &comm, uint8_t *data, uint8_t len){
  // Arguments: pin_num, state (0 = LOW, 1 = HIGH)
  if(len < 2){
    return comm.respond(ErrorCode::NOT_ENOUGH_ARGS, nullptr, 0);
  }
  switch(data[1]){
  case 0:
    digitalWrite(data[0], LOW);
    return comm.respond(ErrorCode::NONE, nullptr, 0);
  case 1:
    digitalWrite(data[0], HIGH);
    return comm.respond(ErrorCode::NONE, nullptr, 0);
  default:
    return comm.respond(ErrorCode::INVALID_ARG, nullptr, 0);
  }
}

void GpioHelper::digitalReadHelper(BaseComm &comm, uint8_t *data, uint8_t len){
  // Arguments: pin_num
  if(len < 1){
    return comm.respond(ErrorCode::NOT_ENOUGH_ARGS, nullptr, 0);
  }
  uint8_t res = (digitalRead(data[0]) == HIGH);
  return comm.respond(ErrorCode::NONE, &res, 1);
}

void GpioHelper::analogWriteHelper(BaseComm &comm, uint8_t *data, uint8_t len){
  // Arguments: pin_num, pwm (0-255)
  if(len < 2){
    return comm.respond(ErrorCode::NOT_ENOUGH_ARGS, nullptr, 0);
  }
  LOG("PWM: ");
  LOGLN((int)data[1]);
  analogWrite(data[0], data[1]);
  return comm.respond(ErrorCode::NONE, nullptr, 0);
}

void GpioHelper::analogReadHelper(BaseComm &comm, uint8_t *data, uint8_t len){
  // Arguments: pin_num
  if(len < 1){
    return comm.respond(ErrorCode::NOT_ENOUGH_ARGS, nullptr, 0);
  }
  uint8_t resData[2];
  Conversions::convertInt16ToData(analogRead(data[0]), resData, false);
  return comm.respond(ErrorCode::NONE, resData, 2);
}

void GpioHelper::analogInputToDigitalPinHelper(BaseComm &comm, uint8_t *data, uint8_t len){
  // Arguments: pin_num
  if(len < 1){
    return comm.respond(ErrorCode::NOT_ENOUGH_ARGS, nullptr, 0);
  }
  uint8_t res = analogInputToDigitalPin(data[0]);
  return comm.respond(ErrorCode::NONE, &res, 1);
}

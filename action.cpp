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

#include "action.hpp"
#include "conversions.hpp"

bool AutoDigitalRead::service(){
  uint8_t state = digitalRead(pin);
  bool res = (state != lastState);
  lastState = state;
  if(res){
    dt = micros() - lastChange;
    lastChange += dt;
  }
  return res;
}

void AutoDigitalRead::sendData(uint8_t actionId, BaseComm &comm){
  // source is actionId is uint8_t
  // dt is uint32_t
  // state is uint8_t
  uint8_t data[6];
  data[0] = actionId;
  Conversions::convertInt32ToData(dt, data + 1, false);
  data[5] = (lastState == HIGH);
  comm.sendStatus(data, 6);
}

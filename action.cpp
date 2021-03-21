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
#include "comm.hpp"

////////////////////////////////////////////////////////////////////////////////
/// AutoAction
////////////////////////////////////////////////////////////////////////////////

uint8_t AutoAction::nextActionId = 0;

AutoAction::AutoAction(){
  actionId = nextActionId;
  nextActionId++;  
}

uint8_t AutoAction::getActionId(){
  return actionId;
}


////////////////////////////////////////////////////////////////////////////////
/// AutoDigitalRead
////////////////////////////////////////////////////////////////////////////////

bool AutoDigitalRead::configure(uint8_t pin){
  this->pin = pin;
  return true; // Any pin is valid
}

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

void AutoDigitalRead::sendData(BaseComm &comm){
  // source is actionId is uint8_t
  // dt is uint32_t
  // state is uint8_t
  uint8_t data[6];
  data[0] = actionId;
  Conversions::convertInt32ToData(dt, data + 1, false);
  data[5] = (lastState == HIGH);
  comm.sendStatus(data, 6);
}


////////////////////////////////////////////////////////////////////////////////
/// AutoAnalogRead
////////////////////////////////////////////////////////////////////////////////

bool AutoAnalogRead::configure(uint8_t pin, uint16_t changeThreshold, uint16_t sendRate){
  this->pin = pin;
  this->changeThreshold = changeThreshold;
  this->sendRate = (unsigned long)sendRate * 1000;
}

bool AutoAnalogRead::service(){
  uint16_t state = analogRead(pin);
  bool res = (abs(state - lastState) > changeThreshold);
  dt = micros() - lastChange;
  res |= ((dt >= sendRate) && (state != lastState));
  if(res){
    lastState = state;
    lastChange += dt;
  }
  return res;
}

void AutoAnalogRead::sendData(BaseComm &comm){
  // source is actionId is uint8_t
  // dt is uint32_t
  // state is uint16_t
  uint8_t data[7];
  data[0] = actionId;
  Conversions::convertInt32ToData(dt, data + 1, false);
  Conversions::convertInt16ToData(lastState, data + 5, false);
  comm.sendStatus(data, 7);
}


////////////////////////////////////////////////////////////////////////////////
/// AutoDigitalCounter
////////////////////////////////////////////////////////////////////////////////

bool AutoDigitalCounter::configure(uint8_t pin, uint16_t changeThreshold, uint16_t sendRate){
  this->pin = pin;
  this->changeThreshold = changeThreshold;
  this->sendRate = (unsigned long)sendRate * 1000;
}

bool AutoDigitalCounter::service(){
  uint16_t state = digitalRead(pin);
  if(lastState != state){
    newCounts++;
  }
  lastState = state;
  bool res = (newCounts >= changeThreshold);
  dt = micros() - lastChange;
  res |= ((dt >= sendRate) && (newCounts != 0));
  if(res){
    lastChange += dt;
  }
  return res;
}

void AutoDigitalCounter::sendData(BaseComm &comm){
  // source is actionId is uint8_t
  // dt is uint32_t
  // counts is uint8_t
  uint8_t data[6];
  data[0] = actionId;
  Conversions::convertInt32ToData(dt, data + 1, false);
  data[5] = newCounts;
  comm.sendStatus(data, 6);
  newCounts = 0;
}

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

#pragma once

#include <stdint.h>

// Forward declare
class BaseComm;

class AutoAction{
public:
  AutoAction();

  virtual bool service() = 0;
  virtual void sendData(BaseComm &comm) = 0;

  uint8_t getActionId();

protected:
  uint8_t actionId;

private:
  static uint8_t nextActionId;
};


// Sends status message when change occurs on polled pin
class AutoDigitalRead : public AutoAction{
public:
  bool configure(uint8_t pin);
  bool service() override;
  void sendData(BaseComm &comm) override;
private:
  uint8_t pin;
  uint8_t lastState = 255;
  unsigned long lastChange = 0;  // Time of last pin change
  unsigned long dt = 0;          // Time since last pin change
};


// Sends status message when one of the following is true of the polled pin
//     - Change greater than change threshold occurs
//     - Any change occurs and dt excedes sendRate (millis)
class AutoAnalogRead : public AutoAction{
public:
  bool configure(uint8_t pin, uint16_t changeThreshold, uint16_t sendRate);
  bool service() override;
  void sendData(BaseComm &comm) override;
private:
  uint8_t pin;
  uint16_t changeThreshold;
  unsigned long sendRate;
  uint16_t lastState = 2000;
  unsigned long lastChange = 0;  // Time of last pin change
  unsigned long dt = 0;          // Time since last pin change
};


// Sends status message containing number of counts (pin state changes)
//    since last message.
// Will send updates when one of the following is true:
//     - New counts exceed changeThreshold
//     - Any change occurs and dt excedes sendRate (millis)
class AutoDigitalCounter : public AutoAction{
public:
  bool configure(uint8_t pin, uint16_t changeThreshold, uint16_t sendRate);
  bool service() override;
  void sendData(BaseComm &comm) override;
private:
  uint8_t pin;
  uint16_t changeThreshold;
  unsigned long sendRate;
  uint8_t newCounts = 0;
  uint8_t lastState = 255;
  unsigned long lastChange = 0;  // Time of last count send
  unsigned long dt = 0;          // Time since last count send
};

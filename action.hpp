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

#include "comm.hpp"

class AutoAction{
public:
  virtual bool service() = 0;
  virtual void sendData(uint8_t actionId, BaseComm &comm) = 0;
};

// Sends status message when change occurs on polled pin
class AutoDigitalRead : public AutoAction{
public:
  bool service() override;
  void sendData(uint8_t actionId, BaseComm &comm) override;
private:
  uint8_t lastState = 255;
  uint8_t pin;
  unsigned long lastChange = 0;  // Time of last pin change
  unsigned long dt = 0;          // Time since last pin change
};

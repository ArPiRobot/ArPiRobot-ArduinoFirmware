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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware. If not, see <https://www.gnu.org/licenses/>. 
 */

#pragma once

#include "settings.h"

class ComputerInterface;

////////////////////////////////////////////////////////////////////////////////
/// Device
////////////////////////////////////////////////////////////////////////////////

class Device {
public:
    Device() = default;
    Device(const Device &other) = delete;
    Device(Device &&other) = delete;

    Device &operator=(const Device &other) = delete;
    Device &operator=(Device &&other) = delete;
    
    virtual ~Device();

    /**
     * Periodically called to service the device
     * Primarily used for polling of sensors
     * @return true when data should be sent to the computer by the interface
     *         Will cause getData to be called
     */
    virtual uint16_t process() = 0;

    /**
     * Called when the interface wants the data that should be sent to the computer for this sensor
     * 
     * @param data Buffer to write the data into. Do not write more than 
     * @return uint16_t The number of bytes written into the buffer
     */
    virtual uint16_t getData(uint8_t *data);

    /**
     * Handle data sent from the arduino to this device
     * 
     * @param data Pointer to the data buffer 
     * @param len Number of bytes of data available in the buffer
     */
    virtual void handleData(const uint8_t *data, const uint16_t len);

private:

};

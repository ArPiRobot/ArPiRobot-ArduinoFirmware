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

#include <Arduino.h>
#include "board.h"

class RPiInterface;

/**
 * Any device that can be added to a RPiInterface
 */
class ArduinoDevice{
public:

    /**
     * @param sendBufferSize How many bytes internal send buffer should be
     */
    ArduinoDevice(uint16_t sendBufferSize);

    virtual ~ArduinoDevice();

    /**
     * Get the size the buffer for getSendData must be
     * This is the maximum number of bytes that may be written into the buffer
     * @return sendBufferSize
     */
    uint16_t getSendBufferSize();

    /**
     * Call this after sending data for this sensor
     * Last send time will be stored
     */
    void updateSendTime();

    /**
     * Get the data to be sent from this device
     * @param data Buffer to write data into (must be at least sendBufferSize)
     * @return The number of bytes written into the buffer
     */
    virtual uint16_t getSendData(uint8_t *data) = 0;

    /**
     * Handle periodic actions for this device
     * @return true if data should be sent from this device to the Pi
     */
    virtual bool service() = 0;

    /**
     * Handle some data sent to this device by the Pi
     */
    virtual void handleMessage(uint8_t *msg, uint16_t len) = 0;

protected:
    uint16_t sendBufferSize;     // Size is max size

    // How often this device should send data back to the Pi (not how often this device should be serviced)
    uint16_t sendRateMs = 50;

    // The last time updateSendTime was called
    unsigned long lastSendTime = 0;

    int16_t deviceId = -1; // Device id is only an 8-bit int (unsigned), but is negative if invalid

    friend class RPiInterface;
};

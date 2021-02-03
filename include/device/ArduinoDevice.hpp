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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware. If not, see <https://www.gnu.org/licenses/>. 
 */

#pragma once

#include <Arduino.h>
#include <board.h>

/**
 * Any device that can be added to a RPiInterface
 */
class ArduinoDevice{
public:

    /**
     * @param sendBufferSize How many bytes internal send buffer should be
     * @param sendRateMs How frequently (in ms) data from this sensor should be sent to the pi
     */
    ArduinoDevice(uint16_t sendBufferSize, uint16_t sendRateMs = 50);

    ArduinoDevice(const ArduinoDevice &other);

    virtual ~ArduinoDevice();

    virtual ArduinoDevice &operator=(const ArduinoDevice &other);

    /**
     * Get the data to be sent from this device
     * @param data Will be set to a pointer to the data to send
     * @param len Will be set to the length of the data to send
     */
    virtual void getSendData(uint8_t **data, uint16_t *maxLen) = 0;

    /**
     * Handle periodic actions for this device
     * @return true if data should be sent from this device to the Pi
     */
    virtual bool service() = 0;

    /**
     * Handle some data sent to this device by the Pi
     */
    virtual void handleMessage(uint8_t *msg, uint16_t len);

protected:
    // How often this device should send data back to the Pi (not how often this device should be serviced)
    uint16_t sendRateMs;

    // The next time to send data (calculated by getSendData function)
    unsigned long nextSendTime = 0;

    // Buffer sensor data is written into to be sent to the Pi
    uint8_t *sendBuffer;
    uint16_t sendBufferLen = 0;  // Len is current number of items in buffer
    uint16_t sendBufferSize;     // Size is max size

    int16_t deviceId = -1; // Device id is only an 8-bit int (unsigned), but is negative if invalid

    friend class RPiInterface;
};

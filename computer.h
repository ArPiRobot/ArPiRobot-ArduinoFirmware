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

#include "board.h"
#include "settings.h"
#include "devices.h"

class ComputerInterface{
public:
    ComputerInterface() = default;
    ComputerInterface(const ComputerInterface &other) = delete;
    ComputerInterface(ComputerInterface &&other) = delete;

    ComputerInterface &operator=(const ComputerInterface &other) = delete;
    ComputerInterface &operator=(ComputerInterface &&other) = delete;
    
    virtual ~ComputerInterface();

    void init();
    void process();

protected:
    virtual void open() = 0;
    virtual void close() = 0;
    virtual void write(uint8_t b) = 0;
    virtual uint8_t read() = 0;
    virtual uint16_t available() = 0;

private:
    void writeData(const uint8_t *data, const uint16_t len);
    void writeData(const char *data, const uint16_t len);
    bool readData();
    bool checkData();

    Device *devices[IFACE_MAX_DEVICES];
    uint16_t devicesLen;
    uint16_t staticDeviceCount;
    uint8_t *readBuffer[IFACE_READ_BUFFER_SIZE];
    uint16_t readBufferLen;

    static const uint8_t START_BYTE = 253;
    static const uint8_t END_BYTE = 254;
    static const uint8_t ESCAPE_BYTE = 255;
};


class ComputerUartInterface : public ComputerInterface {
public:
    ComputerUartInterface(IFACE_SERIAL_T &serial, unsigned long baud);
    ComputerUartInterface(const ComputerUartInterface &other) = delete;
    ComputerUartInterface(ComputerUartInterface &&other) = delete;

    ~ComputerUartInterface() = default;

    ComputerUartInterface &operator=(const ComputerUartInterface &other) = delete;
    ComputerUartInterface &operator=(ComputerUartInterface &&other) = delete;

protected:
    void open() override;
    void close() override;
    void write(uint8_t b) override;
    uint8_t read() override;
    uint16_t available() override;

private:
    IFACE_SERIAL_T &serial;
    unsigned long baud;
};
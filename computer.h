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
#include <FastCRC.h>

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
    void reset();

    uint8_t addDeviceFromData(uint8_t *data, uint16_t len);

    void writeData(const uint8_t *data, const uint16_t len);
    bool readData();
    bool checkData();
    

    bool parseStarted, parseEscaped;

    Device *devices[IFACE_MAX_DEVICES];
    uint16_t devicesLen;
    uint16_t staticDeviceCount;
    uint8_t readBuffer[IFACE_READ_BUFFER_SIZE];
    uint16_t readBufferLen;

    FastCRC16 crcInst;
};

template <class SER_T>
class ComputerUartInterface : public ComputerInterface {
public:
    ComputerUartInterface(SER_T &serial, unsigned long baud) : serial(serial), baud(baud) {  }

    ComputerUartInterface(const ComputerUartInterface &other) = delete;
    ComputerUartInterface(ComputerUartInterface &&other) = delete;

    ~ComputerUartInterface(){ close(); }

    ComputerUartInterface &operator=(const ComputerUartInterface &other) = delete;
    ComputerUartInterface &operator=(ComputerUartInterface &&other) = delete;

protected:
    void open() override { serial.begin(baud); }
    void close() override { serial.end(); }
    void write(uint8_t b) override { serial.write(b); }
    uint8_t read() override { return serial.read(); }
    uint16_t available() override { return serial.available(); }

    SER_T &serial;
    unsigned long baud;
};

class ComputerInterfaceFactory{
public:
    // Template functions can infer types, template classes cannot
    template<class T>
    static ComputerInterface *createUart(T serial, unsigned long baud) { 
        return new ComputerUartInterface<T>(serial, baud);
    }
};

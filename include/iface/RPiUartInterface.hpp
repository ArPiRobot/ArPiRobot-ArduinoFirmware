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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware.  If not, see <https://www.gnu.org/licenses/>. 
 */

#pragma once

#include <Arduino.h>
#include <board.h>
#include <iface/RPiInterface.hpp>

/**
 * Raspberry Pi interface that uses UART to communicate with the Pi
 */
class RPiUartInterface : public RPiInterface {
public:
    /**
     * @param serail The serial port to use (hardware or software serial)
     * @param baud The baud rate to use
     */
    RPiUartInterface(HW_SERIAL_T &serial, uint32_t baud);

    void open() override;
    uint16_t available() override;
    int16_t read() override;
    void write(uint8_t data) override;
    void flush() override;

private:
    HW_SERIAL_T &serial;
    uint32_t baud;
};

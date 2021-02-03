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

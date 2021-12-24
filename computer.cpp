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

#include "computer.h"
#include "conversions.h"


////////////////////////////////////////////////////////////////////////////////
/// ComputerInterface
////////////////////////////////////////////////////////////////////////////////

ComputerInterface::~ComputerInterface(){
    for(uint16_t i = staticDeviceCount; i < devicesLen; ++i){
        delete devices[i];
    }
    devicesLen = staticDeviceCount;
    close();
}

void ComputerInterface::init(){
    open();
}

void ComputerInterface::process(){
    
}

void ComputerInterface::writeData(const uint8_t *data, const uint16_t len){
    // Messages are in the form [START_BYTE],[MESSAGE_DATA]...,[CRC_HIGH],[CRC_LOW],[END_BYTE]
    // If the MESSAGE_DATA contains a START_BYTE or END_BYTE, it is prefixed with an ESCAPE_BYTE
    // If the message contains an ESCAPE_BYTE, it is also prefixed with an ESCAPE_BYTE
    // If the CRC bytes are START, END, OR ESCAPE BYTES they are prefixed with ESCAPE_BYTE

    uint8_t i;
    write(START_BYTE);
    for(i = 0; i < len; ++i){
        if(data[i] == START_BYTE || data[i] == END_BYTE || data[i] == ESCAPE_BYTE)
            write(ESCAPE_BYTE);
        write(data[i]);
    }

    uint16_t crc = rpi_crc.ccitt(data, len);
    uint8_t crc_data[2];
    Conversions::int16ToData(crc, crc_data, false);
    if(crc_data[0] == START_BYTE || crc_data[0] == END_BYTE || crc_data[0] == ESCAPE_BYTE)
        write(ESCAPE_BYTE);
    write(crc_data[0]);
    if(crc_data[1] == START_BYTE || crc_data[1] == END_BYTE || crc_data[1] == ESCAPE_BYTE)
        write(ESCAPE_BYTE);
    write(crc_data[1]);

    write(END_BYTE);
}

void ComputerInterface::writeData(const char *data, const uint16_t len){

}

bool ComputerInterface::readData(){

}

bool ComputerInterface::checkData(){

}


////////////////////////////////////////////////////////////////////////////////
/// ComputerUartInterface
////////////////////////////////////////////////////////////////////////////////

ComputerUartInterface::ComputerUartInterface(IFACE_SERIAL_T &serial, unsigned long baud) : 
        serial(serial), baud(baud){

}

void ComputerUartInterface::open(){
    serial.begin(baud);
}

void ComputerUartInterface::close(){
    serial.end();
}

void ComputerUartInterface::write(uint8_t b){
    serial.write(b);
}

uint8_t ComputerUartInterface::read(){
    return serial.read();
}

uint16_t ComputerUartInterface::available(){
    return serial.available();
}
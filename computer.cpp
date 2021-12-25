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
#include "util.h"

////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////

// Communication protocol constants
#define START_BYTE                      253
#define END_BYTE                        254
#define ESCAPE_BYTE                     255

// Sensor ID constants
#define DEVICE_SINGLE_ENCODER           0
#define DEVICE_ULTRASONIC_4             1
#define DEVICE_IR_REFLECTOR             2
#define DEVICE_OLDADA9DOF               3
#define DEVICE_NXPADA9DOF               4
#define DEVICE_MPU6050                  5
#define DEVICE_VMON_ANALOG              6


////////////////////////////////////////////////////////////////////////////////
/// ComputerInterface
////////////////////////////////////////////////////////////////////////////////

ComputerInterface::~ComputerInterface(){
    reset();
}

void ComputerInterface::init(){
    open();
    parseStarted = false;
    parseEscaped = false;

    // Send "READY" when the interface is first started
    // Some arduinos will hardware reset when UART is opened via USB
    // This allows the computer to know that the reset has finished
    // Allows allows the computer to identify if the arduino has unintentionally reset
    writeData((uint8_t*)"READY", 5);
}

void ComputerInterface::process(){
    if(readData() && checkData()){
        // Don't need CRC anymore
        readBufferLen -= 2;

        if(readBuffer[0] == '^'){
            // Data starting with ^ is data to give to a device instance
        }else{
            // Data is a command
            // Commands: ADD = Add a device
            //           RESET = Remove all devices
            // The data used to create a device depends on what the device is
            if(Util::dataMatches(readBuffer, readBufferLen, (uint8_t*)"ADD", 3)){
                // Response should be ADD[deviceId]
                // Device ID of 255 unsigned == -1 signed means add failure
                uint8_t response[] = {'A', 'D', 'D', 0};
                response[3] = addDeviceFromData(readBuffer + 3, readBufferLen - 2);
                writeData(response, 4);
            }else if(Util::dataMatches(readBuffer, readBufferLen, (uint8_t*)"RESET", 5)){
                reset();
                // Confirm reset 
                writeData((uint8_t*)"RESET", 5);
            }
        }

        // Clear read buffer
        readBufferLen = 0;
    }
}

void ComputerInterface::reset(){
    for(uint16_t i = staticDeviceCount; i < devicesLen; ++i){
        delete devices[i];
    }
    devicesLen = staticDeviceCount;
}

uint8_t ComputerInterface::addDeviceFromData(uint8_t *data, uint16_t len){
    // First byte indicates what type of device to add
    // Second and third bytes are a data send rate in milliseconds
    // Rest of the bytes are configuration data and are device specific
    switch(data[0]){
    case DEVICE_SINGLE_ENCODER:
        break;
    case DEVICE_ULTRASONIC_4:
        break;
    case DEVICE_IR_REFLECTOR:
        break;
    case DEVICE_OLDADA9DOF:
        break;
    case DEVICE_NXPADA9DOF:
        break;
    case DEVICE_MPU6050:
        break;
    case DEVICE_VMON_ANALOG:
        break;
    }
    return 255; // Inidicates add failure
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

    uint16_t crc = crcInst.ccitt(data, len);
    uint8_t crcData[2];
    Conversions::int16ToData(crc, crcData, false);
    if(crcData[0] == START_BYTE || crcData[0] == END_BYTE || crcData[0] == ESCAPE_BYTE)
        write(ESCAPE_BYTE);
    write(crcData[0]);
    if(crcData[1] == START_BYTE || crcData[1] == END_BYTE || crcData[1] == ESCAPE_BYTE)
        write(ESCAPE_BYTE);
    write(crcData[1]);

    write(END_BYTE);
}

bool ComputerInterface::readData(){
    if(available()){
        uint8_t c = read();

        if(parseEscaped){
            // Ignore invalid escaped data
            if(c == START_BYTE || c == END_BYTE || c == ESCAPE_BYTE){
                readBuffer[readBufferLen++] = c;
                if(readBufferLen == IFACE_READ_BUFFER_SIZE)
                    readBufferLen = 0;
            }
            parseEscaped = false;
        }else{
            if(c == START_BYTE){
                readBufferLen = 0;
                parseStarted = true;
            }else if(c == END_BYTE && parseStarted){
                parseStarted = false;
                return true;
            }else if(c == ESCAPE_BYTE && parseStarted){
                parseEscaped = true;
            }else if(parseStarted){
                readBuffer[readBufferLen++] = c;
                if(readBufferLen == IFACE_READ_BUFFER_SIZE)
                    readBufferLen = 0;
            }
        }
    }
    return false;
}

bool ComputerInterface::checkData(){
    uint16_t readCrc = Conversions::dataToInt16(&readBuffer[readBufferLen - 2], false);
    uint16_t calcCrc = crcInst.ccitt(readBuffer, readBufferLen - 2);
    if(readCrc == calcCrc){
        return true;
    }else{
        log_println("CRC!=");
        return false;
    }
}

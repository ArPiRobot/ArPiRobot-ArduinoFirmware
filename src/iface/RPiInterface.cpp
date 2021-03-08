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

#include <iface/RPiInterface.hpp>
#include <Conversions.hpp>
#include <settings.h>
#include <sensor/SingleEncoder.hpp>
#include <sensor/VoltageMonitor.hpp>
#include <sensor/IRReflectorModule.hpp>
#include <sensor/Ultrasonic4Pin.hpp>
#include <sensor/OldAdafruit9Dof.hpp>
#include <sensor/NxpAdafruit9Dof.hpp>
#include <sensor/Mpu6050Imu.hpp>

FastCRC16 CRC16;

RPiInterface::~RPiInterface(){
    // Nothing special here. Just virtual destructor to ensure proper behavior from child classes.
}

int16_t RPiInterface::addStaticDevice(ArduinoDevice *device){
    if(!canAddStatic)
        return -1;

    // Add static devices at the beginning of the linked list
    device->deviceId = devices.size();
    devices.add(0, device);

    staticDeviceCount++;

    return device->deviceId;
}

int16_t RPiInterface::addDevice(){
    ArduinoDevice *device = nullptr;
    if(dataStartsWith(readBuffer, readBufferLen, (uint8_t*)"ADDSENC", 7)){
        #ifdef SingleEncoder_ENABLE
        // Pass buffer without "ADDSENC" and without CRC
        device = new SingleEncoder(&readBuffer[7], readBufferLen - 9);
        #endif
    }else if(dataStartsWith(readBuffer, readBufferLen, (uint8_t*)"ADDVMON", 7)){
        #ifdef VoltageMonitor_ENABLE
        // Pass buffer without "ADDVMON" and without CRC
        device = new VoltageMonitor(&readBuffer[7], readBufferLen - 9);
        #endif
    }else if(dataStartsWith(readBuffer, readBufferLen, (uint8_t*)"ADDIRREFLECTOR", 14)){
        #ifdef IRReflectorModule_ENABLE
        // Pass buffer without "ADDIRREFLECTOR" and without CRC
        device = new IRReflectorModule(&readBuffer[14], readBufferLen - 16);
        #endif
    }else if(dataStartsWith(readBuffer, readBufferLen, (uint8_t*)"ADDUSONIC4", 10)){
        #ifdef Ultrasonic4Pin_ENABLE
        // Pass buffer without "ADDUSONIC4" and without CRC
        device = new Ultrasonic4Pin(&readBuffer[10], readBufferLen - 12);
        #endif
    }else if(dataStartsWith(readBuffer, readBufferLen, (uint8_t*)"ADDOLDADA9DOF", 13)){
        #ifdef OldAdafruit9Dof_ENABLE
        // Pass buffer without "ADDOLDADA9DOF" and without CRC
        device = new OldAdafruit9Dof(&readBuffer[13], readBufferLen - 15);
        #endif
    }else if(dataStartsWith(readBuffer, readBufferLen, (uint8_t*)"ADDNXPADA9DOF", 13)){
        #ifdef NxpAdafruit9Dof_ENABLE
        // Pass buffer without "ADDNXPADA9DOF" and without CRC
        device = new NxpAdafruit9Dof(&readBuffer[13], readBufferLen - 15);
        #endif
    }else if(dataStartsWith(readBuffer, readBufferLen, (uint8_t*)"ADDMPU6050", 10)){
        #ifdef Mpu6050Imu_ENABLE
        // Pass buffer without "ADDMPU6050" and without CRC
        device = new Mpu6050Imu(&readBuffer[10], readBufferLen - 12);
        #endif
    }

    if(device != nullptr){
        devices.add(device);
        device->deviceId = devices.size() - 1;
        uint8_t buf[11] = "ADDSUCCESS"; // 10 character string  + 1 for a device id
        buf[10] = device->deviceId;
        writeData(buf, 11);
        return device->deviceId;
    }else{
        writeData((uint8_t*)"ADDFAIL", 7);
        return -1;
    }
}

void RPiInterface::reset(){
    // Delete non-static devices ((deviceCount - statiDeviceCount) devices removed from the front)
    // Non-static devices added to the front of the linked list after static devices
    for(int i = staticDeviceCount; i < devices.size(); ++i){
        ArduinoDevice *d = devices.get(0);
        delete d;
        devices.remove(0);
    }
    
    // Empty buffer
    readBufferLen = 0;
}

void RPiInterface::run(){

    open();

    // No longer allow static devices. 
    // They must be added to the linked list before any dynamic devices (devices created by data from the pi)
    canAddStatic = false;

    writeData((uint8_t*)"START", 5);
    while(true){
        if(readData() && checkData()){
            // Ignore CRC in the buffer now
            readBufferLen -= 2;

            if(dataDoesMatch(readBuffer, readBufferLen, (uint8_t*)"END", 3)){
                break;
            }else if(dataDoesMatch(readBuffer, readBufferLen, (uint8_t*)"RESET", 5)){
                reset();
                writeData((uint8_t*)"START", 5);                
            }else if(dataStartsWith(readBuffer, readBufferLen, (uint8_t*)"ADD", 3)){
                addDevice();
            }

            // Clear read buffer
            readBufferLen = 0;
        }
    }

    readBufferLen = 0;
    writeData((uint8_t*)"END", 3);
    flush();

    while(true){
        // Service devices and send data as needed
        for(int i = 0; i < devices.size(); ++i){
            ArduinoDevice *d = devices.get(i);

            // Service will return true if there is data to send
            if(d->service()){
                uint8_t data[d->getSendBufferSize() + 1];
                data[0] = d->deviceId;
                uint16_t len = d->getSendData(&data[1]);
                d->updateNextSendTime();
                writeData(data, len + 1);
                flush();
            }
        }

        // Read available data
        uint16_t count = available();
        while(count > 0){

            if(readData() && checkData()){
                // Ignore CRC now
                readBufferLen -= 2;

                if(dataDoesMatch(readBuffer, readBufferLen, (uint8_t*)"RESET", 5)){
                    reset();
                    run();
                    return;
                }else if (dataStartsWith(readBuffer, readBufferLen, (uint8_t*)"-", 1)){
                    uint8_t id = readBuffer[1];
                    for(uint8_t i = 0; i < devices.size(); ++i){
                        // Iterating in forward order will not cause performance issues. get() method caches.
                        ArduinoDevice *d = devices.get(i);
                        if(d->deviceId == id){
                            // Skip the '-' in the data given to the device as well as device id
                            d->handleMessage(&readBuffer[2], readBufferLen - 2);
                            break;
                        }
                    }
                }

                // Clear buffer after handling data
                readBufferLen = 0;
            }

            count--;
        }
    }

}

bool RPiInterface::readData(){
    int16_t c;
    if(available() > 0){
        c = read();
    }else{
        return false;
    }

    if(parseEscaped){
        // Ignore invalid escaped data
        if(c == START_BYTE || c == END_BYTE || c == ESCAPE_BYTE){
            readBuffer[readBufferLen++] = c;
        }
        parseEscaped = false;
    }else{
        if(c == START_BYTE){
            if(parseStarted){
                // Got a second start byte. Empty buffer
                readBufferLen = 0;
            }
            parseStarted = true;
        }else if(c == END_BYTE && parseStarted){
            parseStarted = false;
            return true;
        }else if(c == ESCAPE_BYTE && parseStarted){
            parseEscaped = true;
        }else if(parseStarted){
            readBuffer[readBufferLen++] = c;
        }
    }
    return false;
}

void RPiInterface::writeData(uint8_t *data, uint16_t len){
    write(START_BYTE);

    // Send the actual data
    for(uint16_t i = 0; i < len; ++i){
        if(data[i] == END_BYTE || data[i] == START_BYTE || data[i] == ESCAPE_BYTE){
            write(ESCAPE_BYTE);
        }
        write(data[i]);
    }

    // Send CRC big endian;
    uint16_t crc = CRC16.ccitt(data, len);
    uint8_t crcData[2];
    Conversions::convertInt16ToData(crc, crcData, false);

    if(crcData[0] == ESCAPE_BYTE || crcData[0] == START_BYTE || crcData[0] == END_BYTE){
        write(ESCAPE_BYTE);
    }
    write(crcData[0]);

    if(crcData[1] == ESCAPE_BYTE || crcData[1] == START_BYTE || crcData[1] == END_BYTE){
        write(ESCAPE_BYTE);
    }
    write(crcData[1]);

    write(END_BYTE);
}

bool RPiInterface::checkData(){
    return (uint16_t)(Conversions::convertDataToInt16(&readBuffer[readBufferLen - 2], false)) 
            == CRC16.ccitt(readBuffer, readBufferLen - 2);
}

bool RPiInterface::dataStartsWith(uint8_t *data1, uint16_t len1, uint8_t *data2, uint16_t len2){
    if(len2 > len1) return false;
    for(uint8_t i = 0; i < len2; ++i){
        if(data1[i] != data2[i]) return false;
    }
    return true;
}

bool RPiInterface::dataDoesMatch(uint8_t *data1, uint16_t len1, uint8_t *data2, uint16_t len2){
    if(len1 != len2) return false;

    for(uint8_t i = 0; i < len1; ++i){
        if(data1[i] != data2[i]) return false;
    }
    return true;
}

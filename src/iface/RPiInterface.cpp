#include <iface/RPiInterface.hpp>
#include <conversions.hpp>

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
    // TODO: Implement device drivers first
    return -1;
}

void RPiInterface::reset(){
    // Delete non-static devices ((deviceCount - statiDeviceCount) devices removed from the front)
    // Non-static devices added to the front of the linked list after static devices
    for(uint8_t i = staticDeviceCount; i < devices.size(); ++i){
        devices.remove(0);
    }
    
    // Empty buffer
    readBufferLen = 0;
}

void RPiInterface::run(){

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
                run();
            }else if(dataStartsWith(readBuffer, readBufferLen, (uint8_t*)"ADD", 3)){
                int deviceId = addDevice();
                if(deviceId == -1){
                    writeData((uint8_t*)"ADDFAIL", 7);
                }else{
                    uint8_t buf[11] = "ADDSUCCESS";
                    buf[10] = deviceId;
                    writeData(buf, 11);
                }
            }

            // Clear read buffer
            readBufferLen = 0;
        }
    }

    readBufferLen = 0;
    writeData((uint8_t*)"END", 3);

    while(true){
        // Service devices and send data as needed
        for(uint16_t i = 0; i < devices.size(); ++i){
            ArduinoDevice *d = devices.get(i);

            // Service will return true if there is data to send
            if(d->service()){
                uint8_t *data;
                uint16_t len;
                d->getSendData(&data, &len);
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
                            // Skip the '-' in the data given to the device
                            d->handleMessage(&readBuffer[1], readBufferLen - 1);
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
    for(uint8_t i = 0; i < len; ++i){
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
    return Conversions::convertDataToInt16(&readBuffer[readBufferLen - 2], false) 
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

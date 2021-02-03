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

int16_t RPiInterface::addDevice(uint8_t data, uint16_t len){
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
    // TODO
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

bool RPiInterface::checkData(uint8_t *data, uint16_t len){
    return Conversions::convertDataToInt16(&data[len - 2], false) == CRC16.ccitt(data, len - 2);
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

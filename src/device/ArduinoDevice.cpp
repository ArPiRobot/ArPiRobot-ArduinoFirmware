#include <device/ArduinoDevice.hpp>

ArduinoDevice::ArduinoDevice(uint16_t sendBufferSize, uint16_t sendRateMs) : 
        sendBufferSize(sendBufferSize), sendRateMs(sendRateMs){
    sendBuffer = new uint8_t[sendBufferSize];
}

ArduinoDevice::ArduinoDevice(const ArduinoDevice &other) : 
        sendBufferSize(other.sendBufferSize), sendRateMs(other.sendRateMs){
    sendBuffer = new uint8_t[sendBufferSize];
}

ArduinoDevice::~ArduinoDevice(){
    delete[] sendBuffer;
}

ArduinoDevice &ArduinoDevice::operator=(const ArduinoDevice &other){
    if(this != &other){
        sendBufferSize = other.sendBufferSize;
        sendRateMs = other.sendRateMs;
        sendBuffer = new uint8_t[sendBufferSize];
        memcpy(sendBuffer, other.sendBuffer, sendBufferSize);
    }
    return *this;
}

void ArduinoDevice::getSendData(uint8_t **data, uint16_t *len){
    *data = sendBuffer;
    *len = sendBufferLen;
    nextSendTime = millis() + sendRateMs;
}

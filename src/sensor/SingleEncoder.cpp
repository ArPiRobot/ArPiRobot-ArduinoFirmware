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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware. If not, see <https://www.gnu.org/licenses/>. 
 */

#include <sensor/SingleEncoder.hpp>
#include <conversions.hpp>

SingleEncoder::SingleEncoder(uint8_t pin, bool pullup) : ArduinoDevice(3), pin(pin){
    pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
    lastState = digitalRead(pin);
}

SingleEncoder::SingleEncoder(uint8_t *data, uint16_t len) : ArduinoDevice(3){

}

SingleEncoder::SingleEncoder(const SingleEncoder &other) : ArduinoDevice(other){
    // No need to configure pinMode in copy constructor
    pin = other.pin;
    lastState = other.lastState;
    count = other.count;
}

SingleEncoder &SingleEncoder::operator=(const SingleEncoder &other){
    if(this != &other){
        ArduinoDevice::operator=(other);
        pin = other.pin;
        lastState = other.lastState;
        count = other.count;
    }
    return *this;
}

bool SingleEncoder::service(RPiInterface *rpi){
    bool state = digitalRead(pin);
    if(state != lastState){
        count++;
        sendBuffer[0] = deviceId;
        // Buffer count little endian
        Conversions::convertInt16ToData(count, &sendBuffer[1], true);
        sendBufferLen = 3;
        lastState = state;
        return millis() >= nextSendTime;
    }
    return false;
}

void SingleEncoder::handleMessage(uint8_t *data, uint16_t len){
    
}

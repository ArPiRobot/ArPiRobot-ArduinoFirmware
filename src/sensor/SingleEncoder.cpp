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

#include <sensor/SingleEncoder.hpp>
#include <Conversions.hpp>

SingleEncoder::SingleEncoder(uint8_t pin, bool pullup) : ArduinoDevice(2), pin(pin){
    pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
    lastState = digitalRead(pin);
}

SingleEncoder::SingleEncoder(uint8_t *data, uint16_t len) : ArduinoDevice(2){
    if(data[0]){
        pin = analogInputToDigitalPin(data[1]);
    }else{
        pin = data[1];
    }
    pinMode(pin, data[2] ? INPUT_PULLUP : INPUT);
    lastState = digitalRead(pin);
}

uint16_t SingleEncoder::getSendData(uint8_t *data){
    lastSentCount = count;
    // Buffer count little endian
    Conversions::convertInt16ToData(count, &data[0], true);
    return 2;
}

bool SingleEncoder::service(){
    bool state = digitalRead(pin);
    if(state != lastState){
        count++;
        lastState = state;
    }
    return ((millis() - lastSendTime) >= sendRateMs) && (count != lastSentCount);
}

void SingleEncoder::handleMessage(uint8_t *data, uint16_t len){
    
}

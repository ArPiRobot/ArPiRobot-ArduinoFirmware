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

#include <sensor/IRReflectorModule.hpp>
#include <Conversions.hpp>

IRReflectorModule::IRReflectorModule(uint8_t digitalPin, uint8_t analogPin) : ArduinoDevice(3), 
        digitalPin(digitalPin), analogPin(analogPin){
    pinMode(digitalPin, INPUT);
    lastDigitalState = digitalRead(digitalPin);
    if(analogPin != 255){
        pinMode(analogPin, INPUT);
        lastAnalogValue = analogRead(analogPin);
    }
}

IRReflectorModule::IRReflectorModule(uint8_t *data, uint16_t len) : ArduinoDevice(3) {
    if(data[0]){
        // Using an analog pin as a digital pin, specified analog pin number (0 = A0, 1 = A1, etc)
        digitalPin = analogInputToDigitalPin(data[1]);
    }else{
        digitalPin = data[1];
    }
    if(data[2] == 255){
         analogPin = 255;
    }else{
        analogPin = analogInputToDigitalPin(data[2]);
    }

    pinMode(digitalPin, INPUT);
    lastDigitalState = digitalRead(digitalPin);
    if(analogPin != 255){
        pinMode(analogPin, INPUT);
        lastAnalogValue = analogRead(analogPin);
    }
}

uint16_t IRReflectorModule::getSendData(uint8_t *data){
    changedSinceLastSend = false;

    // Invert this so 1 means reflection detected and 0 means no reflection detected. 
    // This way it matches the onboard LED
    data[0] = !lastDigitalState;

    if(analogPin != 255){
        // Send analog value as well
        Conversions::convertInt16ToData(lastAnalogValue, &data[1], true);
        return 3;
    }

    // No analog configured. Send digital only.
    return 1;
}

bool IRReflectorModule::service(){
    uint8_t state = digitalRead(digitalPin);
    if(state != lastDigitalState){
        changedSinceLastSend = true;
    }
    lastDigitalState = state == HIGH;

    if(analogPin != 255){
        uint16_t analogVal = analogRead(analogPin);
        if(analogVal != lastAnalogValue){
        changedSinceLastSend = true;
        }
        lastAnalogValue = analogVal;
    }

    return ((millis() - lastSendTime) >= sendRateMs) && changedSinceLastSend;
}

void IRReflectorModule::handleMessage(uint8_t *data, uint16_t len){

}

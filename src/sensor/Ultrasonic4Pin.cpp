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

#include <sensor/Ultrasonic4Pin.hpp>
#include <Conversions.hpp>

Ultrasonic4Pin::Ultrasonic4Pin(uint8_t triggerPin, uint8_t echoPin) : ArduinoDevice(2), 
        triggerPin(triggerPin), echoPin(echoPin){
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Don't need to send data frequently for this sensor
    sendRateMs = 150;
}

Ultrasonic4Pin::Ultrasonic4Pin(uint8_t *data, uint16_t len) : ArduinoDevice(2){
    if(data[0]){
        triggerPin = analogInputToDigitalPin(data[1]);
    }else{
        triggerPin = data[1];
    }
    if(data[2]){
        echoPin = analogInputToDigitalPin(data[3]);
    }else{
        echoPin = data[3];
    }
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Don't need to send data frequently for this sensor
    sendRateMs = 150;
}

uint16_t Ultrasonic4Pin::getSendData(uint8_t *data){
    Conversions::convertInt16ToData(distance, data, true);
    return 2;
}

bool Ultrasonic4Pin::service(RPiInterface *rpi){
    // Don't service this sensor unless data is to be sent. Servicing it is time-expensive.
    // Servicing this sensor polls for a pulse to come back, preventing servicing of other sensors.
    if(millis() >= nextSendTime){
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPin, LOW);

        // Wait at most 5ms for pulse to return.
        // Waiting too long for pulse can affect other sensors, such as IMUs, doing accumulation of rates
        // This limits max range to about 85cm
        uint16_t duration = pulseIn(echoPin, HIGH, 5000);

        if(duration > 0){
            distance = duration * 0.0343 / 2;
        }else{
            distance = 999;
        }
        
        return true;
    }
    return false;
}

void Ultrasonic4Pin::handleMessage(uint8_t *data, uint16_t len){

}
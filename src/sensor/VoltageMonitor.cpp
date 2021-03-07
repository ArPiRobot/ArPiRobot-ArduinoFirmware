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

#include <sensor/VoltageMonitor.hpp>
#include <Conversions.hpp>

VoltageMonitor::VoltageMonitor(uint8_t analogPin, float vboard, uint32_t r1, uint32_t r2) : 
        ArduinoDevice(5), analogPin(analogPin){  
    
    pinMode(analogPin, INPUT);
    readingScaleFactor = vboard * (r1 + r2) / r2 / 1023 / AVG_READINGS;

    // Zero the samples buffer
    for(uint8_t i = 0; i < AVG_READINGS; ++i){
        readings[i] = 0;
    }

    // Don't need to send data frequently for this sensor
    sendRateMs = 150;
}

VoltageMonitor::VoltageMonitor(uint8_t *data, uint16_t len) : ArduinoDevice(5){

    analogPin = data[0];
    float vboard = Conversions::convertDataToFloat(&data[1], false);
    uint32_t r1 = Conversions::convertDataToInt32(&data[5], false);
    uint32_t r2 = Conversions::convertDataToInt32(&data[9], false);

    pinMode(analogPin, INPUT);
    readingScaleFactor = vboard * (r1 + r2) / r2 / 1023 / AVG_READINGS;

    // Zero the samples buffer
    for(uint8_t i = 0; i < AVG_READINGS; ++i){
        readings[i] = 0;
    }

    // Don't need to send data frequently for this sensor
    sendRateMs = 150;
}

uint16_t VoltageMonitor::getSendData(uint8_t *data){
    // Buffer voltage little endian
    Conversions::convertFloatToData(voltage, &data[0], true);
    return 4;
}

bool VoltageMonitor::service(){
    readingRunningSum -= readings[readingIndex];
    readings[readingIndex] = analogRead(analogPin);
    readingRunningSum += readings[readingIndex];
    readingIndex++;
    if(readingIndex >= AVG_READINGS) {
        readingIndex = 0;
    }

    voltage = readingRunningSum * readingScaleFactor;

    return millis() >= nextSendTime;
}

void VoltageMonitor::handleMessage(uint8_t *data, uint16_t len){

}

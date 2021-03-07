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

#include <sensor/NxpAdafruit9Dof.hpp>
#include <Conversions.hpp>

bool NxpAdafruit9Dof::locked = false;

NxpAdafruit9Dof::NxpAdafruit9Dof() : ArduinoDevice(24){
    if(locked)
        return;

    locked = true;

    bool success = accel.begin(FXOS8700::AccelRange::RANGE_2G) && gyro.begin(FXAS2100::GyroRange::RANGE_500DPS);
    if(!success){
        locked = false;
    }

    valid = true;
}

NxpAdafruit9Dof::NxpAdafruit9Dof(uint8_t *data, uint16_t len) : ArduinoDevice(24){

    // No arguments passed when creating this device so data is ignored

    if(locked)
        return;

    locked = true;

    bool success = accel.begin() && gyro.begin();
    if(!success){
        locked = false;
    }

    valid = true;
}

uint16_t NxpAdafruit9Dof::getSendData(uint8_t *data){
    // This will never be called if not valid because service returns false
    Conversions::convertFloatToData(gx, &data[0], true);
    Conversions::convertFloatToData(gy, &data[4], true);
    Conversions::convertFloatToData(gz, &data[8], true);
    Conversions::convertFloatToData(ax, &data[12], true);
    Conversions::convertFloatToData(ay, &data[16], true);
    Conversions::convertFloatToData(az, &data[20], true);
    return 24;
}

bool NxpAdafruit9Dof::service(){
    if(!valid) return false;

    unsigned long now = micros();
    double dt = (now - lastSample) / 1e6;
    lastSample = now;

    if(dt < 0){
        // micros() rolled over. Some data has been lost...
        return false;
    }

    // m/s^2
    auto a = accel.getAccel();
    ax = a.x;
    ay = a.y;
    az = a.z;

    // deg / sec
    auto g = gyro.getRates();

    // Deg (accumulated)
    gx += g.x * dt;
    gy += g.y * dt;
    gz += g.z * dt;

    return millis() >= nextSendTime;
}

void NxpAdafruit9Dof::handleMessage(uint8_t *data, uint16_t len){
    if(!valid) return;
}

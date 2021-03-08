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

#include <sensor/Mpu6050Imu.hpp>
#include <Conversions.hpp>

bool Mpu6050Imu::locked = false;

Mpu6050Imu::Mpu6050Imu() : ArduinoDevice(24){
    if(locked)
        return;

    locked = true;

    bool success = imu.begin(MPU6050::AccelRange::RANGE_2G, MPU6050::GyroRange::RANGE_500DPS);
    if(!success){
        locked = false;
        return;
    }

    valid = true;
}

Mpu6050Imu::Mpu6050Imu(uint8_t *data, uint16_t len) : ArduinoDevice(24){

    // No arguments passed when creating this device so data is ignored

    if(locked)
        return;

    locked = true;

    bool success = imu.begin(MPU6050::AccelRange::RANGE_2G, MPU6050::GyroRange::RANGE_500DPS);
    if(!success){
        locked = false;
        return;
    }

    valid = true;
}

uint16_t Mpu6050Imu::getSendData(uint8_t *data){
    // This will never be called if not valid because service returns false
    Conversions::convertFloatToData(gx, &data[0], true);
    Conversions::convertFloatToData(gy, &data[4], true);
    Conversions::convertFloatToData(gz, &data[8], true);
    Conversions::convertFloatToData(ax, &data[12], true);
    Conversions::convertFloatToData(ay, &data[16], true);
    Conversions::convertFloatToData(az, &data[20], true);
    return 24;
}

bool Mpu6050Imu::service(){
    if(!valid) return false;

    unsigned long now = micros();
    double dt = (now - lastSample) / 1e6;
    lastSample = now;

    if(dt < 0){
        // micros() rolled over. Some data has been lost...
        return false;
    }

    // m/s^2
    auto data = imu.getData();
    ax = data.accelX - axCal;
    ay = data.accelY - ayCal;
    az = data.accelZ - azCal;

    // Deg (accumulated)
    gx += (data.gyroX - gxCal) * dt;
    gy += (data.gyroY - gyCal) * dt;
    gz += (data.gyroZ - gzCal) * dt;

    return millis() >= nextSendTime;
}

void Mpu6050Imu::handleMessage(uint8_t *data, uint16_t len){
    if(!valid) return;

    // Only one valid message 'C', samples
    // 'C' = Calibrate, samples is 16 bit little endian # samples
    if(data[0] == 'C' && len >= 3){
        uint16_t samples = Conversions::convertDataToInt16(&data[1], true);
        calibrate(samples);
    }
}

void Mpu6050Imu::calibrate(uint16_t samples){
    // Calibration is performed by taking given number of samples 1ms apart each
    // Values for each are averaged and subtracted from accepted values for each measurement
    // Calibration assumes device is stationary and gravity is along z axis (device is level)

    // Wait 100ms before starting calibration
    delay(100);

    gxCal = 0;
    gyCal = 0;
    gzCal = 0;
    axCal = 0;
    ayCal = 0;
    azCal = 0;

    for(uint16_t i = 0; i < samples; ++i){
        auto data = imu.getData();
        gxCal += data.gyroX;
        gyCal += data.gyroY;
        gzCal += data.gyroZ;
        axCal += data.accelX;
        ayCal += data.accelY;
        azCal += data.accelZ;
        delay(1);
    }

    gxCal /= samples;
    gyCal /= samples;
    gzCal /= samples;
    axCal /= samples;
    ayCal /= samples;
    azCal /= samples;

    // When stationary Z axis reads -g not zero
    azCal -= 9.80665;

    // The cal variables now contain values that should be subtracted from each value read from imu

}

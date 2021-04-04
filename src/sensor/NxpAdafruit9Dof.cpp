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
#include <settings.h>
#include <I2CHelper.hpp>

bool NxpAdafruit9Dof::locked = false;

NxpAdafruit9Dof::NxpAdafruit9Dof() : ArduinoDevice(24){
    if(locked)
        return;
    locked = true;
    valid = initSensors();
}

NxpAdafruit9Dof::NxpAdafruit9Dof(uint8_t *data, uint16_t len) : ArduinoDevice(24){
    // No arguments passed when creating this device so data is ignored
    if(locked)
        return;
    locked = true;
    valid = initSensors();
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
    unsigned long dt = now - lastSample;
    lastSample = now;

    // Have to ignore the first sample b/c lastSample is an arbitrary 0
    // This means dt is unknown
    // Cannot just use lastSample==0 bc micros can roll over and be zero at other times
    if(startup){
        startup = false; // Have the first sample so dt is valid next time
        return false;
    }else{
        // m/s^2
        auto a = getAccelData();
        ax = a.x - axCal;
        ay = a.y - ayCal;
        az = a.z - azCal;

        // deg / sec
        auto g = getGyroData();

        // Deg (accumulated)
        gx += (g.x - gxCal) * (dt / 1e6f);
        gy += (g.y - gyCal) * (dt / 1e6f);
        gz += (g.z - gzCal) * (dt / 1e6f);

        return (millis() - lastSendTime) >= sendRateMs;
    }
}

void NxpAdafruit9Dof::handleMessage(uint8_t *data, uint16_t len){
    if(!valid) return;

    // Only one valid message 'C', samples
    // 'C' = Calibrate, samples is 16 bit little endian # samples
    if(data[0] == 'C' && len >= 3){
        uint16_t samples = Conversions::convertDataToInt16(&data[1], true);
        calibrate(samples);
    }
}

void NxpAdafruit9Dof::calibrate(uint16_t samples){
    if(!valid) return;

    // Calibration is performed by taking given number of samples 1ms apart each
    // Values for each are averaged and subtracted from accepted values for each measurement
    // Calibration assumes device is stationary and gravity is along z axis (device is level)

    // Wait 500ms before starting calibration
    delay(500);

    gxCal = 0;
    gyCal = 0;
    gzCal = 0;
    axCal = 0;
    ayCal = 0;
    azCal = 0;

    for(uint16_t i = 0; i < samples; ++i){
        auto a = getAccelData();
        auto g = getGyroData();
        gxCal += g.x;
        gyCal += g.y;
        gzCal += g.z;
        axCal += a.x;
        ayCal += a.y;
        azCal += (a.z - 9.80665f);  // Z axis reads +g when IMU flat
        delay(1);
    }

    gxCal /= samples;
    gyCal /= samples;
    gzCal /= samples;
    axCal /= samples;
    ayCal /= samples;
    azCal /= samples;
}

bool NxpAdafruit9Dof::initSensors(){

    Wire.begin();

    ////////////////////////////////////////////////////////////////////////////
    /// FXOS8700 (accelerometer) setup
    ////////////////////////////////////////////////////////////////////////////
    // Verify correct device
    int16_t id = I2CHelper::readByte(Wire, FXOS8700_ADDRESS, FXOS8700_REGISTER_WHO_AM_I);
    if(id != FXOS8700_ID)
        return false;

    // Put in standby mode before configuring
    I2CHelper::writeByte(Wire, FXOS8700_ADDRESS, FXOS8700_REGISTER_CTRL_REG1, 0x00);

    // Set range
    // 0x00 = +/- 2G
    // 0x01 = +/- 4G
    // 0x02 = +/- 8G
    I2CHelper::writeByte(Wire, FXOS8700_ADDRESS, FXOS8700_REGISTER_XYZ_DATA_CFG, 0x00);

    // High resolution mode
    I2CHelper::writeByte(Wire, FXOS8700_ADDRESS, FXOS8700_REGISTER_CTRL_REG2, 0x02);

    // Enabled, normal mode, 100Hz
    I2CHelper::writeByte(Wire, FXOS8700_ADDRESS, FXOS8700_REGISTER_CTRL_REG1, 0x15);

    ////////////////////////////////////////////////////////////////////////////
    /// FXAS2100 (gyro) setup
    ////////////////////////////////////////////////////////////////////////////
    // Verify correct device
    id = I2CHelper::readByte(Wire, FXAS21002C_ADDRESS, GYRO_REGISTER_WHO_AM_I);
    if(id != FXAS21002C_ID)
        return false;

    // Switch to standby mode
    I2CHelper::writeByte(Wire, FXAS21002C_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x00);

    // Reset
    I2CHelper::writeByte(Wire, FXAS21002C_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x40);

    // Set sensitivity
    // 250DPS = 0x03
    // 500DPS = 0x02
    // 1000DPS = 0x01
    // 2000DPS = 0x00
    I2CHelper::writeByte(Wire, FXAS21002C_ADDRESS, GYRO_REGISTER_CTRL_REG0, 0x02);

    // Active mode 100Hz
    I2CHelper::writeByte(Wire, FXAS21002C_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x0E);

    return true;
}

NxpAdafruit9Dof::Data NxpAdafruit9Dof::getGyroData(){
    Data data;
    I2CHelper::write(Wire, FXAS21002C_ADDRESS, GYRO_REGISTER_OUT_X_MSB | 0x80);

    uint8_t rawData[6];
    if(I2CHelper::readBytes(Wire, FXAS21002C_ADDRESS, rawData, 6) != 6){
        return data;
    }

    data.x = (int16_t)(rawData[0] | (rawData[1] << 8));
    data.y = (int16_t)(rawData[2] | (rawData[3] << 8));
    data.z = (int16_t)(rawData[4] | (rawData[5] << 8));

    // Conversion to deg/sec depends on range
    // 250DPS = 0.0078125f
    // 500DPS = 0.015625f
    // 1000DPS = 0.03125f
    // 2000DPS = 0.0625f
    data.x *= 0.015625f;
    data.y *= 0.015625f;
    data.z *= 0.015625f;

    return data;
}

NxpAdafruit9Dof::Data NxpAdafruit9Dof::getAccelData(){
    Data data;
    I2CHelper::write(Wire, FXOS8700_ADDRESS, FXOS8700_REGISTER_OUT_X_MSB | 0x80);

    uint8_t rawData[6];
    if(I2CHelper::readBytes(Wire, FXOS8700_ADDRESS, rawData, 6) != 6){
        return data;
    }

    // Right shift b/c data is left aligned and only 14 bits wide
    data.x = (int16_t)((rawData[1] << 8) | rawData[0]) >> 2;
    data.y = (int16_t)((rawData[3] << 8) | rawData[2]) >> 2;
    data.z = (int16_t)((rawData[5] << 8) | rawData[4]) >> 2;

    // Scale values depend both on mode and range.
    // Sensor is in high resolution mode so:
    // +/- 2G:  0.000244f
    // +/- 4G:  0.000488f
    // +/- 8G:  0.000976f
    data.x *= 0.000244f * 9.80665f;
    data.y *= 0.000244f * 9.80665f;
    data.z *= 0.000244f * 9.80665f;

    return data;
}

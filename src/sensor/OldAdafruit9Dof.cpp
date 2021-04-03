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

#include <sensor/OldAdafruit9Dof.hpp>
#include <Conversions.hpp>
#include <settings.h>
#include <I2CHelper.hpp>

bool OldAdafruit9Dof::locked = false;

OldAdafruit9Dof::OldAdafruit9Dof() : ArduinoDevice(24){
    if(locked)
        return;
    locked = true;
    valid = initSensors();
}

OldAdafruit9Dof::OldAdafruit9Dof(uint8_t *data, uint16_t len) : ArduinoDevice(24){
    // No arguments passed when creating this device so data is ignored
    if(locked)
        return;
    locked = true;
    valid = initSensors();
}

uint16_t OldAdafruit9Dof::getSendData(uint8_t *data){
    // This will never be called if not valid because service returns false
    Conversions::convertFloatToData(gx, &data[0], true);
    Conversions::convertFloatToData(gy, &data[4], true);
    Conversions::convertFloatToData(gz, &data[8], true);
    Conversions::convertFloatToData(ax, &data[12], true);
    Conversions::convertFloatToData(ay, &data[16], true);
    Conversions::convertFloatToData(az, &data[20], true);
    return 24;
}

bool OldAdafruit9Dof::service(){
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

void OldAdafruit9Dof::handleMessage(uint8_t *data, uint16_t len){
    if(!valid) return;

    // Only one valid message 'C', samples
    // 'C' = Calibrate, samples is 16 bit little endian # samples
    if(data[0] == 'C' && len >= 3){
        uint16_t samples = Conversions::convertDataToInt16(&data[1], true);
        calibrate(samples);
    }
}

void OldAdafruit9Dof::calibrate(uint16_t samples){
    if(!valid) return;

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

bool OldAdafruit9Dof::initSensors(){

    Wire.begin();

    ////////////////////////////////////////////////////////////////////////////
    /// LSM303 (accelerometer) setup
    ////////////////////////////////////////////////////////////////////////////
    // Verify correct device
    int16_t id = I2CHelper::readByte(Wire, LSM303_ADDRESS, LSM303_REGISTER_ACCEL_WHO_AM_I);
    if(id != 0x33)
        return false;

    // Enable accelerometer at 100Hz
    if(I2CHelper::writeByte(Wire, LSM303_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57) != 0)
        return false;

    // Set range
    // 0x00 = +/- 2G
    // 0x01 = +/- 4G
    // 0x02 = +/- 8G
    // 0x03 = +/- 16G
    uint8_t c4val = I2CHelper::readByte(Wire, LSM303_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    c4val = I2CHelper::replaceBits(c4val, 0x00, 2, 4);
    I2CHelper::writeByte(Wire, LSM303_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG4_A, c4val);

    // Set to high resolution mode  (12-bit values)
    c4val = I2CHelper::readByte(Wire, LSM303_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG4_A);
    uint8_t c1val = I2CHelper::readByte(Wire, LSM303_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    c4val = I2CHelper::replaceBits(c4val, 0x01, 1, 3);
    c1val = I2CHelper::replaceBits(c1val, 0x00, 1, 3);
    I2CHelper::writeByte(Wire, LSM303_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG4_A, c4val);
    I2CHelper::writeByte(Wire, LSM303_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG1_A, c1val);

    ////////////////////////////////////////////////////////////////////////////
    /// L2GD20 (gyro) setup
    ////////////////////////////////////////////////////////////////////////////
    // Verify correct device
    id = I2CHelper::readByte(Wire, L3GD20_ADDRESS, GYRO_REGISTER_WHO_AM_I);
    if(id != 0xD4 && id != 0xD7){
        return false;
    }

    // Reset
    I2CHelper::writeByte(Wire, L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x00);

    // Switch to normal mode (enable X, Y, Z axes)
    I2CHelper::writeByte(Wire, L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x0F);
   
    // Set range
    // 0x00 = 250DPS
    // 0x10 = 500DPS
    // 0x20 = 2000DPS
    I2CHelper::writeByte(Wire, L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x10);

    return true;
}

OldAdafruit9Dof::Data OldAdafruit9Dof::getGyroData(){
    Data data;
    I2CHelper::write(Wire, L3GD20_ADDRESS, GYRO_REGISTER_OUT_X_L | 0x80);

    uint8_t rawData[6];
    if(I2CHelper::readBytes(Wire, L3GD20_ADDRESS, rawData, 6) != 6){
        return data;
    }
    int16_t x = (int16_t)(rawData[0] | (rawData[1] << 8));
    int16_t y = (int16_t)(rawData[2] | (rawData[3] << 8));
    int16_t z = (int16_t)(rawData[4] | (rawData[5] << 8));

    // Conversion to deg/sec depends on range
    // 250DPS:  0.00875f
    // 500DPS:  0.0175f
    // 2000DPS: 0.070f
    data.x = x * 0.0175f;
    data.y = y * 0.0175f;
    data.z = z * 0.0175f;

    return data;
}

OldAdafruit9Dof::Data OldAdafruit9Dof::getAccelData(){
    Data data;
    I2CHelper::write(Wire, LSM303_ADDRESS, LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);

    uint8_t rawData[6];
    if(I2CHelper::readBytes(Wire, LSM303_ADDRESS, rawData, 6) != 6){
        return data;
    }

    // Right shift 4 b/c 12-bit number left aligned (sensor is in high resolution mode)
    data.x = (int16_t)((rawData[0] | (rawData[1] << 8)) >> 4);
    data.y = (int16_t)((rawData[2] | (rawData[3] << 8)) >> 4);
    data.z = (int16_t)((rawData[4] | (rawData[5] << 8)) >> 4);

    // Scale values depend both on mode and range.
    // Sensor is in high resolution mode so:
    // +/- 2G:  0.00098f
    // +/- 4G:  0.00195f
    // +/- 8G:  0.0039f
    // +/- 16G: 0.01172f
    data.x *= 0.00098f * 9.80665f;
    data.y *= 0.00098f * 9.80665f;
    data.z *= 0.00098f * 9.80665f;

    return data;
}

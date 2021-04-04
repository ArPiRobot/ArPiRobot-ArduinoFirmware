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
#include <settings.h>
#include <I2CHelper.hpp>

bool Mpu6050Imu::locked = false;

Mpu6050Imu::Mpu6050Imu() : ArduinoDevice(24){
    if(locked)
        return;
    locked = true;
    valid = initSensors();
}

Mpu6050Imu::Mpu6050Imu(uint8_t *data, uint16_t len) : ArduinoDevice(24){
    // No arguments passed when creating this device so data is ignored
    if(locked)
        return;
    locked = true;
    valid = initSensors();
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

bool Mpu6050Imu::initSensors(){

    Wire.begin();

    // Verify correct device
    uint8_t id = I2CHelper::readByte(Wire, MPU6050_I2CADDR, MPU6050_WHO_AM_I);
    if(id != MPU6050_DEVICE_ID){
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Reset MPU6050
    ////////////////////////////////////////////////////////////////////////////
    uint8_t pm1val = I2CHelper::readByte(Wire, MPU6050_I2CADDR, MPU6050_PWR_MGMT_1);
    pm1val = I2CHelper::replaceBits(pm1val, 1, 1, 7); // Set reset bit to 1
    I2CHelper::writeByte(Wire, MPU6050_I2CADDR, MPU6050_PWR_MGMT_1, pm1val);

    // Wait for reset to finish
    while(I2CHelper::getBits(I2CHelper::readByte(Wire, MPU6050_I2CADDR, MPU6050_PWR_MGMT_1), 1, 7) == 1){
        delay(1);
    }
    delay(100);

    I2CHelper::writeByte(Wire, MPU6050_I2CADDR, MPU6050_SIGNAL_PATH_RESET, 0x07);
    delay(100);

    ////////////////////////////////////////////////////////////////////////////
    /// Set sample rate divider = 0
    ////////////////////////////////////////////////////////////////////////////
    I2CHelper::writeByte(Wire, MPU6050_I2CADDR, MPU6050_SMPLRT_DIV, 0x00);

    ////////////////////////////////////////////////////////////////////////////
    /// Disable low pass filter
    ////////////////////////////////////////////////////////////////////////////
    uint8_t configval = I2CHelper::readByte(Wire, MPU6050_I2CADDR, MPU6050_CONFIG);
    configval = I2CHelper::replaceBits(configval, 0, 3, 0);
    I2CHelper::writeByte(Wire, MPU6050_I2CADDR, MPU6050_CONFIG, configval);

    ////////////////////////////////////////////////////////////////////////////
    /// Set clk = pll w/ gyro x ref
    ////////////////////////////////////////////////////////////////////////////
    I2CHelper::writeByte(Wire, MPU6050_I2CADDR, MPU6050_PWR_MGMT_1, 0x01);
    delay(100);

    ////////////////////////////////////////////////////////////////////////////
    /// Set gyro range
    ////////////////////////////////////////////////////////////////////////////
    // 250DPS = 0x00
    // 500DPS = 0x01
    // 1000DPS = 0x02
    // 2000DPS = 0x03
    uint8_t gconfigval = I2CHelper::readByte(Wire, MPU6050_I2CADDR, MPU6050_GYRO_CONFIG);
    gconfigval = I2CHelper::replaceBits(gconfigval, 0x01, 2, 3);
    I2CHelper::writeByte(Wire, MPU6050_I2CADDR, MPU6050_GYRO_CONFIG, gconfigval);

    ////////////////////////////////////////////////////////////////////////////
    /// Set accel range
    ////////////////////////////////////////////////////////////////////////////
    // +/- 2G = 0x00
    // +/- 4G = 0x01
    // +/- 8G = 0x02
    // +/- 16G = 0x03
    uint8_t aconfigval = I2CHelper::readByte(Wire, MPU6050_I2CADDR, MPU6050_ACCEL_CONFIG);
    aconfigval = I2CHelper::replaceBits(aconfigval, 0x00, 2, 3);
    I2CHelper::writeByte(Wire, MPU6050_I2CADDR, MPU6050_ACCEL_CONFIG, aconfigval);

    return true;
}

Mpu6050Imu::Data Mpu6050Imu::getGyroData(){
    Data data;
    I2CHelper::write(Wire, MPU6050_I2CADDR, MPU6050_GYRO_OUT);

    uint8_t rawData[6];
    if(I2CHelper::readBytes(Wire, MPU6050_I2CADDR, rawData, 6) != 6){
        return data;
    }
    data.x = (int16_t)(rawData[1] | (rawData[0] << 8));
    data.y = (int16_t)(rawData[3] | (rawData[2] << 8));
    data.z = (int16_t)(rawData[5] | (rawData[4] << 8));

    // Conversion to deg/sec depends on range
    // 250DPS:  131.0f;
    // 500DPS:  65.5f;
    // 1000DPS: 32.8f;
    // 2000DPS: 16.4f;
    data.x /= 65.5f;
    data.y /= 65.5f;
    data.z /= 65.5f;

    return data;
}

Mpu6050Imu::Data Mpu6050Imu::getAccelData(){
    Data data;
    I2CHelper::write(Wire, MPU6050_I2CADDR, MPU6050_ACCEL_OUT);

    uint8_t rawData[6];
    if(I2CHelper::readBytes(Wire, MPU6050_I2CADDR, rawData, 6) != 6){
        return data;
    }

    data.x = (int16_t)((rawData[1] | (rawData[0] << 8)));
    data.y = (int16_t)((rawData[3] | (rawData[2] << 8)));
    data.z = (int16_t)((rawData[5] | (rawData[4] << 8)));

    // Scale values depend both on mode and range.
    // Sensor is in high resolution mode so:
    // +/- 2G:  16384.0f
    // +/- 4G:  8192.0f
    // +/- 8G:  4096.0f
    // +/- 16G: 2048.0f
    data.x /= 16384.0f;
    data.y /= 16384.0f;
    data.z /= 16384.0f;

    // Convert from g's to DPS
    data.x *= 9.80665f;
    data.y *= 9.80665f;
    data.z *= 9.80665f;

    return data;
}

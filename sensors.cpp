
#include "sensors.h"
#include "conversions.h"
#include "board.h"
#include "settings.h"
#include "interrupts.h"
#include "i2c.h"
#include <Wire.h>


////////////////////////////////////////////////////////////////////////////////
/// IRReflectorModule
////////////////////////////////////////////////////////////////////////////////
IRReflectorModule::IRReflectorModule(uint8_t digitalPin, uint8_t analogPin) : ArduinoDevice(3){
    init(digitalPin, analogPin);
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
    init(digitalPin, analogPin);
}

void IRReflectorModule::init(uint8_t digitalPin, uint8_t analogPin){
    this->digitalPin = digitalPin;
    this->analogPin = analogPin;
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


////////////////////////////////////////////////////////////////////////////////
/// Mpu6050Imu
////////////////////////////////////////////////////////////////////////////////
bool Mpu6050Imu::locked = false;

Mpu6050Imu::Mpu6050Imu() : ArduinoDevice(24){
    init();
}

Mpu6050Imu::Mpu6050Imu(uint8_t *data, uint16_t len) : ArduinoDevice(24){
    // No arguments passed when creating this device so data is ignored
    init();
}

void Mpu6050Imu::init(){
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


////////////////////////////////////////////////////////////////////////////////
/// NxpAdafruit9Dof
////////////////////////////////////////////////////////////////////////////////
bool NxpAdafruit9Dof::locked = false;

NxpAdafruit9Dof::NxpAdafruit9Dof() : ArduinoDevice(24){
    init();
}

NxpAdafruit9Dof::NxpAdafruit9Dof(uint8_t *data, uint16_t len) : ArduinoDevice(24){
    // No arguments passed when creating this device so data is ignored
    init();
}

void NxpAdafruit9Dof::init(){
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


////////////////////////////////////////////////////////////////////////////////
/// OldAdafruit9Dof
////////////////////////////////////////////////////////////////////////////////
bool OldAdafruit9Dof::locked = false;

OldAdafruit9Dof::OldAdafruit9Dof() : ArduinoDevice(24){
    init();
}

OldAdafruit9Dof::OldAdafruit9Dof(uint8_t *data, uint16_t len) : ArduinoDevice(24){
    // No arguments passed when creating this device so data is ignored
    init();
}

void OldAdafruit9Dof::init(){
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
    data.x = (int16_t)(rawData[0] | (rawData[1] << 8));
    data.y = (int16_t)(rawData[2] | (rawData[3] << 8));
    data.z = (int16_t)(rawData[4] | (rawData[5] << 8));

    // Conversion to deg/sec depends on range
    // 250DPS:  0.00875f
    // 500DPS:  0.0175f
    // 2000DPS: 0.070f
    data.x *= 0.0175f;
    data.y *= 0.0175f;
    data.z *= 0.0175f;

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


////////////////////////////////////////////////////////////////////////////////
/// SingleEncoder
////////////////////////////////////////////////////////////////////////////////
SingleEncoder::SingleEncoder(uint8_t pin, bool pullup) : ArduinoDevice(6){
    init(pin, pullup);
}

SingleEncoder::SingleEncoder(uint8_t *data, uint16_t len) : ArduinoDevice(6){
    if(data[0]){
        pin = analogInputToDigitalPin(data[1]);
    }else{
        pin = data[1];
    }
    init(pin, data[2]);
}

void SingleEncoder::init(uint8_t pin, bool pullup){
    this->pin = pin;
    // Check if the given pin is able to be used as an interrupt
    isInterrupt = Interrupts::isInterrupt(pin);

    // Configure as needed
    pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
    if(isInterrupt){
        Interrupts::enable(pin, CHANGE, &SingleEncoder::isr, (void*)this);
    }else{
        lastState = digitalRead(pin);
    }
}

SingleEncoder::~SingleEncoder(){
    if(isInterrupt){
        Interrupts::disable(pin);
    }
}

uint16_t SingleEncoder::getSendData(uint8_t *data){
    // Buffer count little endian
    Conversions::convertInt16ToData(count, &data[0], true);

    // Send data to the pi to be used to calculate speed
    Conversions::convertInt16ToData(count - lastSendCount, &data[2], true);
    Conversions::convertInt16ToData(millis() - lastSendTime, &data[4], true);
    lastSendCount = count;
    return 6;
}

bool SingleEncoder::service(){
    if(!isInterrupt){
        bool state = digitalRead(pin);
        if(state != lastState){
            count++;
            lastState = state;
        }
    }
    return (millis() - lastSendTime) >= sendRateMs;
}

void SingleEncoder::handleMessage(uint8_t *data, uint16_t len){
    
}

void SingleEncoder::isrMember(){
    count++;
}

void SingleEncoder::isr(void* userData){
    ((SingleEncoder*)userData)->isrMember();
}


////////////////////////////////////////////////////////////////////////////////
/// Ultrasonic4Pin
////////////////////////////////////////////////////////////////////////////////
Ultrasonic4Pin::Ultrasonic4Pin(uint8_t triggerPin, uint8_t echoPin) : ArduinoDevice(2){
    init(triggerPin, echoPin);
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
    init(triggerPin, echoPin);
}

void Ultrasonic4Pin::init(uint8_t triggerPin, uint8_t echoPin){
    this->triggerPin = triggerPin;
    this->echoPin = echoPin;

    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);

    usingInterrupt = Interrupts::isInterrupt(echoPin);
    needsPulse = true;

    if(usingInterrupt){
        Interrupts::enable(echoPin, CHANGE, &Ultrasonic4Pin::isr, (void*)this);
    }

    // Don't need to send data frequently for this sensor
    sendRateMs = 150;
}

Ultrasonic4Pin::~Ultrasonic4Pin(){
    if(usingInterrupt){
        Interrupts::disable(echoPin);
    }
}

uint16_t Ultrasonic4Pin::getSendData(uint8_t *data){
    Conversions::convertInt16ToData(distance, data, true);
    return 2;
}

bool Ultrasonic4Pin::service(){
    if(usingInterrupt && needsPulse){
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPin, LOW);
        needsPulse = false;
        distance = distance * 0.0343 / 2;
        return (millis() - lastSendTime) >= sendRateMs;
    }else if(!usingInterrupt && ((millis() - lastSendTime) >= sendRateMs)){
        // Don't service this sensor unless data is to be sent. Servicing it is time-expensive.
        // Servicing this sensor polls for a pulse to come back, preventing servicing of other sensors.
        
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

void Ultrasonic4Pin::isrMember(){
    if(digitalRead(echoPin)){
        // Rising edge of pulse
        startTime = micros();
    }else{
        // Falling edge of pulse
        // Reuse distance variable instead of a second "duration" variable
        distance = micros() - startTime;
        needsPulse = true;
    }
}

void Ultrasonic4Pin::isr(void* userData){
    ((Ultrasonic4Pin*)userData)->isrMember();
}


////////////////////////////////////////////////////////////////////////////////
/// VoltageMonitor
////////////////////////////////////////////////////////////////////////////////
VoltageMonitor::VoltageMonitor(uint8_t analogPin, float vboard, uint32_t r1, uint32_t r2) : ArduinoDevice(5){  
    init(analogPin, vboard, r1, r2);
}

VoltageMonitor::VoltageMonitor(uint8_t *data, uint16_t len) : ArduinoDevice(5){
    analogPin = data[0];
    float vboard = Conversions::convertDataToFloat(&data[1], false);
    uint32_t r1 = Conversions::convertDataToInt32(&data[5], false);
    uint32_t r2 = Conversions::convertDataToInt32(&data[9], false);

    init(analogPin, vboard, r1, r2);
}

void VoltageMonitor::init(uint8_t analogPin, float vboard, uint32_t r1, uint32_t r2){
    this->analogPin = analogPin;

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

    return (millis() - lastSendTime) >= sendRateMs;
}

void VoltageMonitor::handleMessage(uint8_t *data, uint16_t len){

}

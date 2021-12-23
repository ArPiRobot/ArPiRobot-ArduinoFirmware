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

#pragma once

#include <Arduino.h>
#include "board.h"
#include "device.h"


/**
 * IR Reflector module
 */
class IRReflectorModule : public ArduinoDevice{
public:
    /**
     * @param digitalPin Digital pin number to use
     * @param analogPin Analog pin number to use. Set to 255 to disable analog readings.
     */
    IRReflectorModule(uint8_t digitalPin, uint8_t analogPin);

    /**
     * Construct an IRReflectorModule from command data
     * Data format: [DANALOG][DIGITALPIN][ANALOGPIN]
     *      DANALOG: If 1 the digital pin number is the number of an analog pin (0 = A0, 1 = A1, etc)
     *      DIGITALPIN: Number of digital pin
     *      ANALOGPIN: Analog pin number (0 = A0, 1 = A1, etc)
     */
    IRReflectorModule(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

private:
    uint8_t digitalPin, analogPin;
    uint8_t lastDigitalState;
    uint16_t lastAnalogValue;
    bool changedSinceLastSend = true;
};


/**
 * MPU6050 IMU
 */
class Mpu6050Imu : public ArduinoDevice {
public:
    Mpu6050Imu();

    /**
     * Construct the IMU from command data
     * Data format: NULL
     */
    Mpu6050Imu(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

    void calibrate(uint16_t samples);

private:
    
    struct Data{
        float x = 0, y = 0, z = 0;
    };

    bool initSensors();

    Data getGyroData();

    Data getAccelData();

    bool valid = false;
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    float gxCal = 0, gyCal = 0, gzCal = 0;
    float axCal = 0, ayCal = 0, azCal = 0;
    unsigned long lastSample = 0;
    bool startup = true; // True means no samples yet. False means have taken a sample before.

    static bool locked;

    const static uint8_t MPU6050_I2CADDR = 0x68;
    const static uint8_t MPU6050_DEVICE_ID = 0x68;

    const static uint8_t MPU6050_SELF_TEST_X = 0x0D;
    const static uint8_t MPU6050_SELF_TEST_Y = 0x0E;
    const static uint8_t MPU6050_SELF_TEST_Z = 0x0F;
    const static uint8_t MPU6050_SELF_TEST_A = 0x10;
    const static uint8_t MPU6050_SMPLRT_DIV = 0x19 ;
    const static uint8_t MPU6050_CONFIG = 0x1A;
    const static uint8_t MPU6050_GYRO_CONFIG = 0x1B;
    const static uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
    const static uint8_t MPU6050_INT_PIN_CONFIG = 0x37;
    const static uint8_t MPU6050_WHO_AM_I = 0x75;
    const static uint8_t MPU6050_SIGNAL_PATH_RESET = 0x68;
    const static uint8_t MPU6050_USER_CTRL = 0x6A;
    const static uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
    const static uint8_t MPU6050_PWR_MGMT_2 = 0x6C;
    const static uint8_t MPU6050_TEMP_H = 0x41;
    const static uint8_t MPU6050_TEMP_L = 0x42;
    const static uint8_t MPU6050_ACCEL_OUT = 0x3B;
    const static uint8_t MPU6050_GYRO_OUT = 0x43;
};


/**
 * Nxp Adafruit 9DOF IMU FXAS2100 + FXOS8700
 */
class NxpAdafruit9Dof : public ArduinoDevice {
public:
    NxpAdafruit9Dof();

    /**
     * Construct the IMU from command data
     * Data format: NULL
     */
    NxpAdafruit9Dof(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

    void calibrate(uint16_t samples);

private:
    
    struct Data{
        float x = 0, y = 0, z = 0;
    };

    bool initSensors();

    Data getGyroData();

    Data getAccelData();

    bool valid = false;
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    float gxCal = 0, gyCal = 0, gzCal = 0;
    float axCal = 0, ayCal = 0, azCal = 0;
    unsigned long lastSample = 0;
    bool startup = true; // True means no samples yet. False means have taken a sample before.

    static bool locked;


    const static uint8_t FXAS21002C_ADDRESS  = 0x21;
    const static uint8_t FXAS21002C_ID = 0xD7;

    const static uint8_t GYRO_REGISTER_STATUS = 0x00;
    const static uint8_t GYRO_REGISTER_OUT_X_MSB = 0x01;
    const static uint8_t GYRO_REGISTER_OUT_X_LSB = 0x02;
    const static uint8_t GYRO_REGISTER_OUT_Y_MSB = 0x03;
    const static uint8_t GYRO_REGISTER_OUT_Y_LSB = 0x04;
    const static uint8_t GYRO_REGISTER_OUT_Z_MSB = 0x05;
    const static uint8_t GYRO_REGISTER_OUT_Z_LSB = 0x06;
    const static uint8_t GYRO_REGISTER_WHO_AM_I = 0x0C;
    const static uint8_t GYRO_REGISTER_CTRL_REG0 = 0x0D;
    const static uint8_t GYRO_REGISTER_CTRL_REG1 = 0x13;
    const static uint8_t GYRO_REGISTER_CTRL_REG2 = 0x14;

    const static uint8_t FXOS8700_ADDRESS = 0x1F;
    const static uint8_t FXOS8700_ID = 0xC7;
    const static uint8_t FXOS8700_REGISTER_STATUS = 0x00;
    const static uint8_t FXOS8700_REGISTER_OUT_X_MSB = 0x01;
    const static uint8_t FXOS8700_REGISTER_OUT_X_LSB = 0x02;
    const static uint8_t FXOS8700_REGISTER_OUT_Y_MSB = 0x03;
    const static uint8_t FXOS8700_REGISTER_OUT_Y_LSB = 0x04;
    const static uint8_t FXOS8700_REGISTER_OUT_Z_MSB = 0x05;
    const static uint8_t FXOS8700_REGISTER_OUT_Z_LSB = 0x06;
    const static uint8_t FXOS8700_REGISTER_WHO_AM_I = 0x0D;
    const static uint8_t FXOS8700_REGISTER_XYZ_DATA_CFG = 0x0E;
    const static uint8_t FXOS8700_REGISTER_CTRL_REG1 = 0x2A;
    const static uint8_t FXOS8700_REGISTER_CTRL_REG2 = 0x2B;
    const static uint8_t FXOS8700_REGISTER_CTRL_REG3 = 0x2C;
    const static uint8_t FXOS8700_REGISTER_CTRL_REG4 = 0x2D;
    const static uint8_t FXOS8700_REGISTER_CTRL_REG5 = 0x2E;
    
};


/**
 * Old Adafruit 9DOF (now discontinued) L3GD20 + LSM303
 */
class OldAdafruit9Dof : public ArduinoDevice {
public:
    OldAdafruit9Dof();

    /**
     * Construct the IMU from command data
     * Data format: NULL
     */
    OldAdafruit9Dof(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

    void calibrate(uint16_t samples);

private:
    
    struct Data{
        float x = 0, y = 0, z = 0;
    };

    bool initSensors();

    Data getGyroData();

    Data getAccelData();

    bool valid = false;
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    float gxCal = 0, gyCal = 0, gzCal = 0;
    float axCal = 0, ayCal = 0, azCal = 0;
    unsigned long lastSample = 0;
    bool startup = true; // True means no samples yet. False means have taken a sample before.

    static bool locked;

    const uint8_t LSM303_ADDRESS = 0x19;
    const uint8_t LSM303_REGISTER_ACCEL_WHO_AM_I = 0x0F;
    const uint8_t LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x20;
    const uint8_t LSM303_REGISTER_ACCEL_CTRL_REG4_A = 0x23;
    const uint8_t LSM303_REGISTER_ACCEL_OUT_X_L_A = 0x28;

    const static uint8_t L3GD20_ADDRESS = 0x6B;
    const static uint8_t GYRO_REGISTER_WHO_AM_I = 0x0F;
    const static uint8_t GYRO_REGISTER_CTRL_REG1 = 0x20;
    const static uint8_t GYRO_REGISTER_CTRL_REG4 = 0x23;
    const static uint8_t GYRO_REGISTER_OUT_X_L = 0x28;
};


/**
 * Single-channel encoder
 */
class SingleEncoder : public ArduinoDevice{
public:
    /**
     * @param pin Digital pin number to use. If this pin is able to be used as an interrupt it will be
     * @param pullup if true internal pullup will be used, if false it will not
     */
    SingleEncoder(uint8_t pin, bool pullup);

    /**
     * Construct a SingleEncoder from command data
     * Data format: [ANALOG][PIN][PULLUP]
     *      ANALOG: 1 = analog pin #, 0 = digital pin #
     *      PIN: Pin number (unsigned 8-bit int)
     *      PULLUP: 1 = use internal pullup resistor, 0 = do not
     */
    SingleEncoder(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

private:
    uint8_t pin;
    bool lastState;
    bool isInterrupt;
    uint16_t count = 0, lastSentCount = 65535;
};


/**
 * Ultrasonic sensor with 4 pins (Vcc, trigger, echo, GND)
 */
class Ultrasonic4Pin : public ArduinoDevice{
public:
    /**
     * @param triggerPin Digital pin number to use for trigger
     * @param echoPin Digital pin number to use for echo
     */
    Ultrasonic4Pin(uint8_t triggerPin, uint8_t echoPin);

    /**
     * Construct a Ultrasonic4Pin from command data
     * Data format: [TRIGANALOG][TRIGPIN][ECHOANALOG][ECHOPIN]
     *      TRIGANALOG: 1 = analog pin #, 0 = digital pin #
     *      TRIGPIN: Pin number (unsigned 8-bit int)
     *      ECHOANALOG: 1 = analog pin #, 0 = digital pin #
     *      ECHOPIN: 1 = use internal pullup resistor, 0 = do not
     */
    Ultrasonic4Pin(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

private:
    uint8_t triggerPin, echoPin;
    uint16_t distance = 0;
};


/**
 * Voltage divider voltage monitor encoder
 */
class VoltageMonitor : public ArduinoDevice{
public:
    /**
     * @param analogPin Analog pin to use
     * @param vboard Analog reference voltage (board voltage by default)
     * @param r1 The top resistor of the voltage divider
     * @param r2 The bottom resistor of the voltage divider (the one voltage is measured across)
     */
    VoltageMonitor(uint8_t analogPin, float vboard, uint32_t r1, uint32_t r2);

    /**
     * Construct a VoltageMonitor from command data
     * Data format: [ANALOG][PIN][PULLUP]
     *      ANALOG: 1 = analog pin #, 0 = digital pin #
     *      PIN: Pin number (unsigned 8-bit int)
     *      PULLUP: 1 = use internal pullup resistor, 0 = do not
     */
    VoltageMonitor(uint8_t *data, uint16_t len);

    uint16_t getSendData(uint8_t *data) override;

    bool service() override;

    void handleMessage(uint8_t *data, uint16_t len) override;

private:
    // Number of analog readings to average together to smooth readings a little
    const static uint8_t AVG_READINGS = 5;

    uint8_t analogPin;
    float readingScaleFactor;

    // Used to smooth voltage readings by averaging previous 10 values together
    uint16_t readings[AVG_READINGS];
    uint16_t readingRunningSum = 0;
    uint8_t readingIndex = 0;

    float voltage = 0;
};

/*
 * Copyright 2020 Marcus Behel
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

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
/// Interface and bandwidth settings
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
/// Debug Interface
///////////////////////////////////////////////////////
//#define DEBUG
#ifdef DEBUG
  #define DEBUG_SERIAL Serial1
  #define DEBUG_SERIAL_BAUD 9600
#endif

///////////////////////////////////////////////////////
/// Interface method
///////////////////////////////////////////////////////

#define INTERFACE_HW_SERIAL
//#define INTERFACE_TEENSY_USB_SERIAL
#define HW_SERIAL_PORT Serial
#define HW_SERIAL_BAUD 57600

//#define INTERFACE_SW_SERIAL
#define SW_SERIAL_TX 3
#define SW_SERIAL_RX 4
#define SW_SERIAL_BAUD 9600

///////////////////////////////////////////////////////
/// Other intereface settings
///////////////////////////////////////////////////////

// How often sensor data should be sent (at most, some sensors will not send each time if nothing has changed)
#define SEND_RATE 50  // ms

// How long to offset sensors send times from each other
#define OFFSET_STEP 5 // ms

// Maximum number of ArduinoDevices supported by a single interface (mem mgmt)
#define MAX_DEVICES 10

// Sensor data buffer size for an interface
#define DATA_WRITE_BUFFER_SIZE 64
#define DATA_READ_BUFFER_SIZE 64

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
/// Sensor specific settings
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// Adafruit NXP Precision 9DOF IMU (FXOS8700 + FXAS21002C)
///////////////////////////////////////////////////////

// Enable the IMU (third party libraries required)
// Note: This cannot coexist with the Old 9DOF IMU (only enable one)
#define NXPADA9DOF_ENABLE

// Calibration data for the IMU's gyroscope and accelerometer (Get from sensor calibration project)
#define NXPADA9DOF_GYRO_X_OFFSET 0
#define NXPADA9DOF_GYRO_Y_OFFSET 0
#define NXPADA9DOF_GYRO_Z_OFFSET 0
#define NXPADA9DOF_ACCEL_X_OFFSET 0
#define NXPADA9DOF_ACCEL_Y_OFFSET 0
#define NXPADA9DOF_ACCEL_Z_OFFSET 0

///////////////////////////////////////////////////////
// Old Adafruit 9DOF IMU (L3GD20H + LSM303)
///////////////////////////////////////////////////////

// Enable the IMU (third party libraries required)
// Note: This cannot coexist with the NXP 9DOF IMU (only enable one)
//#define OLDADA9DOF_ENABLE

// Calibration data for the IMU's gyroscope and accelerometer (Get from sensor calibration project)
#define OLDADA9DOF_GYRO_X_OFFSET  0
#define OLDADA9DOF_GYRO_Y_OFFSET  0
#define OLDADA9DOF_GYRO_Z_OFFSET  0
#define OLDADA9DOF_ACCEL_X_OFFSET 0
#define OLDADA9DOF_ACCEL_Y_OFFSET 0
#define OLDADA9DOF_ACCEL_Z_OFFSET 0

///////////////////////////////////////////////////////
// Voltage Monitor
///////////////////////////////////////////////////////
// Number of iterations between data sends for voltage monitor
#define VMON_MAX_SEND_RATE 10


///////////////////////////////////////////////////////
// 4-Pin Ultrasonic Sensor (HC-SR04)
///////////////////////////////////////////////////////

// Number of iterations between polls for 4-pin ultrasonic sensor (affects update rate)
#define ULTRASONIC_4PIN_POLL_RATE 3

///////////////////////////////////////////////////////
// IR Reflector Module (TCRT5000 for example)
///////////////////////////////////////////////////////

// How much the analog reading has to change before new data is sent
#define IR_REFLECTOR_ANALOG_CHANGE_THRESHOLD 16

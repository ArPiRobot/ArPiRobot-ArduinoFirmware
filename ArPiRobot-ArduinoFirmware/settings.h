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
#define HW_SERIAL_BAUD 500000

//#define INTERFACE_SW_SERIAL
#define SW_SERIAL_TX 3
#define SW_SERIAL_RX 4
#define SW_SERIAL_BAUD 9600

///////////////////////////////////////////////////////
/// Other intereface settings
///////////////////////////////////////////////////////

// How often sensor data should be sent
#define SEND_RATE 20  // ms

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
// Old Adafruit 9DOF IMU (L3GD20H + LSM303)
///////////////////////////////////////////////////////

// Enable the IMU (third party libraries required)
#define OLDADA9DOF_ENABLE

// Calibration data for the IMU's gyroscope (Get from sensor calibration project)
#define OLDADA9DOF_GYRO_X_OFFSET 0
#define OLDADA9DOF_GYRO_Y_OFFSET 0
#define OLDADA9DOF_GYRO_Z_OFFSET 0

///////////////////////////////////////////////////////
// 4-Pin Ultrasonic Sensor (HC-SR04)
///////////////////////////////////////////////////////

// Number of iterations between polls for 4-pin ultrasonic sensor (affects update rate)
#define ULTRASONIC_4PIN_POLL_RATE 3

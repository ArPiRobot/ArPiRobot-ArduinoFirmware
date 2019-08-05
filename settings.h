#pragma once

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
/// Interface and bandwidth settinsg
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
/// Interface method
///////////////////////////////////////////////////////

#define INTERFACE_HW_SERIAL
//#define INTERFACE_TEENSY_USB_SERIAL
#define HW_SERIAL_PORT Serial
#define HW_SERIAL_BAUD 250000

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
#define DATA_BUFFER_SIZE 96

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

// Enable pitch, roll, and yaw
#define OLDADA9DOF_ENABLE_AHRS

// Number of samples to take when calibrating IMU (-1 to disable automatic calibration)
#define OLDADA9DOF_CALIBRATE_SAMPLES -1

// Time to wait before calibrating IMU (ms)
#define OLDADA9DOF_CALIBRATE_DELAY 1000

// Time between samples for calibration (ms)
#define OLDADA9DOF_CALIBRATE_SPACING 30

// Pre-defined (manual) calibration for the IMU
#define OLDADA9DOF_X_OFFSET -0.031700856983661
#define OLDADA9DOF_Y_OFFSET 0.040051378309726
#define OLDADA9DOF_Z_OFFSET 0.003336852183565

///////////////////////////////////////////////////////
// 4-Pin Ultrasonic Sensor (HC-SR04)
///////////////////////////////////////////////////////

// Number of iterations between polls for 4-pin ultrasonic sensor
#define ULTRASONIC_4PIN_POLL_RATE 3

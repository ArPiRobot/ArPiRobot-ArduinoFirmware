#pragma once

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
/// Interface and bandwidth settinsg
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

// How often sensor data should be sent
#define SEND_RATE 20  // ms

// How long to offset sensors send times from each other
#define OFFSET_STEP 5 // ms

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

// Number of samples to take when calibrating IMU
#define OLDADA9DOF_CALIBRATE_SAMPLES 0

// Time to wait before calibrating IMU (ms)
#define OLDADA9DOF_CALIBRATE_DELAY 1000

// Time between samples for calibration (ms)
#define OLDADA9DOF_CALIBRATE_SPACING 30

// Pre-defined calibration for the IMU
#define OLDADA9DOF_X_OFFSET -0.031700856983661
#define OLDADA9DOF_Y_OFFSET 0.040051378309726
#define OLDADA9DOF_Z_OFFSET 0.003336852183565

///////////////////////////////////////////////////////
// 4-Pin Ultrasonic Sensor (HC-SR04)
///////////////////////////////////////////////////////

// Number of iterations between polls for 4-pin ultrasonic sensor
#define ULTRASONIC_4PIN_POLL_RATE 3

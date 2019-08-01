#pragma once

#define SEND_RATE 20  // ms
#define OFFSET_STEP 5 // ms

///////////////////////////////////////////////////////
/// Sensor specific settings
///////////////////////////////////////////////////////

// Old Adafruit 9DOF IMU Enable AHRS
#define OLDADA9DOF_ENABLE_AHRS
#define OLDADA9DOF_CALIBRATE_SAMPLES 10
#define OLDADA9DOF_X_OFFSET 0
#define OLDADA9DOF_Y_OFFSET 0
#define OLDADA9DOF_Z_OFFSET 0

// Number of iterations between polls for 4-pin ultrasonic sensor
#define ULTRASONIC_4PIN_POLL_RATE 3

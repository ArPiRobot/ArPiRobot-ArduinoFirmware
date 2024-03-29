#pragma once

////////////////////////////////////////////////////////////////////////////////
/// Interface settings
////////////////////////////////////////////////////////////////////////////////

// Size of the buffer to hold data read from the pi
#define DATA_READ_BUFFER_SIZE   64

// Maxiumum number of devices able to be instantiated by the pi
#define MAX_DEVICES             10

// Interfaces (uncomment only one)
#define IFACE_UART

// Interface settings (do not comment these out, only relevant options are used)
#define UART_PORT       Serial
#define UART_BAUD       57600    

////////////////////////////////////////////////////////////////////////////////
/// Flags to enable each supported sensor
///     Uncomment each definition below to enable the sensor
///     Comment each to disable the sensor
///     Sensors can be disabled to reduce firmware flash usage
////////////////////////////////////////////////////////////////////////////////

#define IRReflectorModule_ENABLE
#define Mpu6050Imu_ENABLE
#define NxpAdafruit9Dof_ENABLE
#define OldAdafruit9Dof_ENABLE
#define SingleEncoder_ENABLE
#define QuadEncoder_ENABLE
#define Ultrasonic4Pin_ENABLE
#define VoltageMonitor_ENABLE


////////////////////////////////////////////////////////////////////////////////
// Other Settings
////////////////////////////////////////////////////////////////////////////////
// Period (ms) that sensor data should be sent at by default.
// Some sensors may change this
#define DEFAULT_SEND_RATE       20


////////////////////////////////////////////////////////////////////////////////
/// Debug settings
////////////////////////////////////////////////////////////////////////////////

// #define ARPFW_DEBUG

#ifdef ARPFW_DEBUG

#include <SoftwareSerial.h>
#include <Arduino.h>

extern SoftwareSerial debugSer;

#define LOG(msg) debugSer.print(msg)
#define LOGLN(msg) debugSer.println(msg)
#define DBG_INIT() debugSer.begin(9600);

#else

#define LOG(msg)
#define LOGLN(msg)
#define DBG_INIT()

#endif

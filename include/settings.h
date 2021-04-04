#pragma once

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
#define Ultrasonic4Pin_ENABLE
#define VoltageMonitor_ENABLE


////////////////////////////////////////////////////////////////////////////////
/// Debug settings
////////////////////////////////////////////////////////////////////////////////

#define ARPFW_DEBUG

#ifdef ARPFW_DEBUG

#include <SoftwareSerial.h>
#include <Arduino.h>

//extern SoftwareSerial debugSer;

extern HardwareSerial &debugSer;

#define LOG(msg) debugSer.print(msg)
#define LOGLN(msg) debugSer.println(msg)
#define DBG_INIT() debugSer.begin(9600);

#else

#define LOG(msg)
#define LOGLN(msg)
#define DBG_INIT()

#endif

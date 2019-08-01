#pragma once

#include "settings.h"
#include "Arduino.h"
#include <stdint.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

union floatAsBytes {
    float fval;
    byte bval[4];
};

class ArduinoDevice {
public:
	ArduinoDevice();

	virtual bool poll(uint8_t *buffer, uint8_t *count) = 0;

	static uint8_t sendTimeOffset;
	unsigned long nextSendTime = 0;
  uint8_t deviceId = 0;
  static uint8_t nextId;
};

class SingleEncoder : public ArduinoDevice {
public:
  SingleEncoder(int pin);

  bool poll(uint8_t *buffer, uint8_t *count) override;
  
  uint8_t lastState = 0;
  uint8_t pin;
  uint16_t count = 0;
};

class Ultrasonic4Pin : public ArduinoDevice {
public:
  Ultrasonic4Pin(int triggerPin, int echoPin);

  bool poll(uint8_t *buffer, uint8_t *count) override;

  uint8_t triggerPin, echoPin;
  uint16_t distance;
  bool waitingForPulse = false;

  uint8_t pollIterationCounter = 0;
};

class OldAdafruit9Dof : public ArduinoDevice{
public:
  OldAdafruit9Dof();
  ~OldAdafruit9Dof();

  bool poll(uint8_t *buffer, uint8_t *count) override;

private:
  static bool locked;

  unsigned long lastSample = 0;
  
  // Sensors
  Adafruit_LSM303_Accel_Unified *accel = NULL;
  Adafruit_LSM303_Mag_Unified *mag = NULL;
  Adafruit_L3GD20_Unified *gyro = NULL;
  Adafruit_9DOF *dof = NULL;

  // Calculated values
  floatAsBytes gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, pitch, roll, yaw;

  float gyro_x_calib = 0, gyro_y_calib = 0, gyro_z_calib = 0;
};

class VoltageMonitor : public ArduinoDevice{
public:
  VoltageMonitor(uint8_t readPin, float vboard, uint32_t r1, uint32_t r2);

  bool poll(uint8_t *buffer, uint8_t *count) override;

private:

  uint8_t readPin;
  float vboard;
  uint32_t r1, r2;

  floatAsBytes voltage;
  floatAsBytes lastSentVoltage; // So difference is always large enough to send

  const static size_t SAMPLE_COUNT = 20;
  float samples[SAMPLE_COUNT];
  int currentSample = -1;
};

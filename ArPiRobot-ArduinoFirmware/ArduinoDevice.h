#pragma once

#include "settings.h"
#include "Arduino.h"
#include <stdint.h>

union floatAsBytes {
    float fval;
    byte bval[4];
};

class ArduinoDevice {
public:
	ArduinoDevice();
  virtual ~ArduinoDevice();

	virtual bool poll(uint8_t *buffer, uint8_t *count) = 0;

  virtual void handleData(uint8_t *data, uint8_t len) = 0;

  void assignDeviceId(uint8_t deviceId);
  
	static uint8_t sendTimeOffset;
	unsigned long nextSendTime = 0;
  uint8_t deviceId = 0;
};

class SingleEncoder : public ArduinoDevice {
public:
  SingleEncoder(int pin);

  bool poll(uint8_t *buffer, uint8_t *count) override;
  void handleData(uint8_t *data, uint8_t len) override;
  
  uint8_t lastState = 0;
  uint8_t pin;
  bool changed;
  uint16_t count = 0;
};

class Ultrasonic4Pin : public ArduinoDevice {
public:
  Ultrasonic4Pin(int triggerPin, int echoPin);

  bool poll(uint8_t *buffer, uint8_t *count) override;
  void handleData(uint8_t *data, uint8_t len) override;

  uint8_t triggerPin, echoPin;
  uint16_t distance;
  bool waitingForPulse = false;

  uint8_t pollIterationCounter = 0;
};

#ifdef OLDADA9DOF_ENABLE

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

class OldAdafruit9Dof : public ArduinoDevice{
public:
  OldAdafruit9Dof();
  ~OldAdafruit9Dof();

  bool poll(uint8_t *buffer, uint8_t *count) override;
  void handleData(uint8_t *data, uint8_t len) override;

private:
  static bool locked;

  unsigned long lastSample = 0;
  
  // Sensors
  Adafruit_LSM303_Accel_Unified *accel = NULL;
  Adafruit_L3GD20_Unified *gyro = NULL;

  // Calculated values
  floatAsBytes gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
  sensors_event_t accel_event, gyro_event;
};

#endif // OLDADA9DOF_ENABLE

class VoltageMonitor : public ArduinoDevice{
public:
  VoltageMonitor(uint8_t readPin, float vboard, uint32_t r1, uint32_t r2);

  bool poll(uint8_t *buffer, uint8_t *count) override;
  void handleData(uint8_t *data, uint8_t len) override;

private:
  uint8_t readPin;
  float vboard;
  uint32_t r1, r2;

  floatAsBytes voltage;
};

#ifdef NXPADA9DOF_ENABLE

#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

class NxpAdafruit9Dof : public ArduinoDevice{
public:
  NxpAdafruit9Dof();
  ~NxpAdafruit9Dof();

  bool poll(uint8_t *buffer, uint8_t *count) override;
  void handleData(uint8_t *data, uint8_t len) override;

private:
  static bool locked;

  unsigned long lastSample = 0;
  
  // Sensors
  Adafruit_FXOS8700 *accelmag = NULL;
  Adafruit_FXAS21002C *gyro = NULL;

  // Calculated values
  floatAsBytes gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
  sensors_event_t accel_event, mag_event, gyro_event;
};

#endif // NXPADA9DOF_ENABLE

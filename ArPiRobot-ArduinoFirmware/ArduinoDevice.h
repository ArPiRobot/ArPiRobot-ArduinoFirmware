#pragma once

#include "settings.h"
#include "Arduino.h"
#include <stdint.h>
#include "conversions.h"


class RPiInterface;

class ArduinoDevice {
public:
	ArduinoDevice(uint8_t buffer_len);
  virtual ~ArduinoDevice();

  /**
   * Polls sensor. If new data to send sets it in the buffer
   */
	virtual void poll() = 0;

  virtual void handleData(uint8_t *data, uint8_t len) = 0;

  void assignDeviceId(uint8_t deviceId);

  /**
   * If there is data to send in the buffer send it using the given RPiInterface
   */
  void sendBuffer(RPiInterface &rpi);
  
	static uint8_t sendTimeOffset;
	unsigned long nextSendTime = 0;
  uint8_t deviceId = 0;

protected:
  // This holds data to be sent
  uint8_t *buffer;
  uint8_t buffer_count = 0;
};

class SingleEncoder : public ArduinoDevice {
public:
  SingleEncoder(int pin);

  void poll() override;
  void handleData(uint8_t *data, uint8_t len) override;
  
  uint8_t lastState = 0;
  uint8_t pin;
  bool changed;
  Any16 count;
};

class Ultrasonic4Pin : public ArduinoDevice {
public:
  Ultrasonic4Pin(int triggerPin, int echoPin);

  void poll() override;
  void handleData(uint8_t *data, uint8_t len) override;

  uint8_t triggerPin, echoPin;
  Any16 distance;
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

  void poll() override;
  void handleData(uint8_t *data, uint8_t len) override;

private:
  static bool locked;

  unsigned long lastSample = 0;
  
  // Sensors
  Adafruit_LSM303_Accel_Unified *accel = NULL;
  Adafruit_L3GD20_Unified *gyro = NULL;

  // Calculated values
  Any32 gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
  sensors_event_t accel_event, gyro_event;
};

#endif // OLDADA9DOF_ENABLE

class VoltageMonitor : public ArduinoDevice{
public:
  VoltageMonitor(uint8_t readPin, float vboard, uint32_t r1, uint32_t r2);

  void poll() override;
  void handleData(uint8_t *data, uint8_t len) override;

private:
  uint8_t readPin;
  float vboard;
  uint32_t r1, r2;

  Any32 voltage;
};

#ifdef NXPADA9DOF_ENABLE

#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

class NxpAdafruit9Dof : public ArduinoDevice{
public:
  NxpAdafruit9Dof();
  ~NxpAdafruit9Dof();

  void poll() override;
  void handleData(uint8_t *data, uint8_t len) override;

private:
  static bool locked;

  unsigned long lastSample = 0;
  
  // Sensors
  Adafruit_FXOS8700 *accelmag = NULL;
  Adafruit_FXAS21002C *gyro = NULL;

  // Calculated values
  Any32 gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
  sensors_event_t accel_event, mag_event, gyro_event;
};

#endif // NXPADA9DOF_ENABLE

#pragma once

#include "settings.h"
#include "Arduino.h"
#include "ArduinoDevice.h"


class RPiInterface{
public:
  int addDevice(String buf);
  void reset();
  void configure();
  void feed();

  virtual void open() = 0;
  virtual int available() = 0;
  virtual char read() = 0;
  virtual void print(String data) = 0;
  virtual void write(uint8_t data) = 0;
  virtual void write(uint8_t *data, uint8_t len) = 0;
  virtual void flush() = 0;

private:
  // Sensor data buffer
  uint8_t buffer[DATA_BUFFER_SIZE];
  uint8_t len = 0;
  
  // Serial read buffer
  String readBuffer = "";
  
  ArduinoDevice *devices[MAX_DEVICES];
  uint8_t deviceCount = 0;
};

#if defined(INTERFACE_HW_SERIAL) || defined(INTERFACE_TEENSY_USB_SERIAL) || defined(INTERFACE_SW_SERIAL)

#ifdef INTERFACE_HW_SERIAL
typedef HardwareSerial Serial_t;
#endif
#ifdef INTERFACE_SW_SERIAL
#include <SoftwareSerial.h>
typedef SoftwareSerial Serial_t;
#endif
#ifdef INTERFACE_TEENSY_USB_SERIAL
typedef usb_serial_class Serial_t;
#endif

class RPiUartInterface : public RPiInterface {
public:
  RPiUartInterface(Serial_t &serial, uint32_t baud);

  void open() override;
  int available() override;
  char read() override;
  void print(String data)override;
  void write(uint8_t data) override;
  void write(uint8_t *data, uint8_t len) override;
  void flush() override;

private:
  Serial_t &serial;
  uint32_t baud;
};

#endif // UART interface

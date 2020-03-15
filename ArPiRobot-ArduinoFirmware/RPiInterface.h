#pragma once

#include "settings.h"
#include "Arduino.h"
#include "ArduinoDevice.h"
#include "FastCRC.h"

/*
 * Interface for raspberry pi
 * The Pi sends commands to the arduino using the same method as described with writeData.
 * Commands (as ascii) include:
 *  "RESET": Clear configured devices and wait for more add device commands
 *  "END": Done configuring devices. Start sensor processing.
 *  "ADD[DEVICE_CODE][PARAMS]": Add a device based on the device code and its parameters (vary by device)
 * All data sent to the pi is sent as described with the writeData function (using a start and end byte escaping data)
 *  This includes responses to commands and the initial indication that the arduino is ready.
 *  
 * Any command starting with a hyphen ("-" ASCII 45) is data to be sent to a device and will be handled as such if 
 *  sensor processing has started (not in the configure stage)
 * 
 * The general communication will work something like:
 *  Arduino is reset on uart connection from Pi (if not like with teensy the Pi sends a reset command anyway)
 *  The arduino then writes a ready command "START" (using writeData method)
 *  The arduino waits for add commands from the pi. For each add command a sensor is created as described by the 
 *    parameters and added to the interfaces device list.
 *  The Pi then writes an "END" command
 *  The arduino responds with "END" (writeData method) then starts sensor processing
 *  The arduino sends sensor data as necessary until a RESET command is read (ASCII newline delimited).
 *  No other data should be sent form the Pi to the arduino.
 *  
 */

extern FastCRC16 CRC16;

// Start, end, and escape byte for sending data from sensors
const uint8_t startByte = 253;
const uint8_t endByte = 254;
const uint8_t escapeByte = 255;

union byte_convert_4{
  uint8_t b[4];
  float fval;
  uint32_t uival;
  int32_t ival;
};

class RPiInterface{
public:

  /**
   * Add a device to the pi from arduino code (not based on data received from the Pi)
   * This device will always exist. This must be done before calling configure
   */
  int addStaticDevice(ArduinoDevice *device);
  int addDevice();
  void reset();
  void configure();
  void feed();

  // Read data handling escape sequences as described with writeData
  // Returns true if a complete data set is in the readBuffer
  bool readData();

  /*
   * Data packets are in the following format
   * start_byte, data..., crc16_high, crc16_low, end_byte
   * 
   * 
   * Where data varies in length and can contain start_byte, end_byte, and escape_byte
   * Before sent data is modified so that 
   *    start_byte is replaced with escape_byte, start_byte
   *    end_byte is replaced with escape_byte, end_byte
   *    escape_byte is replaced with escape_byte, escape_byte
   * The CRC16 (ccitt false) is calculated on the original unmodified (not escaped) data. The high and low bytes are
   *    modified the same way (escape sequences) if the values match start, end, or escape bytes
   */
  void writeData(uint8_t *data, uint8_t len);

  // Check that received data crc is valid
  bool checkData(uint8_t *data, uint8_t len);

  // Does data 1 start with data 2
  bool dataStartsWith(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2);

  // Does data 1 match data 2
  bool dataDoesMatch(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2);

  virtual void open() = 0;
  virtual uint8_t available() = 0;
  virtual int16_t read() = 0;
  virtual void write(uint8_t data) = 0;
  virtual void flush() = 0;

private:
  // Sensor data buffer
  uint8_t buffer[DATA_WRITE_BUFFER_SIZE];
  uint8_t len = 0;
  
  // Serial read buffer
  uint8_t readBuffer[DATA_READ_BUFFER_SIZE];
  uint8_t readBufferLen = 0;

  // Status of parsing data (readData also handles parsing)
  bool parse_started = false;  // Has start byte
  bool parse_escaped = false;  // Is parse escaped
  
  ArduinoDevice *devices[MAX_DEVICES];
  uint8_t deviceCount = 0;
  uint8_t staticDeviceCount = 0;
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
  uint8_t available() override;
  int16_t read() override; // int16_t because returns -1 if no data to read
  void write(uint8_t data) override;
  void flush() override;

private:
  Serial_t &serial;
  uint32_t baud;
};

#endif // UART interface

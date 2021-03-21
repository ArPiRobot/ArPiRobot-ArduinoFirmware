/*
 * Copyright 2021 Marcus Behel
 *
 * This file is part of ArPiRobot-ArduinoFirmware.
 * 
 * ArPiRobot-ArduinoFirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * ArPiRobot-ArduinoFirmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware.  If not, see <https://www.gnu.org/licenses/>. 
 */


#pragma once

#include <FastCRC.h>
#include <Arduino.h>
#include <LinkedList.h>
#include "board.hpp"
#include "action.hpp"

/**
 * Communications with the Raspberry Pi can occur over one of many communication buses. 
 * The BaseComm class abstracts high-level (bus agnostic) communication logic.
 * Currently implemented are
 *  - UART (UartComm class)
 * Data received from the Pi is interpreted as a command
 * Data sent to the Pi will either be a response or a status message.
 * Responses are synchronous responses to commands. 
 * After running a command the arduino responds to the pi with an error code.
 * Status messages are asynchronous "updates" containing data from interrupts or polling
 * 
 * The following describes what data is sent for each type of message. Bracketed data is an array.
 *  Commands:  FLAG_COMMAND, COMMAND_NUM, [COMMAND_ARGS]
 *  Responses: FLAG_RESPONSE, ERROR_CODE, [RESPONSE_DATA]
 *  Status:    FLAG_STATUS, STATUS_DATA
 *             The first byte of STATUS_DATA is the STATUS_SOURCE, which is an ID for the auto action
 *  
 * The actual message data is sent in a packet of the following format
 * START_BYTE, [data], crc16_high, crc16_low, END_BYTE
 * An ESCAPE_BYTE is also defined. Any instances of the START_BYTE, END_BYTE, or ESCAPE_BYTE
 * in the data or in the CRC bytes will be prefixed with an escape byte.
 * This format ensures that it is always possible to determine when a packet starts and ends, 
 * while still ensuring that the data can contain any possible value of a byte.
 */


enum class MessageType{
  COMMAND = 0,
  RESPONSE = 1,
  STATUS = 2
};

// TODO: Periodic digitalRead (including dt micros)
// TODO: Periodic analogRead (including dt micros)
// TODO: Polling digital counter (state change)
// TODO: Periodic write and pulseIn timing
// TODO: I2CWrite
// TODO: I2CRead
// TODO: I2CRequest
// TODO: I2CAvailable
// TODO: Periodic read register (including dt micros)

// TODO: Interrupt digital counter (state change)
// TODO: Interrupt write and pulseIn timing

enum class Command{
  PIN_MODE = 0,
  DIGITAL_WRITE = 1,
  DIGITAL_READ = 2,
  ANALOG_READ = 3,
  ANALOG_WRITE = 4,
  ANALOG_INPUT_TO_DIGITAL_PIN = 5,
  
  STOP_AUTO_ACTION = 6, // Stop an auto action (auto actions send data using status messages)
  POLL_DIG_READ = 7     // Start auto action to digitalRead a pin (polling)
};

enum class ErrorCode{
  NONE = 0,
  INVALID_ARG = 1,
  NOT_ENOUGH_ARGS = 2,
  EXECUTION = 3,
  UNKNOWN_COMMAND = 4
};

class BaseComm{
public:
  BaseComm(const BaseComm &other) = delete;
  BaseComm &operator=(const BaseComm &other) = delete;
  virtual ~BaseComm();
  
  void begin();
  void end();
  void service();
  void handleCommand();

  void respond(ErrorCode errorCode, uint8_t *data, uint8_t len);

  void sendStatus(uint8_t *data, uint8_t len);

protected:

  BaseComm();
  
  // Communication bus specific functions
  virtual void open() = 0;
  virtual void close() = 0;
  virtual uint8_t available() = 0;
  virtual uint8_t read() = 0;
  virtual void write(uint8_t b) = 0;

private:
  bool readData();
  bool checkData();
  void writeData(uint8_t *data, uint8_t len);

  // Constaints, static objects
  const static uint8_t START_BYTE = 253;
  const static uint8_t END_BYTE = 254;
  const static uint8_t ESCAPE_BYTE = 255;
  const static uint8_t READ_BUFFER_SIZE = 64;
  const static FastCRC16 CRC16;

  // Member variables
  bool parse_started = false;
  bool parse_escaped = false;
  uint8_t readBuffer[BaseComm::READ_BUFFER_SIZE];
  uint8_t readBufferLen = 0;

  LinkedList<AutoAction*> autoActions;
};

template <class SERIAL_T>
class UartComm : public BaseComm{
public:
  UartComm(SERIAL_T &serial, unsigned long baud) : serial(serial), baud(baud){}
  UartComm(const UartComm &other) = delete;
  UartComm& operator=(const UartComm& other) = delete;
protected:
  void open() override{
    serial.begin(baud);
  }
  void close() override{
    serial.end();
  }
  uint8_t available() override{
    return serial.available();
  }
  uint8_t read() override{
    return serial.read();
  }
  void write(uint8_t b) override{
    serial.write(b);
  }
private:
  SERIAL_T &serial;
  unsigned long baud;
};

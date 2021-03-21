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

#include "comm.hpp"
#include "gpio.hpp"
#include "conversions.hpp"

const static FastCRC16 BaseComm::CRC16;

BaseComm::~BaseComm(){
  end();
}

void BaseComm::begin(){
  open();
  writeData("READY", 5);
}

void BaseComm::end(){
  close();
}

void BaseComm::service(){
  uint8_t count = Serial.available();
  for(uint8_t i = 0; i < count; ++i){
    if(readData()){
      if(checkData()){
        if(readBuffer[0] == static_cast<uint8_t>(MessageType::COMMAND)){
          handleCommand();
        }
      }
      readBufferLen = 0;
    }
  }
  for(uint8_t i = 0; i < autoActions.size(); ++i){
    auto it = autoActions.get(i);
    if(it->service()){
      it->sendData(*this);
    }
  }
}

void BaseComm::handleCommand(){
  Command cmd = static_cast<Command>(readBuffer[1]);
  if(cmd == Command::PIN_MODE){
    return GpioHelper::pinModeHelper(*this, readBuffer + 2, readBufferLen - 2);
  }else if(cmd == Command::DIGITAL_WRITE){
    return GpioHelper::digitalWriteHelper(*this, readBuffer + 2, readBufferLen - 2);
  }else if(cmd == Command::DIGITAL_READ){
    return GpioHelper::digitalReadHelper(*this, readBuffer + 2, readBufferLen - 2);
  }else if(cmd == Command::ANALOG_WRITE){
    return GpioHelper::analogWriteHelper(*this, readBuffer + 2, readBufferLen - 2);
  }else if(cmd == Command::ANALOG_READ){
    return GpioHelper::analogReadHelper(*this, readBuffer + 2, readBufferLen - 2);
  }else if(cmd == Command::ANALOG_INPUT_TO_DIGITAL_PIN){
    return GpioHelper::analogInputToDigitalPinHelper(*this, readBuffer + 2, readBufferLen - 2);
  }else if(cmd == Command::STOP_AUTO_ACTION){
    // Args: ActionId (uint8_t) = index in list
    if(readBufferLen < 3){
      return respond(ErrorCode::NOT_ENOUGH_ARGS, nullptr, 0);
    }
    if(readBuffer[2] >= autoActions.size()){
      return respond(ErrorCode::EXECUTION, nullptr, 0);
    }
    AutoAction *action = autoActions.remove(readBuffer[2]);
    delete action;
    return respond(ErrorCode::NONE, nullptr, 0);
  }else if(cmd == Command::POLL_DIG_READ){
    // Args: pin(uint8_t)
    if(readBufferLen < 3){
      respond(ErrorCode::NOT_ENOUGH_ARGS, nullptr, 0);
      return;
    }
    AutoDigitalRead *autoDigRead = new AutoDigitalRead();
    autoDigRead->configure(readBuffer[2]);
    autoActions.add(autoDigRead);
    uint8_t actionId = autoDigRead->getActionId();
    respond(ErrorCode::NONE, &actionId, 1);
    return;
  }else if(cmd == Command::POLL_ANA_READ){
    // Args: pin(uint8_t),changeThreshold(uint16_t),sendRate(uint16_t)
    if(readBufferLen < 7){
      respond(ErrorCode::NOT_ENOUGH_ARGS, nullptr, 0);
      return;
    }
    uint16_t changeThreshold = Conversions::convertDataToInt16(readBuffer + 3, false);
    uint16_t sendRate = Conversions::convertDataToInt16(readBuffer + 5, false);
    
    AutoAnalogRead *autoAnaRead = new AutoAnalogRead();
    autoAnaRead->configure(readBuffer[2], changeThreshold, sendRate);
    autoActions.add(autoAnaRead);
    uint8_t actionId = autoAnaRead->getActionId();
    respond(ErrorCode::NONE, &actionId, 1);
    return;
  }else if(cmd == Command::POLL_DIG_COUNT){
    // Args: pin(uint8_t),changeThreshold(uint16_t),sendRate(uint16_t)
    if(readBufferLen < 7){
      respond(ErrorCode::NOT_ENOUGH_ARGS, nullptr, 0);
      return;
    }
    uint16_t changeThreshold = Conversions::convertDataToInt16(readBuffer + 3, false);
    uint16_t sendRate = Conversions::convertDataToInt16(readBuffer + 5, false);
    
    AutoDigitalCounter *autoDigCount = new AutoDigitalCounter();
    autoDigCount->configure(readBuffer[2], changeThreshold, sendRate);
    autoActions.add(autoDigCount);
    uint8_t actionId = autoDigCount->getActionId();
    respond(ErrorCode::NONE, &actionId, 1);
    return;
  }
  return respond(ErrorCode::UNKNOWN_COMMAND, nullptr, 0);
}

void BaseComm::respond(ErrorCode errorCode, uint8_t *data, uint8_t len){
  uint8_t *fullMessage = new uint8_t[len + 2];
  fullMessage[0] = static_cast<uint8_t>(MessageType::RESPONSE);
  fullMessage[1] = static_cast<uint8_t>(errorCode);
  for(uint8_t i = 0; i < len; ++i){
    fullMessage[i + 2] = data[i];
  }
  writeData(fullMessage, len + 2);
  delete[] fullMessage;
}

void BaseComm::sendStatus(uint8_t *data, uint8_t len){
  uint8_t *fullMessage = new uint8_t[len + 1];
  fullMessage[0] = static_cast<uint8_t>(MessageType::STATUS);
  for(uint8_t i = 0; i < len; ++i){
    fullMessage[i + 1] = data[i];
  }
  writeData(fullMessage, len + 1);
  delete[] fullMessage;
}

BaseComm::BaseComm(){
  
}

// Read one byte and add (as necessary) to read buffer
bool BaseComm::readData(){
  int16_t c;
  if(available() > 0){
    c = read();
  }else{
    return false;
  }
  
  // If buffer is full, empty the buffer.
  // Not ideal, as data is lost, but cannot overfill
  // buffer and this ensures future data is still handled.
  if(readBufferLen == READ_BUFFER_SIZE){
    readBufferLen = 0;
  }

  if(parse_escaped){
    // Ignore invalid escaped data
    if(c == START_BYTE || c == END_BYTE || c == ESCAPE_BYTE){
      readBuffer[readBufferLen++] = c;
    }
    parse_escaped = false; // Past the next byte. No longer escaped
  }else{
    if(c == START_BYTE){
      if(parse_started){
        // Got a second start byte. Trash what is already in the buffer
        readBufferLen = 0;
      }
      parse_started = true;
    }else if(c == END_BYTE && parse_started){
      parse_started = false;
      return true; // Have complete data set
    }else if(c == ESCAPE_BYTE && parse_started){
      parse_escaped = true;
    }else if(parse_started){
      readBuffer[readBufferLen++] = c;
    }
  }

  return false; // Not complete data set
}

// Check the data in the read buffer to determine if it is valid
bool BaseComm::checkData(){
  // Big endian CRC at end of data
  uint16_t readCrc = (readBuffer[readBufferLen - 2] << 8) | readBuffer[readBufferLen - 1];
  uint16_t calcCrc = CRC16.ccitt(readBuffer, readBufferLen - 2);
  return readCrc == calcCrc;
}

void BaseComm::writeData(uint8_t *data, uint8_t len){
  write(START_BYTE);
  for(uint8_t i = 0; i < len; ++i){
    if(data[i] == END_BYTE){
      write(ESCAPE_BYTE);
      write(END_BYTE);
    }else if(data[i] == START_BYTE){
      write(ESCAPE_BYTE);
      write(START_BYTE);
    }else if(data[i] == ESCAPE_BYTE){
      write(ESCAPE_BYTE);
      write(ESCAPE_BYTE);
    }else{
      write(data[i]);
    }
  }

  // Send CRC big endian
  uint16_t crc = CRC16.ccitt(data, len);
  uint8_t crcHigh = (crc >> 8);
  uint8_t crcLow = (crc & 0xFF);

  // High byte
  if(crcHigh == START_BYTE || crcHigh == END_BYTE || crcHigh == ESCAPE_BYTE){
    write(ESCAPE_BYTE);
  }
  write(crcHigh);

  // Low byte
  if(crcLow == START_BYTE || crcLow == END_BYTE || crcLow == ESCAPE_BYTE){
    write(ESCAPE_BYTE);
  }
  write(crcLow);

  write(END_BYTE);
}

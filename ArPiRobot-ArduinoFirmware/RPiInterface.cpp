/*
 * Copyright 2020 Marcus Behel
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

#include "RPiInterface.h"
#include "conversions.h"

FastCRC16 CRC16;


int RPiInterface::addStaticDevice(ArduinoDevice *device){
  if(!canAddStatic){
    return -2;
  }
  
  if(devices.size() >= MAX_DEVICES){
    return -1;
  }

  // Add at beginning of linked list
  device->assignDeviceId(devices.size());
  devices.add(0, device);
  
  staticDeviceCount++; // This is used when resetting deviceCount

  return devices.size() - 1;
}

/*
 * Device add commands:
 *  Commands are ascii as it is easy to code comparisons
 *  ADDSENC[PIN_NUMBER][PULLUP] - SingleEncoder(PIN_NUMBER)
 *  ADDOLDADA9DOF - OldAdafruit9DofImu()
 *  ADDUSONIC4[TRIG_PIN][ECHO_PIN]\n
 *  ADDVMON[ANALOG_PIN][VBOARD_FLOAT_4_BYTES_BIG_ENDIAN][4_BYTE_R1_UINT32_BIG_ENDIAN][4_BYTE_R2_UINT32_BIG_ENDIAN] - VoltageMonitor(readPin, vboard, r1, r2)
 *  ADDNXPADA9DOF - NxpAdafruit9DofImu()
 *  ADDIRREFLECTOR[DIGITAL_PIN_NUM][ANALOG_PIN_NUM] - IRReflectorModule(DIGITAL_PIN_NUM, ANALOG_PIN_NUM)
 *  
 *  Digital pin numbers are sent as two bytes each (any analog pin is also able to be used as a digital input):
 *    IS_ANALOG, PIN_NUMBER
 *  
 *  Pins that must be analog are sent as one byte: the pin number.
 *  
 *  A 1 or 0 flag can be used to indicate if an input pin should use the internal pullup resistor (1 = on, 0 = off)
 */


// Returns device id. Uses readBuffer to get data
// Note: always add to the beginning of the linked list - add(0, device)
int RPiInterface::addDevice(){
  String buf;
  if(devices.size() >= MAX_DEVICES){
    return -1;
  }
  if(readBufferLen >= 9 && dataStartsWith(readBuffer, readBufferLen, "ADDSENC", 7)){
    uint8_t readPin = readBuffer[8];

    // If analog
    if(readBuffer[7]){
      readPin = analogInputToDigitalPin(readPin);
    }    

    bool pullup = readBuffer[9];

#ifdef DEBUG
    DEBUG_SERIAL.print("Encoder pin: ");
    DEBUG_SERIAL.println(readPin);
    DEBUG_SERIAL.print("Pullup enabled: ");
    DEBUG_SERIAL.println(pullup);
#endif
    
    SingleEncoder *d = new SingleEncoder(readPin, pullup);
    d->assignDeviceId(devices.size());
    devices.add(0, d);
    return devices.size() - 1;
  }else if(dataDoesMatch(readBuffer, readBufferLen, "ADDOLDADA9DOF", 13)){
#ifdef OLDADA9DOF_ENABLE
    OldAdafruit9Dof *d = new OldAdafruit9Dof();
    d->assignDeviceId(devices.size());
    devices.add(0, d);
    return devices.size() - 1;
#else
    return -1;
#endif
  }else if(readBufferLen >= 14 && dataStartsWith(readBuffer, readBufferLen, "ADDUSONIC4", 10)){
    uint8_t trigPin = readBuffer[11];
    // If analog
    if(readBuffer[10])
      trigPin = analogInputToDigitalPin(trigPin);

    uint8_t echoPin = readBuffer[13];
    // If analog
    if(readBuffer[12])
      echoPin = analogInputToDigitalPin(echoPin);

#ifdef DEBUG
    DEBUG_SERIAL.print("Trig pin: ");
    DEBUG_SERIAL.println(trigPin);
    DEBUG_SERIAL.print("Echo pin: ");
    DEBUG_SERIAL.println(echoPin);
#endif
    
    Ultrasonic4Pin *d = new Ultrasonic4Pin(trigPin, echoPin);
    d->assignDeviceId(devices.size());
    devices.add(0, d);
    return devices.size() - 1;
  }else if(readBufferLen >= 20 && dataStartsWith(readBuffer, readBufferLen, "ADDVMON", 7)){
    uint8_t analogPin = readBuffer[7];

    Any32 vboard, r1, r2;

    // This data was sent big endian
    unbufferValue32(readBuffer, 8, false, &vboard);
    unbufferValue32(readBuffer, 12, false, &r1);
    unbufferValue32(readBuffer, 16, false, &r2);

#ifdef DEBUG
    DEBUG_SERIAL.print("Pin: ");
    DEBUG_SERIAL.println(analogPin);
    DEBUG_SERIAL.print("Vboard: ");
    DEBUG_SERIAL.println(vboard.fval);
    DEBUG_SERIAL.print("R1: ");
    DEBUG_SERIAL.println((int)r1.uival);
    DEBUG_SERIAL.print("R2: ");
    DEBUG_SERIAL.println((int)r2.uival);
#endif
    
    VoltageMonitor *d = new VoltageMonitor(analogInputToDigitalPin(analogPin), vboard.fval, r1.uival, r2.uival);
    d->assignDeviceId(devices.size());
    devices.add(0, d);
    return devices.size() - 1;
  }else if(dataDoesMatch(readBuffer, readBufferLen, "ADDNXPADA9DOF", 13)){
#ifdef NXPADA9DOF_ENABLE
    NxpAdafruit9Dof *d = new NxpAdafruit9Dof();
    d->assignDeviceId(devices.size());
    devices.add(0, d);
    return devices.size() - 1;
#else
    return -1;
#endif
  }else if(readBufferLen >= 16 && dataStartsWith(readBuffer, readBufferLen, "ADDIRREFLECTOR", 14)){
    uint8_t digitalPin = readBuffer[15];
    if(readBuffer[14]){
      // Digital pin was specified using analog number: meaning if readBuffer[15] is 0 this means A0
      digitalPin = analogInputToDigitalPin(digitalPin);
    }

    // This is always an analog number (0=A0, 1=A1, etc)
    // Can still use analogRead with the digital pin number for an analog pin
    // 255 is a special value used to disable analog measurements
    uint8_t analogPin;
    if(readBuffer[16] == 255)
      analogPin = 255;
    else
      analogPin = analogInputToDigitalPin(readBuffer[16]);

    IRReflectorModule *d = new IRReflectorModule(digitalPin, analogPin);
    d->assignDeviceId(devices.size());
    devices.add(0, d);
    return devices.size() - 1;
  }

#ifdef DEBUG
  DEBUG_SERIAL.println("Got command to add unknown device...");
#endif
  return -1;
}

void RPiInterface::reset(){
  // Delete non-static devices ((deviceCount - staticDeviceCount) devices removed from the front)
  // None-static devices added to the front of the linked list after static devices.
  for(uint8_t i = staticDeviceCount; i < devices.size(); ++i){
    devices.remove(0);
  }
  // There are now only staticDeviceCount devices in the list
  readBufferLen = 0;
}

void RPiInterface::configure(){
#ifdef DEBUG
  DEBUG_SERIAL.println("Starting configure");
#endif

  // No longer allow static devices. They must be added to the linked list before any dynamic devices (devices created by data from the pi)
  canAddStatic = false;

  writeData("START", 5);
  while(true){
    if(readData() && checkData(readBuffer, readBufferLen)){
      // Ignore CRC in the buffer now
      readBufferLen -= 2;
      if(dataDoesMatch(readBuffer, readBufferLen, "END", 3)){
#ifdef DEBUG
        DEBUG_SERIAL.println("Ending configure");
#endif
        break;
      }else if(dataDoesMatch(readBuffer, readBufferLen, "RESET", 5)){
        readBufferLen = 0;
        reset();
        configure();
        return;
      }else if(dataStartsWith(readBuffer, readBufferLen, "ADD", 3)){
        int deviceId = addDevice();
        if(deviceId == -1){
          writeData("ADDFAIL", 7);
        }else{
          uint8_t buf[11] = "ADDSUCCESS"; // 10 character string  + 1 for a device id
          buf[10] = deviceId;
          writeData(buf, 11);
        }
      }
      // Reset read buffer
      readBufferLen = 0;
    }
    delay(10);
  }
  
  readBufferLen = 0;
  writeData("END", 3);
}

void RPiInterface::feed(){
  unsigned long now = millis();
  for(uint8_t i = 0; i < devices.size(); ++i){

    // Poll the device. If the device has new data it will put it in its buffer here

    // Iterating in order will not cause performance issues. get() method caches
    ArduinoDevice *d = devices.get(i);
    
    d->poll();    
    if(now >= d->nextSendTime){
      // Send the data (if the sensor has any in its buffer)
      d->sendBuffer(*this);
      d->nextSendTime += SEND_RATE; // Send again
    }
  }

  while(available()){
    if(readData() && checkData(readBuffer, readBufferLen)){
      // Ignore CRC now
      readBufferLen -= 2;
      
      if(dataDoesMatch(readBuffer, readBufferLen, "RESET", 5)){
        reset();
        configure();
        return;
      }else if (dataStartsWith(readBuffer, readBufferLen, "-", 1)){
        uint8_t id = readBuffer[1];
#ifdef DEBUG
        DEBUG_SERIAL.print("Got data for device with id: ");
        DEBUG_SERIAL.println((int)id);
#endif
        for(uint8_t i = 0; i < devices.size(); ++i){
          // Iterating in forward order will not cause performance issues. get() method caches.
          ArduinoDevice *d = devices.get(i);
          if(d->deviceId == id){
            // Skip the '-' in the data given to the device
            d->handleData(&readBuffer[1], readBufferLen - 1);
            break;
          }
        }
      }
      readBufferLen = 0; // Clear buffer after handling data
    }
  }
}

// Read one byte and add (as necessary) to read buffer
bool RPiInterface::readData(){
  int16_t c;
  if(available() > 0){
    c = read();
  }else{
    return false;
  }

  if(parse_escaped){
    // Ignore invalid escaped data
    if(c == startByte || c == endByte || c == escapeByte){
      readBuffer[readBufferLen++] = c;
    }
    parse_escaped = false; // Past the next byte. No longer escaped
  }else{
    if(c == startByte){
      if(parse_started){
        // Got a second start byte. Trash what is already in the buffer
        readBufferLen = 0;
      }
      parse_started = true;
    }else if(c == endByte && parse_started){
      parse_started = false;
      return true; // Have complete data set
    }else if(c == escapeByte && parse_started){
      parse_escaped = true;
    }else if(parse_started){
      readBuffer[readBufferLen++] = c;
    }
  }

  return false; // Not complete data set
}


void RPiInterface::writeData(uint8_t *data, uint8_t len){
  write(startByte);
  for(uint8_t i = 0; i < len; ++i){
    if(data[i] == endByte){
      write(escapeByte);
      write(endByte);
    }else if(data[i] == startByte){
      write(escapeByte);
      write(startByte);
    }else if(data[i] == escapeByte){
      write(escapeByte);
      write(escapeByte);
    }else{
      write(data[i]);
    }
  }

  // Send CRC big endian
  Any16 crc;
  crc.uival = CRC16.ccitt(data, len);
  uint8_t crcbuffer[2];
  bufferValue16(crc, false, crcbuffer, 0);

  // High byte
  if(crcbuffer[0] == startByte || crcbuffer[0] == endByte || crcbuffer[0] == escapeByte){
    write(escapeByte);
  }
  write(crcbuffer[0]);

  // Low byte
  if(crcbuffer[1] == startByte || crcbuffer[1] == endByte || crcbuffer[1] == escapeByte){
    write(escapeByte);
  }
  write(crcbuffer[1]);

  write(endByte);
}

bool RPiInterface::checkData(uint8_t *data, uint8_t len){
  Any16 read_crc;

  // Big endian CRC at end of data
  unbufferValue16(data, len - 2, false, &read_crc);

  uint16_t calc_crc = CRC16.ccitt(data, len - 2);

  return calc_crc == read_crc.uival;
}

// Does data 1 start with data 2
bool RPiInterface::dataStartsWith(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2){
  if(len2 > len1) return false;
  for(uint8_t i = 0; i < len2; ++i){
    if(data1[i] != data2[i]) return false;
  }
  return true;
}

// Does data 1 match data 2
bool RPiInterface::dataDoesMatch(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2){
  if(len1 != len2) return false;

  for(uint8_t i = 0; i < len1; ++i){
    if(data1[i] != data2[i]) return false;
  }
  return true;
}


#if defined(INTERFACE_HW_SERIAL) || defined(INTERFACE_TEENSY_USB_SERIAL) || defined(INTERFACE_SW_SERIAL)

RPiUartInterface::RPiUartInterface(Serial_t &serial, uint32_t baud) : serial(serial), baud(baud){
  
}

void RPiUartInterface::open(){
  serial.begin(baud);
  while(!serial);
}

uint8_t RPiUartInterface::available(){
  return serial.available();
}

int16_t RPiUartInterface::read(){
  return serial.read();
}

void RPiUartInterface::write(uint8_t data){
  serial.write(data);
}

void RPiUartInterface::flush(){
  serial.flush();
}

#endif

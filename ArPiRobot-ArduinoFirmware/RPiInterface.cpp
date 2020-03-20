#include "RPiInterface.h"


FastCRC16 CRC16;


int RPiInterface::addStaticDevice(ArduinoDevice *device){
  if(deviceCount >= MAX_DEVICES){
    return -1;
  }

  devices[deviceCount] = device;
  devices[deviceCount]->assignDeviceId(deviceCount);
  
  staticDeviceCount++; // This is used when resetting deviceCount
  deviceCount++;

  return deviceCount - 1;
}

/*
 * Device add commands:
 *  Commands are ascii as it is easy to code comparisons
 *  ADDSENC[PIN_NUMBER] - SingleEncoder(PIN_NUMBER)
 *  ADDOLDADA9DOF - OldAdafruit9DofImu()
 *  ADDUSONIC4[TRIG_PIN][ECHO_PIN]\n
 *  ADDVMON[ANALOG_PIN][VBOARD_FLOAT_4_BYTES_BIG_ENDIAN][4_BYTE_R1_UINT32_BIG_ENDIAN][4_BYTE_R2_UINT32_BIG_ENDIAN] - VoltageMonitor(readPin, vboard, r1, r2)
 *  ADDNXPADA9DOF - NxpAdafruit9DofImu()
 *  
 *  Digital pin numbers are sent as two bytes each (any analog pin is also able to be used as a digital input):
 *    IS_ANALOG, PIN_NUMBER
 *  
 *  Pins that must be analog are sent as one byte: the pin number.
 */


// Returns device id. Uses readBuffer to get data
int RPiInterface::addDevice(){
  String buf;
  if(deviceCount >= MAX_DEVICES){
    return -1;
  }
  if(readBufferLen >= 9 && dataStartsWith(readBuffer, readBufferLen, "ADDSENC", 7)){
    uint8_t readPin = readBuffer[8];

    // If analog
    if(readBuffer[7]){
      readPin = analogInputToDigitalPin(readPin);
    }    

#ifdef DEBUG
    DEBUG_SERIAL.print("Encoder pin: ");
    DEBUG_SERIAL.println(readPin);
#endif
    
    devices[deviceCount] = new SingleEncoder(readPin);
    devices[deviceCount]->assignDeviceId(deviceCount);
    deviceCount++;
    return deviceCount - 1;
  }else if(dataDoesMatch(readBuffer, readBufferLen, "ADDOLDADA9DOF", 13)){
#ifdef OLDADA9DOF_ENABLE
    devices[deviceCount] = new OldAdafruit9Dof();
    devices[deviceCount]->assignDeviceId(deviceCount);
    deviceCount++;
    return deviceCount - 1;
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
    
    devices[deviceCount] = new Ultrasonic4Pin(trigPin, echoPin);
    devices[deviceCount]->assignDeviceId(deviceCount);
    deviceCount++;
    return deviceCount - 1;
  }else if(readBufferLen >= 20 && dataStartsWith(readBuffer, readBufferLen, "ADDVMON", 7)){
    uint8_t analogPin = readBuffer[7];

    byte_convert_4 vboard, r1, r2;

    uint16_t i = 1;  
    char *c = (char*)&i;

    if(c){
      // This is a little endian system (reverse order because these were sent big endian)
      vboard.b[0] = readBuffer[11];
      vboard.b[1] = readBuffer[10];
      vboard.b[2] = readBuffer[9];
      vboard.b[3] = readBuffer[8];
  
      r1.b[0] = readBuffer[15];
      r1.b[1] = readBuffer[14];
      r1.b[2] = readBuffer[13];
      r1.b[3] = readBuffer[12];
  
      r2.b[0] = readBuffer[19];
      r2.b[1] = readBuffer[18];
      r2.b[2] = readBuffer[17];
      r2.b[3] = readBuffer[16];
    }else{
      // This is a big endian system
      vboard.b[0] = readBuffer[8];
      vboard.b[1] = readBuffer[9];
      vboard.b[2] = readBuffer[10];
      vboard.b[3] = readBuffer[11];
  
      r1.b[0] = readBuffer[12];
      r1.b[1] = readBuffer[13];
      r1.b[2] = readBuffer[14];
      r1.b[3] = readBuffer[15];
  
      r2.b[0] = readBuffer[16];
      r2.b[1] = readBuffer[17];
      r2.b[2] = readBuffer[18];
      r2.b[3] = readBuffer[19];
    }

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
    
    devices[deviceCount] = new VoltageMonitor(analogInputToDigitalPin(analogPin), vboard.fval, r1.uival, r2.uival);
    devices[deviceCount]->assignDeviceId(deviceCount);
    deviceCount++;
    return deviceCount - 1;
  }else if(dataDoesMatch(readBuffer, readBufferLen, "ADDNXPADA9DOF", 13)){
#ifdef NXPADA9DOF_ENABLE
    devices[deviceCount] = new NxpAdafruit9Dof();
    devices[deviceCount]->assignDeviceId(deviceCount);
    deviceCount++;
    return deviceCount - 1;
#else
    return -1;
#endif
  }

#ifdef DEBUG
  DEBUG_SERIAL.println("Got command to add unknown device...");
#endif
  return -1;
}

void RPiInterface::reset(){
  // Delete non-static devices
  for(uint8_t i = staticDeviceCount; i < deviceCount; ++i){
    delete devices[i];
  }
  deviceCount = staticDeviceCount;
  readBufferLen = 0;
}

void RPiInterface::configure(){
#ifdef DEBUG
  DEBUG_SERIAL.println("Starting configure");
#endif
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
  for(uint8_t i = 0; i < deviceCount; ++i){

    // Start at buffer[1] so the device id can be put at buffer[0]
    bool shouldSend = devices[i]->poll(&buffer[1], &len);    
    if(shouldSend && now >= devices[i]->nextSendTime){
      // Send the data
      buffer[0] = devices[i]->deviceId;
      writeData(buffer, len + 1);
      flush();
      devices[i]->nextSendTime += SEND_RATE; // Send again
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
        for(uint8_t i = 0; i < deviceCount; ++i){
          if(devices[i]->deviceId == id){
            // Skip the '-' in the data given to the device
            devices[i]->handleData(&readBuffer[1], readBufferLen - 1);
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

  // Send CRC big endian (high byte first)
  uint16_t crc = CRC16.ccitt(data, len);
  uint8_t crc_high = crc >> 8;
  uint8_t crc_low = crc;

  if(crc_high == startByte || crc_high == endByte || crc_high == escapeByte){
    write(escapeByte);
  }
  write(crc_high);

  if(crc_low == startByte || crc_low == endByte || crc_low == escapeByte){
    write(escapeByte);
  }
  write(crc_low);

  write(endByte);
}

bool RPiInterface::checkData(uint8_t *data, uint8_t len){
  uint16_t read_crc = data[len - 1];
  read_crc = read_crc | (data[len - 2] << 8);

  uint16_t calc_crc = CRC16.ccitt(data, len - 2);

  return calc_crc == read_crc;
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

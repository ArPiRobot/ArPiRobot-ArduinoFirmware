#include "comm.h"

#include "gpio.h"

const FastCRC16 CRC16;
const uint8_t startByte = 253;
const uint8_t endByte = 254;
const uint8_t escapeByte = 255;
bool parse_started = false;
bool parse_escaped = false;
uint8_t readBuffer[READ_BUFFER_SIZE];
uint8_t readBufferLen = 0;

void startComm(){
  SER.begin(SER_BAUD);
  writeData(MSG_READY, strlen(MSG_READY));
}

void handleIncomingData(){
  uint8_t count = SER.available();
  for(uint8_t i = 0; i < count; ++i){
    if(readData()){
      if(checkData(readBuffer, readBufferLen)){
        handleCommand();
      }
      readBufferLen = 0;
    }
  }
}

void handleCommand(){
  // Make buffer a valid null terminated string, excluding the CRC
  // When passing cmd string to functions, remove the command and the first comma
  // Second null terminator in case no data after command 
  readBuffer[readBufferLen - 2] = '\0';
  readBuffer[readBufferLen - 1] = '\0'; 
  if(dataStartsWith(readBuffer, readBufferLen, CMD_PINMODE, strlen(CMD_PINMODE))){
    doPinMode(&readBuffer[strlen(CMD_PINMODE)]);
  }else if(dataStartsWith(readBuffer, readBufferLen, CMD_DIGWRITE, strlen(CMD_DIGWRITE))){
    doDigWrite(&readBuffer[strlen(CMD_DIGWRITE)]);
  }else if(dataStartsWith(readBuffer, readBufferLen, CMD_DIGREAD, strlen(CMD_DIGREAD))){
    doDigRead(&readBuffer[strlen(CMD_DIGREAD)]);
  }else if(dataStartsWith(readBuffer, readBufferLen, CMD_ANAREAD, strlen(CMD_ANAREAD))){
    doAnaRead(&readBuffer[strlen(CMD_ANAREAD)]);
  }else if(dataStartsWith(readBuffer, readBufferLen, CMD_ANATODIG, strlen(CMD_ANATODIG))){
    doAnaToDig(&readBuffer[strlen(CMD_ANATODIG)]);
  }else if(dataStartsWith(readBuffer, readBufferLen, CMD_ANAWRITE, strlen(CMD_ANAWRITE))){
    doAnaWrite(&readBuffer[strlen(CMD_ANAWRITE)]);
  }else{
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
  }
}

// Read one byte and add (as necessary) to read buffer
bool readData(){
  int16_t c;
  if(SER.available() > 0){
    c = SER.read();
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

void writeData(uint8_t *data, uint8_t len){
  SER.write(startByte);
  for(uint8_t i = 0; i < len; ++i){
    if(data[i] == endByte){
      SER.write(escapeByte);
      SER.write(endByte);
    }else if(data[i] == startByte){
      SER.write(escapeByte);
      SER.write(startByte);
    }else if(data[i] == escapeByte){
      SER.write(escapeByte);
      SER.write(escapeByte);
    }else{
      SER.write(data[i]);
    }
  }

  // Send CRC big endian
  uint16_t crc = CRC16.ccitt(data, len);
  uint8_t crcHigh = (crc >> 8);
  uint8_t crcLow = (crc & 0xFF);

  // High byte
  if(crcHigh == startByte || crcHigh == endByte || crcHigh == escapeByte){
    SER.write(escapeByte);
  }
  SER.write(crcHigh);

  // Low byte
  if(crcLow == startByte || crcLow == endByte || crcLow == escapeByte){
    SER.write(escapeByte);
  }
  SER.write(crcLow);

  SER.write(endByte);
}

bool checkData(uint8_t *data, uint8_t len){
  // Big endian CRC at end of data
  return ((data[len - 2] << 8) | data[len - 1]) == CRC16.ccitt(data, len - 2);
}

// Does data 1 start with data 2
bool dataStartsWith(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2){
  if(len2 > len1) return false;
  for(uint8_t i = 0; i < len2; ++i){
    if(data1[i] != data2[i]) return false;
  }
  return true;
}

// Does data 1 match data 2
bool dataDoesMatch(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2){
  if(len1 != len2) return false;

  for(uint8_t i = 0; i < len1; ++i){
    if(data1[i] != data2[i]) return false;
  }
  return true;
}

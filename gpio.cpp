#include "gpio.h"
#include "comm.h"

void doPinMode(const char *args){
  // GPPM,pin(#),mode(out, in, inpu)
  uint8_t pin;
  char *ptr = strtok(args, ",");
  if(ptr == NULL){
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  pin = atoi(ptr);
  ptr = strtok(NULL, ",");
  if(ptr == NULL){
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  if(strcmp(ptr, "out") == 0){
    pinMode(pin, OUTPUT);
  }else if(strcmp(ptr, "in") == 0){
    pinMode(pin, INPUT);
  }else if(strcmp(ptr, "inpu") == 0){
    pinMode(pin, INPUT_PULLUP);
  }else{
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  writeData(MSG_SUCCESS, strlen(MSG_SUCCESS));
}

void doDigWrite(const char *args){
  // GPDW,pin(#),state(1/0)
  uint8_t pin;
  char *ptr = strtok(args, ",");
  if(ptr == NULL){
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  pin = atoi(ptr);
  ptr = strtok(NULL, ",");
  if(ptr == NULL){
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  if(strcmp(ptr, "0") == 0){
    digitalWrite(pin, LOW);
  }else if(strcmp(ptr, "1") == 0){
    digitalWrite(pin, HIGH);
  }else{
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  writeData(MSG_SUCCESS, strlen(MSG_SUCCESS));
}

void doDigRead(const char *args){
  // GPDR,pin(#)
  char *ptr = strtok(args, ",");
  if(ptr == NULL){
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  uint8_t pin = atoi(ptr);
  // SUCCESS,res\0 = SUCCESS + 3 bytes (comma, res, null)
  char buf[strlen(MSG_SUCCESS) + 3];
  strcpy(buf, MSG_SUCCESS);
  buf[strlen(MSG_SUCCESS)] = ',';
  // itoa adds null terminator
  itoa(digitalRead(pin), &buf[strlen(MSG_SUCCESS) + 1], 10);
  writeData(buf, strlen(buf));
}

void doAnaRead(const char *args){
  // GPAR,pin(#)
  char *ptr = strtok(args, ",");
  if(ptr == NULL){
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  uint8_t pin = atoi(ptr);
  // SUCCESS,res\0 = SUCCESS + 6 bytes (comma, res(4 bytes max = 4 digits), null)
  char buf[strlen(MSG_SUCCESS) + 6];
  strcpy(buf, MSG_SUCCESS);
  buf[strlen(MSG_SUCCESS)] = ',';
  itoa(analogRead(pin), &buf[strlen(MSG_SUCCESS) + 1], 10);
  writeData(buf, strlen(buf));
}

void doAnaToDig(const char *args){
  // GPAD,pin(#)
  char *ptr = strtok(args, ",");
  if(ptr == NULL){
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  uint8_t pin = atoi(ptr);
  // SUCCESS,res\0 = SUCCESS + 5 bytes (comma, res(3 bytes max = 3 digits), null)
  char buf[strlen(MSG_SUCCESS) + 5];
  strcpy(buf, MSG_SUCCESS);
  buf[strlen(MSG_SUCCESS)] = ',';
  itoa(analogInputToDigitalPin(pin), &buf[strlen(MSG_SUCCESS) + 1], 10);
  writeData(buf, strlen(buf));
}

void doAnaWrite(const char *args){
  // GPAW,pin(#),value
  uint8_t pin;
  char *ptr = strtok(args, ",");
  if(ptr == NULL){
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  pin = atoi(ptr);
  ptr = strtok(NULL, ",");
  if(ptr == NULL){
    writeData(MSG_FAILURE, strlen(MSG_FAILURE));
    return;
  }
  analogWrite(pin, atoi(ptr));
  writeData(MSG_SUCCESS, strlen(MSG_SUCCESS));
}

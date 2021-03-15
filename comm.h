#pragma once

#include <FastCRC.h>
#include <Arduino.h>

// UART Settings
#define SER Serial
#define SER_BAUD 115200
#define READ_BUFFER_SIZE 64

// Commands
#define CMD_PINMODE  "GPPM"
#define CMD_DIGWRITE "GPDW"
#define GMD_DIGREAD  "GPDR"

// Messages
#define MSG_READY    "READY"
#define MSG_SUCCESS  "SUCCESS"
#define MSG_FAILURE  "FAILURE"

extern const FastCRC16 CRC16;
extern const uint8_t startByte;
extern const uint8_t endByte;
extern const uint8_t escapeByte;
extern bool parse_started;
extern bool parse_escaped;
extern uint8_t readBuffer[];
extern uint8_t readBufferLen;

void startComm();

void handleIncomingData();

void handleCommand();

// Read one byte and add (as necessary) to read buffer
bool readData();

void writeData(uint8_t *data, uint8_t len);

bool checkData(uint8_t *data, uint8_t len);

// Does data 1 start with data 2
bool dataStartsWith(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2);

// Does data 1 match data 2
bool dataDoesMatch(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2);

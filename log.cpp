
#include "log.h"


void log_internal(const char *message){
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(message);
#endif
}

void log_internal(char c){
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(c);
#endif
}

void log_internal(int value){
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(value);
#endif
}

void log_internal(float value){
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(value);
#endif
}

void log_internal(double value){
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(value);
#endif
}

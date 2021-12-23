
#include "logger.h"


void log_internal(const char *message){
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(message);
#endif
}

void log_char_internal(char c){
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(c);
#endif
}

void log_int_internal(int value){
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(value);
#endif
}

void log_float_internal(float value){
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(value);
#endif
}

void log_double_internal(double value){
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(value);
#endif
}

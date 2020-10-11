#pragma once

#if defined(ARDUINO_AVR_NANO_EVERY) && !defined(analogInputToDigitalPin)

// For some reason this is not defined for the arduino nano every...
uint8_t analogInputToDigitalPin(uint8_t p){
  switch(p){
  case 0:
    return A0;
  case 1:
    return A1;
  case 2:
    return A2;
  case 3:
    return A3;
  case 4:
    return A4;
  case 5:
    return A5;
  case 6:
    return A6;
  case 7:
    return A7;
  }
}

#endif

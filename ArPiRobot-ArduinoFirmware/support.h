#pragma once

#if defined(ARDUINO_AVR_NANO_EVERY) && !defined(analogInputToDigitalPin)

// For some reason this is not defined for the arduino nano every...
uint8_t analogInputToDigitalPin(uint8_t p){
  // Inverse mapping of analog_pin_to_channel array in pins_arduino.h for megaAVR 4809 
  switch(p){
  case 3:
    return 0;
  case 2:
    return 1;
  case 1:
    return 2;
  case 0:
    return 3;
  case 12:
    return 4;
  case 13:
    return 5;
  case 4:
    return 6;
  case 5:
    return 7;
  }
}

#endif

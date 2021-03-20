#pragma once

#if defined(ARDUINO_AVR_NANO_EVERY) && !defined(analogInputToDigitalPin)
  // Arduino nano every helper code
  // For some reason this is not defined for the arduino nano every...
  #define analogInputToDigitalPin(pin) (pin + 14)
#endif // ARDUINO_AVR_NANO_EVERY

#if defined(CORE_TEENSY)
  // Teensy USB serial uses different HW serial class
  // Comment this out if using another UART port on the teensy
  #define HW_SERIAL_T usb_serial_class
#endif

// Default settings
#ifndef HW_SERIAL_T
  #define HW_SERIAL_T HardwareSerial
#endif

#pragma once

#if defined(ARDUINO_AVR_NANO_EVERY) && !defined(analogInputToDigitalPin)
    // Arduino nano every helper code
    // For some reason this is not defined for the arduino nano every...
    #define analogInputToDigitalPin(pin) (pin + 14)

#endif // ARDUINO_AVR_NANO_EVERY

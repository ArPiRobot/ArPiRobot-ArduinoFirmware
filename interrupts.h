/*
 * Copyright 2021 Marcus Behel
 *
 * This file is part of ArPiRobot-ArduinoFirmware.
 * 
 * ArPiRobot-ArduinoFirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * ArPiRobot-ArduinoFirmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware. If not, see <https://www.gnu.org/licenses/>. 
 */

#pragma once

#include <Arduino.h>
#include "board.h"

// From https://github.com/swansontec/map-macro
// Used to recursively call PIN_ISR_DECLARE and PIN_ISR_DEFINE macros using preprocessor
#include "preprocessor_map.h"

// Pin ISR flag name
#define INTERRUPTS_FLAG_NAME(x)             interrupts_##x##_flag

// Declares pin ISR flags (used in header)
#define INTERRUPTS_FLAG_DECLARE(x)          extern uint8_t INTERRUPTS_FLAG_NAME(x);

// Defines pin ISR flags (used in source file)
#define INTERRUPTS_FLAG_DEFINE(x)           uint8_t INTERRUPTS_FLAG_NAME(x) = 0;


// Pin ISR function name
#define INTERRUPTS_ISR_NAME(x)              interrupts_##x##_isr

// Declares pin ISR functions (used in header)
#define INTERRUPTS_ISR_DECLARE(x)           void INTERRUPTS_ISR_NAME(x) ();

// Defines pin ISR functions (used in source file)
#define INTERRUPTS_ISR_DEFINE(x)            void INTERRUPTS_ISR_NAME(x) () { \
                                                delay(1); \
                                            }


// Used in check function to determine if a flag is set
#define INTERRUPTS_CHECK_ROUTINE(x)         if(INTERRUPTS_FLAG_NAME(x)) { \
                                                *pin = x; \
                                                return 1; \
                                            }

// Used in enable interrupts function to enable an interrupt
#define INTERRUPTS_ENABLE_CASE(x)           case x: attachInterrupt(digitalPinToInterrupt(x), &INTERRUPTS_ISR_NAME(x), mode); break;

// Used in disable interrupts function to disable an interrupt
#define INTERRUPTS_DISABLE_CASE(x)          case x: detachInterrupt(digitalPinToInterrupt(x)); break;


// Calls INTERRUPTS_FLAG_DECLARE (in preprocessor) for each pin in INTERRUPT_PINS
MAP(INTERRUPTS_FLAG_DECLARE, INTERRUPT_PINS)


/**
 * Setup anything needed for interrupts
 */
void interrupts_init();

/**
 * Checks for interrupt flags that have been set. 
 * Gives the pin number for an interrupt that has occured.
 * Only provides one pin number at a time.
 *
 * @param pin Pinter to uint8_t to store the pin number in
 * @return 1 if any pin interrupted, else 0
 */
bool interrupts_check_flags(uint8_t *pin);

/**
 * Enables the interrupt for the given pin
 * 
 * @param pin The pin to enable the interrupt for
 * @param mode The mode for the interrupt (LOW, CHANGE, RISING, FALLING, HIGH)
 */
void interrupts_enable_pin(uint8_t pin, int mode);

/**
 * Disables the interrupt for the given pin
 * 
 * @param pin The pin to disable the interrupt for
 */
void interrupts_disable_pin(uint8_t pin);


// Calls INTERRUPTS_ISR_DECLARE (in preprocessor) for each pin in INTERRUPT_PINS
MAP(INTERRUPTS_ISR_DECLARE, INTERRUPT_PINS)

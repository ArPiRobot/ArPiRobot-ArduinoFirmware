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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// From https://github.com/swansontec/map-macro
// Used to recursively call PIN_ISR_DECLARE and PIN_ISR_DEFINE macros using preprocessor
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Created by William Swanson in 2012.
 *
 * I, William Swanson, dedicate this work to the public domain.
 * I waive all rights to the work worldwide under copyright law,
 * including all related and neighboring rights,
 * to the extent allowed by law.
 *
 * You can copy, modify, distribute and perform the work,
 * even for commercial purposes, all without asking permission.
 */

#ifndef MAP_H_INCLUDED
#define MAP_H_INCLUDED

#define EVAL0(...) __VA_ARGS__
#define EVAL1(...) EVAL0(EVAL0(EVAL0(__VA_ARGS__)))
#define EVAL2(...) EVAL1(EVAL1(EVAL1(__VA_ARGS__)))
#define EVAL3(...) EVAL2(EVAL2(EVAL2(__VA_ARGS__)))
#define EVAL4(...) EVAL3(EVAL3(EVAL3(__VA_ARGS__)))
#define EVAL(...)  EVAL4(EVAL4(EVAL4(__VA_ARGS__)))

#define MAP_END(...)
#define MAP_OUT
#define MAP_COMMA ,

#define MAP_GET_END2() 0, MAP_END
#define MAP_GET_END1(...) MAP_GET_END2
#define MAP_GET_END(...) MAP_GET_END1
#define MAP_NEXT0(test, next, ...) next MAP_OUT
#define MAP_NEXT1(test, next) MAP_NEXT0(test, next, 0)
#define MAP_NEXT(test, next)  MAP_NEXT1(MAP_GET_END test, next)

#define MAP0(f, x, peek, ...) f(x) MAP_NEXT(peek, MAP1)(f, peek, __VA_ARGS__)
#define MAP1(f, x, peek, ...) f(x) MAP_NEXT(peek, MAP0)(f, peek, __VA_ARGS__)

#define MAP_LIST_NEXT1(test, next) MAP_NEXT0(test, MAP_COMMA next, 0)
#define MAP_LIST_NEXT(test, next)  MAP_LIST_NEXT1(MAP_GET_END test, next)

#define MAP_LIST0(f, x, peek, ...) f(x) MAP_LIST_NEXT(peek, MAP_LIST1)(f, peek, __VA_ARGS__)
#define MAP_LIST1(f, x, peek, ...) f(x) MAP_LIST_NEXT(peek, MAP_LIST0)(f, peek, __VA_ARGS__)

/**
 * Applies the function macro `f` to each of the remaining parameters.
 */
#define MAP(f, ...) EVAL(MAP1(f, __VA_ARGS__, ()()(), ()()(), ()()(), 0))

/**
 * Applies the function macro `f` to each of the remaining parameters and
 * inserts commas between the results.
 */
#define MAP_LIST(f, ...) EVAL(MAP_LIST1(f, __VA_ARGS__, ()()(), ()()(), ()()(), 0))

#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
                                                INTERRUPTS_FLAG_NAME(x) = 1; \
                                            }


// Used in check function to determine if a flag is set
#define INTERRUPTS_CHECK_ROUTINE(x)         if(INTERRUPTS_FLAG_NAME(x)) { \
                                                INTERRUPTS_FLAG_NAME(x) = 0; \
                                                *pin = x; \
                                                return 1; \
                                            }

// Used in check function to determine if a flag is set
#define INTERRUPTS_CHECK_SPECIFIC(x)        case x: \
                                                if(INTERRUPTS_FLAG_NAME(x)){ \
                                                    INTERRUPTS_FLAG_NAME(x) = 0; \
                                                    return 1; \
                                                } \
                                                return 0; \

// Used in enable interrupts function to enable an interrupt
#define INTERRUPTS_ENABLE_CASE(x)           case x: \
                                                detachInterrupt(digitalPinToInterrupt(x)); \
                                                attachInterrupt(digitalPinToInterrupt(x), INTERRUPTS_ISR_NAME(x), mode); \
                                                LOG("IE "); \
                                                LOGLN(pin); \
                                                return true;

// Used in disable interrupts function to disable an interrupt
#define INTERRUPTS_DISABLE_CASE(x)          case x: \
                                                detachInterrupt(digitalPinToInterrupt(x)); \
                                                LOG("IE !"); \
                                                LOGLN(pin); \
                                                return true;


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
 * @return true if any pin interrupted, else false
 */
bool interrupts_check_flags(uint8_t *pin);

/**
 * Checks if a SPECIFIC pin's flag is set
 * @param pin The pin to check the flag for
 * @return true if the pin's flag is set
 */
bool interrupts_check_flag(uint8_t pin);

/**
 * Enables the interrupt for the given pin
 * 
 * @param pin The pin to enable the interrupt for
 * @param mode The mode for the interrupt (LOW, CHANGE, RISING, FALLING, HIGH)
 * @return true if the pin is a valid interrupt, else false
 */
bool interrupts_enable_pin(uint8_t pin, AI_MODE_T mode);

/**
 * Disables the interrupt for the given pin
 * 
 * @param pin The pin to disable the interrupt for
 * @return true if the pin is a valid interrupt, else false
 */
bool interrupts_disable_pin(uint8_t pin);


// Calls INTERRUPTS_ISR_DECLARE (in preprocessor) for each pin in INTERRUPT_PINS
MAP(INTERRUPTS_ISR_DECLARE, INTERRUPT_PINS)

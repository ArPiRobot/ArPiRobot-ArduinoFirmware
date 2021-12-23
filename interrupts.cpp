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

#include "interrupts.h"
#include "settings.h"

// Calls INTERRUPTS_FLAG_DEFINE (in preprocessor) for each pin in INTERRUPT_PINS
MAP(INTERRUPTS_FLAG_DEFINE, INTERRUPT_PINS)


void interrupts_init(void){
    // Currently nothing here, but it is possible this will be useful for some board
}

bool interrupts_check_flags(uint8_t *pin) {
    // Calls INTERRUPTS_CHECK_ROUTINE (in preprocessor) for each pin in INTERRUPT_PINS
    MAP(INTERRUPTS_CHECK_ROUTINE, INTERRUPT_PINS)
    return false;
}

bool interrupts_enable_pin(uint8_t pin, AI_MODE_T mode){
    switch(pin){
    // Uses preprocessor to generate a case for each pin in INTERRUPT_PINS
    MAP(INTERRUPTS_ENABLE_CASE, INTERRUPT_PINS)
    default: 
        // Interrupt Enable Bad Pin
        LOG("IEBP "); 
        LOGLN(pin);
        return false;
    }
}

bool interrupts_disable_pin(uint8_t pin){
    switch(pin){
    // Uses preprocessor to generate a case for each pin in INTERRUPT_PINS
    MAP(INTERRUPTS_DISABLE_CASE, INTERRUPT_PINS)
    default: 
        // Interrupt Disable Bad Pin
        LOG("IEBP !"); 
        LOGLN(pin);
        return false;
    }
}

// Calls PIN_ISR_DEFINE (in preprocessor) for each pin in INTERRUPT_PINS
MAP(INTERRUPTS_ISR_DEFINE, INTERRUPT_PINS)

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


// Calls INTERRUPTS_FLAG_DEFINE (in preprocessor) for each pin in INTERRUPT_PINS
MAP(INTERRUPTS_FLAG_DEFINE, INTERRUPT_PINS)


bool interrupts_check_flags(uint8_t *pin) {
    // Calls INTERRUPTS_CHECK_ROUTINE (in preprocessor) for each pin in INTERRUPT_PINS
    MAP(INTERRUPTS_CHECK_ROUTINE, INTERRUPT_PINS)
}

void interrupts_enable_pin(uint8_t pin, int mode){
    switch(pin){
    // Uses preprocessor to generate a case for each pin in INTERRUPT_PINS
    MAP(INTERRUPTS_ENABLE_CASE, INTERRUPT_PINS)
    }
}

void interrupts_disable_pin(uint8_t pin){

}

// Calls PIN_ISR_DEFINE (in preprocessor) for each pin in INTERRUPT_PINS
MAP(INTERRUPTS_ISR_DEFINE, INTERRUPT_PINS)


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

class Interrupts{
public:
    /**
     * Enable the interrupt for the given pin
     * 
     * @param pin Pin to enable the interrupt for
     * @param mode Interrupt mode (CHANGE, RISING, FALLING, LOW, HIGH)
     * @param function The function to run when the interrupt occurs. Takes a void pointer argument
     * @param user_data Optional void pointer to pass to the given function when the interrupt occurs
     * @return true If the interrupt is enabled successfully, else false.
     */
    static bool enable(uint8_t pin, AI_MODE_T mode, void (*function)(void*), void *user_data);

    /**
     * Disable the interrupt for the given pin
     * 
     * @param pin Pin to disable the interrupt for
     * @return true If the interrupt is disabled successfully, else false.
     */
    static bool disable(uint8_t pin);

    /**
     * Check if the given pin is an interrupt.
     * 
     * @param pin The pin to check
     * @return true if the pin is an interrupt, else false
     */
    static bool isInterrupt(uint8_t pin);

private:

#if NUM_INTERRUPTS > 0
    static void isr0(void);        
#endif // NUM_INTERRUPTS > 0

#if NUM_INTERRUPTS > 1      
    static void isr1(void);        
#endif // NUM_INTERRUPTS > 1

#if NUM_INTERRUPTS > 2      
    static void isr2(void);        
#endif // NUM_INTERRUPTS > 2

#if NUM_INTERRUPTS > 3
    static void isr3(void);
#endif // NUM_INTERRUPTS > 3

#if NUM_INTERRUPTS > 4
    static void isr4(void);
#endif // NUM_INTERRUPTS > 4

#if NUM_INTERRUPTS > 5
    static void isr5(void);
#endif // NUM_INTERRUPTS > 5

#if NUM_INTERRUPTS > 6
    static void isr6(void);
#endif // NUM_INTERRUPTS > 6

#if NUM_INTERRUPTS > 7
    static void isr7(void);
#endif // NUM_INTERRUPTS > 7

#if NUM_INTERRUPTS > 8
    static void isr8(void);
#endif // NUM_INTERRUPTS > 8

#if NUM_INTERRUPTS > 9
    static void isr9(void);
#endif // NUM_INTERRUPTS > 9

#if NUM_INTERRUPTS > 10
    static void isr10(void);
#endif // NUM_INTERRUPTS > 10

#if NUM_INTERRUPTS > 11
    static void isr11(void);
#endif // NUM_INTERRUPTS > 11

#if NUM_INTERRUPTS > 12
    static void isr12(void);
#endif // NUM_INTERRUPTS > 12

#if NUM_INTERRUPTS > 13
    static void isr13(void);
#endif // NUM_INTERRUPTS > 13

#if NUM_INTERRUPTS > 14
    static void isr14(void);
#endif // NUM_INTERRUPTS > 14

#if NUM_INTERRUPTS > 15
    static void isr15(void);
#endif // NUM_INTERRUPTS > 15

#if NUM_INTERRUPTS > 16
    static void isr16(void);
#endif // NUM_INTERRUPTS > 16

#if NUM_INTERRUPTS > 17
    static void isr17(void);
#endif // NUM_INTERRUPTS > 17

#if NUM_INTERRUPTS > 18
    static void isr18(void);
#endif // NUM_INTERRUPTS > 18

#if NUM_INTERRUPTS > 19
    static void isr19(void);
#endif // NUM_INTERRUPTS > 19

#if NUM_INTERRUPTS > 20
    static void isr20(void);
#endif // NUM_INTERRUPTS > 20

#if NUM_INTERRUPTS > 21
    static void isr21(void);
#endif // NUM_INTERRUPTS > 21

#if NUM_INTERRUPTS > 22
    static void isr22(void);
#endif // NUM_INTERRUPTS > 22

#if NUM_INTERRUPTS > 23
    static void isr23(void);
#endif // NUM_INTERRUPTS > 23

#if NUM_INTERRUPTS > 24
    static void isr24(void);
#endif // NUM_INTERRUPTS > 24

#if NUM_INTERRUPTS > 25
    static void isr25(void);
#endif // NUM_INTERRUPTS > 25

#if NUM_INTERRUPTS > 26
    static void isr26(void);
#endif // NUM_INTERRUPTS > 26

#if NUM_INTERRUPTS > 27
    static void isr27(void);
#endif // NUM_INTERRUPTS > 27

#if NUM_INTERRUPTS > 28
    static void isr28(void);
#endif // NUM_INTERRUPTS > 28

#if NUM_INTERRUPTS > 29
    static void isr29(void);
#endif // NUM_INTERRUPTS > 29

#if NUM_INTERRUPTS > 30
    static void isr30(void);
#endif // NUM_INTERRUPTS > 30

#if NUM_INTERRUPTS > 31
    static void isr31(void);
#endif // NUM_INTERRUPTS > 31

#if NUM_INTERRUPTS > 32
    static void isr32(void);
#endif // NUM_INTERRUPTS > 32

#if NUM_INTERRUPTS > 33
    static void isr33(void);
#endif // NUM_INTERRUPTS > 33

#if NUM_INTERRUPTS > 0
    static void (* volatile userFunctions[NUM_INTERRUPTS])(void*);
    static void * volatile userData[NUM_INTERRUPTS];
#endif
};

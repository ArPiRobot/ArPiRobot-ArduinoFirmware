
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
    static void ISR_ATTRS isr0(void);        
#endif // NUM_INTERRUPTS > 0

#if NUM_INTERRUPTS > 1      
    static void ISR_ATTRS isr1(void);        
#endif // NUM_INTERRUPTS > 1

#if NUM_INTERRUPTS > 2      
    static void ISR_ATTRS isr2(void);        
#endif // NUM_INTERRUPTS > 2

#if NUM_INTERRUPTS > 3
    static void ISR_ATTRS isr3(void);
#endif // NUM_INTERRUPTS > 3

#if NUM_INTERRUPTS > 4
    static void ISR_ATTRS isr4(void);
#endif // NUM_INTERRUPTS > 4

#if NUM_INTERRUPTS > 5
    static void ISR_ATTRS isr5(void);
#endif // NUM_INTERRUPTS > 5

#if NUM_INTERRUPTS > 6
    static void ISR_ATTRS isr6(void);
#endif // NUM_INTERRUPTS > 6

#if NUM_INTERRUPTS > 7
    static void ISR_ATTRS isr7(void);
#endif // NUM_INTERRUPTS > 7

#if NUM_INTERRUPTS > 8
    static void ISR_ATTRS isr8(void);
#endif // NUM_INTERRUPTS > 8

#if NUM_INTERRUPTS > 9
    static void ISR_ATTRS isr9(void);
#endif // NUM_INTERRUPTS > 9

#if NUM_INTERRUPTS > 10
    static void ISR_ATTRS isr10(void);
#endif // NUM_INTERRUPTS > 10

#if NUM_INTERRUPTS > 11
    static void ISR_ATTRS isr11(void);
#endif // NUM_INTERRUPTS > 11

#if NUM_INTERRUPTS > 12
    static void ISR_ATTRS isr12(void);
#endif // NUM_INTERRUPTS > 12

#if NUM_INTERRUPTS > 13
    static void ISR_ATTRS isr13(void);
#endif // NUM_INTERRUPTS > 13

#if NUM_INTERRUPTS > 14
    static void ISR_ATTRS isr14(void);
#endif // NUM_INTERRUPTS > 14

#if NUM_INTERRUPTS > 15
    static void ISR_ATTRS isr15(void);
#endif // NUM_INTERRUPTS > 15

#if NUM_INTERRUPTS > 16
    static void ISR_ATTRS isr16(void);
#endif // NUM_INTERRUPTS > 16

#if NUM_INTERRUPTS > 17
    static void ISR_ATTRS isr17(void);
#endif // NUM_INTERRUPTS > 17

#if NUM_INTERRUPTS > 18
    static void ISR_ATTRS isr18(void);
#endif // NUM_INTERRUPTS > 18

#if NUM_INTERRUPTS > 19
    static void ISR_ATTRS isr19(void);
#endif // NUM_INTERRUPTS > 19

#if NUM_INTERRUPTS > 20
    static void ISR_ATTRS isr20(void);
#endif // NUM_INTERRUPTS > 20

#if NUM_INTERRUPTS > 21
    static void ISR_ATTRS isr21(void);
#endif // NUM_INTERRUPTS > 21

#if NUM_INTERRUPTS > 22
    static void ISR_ATTRS isr22(void);
#endif // NUM_INTERRUPTS > 22

#if NUM_INTERRUPTS > 23
    static void ISR_ATTRS isr23(void);
#endif // NUM_INTERRUPTS > 23

#if NUM_INTERRUPTS > 24
    static void ISR_ATTRS isr24(void);
#endif // NUM_INTERRUPTS > 24

#if NUM_INTERRUPTS > 25
    static void ISR_ATTRS isr25(void);
#endif // NUM_INTERRUPTS > 25

#if NUM_INTERRUPTS > 26
    static void ISR_ATTRS isr26(void);
#endif // NUM_INTERRUPTS > 26

#if NUM_INTERRUPTS > 27
    static void ISR_ATTRS isr27(void);
#endif // NUM_INTERRUPTS > 27

#if NUM_INTERRUPTS > 28
    static void ISR_ATTRS isr28(void);
#endif // NUM_INTERRUPTS > 28

#if NUM_INTERRUPTS > 29
    static void ISR_ATTRS isr29(void);
#endif // NUM_INTERRUPTS > 29

#if NUM_INTERRUPTS > 30
    static void ISR_ATTRS isr30(void);
#endif // NUM_INTERRUPTS > 30

#if NUM_INTERRUPTS > 31
    static void ISR_ATTRS isr31(void);
#endif // NUM_INTERRUPTS > 31

#if NUM_INTERRUPTS > 32
    static void ISR_ATTRS isr32(void);
#endif // NUM_INTERRUPTS > 32

#if NUM_INTERRUPTS > 33
    static void ISR_ATTRS isr33(void);
#endif // NUM_INTERRUPTS > 33

#if NUM_INTERRUPTS > 34
    static void ISR_ATTRS isr34(void);
#endif // NUM_INTERRUPTS > 34

#if NUM_INTERRUPTS > 35
    static void ISR_ATTRS isr35(void);
#endif // NUM_INTERRUPTS > 35

#if NUM_INTERRUPTS > 36
    static void ISR_ATTRS isr36(void);
#endif // NUM_INTERRUPTS > 36

#if NUM_INTERRUPTS > 37
    static void ISR_ATTRS isr37(void);
#endif // NUM_INTERRUPTS > 37

#if NUM_INTERRUPTS > 38
    static void ISR_ATTRS isr38(void);
#endif // NUM_INTERRUPTS > 38

#if NUM_INTERRUPTS > 39
    static void ISR_ATTRS isr39(void);
#endif // NUM_INTERRUPTS > 39

#if NUM_INTERRUPTS > 40
    static void ISR_ATTRS isr40(void);
#endif // NUM_INTERRUPTS > 40

#if NUM_INTERRUPTS > 41
    static void ISR_ATTRS isr41(void);
#endif // NUM_INTERRUPTS > 41

#if NUM_INTERRUPTS > 42
    static void ISR_ATTRS isr42(void);
#endif // NUM_INTERRUPTS > 42

#if NUM_INTERRUPTS > 43
    static void ISR_ATTRS isr43(void);
#endif // NUM_INTERRUPTS > 43

#if NUM_INTERRUPTS > 44
    static void ISR_ATTRS isr44(void);
#endif // NUM_INTERRUPTS > 44

#if NUM_INTERRUPTS > 45
    static void ISR_ATTRS isr45(void);
#endif // NUM_INTERRUPTS > 45


#if NUM_INTERRUPTS > 0
    static void (* volatile userFunctions[NUM_INTERRUPTS])(void*);
    static void * volatile userData[NUM_INTERRUPTS];
#endif
};

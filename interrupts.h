
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

namespace Interrupts{

    ////////////////////////////////////////////////////////////////////////////
    /// Globals
    ////////////////////////////////////////////////////////////////////////////

#if NUM_INTERRUPTS > 0
    extern void (* volatile userFunctions[NUM_INTERRUPTS])(void*);
    extern void * volatile userData[NUM_INTERRUPTS];
#endif

    ////////////////////////////////////////////////////////////////////////////
    /// Functions
    ////////////////////////////////////////////////////////////////////////////

    void init();

    bool enable(uint8_t pin, AI_MODE_T mode, void (*function)(void*), void *user_data);

    bool disable(uint8_t pin);

    bool isInterrupt(uint8_t pin);


    ////////////////////////////////////////////////////////////////////////////
    /// ISRs
    ////////////////////////////////////////////////////////////////////////////

#if NUM_INTERRUPTS > 0
    void isr0(void);        
#endif // NUM_INTERRUPTS > 0

#if NUM_INTERRUPTS > 1      
    void isr1(void);        
#endif // NUM_INTERRUPTS > 1

#if NUM_INTERRUPTS > 2      
    void isr2(void);        
#endif // NUM_INTERRUPTS > 2

#if NUM_INTERRUPTS > 3
    void isr3(void);
#endif // NUM_INTERRUPTS > 3

#if NUM_INTERRUPTS > 4
    void isr4(void);
#endif // NUM_INTERRUPTS > 4

#if NUM_INTERRUPTS > 5
    void isr5(void);
#endif // NUM_INTERRUPTS > 5

#if NUM_INTERRUPTS > 6
    void isr6(void);
#endif // NUM_INTERRUPTS > 6

#if NUM_INTERRUPTS > 7
    void isr7(void);
#endif // NUM_INTERRUPTS > 7

#if NUM_INTERRUPTS > 8
    void isr8(void);
#endif // NUM_INTERRUPTS > 8

#if NUM_INTERRUPTS > 9
    void isr9(void);
#endif // NUM_INTERRUPTS > 9

#if NUM_INTERRUPTS > 10
    void isr10(void);
#endif // NUM_INTERRUPTS > 10

#if NUM_INTERRUPTS > 11
    void isr11(void);
#endif // NUM_INTERRUPTS > 11

#if NUM_INTERRUPTS > 12
    void isr12(void);
#endif // NUM_INTERRUPTS > 12

#if NUM_INTERRUPTS > 13
    void isr13(void);
#endif // NUM_INTERRUPTS > 13

#if NUM_INTERRUPTS > 14
    void isr14(void);
#endif // NUM_INTERRUPTS > 14

#if NUM_INTERRUPTS > 15
    void isr15(void);
#endif // NUM_INTERRUPTS > 15

#if NUM_INTERRUPTS > 16
    void isr16(void);
#endif // NUM_INTERRUPTS > 16

#if NUM_INTERRUPTS > 17
    void isr17(void);
#endif // NUM_INTERRUPTS > 17

#if NUM_INTERRUPTS > 18
    void isr18(void);
#endif // NUM_INTERRUPTS > 18

#if NUM_INTERRUPTS > 19
    void isr19(void);
#endif // NUM_INTERRUPTS > 19

#if NUM_INTERRUPTS > 20
    void isr20(void);
#endif // NUM_INTERRUPTS > 20

#if NUM_INTERRUPTS > 21
    void isr21(void);
#endif // NUM_INTERRUPTS > 21

#if NUM_INTERRUPTS > 22
    void isr22(void);
#endif // NUM_INTERRUPTS > 22

#if NUM_INTERRUPTS > 23
    void isr23(void);
#endif // NUM_INTERRUPTS > 23

#if NUM_INTERRUPTS > 24
    void isr24(void);
#endif // NUM_INTERRUPTS > 24

#if NUM_INTERRUPTS > 25
    void isr25(void);
#endif // NUM_INTERRUPTS > 25

#if NUM_INTERRUPTS > 26
    void isr26(void);
#endif // NUM_INTERRUPTS > 26

#if NUM_INTERRUPTS > 27
    void isr27(void);
#endif // NUM_INTERRUPTS > 27

#if NUM_INTERRUPTS > 28
    void isr28(void);
#endif // NUM_INTERRUPTS > 28

#if NUM_INTERRUPTS > 29
    void isr29(void);
#endif // NUM_INTERRUPTS > 29

#if NUM_INTERRUPTS > 30
    void isr30(void);
#endif // NUM_INTERRUPTS > 30

#if NUM_INTERRUPTS > 31
    void isr31(void);
#endif // NUM_INTERRUPTS > 31

#if NUM_INTERRUPTS > 32
    void isr32(void);
#endif // NUM_INTERRUPTS > 32

#if NUM_INTERRUPTS > 33
    void isr33(void);
#endif // NUM_INTERRUPTS > 33

}

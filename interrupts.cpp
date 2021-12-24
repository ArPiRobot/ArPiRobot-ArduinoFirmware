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


////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////
void (* volatile interrupts_functions[NUM_INTERRUPTS])(void*);
void * volatile interrupts_data[NUM_INTERRUPTS];


////////////////////////////////////////////////////////////////////////////////
/// ISR's
////////////////////////////////////////////////////////////////////////////////

#if NUM_INTERRUPTS > 0
void interrupts_isr0(void){
    (*interrupts_functions[0])(interrupts_data[0]);
}
#endif // NUM_INTERRUPTS > 0

#if NUM_INTERRUPTS > 1
void interrupts_isr1(void){
    (*interrupts_functions[1])(interrupts_data[1]);
}
#endif // NUM_INTERRUPTS > 1

#if NUM_INTERRUPTS > 2
void interrupts_isr2(void){
    (*interrupts_functions[2])(interrupts_data[2]);
}
#endif // NUM_INTERRUPTS > 2

#if NUM_INTERRUPTS > 3
void interrupts_isr3(void){
    (*interrupts_functions[3])(interrupts_data[3]);
}
#endif // NUM_INTERRUPTS > 3

#if NUM_INTERRUPTS > 4
void interrupts_isr4(void){
    (*interrupts_functions[4])(interrupts_data[4]);
}
#endif // NUM_INTERRUPTS > 4

#if NUM_INTERRUPTS > 5
void interrupts_isr5(void){
    (*interrupts_functions[5])(interrupts_data[5]);
}
#endif // NUM_INTERRUPTS > 5

#if NUM_INTERRUPTS > 6
void interrupts_isr6(void){
    (*interrupts_functions[6])(interrupts_data[6]);
}
#endif // NUM_INTERRUPTS > 6

#if NUM_INTERRUPTS > 7
void interrupts_isr7(void){
    (*interrupts_functions[7])(interrupts_data[7]);
}
#endif // NUM_INTERRUPTS > 7

#if NUM_INTERRUPTS > 8
void interrupts_isr8(void){
    (*interrupts_functions[8])(interrupts_data[8]);
}
#endif // NUM_INTERRUPTS > 8

#if NUM_INTERRUPTS > 9
void interrupts_isr9(void){
    (*interrupts_functions[9])(interrupts_data[9]);
}
#endif // NUM_INTERRUPTS > 9

#if NUM_INTERRUPTS > 10
void interrupts_isr10(void){
    (*interrupts_functions[10])(interrupts_data[10]);
}
#endif // NUM_INTERRUPTS > 10

#if NUM_INTERRUPTS > 11
void interrupts_isr11(void){
    (*interrupts_functions[11])(interrupts_data[11]);
}
#endif // NUM_INTERRUPTS > 11

#if NUM_INTERRUPTS > 12
void interrupts_isr12(void){
    (*interrupts_functions[12])(interrupts_data[12]);
}
#endif // NUM_INTERRUPTS > 12

#if NUM_INTERRUPTS > 13
void interrupts_isr13(void){
    (*interrupts_functions[13])(interrupts_data[13]);
}
#endif // NUM_INTERRUPTS > 13

#if NUM_INTERRUPTS > 14
void interrupts_isr14(void){
    (*interrupts_functions[14])(interrupts_data[14]);
}
#endif // NUM_INTERRUPTS > 14

#if NUM_INTERRUPTS > 15
void interrupts_isr15(void){
    (*interrupts_functions[15])(interrupts_data[15]);
}
#endif // NUM_INTERRUPTS > 15

#if NUM_INTERRUPTS > 16
void interrupts_isr16(void){
    (*interrupts_functions[16])(interrupts_data[16]);
}
#endif // NUM_INTERRUPTS > 16

#if NUM_INTERRUPTS > 17
void interrupts_isr17(void){
    (*interrupts_functions[17])(interrupts_data[17]);
}
#endif // NUM_INTERRUPTS > 17

#if NUM_INTERRUPTS > 18
void interrupts_isr18(void){
    (*interrupts_functions[18])(interrupts_data[18]);
}
#endif // NUM_INTERRUPTS > 18

#if NUM_INTERRUPTS > 19
void interrupts_isr19(void){
    (*interrupts_functions[19])(interrupts_data[19]);
}
#endif // NUM_INTERRUPTS > 19

#if NUM_INTERRUPTS > 20
void interrupts_isr20(void){
    (*interrupts_functions[20])(interrupts_data[20]);
}
#endif // NUM_INTERRUPTS > 20

#if NUM_INTERRUPTS > 21
void interrupts_isr21(void){
    (*interrupts_functions[21])(interrupts_data[21]);
}
#endif // NUM_INTERRUPTS > 21

#if NUM_INTERRUPTS > 22
void interrupts_isr22(void){
    (*interrupts_functions[22])(interrupts_data[22]);
}
#endif // NUM_INTERRUPTS > 22

#if NUM_INTERRUPTS > 23
void interrupts_isr23(void){
    (*interrupts_functions[23])(interrupts_data[23]);
}
#endif // NUM_INTERRUPTS > 23

#if NUM_INTERRUPTS > 24
void interrupts_isr24(void){
    (*interrupts_functions[24])(interrupts_data[24]);
}
#endif // NUM_INTERRUPTS > 24

#if NUM_INTERRUPTS > 25
void interrupts_isr25(void){
    (*interrupts_functions[25])(interrupts_data[25]);
}
#endif // NUM_INTERRUPTS > 25

#if NUM_INTERRUPTS > 26
void interrupts_isr26(void){
    (*interrupts_functions[26])(interrupts_data[26]);
}
#endif // NUM_INTERRUPTS > 26

#if NUM_INTERRUPTS > 27
void interrupts_isr27(void){
    (*interrupts_functions[27])(interrupts_data[27]);
}
#endif // NUM_INTERRUPTS > 27

#if NUM_INTERRUPTS > 28
void interrupts_isr28(void){
    (*interrupts_functions[28])(interrupts_data[28]);
}
#endif // NUM_INTERRUPTS > 28

#if NUM_INTERRUPTS > 29
void interrupts_isr29(void){
    (*interrupts_functions[29])(interrupts_data[29]);
}
#endif // NUM_INTERRUPTS > 29

#if NUM_INTERRUPTS > 30
void interrupts_isr30(void){
    (*interrupts_functions[30])(interrupts_data[30]);
}
#endif // NUM_INTERRUPTS > 30

#if NUM_INTERRUPTS > 31
void interrupts_isr31(void){
    (*interrupts_functions[31])(interrupts_data[31]);
}
#endif // NUM_INTERRUPTS > 31

#if NUM_INTERRUPTS > 32
void interrupts_isr32(void){
    (*interrupts_functions[32])(interrupts_data[32]);
}
#endif // NUM_INTERRUPTS > 32

#if NUM_INTERRUPTS > 33
void interrupts_isr33(void){
    (*interrupts_functions[33])(interrupts_data[33]);
}
#endif // NUM_INTERRUPTS > 33


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

bool interrupts_enable(uint8_t pin, AI_MODE_T mode, void (*function)(void*), void *user_data){
    uint8_t interrupt = digitalPinToInterrupt(pin);
    if(interrupt == NOT_AN_INTERRUPT || interrupt > NUM_INTERRUPTS){
        return false;
    }
    interrupts_functions[interrupt] = function;
    interrupts_data[interrupt] = user_data;
    detachInterrupt(interrupt);
    switch(interrupt){
#if NUM_INTERRUPTS > 0
    case 0:
        attachInterrupt(interrupt, &interrupts_isr0, mode);
        return true;
#endif // NUM_INTERRUPTS > 0

#if NUM_INTERRUPTS > 1
    case 1:
        attachInterrupt(interrupt, &interrupts_isr1, mode);
        return true;
#endif // NUM_INTERRUPTS > 1

#if NUM_INTERRUPTS > 2
    case 2:
        attachInterrupt(interrupt, &interrupts_isr2, mode);
        return true;
#endif // NUM_INTERRUPTS > 2

#if NUM_INTERRUPTS > 3
    case 3:
        attachInterrupt(interrupt, &interrupts_isr3, mode);
        return true;
#endif // NUM_INTERRUPTS > 3

#if NUM_INTERRUPTS > 4
    case 4:
        attachInterrupt(interrupt, &interrupts_isr4, mode);
        return true;
#endif // NUM_INTERRUPTS > 4

#if NUM_INTERRUPTS > 5
    case 5:
        attachInterrupt(interrupt, &interrupts_isr5, mode);
        return true;
#endif // NUM_INTERRUPTS > 5

#if NUM_INTERRUPTS > 6
    case 6:
        attachInterrupt(interrupt, &interrupts_isr6, mode);
        return true;
#endif // NUM_INTERRUPTS > 6

#if NUM_INTERRUPTS > 7
    case 7:
        attachInterrupt(interrupt, &interrupts_isr7, mode);
        return true;
#endif // NUM_INTERRUPTS > 7

#if NUM_INTERRUPTS > 8
    case 8:
        attachInterrupt(interrupt, &interrupts_isr8, mode);
        return true;
#endif // NUM_INTERRUPTS > 8

#if NUM_INTERRUPTS > 9
    case 9:
        attachInterrupt(interrupt, &interrupts_isr9, mode);
        return true;
#endif // NUM_INTERRUPTS > 9

#if NUM_INTERRUPTS > 10
    case 10:
        attachInterrupt(interrupt, &interrupts_isr10, mode);
        return true;
#endif // NUM_INTERRUPTS > 10

#if NUM_INTERRUPTS > 11
    case 11:
        attachInterrupt(interrupt, &interrupts_isr11, mode);
        return true;
#endif // NUM_INTERRUPTS > 11

#if NUM_INTERRUPTS > 12
    case 12:
        attachInterrupt(interrupt, &interrupts_isr12, mode);
        return true;
#endif // NUM_INTERRUPTS > 12

#if NUM_INTERRUPTS > 13
    case 13:
        attachInterrupt(interrupt, &interrupts_isr13, mode);
        return true;
#endif // NUM_INTERRUPTS > 13

#if NUM_INTERRUPTS > 14
    case 14:
        attachInterrupt(interrupt, &interrupts_isr14, mode);
        return true;
#endif // NUM_INTERRUPTS > 14

#if NUM_INTERRUPTS > 15
    case 15:
        attachInterrupt(interrupt, &interrupts_isr15, mode);
        return true;
#endif // NUM_INTERRUPTS > 15

#if NUM_INTERRUPTS > 16
    case 16:
        attachInterrupt(interrupt, &interrupts_isr16, mode);
        return true;
#endif // NUM_INTERRUPTS > 16

#if NUM_INTERRUPTS > 17
    case 17:
        attachInterrupt(interrupt, &interrupts_isr17, mode);
        return true;
#endif // NUM_INTERRUPTS > 17

#if NUM_INTERRUPTS > 18
    case 18:
        attachInterrupt(interrupt, &interrupts_isr18, mode);
        return true;
#endif // NUM_INTERRUPTS > 18

#if NUM_INTERRUPTS > 19
    case 19:
        attachInterrupt(interrupt, &interrupts_isr19, mode);
        break;
#endif // NUM_INTERRUPTS > 19

#if NUM_INTERRUPTS > 20
    case 20:
        attachInterrupt(interrupt, &interrupts_isr20, mode);
        return true;
#endif // NUM_INTERRUPTS > 20

#if NUM_INTERRUPTS > 21
    case 21:
        attachInterrupt(interrupt, &interrupts_isr21, mode);
        return true;
#endif // NUM_INTERRUPTS > 21

#if NUM_INTERRUPTS > 22
    case 22:
        attachInterrupt(interrupt, &interrupts_isr22, mode);
        return true;
#endif // NUM_INTERRUPTS > 22

#if NUM_INTERRUPTS > 23
    case 23:
        attachInterrupt(interrupt, &interrupts_isr23, mode);
        return true;
#endif // NUM_INTERRUPTS > 23

#if NUM_INTERRUPTS > 24
    case 24:
        attachInterrupt(interrupt, &interrupts_isr24, mode);
        return true;
#endif // NUM_INTERRUPTS > 24

#if NUM_INTERRUPTS > 25
    case 25:
        attachInterrupt(interrupt, &interrupts_isr25, mode);
        return true;
#endif // NUM_INTERRUPTS > 25

#if NUM_INTERRUPTS > 26
    case 26:
        attachInterrupt(interrupt, &interrupts_isr26, mode);
        return true;
#endif // NUM_INTERRUPTS > 26

#if NUM_INTERRUPTS > 27
    case 27:
        attachInterrupt(interrupt, &interrupts_isr27, mode);
        return true;
#endif // NUM_INTERRUPTS > 27

#if NUM_INTERRUPTS > 28
    case 28:
        attachInterrupt(interrupt, &interrupts_isr28, mode);
        return true;
#endif // NUM_INTERRUPTS > 28

#if NUM_INTERRUPTS > 29
    case 29:
        attachInterrupt(interrupt, &interrupts_isr29, mode);
        return true;
#endif // NUM_INTERRUPTS > 29

#if NUM_INTERRUPTS > 30
    case 30:
        attachInterrupt(interrupt, &interrupts_isr30, mode);
        return true;
#endif // NUM_INTERRUPTS > 30

#if NUM_INTERRUPTS > 31
    case 31:
        attachInterrupt(interrupt, &interrupts_isr31, mode);
        return true;
#endif // NUM_INTERRUPTS > 31

#if NUM_INTERRUPTS > 32
    case 32:
        attachInterrupt(interrupt, &interrupts_isr32, mode);
        return true;
#endif // NUM_INTERRUPTS > 32

#if NUM_INTERRUPTS > 33
    case 33:
        attachInterrupt(interrupt, &interrupts_isr33, mode);
        return true;
#endif // NUM_INTERRUPTS > 33
    }
    return false;
}

bool interrupts_disable(uint8_t pin){
    uint8_t interrupt = digitalPinToInterrupt(pin);
    if(interrupt == NOT_AN_INTERRUPT || interrupt > NUM_INTERRUPTS){
        return false;
    }
    detachInterrupt(interrupt);
    return true;
}

bool interrupts_is_interrupt(uint8_t pin){
    uint8_t interrupt = digitalPinToInterrupt(pin);
    return !(interrupt == NOT_AN_INTERRUPT || interrupt > NUM_INTERRUPTS);
}

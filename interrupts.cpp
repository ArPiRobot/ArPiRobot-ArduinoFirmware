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

#if NUM_INTERRUPTS > 0
    void (* volatile Interrupts::userFunctions[NUM_INTERRUPTS])(void*);
    void * volatile Interrupts::userData[NUM_INTERRUPTS];
#endif


////////////////////////////////////////////////////////////////////////////////
/// ISR's
////////////////////////////////////////////////////////////////////////////////

#if NUM_INTERRUPTS > 0
void Interrupts::isr0(void){
    (*userFunctions[0])(userData[0]);
}
#endif // NUM_INTERRUPTS > 0

#if NUM_INTERRUPTS > 1
void Interrupts::isr1(void){
    (*userFunctions[1])(userData[1]);
}
#endif // NUM_INTERRUPTS > 1

#if NUM_INTERRUPTS > 2
void Interrupts::isr2(void){
    (*userFunctions[2])(userData[2]);
}
#endif // NUM_INTERRUPTS > 2

#if NUM_INTERRUPTS > 3
void Interrupts::isr3(void){
    (*userFunctions[3])(userData[3]);
}
#endif // NUM_INTERRUPTS > 3

#if NUM_INTERRUPTS > 4
void Interrupts::isr4(void){
    (*userFunctions[4])(userData[4]);
}
#endif // NUM_INTERRUPTS > 4

#if NUM_INTERRUPTS > 5
void Interrupts::isr5(void){
    (*userFunctions[5])(userData[5]);
}
#endif // NUM_INTERRUPTS > 5

#if NUM_INTERRUPTS > 6
void Interrupts::isr6(void){
    (*userFunctions[6])(userData[6]);
}
#endif // NUM_INTERRUPTS > 6

#if NUM_INTERRUPTS > 7
void Interrupts::isr7(void){
    (*userFunctions[7])(userData[7]);
}
#endif // NUM_INTERRUPTS > 7

#if NUM_INTERRUPTS > 8
void Interrupts::isr8(void){
    (*userFunctions[8])(userData[8]);
}
#endif // NUM_INTERRUPTS > 8

#if NUM_INTERRUPTS > 9
void Interrupts::isr9(void){
    (*userFunctions[9])(userData[9]);
}
#endif // NUM_INTERRUPTS > 9

#if NUM_INTERRUPTS > 10
void Interrupts::isr10(void){
    (*userFunctions[10])(userData[10]);
}
#endif // NUM_INTERRUPTS > 10

#if NUM_INTERRUPTS > 11
void Interrupts::isr11(void){
    (*userFunctions[11])(userData[11]);
}
#endif // NUM_INTERRUPTS > 11

#if NUM_INTERRUPTS > 12
void Interrupts::isr12(void){
    (*userFunctions[12])(userData[12]);
}
#endif // NUM_INTERRUPTS > 12

#if NUM_INTERRUPTS > 13
void Interrupts::isr13(void){
    (*userFunctions[13])(userData[13]);
}
#endif // NUM_INTERRUPTS > 13

#if NUM_INTERRUPTS > 14
void Interrupts::isr14(void){
    (*userFunctions[14])(userData[14]);
}
#endif // NUM_INTERRUPTS > 14

#if NUM_INTERRUPTS > 15
void Interrupts::isr15(void){
    (*userFunctions[15])(userData[15]);
}
#endif // NUM_INTERRUPTS > 15

#if NUM_INTERRUPTS > 16
void Interrupts::isr16(void){
    (*userFunctions[16])(userData[16]);
}
#endif // NUM_INTERRUPTS > 16

#if NUM_INTERRUPTS > 17
void Interrupts::isr17(void){
    (*userFunctions[17])(userData[17]);
}
#endif // NUM_INTERRUPTS > 17

#if NUM_INTERRUPTS > 18
void Interrupts::isr18(void){
    (*userFunctions[18])(userData[18]);
}
#endif // NUM_INTERRUPTS > 18

#if NUM_INTERRUPTS > 19
void Interrupts::isr19(void){
    (*userFunctions[19])(userData[19]);
}
#endif // NUM_INTERRUPTS > 19

#if NUM_INTERRUPTS > 20
void Interrupts::isr20(void){
    (*userFunctions[20])(userData[20]);
}
#endif // NUM_INTERRUPTS > 20

#if NUM_INTERRUPTS > 21
void Interrupts::isr21(void){
    (*userFunctions[21])(userData[21]);
}
#endif // NUM_INTERRUPTS > 21

#if NUM_INTERRUPTS > 22
void Interrupts::isr22(void){
    (*userFunctions[22])(userData[22]);
}
#endif // NUM_INTERRUPTS > 22

#if NUM_INTERRUPTS > 23
void Interrupts::isr23(void){
    (*userFunctions[23])(userData[23]);
}
#endif // NUM_INTERRUPTS > 23

#if NUM_INTERRUPTS > 24
void Interrupts::isr24(void){
    (*userFunctions[24])(userData[24]);
}
#endif // NUM_INTERRUPTS > 24

#if NUM_INTERRUPTS > 25
void Interrupts::isr25(void){
    (*userFunctions[25])(userData[25]);
}
#endif // NUM_INTERRUPTS > 25

#if NUM_INTERRUPTS > 26
void Interrupts::isr26(void){
    (*userFunctions[26])(userData[26]);
}
#endif // NUM_INTERRUPTS > 26

#if NUM_INTERRUPTS > 27
void Interrupts::isr27(void){
    (*userFunctions[27])(userData[27]);
}
#endif // NUM_INTERRUPTS > 27

#if NUM_INTERRUPTS > 28
void Interrupts::isr28(void){
    (*userFunctions[28])(userData[28]);
}
#endif // NUM_INTERRUPTS > 28

#if NUM_INTERRUPTS > 29
void Interrupts::isr29(void){
    (*userFunctions[29])(userData[29]);
}
#endif // NUM_INTERRUPTS > 29

#if NUM_INTERRUPTS > 30
void Interrupts::isr30(void){
    (*userFunctions[30])(userData[30]);
}
#endif // NUM_INTERRUPTS > 30

#if NUM_INTERRUPTS > 31
void Interrupts::isr31(void){
    (*userFunctions[31])(userData[31]);
}
#endif // NUM_INTERRUPTS > 31

#if NUM_INTERRUPTS > 32
void Interrupts::isr32(void){
    (*userFunctions[32])(userData[32]);
}
#endif // NUM_INTERRUPTS > 32

#if NUM_INTERRUPTS > 33
void Interrupts::isr33(void){
    (*userFunctions[33])(userData[33]);
}
#endif // NUM_INTERRUPTS > 33


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

void Interrupts::init(){
    // Nothing to do for now
    // This is included in case any boards need something here in the future
}

bool Interrupts::enable(uint8_t pin, AI_MODE_T mode, void (*function)(void*), void *user_data){
    uint8_t interrupt = digitalPinToInterrupt(pin);
    if(interrupt == NOT_AN_INTERRUPT || interrupt > NUM_INTERRUPTS){
        return false;
    }
#if NUM_INTERRUPTS > 0
    userFunctions[interrupt] = function;
    userData[interrupt] = user_data;
#endif
    detachInterrupt(interrupt);
    switch(interrupt){
#if NUM_INTERRUPTS > 0
    case 0:
        attachInterrupt(interrupt, &isr0, mode);
        return true;
#endif // NUM_INTERRUPTS > 0

#if NUM_INTERRUPTS > 1
    case 1:
        attachInterrupt(interrupt, &isr1, mode);
        return true;
#endif // NUM_INTERRUPTS > 1

#if NUM_INTERRUPTS > 2
    case 2:
        attachInterrupt(interrupt, &isr2, mode);
        return true;
#endif // NUM_INTERRUPTS > 2

#if NUM_INTERRUPTS > 3
    case 3:
        attachInterrupt(interrupt, &isr3, mode);
        return true;
#endif // NUM_INTERRUPTS > 3

#if NUM_INTERRUPTS > 4
    case 4:
        attachInterrupt(interrupt, &isr4, mode);
        return true;
#endif // NUM_INTERRUPTS > 4

#if NUM_INTERRUPTS > 5
    case 5:
        attachInterrupt(interrupt, &isr5, mode);
        return true;
#endif // NUM_INTERRUPTS > 5

#if NUM_INTERRUPTS > 6
    case 6:
        attachInterrupt(interrupt, &isr6, mode);
        return true;
#endif // NUM_INTERRUPTS > 6

#if NUM_INTERRUPTS > 7
    case 7:
        attachInterrupt(interrupt, &isr7, mode);
        return true;
#endif // NUM_INTERRUPTS > 7

#if NUM_INTERRUPTS > 8
    case 8:
        attachInterrupt(interrupt, &isr8, mode);
        return true;
#endif // NUM_INTERRUPTS > 8

#if NUM_INTERRUPTS > 9
    case 9:
        attachInterrupt(interrupt, &isr9, mode);
        return true;
#endif // NUM_INTERRUPTS > 9

#if NUM_INTERRUPTS > 10
    case 10:
        attachInterrupt(interrupt, &isr10, mode);
        return true;
#endif // NUM_INTERRUPTS > 10

#if NUM_INTERRUPTS > 11
    case 11:
        attachInterrupt(interrupt, &isr11, mode);
        return true;
#endif // NUM_INTERRUPTS > 11

#if NUM_INTERRUPTS > 12
    case 12:
        attachInterrupt(interrupt, &isr12, mode);
        return true;
#endif // NUM_INTERRUPTS > 12

#if NUM_INTERRUPTS > 13
    case 13:
        attachInterrupt(interrupt, &isr13, mode);
        return true;
#endif // NUM_INTERRUPTS > 13

#if NUM_INTERRUPTS > 14
    case 14:
        attachInterrupt(interrupt, &isr14, mode);
        return true;
#endif // NUM_INTERRUPTS > 14

#if NUM_INTERRUPTS > 15
    case 15:
        attachInterrupt(interrupt, &isr15, mode);
        return true;
#endif // NUM_INTERRUPTS > 15

#if NUM_INTERRUPTS > 16
    case 16:
        attachInterrupt(interrupt, &isr16, mode);
        return true;
#endif // NUM_INTERRUPTS > 16

#if NUM_INTERRUPTS > 17
    case 17:
        attachInterrupt(interrupt, &isr17, mode);
        return true;
#endif // NUM_INTERRUPTS > 17

#if NUM_INTERRUPTS > 18
    case 18:
        attachInterrupt(interrupt, &isr18, mode);
        return true;
#endif // NUM_INTERRUPTS > 18

#if NUM_INTERRUPTS > 19
    case 19:
        attachInterrupt(interrupt, &isr19, mode);
        break;
#endif // NUM_INTERRUPTS > 19

#if NUM_INTERRUPTS > 20
    case 20:
        attachInterrupt(interrupt, &isr20, mode);
        return true;
#endif // NUM_INTERRUPTS > 20

#if NUM_INTERRUPTS > 21
    case 21:
        attachInterrupt(interrupt, &isr21, mode);
        return true;
#endif // NUM_INTERRUPTS > 21

#if NUM_INTERRUPTS > 22
    case 22:
        attachInterrupt(interrupt, &isr22, mode);
        return true;
#endif // NUM_INTERRUPTS > 22

#if NUM_INTERRUPTS > 23
    case 23:
        attachInterrupt(interrupt, &isr23, mode);
        return true;
#endif // NUM_INTERRUPTS > 23

#if NUM_INTERRUPTS > 24
    case 24:
        attachInterrupt(interrupt, &isr24, mode);
        return true;
#endif // NUM_INTERRUPTS > 24

#if NUM_INTERRUPTS > 25
    case 25:
        attachInterrupt(interrupt, &isr25, mode);
        return true;
#endif // NUM_INTERRUPTS > 25

#if NUM_INTERRUPTS > 26
    case 26:
        attachInterrupt(interrupt, &isr26, mode);
        return true;
#endif // NUM_INTERRUPTS > 26

#if NUM_INTERRUPTS > 27
    case 27:
        attachInterrupt(interrupt, &isr27, mode);
        return true;
#endif // NUM_INTERRUPTS > 27

#if NUM_INTERRUPTS > 28
    case 28:
        attachInterrupt(interrupt, &isr28, mode);
        return true;
#endif // NUM_INTERRUPTS > 28

#if NUM_INTERRUPTS > 29
    case 29:
        attachInterrupt(interrupt, &isr29, mode);
        return true;
#endif // NUM_INTERRUPTS > 29

#if NUM_INTERRUPTS > 30
    case 30:
        attachInterrupt(interrupt, &isr30, mode);
        return true;
#endif // NUM_INTERRUPTS > 30

#if NUM_INTERRUPTS > 31
    case 31:
        attachInterrupt(interrupt, &isr31, mode);
        return true;
#endif // NUM_INTERRUPTS > 31

#if NUM_INTERRUPTS > 32
    case 32:
        attachInterrupt(interrupt, &isr32, mode);
        return true;
#endif // NUM_INTERRUPTS > 32

#if NUM_INTERRUPTS > 33
    case 33:
        attachInterrupt(interrupt, &isr33, mode);
        return true;
#endif // NUM_INTERRUPTS > 33
    }
    return false;
}

bool Interrupts::disable(uint8_t pin){
    uint8_t interrupt = digitalPinToInterrupt(pin);
    if(interrupt == NOT_AN_INTERRUPT || interrupt > NUM_INTERRUPTS){
        return false;
    }
    detachInterrupt(interrupt);
    return true;
}

bool Interrupts::isInterrupt(uint8_t pin){
    uint8_t interrupt = digitalPinToInterrupt(pin);
    return !(interrupt == NOT_AN_INTERRUPT || interrupt > NUM_INTERRUPTS);
}

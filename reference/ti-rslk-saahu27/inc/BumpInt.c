// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches on the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"

void (*BumpInt_Task)(uint8_t);   // user function

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
//void BumpInt_Init(void(*task)(uint8_t)){
void BumpInt_Init(void(*task)(uint8_t)){
    // write this as part of Lab 14
    BumpInt_Task = task;

    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;

    P4->DIR &= ~0xED;
    P4->REN |= 0xED;
    P4->OUT |= 0xED;

    P4->IES |= 0xED;    //falling edge event
    P4->IFG &= ~0xED;   //clear flags
    P4->IE |= 0xED;     //arm interrupt
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00200000;  //Priority 1
    NVIC->ISER[1] = 0x00000040;    //interrupt 38
//    EnableInterrupts();
}
// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    // write this as part of Lab 14
    uint8_t state;
    state = P4->IN&0xED;
    state = ~state;
    uint8_t new_state;
    new_state = 0x00;
    if((state&0x01) == 0x01){     // Bump0
        new_state |= 0x01;
    }
    if((state&0x04) == 0x04){     // Bump1
        new_state |= 0x02;
    }
    if((state&0x08) == 0x08){     // Bump2
        new_state |= 0x04;
    }
    if((state&0x20) == 0x20){     // Bump3
        new_state |= 0x08;
    }
    if((state&0x40) == 0x40){     // Bump4
        new_state |= 0x10;
    }
    if((state&0x80) == 0x80){     // Bump5
        new_state |= 0x20;
    }

    return new_state; // replace this line
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    // write this as part of Lab 14
    P4->IFG &= ~0xED;   //clear flags
    (*BumpInt_Task)(Bump_Read());             // execute user task
}


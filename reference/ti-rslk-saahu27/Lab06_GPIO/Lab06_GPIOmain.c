// Lab06_GPIOmain.c
// Runs on MSP432
// Solution to GPIO lab
// Daniel and Jonathan Valvano
// April 15, 2019
// Provide test main program for QTRX reflectance sensor array
// Pololu part number 3672.

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

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\TExaS.h"
//#include "..\inc\LaunchPad.h"

void LaunchPad_Init(void){
  P1->SEL0 &= ~0x13;
  P1->SEL1 &= ~0x13;    // 1) configure P1.4, P1.1, and P1.0 as GPIO
  P1->DIR &= ~0x12;     // 2) make P1.4 and P1.1 in
  P1->DIR |= 0x01;      //    make P1.0 out
  P1->REN |= 0x12;      // 3) enable pull resistors on P1.4 and P1.1
  P1->OUT |= 0x12;      //    P1.4 and P1.1 are pull-up
  P2->SEL0 &= ~0x07;
  P2->SEL1 &= ~0x07;    // 1) configure P2.2-P2.0 as GPIO
  P2->DIR |= 0x07;      // 2) make P2.2-P2.0 out
  P2->DS |= 0x07;       // 3) activate increased drive strength
  P2->OUT &= ~0x07;     //    all LEDs off
}

uint8_t Data; // QTRX
int32_t Position;
// Test main for section 6.4.3
int main(void){
  Clock_Init48MHz();
  LaunchPad_Init();
  Reflectance_Init(); // your initialization
  TExaS_Init(LOGICANALYZER_P7);
  while(1){
    Data = Reflectance_Read(1000);// your measurement
    Position = Reflectance_Position(Data);
    if(Position < 0){
        P2->OUT |= 0x01;
        P2->OUT &= ~0x04;
    }
    else if(Position == 0){
        P2->OUT |= 0x07;
    }
    else{
        P2->OUT |= 0x04;
        P2->OUT &= ~0x01;
    }
    Clock_Delay1ms(10);
  }
}

int32_t Position; // 332 is right, and -332 is left of center
int main4(void){
  //Clock_Init48MHz();
  Reflectance_Init();
  TExaS_Init(LOGICANALYZER_P4);
  while(1){
    P5->OUT |= 0x08;
    P7->DIR |= 0x01;
    P7->OUT |= 0x01;
    Clock_Delay1us(10);
    P7->DIR &= ~0x01;
    for(int i = 1;i <= 10000;i++){
        P4->OUT = P7->IN&0x01;
    }
    P5->OUT &= ~0x08;
    // Data = Reflectance_Read(1000);
    // Position = Reflectance_Position(Data);
    Clock_Delay1ms(10);
  }
}

int main2(void){ // main2(void){
    int32_t i;
  Clock_Init48MHz();
  Reflectance_Init();
  P4->SEL0 &= ~0x01;
  P4->SEL1 &= ~0x01;    //  P4.0 as GPIO
  P4->DIR |= 0x01;      //  make P4.0 out
  TExaS_Init(LOGICANALYZER_P7);
  while(1){
    P5->OUT |= 0x08;      // turn on 4 even IR LEDs
    P9->OUT |= 0x04;      // turn on 4 odd IR LEDs
    P7->DIR = 0xFF;       // make P7.7-P7.0 out
    P7->OUT = 0xFF;       // prime for measurement
    Clock_Delay1us(10);   // wait 10 us
    P7->DIR = 0x00;       // make P7.7-P7.0 in
    for(i=0;i<10000;i++){
      P4->OUT = P7->IN&0x01; // convert P7.0 input to digital
    }
    P5->OUT &= ~0x08;     // turn off 4 even IR LEDs
    P9->OUT &= ~0x04;     // turn off 4 odd IR LEDs
    Clock_Delay1ms(10);
  }
}

int main3(void){ // main3
    int32_t i;
  Clock_Init48MHz();
  Reflectance_Init();
  P4->SEL0 &= ~0x01;
  P4->SEL1 &= ~0x01;    //  P4.0 as GPIO
  P4->DIR |= 0x01;      //  make P4.0 out
  TExaS_Init(LOGICANALYZER_P7);
  while(1){
    P5->OUT |= 0x08;      // turn on 4 even IR LEDs
    P7->DIR = 0xFF;       // make P7.7-P7.0 out
    P7->OUT = 0xFF;       // prime for measurement
    Clock_Delay1us(10);   // wait 10 us
    P7->DIR = 0x00;       // make P7.7-P7.0 in
    for(i=0;i<10000;i++){
      P4->OUT = P7->IN&0x01; // convert P7.0 input to digital
    }
    P5->OUT &= ~0x08;     // turn off 4 even IR LEDs
    Clock_Delay1ms(10);
  }
}

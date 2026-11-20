// Reflectance.c
// Provide functions to take measurements using the kit's built-in
// QTRX reflectance sensor array.  Pololu part number 3672. This works by outputting to the
// sensor, waiting, then reading the digital value of each of the
// eight phototransistors.  The more reflective the target surface is,
// the faster the voltage decays.
// Daniel and Jonathan Valvano
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
#include "msp432.h"
#include "..\inc\Clock.h"
//#include "..\inc\SysTick.h"

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
void Reflectance_Init(void){
    // write this as part of Lab 6
    //The code written below is for 6.4.3
    Clock_Init48MHz();      //Initialize the clock system to run at 48 MHz

    P5->SEL0 &= ~0x08;      //Set pin 5.3 as GPIO
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08;        //Make pin 5.3 as output
    P5->OUT &= ~0x08;       //Set 5.3 low

    P9->SEL0 &= ~0x04;      //Set pin 9.2 as GPIO
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;        //Make pin 9.2 as output
    P9->OUT &= ~0x04;       //Set 9.2 low

    P7->SEL0 = 0x00;        //Set pins 7.7-7.0 as GPIO
    P7->SEL1 = 0x00;
    P7->DIR = 0x00;         //Make pins 7.7-7.0 as input
}

// ------------Reflectance_Read------------
// Read the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t time){
    uint8_t result = 0;
    // write this as part of Lab 6
    P5->OUT |= 0x08;        //Set pin 5.3 high  (Turns on IR LED)
    P9->OUT |= 0x04;        //Set pin 9.2 high  (Turns on IR LED)
    P7->DIR = 0xFF;         //Make pins 7.7-7.0 output
    P7->OUT = 0xFF;         //Set pins 7.7-7.0 high (charges the eight capacitors)
    Clock_Delay1us(10);     //Wait for 10 us
    P7->DIR = 0x00;         //Make pins 7.7-7.0 input
    Clock_Delay1us(time);   //Wait for time us which is given as input to this function
    result = P7->IN;        //Read pins 7.7-7.0
    P5->OUT &= ~0x08;       //Set pin 5.3 low (Turns off IR LED)
    P9->OUT &= ~0x04;       //Set pin 9.2 low (Turns off IR LED)
  return result;
}

// ------------Reflectance_Center------------
// Read the two center sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: 0 (off road), 1 off to left, 2 off to right, 3 on road
// (Left,Right) Sensors
// 1,1          both sensors   on line
// 0,1          just right     off to left
// 1,0          left left      off to right
// 0,0          neither        lost
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Center(uint32_t time){
    // write this as part of Lab 6
  return 0; // replace this line
}


// Perform sensor integration
// Input: data is 8-bit result from line sensor
// Output: position in 0.1mm relative to center of line
int32_t Reflectance_Position(uint8_t data){
    // write this as part of Lab 6
    int W[8] = {334,238,142,48,-48,-142,-238,-334};     //Distance of each sensor from center of robot in 0.1 mm units
    int b[8] = {1,1,1,1,1,1,1,1};                       //Initially assume that all sensors read black
    //The conditional statements below checks each sensor if it detects white and updates that information
    if((data&0x80) == 0x00){        // white = 0, black = 1
        b[0] = 0;
    }
    if((data&0x40) == 0x00){
            b[1] = 0;
    }
    if((data&0x20) == 0x00){
            b[2] = 0;
    }
    if((data&0x10) == 0x00){
            b[3] = 0;
    }
    if((data&0x08) == 0x00){
            b[4] = 0;
    }
    if((data&0x04) == 0x00){
            b[5] = 0;
    }
    if((data&0x02) == 0x00){
            b[6] = 0;
    }
    if((data&0x01) == 0x00){
            b[7] = 0;
    }
    int32_t d;
    int32_t numerator = 0;
    int32_t denominator = 0;
    //The code below checks the distance of the black line from the center of the robot
    for(int i = 0; i < 8; i++){
        numerator = numerator + b[i]*W[i];
        denominator = denominator + b[i];
    }
    if(denominator == 0){
        d = 0;
    }
    else{
        d = numerator/denominator;
    }
 return d;
}

// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){
    // write this as part of Lab 10
    P5->OUT |= 0x08;
    P9->OUT |= 0x04;

    P7->DIR = 0xFF;
    P7->OUT = 0xFF;

//    SysTick_Wait1us(10);

    P7->DIR = 0x00;
}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    // write this as part of Lab 10
    uint8_t result;
    result = P7->IN;

    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;

 return result; // replace this line
}


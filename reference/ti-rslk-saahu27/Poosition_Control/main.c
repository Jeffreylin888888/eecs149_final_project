/*****************************************************************************
*
* Copyright (C) 2013 - 2017 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the
*   distribution.
*
* * Neither the name of Texas Instruments Incorporated nor the names of
*   its contributors may be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*
* MSP432 empty main.c template
*
******************************************************************************/

#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "..\inc\Clock.h"
#include "..\inc\Motor.h"
#include "..\inc\MotorSimple.h"
#include "..\inc\SysTick.h"
#include "..\inc\Reflectance.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\TA3InputCapture.h"
#include "..\inc\TimerA1.h"
#include "..\inc\FlashProgram.h"

//state data structure
struct State{
    uint32_t out;    //8-bit output
    uint32_t delay;
    const struct State *next[4];   //pointer to next state
};

typedef const struct State State_t;

#define Center &fsm[0]
#define Left &fsm[1]
#define Right &fsm[2]

State_t fsm[3]={
  {0x03, 1, { Right, Left,   Right,  Center }},  // Center
  {0x02, 1, { Left,  Left, Right,  Center }},  // Left
  {0x01, 1, { Right, Left,   Right, Center }}   // Right
};

State_t *Spt;  // pointer to the current state
uint8_t Input;
uint32_t Output;

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))
#define P1_0 (*((volatile uint8_t *)(0x42098040)))

int32_t Dstar;
int32_t Dprime;
int32_t Error;
int32_t U;

uint8_t Data;

//void DistanceMeasure(void){
//    Data = Reflectance_Read(1000);
//    Dprime = Reflectance_Position(Data);
//}

int UL[100];
int UR[100];
double a = 21.5; // 21.5; 18
double b = 36.5; // 36.5; 45.625
double w = 14;
double c = 20.41;  // 21.9911;
int count = 0;
double theta;
double L;
double d;
double Lr;
double Rr;
double dtheta = 6.28/100.0;
double dt = 5.0/100.0;

void generate_inputs(void){
    double i = 0;
    while(i < 100){
        theta = i*dtheta;
        d = sqrt((a*(cos(theta+dtheta)-cos(theta)))*(a*(cos(theta+dtheta)-cos(theta))) + (b*(sin(theta+dtheta)-sin(theta)))*(b*(sin(theta+dtheta)-sin(theta))));
        L = sqrt(a*cos(theta)*a*cos(theta) + b*sin(theta)*b*sin(theta)) - 0.5*w;
        Lr = (L*d)/(L + 0.5*w);
        Rr = (L + w)*d/(L + 0.5*w);
        UL[(int) i] = Lr*60/(dt*c); //*(7500/120);
        UR[(int) i] = Rr*60/(dt*c); //*(7500/120);
        i++;
    }
}

int XstarL;
uint32_t XprimeL;
int ErrorL;
int VL;
uint32_t PeriodL;
uint32_t FirstL;
int DoneL;
int flag;

void PeriodMeasureL(uint16_t time){
  P2_0 = P2_0^0x01;            // thread profile, P2.0
  PeriodL = (time - FirstL)&0xFFFF; // 16 bits, 83.3 ns resolution
  FirstL = time;               // setup for next
  DoneL = 1;
  XprimeL = 2000000/PeriodL;
}

int XstarR;
int XprimeR;
int ErrorR;
int VR;
uint32_t PeriodR;
uint32_t FirstR;
int DoneR;
uint32_t time;

int VPL;
int VPR;
int DP;

//double prev_errorL = 0;
//double prev_errorR = 0;
//double Dt = 1;
//double VDL;
//double VDR;

void PeriodMeasureR(uint16_t time){
  P1_0 = P1_0^0x01;            // thread profile, P1.0
  PeriodR = (time - FirstR)&0xFFFF; // 16 bits, 83.3 ns resolution
  FirstR = time;               // setup for next
  DoneR = 1;
  XprimeR = 2000000/PeriodR;
}

int result2;
uint32_t addr;

void Controller(void){

    if(count%2 == 0){
        Data = Reflectance_Read(1000);
        Dprime = Reflectance_Position(Data);
        if(Dprime == 0){
                U = 0;
            }
        //    else if (Dprime > 0){
        //        U = 1000;
        //    }
        //    else if (Dprime < 0){
        //        U = -1000;
        //    }
            else if ((Dprime <= 95) && (Dprime > 0)){
                U = 500;
            }
            else if ((Dprime <= 190) && (Dprime > 95)){
                U = 1000;
            }
            else if ((Dprime <= 286) && (Dprime > 190)){
                U = 1500;
            }
            else if ((Dprime >= -95) && (Dprime < 0)){
                U = -500;
            }
            else if ((Dprime >= -190) && (Dprime < -95)){
                U = -1000;
            }
            else if ((Dprime >= -286) && (Dprime < -190)){
                U = -1500;
            }
    }
    else{
        U = 0;
    }

    Error = Dprime - Dstar;
//    U = U + DP*Error;

//    if(U < -800){
//        U = -800;
//        Dprime = -334;
//    }
//    else if(U > 800){
//        U = 800;
//        Dprime = 334;
//    }

    if(count%50 == 0){
        XstarL = UL[count/50];
        XstarR = UR[count/50];
    }

    ErrorL = XstarL - XprimeL;

    VL = VL + VPL*ErrorL;

    ErrorR = XstarR - XprimeR;
    VR = VR + VPR*ErrorR;
    if(VL < 2){
        VL = 2;
        XprimeL = 0;
    }
    else if(VL > 14998){
        VL = 14998;
        XprimeL = 250;
    }

    if(VR < 2){
        VR = 2;
        XprimeR = 0;
    }
    else if(VR > 14998){
        VR = 14998;
        XprimeR = 250;
    }

    if(count < 5000){
        Motor_Forward(VL - U,VR + U);
        count++;
        addr = 0x00020000 + count*0x01;
        result2 = Flash_Write(addr,XprimeL);
    }
    else{
        Motor_Forward(0,0);
    }
}

void Debug_FlashInit(void){
  // write this as part of Lab 10
//    int i = 0;
//    for(i = 0; i < 32; i++){
//        FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, sector_array[i]);
//    }
//    FlashCtl_initiateMassErase();
    uint32_t addr = 0x00020000;
    int result;
    while(addr <= 0x0003FFFF){
        result = Flash_Erase(addr);
        addr += 0x1000;
    }


}

void Debug_FlashRecord(uint32_t *pt){ // uint16_t
  // write this as part of Lab 10

    int result1;
    uint32_t addr1 = 0x00020000;
    result1 = Flash_Write(addr1,*pt);
}

int main(void)
{
    DisableInterrupts();
    Clock_Init48MHz();
    P1->SEL0 &= ~0x01;
    P1->SEL1 &= ~0x01;   // configure P1.0 as GPIO
    P1->DIR |= 0x01;     // P1.0 output
    P2->SEL0 &= ~0x01;
    P2->SEL1 &= ~0x01;   // configure P2.0 as GPIO
    P2->DIR |= 0x01;
    FirstL = FirstR = 0;
    DoneL = DoneR = 0;
    Motor_Init();
    TimerA3Capture_Init01(&PeriodMeasureR, &PeriodMeasureL);
    Reflectance_Init();
    Debug_FlashInit();
    generate_inputs();
    TimerA1_Init(&Controller,500);  // 1 ms
    VPL = 8; // 8
    VPR = 8;
//    VDL = 0.01;
//    VDR = 0.01;
    Dstar = 0;
    DP = 1;
//    XstarL = 60;
//    XstarR = 60;
    EnableInterrupts();
    while(1){
        WaitForInterrupt();
    }
}

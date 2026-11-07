#include "board.h"
#include "config.h"
#include "feedback.h"
/* A21/A22/A23, AVCC reference. Wiring/voltage must be verified first.
 * Nonblocking sequence: foreground never waits for a broken ADC. */
static uint16_t values[3];
static uint32_t sampled, started;
static bool valid, pending;
void feedback_init(void) {
    if (!FEEDBACK_WIRING_VERIFIED) return;
    ADC14->CTL0 &= ~2u;
    ADC14->CTL0=0x04223390u;
    ADC14->CTL1=0x00020030u;
    ADC14->MCTL[2]=21; ADC14->MCTL[3]=22; ADC14->MCTL[4]=0x80u|23;
    ADC14->IER0=ADC14->IER1=0;
    P8->SEL0|=BIT2|BIT3|BIT4; P8->SEL1|=BIT2|BIT3|BIT4;
    ADC14->CTL0|=2;
}
void feedback_poll(uint32_t now) {
    if (!FEEDBACK_WIRING_VERIFIED) return;
    if (pending && (ADC14->IFGR0 & 0x10)) {
        for(unsigned i=0;i<3;i++) values[i]=(uint16_t)ADC14->MEM[i+2];
        sampled=now; valid=true; pending=false;
    }
    if(pending && (uint32_t)(now-started)>=20) valid=false;
    if(!pending && (uint32_t)(now-started)>=20 && !(ADC14->CTL0&0x10000)) {
        started=now; pending=true; ADC14->CTL0|=1;
    }
}
bool feedback_read(uint32_t now,uint16_t result[3]) {
    if(!FEEDBACK_WIRING_VERIFIED || !valid || (uint32_t)(now-sampled)>100) return false;
    for(unsigned i=0;i<3;i++) result[i]=values[i];
    return true;
}

#include "board.h"
#include "core.h"
#include "config.h"
#include "servo.h"
static const servo_config calibration[3]=SERVO_CALIBRATION;
static const uint16_t poses[5][3]=ARM_POSES;
static volatile uint16_t pulse[3], target[3];
static volatile uint8_t enabled;
void servo_stop(void) {
    /* Remove pulses; mechanical support is still needed for a loaded arm. */
    if (!SERVO_WIRING_VERIFIED) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    enabled=0;
    P2->SEL0&=~BIT4; P2->OUT&=~BIT4;
    P3->SEL0&=~BIT5; P3->OUT&=~BIT5;
    P5->SEL0&=~BIT7; P5->OUT&=~BIT7;
    TIMER_A1->CCTL[1]=TIMER_A1->CCTL[2]=TIMER_A2->CCTL[2]=0;
    __set_PRIMASK(mask);
}
void servo_init(void) {
    if (!SERVO_WIRING_VERIFIED) return;
    P2->OUT&=~BIT4; P2->DIR|=BIT4; P2->SEL1&=~BIT4;
    P3->OUT&=~BIT5; P3->DIR|=BIT5; P3->SEL1&=~BIT5;
    P5->OUT&=~BIT7; P5->DIR|=BIT7; P5->SEL1&=~BIT7;
    PMAP->KEYID=PMAP_KEYID_VAL;
    P2MAP->PMAP_REGISTER4=PMAP_TA1CCR1A;
    P3MAP->PMAP_REGISTER5=PMAP_TA1CCR2A;
    PMAP->KEYID=0;
    /* SMCLK 3 MHz / 3 = 1 MHz; 20000 counts gives 50 Hz. */
    TIMER_A1->CCR[0]=TIMER_A2->CCR[0]=19999;
    TIMER_A1->EX0=TIMER_A2->EX0=2;
    TIMER_A1->CTL=TIMER_A2->CTL=TIMER_A_CTL_SSEL__SMCLK|TIMER_A_CTL_MC__UP|TIMER_A_CTL_CLR;
    servo_stop();
    TIMER_A1->CCTL[0]=TIMER_A2->CCTL[0]=TIMER_A_CCTLN_CCIE;
    NVIC_SetPriority(TA1_0_IRQn,0); NVIC_SetPriority(TA2_0_IRQn,0);
    NVIC_EnableIRQ(TA1_0_IRQn); NVIC_EnableIRQ(TA2_0_IRQn);
}
bool servo_set(unsigned ch,int32_t us) {
    if (!SERVO_WIRING_VERIFIED || ch>=3 || !servo_target_valid(calibration[ch],us)) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    target[ch]=(uint16_t)us;
    if (!(enabled&(1u<<ch))) {
        pulse[ch]=calibration[ch].start;
        enabled|=1u<<ch;
        /* Initial pulse is calibrated assembly position, never assumed neutral. */
    }
    __set_PRIMASK(mask);
    return true;
}
bool servo_pose(unsigned pose) {
    if (!SERVO_WIRING_VERIFIED || pose>=5) return false;
    for(unsigned i=0;i<3;i++) if (!servo_target_valid(calibration[i],poses[pose][i])) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    for(unsigned i=0;i<3;i++) (void)servo_set(i,poses[pose][i]);
    __set_PRIMASK(mask);
    return true;
}
/* Foreground stages targets; compare registers change only at frame boundary. */
void servo_update(void) { }
static void frame(unsigned i) {
    if (!(enabled&(1u<<i))) return;
    Timer_A_Type *timer=i==2 ? TIMER_A2 : TIMER_A1;
    unsigned channel=i==0 ? 1 : 2;
    bool first=timer->CCTL[channel]==0;
    if (!first) pulse[i]=servo_step(pulse[i],target[i],calibration[i].slew);
    timer->CCR[channel]=pulse[i];
    if(first) timer->CCTL[channel]=TIMER_A_CCTLN_OUTMOD_7;
    if(i==0) P2->SEL0|=BIT4;
    else if(i==1) P3->SEL0|=BIT5;
    else P5->SEL0|=BIT7;
}
void TA1_0_IRQHandler(void) {
    TIMER_A1->CCTL[0]&=~TIMER_A_CCTLN_CCIFG;
    frame(0);frame(1);
}
void TA2_0_IRQHandler(void) {
    TIMER_A2->CCTL[0]&=~TIMER_A_CCTLN_CCIFG;
    frame(2);
}
bool servo_settled(void) {
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool settled=enabled==7;
    for(unsigned i=0;i<3;i++) if(pulse[i]!=target[i]) settled=false;
    __set_PRIMASK(mask);
    return settled;
}

#include "board.h"
/* TA0: SMCLK=3 MHz, period=3000 counts => 1 kHz, 20% duty. */
static volatile uint16_t remaining_ms;
void motor_stop(void) {
    P3->OUT &= ~MOTOR_SLEEP;
    P2->SEL0 &= ~MOTOR_PWM;
    P2->SEL1 &= ~MOTOR_PWM;
    P2->OUT &= ~MOTOR_PWM;
    TIMER_A0->CTL = 0;
    TIMER_A0->CCTL[3] = TIMER_A0->CCTL[4] = 0;
    remaining_ms = 0;
}
void motor_init(void) {
    P3->OUT &= ~MOTOR_SLEEP;
    P3->SEL0 &= ~MOTOR_SLEEP; P3->SEL1 &= ~MOTOR_SLEEP;
    P3->DIR |= MOTOR_SLEEP;
    P2->OUT &= ~MOTOR_PWM; P2->DIR |= MOTOR_PWM;
    P5->OUT &= ~MOTOR_DIR;
    P5->SEL0 &= ~MOTOR_DIR; P5->SEL1 &= ~MOTOR_DIR;
    P5->DIR |= MOTOR_DIR;
    motor_stop();
}
uint16_t motor_stop_inputs(void) {
    /* Low byte: active P4 bump bits; bit 8: S2 pressed. */
    return ((uint16_t)(~P4->IN) & BUMPS) | ((P1->IN & BIT4) ? 0 : 0x100);
}
bool motor_run(bool left) {
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    motor_stop();
    if (motor_stop_inputs()) { __set_PRIMASK(mask); return false; }
    const unsigned channel = left ? 4 : 3;
    const uint8_t pin = left ? BIT7 : BIT6;
    TIMER_A0->CCR[0] = 2999;
    TIMER_A0->CCR[channel] = 600;
    TIMER_A0->CCTL[channel] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;
    P2->SEL0 |= pin;
    remaining_ms = 300;
    P3->OUT |= pin;
    __set_PRIMASK(mask);
    return true;
}
void motor_tick(void) {
    /* Independent of foreground state-machine progress; active-run S2/bump abort is latched by main. */
    if (motor_stop_inputs()) motor_stop();
    else if (remaining_ms && --remaining_ms == 0) motor_stop();
}

/* Runtime command path. Lease refreshed by foreground; ISR enforces expiry. */
bool motor_set(int left, int right, uint16_t lease_ms) {
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (motor_stop_inputs() || left<-25 || left>25 || right<-25 || right>25 ||
        lease_ms==0 || lease_ms>250) {
        motor_stop(); __set_PRIMASK(mask); return false;
    }
    const uint8_t direction=(left<0 ? BIT4 : 0)|(right<0 ? BIT5 : 0);
    if ((P5->OUT&MOTOR_DIR)!=direction) motor_stop();
    P3->OUT&=~MOTOR_SLEEP;
    P5->OUT=(P5->OUT&~MOTOR_DIR)|direction;
    if (!(TIMER_A0->CTL&TIMER_A_CTL_MC__UP)) {
        TIMER_A0->CCR[0]=2999;
        TIMER_A0->CTL=TIMER_A_CTL_SSEL__SMCLK|TIMER_A_CTL_MC__UP|TIMER_A_CTL_CLR;
    }
    const int value[2]={left,right};
    for(unsigned i=0;i<2;i++) {
        const unsigned ch=i==0 ? 4 : 3;
        const uint8_t pin=i==0 ? BIT7 : BIT6;
        if (!value[i]) { P2->SEL0&=~pin; P2->OUT&=~pin; TIMER_A0->CCTL[ch]=0; }
        else {
            unsigned magnitude=(unsigned)(value[i]<0 ? -value[i] : value[i]);
            TIMER_A0->CCR[ch]=(uint16_t)(30*magnitude);
            if (!(P2->SEL0&pin)) TIMER_A0->CCTL[ch]=TIMER_A_CCTLN_OUTMOD_7;
            P2->SEL0|=pin; P3->OUT|=pin;
        }
    }
    remaining_ms=lease_ms;
    __set_PRIMASK(mask); return true;
}

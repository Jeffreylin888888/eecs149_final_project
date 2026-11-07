#include "board.h"
#include <assert.h>
#include <stdio.h>
port p1,p2,p3,p4,p5;
timer ta0;
static void stopped(void) {
    assert(!(p3.OUT & MOTOR_SLEEP)); assert(!(p2.SEL0 & MOTOR_PWM));
    assert(!(p2.OUT & MOTOR_PWM)); assert(ta0.CTL == 0);
}
int main(void) {
    p1.IN=BIT4; p4.IN=BUMPS;
    p2.OUT=255; p3.OUT=255; p2.SEL0=255; p2.SEL1=255;
    motor_init(); stopped();
    for (unsigned side=0; side<2; ++side) {
        const bool left = side == 0;
        motor_run(left);
        assert((p3.OUT & MOTOR_SLEEP) == (left ? BIT7 : BIT6));
        assert((p2.SEL0 & MOTOR_PWM) == (left ? BIT7 : BIT6));
        assert(ta0.CCR[0]==2999 && ta0.CCR[left ? 4 : 3]==600);
        for(unsigned i=0; i<299; ++i) motor_tick();
        assert(p3.OUT & MOTOR_SLEEP);
        motor_tick(); stopped();
    }
    motor_run(true); p1.IN=0; motor_tick(); stopped(); p1.IN=BIT4;
    for (unsigned bit=0; bit<8; ++bit) if (BUMPS & (1u<<bit)) {
        motor_run(false); p4.IN=BUMPS & ~(1u<<bit); motor_tick(); stopped(); p4.IN=BUMPS;
    }
    p1.IN=0;
    assert(!motor_run(true)); stopped(); assert(motor_stop_inputs()==0x100);
    p1.IN=BIT4;
    for (unsigned bit=0; bit<8; ++bit) if (BUMPS & (1u<<bit)) {
        p4.IN=BUMPS & ~(1u<<bit);
        assert(!motor_run(false)); stopped(); assert(motor_stop_inputs()==(1u<<bit));
    }
    p4.IN=BUMPS;
    motor_stop(); motor_stop(); stopped();
    assert(motor_set(-25,20,60));
    assert((p5.OUT&MOTOR_DIR)==BIT4);
    assert(ta0.CCR[4]==750 && ta0.CCR[3]==600);
    for(unsigned i=0;i<59;i++) motor_tick();
    assert((p3.OUT&MOTOR_SLEEP)==MOTOR_SLEEP);
    motor_tick(); stopped();
    assert(motor_set(0,-10,1));
    assert((p3.OUT&MOTOR_SLEEP)==BIT6);
    assert((p5.OUT&MOTOR_DIR)==BIT5);
    motor_tick(); stopped();
    assert(!motor_set(26,0,60)); stopped();
    assert(!motor_set(0,-26,60)); stopped();
    assert(!motor_set(1,1,0)); stopped();
    assert(!motor_set(1,1,251)); stopped();
    p1.IN=0; assert(!motor_set(20,20,60)); stopped();
    puts("PASS: safe GPIO setup, left/right routing, PWM period/duty, 300-tick cutoff, S2 and all six bump stops, idempotent disable (mock registers)");
}

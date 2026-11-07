#include "control/servo.h"
#include <assert.h>
#include <stdio.h>
port p1,p2,p3,p4,p5;
timer ta0;
Timer_A_Type ta1,ta2;
pmap map;
mapping p2map,p3map;
void TA1_0_IRQHandler(void);
void TA2_0_IRQHandler(void);
static void frame(void) { TA1_0_IRQHandler(); TA2_0_IRQHandler(); }
int main(void) {
    servo_init();
    assert(ta1.CCR[0]==19999 && ta2.CCR[0]==19999 && ta1.EX0==2);
    assert(!servo_set(3,1500) && !servo_set(0,999));
    assert(!servo_pose(5));
    assert(servo_pose(1));
    assert(!(p2.SEL0&BIT4) && !(p3.SEL0&BIT5) && !(p5.SEL0&BIT7));
    frame();
    assert(ta1.CCR[1]==1500 && ta1.CCR[2]==1500 && ta2.CCR[2]==1500);
    assert((p2.SEL0&BIT4) && (p3.SEL0&BIT5) && (p5.SEL0&BIT7));
    assert(!servo_settled());
    for(unsigned i=0;i<5;i++) frame();
    assert(servo_settled()); assert(ta1.CCR[1]==1600);
    assert(servo_set(0,1000)); frame(); assert(ta1.CCR[1]==1580);
    servo_stop(); frame();
    assert(!servo_settled());
    assert(!(p2.SEL0&BIT4) && !(p3.SEL0&BIT5) && !(p5.SEL0&BIT7));
    assert(!ta1.CCTL[1] && !ta1.CCTL[2] && !ta2.CCTL[2]);
    assert(servo_set(0,1800)); frame(); assert(ta1.CCR[1]==1500);
    puts("PASS: servo mapping, 50 Hz configuration, staged frame update, slew, bounds, stop and restart (mock registers)");
}

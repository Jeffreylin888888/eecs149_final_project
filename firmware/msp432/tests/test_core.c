#include "control/core.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
int main(void) {
    command c;
    assert(parse_command(" PWM -25 25 250 ",&c) && c.count==3 && c.arg[0]==-25);
    assert(parse_command("MOVE -2147483648 2147483647 100",&c));
    assert(c.arg[0]==INT32_MIN && c.arg[1]==INT32_MAX);
    const char *bad[]={""," ","pwm 1","PWM 1 2 3 4","ABCDEFGHI","X 2147483648","X -2147483649","X -","X 1a","X\t1","X +1","X 999999999999999999"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++) assert(!parse_command(bad[i],&c));
    servo_config s={1000,2000,1400,20};
    assert(servo_config_valid(s)); assert(!servo_config_valid((servo_config){0}));
    assert(!servo_config_valid((servo_config){1000,2000,900,20}));
    assert(!servo_config_valid((servo_config){1000,2000,1500,0}));
    assert(servo_target_valid(s,1000) && servo_target_valid(s,2000));
    assert(!servo_target_valid(s,999) && !servo_target_valid(s,2001));
    assert(servo_step(1000,1100,20)==1020 && servo_step(1090,1100,20)==1100);
    assert(servo_step(1100,1000,20)==1080 && servo_step(1010,1000,20)==1000);
    for(int a=0;a<2;a++) for(int b=0;b<2;b++) assert(encoder_delta(a,b)==(a==b?1:-1));
    move_control m;
    assert(!move_begin(&m,100001,0,1000,0));
    assert(!move_begin(&m,0,0,99,0));
    assert(move_begin(&m,100,-100,2000,0)); move_update(&m,0,0,20);
    assert(m.duty[0]==25 && m.duty[1]==-25);
    move_update(&m,100,-50,100); assert(!m.duty[0] && m.duty[1]==-25);
    move_update(&m,100,-99,200); assert(m.state==MOVE_DONE && !m.duty[1]);
    assert(move_begin(&m,100,0,2000,0)); move_update(&m,0,0,500);
    assert(m.state==MOVE_STALL && !m.duty[0] && !m.duty[1]);
    assert(move_begin(&m,100,100,100,UINT32_MAX-50)); move_update(&m,20,20,49);
    assert(m.state==MOVE_TIMEOUT && !m.duty[0]);
    assert(move_begin(&m,100,100,2000,UINT32_MAX-50)); move_update(&m,50,50,49);
    assert(m.state==MOVE_RUNNING);
    move_update(&m,INT32_MIN,INT32_MAX,50); assert(m.duty[0]==25 && m.duty[1]==-25);
    puts("PASS: parser boundaries, servo bounds/slew, encoder polarity, motion convergence/stall/timeout/wrap");
}

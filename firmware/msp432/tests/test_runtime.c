#include "../src/control/runtime.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>
uint32_t mock_irq_mask;
port p1,p2,p3,p4,p5;
timer ta0;
static unsigned motor_stops, servo_stops, servo_calls;
static uint16_t inputs;
static char reply_text[200];
void motor_init(void) {}
void motor_stop(void) { motor_stops++; }
void motor_tick(void) {}
uint16_t motor_stop_inputs(void) { return inputs; }
void servo_stop(void) { servo_stops++; }
bool servo_set(unsigned ch,int32_t us) { servo_calls++; return ch<3 && us>=1000 && us<=2000; }
bool servo_pose(unsigned pose) { servo_calls++;return pose<5; }
bool servo_settled(void) { return false; }
void encoder_read(int32_t *l,int32_t *r,uint32_t *e) { *l=10;*r=20;*e=0; }
bool feedback_read(uint32_t now,uint16_t v[3]) { (void)now;(void)v;return false; }
void serial_put(const char *s) { strncat(reply_text,s,sizeof(reply_text)-strlen(reply_text)-1); }
void serial_number(int32_t n) { (void)n; }
static void send(const char *s) { reply_text[0]=0;handle(s); }
static void arm_for_test(void) { armed=true;fault=0;inputs=0;last_contact=ticks; }
int main(void) {
    send("PWM 20 20 200");assert(!armed && !raw_duration && !strcmp(reply_text,"ERR\n"));
    send("PING");assert(link_seen && !strcmp(reply_text,"OK\n"));
    arm_for_test();send("POSE 3");assert(servo_calls==1 && armed);
    unsigned stopped=servo_stops;
    send("PWM 20 -20 200");assert(raw_duration==200 && raw_l==20 && raw_r==-20);
    assert(servo_stops==stopped); /* preserve the carried object's holding pose */
    send("MOVE 100 -100 1000");assert(movement.state==MOVE_RUNNING && !raw_duration);
    assert(servo_stops==stopped && origin_l==10 && origin_r==20);
    send("SERVO 0 1600");assert(!armed && movement.state==MOVE_IDLE); /* moving: reject */
    arm_for_test();send("PWM 26 0 100");assert(!armed && !raw_duration);
    arm_for_test();send("STOP");assert(!armed && !raw_duration && servo_stops>stopped);
    arm_for_test();send("garbage");assert(!armed);
    arm_for_test();last_contact=UINT32_MAX-100;ticks=398;
    SysTick_Handler();assert(!armed && (fault&0x400)); /* wrap, exactly 500 ms */
    inputs=0;send("CLEAR");assert(!fault && !armed && arm_button.state==WAIT_RELEASE);
    arm_for_test();inputs=0x100;SysTick_Handler();assert(!armed && (fault&0x100));
    send("CLEAR");assert(fault); /* held stop cannot clear */
    inputs=0;send("CLEAR");assert(!fault);
    for(unsigned bit=0;bit<8;bit++) if(BUMPS&(1u<<bit)) {
        arm_for_test();inputs=1u<<bit;SysTick_Handler();assert(!armed && (fault&inputs));
    }
    inputs=0;send("FEEDBACK");assert(!strcmp(reply_text,"ERR\n"));
    assert(motor_stops && servo_stops);
    puts("PASS: runtime command gates, holding pose preservation, stop, heartbeat wrap, S2/all bump faults, clear interlock");
}

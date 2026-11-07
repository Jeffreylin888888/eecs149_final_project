#include "board.h"
#include "diagnostics.h"
#include "config.h"
#include "core.h"
#include "encoder.h"
#include "servo.h"
#include "serial.h"
#include "feedback.h"
static volatile uint32_t ticks, last_contact;
static volatile bool armed, link_seen;
static volatile uint16_t fault;
static move_control movement;
static int32_t origin_l,origin_r;
static int raw_l,raw_r;
static uint32_t raw_start,raw_duration;
static diagnostic arm_button;
static bool same(const char *a,const char *b) { while(*a && *a==*b) {++a;++b;} return *a==*b; }
static void stop_drive(void) { motor_stop(); raw_duration=0; movement.state=MOVE_IDLE; }
static void stop_all(void) { stop_drive(); servo_stop(); }
void SysTick_Handler(void) {
    ++ticks; motor_tick();
    uint16_t reason=motor_stop_inputs();
    if (armed && (reason || (uint32_t)(ticks-last_contact)>=LINK_TIMEOUT_MS)) {
        fault|=reason ? reason : 0x400; armed=false;
        motor_stop(); servo_stop();
    }
}
static void hard_stop(void) { __disable_irq(); motor_init(); servo_stop(); for(;;) {} }
void HardFault_Handler(void) { hard_stop(); }
void NMI_Handler(void) { hard_stop(); }
static void reply(bool ok) { serial_put(ok ? "OK\n" : "ERR\n"); }
static void status(void) {
    int32_t l,r; uint32_t e; encoder_read(&l,&r,&e);
    serial_put("STATUS "); serial_number(armed); serial_put(" "); serial_number(fault);
    serial_put(" "); serial_number(l); serial_put(" "); serial_number(r);
    serial_put(" "); serial_number(movement.state); serial_put(" "); serial_number((int32_t)e);
    serial_put(" "); serial_number(servo_settled());
    serial_put("\n");
}
static void handle(const char *line) {
    command c;
    if (!parse_command(line,&c)) { armed=false; stop_all(); reply(false); return; }
    if (same(c.op,"STOP") && !c.count) { armed=false; stop_all(); reply(true); return; }
    if (same(c.op,"PING") && !c.count) { last_contact=ticks; link_seen=true; reply(true); return; }
    if (same(c.op,"STATUS") && !c.count) { status(); return; }
    if (same(c.op,"FEEDBACK") && !c.count) {
        uint16_t v[3];
        if (!feedback_read(ticks,v)) { reply(false); return; }
        serial_put("FEEDBACK");
        for(unsigned i=0;i<3;i++) { serial_put(" "); serial_number(v[i]); }
        serial_put("\n"); return;
    }
    if (same(c.op,"CLEAR") && !c.count && !armed && !motor_stop_inputs()) {
        stop_all(); fault=0; arm_button=(diagnostic){WAIT_RELEASE,ticks}; reply(true); return;
    }
    if (!armed || fault || motor_stop_inputs()) { reply(false); return; }
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!armed || fault || motor_stop_inputs()) { __set_PRIMASK(mask); reply(false); return; }
    bool ok=false;
    if (same(c.op,"PWM") && c.count==3 && c.arg[0]>=-MAX_DUTY && c.arg[0]<=MAX_DUTY &&
        c.arg[1]>=-MAX_DUTY && c.arg[1]<=MAX_DUTY && c.arg[2]>=1 && c.arg[2]<=250) {
        stop_drive(); raw_l=c.arg[0]; raw_r=c.arg[1]; raw_start=ticks; raw_duration=(uint32_t)c.arg[2]; ok=true;
    } else if (same(c.op,"MOVE") && c.count==3 && DRIVE_CALIBRATED &&
               (ENCODER_LEFT_SIGN==1 || ENCODER_LEFT_SIGN==-1) &&
               (ENCODER_RIGHT_SIGN==1 || ENCODER_RIGHT_SIGN==-1)) {
        stop_drive(); uint32_t e; encoder_read(&origin_l,&origin_r,&e);
        ok=!e && move_begin(&movement,c.arg[0],c.arg[1],(uint32_t)c.arg[2],ticks);
    } else if (same(c.op,"SERVO") && c.count==2 && movement.state!=MOVE_RUNNING && !raw_duration) {
        ok=c.arg[0]>=0 && servo_set((unsigned)c.arg[0],c.arg[1]);
    } else if (same(c.op,"POSE") && c.count==1 && movement.state!=MOVE_RUNNING && !raw_duration) {
        ok=c.arg[0]>=0 && servo_pose((unsigned)c.arg[0]);
    }
    if (!ok) { stop_all(); armed=false; }
    else last_contact=ticks;
    __set_PRIMASK(mask);
    reply(ok);
}
#ifndef CONTROL_HOST_TEST
int main(void) {
    motor_init();
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);
    CS_initClockSignal(CS_MCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    CS_setReferenceOscillatorFrequency(CS_REFO_32KHZ);
    CS_initClockSignal(CS_ACLK,CS_REFOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,BIT1|BIT4);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,BUMPS);
    GPIO_setAsOutputPin(GPIO_PORT_P2,BIT0|BIT1|BIT2);
    encoder_init(); servo_init(); feedback_init(); serial_init();
    WDT_A_initWatchdogTimer(WDT_A_CLOCKSOURCE_ACLK,WDT_A_CLOCKITERATIONS_8192); WDT_A_startTimer();
    SysTick_Config(3000); __enable_irq();
    serial_put("READY MSP432 CONTROL 1\n");
    char line[80]; unsigned len=0; bool discard=false;
    uint32_t previous=0,control_at=0;
    for(;;) {
        uint32_t now=ticks;
        if(now!=previous) { previous=now; WDT_A_clearTimer(); }
        if(serial_error()) { armed=false; stop_all(); len=0; discard=true; fault|=0x800; }
        /* Bounded RX work preserves foreground control even under host flooding. */
        for(unsigned budget=0;budget<16;budget++) {
            int ch=serial_get(); if(ch<0) break;
            if(ch=='\r') continue;
            if(ch=='\n') {
                if(!discard) { line[len]=0; handle(line); }
                else reply(false);
                len=0; discard=false;
            } else if(discard) continue;
            else if(ch<32 || ch>126 || len==sizeof(line)-1) {
                armed=false; stop_all(); discard=true; len=0;
            } else line[len++]=(char)ch;
        }
        now=ticks;
        feedback_poll(now);
        if(!armed) {
            stop_all();
            if(link_seen && !fault && (uint32_t)(now-last_contact)<LINK_TIMEOUT_MS) {
                diagnostic_step(&arm_button,now,!(P1->IN&BIT1),motor_stop_inputs()!=0);
                if(arm_button.state==LEFT_RUN) { armed=true; arm_button=(diagnostic){WAIT_RELEASE,now}; }
            } else arm_button=(diagnostic){WAIT_RELEASE,now};
        }
        if((uint32_t)(now-control_at)>=20) {
            control_at=now;
            if(armed && !fault) {
                int l=0,r=0; uint16_t lease=60;
                if(raw_duration) {
                    if((uint32_t)(now-raw_start)>=raw_duration) raw_duration=0;
                    else {
                        l=raw_l;r=raw_r;
                        uint32_t remaining=raw_duration-(uint32_t)(now-raw_start);
                        if(remaining<lease) lease=(uint16_t)remaining;
                    }
                } else if(movement.state==MOVE_RUNNING) {
                    int32_t a,b; uint32_t errors; encoder_read(&a,&b,&errors);
                    int32_t dl=(int32_t)((uint32_t)a-(uint32_t)origin_l);
                    int32_t dr=(int32_t)((uint32_t)b-(uint32_t)origin_r);
                    /* Distances are bounded well below signed wrap limits. */
                    if (dl==INT32_MIN || dr==INT32_MIN || errors) { fault|=0x1000; armed=false; }
                    else {
                        move_update(&movement,dl*ENCODER_LEFT_SIGN,dr*ENCODER_RIGHT_SIGN,now);
                        l=movement.duty[0];r=movement.duty[1];
                        if(movement.state>=MOVE_TIMEOUT) { fault|=0x2000;armed=false; }
                    }
                }
                __disable_irq();
                if(armed && !fault && !motor_stop_inputs()) {
                    if(l || r) (void)motor_set(l,r,lease); else motor_stop();
                    servo_update();
                } else { motor_stop();servo_stop(); }
                __enable_irq();
            }
            P2->OUT=(P2->OUT&~7u)|(fault ? BIT0 : armed ? BIT1 : BIT2);
        }
    }
}
extern uint32_t __bss_start__,__bss_end__;
void _mainCRTStartup(void) { for(uint32_t *p=&__bss_start__;p<&__bss_end__;++p) *p=0; (void)main();hard_stop(); }

#endif /* CONTROL_HOST_TEST */

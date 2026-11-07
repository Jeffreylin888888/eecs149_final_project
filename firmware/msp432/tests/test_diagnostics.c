#include "diagnostics.h"
#include <assert.h>
#include <stdio.h>
static void sequence(uint32_t base) {
    diagnostic d = { WAIT_RELEASE, base };
    diagnostic_step(&d, base+49, false, false); assert(d.state == WAIT_RELEASE);
    diagnostic_step(&d, base+50, false, false); assert(d.state == WAIT_PRESS);
    diagnostic_step(&d, base+60, true, false); assert(d.state == WAIT_PRESS);
    diagnostic_step(&d, base+65, false, false); assert(d.state == WAIT_PRESS);
    diagnostic_step(&d, base+114, true, false); assert(d.state == WAIT_PRESS);
    diagnostic_step(&d, base+115, true, false); assert(d.state == WAIT_START_RELEASE);
    diagnostic_step(&d, base+150, true, false); assert(d.state == WAIT_START_RELEASE);
    diagnostic_step(&d, base+199, false, false); assert(d.state == WAIT_START_RELEASE);
    diagnostic_step(&d, base+200, false, false); assert(d.state == LEFT_RUN);
    diagnostic_step(&d, base+499, false, false); assert(d.state == LEFT_RUN);
    diagnostic_step(&d, base+500, false, false); assert(d.state == GAP);
    diagnostic_step(&d, base+1499, false, false); assert(d.state == GAP);
    diagnostic_step(&d, base+1500, false, false); assert(d.state == RIGHT_RUN);
    diagnostic_step(&d, base+1799, false, false); assert(d.state == RIGHT_RUN);
    diagnostic_step(&d, base+1800, false, false); assert(d.state == DONE);
    diagnostic_step(&d, base+2000, true, false); assert(d.state == DONE);
    diagnostic_step(&d, base+3000, false, false); assert(d.state == DONE);
}
int main(void) {
    sequence(0); sequence(UINT32_MAX-100);
    for (unsigned state = LEFT_RUN; state <= ABORTED; ++state) {
        diagnostic d = { (diag_state)state, 0 };
        diagnostic_step(&d, 100, true, true); assert(d.state == (state == DONE ? DONE : ABORTED));
        diagnostic_step(&d, 10000, false, false); assert(d.state == (state == DONE ? DONE : ABORTED));
    }
    /* Startup/idle stop pulses must inhibit, then allow a NEW deliberate start. */
    for (unsigned state=WAIT_RELEASE; state<=WAIT_START_RELEASE; ++state) {
        diagnostic d = { (diag_state)state, 0 };
        diagnostic_step(&d, 100, true, true); assert(d.state == WAIT_RELEASE);
        diagnostic_step(&d, 1000, true, false); assert(d.state == WAIT_RELEASE);
        diagnostic_step(&d, 1049, false, false); assert(d.state == WAIT_RELEASE);
        diagnostic_step(&d, 1050, false, false); assert(d.state == WAIT_PRESS);
        diagnostic_step(&d, 2000, false, false); assert(d.state == WAIT_PRESS);
        diagnostic_step(&d, 2050, true, false); assert(d.state == WAIT_START_RELEASE);
        diagnostic_step(&d, 2100, false, false); assert(d.state == LEFT_RUN);
    }
    diagnostic held = { WAIT_RELEASE, 0 };
    for (unsigned t=0; t<5000; ++t) diagnostic_step(&held,t,true,false);
    assert(held.state == WAIT_RELEASE);
    diagnostic invalid = { (diag_state)99, 0 };
    diagnostic_step(&invalid, 0, false, false); assert(invalid.state == ABORTED);
    diagnostic delayed = { LEFT_RUN, 0 };
    diagnostic_step(&delayed, 100000, false, false); assert(delayed.state == GAP);
    puts("PASS: timing boundaries, debounce, startup-held button, one-shot lockout, pre-start interlock recovery, active-run abort, clock wrap, invalid state, delayed foreground");
}

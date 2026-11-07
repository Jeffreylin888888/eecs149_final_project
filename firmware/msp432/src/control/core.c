#include "core.h"
#include <limits.h>
#include "config.h"
bool parse_command(const char *s, command *out) {
    command c = {{0},{0},0}; unsigned i=0;
    while (*s==' ') ++s;
    while (*s && *s!=' ') {
        if (*s<'A' || *s>'Z' || i>=8) return false;
        c.op[i++]=*s++;
    }
    if (!i) return false;
    while (*s) {
        while (*s==' ') ++s;
        if (!*s) break;
        if (c.count==3) return false;
        bool neg=*s=='-'; if (neg) ++s;
        if (*s<'0' || *s>'9') return false;
        uint32_t n=0, limit=neg ? 2147483648u : 2147483647u;
        while (*s>='0' && *s<='9') {
            unsigned d=(unsigned)(*s++-'0');
            if (n>(limit-d)/10u) return false;
            n=n*10+d;
        }
        if (*s && *s!=' ') return false;
        c.arg[c.count++]=neg ? (n==2147483648u ? INT32_MIN : -(int32_t)n) : (int32_t)n;
    }
    *out=c; return true;
}
bool servo_config_valid(servo_config c) {
    /* Broad electrical sanity envelope, NOT a mechanical calibration. */
    return c.min>=400 && c.max<=2600 && c.min<c.max && c.start>=c.min &&
           c.start<=c.max && c.slew>0 && c.slew<=50;
}
bool servo_target_valid(servo_config c, int32_t target) {
    return servo_config_valid(c) && target>=c.min && target<=c.max;
}
uint16_t servo_step(uint16_t current, uint16_t target, uint16_t slew) {
    if (current<target) return target-current>slew ? current+slew : target;
    return current-target>slew ? current-slew : target;
}
int encoder_delta(bool a, bool b) { return a==b ? 1 : -1; }
bool move_begin(move_control *m, int32_t l, int32_t r, uint32_t timeout, uint32_t now) {
    *m=(move_control){0};
    if (l<-100000 || l>100000 || r<-100000 || r>100000 || timeout<100 || timeout>30000) {
        m->state=MOVE_BAD; return false;
    }
    m->goal[0]=l; m->goal[1]=r; m->since=now;
    m->progress[0]=m->progress[1]=now; m->timeout=timeout; m->state=MOVE_RUNNING;
    return true;
}
void move_update(move_control *m, int32_t l, int32_t r, uint32_t now) {
    m->duty[0]=m->duty[1]=0;
    if (m->state!=MOVE_RUNNING) return;
    if ((uint32_t)(now-m->since)>=m->timeout) { m->state=MOVE_TIMEOUT; return; }
    int32_t counts[2]={l,r}; bool done=true;
    for (unsigned i=0;i<2;i++) {
        int64_t error=(int64_t)m->goal[i]-counts[i];
        if (counts[i]!=m->last[i]) { m->progress[i]=now; m->last[i]=counts[i]; }
        if (error>=-2 && error<=2) continue;
        done=false;
        if ((uint32_t)(now-m->progress[i])>=500) {
            m->state=MOVE_STALL; m->duty[0]=m->duty[1]=0; return;
        }
        int64_t mag=error<0 ? -error : error;
        int duty=(int)(mag>MAX_DUTY ? MAX_DUTY : mag);
        if (duty<10) duty=10;
        m->duty[i]=error<0 ? -duty : duty;
    }
    if (done) m->state=MOVE_DONE;
}

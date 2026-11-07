#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { WAIT_RELEASE, WAIT_PRESS, WAIT_START_RELEASE, LEFT_RUN, GAP, RIGHT_RUN, DONE, ABORTED } diag_state;
typedef struct { diag_state state; uint32_t since; } diagnostic;
void diagnostic_step(diagnostic *d, uint32_t now, bool start, bool stop);
#endif

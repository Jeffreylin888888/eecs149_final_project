#include "diagnostics.h"
/* One shot per reset. Start requires release, debounced press, then release. */
void diagnostic_step(diagnostic *d, uint32_t now, bool start, bool stop) {
    if (stop) {
        /* Before motion, inhibit and discard any partly completed start gesture.
         * Once a run begins, a stop remains latched until reset. */
        if (d->state == WAIT_RELEASE || d->state == WAIT_PRESS || d->state == WAIT_START_RELEASE) {
            d->state = WAIT_RELEASE;
            d->since = now;
        } else if (d->state != DONE) {
            d->state = ABORTED;
        }
        return;
    }
    switch (d->state) {
    case WAIT_RELEASE:
        if (start) d->since = now;
        else if ((uint32_t)(now - d->since) >= 50) { d->state = WAIT_PRESS; d->since = now; }
        break;
    case WAIT_PRESS:
        if (!start) d->since = now;
        else if ((uint32_t)(now - d->since) >= 50) { d->state = WAIT_START_RELEASE; d->since = now; }
        break;
    case WAIT_START_RELEASE:
        if (start) d->since = now;
        else if ((uint32_t)(now - d->since) >= 50) { d->state = LEFT_RUN; d->since = now; }
        break;
    case LEFT_RUN:
        if ((uint32_t)(now - d->since) >= 300) { d->state = GAP; d->since = now; }
        break;
    case GAP:
        if ((uint32_t)(now - d->since) >= 1000) { d->state = RIGHT_RUN; d->since = now; }
        break;
    case RIGHT_RUN:
        if ((uint32_t)(now - d->since) >= 300) d->state = DONE;
        break;
    case DONE: case ABORTED: break;
    default: d->state = ABORTED; break;
    }
}

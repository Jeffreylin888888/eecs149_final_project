#ifndef ROBOT_FEEDBACK_H
#define ROBOT_FEEDBACK_H
#include <stdbool.h>
#include <stdint.h>
void feedback_init(void);
void feedback_poll(uint32_t now);
bool feedback_read(uint32_t now, uint16_t result[3]);
#endif

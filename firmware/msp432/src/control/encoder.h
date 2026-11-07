#ifndef ROBOT_ENCODER_H
#define ROBOT_ENCODER_H
#include <stdint.h>
void encoder_init(void);
void encoder_read(int32_t *left, int32_t *right, uint32_t *errors);
#endif

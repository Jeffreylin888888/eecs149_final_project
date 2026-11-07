#ifndef ROBOT_SERVO_H
#define ROBOT_SERVO_H
#include <stdbool.h>
#include <stdint.h>
void servo_init(void);
bool servo_set(unsigned channel,int32_t us);
bool servo_pose(unsigned pose);
void servo_update(void);
void servo_stop(void);
#endif
bool servo_settled(void);

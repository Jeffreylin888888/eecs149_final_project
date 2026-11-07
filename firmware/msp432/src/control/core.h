#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { char op[9]; int32_t arg[3]; unsigned count; } command;
bool parse_command(const char *line, command *out);
typedef struct { uint16_t min, max, start, slew; } servo_config;
bool servo_config_valid(servo_config c);
bool servo_target_valid(servo_config c, int32_t target);
uint16_t servo_step(uint16_t current, uint16_t target, uint16_t slew);
int encoder_delta(bool a, bool b);
typedef enum { MOVE_IDLE, MOVE_RUNNING, MOVE_DONE, MOVE_TIMEOUT, MOVE_STALL, MOVE_BAD } move_state;
typedef struct {
    move_state state;
    int32_t goal[2], last[2];
    uint32_t since, progress[2], timeout;
    int duty[2];
} move_control;
bool move_begin(move_control *m, int32_t left, int32_t right, uint32_t timeout, uint32_t now);
void move_update(move_control *m, int32_t left, int32_t right, uint32_t now);
#endif

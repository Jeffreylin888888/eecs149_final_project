#ifndef ROBOT_SERIAL_H
#define ROBOT_SERIAL_H
#include <stdbool.h>
#include <stdint.h>
void serial_init(void);
int serial_get(void);
bool serial_error(void);
void serial_put(const char *s);
void serial_number(int32_t n);
#endif

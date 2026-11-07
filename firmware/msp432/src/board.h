#ifndef BOARD_H
#define BOARD_H
#ifdef MOTOR_HOST_TEST
#include "mock_registers.h"
#else
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#endif
#define MOTOR_PWM (BIT6 | BIT7)
#define MOTOR_SLEEP (BIT6 | BIT7)
#define MOTOR_DIR (BIT4 | BIT5)
#define BUMPS (BIT0 | BIT2 | BIT3 | BIT5 | BIT6 | BIT7)
void motor_init(void);
void motor_stop(void);
bool motor_run(bool left);
uint16_t motor_stop_inputs(void);
bool motor_set(int left, int right, uint16_t lease_ms);
void motor_tick(void);
#endif
